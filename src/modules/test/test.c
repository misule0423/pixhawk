#include <px4_config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_0.h>
#include <uORB/topics/actuator_controls_1.h>
#include <uORB/topics/actuator_controls_2.h>
#include <uORB/topics/actuator_controls_3.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
 
/* process-specific header files */
#include "params.h"
 
/* Prototypes */
 
/**
 * Daemon management function.
 *
 * This function allows to start / stop the background task (daemon).
 * The purpose of it is to be able to start the controller on the
 * command line, query its status and stop it, without giving up
 * the command line to one particular process or the need for bg/fg
 * ^Z support by the shell.
 */
__EXPORT int test_main(int argc, char *argv[]);
 
/**
 * Mainloop of daemon.
 */
int test_main(int argc, char *argv[]);
 
/**
 * Print the correct usage.
 */
static void usage(const char *reason);
 
/**
 * Control roll and pitch angle.
 *
 * This very simple roll and pitch controller takes the current roll angle
 * of the system and compares it to a reference. Pitch is controlled to zero and yaw remains
 * uncontrolled (tutorial code, not intended for flight).
 *
 * @param att_sp The current attitude setpoint - the values the system would like to reach.
 * @param att The current attitude. The controller should make the attitude match the setpoint
 * @param rates_sp The angular rate setpoint. This is the output of the controller.
 */
void control_attitude(const struct vehicle_attitude_setpoint_s *att_sp, const struct vehicle_attitude_s *att,
		      struct vehicle_rates_setpoint_s *rates_sp,
		      struct actuator_controls_s *actuators);
 
/**
 * Control heading.
 *
 * This very simple heading to roll angle controller outputs the desired roll angle based on
 * the current position of the system, the desired position (the setpoint) and the current
 * heading.
 *
 * @param pos The current position of the system
 * @param sp The current position setpoint
 * @param att The current attitude
 * @param att_sp The attitude setpoint. This is the output of the controller
 */
void control_heading(const struct vehicle_global_position_s *pos, const struct position_setpoint_s *sp,
		     const struct vehicle_attitude_s *att, struct vehicle_attitude_setpoint_s *att_sp);
 
/* Variables */
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static struct params p;
static struct param_handles ph;
 
void control_attitude(const struct vehicle_attitude_setpoint_s *att_sp, const struct vehicle_attitude_s *att,
		      struct vehicle_rates_setpoint_s *rates_sp,
		      struct actuator_controls_s *actuators)
{
 
	/*
	 * The PX4 architecture provides a mixer outside of the controller.
	 * The mixer is fed with a default vector of actuator controls, representing
	 * moments applied to the vehicle frame. This vector
	 * is structured as:
	 *
	 * Control Group 0 (attitude):
	 *
	 *    0  -  roll   (-1..+1)
	 *    1  -  pitch  (-1..+1)
	 *    2  -  yaw    (-1..+1)
	 *    3  -  thrust ( 0..+1)
	 *    4  -  flaps  (-1..+1)
	 *    ...
	 *
	 * Control Group 1 (payloads / special):
	 *
	 *    ...
	 */
 
	/*
	 * Calculate roll error and apply P gain
	 */
	float roll_err = att->roll - att_sp->roll_body;
	actuators->control[0] = roll_err * p.roll_p;
 
	/*
	 * Calculate pitch error and apply P gain
	 */
	float pitch_err = att->pitch - att_sp->pitch_body;
	actuators->control[1] = pitch_err * p.pitch_p;
}
 
void control_heading(const struct vehicle_global_position_s *pos, const struct position_setpoint_s *sp,
		     const struct vehicle_attitude_s *att, struct vehicle_attitude_setpoint_s *att_sp)
{
 
	/*
	 * Calculate heading error of current position to desired position
	 */
 
	float bearing = get_bearing_to_next_waypoint(pos->lat, pos->lon, sp->lat, sp->lon);
 
	/* calculate heading error */
	float yaw_err = att->yaw - bearing;
	/* apply control gain */
	att_sp->roll_body = yaw_err * p.hdng_p;
 
	/* limit output, this commonly is a tuning parameter, too */
	if (att_sp->roll_body < -0.6f) {
		att_sp->roll_body = -0.6f;
 
	} else if (att_sp->roll_body > 0.6f) {
		att_sp->roll_body = 0.6f;
	}
}
 
/* Main Thread */
int test_main(int argc, char *argv[])
{
	/* read arguments */
	bool verbose = false;
 
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			verbose = true;
		}
	}
 
	/* welcome user (warnx prints a line, including an appended\n, with variable arguments */
	warnx("[test] started");
 
	/* initialize parameters, first the handles, then the values */
	parameters_init(&ph);
	parameters_update(&ph, &p);
 
 
	/*
	 * PX4 uses a publish/subscribe design pattern to enable
	 * multi-threaded communication.
	 *
	 * The most elegant aspect of this is that controllers and
	 * other processes can either 'react' to new data, or run
	 * at their own pace.
	 *
	 * PX4 developer guide:
	 * https://pixhawk.ethz.ch/px4/dev/shared_object_communication
	 *
	 * Wikipedia description:
	 * http://en.wikipedia.org/wiki/Publish–subscribe_pattern
	 *
	 */
 
 
 
 
	/*
	 * Declare and safely initialize all structs to zero.
	 *
	 * These structs contain the system state and things
	 * like attitude, position, the current waypoint, etc.
	 */
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));
	struct vehicle_rates_setpoint_s rates_sp;
	memset(&rates_sp, 0, sizeof(rates_sp));
	struct vehicle_global_position_s global_pos;
	memset(&global_pos, 0, sizeof(global_pos));
	struct manual_control_setpoint_s manual_sp;
	memset(&manual_sp, 0, sizeof(manual_sp));
	struct vehicle_status_s vstatus;
	memset(&vstatus, 0, sizeof(vstatus));
	struct position_setpoint_s global_sp;
	memset(&global_sp, 0, sizeof(global_sp));
 
	/* output structs - this is what is sent to the mixer */
	struct actuator_controls_s actuators;
	memset(&actuators, 0, sizeof(actuators));
 
 
	/* publish actuator controls with zero values */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++) {
		actuators.control[i] = 0.0f;
	}
 
	/*
	 * Advertise that this controller will publish actuator
	 * control values and the rate setpoint
	 */
	orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);
	orb_advert_t rates_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &rates_sp);
 
	/* subscribe to topics. */
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	int manual_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
	int global_sp_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	int param_sub = orb_subscribe(ORB_ID(parameter_update));
 
	/* Setup of loop */
 
	struct pollfd fds[2] = {{ .fd = param_sub, .events = POLLIN },
		{ .fd = att_sub, .events = POLLIN }
	};
 
	while (!thread_should_exit) {
 
		/*
		 * Wait for a sensor or param update, check for exit condition every 500 ms.
		 * This means that the execution will block here without consuming any resources,
		 * but will continue to execute the very moment a new attitude measurement or
		 * a param update is published. So no latency in contrast to the polling
		 * design pattern (do not confuse the poll() system call with polling).
		 *
		 * This design pattern makes the controller also agnostic of the attitude
		 * update speed - it runs as fast as the attitude updates with minimal latency.
		 */
		int ret = poll(fds, 2, 500);
 
		if (ret < 0) {
			/*
			 * Poll error, this will not really happen in practice,
			 * but its good design practice to make output an error message.
			 */
			warnx("poll error");
 
		} else if (ret == 0) {
			/* no return value = nothing changed for 500 ms, ignore */
		} else {
 
			/* only update parameters if they changed */
			if (fds[0].revents & POLLIN) {
				/* read from param to clear updated flag (uORB API requirement) */
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), param_sub, &update);
 
				/* if a param update occured, re-read our parameters */
				parameters_update(&ph, &p);
			}
 
			/* only run controller if attitude changed */
			if (fds[1].revents & POLLIN) {
 
 
				/* Check if there is a new position measurement or position setpoint */
				bool pos_updated;
				orb_check(global_pos_sub, &pos_updated);
				bool global_sp_updated;
				orb_check(global_sp_sub, &global_sp_updated);
				bool manual_sp_updated;
				orb_check(manual_sp_sub, &manual_sp_updated);
 
				/* get a local copy of attitude */
				orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
 
				if (global_sp_updated) {
					struct position_setpoint_triplet_s triplet;
					orb_copy(ORB_ID(position_setpoint_triplet), global_sp_sub, &triplet);
					memcpy(&global_sp, &triplet.current, sizeof(global_sp));
				}
 
				printf("say hello!");
			}
		}
	}
 
	printf("[ex_fixedwing_control] exiting, stopping all motors.\n");
	thread_running = false;
 
	/* kill all outputs */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++) {
		actuators.control[i] = 0.0f;
	}
 
	orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
 
	fflush(stdout);
 
	return 0;
}
 
/* Startup Functions */
 
static void
usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}
 
	fprintf(stderr, "usage: ex_fixedwing_control {start|stop|status}\n\n");
	exit(1);
}
 
/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to px4_task_spawn_cmd().
 */
int ex_fixedwing_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
	}
 
	if (!strcmp(argv[1], "start")) {
 
		if (thread_running) {
			printf("ex_fixedwing_control already running\n");
			/* this is not an error */
			exit(0);
		}
 
		thread_should_exit = false;
		deamon_task = px4_task_spawn_cmd("ex_fixedwing_control",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 20,
						 2048,
						 fixedwing_control_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		thread_running = true;
		exit(0);
	}
 
	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}
 
	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tex_fixedwing_control is running\n");
 
		} else {
			printf("\tex_fixedwing_control not started\n");
		}
 
		exit(0);
	}
 
	usage("unrecognized command");
	exit(1);
}