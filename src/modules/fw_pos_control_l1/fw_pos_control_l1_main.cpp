 /****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


/**
 * @file fw_pos_control_l1_main.c
 * Implementation of a generic position controller based on the L1 norm. Outputs a bank / roll
 * angle, equivalent to a lateral motion (for copters and rovers).
 *
 * Original publication for horizontal control class:
 *    S. Park, J. Deyst, and J. P. How, "A New Nonlinear Guidance Logic for Trajectory Tracking,"
 *    Proceedings of the AIAA Guidance, Navigation and Control
 *    Conference, Aug 2004. AIAA-2004-4900.
 *
 * Original implementation for total energy control class:
 *    Paul Riseborough and Andrew Tridgell, 2013 (code in lib/external_lgpl)
 *
 * More details and acknowledgements in the referenced library headers.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

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
#include <drivers/drv_accel.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/navigation_capabilities.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <mavlink/mavlink_log.h>
#include <launchdetection/LaunchDetector.h>
#include <ecl/l1/ecl_l1_pos_controller.h>
#include <external_lgpl/tecs/tecs.h>
#include "landingslope.h"
#include "mtecs/mTecs.h"
#include <platforms/px4_defines.h>

static int	_control_task = -1;			/**< task handle for sensor task */
#define HDG_HOLD_DIST_NEXT 			3000.0f 	// initial distance of waypoint in front of plane in heading hold mode
#define HDG_HOLD_REACHED_DIST 		1000.0f 	// distance (plane to waypoint in front) at which waypoints are reset in heading hold mode
#define HDG_HOLD_SET_BACK_DIST 		100.0f 		// distance by which previous waypoint is set behind the plane
#define HDG_HOLD_YAWRATE_THRESH 	0.15f 		// max yawrate at which plane locks yaw for heading hold mode
#define HDG_HOLD_MAN_INPUT_THRESH 	0.01f 		// max manual roll input from user which does not change the locked heading
#define TAKEOFF_IDLE				0.2f 		// idle speed for POSCTRL/ATTCTRL (when landed and throttle stick > 0)

static constexpr float THROTTLE_THRESH = 0.05f; 	///< max throttle from user which will not lead to motors spinning up in altitude controlled modes
static constexpr float MANUAL_THROTTLE_CLIMBOUT_THRESH = 0.85f;	///< a throttle / pitch input above this value leads to the system switching to climbout mode
static constexpr float ALTHOLD_EPV_RESET_THRESH = 5.0f;

/**
 * L1 control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int fw_pos_control_l1_main(int argc, char *argv[]);

using namespace launchdetection;

class FixedwingPositionControl
{
public:
	/**
	 * Constructor
	 */
	FixedwingPositionControl();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~FixedwingPositionControl();

	/**
	 * Start the sensors task.
	 *
	 * @return	OK on success.
	 */
	static int	start();

	/**
	 * Task status
	 *
	 * @return	true if the mainloop is running
	 */
	bool		task_running() { return _task_running; }

private:
	int		_mavlink_fd;

	bool		_task_should_exit;		/**< if true, sensor task should exit */
	bool		_task_running;			/**< if true, task is running in its mainloop */

	int		_global_pos_sub;
	int		_pos_sp_triplet_sub;
	int		_att_sub;			/**< vehicle attitude subscription */
	int		_airspeed_sub;			/**< airspeed subscription */
	int		_control_mode_sub;		/**< control mode subscription */
	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int 		_params_sub;			/**< notification of parameter updates */
	int 		_manual_control_sub;		/**< notification of manual control updates */
	int		_sensor_combined_sub;		/**< for body frame accelerations */

	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint */
	orb_advert_t	_tecs_status_pub;		/**< TECS status publication */
	orb_advert_t	_nav_capabilities_pub;		/**< navigation capabilities publication */

	struct vehicle_attitude_s			_att;				/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s		_att_sp;			/**< vehicle attitude setpoint */
	struct navigation_capabilities_s		_nav_capabilities;		/**< navigation capabilities */
	struct manual_control_setpoint_s		_manual;			/**< r/c channel data */
	struct airspeed_s				_airspeed;			/**< airspeed */
	struct vehicle_control_mode_s			_control_mode;			/**< control mode */
	struct vehicle_status_s				_vehicle_status;		/**< vehicle status */
	struct vehicle_global_position_s		_global_pos;			/**< global vehicle position */
	struct position_setpoint_triplet_s		_pos_sp_triplet;		/**< triplet of mission items */
	struct sensor_combined_s			_sensor_combined;		/**< for body frame accelerations */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	float	_hold_alt;				/**< hold altitude for altitude mode */
	float	_takeoff_ground_alt;				/**< ground altitude at which plane was launched */
	float	_hdg_hold_yaw;				/**< hold heading for velocity mode */
	bool	_hdg_hold_enabled;			/**< heading hold enabled */
	bool	_yaw_lock_engaged;			/**< yaw is locked for heading hold */
	float	_althold_epv;				/**< the position estimate accuracy when engaging alt hold */
	bool	_was_in_deadband;				/**< wether the last stick input was in althold deadband */
	struct position_setpoint_s _hdg_hold_prev_wp;	/**< position where heading hold started */
	struct position_setpoint_s _hdg_hold_curr_wp;	/**< position to which heading hold flies */
	hrt_abstime _control_position_last_called; /**<last call of control_position  */

	/* land states */
	bool land_noreturn_horizontal;
	bool land_noreturn_vertical;
	bool land_stayonground;
	bool land_motor_lim;
	bool land_onslope;
	bool land_useterrain;

	bool _was_in_air;	/**< indicated wether the plane was in the air in the previous interation*/
	hrt_abstime _time_went_in_air;	/**< time at which the plane went in the air */

	/* takeoff/launch states */
	LaunchDetectionResult launch_detection_state;

	bool last_manual;				///< true if the last iteration was in manual mode (used to determine when a reset is needed)

	/* Landingslope object */
	Landingslope landingslope;
	float flare_curve_alt_rel_last;

	/* heading hold */
	float target_bearing;

	/* Launch detection */
	launchdetection::LaunchDetector launchDetector;

	/* throttle and airspeed states */
	float _airspeed_error;				///< airspeed error to setpoint in m/s
	bool _airspeed_valid;				///< flag if a valid airspeed estimate exists
	uint64_t _airspeed_last_valid;			///< last time airspeed was valid. Used to detect sensor failures
	float _groundspeed_undershoot;			///< ground speed error to min. speed in m/s
	bool _global_pos_valid;				///< global position is valid
	math::Matrix<3, 3> _R_nb;			///< current attitude

	ECL_L1_Pos_Controller				_l1_control;
	TECS						_tecs;
	fwPosctrl::mTecs				_mTecs;
	enum FW_POSCTRL_MODE {
		FW_POSCTRL_MODE_AUTO,
		FW_POSCTRL_MODE_POSITION,
		FW_POSCTRL_MODE_ALTITUDE,
		FW_POSCTRL_MODE_OTHER
	} _control_mode_current;			///< used to check the mode in the last control loop iteration. Use to check if the last iteration was in the same mode.

	struct {
		float l1_period;
		float l1_damping;

		float time_const;
		float time_const_throt;
		float min_sink_rate;
		float max_sink_rate;
		float max_climb_rate;
		float climbout_diff;
		float heightrate_p;
		float heightrate_ff;
		float speedrate_p;
		float throttle_damp;
		float integrator_gain;
		float vertical_accel_limit;
		float height_comp_filter_omega;
		float speed_comp_filter_omega;
		float roll_throttle_compensation;
		float speed_weight;
		float pitch_damping;

		float airspeed_min;
		float airspeed_trim;
		float airspeed_max;

		float pitch_limit_min;
		float pitch_limit_max;
		float roll_limit;
		float throttle_min;
		float throttle_max;
		float throttle_cruise;
		float throttle_slew_max;

		float throttle_land_max;

		float land_slope_angle;
		float land_H1_virt;
		float land_flare_alt_relative;
		float land_thrust_lim_alt_relative;
		float land_heading_hold_horizontal_distance;
		float land_flare_pitch_min_deg;
		float land_flare_pitch_max_deg;
		int land_use_terrain_estimate;

	}		_parameters;			/**< local copies of interesting parameters */

	struct {

		param_t l1_period;
		param_t l1_damping;

		param_t time_const;
		param_t time_const_throt;
		param_t min_sink_rate;
		param_t max_sink_rate;
		param_t max_climb_rate;
		param_t climbout_diff;
		param_t heightrate_p;
		param_t heightrate_ff;
		param_t speedrate_p;
		param_t throttle_damp;
		param_t integrator_gain;
		param_t vertical_accel_limit;
		param_t height_comp_filter_omega;
		param_t speed_comp_filter_omega;
		param_t roll_throttle_compensation;
		param_t speed_weight;
		param_t pitch_damping;

		param_t airspeed_min;
		param_t airspeed_trim;
		param_t airspeed_max;

		param_t pitch_limit_min;
		param_t pitch_limit_max;
		param_t roll_limit;
		param_t throttle_min;
		param_t throttle_max;
		param_t throttle_cruise;
		param_t throttle_slew_max;

		param_t throttle_land_max;

		param_t land_slope_angle;
		param_t land_H1_virt;
		param_t land_flare_alt_relative;
		param_t land_thrust_lim_alt_relative;
		param_t land_heading_hold_horizontal_distance;
		param_t land_flare_pitch_min_deg;
		param_t land_flare_pitch_max_deg;
		param_t land_use_terrain_estimate;

	}		_parameter_handles;		/**< handles for interesting parameters */


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Update control outputs
	 *
	 */
	void		control_update();

	/**
	 * Check for changes in control mode
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in vehicle status.
	 */
	void		vehicle_status_poll();

	/**
	 * Check for manual setpoint updates.
	 */
	bool		vehicle_manual_control_setpoint_poll();

	/**
	 * Check for airspeed updates.
	 */
	bool		vehicle_airspeed_poll();

	/**
	 * Check for position updates.
	 */
	void		vehicle_attitude_poll();

	/**
	 * Check for accel updates.
	 */
	void		vehicle_sensor_combined_poll();

	/**
	 * Check for set triplet updates.
	 */
	void		vehicle_setpoint_poll();

	/**
	 * Publish navigation capabilities
	 */
	void		navigation_capabilities_publish();

	/**
	 * Get a new waypoint based on heading and distance from current position
	 *
	 * @param heading the heading to fly to
	 * @param distance the distance of the generated waypoint
	 * @param waypoint_prev the waypoint at the current position
	 * @param waypoint_next the waypoint in the heading direction
	 */
	void		get_waypoint_heading_distance(float heading, float distance,
					struct position_setpoint_s &waypoint_prev, struct position_setpoint_s &waypoint_next, bool flag_init);

	/**
	 * Return the terrain estimate during landing: uses the wp altitude value or the terrain estimate if available
	 */
	float		get_terrain_altitude_landing(float land_setpoint_alt, const struct vehicle_global_position_s &global_pos);

	/**
	 * Check if we are in a takeoff situation
	 */
	bool in_takeoff_situation();

	/**
	 * Do takeoff help when in altitude controlled modes
	 * @param hold_altitude altitude setpoint for controller
	 * @param pitch_limit_min minimum pitch allowed
	 */
	void 		do_takeoff_help(float *hold_altitude, float *pitch_limit_min);

	/**
	 * Update desired altitude base on user pitch stick input
	 *
	 * @param dt Time step
	 * @return true if climbout mode was requested by user (climb with max rate and min airspeed)
	 */
	bool		update_desired_altitude(float dt);

	/**
	 * Control position.
	 */
	bool		control_position(const math::Vector<2> &global_pos, const math::Vector<3> &ground_speed,
					 const struct position_setpoint_triplet_s &_pos_sp_triplet);

	float		calculate_target_airspeed(float airspeed_demand);
	void		calculate_gndspeed_undershoot(const math::Vector<2> &current_position, const math::Vector<2> &ground_speed_2d, const struct position_setpoint_triplet_s &pos_sp_triplet);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main();

	/*
	 * Reset takeoff state
	 */
	void		reset_takeoff_state();

	/*
	 * Reset landing state
	 */
	void		reset_landing_state();

	/*
	 * Call TECS : a wrapper function to call one of the TECS implementations (mTECS is called only if enabled via parameter)
	 * XXX need to clean up/remove this function once mtecs fully replaces TECS
	 */
	void tecs_update_pitch_throttle(float alt_sp, float v_sp, float eas2tas,
			float pitch_min_rad, float pitch_max_rad,
			float throttle_min, float throttle_max, float throttle_cruise,
			bool climbout_mode, float climbout_pitch_min_rad,
			float altitude,
			const math::Vector<3> &ground_speed,
			unsigned mode = tecs_status_s::TECS_MODE_NORMAL,
			bool pitch_max_special = false);

};

namespace l1_control
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

FixedwingPositionControl	*g_control = nullptr;
}

FixedwingPositionControl::FixedwingPositionControl() :

	_mavlink_fd(-1),
	_task_should_exit(false),
	_task_running(false),

/* subscriptions */
	_global_pos_sub(-1),
	_pos_sp_triplet_sub(-1),
	_att_sub(-1),
	_airspeed_sub(-1),
	_control_mode_sub(-1),
	_vehicle_status_sub(-1),
	_params_sub(-1),
	_manual_control_sub(-1),
	_sensor_combined_sub(-1),

/* publications */
	_attitude_sp_pub(nullptr),
	_tecs_status_pub(nullptr),
	_nav_capabilities_pub(nullptr),

/* states */
	_att(),
	_att_sp(),
	_nav_capabilities(),
	_manual(),
	_airspeed(),
	_control_mode(),
	_vehicle_status(),
	_global_pos(),
	_pos_sp_triplet(),
	_sensor_combined(),

/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "fw l1 control")),

	_hold_alt(0.0f),
	_takeoff_ground_alt(0.0f),
	_hdg_hold_yaw(0.0f),
	_hdg_hold_enabled(false),
	_yaw_lock_engaged(false),
	_althold_epv(0.0f),
	_was_in_deadband(false),
	_hdg_hold_prev_wp{},
	_hdg_hold_curr_wp{},
	_control_position_last_called(0),

	land_noreturn_horizontal(false),
	land_noreturn_vertical(false),
	land_stayonground(false),
	land_motor_lim(false),
	land_onslope(false),
	land_useterrain(false),
	_was_in_air(false),
	_time_went_in_air(0),
	launch_detection_state(LAUNCHDETECTION_RES_NONE),
	last_manual(false),
	landingslope(),
	flare_curve_alt_rel_last(0.0f),
	target_bearing(0.0f),
	launchDetector(),
	_airspeed_error(0.0f),
	_airspeed_valid(false),
	_airspeed_last_valid(0),
	_groundspeed_undershoot(0.0f),
	_global_pos_valid(false),
	_l1_control(),
	_mTecs(),
	_control_mode_current(FW_POSCTRL_MODE_OTHER)
{
	_nav_capabilities.turn_distance = 0.0f;

	_parameter_handles.l1_period = param_find("FW_L1_PERIOD");
	_parameter_handles.l1_damping = param_find("FW_L1_DAMPING");

	_parameter_handles.airspeed_min = param_find("FW_AIRSPD_MIN");
	_parameter_handles.airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_parameter_handles.airspeed_max = param_find("FW_AIRSPD_MAX");

	_parameter_handles.pitch_limit_min = param_find("FW_P_LIM_MIN");
	_parameter_handles.pitch_limit_max = param_find("FW_P_LIM_MAX");
	_parameter_handles.roll_limit = param_find("FW_R_LIM");
	_parameter_handles.throttle_min = param_find("FW_THR_MIN");
	_parameter_handles.throttle_max = param_find("FW_THR_MAX");
	_parameter_handles.throttle_slew_max = param_find("FW_THR_SLEW_MAX");
	_parameter_handles.throttle_cruise = param_find("FW_THR_CRUISE");
	_parameter_handles.throttle_land_max = param_find("FW_THR_LND_MAX");

	_parameter_handles.land_slope_angle = param_find("FW_LND_ANG");
	_parameter_handles.land_H1_virt = param_find("FW_LND_HVIRT");
	_parameter_handles.land_flare_alt_relative = param_find("FW_LND_FLALT");
	_parameter_handles.land_thrust_lim_alt_relative = param_find("FW_LND_TLALT");
	_parameter_handles.land_heading_hold_horizontal_distance = param_find("FW_LND_HHDIST");
	_parameter_handles.land_flare_pitch_min_deg = param_find("FW_FLARE_PMIN");
	_parameter_handles.land_flare_pitch_max_deg = param_find("FW_FLARE_PMAX");
	_parameter_handles.land_use_terrain_estimate= param_find("FW_LND_USETER");

	_parameter_handles.time_const = 			param_find("FW_T_TIME_CONST");
	_parameter_handles.time_const_throt = 			param_find("FW_T_THRO_CONST");
	_parameter_handles.min_sink_rate = 			param_find("FW_T_SINK_MIN");
	_parameter_handles.max_sink_rate =			param_find("FW_T_SINK_MAX");
	_parameter_handles.max_climb_rate =			param_find("FW_T_CLMB_MAX");
	_parameter_handles.climbout_diff =			param_find("FW_CLMBOUT_DIFF");
	_parameter_handles.throttle_damp = 			param_find("FW_T_THR_DAMP");
	_parameter_handles.integrator_gain =			param_find("FW_T_INTEG_GAIN");
	_parameter_handles.vertical_accel_limit =		param_find("FW_T_VERT_ACC");
	_parameter_handles.height_comp_filter_omega =		param_find("FW_T_HGT_OMEGA");
	_parameter_handles.speed_comp_filter_omega =		param_find("FW_T_SPD_OMEGA");
	_parameter_handles.roll_throttle_compensation = 	param_find("FW_T_RLL2THR");
	_parameter_handles.speed_weight = 			param_find("FW_T_SPDWEIGHT");
	_parameter_handles.pitch_damping = 			param_find("FW_T_PTCH_DAMP");
	_parameter_handles.heightrate_p =			param_find("FW_T_HRATE_P");
	_parameter_handles.heightrate_ff =			param_find("FW_T_HRATE_FF");
	_parameter_handles.speedrate_p =			param_find("FW_T_SRATE_P");

	/* fetch initial parameter values */
	parameters_update();
}

FixedwingPositionControl::~FixedwingPositionControl()
{
	if (_control_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	l1_control::g_control = nullptr;
}

int
FixedwingPositionControl::parameters_update()
{

	/* L1 control parameters */
	param_get(_parameter_handles.l1_damping, &(_parameters.l1_damping));
	param_get(_parameter_handles.l1_period, &(_parameters.l1_period));

	param_get(_parameter_handles.airspeed_min, &(_parameters.airspeed_min));
	param_get(_parameter_handles.airspeed_trim, &(_parameters.airspeed_trim));
	param_get(_parameter_handles.airspeed_max, &(_parameters.airspeed_max));

	param_get(_parameter_handles.pitch_limit_min, &(_parameters.pitch_limit_min));
	param_get(_parameter_handles.pitch_limit_max, &(_parameters.pitch_limit_max));
	param_get(_parameter_handles.roll_limit, &(_parameters.roll_limit));
	param_get(_parameter_handles.throttle_min, &(_parameters.throttle_min));
	param_get(_parameter_handles.throttle_max, &(_parameters.throttle_max));
	param_get(_parameter_handles.throttle_cruise, &(_parameters.throttle_cruise));
	param_get(_parameter_handles.throttle_slew_max, &(_parameters.throttle_slew_max));

	param_get(_parameter_handles.throttle_land_max, &(_parameters.throttle_land_max));

	param_get(_parameter_handles.time_const, &(_parameters.time_const));
	param_get(_parameter_handles.time_const_throt, &(_parameters.time_const_throt));
	param_get(_parameter_handles.min_sink_rate, &(_parameters.min_sink_rate));
	param_get(_parameter_handles.max_sink_rate, &(_parameters.max_sink_rate));
	param_get(_parameter_handles.throttle_damp, &(_parameters.throttle_damp));
	param_get(_parameter_handles.integrator_gain, &(_parameters.integrator_gain));
	param_get(_parameter_handles.vertical_accel_limit, &(_parameters.vertical_accel_limit));
	param_get(_parameter_handles.height_comp_filter_omega, &(_parameters.height_comp_filter_omega));
	param_get(_parameter_handles.speed_comp_filter_omega, &(_parameters.speed_comp_filter_omega));
	param_get(_parameter_handles.roll_throttle_compensation, &(_parameters.roll_throttle_compensation));
	param_get(_parameter_handles.speed_weight, &(_parameters.speed_weight));
	param_get(_parameter_handles.pitch_damping, &(_parameters.pitch_damping));
	param_get(_parameter_handles.max_climb_rate, &(_parameters.max_climb_rate));
	param_get(_parameter_handles.climbout_diff, &(_parameters.climbout_diff));

	param_get(_parameter_handles.heightrate_p, &(_parameters.heightrate_p));
	param_get(_parameter_handles.heightrate_ff, &(_parameters.heightrate_ff));
	param_get(_parameter_handles.speedrate_p, &(_parameters.speedrate_p));

	param_get(_parameter_handles.land_slope_angle, &(_parameters.land_slope_angle));
	param_get(_parameter_handles.land_H1_virt, &(_parameters.land_H1_virt));
	param_get(_parameter_handles.land_flare_alt_relative, &(_parameters.land_flare_alt_relative));
	param_get(_parameter_handles.land_thrust_lim_alt_relative, &(_parameters.land_thrust_lim_alt_relative));

	/* check if negative value for 2/3 of flare altitude is set for throttle cut */
	if (_parameters.land_thrust_lim_alt_relative < 0.0f) {
		_parameters.land_thrust_lim_alt_relative = 0.66f * _parameters.land_flare_alt_relative;
	}

	param_get(_parameter_handles.land_heading_hold_horizontal_distance, &(_parameters.land_heading_hold_horizontal_distance));
	param_get(_parameter_handles.land_flare_pitch_min_deg, &(_parameters.land_flare_pitch_min_deg));
	param_get(_parameter_handles.land_flare_pitch_max_deg, &(_parameters.land_flare_pitch_max_deg));
	param_get(_parameter_handles.land_use_terrain_estimate, &(_parameters.land_use_terrain_estimate));

	_l1_control.set_l1_damping(_parameters.l1_damping);
	_l1_control.set_l1_period(_parameters.l1_period);
	_l1_control.set_l1_roll_limit(math::radians(_parameters.roll_limit));

	_tecs.set_time_const(_parameters.time_const);
	_tecs.set_time_const_throt(_parameters.time_const_throt);
	_tecs.set_min_sink_rate(_parameters.min_sink_rate);
	_tecs.set_max_sink_rate(_parameters.max_sink_rate);
	_tecs.set_throttle_damp(_parameters.throttle_damp);
	_tecs.set_throttle_slewrate(_parameters.throttle_slew_max);
	_tecs.set_integrator_gain(_parameters.integrator_gain);
	_tecs.set_vertical_accel_limit(_parameters.vertical_accel_limit);
	_tecs.set_height_comp_filter_omega(_parameters.height_comp_filter_omega);
	_tecs.set_speed_comp_filter_omega(_parameters.speed_comp_filter_omega);
	_tecs.set_roll_throttle_compensation(_parameters.roll_throttle_compensation);
	_tecs.set_speed_weight(_parameters.speed_weight);
	_tecs.set_pitch_damping(_parameters.pitch_damping);
	_tecs.set_indicated_airspeed_min(_parameters.airspeed_min);
	_tecs.set_indicated_airspeed_max(_parameters.airspeed_max);
	_tecs.set_max_climb_rate(_parameters.max_climb_rate);
	_tecs.set_heightrate_p(_parameters.heightrate_p);
	_tecs.set_heightrate_ff(_parameters.heightrate_ff);
	_tecs.set_speedrate_p(_parameters.speedrate_p);

	/* sanity check parameters */
	if (_parameters.airspeed_max < _parameters.airspeed_min ||
	    _parameters.airspeed_max < 5.0f ||
	    _parameters.airspeed_min > 100.0f ||
	    _parameters.airspeed_trim < _parameters.airspeed_min ||
	    _parameters.airspeed_trim > _parameters.airspeed_max) {
		warnx("error: airspeed parameters invalid");
		return 1;
	}

	/* Update the landing slope */
	landingslope.update(math::radians(_parameters.land_slope_angle), _parameters.land_flare_alt_relative, _parameters.land_thrust_lim_alt_relative, _parameters.land_H1_virt);

	/* Update and publish the navigation capabilities */
	_nav_capabilities.landing_slope_angle_rad = landingslope.landing_slope_angle_rad();
	_nav_capabilities.landing_horizontal_slope_displacement = landingslope.horizontal_slope_displacement();
	_nav_capabilities.landing_flare_length = landingslope.flare_length();
	navigation_capabilities_publish();

	/* Update Launch Detector Parameters */
	launchDetector.updateParams();

	/* Update the mTecs */
	_mTecs.updateParams();

	return OK;
}

void
FixedwingPositionControl::vehicle_control_mode_poll()
{
	bool updated;

	orb_check(_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
	}
}

void
FixedwingPositionControl::vehicle_status_poll()
{
	bool updated;

	orb_check(_vehicle_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
	}
}

bool
FixedwingPositionControl::vehicle_airspeed_poll()
{
	/* check if there is an airspeed update or if it timed out */
	// 这里主要的用于检查 airspeed 的信息是否更新了。或者数据请求超时
	// 给一个标志位的变量，用于指明这些数值是否有新的数据。
	bool airspeed_updated;
	// 这里有一个小细节，可以看到airspeed_updated 前面使用了取地址的符号，这样的话，就可以改变airspeed_updated的值了。
	orb_check(_airspeed_sub, &airspeed_updated);
        // 如果更新了数据，就可以执行下面的语句了
	if (airspeed_updated) {
		orb_copy(ORB_ID(airspeed), _airspeed_sub, &_airspeed);
		_airspeed_valid = true;
		_airspeed_last_valid = hrt_absolute_time();

	} else {

		/* no airspeed updates for one second */
		// 如果等待一分钟后都没有数据的更新，那么执行下面的语句
		if (_airspeed_valid && (hrt_absolute_time() - _airspeed_last_valid) > 1e6) {
			_airspeed_valid = false;
		}
	}

	/* update TECS state */
	_tecs.enable_airspeed(_airspeed_valid);

	return airspeed_updated;
}

bool
FixedwingPositionControl::vehicle_manual_control_setpoint_poll()
{
	bool manual_updated;

	/* Check if manual setpoint has changed */
	// 检查位置信息是否改变了。
	orb_check(_manual_control_sub, &manual_updated);

	if (manual_updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sub, &_manual);
	}

	// 这里存在一个小疑问，为什么空速机信息和手动控制设置航点信息需要放回值，而其他的poll函数不需要

	return manual_updated;
}


void
FixedwingPositionControl::vehicle_attitude_poll()
{
	/* check if there is a new position */
	bool att_updated;
	orb_check(_att_sub, &att_updated);

	if (att_updated) {
		orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);

		/* set rotation matrix */
		// 设置回转矩阵
		for (int i = 0; i < 3; i++) 
			for (int j = 0; j < 3; j++)
				_R_nb(i, j) = PX4_R(_att.R, i, j);
	}
}

void
FixedwingPositionControl::vehicle_sensor_combined_poll()
{
	/* check if there is a new position */
	bool sensors_updated;
	orb_check(_sensor_combined_sub, &sensors_updated);

	if (sensors_updated) {
		orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);
	}
}

void
FixedwingPositionControl::vehicle_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool pos_sp_triplet_updated;
	orb_check(_pos_sp_triplet_sub, &pos_sp_triplet_updated);

	if (pos_sp_triplet_updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);
	}
}

void
FixedwingPositionControl::task_main_trampoline(int argc, char *argv[])
{
	// 从函数的声明上可以看出来，这个是主函数
	// 这里的g_control 很奇怪
	// 开始创建一个对象，用于控制飞行控制
	l1_control::g_control = new FixedwingPositionControl();

	if (l1_control::g_control == nullptr) {
		// 输出内存不足的提示信息
		warnx("OUT OF MEM");
		return;
	}

	/* only returns on exit */
	// 在退出的时候只要返回一个值即可
	// g_control 的任务完成了，可以隐退了
	l1_control::g_control->task_main();
	delete l1_control::g_control;
	l1_control::g_control = nullptr;
}

float
FixedwingPositionControl::calculate_target_airspeed(float airspeed_demand)
{
	float airspeed;

	if (_airspeed_valid) {
		airspeed = _airspeed.true_airspeed_m_s;

	} else {
		airspeed = _parameters.airspeed_min + (_parameters.airspeed_max - _parameters.airspeed_min) / 2.0f;
	}

	/* cruise airspeed for all modes unless modified below */
	float target_airspeed = airspeed_demand;

	/* add minimum ground speed undershoot (only non-zero in presence of sufficient wind) */
	target_airspeed += _groundspeed_undershoot;

	if (0/* throttle nudging enabled */) {
		//target_airspeed += nudge term.
	}

	/* sanity check: limit to range */
	target_airspeed = math::constrain(target_airspeed, _parameters.airspeed_min, _parameters.airspeed_max);

	/* plain airspeed error */
	_airspeed_error = target_airspeed - airspeed;

	return target_airspeed;
}

void
FixedwingPositionControl::calculate_gndspeed_undershoot(const math::Vector<2> &current_position, const math::Vector<2> &ground_speed_2d, const struct position_setpoint_triplet_s &pos_sp_triplet)
{

	if (pos_sp_triplet.current.valid && !(pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER)) {

		/* rotate ground speed vector with current attitude */
		math::Vector<2> yaw_vector(_R_nb(0, 0), _R_nb(1, 0));
		yaw_vector.normalize();
		float ground_speed_body = yaw_vector * ground_speed_2d;

		/* The minimum desired ground speed is the minimum airspeed projected on to the ground using the altitude and horizontal difference between the waypoints if available*/
		float distance = 0.0f;
		float delta_altitude = 0.0f;
		if (pos_sp_triplet.previous.valid) {
			distance = get_distance_to_next_waypoint(pos_sp_triplet.previous.lat, pos_sp_triplet.previous.lon, pos_sp_triplet.current.lat, pos_sp_triplet.current.lon);
			delta_altitude = pos_sp_triplet.current.alt - pos_sp_triplet.previous.alt;
		} else {
			distance = get_distance_to_next_waypoint(current_position(0), current_position(1), pos_sp_triplet.current.lat, pos_sp_triplet.current.lon);
			delta_altitude = pos_sp_triplet.current.alt -  _global_pos.alt;
		}

		float ground_speed_desired = _parameters.airspeed_min * cosf(atan2f(delta_altitude, distance));


		/*
		 * Ground speed undershoot is the amount of ground velocity not reached
		 * by the plane. Consequently it is zero if airspeed is >= min ground speed
		 * and positive if airspeed < min ground speed.
		 *
		 * This error value ensures that a plane (as long as its throttle capability is
		 * not exceeded) travels towards a waypoint (and is not pushed more and more away
		 * by wind). Not countering this would lead to a fly-away.
		 */
		_groundspeed_undershoot = math::max(ground_speed_desired - ground_speed_body, 0.0f);

	} else {
		_groundspeed_undershoot = 0;
	}
}

void FixedwingPositionControl::navigation_capabilities_publish()
{
	_nav_capabilities.timestamp = hrt_absolute_time();

	if (_nav_capabilities_pub != nullptr) {
		orb_publish(ORB_ID(navigation_capabilities), _nav_capabilities_pub, &_nav_capabilities);
	} else {
		_nav_capabilities_pub = orb_advertise(ORB_ID(navigation_capabilities), &_nav_capabilities);
	}
}

void FixedwingPositionControl::get_waypoint_heading_distance(float heading, float distance,
					struct position_setpoint_s &waypoint_prev, struct position_setpoint_s &waypoint_next, bool flag_init)
{
	waypoint_prev.valid = true;
	waypoint_prev.alt = _hold_alt;

	if (flag_init) {
		// on init set first waypoint to momentary position
		waypoint_prev.lat = _global_pos.lat - cos(heading) * (double)(HDG_HOLD_SET_BACK_DIST) / 1e6;
		waypoint_prev.lon = _global_pos.lon - sin(heading) * (double)(HDG_HOLD_SET_BACK_DIST) / 1e6;
	} else {
		/*
		for previous waypoint use the one still in front of us but shift it such that it is
		located on the desired flight path but HDG_HOLD_SET_BACK_DIST behind us
		*/
		waypoint_prev.lat = waypoint_next.lat - cos(heading) * (double)(HDG_HOLD_REACHED_DIST + HDG_HOLD_SET_BACK_DIST) / 1e6;
		waypoint_prev.lon = waypoint_next.lon - sin(heading) * (double)(HDG_HOLD_REACHED_DIST + HDG_HOLD_SET_BACK_DIST) / 1e6;
	}

	waypoint_next.valid = true;
	waypoint_next.lat = waypoint_prev.lat + cos(heading) * (double)(distance + HDG_HOLD_SET_BACK_DIST) / 1e6;
	waypoint_next.lon = waypoint_prev.lon + sin(heading) * (double)(distance + HDG_HOLD_SET_BACK_DIST) / 1e6;
	waypoint_next.alt = _hold_alt;
}

float FixedwingPositionControl::get_terrain_altitude_landing(float land_setpoint_alt, const struct vehicle_global_position_s &global_pos)
{
	if (!isfinite(global_pos.terrain_alt)) {
		return land_setpoint_alt;
	}

	/* Decide if the terrain estimation can be used, once we switched to using the terrain we stick with it
	 * for the whole landing */
	if (_parameters.land_use_terrain_estimate && (global_pos.terrain_alt_valid || land_useterrain)) {
		if(!land_useterrain) {
			mavlink_log_info(_mavlink_fd, "#audio: Landing, using terrain estimate");
			land_useterrain = true;
		}
		return global_pos.terrain_alt;
	} else {
		return land_setpoint_alt;
	}
}

bool FixedwingPositionControl::update_desired_altitude(float dt)
{
	/*
	 * The complete range is -1..+1, so this is 6%
	 * of the up or down range or 3% of the total range.
	 */
	const float deadBand = 0.06f;

	/*
	 * The correct scaling of the complete range needs
	 * to account for the missing part of the slope
	 * due to the deadband
	 */
	const float factor = 1.0f - deadBand;

	/* Climbout mode sets maximum throttle and pitch up */
	bool climbout_mode = false;

	/*
	 * Reset the hold altitude to the current altitude if the uncertainty
	 * changes significantly.
	 * This is to guard against uncommanded altitude changes
	 * when the altitude certainty increases or decreases.
	 */

	if (fabsf(_althold_epv - _global_pos.epv) > ALTHOLD_EPV_RESET_THRESH) {
		_hold_alt = _global_pos.alt;
		_althold_epv = _global_pos.epv;
	}

	/*
	 * Manual control has as convention the rotation around
	 * an axis. Positive X means to rotate positively around
	 * the X axis in NED frame, which is pitching down
	 */
	if (_manual.x > deadBand) {
		/* pitching down */
		float pitch = -(_manual.x - deadBand) / factor;
		_hold_alt += (_parameters.max_sink_rate * dt) * pitch;
		_was_in_deadband = false;
	} else if (_manual.x < - deadBand) {
		/* pitching up */
		float pitch = -(_manual.x + deadBand) / factor;
		_hold_alt += (_parameters.max_climb_rate * dt) * pitch;
		_was_in_deadband = false;
		climbout_mode = (pitch > MANUAL_THROTTLE_CLIMBOUT_THRESH);
	} else if (!_was_in_deadband) {
		 /* store altitude at which manual.x was inside deadBand
		  * The aircraft should immediately try to fly at this altitude
		  * as this is what the pilot expects when he moves the stick to the center */
		_hold_alt = _global_pos.alt;
		_althold_epv = _global_pos.epv;
		_was_in_deadband = true;
	}

	return climbout_mode;
}

bool FixedwingPositionControl::in_takeoff_situation() {
	const hrt_abstime delta_takeoff = 10000000;
	const float throttle_threshold = 0.1f;

	if (hrt_elapsed_time(&_time_went_in_air) < delta_takeoff && _manual.z > throttle_threshold && _global_pos.alt <= _takeoff_ground_alt + _parameters.climbout_diff) {
		return true;
	}

	return false;
}

void FixedwingPositionControl::do_takeoff_help(float *hold_altitude, float *pitch_limit_min)
{
	/* demand "climbout_diff" m above ground if user switched into this mode during takeoff */
	if (in_takeoff_situation()) {
		*hold_altitude = _takeoff_ground_alt + _parameters.climbout_diff;
		*pitch_limit_min = math::radians(10.0f);
	} else {
		*pitch_limit_min = _parameters.pitch_limit_min;
	}
}

bool
FixedwingPositionControl::control_position(const math::Vector<2> &current_position, const math::Vector<3> &ground_speed,
		const struct position_setpoint_triplet_s &pos_sp_triplet)
{
	float dt = 0.01; // Using non zero value to a avoid division by zero
	if (_control_position_last_called > 0) {
		dt = (float)hrt_elapsed_time(&_control_position_last_called) * 1e-6f;
	}
	_control_position_last_called = hrt_absolute_time();

	bool setpoint = true;

	float eas2tas = 1.0f; // XXX calculate actual number based on current measurements

	/* filter speed and altitude for controller */
	// 过滤速度和高度控制器
	math::Vector<3> accel_body(_sensor_combined.accelerometer_m_s2);
	math::Vector<3> accel_earth = _R_nb * accel_body;

	/* tell TECS to update its state, but let it know when it cannot actually control the plane */
	// 告诉TECS更新其状态，但让它知道什么时候不能实际控制飞机
	bool in_air_alt_control = (!_vehicle_status.condition_landed &&
		(_control_mode.flag_control_auto_enabled ||
		 _control_mode.flag_control_velocity_enabled ||
		 _control_mode.flag_control_altitude_enabled));

	/* update TECS filters */
	// 更新 TECS 滤波器
	if (!_mTecs.getEnabled()) {
		_tecs.update_state(_global_pos.alt, _airspeed.indicated_airspeed_m_s, _R_nb,
			accel_body, accel_earth, (_global_pos.timestamp > 0), in_air_alt_control);
	}

	math::Vector<2> ground_speed_2d = {ground_speed(0), ground_speed(1)};
	calculate_gndspeed_undershoot(current_position, ground_speed_2d, pos_sp_triplet);

	/* define altitude error */
	float altitude_error = _pos_sp_triplet.current.alt - _global_pos.alt;

	/* no throttle limit as default */
	float throttle_max = 1.0f;

	/* save time when airplane is in air */
	if (!_was_in_air && !_vehicle_status.condition_landed) {
		_was_in_air = true;
		_time_went_in_air = hrt_absolute_time();
		_takeoff_ground_alt = _global_pos.alt;
	}
	/* reset flag when airplane landed */
	if (_vehicle_status.condition_landed) {
		_was_in_air = false;
	}

	if (_control_mode.flag_control_auto_enabled &&
			pos_sp_triplet.current.valid) {
		/* AUTONOMOUS FLIGHT */

		/* Reset integrators if switching to this mode from a other mode in which posctl was not active */
		if (_control_mode_current == FW_POSCTRL_MODE_OTHER) {
			/* reset integrators */
			if (_mTecs.getEnabled()) {
				_mTecs.resetIntegrators();
				_mTecs.resetDerivatives(_airspeed.true_airspeed_m_s);
			}
		}
		_control_mode_current = FW_POSCTRL_MODE_AUTO;

		/* reset hold altitude */
		_hold_alt = _global_pos.alt;
		/* reset hold yaw */
		_hdg_hold_yaw = _att.yaw;

		/* get circle mode */
		bool was_circle_mode = _l1_control.circle_mode();

		/* restore speed weight, in case changed intermittently (e.g. in landing handling) */
		_tecs.set_speed_weight(_parameters.speed_weight);

		/* current waypoint (the one currently heading for) */
		math::Vector<2> next_wp((float)pos_sp_triplet.current.lat, (float)pos_sp_triplet.current.lon);

		/* current waypoint (the one currently heading for) */
		math::Vector<2> curr_wp((float)pos_sp_triplet.current.lat, (float)pos_sp_triplet.current.lon);

		/* Initialize attitude controller integrator reset flags to 0 */
		_att_sp.roll_reset_integral = false;
		_att_sp.pitch_reset_integral = false;
		_att_sp.yaw_reset_integral = false;

		/* previous waypoint */
		math::Vector<2> prev_wp;

		if (pos_sp_triplet.previous.valid) {
			prev_wp(0) = (float)pos_sp_triplet.previous.lat;
			prev_wp(1) = (float)pos_sp_triplet.previous.lon;

		} else {
			/*
			 * No valid previous waypoint, go for the current wp.
			 * This is automatically handled by the L1 library.
			 */
			prev_wp(0) = (float)pos_sp_triplet.current.lat;
			prev_wp(1) = (float)pos_sp_triplet.current.lon;

		}

		if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
			_att_sp.thrust = 0.0f;
			_att_sp.roll_body = 0.0f;
			_att_sp.pitch_body = 0.0f;

		} else if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
			/* waypoint is a plain navigation waypoint */
			_l1_control.navigate_waypoints(prev_wp, curr_wp, current_position, ground_speed_2d);
			_att_sp.roll_body = _l1_control.nav_roll();
			_att_sp.yaw_body = _l1_control.nav_bearing();

			tecs_update_pitch_throttle(_pos_sp_triplet.current.alt, calculate_target_airspeed(_parameters.airspeed_trim), eas2tas,
						math::radians(_parameters.pitch_limit_min), math::radians(_parameters.pitch_limit_max),
						_parameters.throttle_min, _parameters.throttle_max, _parameters.throttle_cruise,
						false, math::radians(_parameters.pitch_limit_min), _global_pos.alt, ground_speed);

		} else if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {

			/* waypoint is a loiter waypoint */
			_l1_control.navigate_loiter(curr_wp, current_position, pos_sp_triplet.current.loiter_radius,
						  pos_sp_triplet.current.loiter_direction, ground_speed_2d);
			_att_sp.roll_body = _l1_control.nav_roll();
			_att_sp.yaw_body = _l1_control.nav_bearing();

			if (in_takeoff_situation()) {
					/* limit roll motion to ensure enough lift */
					_att_sp.roll_body = math::constrain(_att_sp.roll_body, math::radians(-15.0f),
							math::radians(15.0f));
			}

			tecs_update_pitch_throttle(_pos_sp_triplet.current.alt, calculate_target_airspeed(_parameters.airspeed_trim), eas2tas,
						math::radians(_parameters.pitch_limit_min), math::radians(_parameters.pitch_limit_max),
						_parameters.throttle_min, _parameters.throttle_max, _parameters.throttle_cruise,
						false, math::radians(_parameters.pitch_limit_min), _global_pos.alt, ground_speed);

		} else if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {

			float bearing_lastwp_currwp = get_bearing_to_next_waypoint(prev_wp(0), prev_wp(1), curr_wp(0), curr_wp(1));
			float bearing_airplane_currwp = get_bearing_to_next_waypoint(current_position(0), current_position(1), curr_wp(0), curr_wp(1));

			/* Horizontal landing control */
			/* switch to heading hold for the last meters, continue heading hold after */
			float wp_distance = get_distance_to_next_waypoint(current_position(0), current_position(1), curr_wp(0), curr_wp(1));
			/* calculate a waypoint distance value which is 0 when the aircraft is behind the waypoint */
			float wp_distance_save = wp_distance;
			if (fabsf(bearing_airplane_currwp - bearing_lastwp_currwp) >= math::radians(90.0f)) {
				wp_distance_save = 0.0f;
			}

			//warnx("wp dist: %d, alt err: %d, noret: %s", (int)wp_distance, (int)altitude_error, (land_noreturn) ? "YES" : "NO");
			if (wp_distance < _parameters.land_heading_hold_horizontal_distance || land_noreturn_horizontal) {

				/* heading hold, along the line connecting this and the last waypoint */

				if (!land_noreturn_horizontal) {//set target_bearing in first occurrence
					if (pos_sp_triplet.previous.valid) {
						target_bearing = bearing_lastwp_currwp;
					} else {
						target_bearing = _att.yaw;
					}
					mavlink_log_info(_mavlink_fd, "#audio: Landing, heading hold");
				}

//					warnx("NORET: %d, target_bearing: %d, yaw: %d", (int)land_noreturn_horizontal, (int)math::degrees(target_bearing), (int)math::degrees(_att.yaw));

				_l1_control.navigate_heading(target_bearing, _att.yaw, ground_speed_2d);

				/* limit roll motion to prevent wings from touching the ground first */
				// 限制侧倾运动，以防止副翼接触地面
				_att_sp.roll_body = math::constrain(_att_sp.roll_body, math::radians(-10.0f), math::radians(10.0f));

				land_noreturn_horizontal = true;

			} else {

				/* normal navigation */
				_l1_control.navigate_waypoints(prev_wp, curr_wp, current_position, ground_speed_2d);
			}
			// 是不是sp 就代表了我们需要给定的值，也就是飞控需要给出的值呢。
			// 这里通过导航算法给出控制量
			_att_sp.roll_body = _l1_control.nav_roll();
			_att_sp.yaw_body = _l1_control.nav_bearing();


			/* Vertical landing control */
			// 飞行器的着落控制
			//xxx: using the tecs altitude controller for slope control for now
			/* apply minimum pitch (flare) and limit roll if close to touch down, altitude error is negative (going down) */
			// XXX this could make a great param
			// 这个着落的参数好像全部都限定死了。
			float throttle_land = _parameters.throttle_min + (_parameters.throttle_max - _parameters.throttle_min) * 0.1f;
			float airspeed_land = 1.3f * _parameters.airspeed_min;
			// 设置靠近油门
			float airspeed_approach = 1.3f * _parameters.airspeed_min;

			/* Get an estimate of the terrain altitude if available, otherwise terrain_alt will be
			 * equal to _pos_sp_triplet.current.alt */
			// 估计一下地形
			float terrain_alt = get_terrain_altitude_landing(_pos_sp_triplet.current.alt, _global_pos);

			/* Calculate distance (to landing waypoint) and altitude of last ordinary waypoint L */
			float L_altitude_rel = _pos_sp_triplet.previous.valid ?
				_pos_sp_triplet.previous.alt - terrain_alt : 0.0f;

			float landing_slope_alt_rel_desired = landingslope.getLandingSlopeRelativeAltitudeSave(wp_distance, bearing_lastwp_currwp, bearing_airplane_currwp);

			/* Check if we should start flaring with a vertical and a
			 * horizontal limit (with some tolerance)
			 * The horizontal limit is only applied when we are in front of the wp
			 */
			if (((_global_pos.alt < terrain_alt + landingslope.flare_relative_alt()) &&
						(wp_distance_save < landingslope.flare_length() + 5.0f)) ||
					land_noreturn_vertical) {  //checking for land_noreturn to avoid unwanted climb out
				/* land with minimal speed */

//					/* force TECS to only control speed with pitch, altitude is only implicitely controlled now */
//					_tecs.set_speed_weight(2.0f);

				/* kill the throttle if param requests it */
				throttle_max = _parameters.throttle_max;

				 if (_global_pos.alt < terrain_alt + landingslope.motor_lim_relative_alt() || land_motor_lim) {
					throttle_max = math::min(throttle_max, _parameters.throttle_land_max);
					if (!land_motor_lim) {
						land_motor_lim  = true;
						mavlink_log_info(_mavlink_fd, "#audio: Landing, limiting throttle");
					}

				 }

				float flare_curve_alt_rel = landingslope.getFlareCurveRelativeAltitudeSave(wp_distance, bearing_lastwp_currwp, bearing_airplane_currwp);

				/* avoid climbout */
				if ((flare_curve_alt_rel_last < flare_curve_alt_rel && land_noreturn_vertical) || land_stayonground)
				{
					flare_curve_alt_rel = 0.0f; // stay on ground
					land_stayonground = true;
				}

				tecs_update_pitch_throttle(terrain_alt + flare_curve_alt_rel,
						calculate_target_airspeed(airspeed_land), eas2tas,
						 math::radians(_parameters.land_flare_pitch_min_deg),
						 math::radians(_parameters.land_flare_pitch_max_deg),
						0.0f, throttle_max, throttle_land,
						false,  land_motor_lim ? math::radians(_parameters.land_flare_pitch_min_deg) : math::radians(_parameters.pitch_limit_min),
						_global_pos.alt, ground_speed,
						land_motor_lim ? tecs_status_s::TECS_MODE_LAND_THROTTLELIM : tecs_status_s::TECS_MODE_LAND);

				if (!land_noreturn_vertical) {
					mavlink_log_info(_mavlink_fd, "#audio: Landing, flaring");
					land_noreturn_vertical = true;
				}
				//warnx("Landing:  flare, _global_pos.alt  %.1f, flare_curve_alt %.1f, flare_curve_alt_last %.1f, flare_length %.1f, wp_distance %.1f", _global_pos.alt, flare_curve_alt, flare_curve_alt_last, flare_length, wp_distance);

				flare_curve_alt_rel_last = flare_curve_alt_rel;
			} else {

				 /* intersect glide slope:
				  * minimize speed to approach speed
				  * if current position is higher than the slope follow the glide slope (sink to the
				  * glide slope)
				  * also if the system captures the slope it should stay
				  * on the slope (bool land_onslope)
				  * if current position is below the slope continue at previous wp altitude
				  * until the intersection with slope
				  * */
				float altitude_desired_rel;
				if (_global_pos.alt > terrain_alt + landing_slope_alt_rel_desired || land_onslope) {
					/* stay on slope */
					altitude_desired_rel = landing_slope_alt_rel_desired;
					if (!land_onslope) {
						mavlink_log_info(_mavlink_fd, "#audio: Landing, on slope");
						land_onslope = true;
					}
				} else {
					/* continue horizontally */
					altitude_desired_rel =  _pos_sp_triplet.previous.valid ? L_altitude_rel :
						_global_pos.alt - terrain_alt;
				}

				tecs_update_pitch_throttle(terrain_alt + altitude_desired_rel,
						calculate_target_airspeed(airspeed_approach), eas2tas,
						math::radians(_parameters.pitch_limit_min),
						math::radians(_parameters.pitch_limit_max),
						_parameters.throttle_min,
						_parameters.throttle_max,
						_parameters.throttle_cruise,
						false,
						math::radians(_parameters.pitch_limit_min),
						_global_pos.alt,
						ground_speed);
			}

		} else if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {

			/* Perform launch detection */
			if (launchDetector.launchDetectionEnabled() &&
					launch_detection_state != LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS) {
				/* Inform user that launchdetection is running */
				static hrt_abstime last_sent = 0;
				if(hrt_absolute_time() - last_sent > 4e6) {
					mavlink_log_critical(_mavlink_fd, "Launchdetection running");
					last_sent = hrt_absolute_time();
				}

				/* Detect launch */
				launchDetector.update(_sensor_combined.accelerometer_m_s2[0]);

				/* update our copy of the laucn detection state */
				launch_detection_state = launchDetector.getLaunchDetected();
			} else	{
				/* no takeoff detection --> fly */
				launch_detection_state = LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS;
			}

			/* Set control values depending on the detection state */
			if (launch_detection_state != LAUNCHDETECTION_RES_NONE) {
				/* Launch has been detected, hence we have to control the plane. */

				_l1_control.navigate_waypoints(prev_wp, curr_wp, current_position, ground_speed_2d);
				_att_sp.roll_body = _l1_control.nav_roll();
				_att_sp.yaw_body = _l1_control.nav_bearing();

				/* Select throttle: only in LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS we want to use
				 * full throttle, otherwise we use the preTakeOff Throttle */
				float takeoff_throttle = launch_detection_state !=
					LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS ?
					launchDetector.getThrottlePreTakeoff() : _parameters.throttle_max;

				/* select maximum pitch: the launchdetector may impose another limit for the pitch
				 * depending on the state of the launch */
				float takeoff_pitch_max_deg = launchDetector.getPitchMax(_parameters.pitch_limit_max);
				float takeoff_pitch_max_rad = math::radians(takeoff_pitch_max_deg);

				/* apply minimum pitch and limit roll if target altitude is not within climbout_diff
				 * meters */
				if (_parameters.climbout_diff > 0.001f && altitude_error > _parameters.climbout_diff) {

					/* enforce a minimum of 10 degrees pitch up on takeoff, or take parameter */
					tecs_update_pitch_throttle(_pos_sp_triplet.current.alt,
							calculate_target_airspeed(1.3f * _parameters.airspeed_min),
							eas2tas,
							math::radians(_parameters.pitch_limit_min),
							takeoff_pitch_max_rad,
							_parameters.throttle_min, takeoff_throttle,
							_parameters.throttle_cruise,
							true,
							math::max(math::radians(pos_sp_triplet.current.pitch_min),
							math::radians(10.0f)),
							_global_pos.alt,
							ground_speed,
							tecs_status_s::TECS_MODE_TAKEOFF,
							takeoff_pitch_max_deg != _parameters.pitch_limit_max);

					/* limit roll motion to ensure enough lift */
					_att_sp.roll_body = math::constrain(_att_sp.roll_body, math::radians(-15.0f),
							math::radians(15.0f));

				} else {
					tecs_update_pitch_throttle(_pos_sp_triplet.current.alt,
							calculate_target_airspeed(_parameters.airspeed_trim),
							eas2tas,
							math::radians(_parameters.pitch_limit_min),
							math::radians(_parameters.pitch_limit_max),
							_parameters.throttle_min,
							takeoff_throttle,
							_parameters.throttle_cruise,
							false,
							math::radians(_parameters.pitch_limit_min),
							_global_pos.alt,
							ground_speed);
				}
			} else {
				/* Tell the attitude controller to stop integrating while we are waiting
				 * for the launch */
				_att_sp.roll_reset_integral = true;
				_att_sp.pitch_reset_integral = true;
				_att_sp.yaw_reset_integral = true;

				/* Set default roll and pitch setpoints during detection phase */
				_att_sp.roll_body = 0.0f;
				_att_sp.pitch_body = math::max(math::radians(pos_sp_triplet.current.pitch_min),
						math::radians(10.0f));
			}

		}

		/* reset landing state */
		if (pos_sp_triplet.current.type != position_setpoint_s::SETPOINT_TYPE_LAND) {
			reset_landing_state();
		}

		/* reset takeoff/launch state */
		if (pos_sp_triplet.current.type != position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {
			reset_takeoff_state();
		}

		if (was_circle_mode && !_l1_control.circle_mode()) {
			/* just kicked out of loiter, reset roll integrals */
			_att_sp.roll_reset_integral = true;
		}

	} else if (_control_mode.flag_control_velocity_enabled &&
			_control_mode.flag_control_altitude_enabled) {
		/* POSITION CONTROL: pitch stick moves altitude setpoint, throttle stick sets airspeed,
		   heading is set to a distant waypoint */

		if (_control_mode_current != FW_POSCTRL_MODE_POSITION) {
			/* Need to init because last loop iteration was in a different mode */
			_hold_alt = _global_pos.alt;
			_hdg_hold_yaw = _att.yaw;
			_hdg_hold_enabled = false; // this makes sure the waypoints are reset below
			_yaw_lock_engaged = false;
		}
		/* Reset integrators if switching to this mode from a other mode in which posctl was not active */
		if (_control_mode_current == FW_POSCTRL_MODE_OTHER) {
			/* reset integrators */
			if (_mTecs.getEnabled()) {
				_mTecs.resetIntegrators();
				_mTecs.resetDerivatives(_airspeed.true_airspeed_m_s);
			}
		}
		_control_mode_current = FW_POSCTRL_MODE_POSITION;

		/* Get demanded airspeed */
		float altctrl_airspeed = _parameters.airspeed_min +
					  (_parameters.airspeed_max - _parameters.airspeed_min) *
					  _manual.z;

		/* update desired altitude based on user pitch stick input */
		bool climbout_requested = update_desired_altitude(dt);

		/* if we assume that user is taking off then help by demanding altitude setpoint well above ground 
		* and set limit to pitch angle to prevent stearing into ground
		*/
		float pitch_limit_min;
		do_takeoff_help(&_hold_alt, &pitch_limit_min);


		/* throttle limiting */
		throttle_max = _parameters.throttle_max;
		if (fabsf(_manual.z) < THROTTLE_THRESH) {
			throttle_max = 0.0f;
		}

		tecs_update_pitch_throttle(_hold_alt,
				altctrl_airspeed,
				eas2tas,
				math::radians(_parameters.pitch_limit_min),
				math::radians(_parameters.pitch_limit_max),
				_parameters.throttle_min,
				throttle_max,
				_parameters.throttle_cruise,
				climbout_requested,
				((climbout_requested) ? math::radians(10.0f) : pitch_limit_min),
				_global_pos.alt,
				ground_speed,
				tecs_status_s::TECS_MODE_NORMAL);

		/* heading control */

		if (fabsf(_manual.y) < HDG_HOLD_MAN_INPUT_THRESH) {
			/* heading / roll is zero, lock onto current heading */
			if (fabsf(_att.yawspeed) < HDG_HOLD_YAWRATE_THRESH && !_yaw_lock_engaged) {
				// little yaw movement, lock to current heading
				_yaw_lock_engaged = true;

			}

			/* user tries to do a takeoff in heading hold mode, reset the yaw setpoint on every iteration
			  to make sure the plane does not start rolling
			*/
			if (in_takeoff_situation()) {
				_hdg_hold_enabled = false;
				_yaw_lock_engaged = true;
			}

			if (_yaw_lock_engaged) {

				/* just switched back from non heading-hold to heading hold */
				if (!_hdg_hold_enabled) {
					_hdg_hold_enabled = true;
					_hdg_hold_yaw = _att.yaw;

					get_waypoint_heading_distance(_hdg_hold_yaw, HDG_HOLD_DIST_NEXT, _hdg_hold_prev_wp, _hdg_hold_curr_wp, true);
				}

				/* we have a valid heading hold position, are we too close? */
				if (get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon,
					_hdg_hold_curr_wp.lat, _hdg_hold_curr_wp.lon) < HDG_HOLD_REACHED_DIST) {
					get_waypoint_heading_distance(_hdg_hold_yaw, HDG_HOLD_DIST_NEXT, _hdg_hold_prev_wp, _hdg_hold_curr_wp, false);
				}

				math::Vector<2> prev_wp;
				prev_wp(0) = (float)_hdg_hold_prev_wp.lat;
				prev_wp(1) = (float)_hdg_hold_prev_wp.lon;

				math::Vector<2> curr_wp;
				curr_wp(0) = (float)_hdg_hold_curr_wp.lat;
				curr_wp(1) = (float)_hdg_hold_curr_wp.lon;

				/* populate l1 control setpoint */
				_l1_control.navigate_waypoints(prev_wp, curr_wp, current_position, ground_speed_2d);

				_att_sp.roll_body = _l1_control.nav_roll();
				_att_sp.yaw_body = _l1_control.nav_bearing();

				if (in_takeoff_situation()) {
					/* limit roll motion to ensure enough lift */
					_att_sp.roll_body = math::constrain(_att_sp.roll_body, math::radians(-15.0f),
							math::radians(15.0f));
				}
			}
		} else {
			_hdg_hold_enabled = false;
			_yaw_lock_engaged = false;
		}

	} else if (_control_mode.flag_control_altitude_enabled) {
		/* ALTITUDE CONTROL: pitch stick moves altitude setpoint, throttle stick sets airspeed */

		if (_control_mode_current != FW_POSCTRL_MODE_POSITION && _control_mode_current != FW_POSCTRL_MODE_ALTITUDE) {
			/* Need to init because last loop iteration was in a different mode */
			_hold_alt = _global_pos.alt;
		}
		/* Reset integrators if switching to this mode from a other mode in which posctl was not active */
		if (_control_mode_current == FW_POSCTRL_MODE_OTHER) {
			/* reset integrators */
			if (_mTecs.getEnabled()) {
				_mTecs.resetIntegrators();
				_mTecs.resetDerivatives(_airspeed.true_airspeed_m_s);
			}
		}
		_control_mode_current = FW_POSCTRL_MODE_ALTITUDE;

		/* Get demanded airspeed */
		float altctrl_airspeed = _parameters.airspeed_min +
					  (_parameters.airspeed_max - _parameters.airspeed_min) *
					  _manual.z;

		/* update desired altitude based on user pitch stick input */
		bool climbout_requested = update_desired_altitude(dt);

		/* if we assume that user is taking off then help by demanding altitude setpoint well above ground 
		* and set limit to pitch angle to prevent stearing into ground
		*/
		float pitch_limit_min;
		do_takeoff_help(&_hold_alt, &pitch_limit_min);

		/* throttle limiting */
		throttle_max = _parameters.throttle_max;
		if (fabsf(_manual.z) < THROTTLE_THRESH) {
			throttle_max = 0.0f;
		}

		tecs_update_pitch_throttle(_hold_alt,
				altctrl_airspeed,
				eas2tas,
				math::radians(_parameters.pitch_limit_min),
				math::radians(_parameters.pitch_limit_max),
				_parameters.throttle_min,
				throttle_max,
				_parameters.throttle_cruise,
				climbout_requested,
				((climbout_requested) ? math::radians(10.0f) : pitch_limit_min),
				_global_pos.alt,
				ground_speed,
				tecs_status_s::TECS_MODE_NORMAL);
	} else {
		_control_mode_current = FW_POSCTRL_MODE_OTHER;

		/** MANUAL FLIGHT **/

		/* reset hold altitude */
		_hold_alt = _global_pos.alt;

		/* no flight mode applies, do not publish an attitude setpoint */
		setpoint = false;

		/* reset landing and takeoff state */
		if (!last_manual) {
			reset_landing_state();
			reset_takeoff_state();
		}
	}

	/* Copy thrust output for publication */
	if (_vehicle_status.engine_failure || _vehicle_status.engine_failure_cmd) {
		/* Set thrust to 0 to minimize damage */
		_att_sp.thrust = 0.0f;
	} else if (_control_mode_current ==  FW_POSCTRL_MODE_AUTO && // launchdetector only available in auto
			pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF &&
			launch_detection_state != LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS) {
		 /* making sure again that the correct thrust is used,
		 * without depending on library calls for safety reasons */
		_att_sp.thrust = launchDetector.getThrottlePreTakeoff();
	} else if (_control_mode_current ==  FW_POSCTRL_MODE_AUTO &&
			pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
		_att_sp.thrust = 0.0f;
	} else {
		/* Copy thrust and pitch values from tecs */
		if (_vehicle_status.condition_landed &&
			(_control_mode_current == FW_POSCTRL_MODE_POSITION || _control_mode_current == FW_POSCTRL_MODE_ALTITUDE))
		{
			// when we are landed in these modes we want the motor to spin
			_att_sp.thrust = math::min(TAKEOFF_IDLE, throttle_max);
		} else {
			_att_sp.thrust = math::min(_mTecs.getEnabled() ? _mTecs.getThrottleSetpoint() :
					_tecs.get_throttle_demand(), throttle_max);
		}


	}

	/* During a takeoff waypoint while waiting for launch the pitch sp is set
	 * already (not by tecs) */
	if (!(_control_mode_current ==  FW_POSCTRL_MODE_AUTO &&
				pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF &&
				launch_detection_state == LAUNCHDETECTION_RES_NONE)) {
		_att_sp.pitch_body = _mTecs.getEnabled() ? _mTecs.getPitchSetpoint() : _tecs.get_pitch_demand();
	}

	if (_control_mode.flag_control_position_enabled) {
		last_manual = false;
	} else {
		last_manual = true;
	}


	return setpoint;
}

void
FixedwingPositionControl::task_main()
{

	// 一波三折，最后执行算法的地方在这里
	/*
	 * do subscriptions
	 */
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

	/* rate limit control mode updates to 5Hz */
	// 限制了 control mode 的更新速率为 5HZ
	orb_set_interval(_control_mode_sub, 200);
	/* rate limit vehicle status updates to 5Hz */
	// 限制了 飞行器状态更新的速率为 5HZ
	orb_set_interval(_vehicle_status_sub, 200);
	/* rate limit position updates to 50 Hz */
	// 限制了位置更新的速率为50HZ
	orb_set_interval(_global_pos_sub, 20);

	/* abort on a nonzero return value from the parameter init */
	// 当初始参数的时候如果没有返回0 那么就退出
	if (parameters_update()) {
		/* parameter setup went wrong, abort */
		// 参数启动错误，无奈退出
		warnx("aborting startup due to errors.");
		_task_should_exit = true;
	}

	/* wakeup source(s) */
	// 唤醒一些资源
	struct pollfd fds[2];

	/* Setup of loop */
	// 启动循环
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _global_pos_sub;
	fds[1].events = POLLIN;

	// 这里就可以设置启动的标志了
	_task_running = true;

	while (!_task_should_exit) {

		/* wait for up to 500ms for data */
		// 每次进入到这里的时候，需要等待 500ms ，等待数据全部更新完
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		// 超时，周期性的检查 _task_should_exit 这个变量
		if (pret == 0)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		// 这个状态是不可取的，但是我们又不能为此做些什么，现在系统处于一个不健康的状态
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		/* check vehicle control mode for changes to publication state */
		// 检查此时飞行控制的模式
		vehicle_control_mode_poll();

		/* check vehicle status for changes to publication state */
		// 检查飞行的飞行状态
		vehicle_status_poll();

		/* only update parameters if they changed */
		// 如果参数改变了，那么就更新他
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			// 读取新的参数从参数列表中，并且清除更新的标志
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			// 从存储器中跟新参数
			parameters_update();
		}

		/* only run controller if position changed */
		// 只用位置改变了才运行控制器
		if (fds[1].revents & POLLIN) {
			perf_begin(_loop_perf);

			/* XXX Hack to get mavlink output going */
			// 这里运用到了 mavlink 了
			if (_mavlink_fd < 0) {
				/* try to open the mavlink log device every once in a while */
				// 每当运行到这里，都需要打开 mavlink 日志文件
				_mavlink_fd = px4_open(MAVLINK_LOG_DEVICE, 0);
			}

			/* load local copies */
			// 启动一个副本
			orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);

			// XXX add timestamp check
			_global_pos_valid = true;

			vehicle_attitude_poll();
			vehicle_setpoint_poll();
			vehicle_sensor_combined_poll();
			vehicle_airspeed_poll();
			vehicle_manual_control_setpoint_poll();
			// vehicle_baro_poll();
			// 这里用于到了矢量这个概念，使用的坐标系是NED
			math::Vector<3> ground_speed(_global_pos.vel_n, _global_pos.vel_e,  _global_pos.vel_d);
			// 这里运用到了第二个矢量，用于表示现在机体的位置。经纬度
			math::Vector<2> current_position((float)_global_pos.lat, (float)_global_pos.lon);

			/*
			 * Attempt to control position, on success (= sensors present and not in manual mode),
			 * publish setpoint.
			 * 试图取控制位置，（成功的标志是，使用当前的传感器的值，并且不在手动模式下）
			 */
			if (control_position(current_position, ground_speed, _pos_sp_triplet)) {
				_att_sp.timestamp = hrt_absolute_time();

				/* lazily publish the setpoint only once available */
				if (_attitude_sp_pub != nullptr && !_vehicle_status.is_rotary_wing) {
					/* publish the attitude setpoint */
					orb_publish(ORB_ID(vehicle_attitude_setpoint), _attitude_sp_pub, &_att_sp);

				} else if (_attitude_sp_pub == nullptr && !_vehicle_status.is_rotary_wing) {
					/* advertise and publish */
					_attitude_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_sp);
				}

				/* XXX check if radius makes sense here */
				float turn_distance = _l1_control.switch_distance(100.0f);

				/* lazily publish navigation capabilities */
				if ((hrt_elapsed_time(&_nav_capabilities.timestamp) > 1000000) || (fabsf(turn_distance - _nav_capabilities.turn_distance) > FLT_EPSILON
					&& turn_distance > 0)) {

					/* set new turn distance */
					_nav_capabilities.turn_distance = turn_distance;

					navigation_capabilities_publish();

				}

			}
			perf_end(_loop_perf);
		}

	}

	_task_running = false;

	warnx("exiting.\n");

	_control_task = -1;
	_exit(0);
}

void FixedwingPositionControl::reset_takeoff_state()
{
	launch_detection_state = LAUNCHDETECTION_RES_NONE;
	launchDetector.reset();
}

void FixedwingPositionControl::reset_landing_state()
{
	land_noreturn_horizontal = false;
	land_noreturn_vertical = false;
	land_stayonground = false;
	land_motor_lim = false;
	land_onslope = false;
	land_useterrain = false;
}

void FixedwingPositionControl::tecs_update_pitch_throttle(float alt_sp, float v_sp, float eas2tas,
		float pitch_min_rad, float pitch_max_rad,
		float throttle_min, float throttle_max, float throttle_cruise,
		bool climbout_mode, float climbout_pitch_min_rad,
		float altitude,
		const math::Vector<3> &ground_speed,
		unsigned mode, bool pitch_max_special)
{
	/* do not run tecs if we are not in air */
	if (_vehicle_status.condition_landed) {
		return;
	}

	if (_mTecs.getEnabled()) {
		/* Using mtecs library: prepare arguments for mtecs call */
		float flightPathAngle = 0.0f;
		float ground_speed_length = ground_speed.length();
		if (ground_speed_length > FLT_EPSILON) {
			flightPathAngle = -asinf(ground_speed(2)/ground_speed_length);
		}
		fwPosctrl::LimitOverride limitOverride;
		if (_vehicle_status.engine_failure || _vehicle_status.engine_failure_cmd) {
			/* Force the slow downwards spiral */
			limitOverride.enablePitchMinOverride(-1.0f);
			limitOverride.enablePitchMaxOverride(5.0f);

		} else if (climbout_mode) {
			limitOverride.enablePitchMinOverride(M_RAD_TO_DEG_F * climbout_pitch_min_rad);
		} else {
			limitOverride.disablePitchMinOverride();
		}

		if (pitch_max_special) {
			/* Use the maximum pitch from the argument */
			limitOverride.enablePitchMaxOverride(M_RAD_TO_DEG_F * pitch_max_rad);
		} else {
			/* use pitch max set by MT param */
			limitOverride.disablePitchMaxOverride();
		}
		_mTecs.updateAltitudeSpeed(flightPathAngle, altitude, alt_sp, _airspeed.true_airspeed_m_s, v_sp, mode,
				limitOverride);
	} else {
		if (_vehicle_status.engine_failure || _vehicle_status.engine_failure_cmd) {
			/* Force the slow downwards spiral */
			pitch_min_rad = M_DEG_TO_RAD_F * -1.0f;
			pitch_max_rad = M_DEG_TO_RAD_F * 5.0f;
		}

/* No underspeed protection in landing mode */
		_tecs.set_detect_underspeed_enabled(!(mode == tecs_status_s::TECS_MODE_LAND || mode == tecs_status_s::TECS_MODE_LAND_THROTTLELIM));

		/* Using tecs library */
		_tecs.update_pitch_throttle(_R_nb, _att.pitch, altitude, alt_sp, v_sp,
					    _airspeed.indicated_airspeed_m_s, eas2tas,
					    climbout_mode, climbout_pitch_min_rad,
					    throttle_min, throttle_max, throttle_cruise,
					    pitch_min_rad, pitch_max_rad);

		struct TECS::tecs_state s;
		_tecs.get_tecs_state(s);

		struct tecs_status_s t;

		t.timestamp = s.timestamp;

		switch (s.mode) {
			case TECS::ECL_TECS_MODE_NORMAL:
				t.mode = tecs_status_s::TECS_MODE_NORMAL;
				break;
			case TECS::ECL_TECS_MODE_UNDERSPEED:
				t.mode = tecs_status_s::TECS_MODE_UNDERSPEED;
				break;
			case TECS::ECL_TECS_MODE_BAD_DESCENT:
				t.mode = tecs_status_s::TECS_MODE_BAD_DESCENT;
				break;
			case TECS::ECL_TECS_MODE_CLIMBOUT:
				t.mode = tecs_status_s::TECS_MODE_CLIMBOUT;
				break;
		}

		t.altitudeSp			= s.hgt_dem;
		t.altitude_filtered		= s.hgt;
		t.airspeedSp			= s.spd_dem;
		t.airspeed_filtered		= s.spd;

		t.flightPathAngleSp		= s.dhgt_dem;
		t.flightPathAngle		= s.dhgt;
		t.flightPathAngleFiltered	= s.dhgt;

		t.airspeedDerivativeSp		= s.dspd_dem;
		t.airspeedDerivative		= s.dspd;

		t.totalEnergyRateSp		= s.thr;
		t.totalEnergyRate		= s.ithr;
		t.energyDistributionRateSp	= s.ptch;
		t.energyDistributionRate	= s.iptch;

		if (_tecs_status_pub != nullptr) {
			orb_publish(ORB_ID(tecs_status), _tecs_status_pub, &t);
		} else {
			_tecs_status_pub = orb_advertise(ORB_ID(tecs_status), &t);
		}
	}
}

int
FixedwingPositionControl::start()
{
	// assert的作用是现计算表达式 expression ，如果其值为假（即为0），那么它先向stderr打印一条出错信息，然后通过调用 abort 来终止程序运行
	// 下面语句用于检查侧向引导模块在这之前没有启动。防止重复启动
	ASSERT(_control_task == -1);

	/* start the task */
	// 启动这个主线程
	_control_task = px4_task_spawn_cmd("fw_pos_control_l1",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5, // 优先级和 姿态控制模块的优先级是一个级别的
				       1600,
				       (main_t)&FixedwingPositionControl::task_main_trampoline, // 这才是主函数可以从这个函数下手
				       nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int fw_pos_control_l1_main(int argc, char *argv[])
{
	if (argc < 2) {
		errx(1, "usage: fw_pos_control_l1 {start|stop|status}");
	}

	if (!strcmp(argv[1], "start")) {

		if (l1_control::g_control != nullptr)
			errx(1, "already running");

		// 这个模块使用了这个方式来启动这个线程
		if (OK != FixedwingPositionControl::start()) {
			err(1, "start failed");
		}

		/* avoid memory fragmentation by not exiting start handler until the task has fully started */
		// 这个方式可以避免误启动：内存碎片的原因可以导致它误启动，要全部启动，才能启动
		// 因为使用了while 所以可以理解为强制执行。
		while (l1_control::g_control == nullptr || !l1_control::g_control->task_running()) {
			usleep(50000);
			printf(".");
			// fflush函数可以用来清除读写缓冲区
			fflush(stdout);
		}
		printf("\n");

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (l1_control::g_control == nullptr)
			errx(1, "not running");

		delete l1_control::g_control;
		l1_control::g_control = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (l1_control::g_control) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
