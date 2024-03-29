/****************************************************************************
 *
 *   Copyright (C) 2012, 2014 PX4 Development Team. All rights reserved.
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
 * @file geo.h
 *
 * Definition of geo / math functions to perform geodesic calculations
 * 这部分代码用于大地的测算
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * Additional functions - @author Doug Weibel <douglas.weibel@colorado.edu>
 */

#pragma once
#include <platforms/px4_defines.h>
__BEGIN_DECLS

#include "geo_lookup/geo_mag_declination.h"

#include <stdbool.h>

#define CONSTANTS_ONE_G					9.80665f		/* m/s^2		*/
#define CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C		1.225f			/* kg/m^3		*/
#define CONSTANTS_AIR_GAS_CONST				287.1f 			/* J/(kg * K)		*/
#define CONSTANTS_ABSOLUTE_NULL_CELSIUS			-273.15f		/* °C			*/
#define CONSTANTS_RADIUS_OF_EARTH			6371000			/* meters (m)		*/

// XXX remove
struct crosstrack_error_s {
	bool past_end;		// Flag indicating we are past the end of the line/arc segment
	float distance;		// Distance in meters to closest point on line/arc
	float bearing;		// Bearing in radians to closest point on line/arc
} ;

/* lat/lon are in radians */
struct map_projection_reference_s {
	double lat_rad;
	double lon_rad;
	double sin_lat;
	double cos_lat;
	bool init_done;
	uint64_t timestamp;
};

struct globallocal_converter_reference_s {
	float alt;
	bool init_done;
};

/**
 * Checks if global projection was initialized
 * @return true if map was initialized before, false else
 */
__EXPORT bool map_projection_global_initialized(void);

/**
 * Checks if projection given as argument was initialized
 * @return true if map was initialized before, false else
 */
__EXPORT bool map_projection_initialized(const struct map_projection_reference_s *ref);

/**
 * Get the timestamp of the global map projection
 * @return the timestamp of the map_projection
 */
__EXPORT uint64_t map_projection_global_timestamp(void);

/**
 * Get the timestamp of the map projection given by the argument
 * @return the timestamp of the map_projection
 */
__EXPORT uint64_t map_projection_timestamp(const struct map_projection_reference_s *ref);

/**
 * Writes the reference values of the global projection to ref_lat and ref_lon
 * @return 0 if map_projection_init was called before, -1 else
 */
__EXPORT int map_projection_global_reference(double *ref_lat_rad, double *ref_lon_rad);

/**
 * Writes the reference values of the projection given by the argument to ref_lat and ref_lon
 * @return 0 if map_projection_init was called before, -1 else
 */
__EXPORT int map_projection_reference(const struct map_projection_reference_s *ref, double *ref_lat_rad, double *ref_lon_rad);

/**
 * Initializes the global map transformation.
 *
 * Initializes the transformation between the geographic coordinate system and
 * the azimuthal equidistant plane
 * @param lat in degrees (47.1234567°, not 471234567°)
 * @param lon in degrees (8.1234567°, not 81234567°)
 */
__EXPORT int map_projection_global_init(double lat_0, double lon_0, uint64_t timestamp);

/**
 * Initializes the map transformation given by the argument.
 *
 * Initializes the transformation between the geographic coordinate system and
 * the azimuthal equidistant plane
 * @param lat in degrees (47.1234567°, not 471234567°)
 * @param lon in degrees (8.1234567°, not 81234567°)
 */
__EXPORT int map_projection_init_timestamped(struct map_projection_reference_s *ref,
		double lat_0, double lon_0, uint64_t timestamp);

/**
 * Initializes the map transformation given by the argument and sets the timestamp to now.
 *
 * Initializes the transformation between the geographic coordinate system and
 * the azimuthal equidistant plane
 * @param lat in degrees (47.1234567°, not 471234567°)
 * @param lon in degrees (8.1234567°, not 81234567°)
 */
__EXPORT int map_projection_init(struct map_projection_reference_s *ref, double lat_0, double lon_0);

/**
 * Transforms a point in the geographic coordinate system to the local
 * azimuthal equidistant plane using the global projection
 * @param x north
 * @param y east
 * @param lat in degrees (47.1234567°, not 471234567°)
 * @param lon in degrees (8.1234567°, not 81234567°)
 * @return 0 if map_projection_init was called before, -1 else
 */
__EXPORT int map_projection_global_project(double lat, double lon, float *x, float *y);


 /* Transforms a point in the geographic coordinate system to the local
  * azimuthal equidistant plane using the projection given by the argument
 * @param x north
 * @param y east
 * @param lat in degrees (47.1234567°, not 471234567°)
 * @param lon in degrees (8.1234567°, not 81234567°)
 * @return 0 if map_projection_init was called before, -1 else
 */
__EXPORT int map_projection_project(const struct map_projection_reference_s *ref, double lat, double lon, float *x, float *y);

/**
 * Transforms a point in the local azimuthal equidistant plane to the
 * geographic coordinate system using the global projection
 *
 * @param x north
 * @param y east
 * @param lat in degrees (47.1234567°, not 471234567°)
 * @param lon in degrees (8.1234567°, not 81234567°)
 * @return 0 if map_projection_init was called before, -1 else
 */
__EXPORT int map_projection_global_reproject(float x, float y, double *lat, double *lon);

/**
 * Transforms a point in the local azimuthal equidistant plane to the
 * geographic coordinate system using the projection given by the argument
 *
 * @param x north
 * @param y east
 * @param lat in degrees (47.1234567°, not 471234567°)
 * @param lon in degrees (8.1234567°, not 81234567°)
 * @return 0 if map_projection_init was called before, -1 else
 */
__EXPORT int map_projection_reproject(const struct map_projection_reference_s *ref, float x, float y, double *lat, double *lon);

/**
 * Get reference position of the global map projection
 */
__EXPORT int map_projection_global_getref(double *lat_0, double *lon_0);

/**
 * Initialize the global mapping between global position (spherical) and local position (NED).
 */
__EXPORT int globallocalconverter_init(double lat_0, double lon_0, float alt_0, uint64_t timestamp);

/**
 * Checks if globallocalconverter was initialized
 * @return true if map was initialized before, false else
 */
__EXPORT bool globallocalconverter_initialized(void);

/**
 * Convert from global position coordinates to local position coordinates using the global reference
 */
__EXPORT int globallocalconverter_tolocal(double lat, double lon, float alt, float *x, float *y, float *z);

/**
 * Convert from local position coordinates to global position coordinates using the global reference
 */
__EXPORT int globallocalconverter_toglobal(float x, float y, float z,  double *lat, double *lon, float *alt);

/**
 * Get reference position of the global to local converter
 */
__EXPORT int globallocalconverter_getref(double *lat_0, double *lon_0, float *alt_0);

/**
 * Returns the distance to the next waypoint in meters.
 *
 * @param lat_now current position in degrees (47.1234567°, not 471234567°)
 * @param lon_now current position in degrees (8.1234567°, not 81234567°)
 * @param lat_next next waypoint position in degrees (47.1234567°, not 471234567°)
 * @param lon_next next waypoint position in degrees (8.1234567°, not 81234567°)
 */
__EXPORT float get_distance_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next);

/**
 * Returns the bearing to the next waypoint in radians.
 *
 * @param lat_now current position in degrees (47.1234567°, not 471234567°)
 * @param lon_now current position in degrees (8.1234567°, not 81234567°)
 * @param lat_next next waypoint position in degrees (47.1234567°, not 471234567°)
 * @param lon_next next waypoint position in degrees (8.1234567°, not 81234567°)
 */
__EXPORT float get_bearing_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next);

__EXPORT void get_vector_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next, float *v_n, float *v_e);

__EXPORT void get_vector_to_next_waypoint_fast(double lat_now, double lon_now, double lat_next, double lon_next, float *v_n, float *v_e);

__EXPORT void add_vector_to_global_position(double lat_now, double lon_now, float v_n, float v_e, double *lat_res, double *lon_res);

__EXPORT int get_distance_to_line(struct crosstrack_error_s *crosstrack_error, double lat_now, double lon_now, double lat_start, double lon_start, double lat_end, double lon_end);

__EXPORT int get_distance_to_arc(struct crosstrack_error_s *crosstrack_error, double lat_now, double lon_now, double lat_center, double lon_center,
				 float radius, float arc_start_bearing, float arc_sweep);

/*
 * Calculate distance in global frame
 */
__EXPORT float get_distance_to_point_global_wgs84(double lat_now, double lon_now, float alt_now,
		double lat_next, double lon_next, float alt_next,
		float *dist_xy, float *dist_z);

/*
 * Calculate distance in local frame (NED)
 */
__EXPORT float mavlink_wpm_distance_to_point_local(float x_now, float y_now, float z_now,
		float x_next, float y_next, float z_next,
		float *dist_xy, float *dist_z);

__EXPORT float _wrap_180(float bearing);
__EXPORT float _wrap_360(float bearing);
__EXPORT float _wrap_pi(float bearing);
__EXPORT float _wrap_2pi(float bearing);

__END_DECLS
