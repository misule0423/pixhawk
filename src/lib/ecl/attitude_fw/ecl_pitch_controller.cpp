/****************************************************************************
 *
 *   Copyright (c) 2013 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file ecl_pitch_controller.cpp
 * Implementation of a simple orthogonal pitch PID controller.
 * 一个简单的垂直间距PID控制器的实现
 * Authors and acknowledgements in header.
 */

#include "ecl_pitch_controller.h"
#include <math.h>
#include <stdint.h>
#include <float.h>
#include <geo/geo.h>
#include <ecl/ecl.h>
#include <mathlib/mathlib.h>
#include <systemlib/err.h>

// 这还是使用了 c ++ 语言
ECL_PitchController::ECL_PitchController() :
	ECL_Controller("pitch"),
	_max_rate_neg(0.0f),
	_roll_ff(0.0f)
{
}
// 析构函数
ECL_PitchController::~ECL_PitchController()
{
}

float ECL_PitchController::control_attitude(const struct ECL_ControlData &ctl_data)
{

	/* Do not calculate control signal with bad inputs */
	// 不计算与坏的输入控制信号
	if (!(isfinite(ctl_data.pitch_setpoint) &&
	      isfinite(ctl_data.roll) &&
	      isfinite(ctl_data.pitch) &&
	      isfinite(ctl_data.airspeed))) {
	      	// 如果这些值不合理，那么就不进行控制
		perf_count(_nonfinite_input_perf);
		warnx("not controlling pitch");
		return _rate_setpoint;
	}

	/* Calculate the error */
	// 校准偏差量
	float pitch_error = ctl_data.pitch_setpoint - ctl_data.pitch;

	/*  Apply P controller: rate setpoint from current error and time CONSTANTS_ONE_G */
	// 应用 P 值控制器
	_rate_setpoint =  pitch_error / _tc;

	/* limit the rate */
	if (_max_rate > 0.01f && _max_rate_neg > 0.01f) {
		if (_rate_setpoint > 0.0f) {
			_rate_setpoint = (_rate_setpoint > _max_rate) ? _max_rate : _rate_setpoint;

		} else {
			_rate_setpoint = (_rate_setpoint < -_max_rate_neg) ? -_max_rate_neg : _rate_setpoint;
		}

	}

	return _rate_setpoint;
}

float ECL_PitchController::control_bodyrate(const struct ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(isfinite(ctl_data.roll) &&
	      isfinite(ctl_data.pitch) &&
	      isfinite(ctl_data.pitch_rate) &&
	      isfinite(ctl_data.yaw_rate) &&
	      isfinite(ctl_data.yaw_rate_setpoint) &&
	      isfinite(ctl_data.airspeed_min) &&
	      isfinite(ctl_data.airspeed_max) &&
	      isfinite(ctl_data.scaler))) {
		perf_count(_nonfinite_input_perf);
		return math::constrain(_last_output, -1.0f, 1.0f);
	}

	/* get the usual dt estimate */
	// 得到 DT 的估计
	uint64_t dt_micros = ecl_elapsed_time(&_last_run);
	_last_run = ecl_absolute_time();
	float dt = (float)dt_micros * 1e-6f;

	/* lock integral for long intervals */
	// 积分锁定长的时间间隔
	bool lock_integrator = ctl_data.lock_integrator;

	if (dt_micros > 500000) {
		lock_integrator = true;
	}

	/* Transform setpoint to body angular rates (jacobian) */
	// 变换设定身体的角速率
	_bodyrate_setpoint = cosf(ctl_data.roll) * _rate_setpoint +
			     cosf(ctl_data.pitch) * sinf(ctl_data.roll) * ctl_data.yaw_rate_setpoint;

	/* apply turning offset to desired bodyrate setpoint*/
	/* flying inverted (wings upside down)*/
	//  飞倒（翅膀倒挂）
	bool inverted = false;
	float constrained_roll;
	/* roll is used as feedforward term and inverted flight needs to be considered */
	if (fabsf(ctl_data.roll) < math::radians(90.0f)) {
		/* not inverted, but numerically still potentially close to infinity */
		constrained_roll = math::constrain(ctl_data.roll, math::radians(-80.0f), math::radians(80.0f));

	} else {
		/* inverted flight, constrain on the two extremes of -pi..+pi to avoid infinity */
		inverted = true;
		/* note: the ranges are extended by 10 deg here to avoid numeric resolution effects */
		if (ctl_data.roll > 0.0f) {
			/* right hemisphere */
			constrained_roll = math::constrain(ctl_data.roll, math::radians(100.0f), math::radians(180.0f));

		} else {
			/* left hemisphere */
			constrained_roll = math::constrain(ctl_data.roll, math::radians(-100.0f), math::radians(-180.0f));
		}
	}

	/* input conditioning */
	float airspeed = constrain_airspeed(ctl_data.airspeed, ctl_data.airspeed_min, ctl_data.airspeed_max);

	/* Calculate desired body fixed y-axis angular rate needed to compensate for roll angle.
	   For reference see Automatic Control of Aircraft and Missiles by John H. Blakelock, pg. 175
	   Availible on google books 8/11/2015: 
	   https://books.google.com/books?id=ubcczZUDCsMC&pg=PA175#v=onepage&q&f=false*/
	float body_fixed_turn_offset = (fabsf((CONSTANTS_ONE_G / airspeed) *
				  		tanf(constrained_roll) * sinf(constrained_roll)));

	if (inverted) {
		body_fixed_turn_offset = -body_fixed_turn_offset;
	}

	/* Finally add the turn offset to your bodyrate setpoint*/
	_bodyrate_setpoint += body_fixed_turn_offset;


	_rate_error = _bodyrate_setpoint - ctl_data.pitch_rate;

	if (!lock_integrator && _k_i > 0.0f) {

		float id = _rate_error * dt * ctl_data.scaler;

		/*
		 * anti-windup: do not allow integrator to increase if actuator is at limit
		 */
		if (_last_output < -1.0f) {
			/* only allow motion to center: increase value */
			id = math::max(id, 0.0f);

		} else if (_last_output > 1.0f) {
			/* only allow motion to center: decrease value */
			id = math::min(id, 0.0f);
		}

		_integrator += id;
	}

	/* integrator limit */
	//xxx: until start detection is available: integral part in control signal is limited here
	float integrator_constrained = math::constrain(_integrator * _k_i, -_integrator_max, _integrator_max);

	/* Apply PI rate controller and store non-limited output */
	_last_output = _bodyrate_setpoint * _k_ff * ctl_data.scaler +
		       _rate_error * _k_p * ctl_data.scaler * ctl_data.scaler
		       + integrator_constrained;  //scaler is proportional to 1/airspeed
//	warnx("pitch: _integrator: %.4f, _integrator_max: %.4f, airspeed %.4f, _k_i %.4f, _k_p: %.4f", (double)_integrator, (double)_integrator_max, (double)airspeed, (double)_k_i, (double)_k_p);
//	warnx("roll: _last_output %.4f", (double)_last_output);
	return math::constrain(_last_output, -1.0f, 1.0f);
}
