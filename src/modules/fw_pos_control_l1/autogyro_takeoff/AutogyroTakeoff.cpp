/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
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
 * @file AutogyroTakeoff.cpp
 *
 *
 * @author Roman Dvorak, ThunderFly s.r.o. <dvorakroman@thunderfly.cz>
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "AutogyroTakeoff.h"
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>

#include <uORB/Publication.hpp>

using matrix::Vector2f;
using namespace time_literals;

namespace autogyrotakeoff
{

AutogyroTakeoff::AutogyroTakeoff(ModuleParams *parent) :
	ModuleParams(parent)
{
}

void AutogyroTakeoff::init(const hrt_abstime &now, float yaw, double current_lat, double current_lon)
{
	_init_yaw = yaw;
	_initialized = true;
	_state = AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE_START;
	_initialized_time = now;
	_time_in_state = now;
	_last_sent_release_status = now;
	_climbout = true; // this is true until climbout is finished
	_takeoff_wp(0) = current_lat;
	_takeoff_wp(1) = current_lon;
	_initial_wp(0) = current_lat;
	_initial_wp(1) = current_lon;
}

void AutogyroTakeoff::update(const hrt_abstime &now, float airspeed, float rotor_rpm, float alt_agl,
			     double current_lat, double current_lon, orb_advert_t *mavlink_log_pub)
{
	_climbout = true;
	takeoff_status_s takeoff_status = {};

	actuator_armed_s actuator_armed;
	_actuator_armed_sub.update(&actuator_armed);

	if (actuator_armed.manual_lockdown && _state <= AutogyroTakeoffState::PRE_TAKEOFF_RAMPUP) {
		_state = AutogyroTakeoffState::TAKEOFF_ERROR;
	}

	switch (_state) {
	/*
	    Hangling error states of takeoff mode. Should lead in allerting operator and/or
	    abrod takeoff process

	    IN: error state
	*/
	case AutogyroTakeoffState::TAKEOFF_ERROR: {
			if (_state != _state_last) {
				PX4_INFO("ERR STATE");
				mavlink_log_info(mavlink_log_pub, "#Takeoff: Error state");
			}
		}
		break;

	/*
	    Initial state of regulator, wait for manual prerotate of rotor.

	    IN: initial, reset
	    OUT: Minimal rotor RPM
	*/
	case AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE_START:

		if (rotor_rpm > _param_ag_prerotator_minimal_rpm.get()) {

			// Eletrical prerotator, controlled from autopilot
			if (_param_ag_prerotator_type.get() == AutogyroTakeoffType::ELPREROT_PLATFORM
			    || _param_ag_prerotator_type.get() == AutogyroTakeoffType::ELPREROT_RUNWAY) {
				if (doPrerotate()) {
					play_next_tone();
					_state = AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE;
					_time_in_state = now;
				}

			} else { // manually controlled prerotator or prerotation by forward movement
				play_next_tone();
				_state = AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE;
				_time_in_state = now;
			}

			mavlink_log_info(mavlink_log_pub, "Takeoff: minimal RPM for prerotator reached");
			//PX4_INFO("Takeoff: minimal RPM for prerotator reached");
		}

		break;


	/*
	    Reach minimal RPM of rotor. It can be ensured with several ways. For autogyro
	    with electronic prerotator, it can be controlled from autopilot or this is command
	    for get some groundspeed for airflow and for prerotation of rotor

	    IN: rotor with minimal RPM,
	    PROCESS: increase rotor RPM to flight RPM
	    OUT: rotor in flight state
	*/
	case AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE:  // 1

		if (rotor_rpm > _param_ag_prerotator_target_rpm.get()) {
			_state = AutogyroTakeoffState::PRE_TAKEOFF_DONE;
			_time_in_state = now;
			play_next_tone();
			mavlink_log_info(mavlink_log_pub, "Takeoff, prerotator RPM reached");
			//PX4_INFO("Takeoff: prerotator RPM reached");
		}

		break;

	/*
	    All required takeoff conditions are satisfied. Now it is prepared to
	    try to start main motor.

	    IN: rotor prepared;
	    OUT: rotor prepared; minimal airspeed; motor max power,
	*/
	case AutogyroTakeoffState::PRE_TAKEOFF_DONE: {     // 2
			bool ready_for_release = true;

			if (rotor_rpm < _param_ag_rotor_flight_rpm.get()) {
				ready_for_release = false;
				//PX4_INFO("Takeofff, waiting for flight rpm.");
				// Some histesis needs to be applied for the start interrupt procedure.
				// Currently, this does not allow the start to be interrupted.
				//_state = AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE;
				//_time_in_state = now;
			}

			// check minimal airspeed
			if (airspeed < (_param_fw_airspd_min.get() * _param_rwto_airspd_scl.get())) {
				ready_for_release = false;
				//PX4_INFO("Takeofff, waiting for min airspeed.");
			}

			if (ready_for_release) {
				_init_yaw = get_bearing_to_next_waypoint(_initial_wp(0), _initial_wp(1), current_lat, current_lon);

				_state = AutogyroTakeoffState::PRE_TAKEOFF_RAMPUP;
				_time_in_state = now;
				mavlink_log_info(mavlink_log_pub, "Ready to start motor");
				//PX4_INFO("Takeoff, Please release.");
				play_next_tone();
			}

		}
		break;

	/*
	    Slowly rampup motor. Keep trying to check other flight parameters.
	    If I can still fly
	*/
	case AutogyroTakeoffState::PRE_TAKEOFF_RAMPUP: {
			bool ready_for_release = true;

			if (rotor_rpm < _param_ag_rotor_flight_rpm.get()) {
				ready_for_release = false;
				//PX4_INFO("Takeofff, waiting for flight rpm.");
				// Some histesis needs to be applied for the start interrupt procedure.
				// Currently, this does not allow the start to be interrupted.
				//_state = AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE;
				//_time_in_state = now;
			}

			// check minimal airspeed
			if (airspeed < (_param_fw_airspd_min.get() * _param_rwto_airspd_scl.get())) {
				ready_for_release = false;
				//PX4_INFO("Takeofff, waiting for min airspeed.");
			}

			// ramup time elapsed
			if (hrt_elapsed_time(&_time_in_state) < (_param_rwto_ramp_time.get() * 1_s * 1.5f)) {
				ready_for_release = false;
			}

			// Check if motor/esc power (properller RPM) is suffiscient
			// TODO
			if (false) {
				ready_for_release = false;
			}

			if (ready_for_release) {

				_state = AutogyroTakeoffState::TAKEOFF_RELEASE;
				_time_in_state = now;
				PX4_INFO("Takeoff, Please release.");
				play_next_tone();
			}
		}
		break;

	/*
	    Command for release. Sound signal for release from hand or release from
	    some takeoff platform with mavlink command. This step ends on release ACK.
	    In the case of hand release it is done inmedietly. If it is not ACKed
	    it fall in error state


	    IN: autogyro is prepared for takeoff
	    OUT: Command for release
	*/
	case AutogyroTakeoffState::TAKEOFF_RELEASE: {
			// Waiting to get full throttle
			if (hrt_elapsed_time(&_time_in_state) < (_param_rwto_ramp_time.get() * 1_s)) {
				// Send release CMD
				play_release_tone();
			}

			if (alt_agl > _param_ag_nav_alt.get()) {
				mavlink_log_info(mavlink_log_pub, "Climbout");
				PX4_INFO("Takeoff: Climbout.");
				_state = AutogyroTakeoffState::TAKEOFF_CLIMBOUT;
				play_next_tone();
				_time_in_state = now;

				// set current position as center of loiter
				if (_param_rwto_hdg.get() == 0) {
					_takeoff_wp(0) = current_lat;
					_takeoff_wp(1) = current_lon;
				}
			}
		}
		break;

	/*
	    Reach minimal altitude and then fly!

	    IN: Released
	    OUT: Mission continue
	*/
	case AutogyroTakeoffState::TAKEOFF_CLIMBOUT:
		if (alt_agl > _param_fw_clmbout_diff.get()) {
			_climbout = false;
			_state = AutogyroTakeoffState::FLY;
			_time_in_state = now;
			PX4_INFO("Takeoff:FLY.");
		}

		//_climbout = false;
		break;

	case AutogyroTakeoffState::FLY:
		_climbout = false;
		break;

	default:
		break;
	}



	takeoff_status.time_in_state = hrt_elapsed_time(&_time_in_state);
	takeoff_status.takeoff_state = (int) _state;
	_takeoff_status_pub.publish(takeoff_status);

	if (hrt_elapsed_time(&_last_sent_release_status) > 1_s / 4 || _state != _state_last) {
		_last_sent_release_status = now;
		debug_value_s takeoff_information{};
		takeoff_information.timestamp = now;
		// templorary sollution, should be changed by custom mavlink message
		takeoff_information.value = _state;
		_takeoff_informations_pub.publish(takeoff_information);
	}

	_state_last = _state;
}

/*
 * Send command for release from hand or from some platform
 */
bool AutogyroTakeoff::doRelease()
{
	return true;
}


/*
 * Send command for start prerotation or for obtain formard movement
 */
bool AutogyroTakeoff::doPrerotate()
{
	return true;
}

float AutogyroTakeoff::getRequestedAirspeed()
{
	switch (_state) {
	case AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE_START:
	case AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE:
	case AutogyroTakeoffState::PRE_TAKEOFF_DONE:
	case AutogyroTakeoffState::PRE_TAKEOFF_RAMPUP:
		return _param_fw_airspd_min.get() * _param_rwto_airspd_scl.get();

	default:
		return _param_fw_airspd_trim.get();
	}
}

/*
 * Returns true as long as we're below navigation altitude
 */
bool AutogyroTakeoff::controlYaw()
{
	// keep controlling yaw directly until we start navigation
	return _state < AutogyroTakeoffState::TAKEOFF_CLIMBOUT;
}

/*
 * Returns pitch setpoint to use.
 *
 * Limited (parameter) as long as the plane is on runway. Otherwise
 * use the one from TECS
 */
float AutogyroTakeoff::getPitch(float tecsPitch)
{
	switch (_state) {

	case AutogyroTakeoffState::TAKEOFF_ERROR:
		return 0;

	case AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE_START: // 0 Null pitch
	case AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE:   // 1 maximal pitch
	case AutogyroTakeoffState::PRE_TAKEOFF_DONE: // 2
	case AutogyroTakeoffState::PRE_TAKEOFF_RAMPUP:
		//return math::radians(_param_rwto_max_pitch.get());
		return math::radians(_param_rwto_psp.get());

	// FLy
	default:
		return tecsPitch;
	}
}

/*
 * Returns the roll setpoint to use.
 */
float AutogyroTakeoff::getRoll(float navigatorRoll)
{
	// until we have enough ground clearance, set roll to 0
	if (_state < AutogyroTakeoffState::TAKEOFF_RELEASE) {
		return 0.0f;
	}

	// allow some limited roll during RELEASE and CLIMBOUT
	else if (_state < AutogyroTakeoffState::FLY) {
		return math::constrain(navigatorRoll,
				       math::radians(-_param_ag_tko_max_roll.get()),
				       math::radians(_param_ag_tko_max_roll.get()));
	}

	return navigatorRoll;
}

/*
 * Returns the yaw setpoint to use.
 *
 * In heading hold mode (_heading_mode == 0), it returns initial yaw as long as it's on the
 * runway. When it has enough ground clearance we start navigation towards WP.
 */
float AutogyroTakeoff::getYaw(float navigatorYaw)
{
	return navigatorYaw;

	if (_param_rwto_hdg.get() == 0 && _state < AutogyroTakeoffState::TAKEOFF_CLIMBOUT) {
		return _init_yaw;

	} else {
		return navigatorYaw;
	}
}

/*
 * Returns the throttle setpoint to use.
 *
 * Ramps up in the beginning, until it lifts off the runway it is set to
 * parameter value, then it returns the TECS throttle.
 */
float AutogyroTakeoff::getThrottle(const hrt_abstime &now, float tecsThrottle)
{

	float idle = (double)_param_fw_thr_idle.get();

	//PX4_INFO("GET THROTTLE %f, state: %f, time: %f", (double)idle, (double)_state, (double)(now - _time_in_state));

	switch (_state) {

	case AutogyroTakeoffState::TAKEOFF_ERROR:
	case AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE_START:
		return 0;

	case AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE:
	case AutogyroTakeoffState::PRE_TAKEOFF_DONE: {
			if (_param_ag_prerotator_type.get() == AutogyroTakeoffType::WOPREROT_RUNWAY) {
				float throttle = ((now - _time_in_state) / (_param_rwto_ramp_time.get() * 1_s)) * _param_rwto_max_thr.get();
				//PX4_INFO("Thortle: %f",(double)throttle);
				return math::min(throttle, _param_rwto_max_thr.get());

			} else {
				return idle;
			}
		}

	case AutogyroTakeoffState::PRE_TAKEOFF_RAMPUP: {
			float throttle = idle;

			if (_param_ag_prerotator_type.get() == AutogyroTakeoffType::WOPREROT_RUNWAY) {
				throttle = _param_rwto_max_thr.get();

			} else if (_param_ag_prerotator_type.get() == AutogyroTakeoffType::WOPREROT_PLATFORM) {
				throttle = ((now - _time_in_state) / (_param_rwto_ramp_time.get() * 1_s)) * _param_rwto_max_thr.get();
				throttle = math::min(throttle, _param_rwto_max_thr.get());
			}

			return throttle;
		}


	case AutogyroTakeoffState::TAKEOFF_RELEASE: {
			float throttle = idle;

			if (_param_ag_prerotator_type.get() == AutogyroTakeoffType::WOPREROT_RUNWAY) {
				throttle = _param_rwto_max_thr.get();

			} else if (_param_ag_prerotator_type.get() == AutogyroTakeoffType::WOPREROT_PLATFORM) {
				throttle = _param_rwto_max_thr.get();
			}

			return math::min(throttle, _param_rwto_max_thr.get());
		}

	// TAKEOFF_CLIMBOUT a FLY
	default:
		return tecsThrottle;
	}
}

bool AutogyroTakeoff::resetIntegrators()
{
	// reset integrators if we're still on runway
	return _state < AutogyroTakeoffState::TAKEOFF_RELEASE;
}


bool AutogyroTakeoff::resetAltTakeoff()
{
	return _state < AutogyroTakeoffState::TAKEOFF_RELEASE;
}

/*
 * Returns the minimum pitch for TECS to use.
 *
 * In climbout we either want what was set on the waypoint (sp_min) but at least
 * the climbtout minimum pitch (parameter).
 * Otherwise use the minimum that is enforced generally (parameter).
 */
float AutogyroTakeoff::getMinPitch(float climbout_min, float min)
{
	if (_state < AutogyroTakeoffState::FLY) {
		return climbout_min;
	}

	else {
		return min;
	}
}

/*
 * Returns the maximum pitch for TECS to use.
 *
 * Limited by parameter (if set) until climbout is done.
 */
float AutogyroTakeoff::getMaxPitch(float max)
{
	// use max pitch from parameter if set (> 0.1)
	if (_state < AutogyroTakeoffState::FLY && _param_rwto_max_pitch.get() > 0.1f) {
		return _param_rwto_max_pitch.get();
	}

	else {
		return max;
	}
}

void AutogyroTakeoff::reset()
{
	_initialized = false;
	_state = AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE_START;
}

void AutogyroTakeoff::play_next_tone()
{
	tune_control_s tune_control{};
	tune_control.tune_id = tune_control_s::TUNE_ID_TAKEOFF_NEXT_STEP;
	tune_control.volume = tune_control_s::VOLUME_LEVEL_MAX;
	tune_control.timestamp = hrt_absolute_time();
	_tune_control.publish(tune_control);
}

void AutogyroTakeoff::play_error_tone()
{
	tune_control_s tune_control{};
	tune_control.tune_id = tune_control_s::TUNE_ID_TAKEOFF_ERROR;
	tune_control.volume = tune_control_s::VOLUME_LEVEL_MAX;
	tune_control.timestamp = hrt_absolute_time();
	_tune_control.publish(tune_control);
}

void AutogyroTakeoff::play_release_tone()
{
	tune_control_s tune_control{};
	tune_control.tune_id = tune_control_s::TUNE_ID_TAKEOFF_RELEASE;
	tune_control.volume = tune_control_s::VOLUME_LEVEL_MAX;
	tune_control.timestamp = hrt_absolute_time();
	_tune_control.publish(tune_control);
}

} // end of namespace autogyrotakeoff
