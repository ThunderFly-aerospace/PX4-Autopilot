/****************************************************************************
 *
 *   Copyright (c) 2021 ThunderFly s.r.o. All rights reserved.
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @author Roman Bapst <roman@px4.io>
 * @author Andreas Antener <andreas@uaventure.com>
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "AutogyroTakeoff.h"
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>

#include <uORB/Publication.hpp>

using matrix::Vector2f;
using namespace time_literals;

namespace autogyrotakeoff
{

AutogyroTakeoff::AutogyroTakeoff(ModuleParams *parent) :
	ModuleParams(parent),
	_state(),
	_initialized(false),
	_initialized_time(0),
	_init_yaw(0),
	_climbout(false)
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
	_start_wp(0) = current_lat;
	_start_wp(1) = current_lon;
}

void AutogyroTakeoff::update(const hrt_abstime &now, float airspeed, float rotor_rpm, float alt_agl,
			     double current_lat, double current_lon, orb_advert_t *mavlink_log_pub)
{
	_climbout = true;
	autogyro_takeoff_status_s autogyro_takeoff_status = {};

	camera_capture_s takeoff_information{};
	takeoff_information.timestamp = now;
	takeoff_information.result = 0;

	autogyro_takeoff_status.rpm = false;

	switch (_state) {
	/*
	    Hangling error states of takeoff mode. Should lead in allerting operator and/or
	    abrod takeoff process

	    IN: error state
	*/
	case AutogyroTakeoffState::TAKEOFF_ERROR:
		PX4_INFO("ERR STATE");
		mavlink_log_info(mavlink_log_pub, "#Takeoff: Error state");
		break;


	/*
	    Initial state of regulator, wait for manual prerotate of rotor.

	    IN: initial, reset
	    OUT: Minimal rotor RPM
	*/
	case AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE_START:

		if (rotor_rpm > _param_ag_prerotator_minimal_rpm.get()) {
			autogyro_takeoff_status.rpm = true;

			if (_param_ag_prerotator_type.get() == 2) { // Eletronic prerotator controlled from autopilot
				if (doPrerotate()) {
					play_next_tone();
					_state = AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE;
					_time_in_state = now;
				}

			} else { // manual prerotator or manually controlled
				play_next_tone();
				_state = AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE;
				_time_in_state = now;
			}

			mavlink_log_info(mavlink_log_pub, "#Takeoff: minimal RPM for prerotator reached");
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

		//TODO: Zde se rozhodovat podle typu prerotatoru. Pokud je ovladany z autopilota, zjisti jeho stav ze zpravy
		if (rotor_rpm > _param_ag_prerotator_target_rpm.get()) {
			autogyro_takeoff_status.rpm = true;
			_state = AutogyroTakeoffState::PRE_TAKEOFF_DONE;
			_time_in_state = now;
			play_next_tone();
			mavlink_log_info(mavlink_log_pub, "Takeoff, prerotator RPM reached");
		}

		break;

	/*
	    All required takeoff conditions are satisfied. Now it is prepared to start.
	    Try to start main motor. Slowly rump up and check it.

	    IN: rotor prepared;
	    OUT: rotor prepared; minimal airspeed; motor max power,
	*/
	case AutogyroTakeoffState::PRE_TAKEOFF_DONE: {     // 2
			bool ready_for_release = true;

			// check minimal rotor RPM again
			//if (rotor_rpm < _param_ag_prerotator_target_rpm.get() || rotor_rpm < _param_ag_rotor_flight_rpm.get()) {
			if (rotor_rpm < _param_ag_rotor_flight_rpm.get()) {
				autogyro_takeoff_status.rpm = true;
				ready_for_release = false;
				// Zde je potreba aplikovat nejakou hysterezi
				//_state = AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE;

				//mavlink_log_info(mavlink_log_pub, "RPM decreased., %f z %f", (double) rotor_rpm,
				//		 (double) _param_ag_rotor_flight_rpm.get());
				_time_in_state = now;
			}

			// check minimal airspeed
			if (airspeed < (_param_fw_airspd_min.get() * _param_rwto_airspd_scl.get())) {
				ready_for_release = false;
			}

			if (ready_for_release) {
				_state = AutogyroTakeoffState::TAKEOFF_RELEASE;
				_time_in_state = now;
				// TODO: pripravit funkci do release
				mavlink_log_info(mavlink_log_pub, "Takeoff, Please release.");
				takeoff_information.result = 1;

				play_next_tone();

				//doRelease();
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
			// Wait for ACK from platform
			//PX4_INFO("RELEASE, agl: %f", (double) alt_agl);
			play_release_tone();

			// Waiting to get full throttle
			if (hrt_elapsed_time(&_time_in_state) < (_param_rwto_ramp_time.get() * 1_s)) {
				// Send release CMD
				takeoff_information.result = 1;
			}

			autogyro_takeoff_status.rpm = true;

			if (alt_agl > _param_rwto_nav_alt.get()) {
				mavlink_log_info(mavlink_log_pub, "Climbout");
				_state = AutogyroTakeoffState::TAKEOFF_CLIMBOUT;
				play_next_tone();
				_time_in_state = now;

				if (_param_rwto_hdg.get() == 0) {
					_start_wp(0) = current_lat;
					_start_wp(1) = current_lon;
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
		autogyro_takeoff_status.rpm = true;

		if (alt_agl > _param_fw_clmbout_diff.get()) {
			_climbout = false;
			_state = AutogyroTakeoffState::FLY;
			_time_in_state = now;
		}

		//_climbout = false;
		break;

	case AutogyroTakeoffState::FLY:
		_climbout = false;
		break;

	default:
		break;
	}



	autogyro_takeoff_status.time_in_state = hrt_elapsed_time(&_time_in_state);
	autogyro_takeoff_status.state = (int) _state;
	autogyro_takeoff_status.climbout = _climbout;
	_autogyro_takeoff_status_pub.publish(autogyro_takeoff_status);

	if (hrt_elapsed_time(&_last_sent_release_status) > 1_s / 3 || _state != _state_last) {
		_last_sent_release_status = now;
		// takeoff_information.timestamp // fillet at beginning of loop
		takeoff_information.timestamp_utc = hrt_elapsed_time(&_time_in_state);
		takeoff_information.seq = _state;
		takeoff_information.alt = alt_agl;
		takeoff_information.lat = rotor_rpm;
		takeoff_information.lon = airspeed;
		takeoff_information.ground_distance = _climbout;

		takeoff_information.q[0] = alt_agl;
		takeoff_information.q[1] = rotor_rpm;
		takeoff_information.q[2] = airspeed;
		takeoff_information.q[3] = _state;

		_takeoff_informations_pub.publish(takeoff_information);
	}

	_state_last = _state;
}

/*
 * Send command for release from hand or from some platform
 */
bool AutogyroTakeoff::doRelease()
{
	// mavlink_log_info(mavlink_log_pub, "#Takeoff: Command for release");

	return true;
}


/*
 * Send command for start prerotation or for obtain formard movement
 */
bool AutogyroTakeoff::doPrerotate()
{
	//mavlink_log_info(mavlink_log_pub, "#Takeoff: Command for prerotation");

	return true;
}

float AutogyroTakeoff::getRequestedAirspeed()
{
	switch (_state) {
	case AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE_START:
	case AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE:
	case AutogyroTakeoffState::PRE_TAKEOFF_DONE:
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
	case AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE_START: // 0 Null pitch
		return 0;

	case AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE:   // 1 maximal pitch, //TODO: PRE_TAKEOFF_PREROTATE SLEW with pitch
	case AutogyroTakeoffState::PRE_TAKEOFF_DONE: // 2
		return math::radians(_param_rwto_max_pitch.get());

	//case AutogyroTakeoffState::TAKEOFF_RELEASE:             // 3 TECS limited
	//case AutogyroTakeoffState::TAKEOFF_CLIMBOUT:             // 4 TECS limited
	//case AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE_START:
	//case AutogyroTakeoffState::PRE_TAKEOFF_DONE:
	//return math::constrain(tecsPitch, (float) math::radians(0), (float) math::radians(15));
	//    return math::constrain(tecsPitch, (float) 0.2, (float) 5);

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
				       math::radians(-_param_rwto_max_roll.get()),
				       math::radians(_param_rwto_max_roll.get()));
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
	switch (_state) {

	case AutogyroTakeoffState::TAKEOFF_ERROR:
	case AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE_START:
		return 0;

	case AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE:
	case AutogyroTakeoffState::PRE_TAKEOFF_DONE: {
			if (_param_ag_prerotator_type.get() == AutogyroTakeoffType::WOPREROT_RUNWAY) {
				float throttlea = ((now - _time_in_state) / (_param_rwto_ramp_time.get() * 1_s)) * _param_rwto_max_thr.get();
				return math::min(throttlea, _param_rwto_max_thr.get());

			} else {
				return 0;
			}
		}


	case AutogyroTakeoffState::TAKEOFF_RELEASE: {
			float throttle = 0;

			if (_param_ag_prerotator_type.get() == AutogyroTakeoffType::WOPREROT_RUNWAY) {
				throttle = _param_rwto_max_thr.get();

			} else if (_param_ag_prerotator_type.get() == AutogyroTakeoffType::WOPREROT_PLATFORM) {
				throttle = ((now - _time_in_state) / (_param_rwto_ramp_time.get() * 1_s)) * _param_rwto_max_thr.get();
				throttle = math::min(throttle, _param_rwto_max_thr.get());
			}

			return math::min(throttle, _param_rwto_max_thr.get());
		}


	//case AutogyroTakeoffState::PRE_TAKEOFF_DONE:
	//case AutogyroTakeoffState::TAKEOFF_CLIMBOUT:
	//case AutogyroTakeoffState::PRE_TAKEOFF_PREROTATE:
	//	return _param_rwto_max_thr.get();

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

}
