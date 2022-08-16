/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file AutogyroLanding.cpp
 * Autogyro automated landing, header files
 *
 * @author Roman Dvorak, ThunderFly s.r.o. <dvorakroman@thunderfly.cz>
 */



#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "AutogyroLanding.h"
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>

#include <uORB/Publication.hpp>



using matrix::Vector2f;
using namespace time_literals;

namespace autogyrolanding
{

AutogyroLanding::AutogyroLanding(ModuleParams *parent) :
	ModuleParams(parent)
{
}

void AutogyroLanding::init(const hrt_abstime &now, float yaw, double current_lat, double current_lon)
{
	_initialized = true;
	_state = AutogyroLandingState::LANDING_INITIAL_POSITION;
	_initialized_time = now;
	_time_in_state = now;

	_initial_wp(0) = current_lat;
	_initial_wp(1) = current_lon;

	double lat_target = 0;
	double lon_target = 0;

	// Heading obtain from wind direction and allowed directions to landing
	// distance obtain from parameters
	waypoint_from_heading_and_distance(current_lat, current_lon, 0, 300, &lat_target, &lon_target);
	_approach_wp(0) = lat_target;
	_approach_wp(1) = lon_target;

	PX4_INFO("CILOVE SOURADNICE: %f %f", (double) current_lat, (double) current_lon);
	PX4_INFO("APPROACH SOURADNICE: %f %f", (double) lat_target, (double) lon_target);

	PX4_INFO("INIT AG Landing, fly 200 m on south");

}

void AutogyroLanding::update(const hrt_abstime &now, float airspeed, float rotor_rpm, float alt_agl,
			     double current_lat, double current_lon, orb_advert_t *mavlink_log_pub, ECL_L1_Pos_Controller *l1_control)
{
	float distance = 0;
	float distance_z = 0;
	get_distance_to_point_global_wgs84(current_lat, current_lon, alt_agl, _initial_wp(0), _initial_wp(1), alt_agl,
					   &distance, &distance_z);


	switch (_state) {
	case AutogyroLandingState::LANDING_INITIAL_POSITION: {
			if (_state != _state_last) {
				//_l1_control.navigateLoiter(prev_wp, curr_wp, curr_pos, ground_speed);
			}

			PX4_INFO("Distance %f m", (double) distance);

			if (distance > 150) {
				//_l1_control.navigate_waypoints(_initial_wp(0), _initial_wp(1), curr_pos, ground_speed);
				_state = AutogyroLandingState::LANDING_APPROACH;
			}

			break;
		}

	case AutogyroLandingState::LANDING_APPROACH: {
			// if(_state != _state_last){
			// 	l1_control.navigate_waypoints(_autogyro_landing._approach_wp, _autogyro_landing._initial_wp, curr_pos, ground_speed);
			// }
			PX4_INFO("Approach");

			if (distance < 75) {
				_state = AutogyroLandingState::LANDING_DESCEND;
			}

		}
		break;

	case AutogyroLandingState::LANDING_DESCEND:
		if (alt_agl < 22) {
			PX4_INFO("Prepinam do break");
			_state = AutogyroLandingState::LANDING_BREAK;
		}

		break;

	case AutogyroLandingState::LANDING_BREAK:

		break;


	case AutogyroLandingState::LANDING_TOUCH:

		break;

	case AutogyroLandingState::LANDING_DONE:

		break;

	default:
		break;
	}

	_state_last = _state;
}


float AutogyroLanding::getPitch(float tecsPitch)
{

	return tecsPitch;
}

float AutogyroLanding::getRoll(float navigatorRoll)
{

	return navigatorRoll;

}

float AutogyroLanding::getYaw(float navigatorYaw)
{

	return navigatorYaw;

}

float AutogyroLanding::getThrottle(float tecsThrottle)
{

	return tecsThrottle;
}


}
