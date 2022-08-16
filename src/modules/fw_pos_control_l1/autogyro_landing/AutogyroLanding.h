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
 * @file AutogyroLanding.h
 * Autogyro automated landing, header files
 *
 * @author Roman Dvorak, ThunderFly s.r.o. <dvorakroman@thunderfly.cz>
 */


#ifndef AUTOGYROLANDING_H
#define AUTOGYROLANDING_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <lib/l1/ECL_L1_Pos_Controller.hpp>

#include <uORB/Publication.hpp>

namespace autogyrolanding
{

enum AutogyroLandingState {
	LANDING_ERROR = -1,
	LANDING_INITIAL_POSITION = 0,
	LANDING_APPROACH,
	LANDING_DESCEND,
	LANDING_BREAK,
	LANDING_TOUCH,
	LANDING_DONE
};

class __EXPORT AutogyroLanding : public ModuleParams
{
public:
	AutogyroLanding(ModuleParams *parent);
	~AutogyroLanding() = default;

	void init(const hrt_abstime &now, float yaw, double current_lat, double current_lon);
	void update(const hrt_abstime &now, float airspeed, float rotor_rpm, float alt_agl, double current_lat,
		    double current_lon,
		    orb_advert_t *mavlink_log_pub,
		    ECL_L1_Pos_Controller *l1_control);

	AutogyroLandingState getState() { return _state; }
//		float getMinAirspeedScaling() { return _param_rwto_airspd_scl.get(); }
	bool isInitialized() { return _initialized; }

	//	bool AutogyroLandingEnabled() { return _param_ag_tkoff.get(); }
	bool autogyroLandingEnabled() { return _param_fw_lnd_type.get() == 2; }
	//float getRequestedAirspeed();
	//float getInitYaw() { return _init_yaw; }

	//bool controlYaw();
	//bool climbout() { return _climbout; }
	float getPitch(float tecsPitch);
	float getRoll(float navigatorRoll);
	float getYaw(float navigatorYaw);
	float getThrottle(float tecsThrottle);
	//bool resetIntegrators();
	//bool resetAltTakeoff();
	//float getMinPitch(float climbout_min, float min);
	//float getMaxPitch(float max);
	// bool setState(int new_state);
	//const matrix::Vector2d &getStartWP() const { return _takeoff_wp; };
	float getTargetBearing() {return 180.0;}
	void reset();

	matrix::Vector2d _initial_wp;
	matrix::Vector2d _approach_wp;

private:
	/** state variables **/
	AutogyroLandingState _state{LANDING_INITIAL_POSITION};
	AutogyroLandingState _state_last{LANDING_ERROR};


	bool _initialized{false};
	hrt_abstime _initialized_time{0};
	hrt_abstime _time_in_state{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::FW_LND_TYPE>) _param_fw_lnd_type

	)
};

}
#endif // AUTOGYROLANDIG_H
