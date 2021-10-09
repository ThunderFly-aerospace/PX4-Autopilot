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
 * @file AutogyroTakeoff.h
 * pridat popis driver
 *
 * @author Roman Dvorak, ThunderFly s.r.o. <dvorakroman@thunderfly.cz>
 * @author Roman Bapst <roman@px4.io>
 * @author Andreas Antener <andreas@uaventure.com>
 */

#ifndef AUTOGYROTAKEOFF_H
#define AUTOGYROTAKEOFF_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/autogyro_takeoff_status.h>
#include <uORB/topics/tune_control.h>
#include <uORB/topics/camera_capture.h>


namespace autogyrotakeoff
{

// enum AutogyroTakeoffState {
// 	THROTTLE_RAMP = 0, /**< ramping up throttle */
// 	CLAMPED_TO_RUNWAY = 1, /**< clamped to runway, controlling yaw directly (wheel or rudder) */
// 	TAKEOFF = 2, /**< taking off, get ground clearance, roll 0 */
// 	CLIMBOUT = 3, /**< climbout to safe height before navigation, roll limited */
// 	FLY = 4 /**< fly towards takeoff waypoint */
// };


enum AutogyroTakeoffState {
	TAKEOFF_ERROR = -1,
	PRE_TAKEOFF_PREROTATE_START = 0, /**< Wait for manual rotor prerotation or for some other trigger */
	PRE_TAKEOFF_PREROTATE = 1, /**< Start prerotation of rotor controlled from AP or prerotation with some movement */
	PRE_TAKEOFF_DONE = 2, /**< autogyro conditions are OK for takeoff, rampup motor */
	TAKEOFF_RELEASE = 3, /**< command for release */
	TAKEOFF_CLIMBOUT = 4, /**< Climbout for minimal altitude */
	FLY /**< fly to next waypoint */
};

enum AutogyroTakeoffType {
	WOPREROT_PLATFORM = 0, // platform without prerotator
	WOPREROT_RUNWAY = 1,   // Without prerotator on runway
	ELPREROT_PLATFORM = 2, // Moving platform wint powered prerotator
	ELPREROT_RUNWAY = 3,   // From runway with powered prerotator
	FG_SITL = 10           // FlightGear SITL
};

class __EXPORT AutogyroTakeoff : public ModuleParams
{
public:
	AutogyroTakeoff(ModuleParams *parent);
	~AutogyroTakeoff() = default;

	void init(const hrt_abstime &now, float yaw, double current_lat, double current_lon);
	void update(const hrt_abstime &now, float airspeed, float rotor_rpm, float alt_agl, double current_lat,
		    double current_lon,
		    orb_advert_t *mavlink_log_pub);

	bool doRelease();
	bool doPrerotate();

	AutogyroTakeoffState getState() { return _state; }
	bool isInitialized() { return _initialized; }

	//	bool autogyroTakeoffEnabled() { return _param_ag_tkoff.get(); }
	bool autogyroTakeoffEnabled() { return _param_ag_tkoff.get(); }
	float getMinAirspeedScaling() { return _param_rwto_airspd_scl.get(); }
	float getInitYaw() { return _init_yaw; }

	bool controlYaw();
	bool climbout() { return _climbout; }
	float getPitch(float tecsPitch);
	float getRoll(float navigatorRoll);
	float getYaw(float navigatorYaw);
	float getThrottle(const hrt_abstime &now, float tecsThrottle);
	bool resetIntegrators();
	float getMinPitch(float climbout_min, float min);
	float getMaxPitch(float max);
	// bool setState(int new_state);
	const matrix::Vector2d &getStartWP() const { return _start_wp; };

	void reset();


	void play_next_tone();
	void play_release_tone();
	void play_error_tone();

private:
	/** state variables **/
	AutogyroTakeoffState _state{PRE_TAKEOFF_PREROTATE_START};
	AutogyroTakeoffState _state_last{PRE_TAKEOFF_PREROTATE_START};
	bool _initialized{false};
	hrt_abstime _initialized_time{0};
	hrt_abstime _time_in_state{0};
	hrt_abstime _last_sent_release_status{0};
	float _init_yaw{0.f};
	bool _climbout{false};
	matrix::Vector2d _start_wp;

	uORB::Publication<tune_control_s> _tune_control{ORB_ID(tune_control)};
	uORB::Publication<autogyro_takeoff_status_s> _autogyro_takeoff_status_pub{ORB_ID(autogyro_takeoff_status)};

	// TODO: templorary sollution. Should be replaced with custom message and custom mavlink message
	// Used to inform launch platform to release drone from lock
	uORB::Publication<camera_capture_s>	_takeoff_informations_pub{ORB_ID(camera_capture)};

	DEFINE_PARAMETERS(

		(ParamBool<px4::params::AG_TKOFF>) _param_ag_tkoff,
		(ParamBool<px4::params::RWTO_TKOFF>) _param_rwto_tkoff,
		(ParamInt<px4::params::RWTO_HDG>) _param_rwto_hdg,
		(ParamFloat<px4::params::RWTO_NAV_ALT>) _param_rwto_nav_alt,
		(ParamFloat<px4::params::RWTO_MAX_THR>) _param_rwto_max_thr,
		(ParamFloat<px4::params::RWTO_PSP>) _param_rwto_psp,
		(ParamFloat<px4::params::RWTO_MAX_PITCH>) _param_rwto_max_pitch,
		(ParamFloat<px4::params::RWTO_MAX_ROLL>) _param_rwto_max_roll,
		(ParamFloat<px4::params::RWTO_AIRSPD_SCL>) _param_rwto_airspd_scl,
		(ParamFloat<px4::params::RWTO_RAMP_TIME>) _param_rwto_ramp_time,

		(ParamFloat<px4::params::FW_CLMBOUT_DIFF>) _param_fw_clmbout_diff,
		(ParamFloat<px4::params::FW_AIRSPD_MAX>) _param_fw_airspd_max,
		(ParamFloat<px4::params::FW_AIRSPD_MIN>) _param_fw_airspd_min,
		(ParamFloat<px4::params::FW_AIRSPD_TRIM>) _param_fw_airspd_trim,

		(ParamFloat<px4::params::AG_PROT_MIN_RPM>) _param_ag_prerotator_minimal_rpm,
		(ParamFloat<px4::params::AG_PROT_TRG_RPM>) _param_ag_prerotator_target_rpm,
		(ParamFloat<px4::params::AG_ROTOR_RPM>) _param_ag_rotor_flight_rpm,
		(ParamInt<px4::params::AG_PROT_TYPE>) _param_ag_prerotator_type


	)
};

}

#endif // AUTOGYROTAKEOFF_H
