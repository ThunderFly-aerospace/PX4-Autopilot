/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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
 *
 * LoRa message processing
 *  periodicly send encoded GPS position
 *  recieved lora message is translated into VehicleCommand message
 */

#pragma once

#include <lib/parameters/param.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/lora_message.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_status.h>

class LoraGpsVCmd : public ModuleBase<LoraGpsVCmd>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	LoraGpsVCmd();
	~LoraGpsVCmd() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:

	void Run() override;

	uORB::SubscriptionCallbackWorkItem _incoming_sub{this, ORB_ID(lora_incoming_message)};

	uORB::Subscription	_gps_sub{ORB_ID(sensor_gps)};
	uORB::Subscription	_status_sub{ORB_ID(vehicle_status)};

	uORB::Publication<lora_message_s>	_outgoing_pub{ORB_ID(lora_outgoing_message)};
	uORB::Publication<lora_message_s>	_vcmd_pub{ORB_ID(vehicle_command)};
  
  lora_message_s incoming;
  sensor_gps_s gps;
  vehicle_status_s status;
  vehicle_command_s cmd;

  uint64_t lastIncomingTimestamp;
  uint64_t lastOutgoingTimestamp;

  enum {
      LATLON_OK = (1<<0),
      ALT_OK    = (1<<1),
      COURSE_OK = (1<<2),
      SPEED_OK  = (1<<3)
  };

  int fill_lora_outgoing_msg(struct sensor_gps_s *in, uint8_t *out, int nmaxbytes);
  uint32_t pack_latlon(double v);

	/*param_t _p_cam_cap_fback;
	int32_t _cam_cap_fback{0};*/
  uint64_t beacon_interval_S;
};
