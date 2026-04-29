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

#include "LoraGpsVCmd.hpp"

using namespace time_literals;

LoraGpsVCmd::LoraGpsVCmd() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{

  incoming.timestamp=0;
  gps.timestamp=0;
  status.timestamp=0;
  cmd.timestamp=0;

  next_hi=_low_count.get();

  lastIncomingTimestamp=0;
  lastOutgoingTimestamp=0;
}

bool
LoraGpsVCmd::init()
{
	if (!_incoming_sub.registerCallback()) {
		PX4_ERR("incomming callback registration failed");
		return false;
	}

	_outgoing_pub.advertise();
  _vcmd_pub.advertise();

  ScheduleDelayed(1000*1000); //1s

	return true;
}

void
LoraGpsVCmd::Run()
{
	if (should_exit()) {
		_incoming_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

  //live param update from doc
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();
	}

  _incoming_sub.copy(&incoming);
  _status_sub.copy(&status);
  _gps_sub.copy(&gps);

  if(incoming.timestamp > lastIncomingTimestamp)
  {
    //resolve incomming message
    PX4_INFO("Incoming Lora message processing..");
    lastIncomingTimestamp=incoming.timestamp;
  }

  uint64_t time=hrt_absolute_time();
  if(time-lastCheck>=900000llu) //check 0.9 sec, dont miss 1s call
  {
    if(time-lastOutgoingTimestamp>=_beacon_interval_S.get()*1000llu*1000llu)
    {
      //publish message for send
      lora_message_s outgoing;
      outgoing.timestamp = time;
      outgoing.sf = _low_sf.get();
      if(next_hi>=0)
      {
        if(next_hi==0)
        {
          PX4_INFO("Setting Hi SF %ld",_high_sf.get());
          outgoing.sf = _high_sf.get();
          next_hi=_low_count.get();
        }
        else
        {
          PX4_INFO("Setting Lo SF %ld",_low_sf.get());
          next_hi--;
        }
      }
      outgoing.len = fill_lora_outgoing_msg(&gps, outgoing.data, sizeof(outgoing.data));
      _outgoing_pub.publish(outgoing);
      lastOutgoingTimestamp=time;    
      PX4_INFO("Next beacon in: %ld s", _beacon_interval_S.get());
    }

    lastCheck=time;
    ScheduleDelayed(1000*1000);
  }
}

uint32_t LoraGpsVCmd::pack_latlon(double v)
{
    return (int32_t) (v*4194304.0);
}

int LoraGpsVCmd::fill_lora_outgoing_msg(struct sensor_gps_s *in, uint8_t *out, int nmaxbytes) {
    uint8_t flags;
    uint8_t *p = out + 1;

    if (nmaxbytes < 16)
        return -1;
    flags = 0;
    uint32_t lat = pack_latlon(in->latitude_deg);
    *p++ = lat >> 24;
    *p++ = lat >> 16;
    *p++ = lat >> 8;
    *p++ = lat;
    uint32_t lon = pack_latlon(in->longitude_deg);
    *p++ = lon >> 24;
    *p++ = lon >> 16;
    *p++ = lon >> 8;
    *p++ = lon;
    if (in->fix_type >= 2)
        flags |= LATLON_OK;
    int32_t age_s = (hrt_absolute_time() - in->timestamp) / 1000000;
    if (age_s < 0)      age_s = 0;
    if (age_s > 0xffff) age_s = 0xffff;
    *p++ = age_s >> 8;
    *p++ = age_s;
    int32_t alt_m = in->altitude_msl_m;
    if (alt_m < -0x7ffe) alt_m = -0x7ffe;
    if (alt_m >  0x7ffe) alt_m =  0x7ffe;
    *p++ = alt_m >> 8;
    *p++ = alt_m;
    if (in->fix_type >= 3)
        flags |= ALT_OK;
    int16_t course = in->cog_rad*180*64/M_PI_F;
    *p++ = course >> 8;
    *p++ = course;
    if (!std::isnan(in->cog_rad))
        flags |= COURSE_OK;
    int16_t speed = in->vel_m_s*16;
    *p++ = speed >> 8;
    *p++ = speed;
    if (!std::isnan(in->vel_m_s))
        flags |= COURSE_OK;
    *out = flags;

    return p - out;
}

int
LoraGpsVCmd::task_spawn(int argc, char *argv[])
{
	LoraGpsVCmd *instance = new LoraGpsVCmd();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int
LoraGpsVCmd::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
LoraGpsVCmd::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

LoraGpsVCmd module consumes GPS and compress data for Lora message. Potentionaly Recieved Message from lora and publish VCMD command.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("lora_gps_vcmd", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


extern "C" __EXPORT int lora_gps_vcmd_main(int argc, char *argv[])
{
	return LoraGpsVCmd::main(argc, argv);
}
