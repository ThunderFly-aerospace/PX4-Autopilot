/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file S35770.cpp
 *
 * @author Roman Dvorak <dvorakroman@thunderfly.cz>
 * @url https://github.com/ThunderFly-aerospace/TFRPMMAG01
 *
 * Driver for RPM sensor TFRPMMAG01. Rotary magnetic RPM sensor using S35770 I2C counter.
 */

#include "s35770.hpp"

S35770::S35770(const I2CSPIDriverConfig &config) :
	I2C(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config)
{
}

int S35770::init()
{
	if (I2C::init() != PX4_OK) {
		return PX4_ERROR;
	}

	PX4_DEBUG("addr: %" PRId8 ", pool: %" PRId32 ", reset: %" PRId32 ", magenet: %" PRId32, get_device_address(),
		  _param_S35770_pool.get(),
		  _param_S35770_magnet.get());

	ScheduleOnInterval(_param_S35770_pool.get());
	_rpm_pub.advertise();

	return PX4_OK;
}

int S35770::probe()
{
	uint32_t raw;
	return getCounter(&raw);
}


uint8_t S35770::getCounter(uint32_t* count)
{
	uint8_t raw[3];
	uint8_t err = transfer(nullptr, 0, &raw[0], 3);
	// *count = uint32_t(hiWord(raw[0]) << 16 | loWord(raw[1]) << 8 | raw[2]);
	*count = (uint32_t(raw[0]) << 16) | (uint32_t(raw[1]) << 8) | uint32_t(raw[2]);

	return err;
}


void S35770::RunImpl()
{
	// read sensor and compute frequency
	uint32_t oldcount = _count;
	int32_t diffTime = hrt_elapsed_time(&_last_measurement_time);

	uint8_t err = getCounter(&_count);

	if (err != PX4_OK) {
		PX4_ERR("Error reading counter");
		return;
	}

	_last_measurement_time = hrt_absolute_time();

	if (err != PX4_OK) {
		PX4_ERR("Error reading counter");
		return;
	}


	uint32_t diffCount = 0;

	if (oldcount > _count) {
		// Counter overflow occured
		PX4_INFO("Counter overflow");
		_overflow_count++;
		diffCount = (_count + (0xFFFFFF - oldcount) + 1);
		//return;

	} else {
		// Normal operation
		diffCount = _count - oldcount;
	}

	// Calculate RPM and accuracy estimation
	float indicated_rpm = (((float)diffCount / _param_S35770_magnet.get()) / ((float)diffTime / 1000000.f)) * 60.f;
	//float estimated_accurancy = 1 / (float)_param_S35770_magnet.get() / ((float)diffTime / 1000000) * 60.f;

	// publish data to uorb
	rpm_s msg{};
	msg.rpm_estimate = indicated_rpm;
	msg.rpm_raw = indicated_rpm;
	msg.timestamp = hrt_absolute_time();
	_rpm_pub.publish(msg);
}

void S35770::print_status()
{
	I2CSPIDriverBase::print_status();
	PX4_INFO("poll interval:  %" PRId32 " us", _param_S35770_pool.get());
	PX4_INFO("Number of owerflows: %d", _overflow_count);
	PX4_INFO("Last count %ld", _count);
}
