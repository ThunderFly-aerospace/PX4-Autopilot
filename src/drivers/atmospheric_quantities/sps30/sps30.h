/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file sps30.h
 *
 * Header file for SPS30 driver
 *
 * @author Roman Dvorak <dvorakroman@thunderfly.cz>
 *
 */

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

#include <mathlib/mathlib.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <drivers/device/i2c.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/airborne_particles.h>
#include <drivers/drv_hrt.h>

#define SPS30_START_MEASUREMENT     0x0010
#define SPS30_STOP_MEARUSEREMNT     0x0104
#define SPS30_READ_READY_FLAG       0x0202
#define SPS30_READ_VALUES           0x0300
#define SPS30_SLEEP                 0x1001
#define SPS30_WAKEUP                0x1103
#define SPS30_FAN_CLEANING          0x5607
#define SPS30_AUTO_CLEANING_INTERVAL   0x8040
#define SPS30_READ_PRODUCT_TYPE     0xD002
#define SPS30_READ_SERIAL_NUMBER    0xD033
#define SPS30_READ_VERSION          0xD100
#define SPS30_READ_DEVICE_STATUS    0xD206
#define SPS30_CLEAR_DEVICE_STATUS   0xD210
#define SPS30_RESET                 0xD304



using namespace time_literals;

extern "C" __EXPORT int sps30_main(int argc, char *argv[]);

enum sps30_state {
	ERROR_GENERAL,
	ERROR_READOUT,
	INIT,
	MEASUREMENT,
	CLEANINNG,
	CLEAN_START
};
const char *psp30_state_names[] = {"General error", "Readout error", "Initialization",
				   "Measurement", "Cleaning in progress", "Creaning started"
				  };


struct sps_info {
	char serial_number[32];
	uint8_t version[2];
};


class SPS30 : public device::I2C, public ModuleParams, public I2CSPIDriver<SPS30>
{
public:
	SPS30(const I2CSPIDriverConfig &config);
	~SPS30() = default;

	static void print_usage();

	void RunImpl();

	void start_clean();

	int    init() override;
	int    probe() override;
	int    init_sensor();
	void   print_status() override;

	void custom_method(const BusCLIArguments &cli);

	int set_pointer(uint16_t command);
	int read_data(uint16_t command, uint8_t *data_ptr, uint8_t length);
	int write_data(uint16_t command, uint8_t buffer[], uint8_t length);

	void sensor_compouse_msg(bool send);
	int sensor_has_new_measurement();

	uint8_t calc_crc(uint8_t data[2]);

private:

	float measured_values[10];
	uint32_t measurement_time = 0;
	uint16_t measurement_index = 0;

	sps_info _sps_info;
	int _state = sps30_state::INIT;
	int _last_state = sps30_state::INIT;
	uint32_t _time_in_state = hrt_absolute_time();
	uint16_t _last_command = 0;
	uORB::Publication<airborne_particles_s> _airborne_particles_pub{ORB_ID(airborne_particles)};

};
