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
 * @file sensirion_airborneparticles.h
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
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_airborne_particles.h>
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

#define SEN5X_START_MEASUREMENT			0x0021
#define SEN5X_START_MEASUREMENT_RHTG	0x0037
#define SEN5X_STOP_MEASUREMENT			0x0104
#define SEN5X_READ_READY_FLAG			0x0202
#define SEN5X_READ_VALUES				0x03C4
#define SEN5X_TEMPERATURE_COMPENSATION	0x60B2
#define SEN5X_WARM_START_PARAMETERS		0x60C6
#define SEN5X_VOC_TUNING_PARAMETERS		0x60D0
#define SEN5X_NOX_TUNING_PARAMETERS		0x60E1
#define SEN5X_RHT_TUNING_PARAMETERS		0x60F7
#define SEN5X_RHT_ACCELERATION_MODE		0x60F7
#define SEN5X_VOC_STATE					0x6181
#define SEN5X_FAN_CLEANING				0x5607
#define SEN5X_AUTO_CLEANING_INTERVAL	0x8004
#define SEN5X_READ_PRODUCT_NAME			0xD014
#define SEN5X_READ_SERIAL_NUMBER		0xD033
#define SEN5X_READ_FIRMWARE_VERSION		0xD100
#define SEN5X_READ_DEVICE_STATUS		0xD206
#define SEN5X_CLEAR_DEVICE_STATUS		0xD210
#define SEN5X_RESET                 	0xD304


using namespace time_literals;

extern "C" __EXPORT int sensirion_ap_main(int argc, char *argv[]);


enum sensirion_airborne_particle_state {
	ERROR_GENERAL,
	ERROR_READOUT,
	INIT,
	MEASUREMENT,
	CLEANINNG,
	CLEAN_START
};

static const char *_state_names[] = {"General error", "Readout error", "Initialization",
					  "Measurement", "Cleaning in progress", "Creaning started"
					 };

enum sensor_type {
	UNKNOWN = -1,
	NONE = 0,
	SPS30,
	SEN50,
	SEN54,
	SEN55,
};

static const char *_sensor_type_names[] = { "Unknown sensor", "Not initialized", "Senserion SPS30", "Senserion SEN50", "Sensirion SEN54", "Sensirion SEN55" };

struct sps_info {
	uint8_t sensor_type;
	char serial_number[32];
	uint8_t version[2];
};

class SENSIRION_AP : public device::I2C, public ModuleParams, public I2CSPIDriver<SENSIRION_AP>
{
public:
	SENSIRION_AP(const I2CSPIDriverConfig &config);
	~SENSIRION_AP() = default;


	static void print_usage();

	void RunImpl();

	void start_clean();

	int    init() override;
	int    probe() override;
	int    init_SPS30_sensor();
	int    init_SEN5X_sensor();
	void   print_status() override;

	void custom_method(const BusCLIArguments &cli);

	int set_pointer(uint16_t command);
	int read_data(uint16_t command, uint8_t *data_ptr, uint8_t length);
	int read_data(uint16_t command, uint8_t *data_ptr, uint8_t length, uint8_t delay);
	int write_data(uint16_t command, uint8_t buffer[], uint8_t length);

	void sensor_compouse_msg(bool send);
	int sensor_has_new_measurement();

	uint8_t calc_crc(uint8_t data[2]);

private:

	SENSIRION_AP *sensor = NULL;

	sensor_airborne_particles_s _msg{};

	//float measured_values[10];
	uint32_t measurement_time = 0;
	uint16_t measurement_index = 0;

	sps_info _sps_info;
	sensor_type _sensor_type = sensor_type::NONE;
	int _state = sensirion_airborne_particle_state::INIT;
	int _last_state = sensirion_airborne_particle_state::INIT;
	uint32_t _time_in_state = hrt_absolute_time();
	uint16_t _last_command = 0;
	uORB::PublicationMulti<sensor_airborne_particles_s> sensor_airborne_particles_pub{ORB_ID(sensor_airborne_particles)};

};
