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
 * @file sht4x.h
 *
 * Header file for SHT4x driver
 *
 * @author Roman Dvorak <dvorakroman@thunderfly.cz>
 *
 */

#pragma once

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_hygrometer.h>

#define SHT4x_CMD_MEASURE_HIGH_PRECISION 	0xFD
#define SHT4x_CMD_MEASURE_MEDIUM_PRECISION 	0xF6
#define SHT4x_CMD_MEASURE_LOW_PRECISION		0xE0
#define SHT4x_CMD_READ_SERIAL 			0x89
#define SHT4x_CMD_SOFT_RESET 			0x94
#define SHT4x_CMD_ACTIVATE_HEATER_200mW_1S 	0x39
#define SHT4x_CMD_ACTIVATE_HEATER_200mW_0_1S 	0x32
#define SHT4x_CMD_ACTIVATE_HEATER_110mW_1S 	0x2F
#define SHT4x_CMD_ACTIVATE_HEATER_110mW_0_1S 	0x24
#define SHT4x_CMD_ACTIVATE_HEATER_20mW_1S 	0x1E
#define SHT4x_CMD_ACTIVATE_HEATER_20mW_0_1S 	0x15


using namespace time_literals;

extern "C" __EXPORT int sht4x_main(int argc, char *argv[]);

struct sht_info {
	uint32_t serial_number;
};


class SHT4X : public device::I2C, public ModuleParams, public I2CSPIDriver<SHT4X>
{
public:
	SHT4X(const I2CSPIDriverConfig &config);
	~SHT4X() = default;

	static void print_usage();
	void RunImpl();

	int    init() override;
	int    probe() override;
	int    init_sensor();
	void   print_status() override;

	//void action_cli(const BusCLIArguments &cli);


	//int read_data(uint8_t command, uint8_t *data_ptr, uint8_t length);
	int write_data(uint8_t command, uint8_t buffer[], uint8_t length);

	uint8_t read_serialnumber();

	uint8_t sensor_compouse_msg(bool send);

	uint8_t calc_crc(uint8_t data[2]);
	uint8_t validate_crc(uint8_t data[6]);

private:

	float measured_temperature = 0;
	float measured_humidity = 0;
	uint32_t measurement_time = 0;
	uint16_t measurement_index = 0;

	sht_info _sht_info;
	//int _state = sht4x_state::INIT;
	//int _last_state = sht4x_state::INIT;
	uint32_t _time_in_state = hrt_absolute_time();
	uint16_t _last_command = 0;
	uORB::PublicationMulti<sensor_hygrometer_s> _sensor_hygrometer_pub{ORB_ID(sensor_hygrometer)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_EN_SHT4X>) _param_sens_en_sht4x
	)
};
