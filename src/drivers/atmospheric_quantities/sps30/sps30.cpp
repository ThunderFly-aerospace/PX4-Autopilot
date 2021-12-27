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
 * @file sps30.c
 *
 * I2C driver for airborne particles meter SPS30
 *
 * @author Roman Dvorak <dvorakroman@thunderfly.cz>
 *
 */

#include "sps30.h"

SPS30::SPS30(const I2CSPIDriverConfig &config) :
	I2C(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config)
	//_interface(interface)
{
}

uint8_t SPS30::calc_crc(uint8_t data[2])
{
	uint8_t crc = 0xFF;

	for (int i = 0; i < 2; i++) {
		crc ^= data[i];

		for (uint8_t bit = 8; bit > 0; --bit) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ 0x31u;

			} else {
				crc = (crc << 1);
			}
		}
	}

	return crc;
}


int SPS30::set_pointer(uint16_t command)
{
	if (_last_command != command) {
		uint8_t cmd[2];
		cmd[0] = static_cast<uint8_t>(command >> 8);
		cmd[1] = static_cast<uint8_t>(command & 0xff);
		_last_command = command;
		return transfer(&cmd[0], 2, nullptr, 0);

	} else {
		return 0;
	}
}

int SPS30::read_data(uint16_t command, uint8_t *data_ptr, uint8_t length)
{
	set_pointer(command);

	uint8_t raw_data[length];
	transfer(nullptr, 0, &raw_data[0], length);

	uint8_t crc_err = 0;

	for (int i = 0; i < length / 3; ++i) {
		uint8_t crc_data[2] = {raw_data[i * 3], raw_data[i * 3 + 1]};

		if (raw_data[i * 3 + 2] != calc_crc(crc_data)) {
			crc_err ++;
		}

		*(data_ptr + i * 2) = raw_data[i * 3];
		*(data_ptr + i * 2 + 1) = raw_data[i * 3 + 1];
	}

	return crc_err;
}

int SPS30::write_data(uint16_t command, uint8_t buffer[], uint8_t length)
{
	_last_command = command;

	uint8_t cmd[2 + 3 * length / 2];
	cmd[0] = static_cast<uint8_t>(command >> 8);
	cmd[1] = static_cast<uint8_t>(command & 0xff);

	for (int i = 0; i < length / 2; ++i) {
		cmd[2 + 3 * i] = buffer[2 * i];
		cmd[2 + 3 * i + 1] = buffer[2 * i + 1];
		uint8_t crc_data[2] = {buffer[2 * i], buffer[2 * i + 1]};
		cmd[2 + 3 * i + 2] = calc_crc(crc_data);
	}

	return transfer(&cmd[0], sizeof(cmd), nullptr, 0);
}


void SPS30::sensor_compouse_msg(bool send)
{
	uint8_t data[60 / 3 * 2];
	read_data(SPS30_READ_VALUES, &data[0], 60);
	measurement_time = hrt_absolute_time();
	measurement_index ++;

	uint32_t d = 0;

	if (send) {
		sensor_airborne_particles_s msg{};
		msg.timestamp = hrt_absolute_time();

		d = ((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | (data[3]));
		measured_values[0] = *(float *)&d;
		msg.mass_concentration_pm1_0 = *(float *)&d;
		d = ((data[4] << 24) | (data[5] << 16) | (data[6] << 8) | (data[7]));
		measured_values[1] = *(float *)&d;
		msg.mass_concentration_pm2_5 = *(float *)&d;
		d = ((data[8] << 24) | (data[9] << 16) | (data[10] << 8) | (data[11]));
		measured_values[2] = *(float *)&d;
		msg.mass_concentration_pm4_0 = *(float *)&d;
		d = ((data[12] << 24) | (data[13] << 16) | (data[14] << 8) | (data[15]));
		measured_values[3] = *(float *)&d;
		msg.mass_concentration_pm10_0 = *(float *)&d;

		d = ((data[16] << 24) | (data[17] << 16) | (data[18] << 8) | (data[19]));
		measured_values[4] = *(float *)&d;
		msg.number_concentration_pm0_5 = *(float *)&d;
		d = ((data[20] << 24) | (data[21] << 16) | (data[22] << 8) | (data[23]));
		measured_values[5] = *(float *)&d;
		msg.number_concentration_pm1_0 = *(float *)&d;
		d = ((data[24] << 24) | (data[25] << 16) | (data[26] << 8) | (data[27]));
		measured_values[6] = *(float *)&d;
		msg.number_concentration_pm2_5 = *(float *)&d;
		d = ((data[28] << 24) | (data[29] << 16) | (data[30] << 8) | (data[31]));
		measured_values[7] = *(float *)&d;
		msg.number_concentration_pm4_0 = *(float *)&d;
		d = ((data[32] << 24) | (data[33] << 16) | (data[34] << 8) | (data[35]));
		measured_values[8] = *(float *)&d;
		msg.number_concentration_pm10_0 = *(float *)&d;

		d = ((data[36] << 24) | (data[37] << 16) | (data[38] << 8) | (data[39]));
		measured_values[9] = *(float *)&d;
		msg.typical_particle_size = *(float *)&d;

		sensor_airborne_particles_pub.publish(msg);
	}
}


int SPS30::sensor_has_new_measurement()
{
	uint8_t data[3 / 3 * 2];
	read_data(SPS30_READ_READY_FLAG, &data[0], 3);
	int new_measurement = (int) data[1];
	return (new_measurement);
}


int
SPS30::probe()
{
	uint8_t type[12 / 3 * 2];
	uint8_t compare [12 / 3 * 2] = {48, 48, 48, 56, 48, 48, 48, 48};
	uint8_t nvalid;
	nvalid = read_data(SPS30_READ_PRODUCT_TYPE, &type[0], 12);

	return (!(nvalid == PX4_OK) && ((bool) memcmp(type, compare, sizeof(type))));
	// 0 means I can see sensor
}

int SPS30::init()
{
	if (I2C::init() != PX4_OK) {
		return PX4_ERROR;
	}

	init_sensor();
	sensor_airborne_particles_pub.advertise();
	ScheduleOnInterval(50000);
	return PX4_OK;
}


int SPS30::init_sensor()
{
	// check if sensor is connected
	probe();

	// get serial number and firmware version
	uint8_t sn[32];
	read_data(SPS30_READ_SERIAL_NUMBER, &sn[0], 48);
	read_data(SPS30_READ_VERSION, _sps_info.version, 3);

	PX4_INFO("Starting SPS30 sensor");

	// convert sn to char array
	for (int i = 0; i < 32; i++) {
		_sps_info.serial_number[i] = (char) sn[i];
	}

	// start-up, start measurement
	set_pointer(SPS30_WAKEUP);
	px4_udelay(500000);

	uint8_t data[2] = {0x03, 0x00};
	write_data(SPS30_START_MEASUREMENT, data,  2);
	px4_udelay(500000);

	return PX4_OK;
}

void SPS30::RunImpl()
{
	switch (_state) {
	case sps30_state::INIT:
		probe();
		init_sensor();
		_state = sps30_state::MEASUREMENT;
		break;

	case sps30_state::MEASUREMENT:
		if (sensor_has_new_measurement() == 1) {
			sensor_compouse_msg(1);
		}

		if ((hrt_absolute_time() - measurement_time) > 3 * 1_s) {
			_state = sps30_state::ERROR_READOUT;
			PX4_ERR("I cant get data from SPS30");
		}

		break;

	case sps30_state::CLEAN_START:
		if (_time_in_state > 1_s) {
			sensor_compouse_msg(0);
			_state = sps30_state::CLEANINNG;
		}

		break;

	case sps30_state::CLEANINNG:
		if (sensor_has_new_measurement()) {
			_state = sps30_state::MEASUREMENT;
		}

		break;

	case sps30_state::ERROR_GENERAL:
	case sps30_state::ERROR_READOUT: {
			sensor_has_new_measurement();

			if (probe() == PX4_OK) {
				_state = sps30_state::INIT;
			}
		}
		break;
	}

	if (_last_state != _state) {
		_time_in_state = hrt_absolute_time();
		_last_state = _state;
	}
}


void SPS30::start_clean()
{
	set_pointer(SPS30_FAN_CLEANING);
	_state = sps30_state::CLEAN_START;
};


void
SPS30::custom_method(const BusCLIArguments &cli)
{
	switch (cli.custom1) {
	case 1: {
			start_clean();
			PX4_INFO("Start CLEANING procedure");
		}
		break;

	case 2: {
			PX4_INFO("Last measured values (%.3fs ago, #%d)", (double)(hrt_absolute_time() - measurement_time) / 1000000.0,
				 measurement_index);
			PX4_INFO("Mass 1:%.3f 2.5:%.3f 4:%.3f 10:%.3f", (double) measured_values[0], (double) measured_values[1],
				 (double) measured_values[2], (double) measured_values[3]);
			PX4_INFO("Count 0.5:%.3f 1:%.3f 2.5:%.3f 4:%.3f 10:%.3f", (double) measured_values[4], (double) measured_values[5],
				 (double) measured_values[6], (double) measured_values[7], (double) measured_values[8]);
			PX4_INFO("TPS:%.3f", (double) measured_values[9]);
		}
		break;

	case 3: {
			px4_udelay(5000);
			PX4_INFO("PROBE: %d", probe());
			px4_udelay(5000);
			PX4_INFO("New measurement: %d", sensor_has_new_measurement());
		}
		break;

	case 4: {
			_state = sps30_state::INIT;
		}
		break;
	}
}


void SPS30::print_status()
{
	PX4_INFO("SPS30 sensor");
	I2CSPIDriverBase::print_status();
	PX4_INFO("FW ver: %d.%d, SN: %s", _sps_info.version[0], _sps_info.version[1], _sps_info.serial_number);
	PX4_INFO("Status: %s", psp30_state_names[_state]);
}


void SPS30::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Senserion SPS30 airborne particles sensor driver
SPS30 is a dust particle sensor by Senserion.
This software takes care of sensor operation, reading and logging measured data.

### Examples
CLI usage example:
$ sps30 start

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sps30", "driver");

    PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	// PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x69); // The sensor has only one address

	PRINT_MODULE_USAGE_PARAMS_I2C_KEEP_RUNNING_FLAG();
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	PRINT_MODULE_USAGE_COMMAND_DESCR("clean", "Run cleaning procedure.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("values", "Print actual data");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Test system...");
    PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reinitialize sensor");

}

int sps30_main(int argc, char *argv[])
{
	using ThisDriver = SPS30;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	cli.i2c_address = 0x69;
	cli.support_keep_running = true;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_SENS_DEVTYPE_SPS30);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	if (!strcmp(verb, "clean")) {
		cli.custom1 = 1;
		return ThisDriver::module_custom_method(cli, iterator, false);
	}

	if (!strcmp(verb, "values")) {
		cli.custom1 = 2;
		return ThisDriver::module_custom_method(cli, iterator, false);
	}
	if (!strcmp(verb, "test")) {
		cli.custom1 = 3;
		return ThisDriver::module_custom_method(cli, iterator, false);
	}
	if (!strcmp(verb, "reset")) {
		cli.custom1 = 4;
		return ThisDriver::module_custom_method(cli, iterator, false);
	}

	ThisDriver::print_usage();
	return -1;
}
