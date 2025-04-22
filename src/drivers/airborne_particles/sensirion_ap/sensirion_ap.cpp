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
 * @file sensirion_airborneparticles.c
 *
 * I2C driver for airborne particles meter SPS30
 *
 * @author Roman Dvorak <dvorakroman@thunderfly.cz>
 *
 */

#include "sensirion_ap.h"


SENSIRION_AP::SENSIRION_AP(const I2CSPIDriverConfig &config) :
	I2C(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config)
	//_interface(interface)
{
}

uint8_t SENSIRION_AP::calc_crc(uint8_t data[2])
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


int SENSIRION_AP::set_pointer(uint16_t command)
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

int SENSIRION_AP::read_data(uint16_t command, uint8_t *data_ptr, uint8_t length)
{
	return read_data(command, data_ptr, length, 5);
};

int SENSIRION_AP::read_data(uint16_t command, uint8_t *data_ptr, uint8_t length, uint8_t delay)
{
	set_pointer(command);

	px4_usleep(delay * 1000);

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

int SENSIRION_AP::write_data(uint16_t command, uint8_t buffer[], uint8_t length)
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


void SENSIRION_AP::sensor_compouse_msg(bool send)
{

	switch (_sensor_type) {
	case sensor_type::SPS30: {
			uint8_t data[60 / 3 * 2];
			read_data(SPS30_READ_VALUES, &data[0], 60);
			measurement_time = hrt_absolute_time();
			measurement_index ++;

			uint32_t d = 0;

			_msg.timestamp = hrt_absolute_time();

			d = ((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | (data[3]));
			_msg.mass_concentration_pm1_0 = *(float *)&d;
			d = ((data[4] << 24) | (data[5] << 16) | (data[6] << 8) | (data[7]));
			_msg.mass_concentration_pm2_5 = *(float *)&d;
			d = ((data[8] << 24) | (data[9] << 16) | (data[10] << 8) | (data[11]));
			_msg.mass_concentration_pm4_0 = *(float *)&d;
			d = ((data[12] << 24) | (data[13] << 16) | (data[14] << 8) | (data[15]));
			_msg.mass_concentration_pm10_0 = *(float *)&d;

			d = ((data[16] << 24) | (data[17] << 16) | (data[18] << 8) | (data[19]));
			_msg.number_concentration_pm0_5 = *(float *)&d;
			d = ((data[20] << 24) | (data[21] << 16) | (data[22] << 8) | (data[23]));
			_msg.number_concentration_pm1_0 = *(float *)&d;
			d = ((data[24] << 24) | (data[25] << 16) | (data[26] << 8) | (data[27]));
			_msg.number_concentration_pm2_5 = *(float *)&d;
			d = ((data[28] << 24) | (data[29] << 16) | (data[30] << 8) | (data[31]));
			_msg.number_concentration_pm4_0 = *(float *)&d;
			d = ((data[32] << 24) | (data[33] << 16) | (data[34] << 8) | (data[35]));
			_msg.number_concentration_pm10_0 = *(float *)&d;

			d = ((data[36] << 24) | (data[37] << 16) | (data[38] << 8) | (data[39]));
			_msg.typical_particle_size = *(float *)&d;

			if (send) {
				sensor_airborne_particles_pub.publish(_msg);
			}
		}
		break;

	case sensor_type::SEN50:
	case sensor_type::SEN54:
	case sensor_type::SEN55: {
			uint8_t data[24 / 3 * 2];
			read_data(SEN5X_READ_VALUES, &data[0], 24, 15);
			measurement_time = hrt_absolute_time();
			measurement_index ++;

			uint32_t d = 0;

			_msg.timestamp = hrt_absolute_time();

			d = ((data[0] << 8) | (data[1]));
			_msg.mass_concentration_pm1_0 = ((float) d) * 0.1f;
			d = ((data[2] << 8) | (data[3]));
			_msg.mass_concentration_pm2_5 = ((float) d) * 0.1f;
			d = ((data[4] << 8) | (data[5]));
			_msg.mass_concentration_pm4_0 = ((float) d) * 0.1f;
			d = ((data[6] << 8) | (data[7]));
			_msg.mass_concentration_pm10_0 = ((float) d) * 0.1f;

			d = ((data[8] << 8) | (data[9]));
			_msg.relative_humidity = ((float) d) * 0.01f;
			d = ((data[10] << 8) | (data[11]));
			_msg.temperature = ((float) d) * 0.005f;

			d = ((data[12] << 8) | (data[13]));
			_msg.voc_index = ((float) d) * 0.1f;
			d = ((data[14] << 8) | (data[15]));
			_msg.nox_index = ((float) d) * 0.1f;

			if (send) {
				sensor_airborne_particles_pub.publish(_msg);
			}
		}
		break;

	default:
		break;
	}
}


int SENSIRION_AP::sensor_has_new_measurement()
{
	switch (_sensor_type) {
	case sensor_type::SPS30: {
			uint8_t data[3 / 3 * 2];
			read_data(SPS30_READ_READY_FLAG, &data[0], 3);
			int new_measurement = (int) data[1];
			return (new_measurement);
		}
		break;

	case sensor_type::SEN50:
	case sensor_type::SEN54:
	case sensor_type::SEN55: {
			uint8_t data[3 / 3 * 2];
			read_data(SEN5X_READ_READY_FLAG, &data[0], 3, 10);
			uint8_t status[3 / 3 * 2];
			read_data(SEN5X_READ_DEVICE_STATUS, &status[0], 3, 10);

			int new_measurement = (bool)(data[1] == 1);
			return (new_measurement);
		}
		break;

	default:
		return 0;
	}
}


int
SENSIRION_AP::probe()
{
	// Probe SPS30 sensor
	uint8_t type_sps[12 / 3 * 2];
	uint8_t compare [12 / 3 * 2] = {48, 48, 48, 56, 48, 48, 48, 48};
	uint8_t nvalid;
	nvalid = read_data(SPS30_READ_PRODUCT_TYPE, &type_sps[0], 12);

	if ((nvalid == PX4_OK) && !((bool) memcmp(type_sps, compare, sizeof(type_sps)))) {
		_sensor_type = sensor_type::SPS30;
		return PX4_OK;
	}

	// Probe SEN5X sensors
	uint8_t type_sen5x[48 / 3 * 2];
	nvalid = read_data(SEN5X_READ_PRODUCT_NAME, &type_sen5x[0], 48);

	uint8_t compare_sen50[5] = {83, 69, 78, 53, 48};
	uint8_t compare_sen54[5] = {83, 69, 78, 53, 52};
	uint8_t compare_sen55[5] = {83, 69, 78, 53, 53};

	if ((nvalid == PX4_OK) && !((bool) memcmp(type_sen5x, compare_sen50, 5))) {
		_sensor_type = sensor_type::SEN50;
		return PX4_OK;

	} else if ((nvalid == PX4_OK) && !((bool) memcmp(type_sen5x, compare_sen54, 5))) {
		_sensor_type = sensor_type::SEN54;
		return PX4_OK;

	} else if ((nvalid == PX4_OK) && !((bool) memcmp(type_sen5x, compare_sen55, 5))) {
		_sensor_type = sensor_type::SEN55;
		return PX4_OK;
	}

	return -1;
	// return 1 - I cant see sensor
}

int SENSIRION_AP::init()
{
	if (I2C::init() != PX4_OK) {
		return PX4_ERROR;
	}

	probe();

	switch (_sensor_type) {
	case sensor_type::SPS30:
		init_SPS30_sensor();
		break;

	case sensor_type::SEN50:
	case sensor_type::SEN54:
	case sensor_type::SEN55:
		init_SEN5X_sensor();
		break;

	default:
		return PX4_ERROR;
	}

	sensor_airborne_particles_pub.advertise();
	ScheduleOnInterval(50000);
	return PX4_OK;
}


int SENSIRION_AP::init_SPS30_sensor()
{
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
	px4_usleep(50000);

	uint8_t data[2] = {0x03, 0x00};
	write_data(SPS30_START_MEASUREMENT, data,  2);
	px4_usleep(50000);
	_state = sensirion_airborne_particle_state::MEASUREMENT;


	return PX4_OK;
}

int SENSIRION_AP::init_SEN5X_sensor()
{
	// get serial number and firmware version
	uint8_t sn[32];
	read_data(SEN5X_READ_SERIAL_NUMBER, &sn[0], 48);
	read_data(SEN5X_READ_FIRMWARE_VERSION, _sps_info.version, 3);

	PX4_INFO("Starting SEN5X sensor");

	// convert sn to char array
	for (int i = 0; i < 32; i++) {
		_sps_info.serial_number[i] = (char) sn[i];
	}

	write_data(SEN5X_START_MEASUREMENT, nullptr, 0);
	px4_usleep(50000);
	_state = sensirion_airborne_particle_state::MEASUREMENT;

	return PX4_OK;
}

void SENSIRION_AP::RunImpl()
{
	switch (_state) {
	case sensirion_airborne_particle_state::INIT:
		init();
		break;

	case sensirion_airborne_particle_state::MEASUREMENT:
		if (sensor_has_new_measurement() == 1) {
			px4_usleep(15000);
			sensor_compouse_msg(1);
		}

		if ((hrt_absolute_time() - measurement_time) > 3 * 1_s) {
			_state = sensirion_airborne_particle_state::ERROR_READOUT;
			PX4_ERR("I cant get data from airborne particles sensor");
		}

		break;

	case sensirion_airborne_particle_state::CLEAN_START:
		if (_time_in_state > 1_s) {
			sensor_compouse_msg(0);
			_state = sensirion_airborne_particle_state::CLEANINNG;
		}

		break;

	case sensirion_airborne_particle_state::CLEANINNG:
		if (sensor_has_new_measurement()) {
			_state = sensirion_airborne_particle_state::MEASUREMENT;
		}

		break;

	case sensirion_airborne_particle_state::ERROR_GENERAL:
	case sensirion_airborne_particle_state::ERROR_READOUT: {
			if (probe() == PX4_OK) {
				_state = sensirion_airborne_particle_state::INIT;
			}
		}
		break;
	}

	if (_last_state != _state) {
		_time_in_state = hrt_absolute_time();
		_last_state = _state;
	}
}


void SENSIRION_AP::start_clean()
{
	switch (_sensor_type) {
	case sensor_type::SPS30:
		set_pointer(SPS30_FAN_CLEANING);
		break;

	default:
		set_pointer(SEN5X_FAN_CLEANING);
	}

	_state = sensirion_airborne_particle_state::CLEAN_START;
};


void
SENSIRION_AP::custom_method(const BusCLIArguments &cli)
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
			PX4_INFO("Mass 1:%.3f 2.5:%.3f 4:%.3f 10:%.3f", (double) _msg.mass_concentration_pm1_0,
				 (double) _msg.mass_concentration_pm2_5,
				 (double) _msg.mass_concentration_pm4_0, (double) _msg.mass_concentration_pm10_0);
			PX4_INFO("Count 0.5:%.3f 1:%.3f 2.5:%.3f 4:%.3f 10:%.3f", (double) _msg.number_concentration_pm0_5,
				 (double) _msg.number_concentration_pm1_0,
				 (double) _msg.number_concentration_pm2_5, (double) _msg.number_concentration_pm4_0,
				 (double) _msg.number_concentration_pm10_0);
			PX4_INFO("TPS:%.3f", (double) _msg.typical_particle_size);
			PX4_INFO("Temp: %.3f C, Hum: %.3f ", (double) _msg.temperature, (double) _msg.relative_humidity);
			PX4_INFO("VOC: %.3f , NOx: %.3f ", (double) _msg.voc_index, (double) _msg.nox_index);
		}
		break;

	case 3: { // test

		}
		break;

	case 4: {
			_state = sensirion_airborne_particle_state::INIT;
		}
		break;
	}
}


void SENSIRION_AP::print_status()
{
	PX4_INFO("Sensirion airborne particles meter.");
	PX4_INFO("Sensor type: %s", _sensor_type_names[_sensor_type]);
	I2CSPIDriverBase::print_status();
	PX4_INFO("FW ver: %d.%d, SN: %s", _sps_info.version[0], _sps_info.version[1], _sps_info.serial_number);
	PX4_INFO("Status: %s", _state_names[_state]);
}


void SENSIRION_AP::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Senserion SPS30/SEN5x airborne particles sensor driver
SPS30 is a dust particle sensor by Senserion.
This software takes care of sensor operation, reading and logging measured data.

### Examples
CLI usage example:
$ sensirion_ap start

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sensirion_ap", "driver");

    PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x69); // The sensor has only one address

	PRINT_MODULE_USAGE_PARAMS_I2C_KEEP_RUNNING_FLAG();
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	PRINT_MODULE_USAGE_COMMAND_DESCR("clean", "Run cleaning procedure.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("values", "Print actual data");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Test system...");
    PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reinitialize sensor");

}

int sensirion_ap_main(int argc, char *argv[])
{
	using ThisDriver = SENSIRION_AP;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	cli.i2c_address = 0x69;
	cli.support_keep_running = true;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_SENS_DEVTYPE_SENSIRION_AP);

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
