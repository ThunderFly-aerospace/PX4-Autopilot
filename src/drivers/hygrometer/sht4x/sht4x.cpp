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
 * @file sht4x.c
 *
 * I2C driver for temperature/humidity sensor SHT4x by senserion
 *
 * @author Roman Dvorak <dvorakroman@thunderfly.cz>
 *
 */

#include "sht4x.h"

SHT4X::SHT4X(const I2CSPIDriverConfig &config) :
	I2C(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config)
{
}


uint8_t
SHT4X::calc_crc(uint8_t data[2])
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


uint8_t
SHT4X::validate_crc(uint8_t data[6])
{

	uint8_t crc_err = 0;

	for (int i = 0; i < 6 / 3; ++i) {
		uint8_t crc_data[2] = {data[i * 3], data[i * 3 + 1]};

		if (data[i * 3 + 2] != calc_crc(crc_data)) {
			crc_err ++;
		}

	}

	return crc_err;

}





uint8_t
SHT4X::read_serialnumber()
{
	uint8_t data[6];
	uint8_t addr = SHT4x_CMD_READ_SERIAL;
	transfer(&addr, 1, nullptr, 0);
	px4_usleep(1000);
	transfer(nullptr, 0, data, 6);

	uint8_t crc = validate_crc(data);

	if (crc == PX4_OK) {
		_sht_info.serial_number = data[0] << 24 | data[1] << 16 | data[3] << 8 | data[4];
	}

	return crc;
}



uint8_t
SHT4X::sensor_compouse_msg(bool send)
{

	uint8_t data[6];
	uint8_t addr = SHT4x_CMD_MEASURE_HIGH_PRECISION;

	transfer(&addr, 1, nullptr, 0);
	px4_usleep(9000); // 9ms for high precision mode
	transfer(nullptr, 0, data, 6);

	uint8_t crc_err = validate_crc(data);

	if (crc_err == PX4_OK) {
		measurement_time = hrt_absolute_time();
		measurement_index ++;

		measured_temperature = (float) 175 * (data[0] << 8 | data[1]) / 65535 - 45;
		measured_humidity = (float) 125 * (data[3] << 8 | data[4]) / 65535 - 6;

		PX4_INFO("SHT4x: Temp: %.3f, Hum: %.3f", (double)measured_temperature, (double)measured_humidity);

		if (measured_humidity < 0) { measured_humidity = 0; }

		if (measured_humidity > 100) { measured_humidity = 100; }


		if (send) {
			sensor_hygrometer_s msg{};
			msg.timestamp = hrt_absolute_time();
			msg.timestamp_sample = measurement_time;
			msg.temperature = measured_temperature;
			msg.humidity = measured_humidity;
			msg.device_id = _sht_info.serial_number;
			_sensor_hygrometer_pub.publish(msg);
		}
	}

	return crc_err;
}


int
SHT4X::probe()
{
	return read_serialnumber();
}

int
SHT4X::init()
{
	if (I2C::init() != PX4_OK) {
		return PX4_ERROR;
	}

	read_serialnumber();

	_sensor_hygrometer_pub.advertise();
	ScheduleOnInterval(100000);
	return PX4_OK;
}



int
SHT4X::init_sensor()
{
	PX4_INFO("Connected to SHT4x sensor, SN: %ld", _sht_info.serial_number);
	PX4_INFO("Last values, T: %.3f, H: %.3f", (double)measured_temperature, (double)measured_humidity);

	return PX4_OK;
}

void
SHT4X::RunImpl()
{
	PX4_INFO("SHT4X: RunImpl");
	sensor_compouse_msg(true);
}




void
SHT4X::print_status()
{
	PX4_INFO("SHT4X sensor");
	I2CSPIDriverBase::print_status();
	PX4_INFO("SN: %ld", _sht_info.serial_number);
	PX4_INFO("Last measured values (%.3fs ago, #%d)", (double)(hrt_absolute_time() - measurement_time) / 1000000.0,
		 measurement_index);
	PX4_INFO("Temp: %.3f, Hum: %.3f", (double)measured_temperature, (double)measured_humidity);
}


void
SHT4X::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
SHT4x Temperature and Humidity Sensor Driver by Senserion.

### Examples
CLI usage example:
$ sht4x start -X
  Start the sensor driver on the external bus

$ sht4x status
  Print driver status

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sht4x", "driver");

	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x44);

	PRINT_MODULE_USAGE_PARAMS_I2C_KEEP_RUNNING_FLAG();
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

}

int
sht4x_main(int argc, char *argv[])
{
	using ThisDriver = SHT4X;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	cli.i2c_address = 0x44;
	cli.support_keep_running = true;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_HYGRO_DEVTYPE_SHT4X);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}


	ThisDriver::print_usage();
	return -1;
}
