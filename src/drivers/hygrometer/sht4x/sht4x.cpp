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


uint8_t SHT4X::calc_crc(uint8_t data[2])
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




int SHT4X::read_data(uint8_t command, uint8_t *data_ptr, uint8_t length)
{

	PX4_INFO("READ DATA...");
	PX4_INFO("%x", command);

	uint8_t raw_data[length/2*3];
	transfer(&command, 1, &raw_data[0], length/2*3);

	PX4_INFO("RAW DATA...");
	for (int i = 0; i < length/2*3; i++) {
		PX4_INFO("> %x", raw_data[i]);
	}

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


int SHT4X::read_serialnumber()
{
	int err = read_data(SHT4x_CMD_MEASURE_HIGH_PRECISION, (uint8_t *)&_sht_info.serial_number, 4);
	PX4_INFO("READ SN STATUS: %d", err);

	return err;
}



void SHT4X::sensor_compouse_msg(bool send)
{
	uint8_t data[4];
	int error = read_data(SHT4x_CMD_MEASURE_HIGH_PRECISION, &data[0], 4);

	if (error == PX4_OK) {
		measurement_time = hrt_absolute_time();
		measurement_index ++;

		measured_temperature = (float) 175 * (data[0] << 8 | data[1]) / 65535 - 45;
		measured_humidity = (float) 100 * (data[2] << 8 | data[3]) / 65535;

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
}


int
SHT4X::probe()
{
	//return read_serialnumber();
	// 0 means I can see sensor
	return 0;
}

int SHT4X::init()
{
	if (I2C::init() != PX4_OK) {
		return PX4_ERROR;
	}

	read_data(SHT4x_CMD_SOFT_RESET, nullptr, 0);
	px4_usleep(2000);
	_sensor_hygrometer_pub.advertise();
	ScheduleOnInterval(20000);
	return PX4_OK;
}



int SHT4X::init_sensor()
{
	probe();

	PX4_INFO("Connected to SHT4x sensor, SN: %ld", _sht_info.serial_number);

	return PX4_OK;
}

void SHT4X::RunImpl()
{
	sensor_compouse_msg(true);
}


void
SHT4X::action_cli(const BusCLIArguments &cli)
{
	switch (cli.custom1) {

	case 1: {
			PX4_INFO("Last measured values (%.3fs ago, #%d)", (double)(hrt_absolute_time() - measurement_time) / 1000000.0,
				 measurement_index);
			PX4_INFO("Temp: %.3f, Hum: %.3f", (double)measured_temperature, (double)measured_humidity);

		}
		break;

	case 2: {
			read_data(SHT4x_CMD_SOFT_RESET, nullptr, 0);
		}
		break;
	}
}



void SHT4X::print_status()
{
	PX4_INFO("SHT4X sensor");
	I2CSPIDriverBase::print_status();
	PX4_INFO("SN: %ld", _sht_info.serial_number);
}


void SHT4X::print_usage()
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

$ sht4x values
  Print last measured values

$ sht4x reset
  Reinitialize senzor, reset flags

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sht4x", "driver");

	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x44);

	PRINT_MODULE_USAGE_PARAMS_I2C_KEEP_RUNNING_FLAG();
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	PRINT_MODULE_USAGE_COMMAND_DESCR("values", "Print actual data");
    PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reinitialize sensor");

}

int sht4x_main(int argc, char *argv[])
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

	if (!strcmp(verb, "values")) {
		cli.custom1 = 1;
		return ThisDriver::module_custom_method(cli, iterator, false);
	}

	if (!strcmp(verb, "reset")) {
		cli.custom1 = 2;
		return ThisDriver::module_custom_method(cli, iterator, false);
	}

	ThisDriver::print_usage();
	return -1;
}
