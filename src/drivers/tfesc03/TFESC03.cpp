/****************************************************************************
 *
 *   Copyright (c) 2018-2022 PX4 Development Team. All rights reserved.
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
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "TFESC03.hpp"

#include <px4_platform_common/sem.hpp>

TFESC03::TFESC03(const I2CSPIDriverConfig &config) 
  :I2C(config),
   I2CSPIDriver(config)
{
  _channel_num=0;
}

TFESC03::~TFESC03()
{

}

int TFESC03::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

  _tf_esc_set_sub.set_interval_us(20_ms);
	if (!_tf_esc_set_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return PX4_ERROR;
	}

	PX4_INFO("Initialize TFESC03.");
	return 0;
}

int TFESC03::probe()
{
  PX4_INFO ("TFESC03 Probe!");
/*  if(this->ReadReg(REG_LORASYNCWORDLSB) == 0x24)
  {
    PX4_INFO ("I see the tflora!");
    return PX4_OK;
  }*/

    return PX4_OK;
}

void TFESC03::RunImpl()
{
  //check message
  if (_tf_esc_set_sub.updated()) 
  {
		if (_tf_esc_set_sub.copy(&_tf_esc_set)) 
    {
			PX4_INFO("recieved set: %d",_tf_esc_set.output[0]);
      setSpeed(_tf_esc_set.output[0]);
    }
  }
}

void TFESC03::exit_and_cleanup()
{
	//stop motor first..

	I2CSPIDriverBase::exit_and_cleanup();
}

void TFESC03::custom_method(const BusCLIArguments &cli)
{

    if(cli.custom2==1)
    {
      uint32_t data;
      readEscRegister( 0x476, (uint8_t*) (&data), 4, 0);
      printf("V: %f\n", data*60.0/(1<<27));
      return;
    }

    if(cli.custom2==2)
    {
	    setSpeed(0);
      return;
    }

    if(cli.custom2==5)
    {
	    setSpeed(5);
      return;
    }

    if(cli.custom2==10)
    {
	    setSpeed(10);
      return;
    }

    if(cli.custom2==20)
    {
	    setSpeed(20);
      return;
    }

  	//print_usage();
}

void TFESC03::print_usage()
{

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for the TFESC03 ESCs.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("TFESC03", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
//	PRINT_MODULE_USAGE_PARAM_INT('c', 0, 0, 7, "channel", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

void TFESC03::print_status()
{
	PX4_INFO("Running");
	PX4_INFO("Channel: %u", (unsigned)_channel_num);
	return;
}

extern "C" __EXPORT int tfesc03_main(int argc, char *argv[])
{
	using ThisDriver = TFESC03;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	cli.i2c_address = 0x01;
	cli.support_keep_running = true; //?

	const char *verb = cli.parseDefaultArguments(argc, argv);

  //parse c chanell to custom1..
  cli.custom1=0;

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_ESC_TFESC03);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	if (!strcmp(verb, "voltage")) {
		cli.custom2 = 1;
		return ThisDriver::module_custom_method(cli, iterator, false);
	}

	if (!strcmp(verb, "0")) {
		cli.custom2 = 2;
		return ThisDriver::module_custom_method(cli, iterator, false);
	}

	if (!strcmp(verb, "5")) {
		cli.custom2 = 5;
		return ThisDriver::module_custom_method(cli, iterator, false);
	}

	if (!strcmp(verb, "10")) {
		cli.custom2 = 10;
		return ThisDriver::module_custom_method(cli, iterator, false);
	}

	if (!strcmp(verb, "20")) {
		cli.custom2 = 20;
		return ThisDriver::module_custom_method(cli, iterator, false);
	}

	if (!strcmp(verb, "reset")) {
		cli.custom2 = 2;
		return ThisDriver::module_custom_method(cli, iterator, false);
	}

	ThisDriver::print_usage();
	return -1;
}

void TFESC03::readEscRegister(uint32_t registerAddress, uint8_t* data, size_t length, int crcEnabled) 
{
    uint8_t OP_R_W = 1; // Read operation
    uint8_t CRC_EN = crcEnabled ? 1 : 0;
    uint8_t DLEN = 0b01; // 32-bit data length
    uint8_t MEM_SEC = (registerAddress >> 16) & 0xF;
    uint8_t MEM_PAGE = (registerAddress >> 12) & 0xF;
    uint16_t MEM_ADDR = registerAddress & 0xFFF;

    uint32_t controlWord = (OP_R_W << 23) | (CRC_EN << 22) | (DLEN << 20) | (MEM_SEC << 16) | (MEM_PAGE << 12) | MEM_ADDR;
    
    uint8_t controlBytes[3];
    controlBytes[0] = (controlWord >> 16) & 0xFF;
    controlBytes[1] = (controlWord >> 8) & 0xFF;
    controlBytes[2] = controlWord & 0xFF;

    printf("Control bytes: ");
    for(int i=0;i<3;i++)
    {
        printf("0x%02x ", (uint8_t)controlBytes[i]);
    }
    printf("\n");


    // Read data from the device
    size_t readLength = length + (crcEnabled ? 1 : 0);

    transfer( controlBytes, 3, data, readLength);
    //transfer( controlBytes, 3, NULL, 0);
    //transfer( NULL, 0, data, readLength);

    if (crcEnabled) {
        // Handle CRC check if enabled
        //uint8_t crc = data[readLength - 1];
        // Add CRC handling code here if necessary
    }
}

uint32_t TFESC03::readEscRegister32_nocrc(uint32_t registerAddress) 
{
    uint8_t OP_R_W = 1; // Read operation
    uint8_t CRC_EN = 0;
    uint8_t DLEN = 0b01; // 32-bit data length
    uint8_t MEM_SEC = (registerAddress >> 16) & 0xF;
    uint8_t MEM_PAGE = (registerAddress >> 12) & 0xF;
    uint16_t MEM_ADDR = registerAddress & 0xFFF;

    uint32_t controlWord = (OP_R_W << 23) | (CRC_EN << 22) | (DLEN << 20) | (MEM_SEC << 16) | (MEM_PAGE << 12) | MEM_ADDR;
    
    uint8_t controlBytes[3];
    controlBytes[0] = (controlWord >> 16) & 0xFF;
    controlBytes[1] = (controlWord >> 8) & 0xFF;
    controlBytes[2] = controlWord & 0xFF;

    printf("Control bytes: ");
    for(int i=0;i<3;i++)
    {
        printf("0x%02x ", (uint8_t)controlBytes[i]);
    }
    printf("\n");

    uint32_t data=0;

    transfer( controlBytes, 3, (uint8_t*)&data, 4);
    //transfer( controlBytes, 3, NULL, 0);
    //transfer( NULL, 0, data, readLength);
    return data;
}

void TFESC03::writeEscRegister32_nocrc(uint32_t registerAddress, uint32_t data) 
{
    uint8_t OP_R_W = 0; // Write operation
    uint8_t CRC_EN = 0; //No CRC
    uint8_t DLEN = 0b01; // 32-bit data length
    uint8_t MEM_SEC = (registerAddress >> 16) & 0xF;
    uint8_t MEM_PAGE = (registerAddress >> 12) & 0xF;
    uint16_t MEM_ADDR = registerAddress & 0xFFF;

    uint32_t controlWord = (OP_R_W << 23) | (CRC_EN << 22) | (DLEN << 20) | (MEM_SEC << 16) | (MEM_PAGE << 12) | MEM_ADDR;
    
    uint8_t sendBytes[7];
    uint8_t *dataBytes=(uint8_t*)&data;
    sendBytes[0] = (controlWord >> 16) & 0xFF;
    sendBytes[1] = (controlWord >> 8) & 0xFF;
    sendBytes[2] = controlWord & 0xFF;


    for(int i=0;i<4;i++)
      sendBytes[3+i]=dataBytes[i];

    printf("Send bytes: ");
    for(int i=0;i<7;i++)
    {
        printf("0x%02x ", (uint8_t)sendBytes[i]);
    }
    printf("\n");

    transfer( sendBytes, 7, NULL, 0);

}

void TFESC03::setSpeed(uint16_t speed)
{
    uint16_t speedScaled;
    speedScaled=(1<<15)/100*speed;
    if(speedScaled>=(1<<15))
      speedScaled=(1<<15)-1;
    printf("speed: %d - 0x%04x\n", speed,speedScaled);

    //set speed
    uint32_t reg;
    reg = readEscRegister32_nocrc(0xEC);
    printf("0x%08x \n", (unsigned int)reg);

    reg &=0xFFFFu;
    reg |= 1u<<31;
    reg |=speedScaled<<16;

    printf("0x%08x \n", (unsigned int)reg);
    writeEscRegister32_nocrc(0xEC, reg);

}

