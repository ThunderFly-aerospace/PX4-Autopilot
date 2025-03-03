/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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
 * @file tflora.hpp
 *
 * Driver for TFLORA connected via SPI.
 *
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/spi.h>
//#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module_params.h>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/outgoing_lora_message.h>

#undef ASSERT
extern "C"{
#include "lmic/lmic.h"
#include "board.h"
}


//using namespace TFLORA;

class TFLORA : public device::SPI, public ModuleParams, public I2CSPIDriver<TFLORA>
{
public:
	TFLORA(const I2CSPIDriverConfig &config);
	~TFLORA() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	//void print_status() override;

  //LMIC HAL backend
  uint8_t busy_enabled;
  uint8_t irq_enabled; //lmic want sometime disable interrupt to not be disturbed
  uint32_t irq_time;

  void wait_busy_pin();
  void spi_select(); 
  uint8_t spi_transfer(uint8_t data);
  void spi_deselect();

  void hal_disableIrq();
  void hal_enableIrq();

private:
	void exit_and_cleanup() override;
	int probe() override;
  void ReadRegs (uint16_t addr, uint8_t* data, uint8_t len);
  uint8_t ReadReg (uint16_t addr);

  uORB::SubscriptionCallbackWorkItem outgoing_lora_message_sub{this, ORB_ID(outgoing_lora_message)};

  //real interupt handling
	static int RadioInterruptCallback(int irq, void *context, void *arg);
	void RadioInterrupt();
	bool RadioInterruptConfigure();
	bool RadioInterruptDisable();

	const spi_drdy_gpio_t _drdy_gpio;

	enum class STATE : uint8_t {
		SLEEPING_FOREVER,
		WAIT_FOR_JOB_TIME,		
		JOB_DONE,
	} _state{STATE::SLEEPING_FOREVER};


  outgoing_lora_message_s outMsg;
  osjob_t* currentJob;

  int ordinary_dr=EU_DR_SF7; //TODO - from param
  int extraordinary_dr;
  int extraordinary_dr_period;

  enum {
    EU_DR_SF12 = 0,
    EU_DR_SF11 = 1,
    EU_DR_SF10 = 2,
    EU_DR_SF9 = 3,
    EU_DR_SF8 = 4,
    EU_DR_SF7 = 5,
    EU_DR_SF7_BW250 = 6,
    EU_DR_FSK = 7,
  };

  void init_lmic();
  bool parse_hex(const char *argname, const char *s, uint8_t *t, int hexlen);


  // LoRaWAN NwkSKey, network session key
  u1_t NWKSKEY[16];
  // LoRaWAN AppSKey, application session key
  u1_t APPSKEY[16];
  // LoRaWAN end-device address (DevAddr)
  u4_t DEVADDR;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::TTN_NSKEY_0>) _param_nwkskey_0,
		(ParamInt<px4::params::TTN_NSKEY_1>) _param_nwkskey_1,
		(ParamInt<px4::params::TTN_NSKEY_2>) _param_nwkskey_2,
		(ParamInt<px4::params::TTN_NSKEY_3>) _param_nwkskey_3,

		(ParamInt<px4::params::TTN_APPKEY_0>) _param_appkey_0,
		(ParamInt<px4::params::TTN_APPKEY_1>) _param_appkey_1,
		(ParamInt<px4::params::TTN_APPKEY_2>) _param_appkey_2,
		(ParamInt<px4::params::TTN_APPKEY_3>) _param_appkey_3,

    (ParamInt<px4::params::TTN_DEVADDR>) _param_devaddr
	)

  void parameters_update();
};



