#pragma once

#include <stdint.h>

#include <px4_defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>

#include <errno.h>

#include <math.h>	// NAN
#include <cstring>

#include <lib/drivers/device/device.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <drivers/device/i2c.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/led/led.h>
#include <lib/tunes/tunes.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/tf_esc_report.h>
#include <uORB/topics/tf_esc_set.h>

#include <drivers/drv_hrt.h>
#include <perf/perf_counter.h>


using namespace time_literals;

#define MODULE_NAME "tfesc03"

class TFESC03 : public device::I2C, public I2CSPIDriver<TFESC03>
{
public:
	TFESC03(const I2CSPIDriverConfig &config);
	virtual ~TFESC03();

	void custom_method(const BusCLIArguments &cli) override;
	static void print_usage();
	void print_status() override;

	int init() override;
  int probe() override;
  void exit_and_cleanup() override;
	virtual void RunImpl();

private:
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	bool _initialized{false};

/*	uORB::PublicationMulti<esc_status_s> _esc_feedback_pub{ORB_ID(esc_status)};
	esc_status_s      _esc_feedback{};
	uORB::Publication<tf_esc_set_s> _tf_esc_set_pub{ORB_ID(tf_esc_set)};
  tf_esc_set_s      _tf_esc_set{};*/

	uint8_t    	  _channel_num{0};

	//uORB::Subscription _tf_esc_set_sub{ORB_ID(tf_esc_set)};
  uORB::SubscriptionCallbackWorkItem _tf_esc_set_sub{this, ORB_ID(tf_esc_set)};

  tf_esc_set_s      _tf_esc_set{};


  void readEscRegister(uint32_t registerAddress, uint8_t* data, size_t length, int crcEnabled);
  void writeEscRegister32_nocrc(uint32_t registerAddress, uint32_t data);
  uint32_t readEscRegister32_nocrc(uint32_t registerAddress);

  void setSpeed(uint16_t speed);
};
