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
#include <px4_platform_common/i2c_spi_buses.h>

#include <errno.h>
#include <math.h>	// NAN
#include <cstring>

#include <lib/drivers/device/device.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/led/led.h>
#include <lib/tunes/tunes.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/led_control.h>

#include <drivers/drv_hrt.h>
#include <perf/perf_counter.h>

#include "TFESC03_common.hpp"

// Constants
#define TAP_ESC_MAX_MOTOR_NUM 4
#define RPMSTOPPED 0
#define ESC_POS {0, 1, 2, 3}
#define ESC_DIR {0, 0, 0, 0}
#define ESC_MAX_RPM 1900
#define ESC_MIN_RPM 1200

using namespace time_literals;

#define MODULE_NAME "tfesc03"

class TFESC03 : public ModuleBase<TFESC03>, public OutputModuleInterface
{
public:
	TFESC03(I2CSPIBusOption bus_option, int bus, int addr, uint8_t channels_count);
	virtual ~TFESC03();

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

	int init();
	bool updateOutputs(bool stop_motors, uint16_t outputs[8],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

private:
	void Run() override;
	inline void send_esc_outputs(const uint16_t *pwm, const uint8_t motor_cnt);

	TFESC03_COMMON *_tfesc03_common{nullptr};

	MixingOutput _mixing_output;
	perf_counter_t _cycle_perf{nullptr};
	perf_counter_t _interval_perf{nullptr};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	bool _initialized{false};

	uORB::PublicationMulti<esc_status_s> _esc_feedback_pub{ORB_ID(esc_status)};
	esc_status_s _esc_feedback{};
	uint8_t _channels_count{0};
	uint8_t _responding_esc{0};

	uORB::Subscription _tune_control_sub{ORB_ID(tune_control)};
	hrt_abstime _interval_timestamp{0};
	unsigned int _silence_length{0};
	unsigned int _frequency{0};
	unsigned int _duration{0};
};
