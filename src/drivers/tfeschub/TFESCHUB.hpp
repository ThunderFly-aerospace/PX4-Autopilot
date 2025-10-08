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





#if !defined(BOARD_TAP_ESC_MODE)
#  define BOARD_TAP_ESC_MODE 0
#endif

#if !defined(DEVICE_ARGUMENT_MAX_LENGTH)
#  define DEVICE_ARGUMENT_MAX_LENGTH 32
#endif

#define TAP_ESC_MAX_MOTOR_NUM 4
#define RPMSTOPPED 0
#define ESC_POS {0, 1, 2, 3}
#define ESC_DIR {0, 0, 0, 0}
#define ESC_MAX_RPM 1900
#define ESC_MIN_RPM 1200


using namespace time_literals;

#define MODULE_NAME "tfeschub"

class TFESCHUB : public ModuleBase<TFESCHUB>, public OutputModuleInterface
{
public:
	TFESCHUB(uint8_t channels_count);
	virtual ~TFESCHUB();

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

	int init();
	bool updateOutputs(bool stop_motors, uint16_t outputs[8],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

private:
	void Run() override;

	MixingOutput		_mixing_output;
	perf_counter_t		_cycle_perf{nullptr};
	perf_counter_t		_interval_perf{nullptr};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};



	bool _initialized{false};

	uORB::PublicationMulti<esc_status_s> _esc_feedback_pub{ORB_ID(esc_status)};
	esc_status_s      _esc_feedback{};
	uORB::Publication<tf_esc_set_s> _tf_esc_set_pub{ORB_ID(tf_esc_set)};
  tf_esc_set_s      _tf_esc_set{};

	uint8_t    	  _channels_count{0}; 		///< number of ESC channels
	uint8_t 	  _responding_esc{0};


	Tunes _tunes{};
	uORB::Subscription _tune_control_sub{ORB_ID(tune_control)};
	hrt_abstime _interval_timestamp{0};
	unsigned int _silence_length{0};	///< If nonzero, silence before next note.
	unsigned int _frequency{0};
	unsigned int _duration{0};

	//LedControlData _led_control_data{};
	//LedController  _led_controller{};
	//perf_counter_t _interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
};
