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

#include "TFESCHUB.hpp"

#include <px4_platform_common/sem.hpp>

TFESCHUB::TFESCHUB(uint8_t channels_count) :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_mixing_output{"TFESCHUB", channels_count, *this, MixingOutput::SchedulingPolicy::Auto, true},
	_channels_count(channels_count)
{
	//strncpy(_device, device, sizeof(_device) - 1);
	//_device[sizeof(_device) - 1] = '\0';  // Fix in case of overflow

	_mixing_output.setAllFailsafeValues(0);
	_mixing_output.setAllDisarmedValues(0);
	_mixing_output.setAllMinValues(0);
	_mixing_output.setAllMaxValues(100);

}

TFESCHUB::~TFESCHUB()
{
	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}

int TFESCHUB::init()
{
	PX4_INFO("Initialize I2C device here.");
	return 0;
}

void TFESCHUB::send_esc_outputs(const uint16_t *pwm, const uint8_t motor_cnt)
{
	PX4_INFO("Send ESC outputs");
	for (uint8_t i = 0; i < motor_cnt; i++) {
		PX4_INFO("Target power for motor %u: %u", i, (unsigned)pwm[i]);
	}
	// Placeholder for I2C write
}

bool TFESCHUB::updateOutputs(bool stop_motors, uint16_t outputs[8], unsigned num_outputs,
			    unsigned num_control_groups_updated)
{
	PX4_INFO("Update outputs");
	if (_initialized) {
		uint16_t motor_out[8] {0};

		for (uint8_t i = 0; i < num_outputs; ++i) {
			motor_out[i] = outputs[i];
		}


		send_esc_outputs(motor_out, num_outputs);

		// Remove parsing real feedback, set dummy data
		for (uint8_t i = 0; i < num_outputs; i++) {
			_esc_feedback.esc[i].timestamp = hrt_absolute_time();
			_esc_feedback.esc[i].esc_rpm = motor_out[i] * 10; // example dummy speed
			_esc_feedback.esc[i].esc_voltage = 8.5f;
			_esc_feedback.esc[i].esc_current = motor_out[i] * 0.01f; // example dummy current
			_esc_feedback.esc[i].esc_state = 0; // arbitrary
		}
		_esc_feedback.esc_count = num_outputs;
		_esc_feedback.timestamp = hrt_absolute_time();
		_esc_feedback_pub.publish(_esc_feedback);

		PX4_INFO("Published esc feedback");
		PX4_INFO("RPM: %u, Voltage: %.2f, Current: %.2f", (unsigned)_esc_feedback.esc[0].esc_rpm,
			 (double)_esc_feedback.esc[0].esc_voltage, (double)_esc_feedback.esc[0].esc_current);

		return true;
	}

	return false;
}

void TFESCHUB::Run()
{
	if (should_exit()) {
		PX4_INFO("Should exit");
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	// push backup schedule
	ScheduleDelayed(20_ms);

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}

	if (!_initialized) {
		PX4_INFO("Not initialized");
		if (init() == PX4_OK) {
			_initialized = true;

		} else {
			PX4_ERR("init failed");
			exit_and_cleanup();
		}

	} else {
		_mixing_output.update();

		/* update output status if armed */
		bool outputs_on;
		outputs_on = _mixing_output.armed().armed;
		PX4_INFO("Outputs on: %d", outputs_on);

	}


	_mixing_output.updateSubscriptions(true);

	perf_end(_cycle_perf);
}

int TFESCHUB::task_spawn(int argc, char *argv[])
{
	TFESCHUB *instance = new TFESCHUB(4);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			instance->ScheduleNow();
			return PX4_OK;

		} else {
			instance->ScheduleClear();
			_object.store(nullptr);
			delete instance;
			return PX4_ERROR;
		}
	}

	return PX4_ERROR;
}

int TFESCHUB::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int TFESCHUB::print_usage(const char *reason)
{
	if (reason) {
		PX4_INFO("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for the TFESCHUB ESCs.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("TFESCHUB", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_INT('c', 4, 1, 8, "Number of channels", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int TFESCHUB::print_status()
{
	PX4_INFO("Running");
	PX4_INFO("Channels: %u", (unsigned)_channels_count);
	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);

	_mixing_output.printStatus();

	return 0;

}

extern "C" __EXPORT int tfeschub_main(int argc, char *argv[])
{
	return TFESCHUB::main(argc, argv);
}

