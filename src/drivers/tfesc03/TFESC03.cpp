#include "TFESC03.hpp"

#include <px4_platform_common/sem.hpp>

TFESC03::TFESC03(I2CSPIBusOption bus_option, int bus, int addr, uint8_t channels_count) :
	OutputModuleInterface(MODULE_NAME, px4::i2c_spi_to_wq(bus_option, bus)),
	_mixing_output{"TFESC03", channels_count, *this, MixingOutput::SchedulingPolicy::Auto, true},
	_channels_count(channels_count)
{
	_mixing_output.setAllFailsafeValues(0);
	_mixing_output.setAllDisarmedValues(0);
	_mixing_output.setAllMinValues(0);
	_mixing_output.setAllMaxValues(100);

	// Create the I2C driver instance
	I2CSPIDriverConfig config{};
	config.bus = bus;
	config.i2c_address = addr;
	config.bus_frequency = 100000;
	config.spi_devid = 0;
	config.drdy_gpio = 0;
	config.spi_mode = 0;
	config.bus_option = bus_option;
	config.keep_running = true;

	_tfesc03_common = new TFESC03_COMMON(config);
}

TFESC03::~TFESC03()
{
	perf_free(_cycle_perf);
	perf_free(_interval_perf);

	if (_tfesc03_common != nullptr) {
		delete _tfesc03_common;
	}
}

int TFESC03::init()
{
	if (_tfesc03_common == nullptr) {
		PX4_ERR("TFESC03_common driver is null");
		return PX4_ERROR;
	}

	return _tfesc03_common->init();
}

void TFESC03::send_esc_outputs(const uint16_t *pwm, const uint8_t motor_cnt)
{
	for (uint8_t i = 0; i < motor_cnt; i++) {
		_tfesc03_common->setMotorSpeed(i, pwm[i]);
	}
}

bool TFESC03::updateOutputs(bool stop_motors, uint16_t outputs[8], unsigned num_outputs,
			    unsigned num_control_groups_updated)
{
	if (_initialized) {
		uint16_t motor_out[8] {0};

		for (uint8_t i = 0; i < num_outputs; ++i) {
			motor_out[i] = outputs[i];
		}

		send_esc_outputs(motor_out, num_outputs);

		// Create dummy feedback data
		for (uint8_t i = 0; i < num_outputs; i++) {
			_esc_feedback.esc[i].timestamp = hrt_absolute_time();
			_esc_feedback.esc[i].esc_rpm = motor_out[i] * 10;
			_esc_feedback.esc[i].esc_voltage = 8.5f;
			_esc_feedback.esc[i].esc_current = motor_out[i] * 0.01f;
			_esc_feedback.esc[i].esc_state = 0;
		}
		_esc_feedback.esc_count = num_outputs;
		_esc_feedback.timestamp = hrt_absolute_time();
		_esc_feedback_pub.publish(_esc_feedback);

		return true;
	}

	return false;
}

void TFESC03::Run()
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
		if (init() == PX4_OK) {
			_initialized = true;
		} else {
			PX4_ERR("init failed");
			exit_and_cleanup();
		}
	} else {
		_mixing_output.update();
	}

	_mixing_output.updateSubscriptions(true);

	perf_end(_cycle_perf);
}

int TFESC03::task_spawn(int argc, char *argv[])
{
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	cli.i2c_address = 0x01;
	cli.support_keep_running = true;
	cli.default_channels = 4;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		TFESC03::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_ESC_DEVTYPE_TFESC03);

	if (!strcmp(verb, "start")) {
		// Loop through all instances
		while (iterator.next()) {
			TFESC03 *instance = new TFESC03(
				iterator.busType(), iterator.bus(), iterator.address(), cli.channels);

			if (instance == nullptr) {
				PX4_ERR("alloc failed");
				return PX4_ERROR;
			}

			if (OK != instance->init()) {
				delete instance;
				continue;
			}

			_object.store(instance);
			_task_id = task_id_is_work_queue;
			instance->ScheduleNow();
			return PX4_OK;
		}

		return PX4_ERROR;
	}

	return print_usage();
}

int TFESC03::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int TFESC03::print_usage(const char *reason)
{
	if (reason) {
		PX4_INFO("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for the TFESC03 ESCs.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tfesc03", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_INT('c', 4, 1, 8, "Number of channels", false);
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int TFESC03::print_status()
{
	PX4_INFO("Running");
	PX4_INFO("Channels: %u", (unsigned)_channels_count);
	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);

	_mixing_output.printStatus();

	if (_tfesc03_common) {
		_tfesc03_common->GetStatus();
	}

	return 0;
}

extern "C" __EXPORT int tfesc03_main(int argc, char *argv[])
{
	return TFESC03::main(argc, argv);
}

