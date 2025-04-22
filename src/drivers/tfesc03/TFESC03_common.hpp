#pragma once

#include <drivers/device/i2c.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <stdint.h>

#include <px4_platform_common/module.h>


#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

// Module name
//#define MODULE_NAME "TFESC03_COMMON"

class TFESC03_COMMON : public device::I2C, public ModuleParams, public I2CSPIDriver<TFESC03_COMMON>
{
public:
	TFESC03_COMMON(const I2CSPIDriverConfig &config);
	~TFESC03_COMMON() override = default;

	int init();
	int read_reg_with_crc(uint8_t reg_addr, uint32_t &value);
	int write_reg_with_crc(uint8_t reg_addr, uint32_t value);

	int setMotorSpeed(uint8_t instance, uint16_t speed);

	void RunImpl();

	uint32_t GetStatus();

private:
	static uint8_t calc_crc(uint8_t *data, uint8_t len);

	static void build_control(uint16_t reg, uint8_t buf[3]);
	static uint32_t read32(int i2c_dev, uint16_t reg);

	static constexpr uint8_t MCF_ADDR         = 0x01;
	static constexpr uint16_t VM_REG_ADDR     = 0x0478;   // VM_VOLTAGE :contentReference[oaicite:0]{index=0}
	static constexpr uint16_t GATE_FAULT_ADDR = 0x00E0;   // Gate Driver Fault Status :contentReference[oaicite:1]{index=1}
	static constexpr uint16_t CTRL_FAULT_ADDR = 0x00E2;   // Controller Fault Status :contentReference[oaicite:2]{index=2}

};
