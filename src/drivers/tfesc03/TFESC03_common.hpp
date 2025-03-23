#pragma once

#include <drivers/device/i2c.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <stdint.h>

// Module name
//#define MODULE_NAME "TFESC03_COMMON"

class TFESC03_COMMON : public device::I2C
{
public:
	TFESC03_COMMON(int bus, int addr);
	~TFESC03_COMMON() = default;

	int init();
	int read_reg_with_crc(uint8_t reg_addr, uint32_t &value);
	int write_reg_with_crc(uint8_t reg_addr, uint32_t value);

	int setMotorSpeed(uint8_t instance, uint16_t speed);


private:
	static uint8_t calc_crc(uint8_t *data, uint8_t len);
};
