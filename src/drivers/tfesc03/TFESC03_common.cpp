#include <px4_log.h>
#include <px4_defines.h>
#include <cmath>

#include "TFESC03_common.hpp"

TFESC03_COMMON::TFESC03_COMMON(int bus, int addr):
	I2C(DRV_ESC_DEVTYPE_TFESC03, MODULE_NAME, 1, 0x01, 100000)
{
	// Constructor implementation
}

int TFESC03_COMMON::init()
{

	PX4_INFO("Starting TFESC03 initialization");

	int ret = I2C::init();

	if (ret != PX4_OK) { return ret; }

	PX4_INFO("Initialize I2C device here.");
	PX4_INFO("I2C device initialized.");

	// if (OK != ret) {
	// 	PX4_ERR("init: i2c::transfer returned %d", ret);
	// 	return ret;
	// }

	uint8_t control_word[3] = {0x00, 0x00, 0xA5}; // RAM read
	uint8_t tx_buf[4];
	memcpy(tx_buf, control_word, 3);
	tx_buf[3] = calc_crc(control_word, 3); // Přidej CRC

	uint8_t rx_buf[4] = {0}; // 3 bajty dat + 1 CRC

	ret = transfer(tx_buf, sizeof(tx_buf), rx_buf, sizeof(rx_buf));

	if (ret == PX4_OK) {
		uint8_t data_crc = calc_crc(rx_buf, 3);

		if (data_crc == rx_buf[3]) {
			uint32_t value = ((uint32_t)rx_buf[0] << 16) |
					((uint32_t)rx_buf[1] << 8) |
					((uint32_t)rx_buf[2]);

			PX4_INFO("Read OK, value = 0x%06lx", value);
		} else {
			PX4_ERR("CRC error on received data");
		}
	} else {
		PX4_ERR("I2C transfer failed");
	}



	PX4_INFO("TFESC03 initialized.");

	return PX4_OK;  // Removed extra closing brace
}

int TFESC03_COMMON::read_reg_with_crc(uint8_t reg_addr, uint32_t &value)
{
	// Control word: [23:16] = 0x00 (RAM read), [15:0] = reg_addr
	uint8_t control_word[3] = {0x00, 0x00, reg_addr};
	uint8_t tx_buf[4]; // control_word + CRC
	memcpy(tx_buf, control_word, 3);
	tx_buf[3] = calc_crc(control_word, 3);

	uint8_t rx_buf[4] = {0}; // 3 bytes of data + CRC

	int ret = transfer(tx_buf, sizeof(tx_buf), rx_buf, sizeof(rx_buf));

	if (ret != PX4_OK) {
		return ret;
	}

	// Verify CRC on received data
	uint8_t received_crc = rx_buf[3];
	uint8_t computed_crc = calc_crc(rx_buf, 3);

	if (received_crc != computed_crc) {
		PX4_ERR("CRC mismatch: received 0x%02x, expected 0x%02x", received_crc, computed_crc);
		return -EIO;
	}

	// Decode 24-bit value
	value = ((uint32_t)rx_buf[0] << 16) |
	        ((uint32_t)rx_buf[1] << 8) |
	        ((uint32_t)rx_buf[2]);

	return PX4_OK;
}

int TFESC03_COMMON::write_reg_with_crc(uint8_t reg_addr, uint32_t value)
{
	// Control word: [23:16] = 0x00 (RAM write), [15:0] = reg_addr
	uint8_t control_word[3] = {0x00, 0x00, reg_addr};
	uint8_t control_crc = calc_crc(control_word, 3);

	// Data: 3 bytes value (MSB first)
	uint8_t data[3] = {
		static_cast<uint8_t>((value >> 16) & 0xFF),
		static_cast<uint8_t>((value >> 8) & 0xFF),
		static_cast<uint8_t>(value & 0xFF)
	};

	uint8_t data_crc = calc_crc(data, 3);

	// Combine everything into one buffer
	uint8_t tx_buf[8];
	memcpy(&tx_buf[0], control_word, 3);
	tx_buf[3] = control_crc;
	memcpy(&tx_buf[4], data, 3);
	tx_buf[7] = data_crc;

	// Send
	return transfer(tx_buf, sizeof(tx_buf), nullptr, 0);
}


int TFESC03_COMMON::setMotorSpeed(uint8_t instance, uint16_t speed)
{
	//PX4_INFO("Set motor %u speed to %u", instance, speed);
	// Placeholder for I2C write


	uint8_t control_word[3] = {0x00, 0x00, 0xA5}; // RAM read
	uint8_t tx_buf[4];
	memcpy(tx_buf, control_word, 3);
	tx_buf[3] = calc_crc(control_word, 3); // Přidej CRC

	uint8_t rx_buf[4] = {0}; // 3 bajty dat + 1 CRC

	int ret = transfer(tx_buf, sizeof(tx_buf), rx_buf, sizeof(rx_buf));

	if (ret == PX4_OK) {
		uint8_t data_crc = calc_crc(rx_buf, 3);

		if (data_crc == rx_buf[3]) {
			uint32_t value = ((uint32_t)rx_buf[0] << 16) |
					((uint32_t)rx_buf[1] << 8) |
					((uint32_t)rx_buf[2]);

			PX4_INFO("Read OK, value = 0x%06lx", value);
		} else {
			PX4_ERR("CRC error on received data");
		}
	} else {
		PX4_ERR("I2C transfer failed");
	}



	return PX4_OK;
}

uint8_t TFESC03_COMMON::calc_crc(uint8_t *data, uint8_t len)
{
	uint8_t crc = 0xFF;

	for (uint8_t i = 0; i < len; i++) {
		crc ^= data[i];

		for (uint8_t j = 0; j < 8; j++) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ 0x07;
			} else {
				crc <<= 1;
			}
		}
	}

	return crc;
}
