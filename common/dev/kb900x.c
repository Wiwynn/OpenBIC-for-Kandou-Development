/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <sys/crc.h>
#include <logging/log.h>
#include <stdio.h>
#include "libutil.h"
#include "kb900x.h"

LOG_MODULE_REGISTER(dev_kb900x, LOG_LEVEL_DBG);
K_MUTEX_DEFINE(kb900x_mutex);

kb900x_error_t kb900x_write_register(I2C_MSG *msg, uint32_t address, uint32_t value);
kb900x_error_t kb900x_read_register(I2C_MSG *msg, uint32_t address, uint32_t *value);
kb900x_error_t kb900x_write_field(I2C_MSG *msg, uint32_t addr, uint8_t field_width,
				  uint8_t field_lsb, uint32_t value);
kb900x_error_t kb900x_read_field(I2C_MSG *msg, uint32_t addr, uint8_t field_width,
				 uint8_t field_lsb, uint32_t *value);

kb900x_error_t twi_read_register(I2C_MSG *msg, uint32_t address, uint32_t *value);
kb900x_error_t twi_write_register(I2C_MSG *msg, uint32_t address, uint32_t value);

KB900X_REGISTER_IO kb900x_register_io = {
	.write = twi_write_register,
	.read = twi_read_register,
};

static uint8_t cal_crc8_pec(uint8_t *data, uint8_t len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, KB900X_E_INVALID_ARG);
	return crc8(data, len, 0x07, 0x00, false);
}

static bool verify_crc8_pec(I2C_MSG *msg, uint8_t command_code)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);

	uint8_t pec_crc, crc_result;
	uint8_t bytecnt = msg->data[0];
	uint8_t crc_list[bytecnt + 4];
	crc_list[0] = msg->target_addr << 1;
	crc_list[1] = command_code;
	crc_list[2] = (msg->target_addr << 1) + 1;
	memcpy(&(crc_list[3]), msg->data, bytecnt + 1);

	pec_crc = msg->data[bytecnt + 1]; // pec value

	crc_result = cal_crc8_pec(crc_list, 3 + bytecnt + 1);

	if (pec_crc != crc_result) {
		LOG_ERR("The read data pec_crc=0x%x is invalid, the right crc value=0x%x", pec_crc,
			crc_result);
		return false;
	}
	return true;
}

kb900x_error_t smbus_read_command(I2C_MSG *msg, uint16_t offsets)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, KB900X_E_INVALID_ARG);

	uint8_t retry = 0;
	kb900x_error_t ret = KB900X_E_OK;

	// Lock mutex
	for (; retry < KB900X_MAX_RETRY; retry++) {
		if (k_mutex_lock(&kb900x_mutex, K_MSEC(KB900X_MUTEX_LOCK_MS))) {
			k_msleep(10);
		} else {
			break;
		}
	}
	if (retry == KB900X_MAX_RETRY) {
		LOG_ERR("kb900x mutex lock failed");
		return KB900X_E_MUX_LOCK_FAILED;
	}

	msg->data[0] = KB900X_CCODE_START_READ_FUNC0; // COMMAND CODE
	msg->data[1] = KB900X_I2C_WRITE_BYTCNT; // byte count
	msg->data[2] = (uint8_t)(offsets & 0xFF); // lower offset
	msg->data[3] = (uint8_t)(offsets >> 8); // upper offset
	msg->tx_len = 5;

	// PEC signature
	uint8_t crc_list[msg->tx_len];
	crc_list[0] = msg->target_addr << 1;
	memcpy(&(crc_list[1]), msg->data, msg->tx_len - 1);
	msg->data[msg->tx_len - 1] = cal_crc8_pec(crc_list, msg->tx_len);

	// Write (Prepare read)
	if (i2c_master_write(msg, retry)) {
		LOG_ERR("Failed to write, 0x%X not set", offsets);
		ret = KB900X_E_I2C_ERROR;
		goto exit;
	}

	// Read
	memset(msg->data, 0, I2C_BUFF_SIZE);
	msg->tx_len = 1;
	msg->rx_len = 8;
	retry = 0;
	do {
		retry++;
		msg->data[0] = KB900X_CCODE_END_READ_FUNC0;
		if (i2c_master_read(msg, KB900X_MAX_RETRY)) {
			LOG_ERR("Failed to read PCIE RETIMER addr 0x%X", offsets);
			ret = KB900X_E_I2C_ERROR;
			goto exit;
		}
	} while (retry < KB900X_MAX_RETRY && !verify_crc8_pec(msg, KB900X_CCODE_END_READ_FUNC0));

	// PEC validation
	if (!verify_crc8_pec(msg, KB900X_CCODE_END_READ_FUNC0)) {
		ret = KB900X_E_CRC_ERROR;
		goto exit;
	}

exit:
	if (k_mutex_unlock(&kb900x_mutex)) {
		LOG_ERR("kb900x mutex unlock failed");
		ret = KB900X_E_MUX_UNLOCK_FAILED;
	}
	return ret;
}

/**
 * \brief Read a 32-bit register from KB900X using TWI
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[in] address the address of the register to read
 * \param[out] value a pointer to the uint32_t used to store the result
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t twi_read_register(I2C_MSG *msg, uint32_t address, uint32_t *value)
{
	CHECK_NULL_ARG_WITH_RETURN(value, KB900X_E_INVALID_ARG);
	CHECK_NULL_ARG_WITH_RETURN(msg, KB900X_E_INVALID_ARG);

	uint8_t retry = 0;
	kb900x_error_t ret = KB900X_E_OK;

	// Lock mutex
	for (; retry < KB900X_MAX_RETRY; retry++) {
		if (k_mutex_lock(&kb900x_mutex, K_MSEC(KB900X_MUTEX_LOCK_MS))) {
			k_msleep(10);
		} else {
			break;
		}
	}
	if (retry == KB900X_MAX_RETRY) {
		LOG_ERR("kb900x mutex lock failed");
		return KB900X_E_MUX_LOCK_FAILED;
	}

	// As we don't care about the APB and tile number (only accessing tile0)
	const uint32_t mask = 0x0CFFFFFF;
	address = address & mask;

	for (int i = 0; i < KB900X_REGISTER_ADDRESS_WIDTH; i++) {
		msg->data[i] =
			(uint8_t)((address >> (KB900X_REGISTER_ADDRESS_WIDTH - i - 1) * 8) & 0xFF);
	}
	msg->tx_len = KB900X_REGISTER_ADDRESS_WIDTH;

	// Write (Prepare read)
	if (i2c_master_write(msg, retry)) {
		LOG_ERR("Failed to write, 0x%X not set", address);
		ret = KB900X_E_I2C_ERROR;
		goto exit;
	}

	// Read
	memset(msg->data, 0, I2C_BUFF_SIZE);
	msg->tx_len = 0;
	msg->rx_len = KB900X_REGISTER_VALUE_WIDTH;
	if (i2c_master_read(msg, KB900X_MAX_RETRY)) {
		LOG_ERR("Failed to read PCIE RETIMER addr 0x%X", address);
		ret = KB900X_E_I2C_ERROR;
		goto exit;
	}

	*value = 0;
	for (int i = 0; i < KB900X_REGISTER_VALUE_WIDTH; i++) {
		*value |= ((msg->data[i]) << (((KB900X_REGISTER_VALUE_WIDTH - i) - 1) * 8));
	}

exit:
	if (k_mutex_unlock(&kb900x_mutex)) {
		LOG_ERR("kb900x mutex unlock failed");
		ret = KB900X_E_MUX_UNLOCK_FAILED;
	}
	return ret;
}

/**
 * \brief Write a 32-bit register to KB900X using TWI
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[in] address the address of the register to read
 * \param[out] value the value to write
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t twi_write_register(I2C_MSG *msg, uint32_t address, uint32_t value)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, KB900X_E_INVALID_ARG);

	uint8_t retry = 0;
	kb900x_error_t ret = KB900X_E_OK;

	// Lock mutex
	for (; retry < KB900X_MAX_RETRY; retry++) {
		if (k_mutex_lock(&kb900x_mutex, K_MSEC(KB900X_MUTEX_LOCK_MS))) {
			k_msleep(10);
		} else {
			break;
		}
	}
	if (retry == KB900X_MAX_RETRY) {
		LOG_ERR("kb900x mutex lock failed");
		return KB900X_E_MUX_LOCK_FAILED;
	}

	// As we don't care about the APB and tile number (only accessing tile0)
	const uint32_t mask = 0x0CFFFFFF;
	address = address & mask;

	for (int i = 0; i < KB900X_REGISTER_ADDRESS_WIDTH; i++) {
		msg->data[i] =
			(uint8_t)((address >> (KB900X_REGISTER_ADDRESS_WIDTH - i - 1) * 8) & 0xFF);
	}
	for (int i = 0; i < KB900X_REGISTER_VALUE_WIDTH; i++) {
		msg->data[i + KB900X_REGISTER_ADDRESS_WIDTH] =
			(uint8_t)((value >> (KB900X_REGISTER_ADDRESS_WIDTH - i - 1) * 8) & 0xFF);
	}
	msg->tx_len = KB900X_REGISTER_ADDRESS_WIDTH + KB900X_REGISTER_VALUE_WIDTH;

	// Write
	if (i2c_master_write(msg, retry)) {
		LOG_ERR("Failed to write, 0x%X not set", address);
		ret = KB900X_E_I2C_ERROR;
		goto exit;
	}

exit:
	if (k_mutex_unlock(&kb900x_mutex)) {
		LOG_ERR("kb900x mutex unlock failed");
		ret = KB900X_E_MUX_UNLOCK_FAILED;
	}
	return ret;
}

/**
 * \brief Read a 32-bit register from KB900X using SMBus
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[in] address the address of the register to read
 * \param[out] value a pointer to the uint32_t used to store the result
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t smbus_read_register(I2C_MSG *msg, uint32_t address, uint32_t *value)
{
	CHECK_NULL_ARG_WITH_RETURN(value, KB900X_E_INVALID_ARG);
	CHECK_NULL_ARG_WITH_RETURN(msg, KB900X_E_INVALID_ARG);

	uint8_t retry = 0;
	kb900x_error_t ret = KB900X_E_OK;

	// Lock mutex
	for (; retry < KB900X_MAX_RETRY; retry++) {
		if (k_mutex_lock(&kb900x_mutex, K_MSEC(KB900X_MUTEX_LOCK_MS))) {
			k_msleep(10);
		} else {
			break;
		}
	}
	if (retry == KB900X_MAX_RETRY) {
		LOG_ERR("kb900x mutex lock failed");
		return KB900X_E_MUX_LOCK_FAILED;
	}

	uint8_t payload_offset = 0;
	// Command code
	msg->data[payload_offset++] = KB900X_CCODE_START_READ_FUNC2;
	// Bytecnt
	msg->data[payload_offset++] = KB900X_REGISTER_ADDRESS_WIDTH;
	for (int i = 0; i < KB900X_REGISTER_ADDRESS_WIDTH; i++) {
		msg->data[payload_offset + i] = (address >> (i * 8)) & 0xFF;
	}
	payload_offset += KB900X_REGISTER_ADDRESS_WIDTH;
	msg->tx_len = payload_offset + 1; // + PEC

	// PEC signature
	uint8_t crc_list[msg->tx_len];
	crc_list[0] = msg->target_addr << 1;
	memcpy(&(crc_list[1]), msg->data, msg->tx_len - 1);
	msg->data[msg->tx_len - 1] = cal_crc8_pec(crc_list, msg->tx_len);

	// Write (Prepare read)
	if (i2c_master_write(msg, retry)) {
		LOG_ERR("Failed to write, 0x%X not set", address);
		ret = KB900X_E_I2C_ERROR;
		goto exit;
	}

	// Read
	memset(msg->data, 0, I2C_BUFF_SIZE);
	msg->tx_len = 1;
	msg->rx_len = 1 + KB900X_REGISTER_ADDRESS_WIDTH + KB900X_REGISTER_VALUE_WIDTH +
		      1; // Bytecnt + address + value + PEC
	retry = 0;
	do {
		retry++;
		msg->data[0] = KB900X_CCODE_END_READ_FUNC2;
		if (i2c_master_read(msg, KB900X_MAX_RETRY)) {
			LOG_ERR("Failed to read PCIE RETIMER addr 0x%X", address);
			ret = KB900X_E_I2C_ERROR;
			goto exit;
		}
	} while (retry < KB900X_MAX_RETRY && !verify_crc8_pec(msg, KB900X_CCODE_END_READ_FUNC2));

	// PEC validation
	if (!verify_crc8_pec(msg, KB900X_CCODE_END_READ_FUNC2)) {
		ret = KB900X_E_CRC_ERROR;
		goto exit;
	}

	*value = 0;
	for (int i = 0; i < KB900X_REGISTER_VALUE_WIDTH; i++) {
		*value |= msg->data[i + 1 + 4] << (i * 8);
	}

exit:
	if (k_mutex_unlock(&kb900x_mutex)) {
		LOG_ERR("kb900x mutex unlock failed");
		ret = KB900X_E_MUX_UNLOCK_FAILED;
	}
	return ret;
}

/**
 * \brief Write a 32-bit register to KB900X using SMBus
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[in] address the address of the register to read
 * \param[out] value the value to write
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t smbus_write_register(I2C_MSG *msg, uint32_t address, uint32_t value)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, KB900X_E_INVALID_ARG);

	uint8_t retry = 0;
	kb900x_error_t ret = KB900X_E_OK;

	// Lock mutex
	for (; retry < KB900X_MAX_RETRY; retry++) {
		if (k_mutex_lock(&kb900x_mutex, K_MSEC(KB900X_MUTEX_LOCK_MS))) {
			k_msleep(10);
		} else {
			break;
		}
	}
	if (retry == KB900X_MAX_RETRY) {
		LOG_ERR("kb900x mutex lock failed");
		return KB900X_E_MUX_LOCK_FAILED;
	}

	uint8_t payload_offset = 0;
	// Command code
	msg->data[payload_offset++] = KB900X_CCODE_START_END_WRITE_FUNC3;
	// Bytecnt
	msg->data[payload_offset++] = KB900X_REGISTER_VALUE_WIDTH + KB900X_REGISTER_ADDRESS_WIDTH;
	for (int i = 0; i < KB900X_REGISTER_ADDRESS_WIDTH; i++) {
		msg->data[payload_offset + i] = (address >> (i * 8)) & 0xFF;
	}
	payload_offset += KB900X_REGISTER_ADDRESS_WIDTH;
	for (int i = 0; i < KB900X_REGISTER_VALUE_WIDTH; i++) {
		msg->data[payload_offset + i] = (value >> (i * 8)) & 0xFF;
	}
	payload_offset += KB900X_REGISTER_VALUE_WIDTH;
	msg->tx_len = payload_offset + 1; // + PEC

	// PEC signature
	uint8_t crc_list[msg->tx_len];
	crc_list[0] = msg->target_addr << 1;
	memcpy(&(crc_list[1]), msg->data, msg->tx_len - 1);
	msg->data[msg->tx_len - 1] = cal_crc8_pec(crc_list, msg->tx_len);

	// Write
	if (i2c_master_write(msg, retry)) {
		LOG_ERR("Failed to write, 0x%X not set", address);
		ret = KB900X_E_I2C_ERROR;
		goto exit;
	}

exit:
	if (k_mutex_unlock(&kb900x_mutex)) {
		LOG_ERR("kb900x mutex unlock failed");
		ret = KB900X_E_MUX_UNLOCK_FAILED;
	}
	return ret;
}

/**
 * \brief Read a 32-bit register from KB900X
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[in] address the address of the register to read
 * \param[out] value a pointer to the uint32_t used to store the result
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_read_register(I2C_MSG *msg, uint32_t address, uint32_t *value)
{
	return kb900x_register_io.read(msg, address, value);
}

/**
 * \brief Write a 32-bit register to KB900X
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[in] address the address of the register to read
 * \param[out] value the value to write
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_write_register(I2C_MSG *msg, uint32_t address, uint32_t value)
{
	return kb900x_register_io.write(msg, address, value);
}

kb900x_error_t kb900x_get_vendor_id_with_err_code(I2C_MSG *msg, int *vendor_id)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, KB900X_E_INVALID_ARG);
	CHECK_NULL_ARG_WITH_RETURN(vendor_id, KB900X_E_INVALID_ARG);

	const kb900x_error_t status = smbus_read_command(msg, KB900X_SMBUS_OFFSET_GLOB_PARAM_REG_1);
	if (status == KB900X_E_OK) {
		uint8_t bytecnt = msg->data[0];
		*vendor_id = msg->data[bytecnt - 3] + (msg->data[bytecnt - 2] << 8) +
			     (msg->data[bytecnt - 1] << 16) + (msg->data[bytecnt] << 24);
	}
	return status;
}

bool kb900x_get_vendor_id(I2C_MSG *msg, int *vendor_id)
{
	const kb900x_error_t status = kb900x_get_vendor_id_with_err_code(msg, vendor_id);
	return status == KB900X_E_OK;
}

kb900x_error_t kb900x_get_fw_version_with_err_code(I2C_MSG *msg, uint8_t *version)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, KB900X_E_INVALID_ARG);
	CHECK_NULL_ARG_WITH_RETURN(version, KB900X_E_INVALID_ARG);

	const kb900x_error_t status = smbus_read_command(msg, KB900X_SMBUS_OFFSET_FW_VERSION);
	if (status == KB900X_E_OK) {
		// Parse response
		uint8_t bytecnt = msg->data[0];
		for (uint8_t i = 0; i < (bytecnt - 2); i++) {
			version[i] = msg->data[bytecnt - i];
		}
	}

	return status;
}

bool kb900x_get_fw_version(I2C_MSG *msg, uint8_t *version)
{
	const kb900x_error_t status = kb900x_get_fw_version_with_err_code(msg, version);
	return status == KB900X_E_OK;
}

kb900x_error_t kb900x_get_firmware_health(I2C_MSG *msg, kb900x_fw_health_t *firmware_health)
{
	LOG_ERR("Not Yet Implemented");
	return KB900X_E_NOT_IMPLEMENTED;
}

kb900x_error_t kb900x_get_rtssm_dump(I2C_MSG *msg, uint32_t *buffer)
{
	LOG_ERR("Not Yet Implemented");
	return KB900X_E_NOT_IMPLEMENTED;
}

kb900x_error_t kb900x_get_link_status(I2C_MSG *msg, int link_id, kb900x_link_status_t *link_status)
{
	LOG_ERR("Not Yet Implemented");
	return KB900X_E_NOT_IMPLEMENTED;
}

kb900x_error_t kb900x_get_temperature(I2C_MSG *msg, float *temperature)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, KB900X_E_INVALID_ARG);
	CHECK_NULL_ARG_WITH_RETURN(temperature, KB900X_E_INVALID_ARG);

	*temperature = ABSOLUTE_ZER0;

	for (int port = KB900X_SIDE_A; port <= KB900X_SIDE_B; ++port) {
		// FIXME: if the number of lanes changes, we will need to implement a way to specify
		// which version of KB900X we are running on
		for (int lane = 0; lane < KB9003_NUM_LANES; ++lane) {
			float lane_temp;
			kb900x_error_t ret =
				kb900x_get_lane_temperature(msg, port, lane, &lane_temp);
			if (ret != KB900X_E_OK) {
				return ret;
			}

			if (lane_temp > *temperature) {
				*temperature = lane_temp;
			}
		}
	}

	return KB900X_E_OK;
}

kb900x_error_t kb900x_get_lane_temperature(I2C_MSG *msg, int port, int lane, float *temperature)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, KB900X_E_INVALID_ARG);
	CHECK_NULL_ARG_WITH_RETURN(temperature, KB900X_E_INVALID_ARG);

	if (port != KB900X_SIDE_A && port != KB900X_SIDE_B) {
		LOG_ERR("Invalid port: %d", port);
		return KB900X_E_INVALID_ARG;
	}
	// FIXME: if the number of lanes changes, we will need to implement a way to specify
	// which version of KB900X we are running on
	if (lane < 0 || lane >= KB9003_NUM_LANES) {
		LOG_ERR("Invalid lane: %d", lane);
		return KB900X_E_INVALID_ARG;
	}

	// Compute port and lane-specific offset
	uint16_t smbus_offset = KB900X_SMBUS_OFFSET_TEMPERATURE;
	smbus_offset &= 0xFF00;
	smbus_offset |= (port == KB900X_SIDE_A) ? (0x10) : (0x30);
	smbus_offset += (lane / 2) * 4;

	kb900x_error_t ret = smbus_read_command(msg, smbus_offset);
	if (ret != KB900X_E_OK) {
		LOG_ERR("Failed to read temperature of port %d lane %d: %d", port, lane, ret);
		return ret;
	}

	// Parse response
	const uint8_t bytecnt = msg->data[0];
	const float divider = ((float)(1 << KB900X_FLOAT_PRECISION));
	const uint32_t raw_value_u32 =
		(msg->data[bytecnt - 3] + (msg->data[bytecnt - 2] << 8) +
		 (msg->data[bytecnt - 1] << 16) + (msg->data[bytecnt] << 24));
	*temperature = ABSOLUTE_ZER0 + ((float)(raw_value_u32)) / divider;
	return KB900X_E_OK;
}

/************** I2C MASTER SECTION START **************/

K_MUTEX_DEFINE(kb900x_i2c_master_mutex);

uint8_t kb900x_eeprom_slave_addr = 0x00;

kb900x_error_t kb900x_i2c_master_write(I2C_MSG *msg, uint8_t slave_addr, uint8_t *data,
				       size_t data_size, bool check);
kb900x_error_t kb900x_i2c_master_read(I2C_MSG *msg, uint8_t slave_addr, uint8_t *addr,
				      uint8_t addr_size, size_t length, uint8_t *result, bool check,
				      bool skip_addr);
kb900x_error_t kb900x_i2c_master_set_slave_address(I2C_MSG *msg, uint8_t slave_address);
kb900x_error_t kb900x_i2c_master_enable(I2C_MSG *msg, bool enable, bool block_fifo);
kb900x_error_t kb900x_i2c_master_wait_for_inactivity(I2C_MSG *msg);
kb900x_error_t kb900x_i2c_master_check_status(I2C_MSG *msg);

kb900x_error_t kb900x_i2c_master_init(I2C_MSG *msg, uint8_t slave_addr)
{
	uint8_t retry = 0;
	int ret = KB900X_E_OK;

	// Lock mutex
	for (; retry < KB900X_MAX_RETRY; retry++) {
		if (k_mutex_lock(&kb900x_i2c_master_mutex, K_MSEC(KB900X_MUTEX_LOCK_MS))) {
			k_msleep(10);
		} else {
			break;
		}
	}
	if (retry == KB900X_MAX_RETRY) {
		LOG_ERR("kb900x i2c master mutex lock failed");
		return KB900X_E_MUX_LOCK_FAILED;
	}

	// Enable clock for I2C Master
	uint32_t payload = 0x1F;
	ret = kb900x_register_io.write(msg, kb900x_cpu_periph_clk_gate_en, payload);
	if (ret != KB900X_E_OK) {
		LOG_ERR("Something went wrong while enabling the I2C master clock");
		goto exit;
	}
	// Disable before configuring
	ret = kb900x_i2c_master_enable(msg, false, false);
	if (ret != KB900X_E_OK) {
		LOG_ERR("Something went wrong while disalbing the I2C master");
		goto exit;
	}
	payload = 0x63;
	ret = kb900x_register_io.write(msg, kb900x_ee_IC_CON, payload);
	if (ret != KB900X_E_OK) {
		LOG_ERR("Something went wrong while configuring the I2C master");
		goto exit;
	}

	// Clear all interrupts to not disrupt FW
	payload = 0;
	ret = kb900x_register_io.write(msg, kb900x_ee_IC_INTR_MASK, payload);
	if (ret != KB900X_E_OK) {
		LOG_ERR("Something went wrong while clearing the interrupts");
		goto exit;
	}

	// Set slave address
	ret = kb900x_i2c_master_set_slave_address(msg, slave_addr);
	if (ret != KB900X_E_OK) {
		LOG_ERR("Something went wrong while setting the slave address");
		goto exit;
	}
	kb900x_eeprom_slave_addr = slave_addr;

	ret = kb900x_i2c_master_enable(msg, true, false);
	if (ret != KB900X_E_OK) {
		LOG_ERR("Something went wrong while enabling the I2C master");
		goto exit;
	}
exit:
	if (k_mutex_unlock(&kb900x_i2c_master_mutex)) {
		LOG_ERR("kb900x i2c master mutex unlock failed");
		ret = KB900X_E_MUX_UNLOCK_FAILED;
	}

	return ret;
}

/**
 * \brief Send an I2C write operation from the KB900x I2C master interface
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[in] slave_addr the target I2C slave address
 * \param[in] data the payload to write
 * \param[in] data_size the payload size
 * \param[in] check enable or disable the write status check after the write, it is recommended to enable it
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_i2c_master_write(I2C_MSG *msg, uint8_t slave_addr, uint8_t *data,
				       size_t data_size, bool check)
{
	CHECK_NULL_ARG_WITH_RETURN(data, KB900X_E_INVALID_ARG);
	CHECK_NULL_ARG_WITH_RETURN(msg, KB900X_E_INVALID_ARG);

	kb900x_error_t ret = KB900X_E_OK;

	if (data_size > KB900X_TX_FIFO_DEPTH) {
		LOG_ERR("Invalid parameters: make sure KB900X_TX_FIFO_DEPTH < data_size");
		ret = KB900X_E_INVALID_ARG;
		goto exit;
	}

	uint8_t retry = 0;

	// Lock mutex
	for (; retry < KB900X_MAX_RETRY; retry++) {
		if (k_mutex_lock(&kb900x_i2c_master_mutex, K_MSEC(KB900X_MUTEX_LOCK_MS))) {
			k_msleep(10);
		} else {
			break;
		}
	}
	if (retry == KB900X_MAX_RETRY) {
		LOG_ERR("kb900x i2c master mutex lock failed");
		ret = KB900X_E_MUX_LOCK_FAILED;
		return ret;
	}

	// Change slave address if needed
	if (slave_addr != kb900x_eeprom_slave_addr) {
		ret = kb900x_i2c_master_set_slave_address(msg, slave_addr);
		if (ret != KB900X_E_OK) {
			LOG_ERR("Something went wrong while setting the slave address");
			goto exit;
		}
		kb900x_eeprom_slave_addr = slave_addr;
	}

	// Block FIFO for pre-fill
	ret = kb900x_i2c_master_enable(msg, true, true);
	if (ret != KB900X_E_OK) {
		LOG_ERR("Something went wrong while blocking the FIFO");
		goto exit;
	}

	// Push data to FIFO
	for (size_t i = 0; i < data_size; i++) {
		uint32_t payload = data[i];
		ret = kb900x_register_io.write(msg, kb900x_ee_IC_DATA_CMD, payload);
		if (ret != KB900X_E_OK) {
			LOG_ERR("Something went wrong while writing to TX FIFO");
			goto exit;
		}
	}

	// Release FIFO
	ret = kb900x_i2c_master_enable(msg, true, false);
	if (ret != KB900X_E_OK) {
		LOG_ERR("Something went wrong while releasing the FIFO");
		goto exit;
	}

	// Wait until transmission is complete
	ret = kb900x_i2c_master_wait_for_inactivity(msg);
	if (ret != KB900X_E_OK) {
		LOG_ERR("Something went wrong while waiting for inactivity");
		goto exit;
	}

	// Check status if requested
	if (check) {
		ret = kb900x_i2c_master_check_status(msg);
		if (ret != KB900X_E_OK) {
			LOG_ERR("Something went wrong while checking the status");
			goto exit;
		}
	}
exit:
	if (k_mutex_unlock(&kb900x_i2c_master_mutex)) {
		LOG_ERR("kb900x i2c master mutex unlock failed");
		ret = KB900X_E_MUX_UNLOCK_FAILED;
	}

	return ret;
}

/**
 * \brief Send I2C read+write operation from the KB900x I2C master interface. First write the provided data, then immediately read back the requested amount of data.
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[in] slave_addr the target I2C slave address
 * \param[in] addr the write data to send before the read, often used as a register address or EEPROM address
 * \param[in] addr_size the size of the write data
 * \param[in] length the number of bytes to read after the write
 * \param[out] result a pointer to an array to store the read data
 * \param[in] check enable or disable the write status check after the write, it is recommended to enable it
 * \param[in] skip_addr set to true to skip writing the address before reading, if true, addr can be NULL
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_i2c_master_read(I2C_MSG *msg, uint8_t slave_addr, uint8_t *addr,
				      uint8_t addr_size, size_t length, uint8_t *result, bool check,
				      bool skip_addr)
{
	if (!skip_addr) {
		CHECK_NULL_ARG_WITH_RETURN(addr, KB900X_E_INVALID_ARG);
	}
	CHECK_NULL_ARG_WITH_RETURN(result, KB900X_E_INVALID_ARG);
	CHECK_NULL_ARG_WITH_RETURN(msg, KB900X_E_INVALID_ARG);
	if (length > KB900X_TX_FIFO_DEPTH || length > KB900X_RX_FIFO_DEPTH) {
		LOG_ERR("Invalid parameters: make sure KB900X_TX_FIFO_DEPTH <= length <= KB900X_RX_FIFO_DEPTH");
		return KB900X_E_INVALID_ARG;
	}

	uint8_t retry = 0;
	kb900x_error_t ret = KB900X_E_OK;

	// Lock mutex
	for (; retry < KB900X_MAX_RETRY; retry++) {
		if (k_mutex_lock(&kb900x_i2c_master_mutex, K_MSEC(KB900X_MUTEX_LOCK_MS))) {
			k_msleep(10);
		} else {
			break;
		}
	}
	if (retry == KB900X_MAX_RETRY) {
		LOG_ERR("kb900x i2c master mutex lock failed");
		return KB900X_E_MUX_LOCK_FAILED;
	}

	if (!skip_addr) {
		// Send address
		ret = kb900x_i2c_master_write(msg, slave_addr, addr, addr_size, check);
		if (ret != KB900X_E_OK) {
			LOG_ERR("Something went wrong while sending the address");
			goto exit;
		}
	}

	// Block FIFO for pre-fill
	ret = kb900x_i2c_master_enable(msg, true, true);
	if (ret != KB900X_E_OK) {
		LOG_ERR("Something went wrong while blocking the FIFO");
		goto exit;
	}

	// Push length "read commands" to TX FIFO
	uint32_t payload = 1 << 8;
	for (size_t i = 0; i < length; i++) {
		ret = kb900x_register_io.write(msg, kb900x_ee_IC_DATA_CMD, payload);
		if (ret != KB900X_E_OK) {
			LOG_ERR("Something went wrong while writing to TX FIFO");
			goto exit;
		}
	}

	// Release FIFO
	ret = kb900x_i2c_master_enable(msg, true, false);
	if (ret != KB900X_E_OK) {
		LOG_ERR("Something went wrong while releasing the FIFO");
		goto exit;
	}

	// Wait untime transmission is complete
	ret = kb900x_i2c_master_wait_for_inactivity(msg);
	if (ret != KB900X_E_OK) {
		LOG_ERR("Something went wrong while waiting for inactivity");
		goto exit;
	}

	// Check status if requested
	if (check) {
		ret = kb900x_i2c_master_check_status(msg);
		if (ret != KB900X_E_OK) {
			LOG_ERR("Something went wrong while checking the status");
			goto exit;
		}
	}

	// Read length bytes from RX FIFO
	uint32_t tmp_val;
	for (size_t i = 0; i < length; i++) {
		ret = kb900x_register_io.read(msg, kb900x_ee_IC_DATA_CMD, &tmp_val);
		if (ret != KB900X_E_OK) {
			LOG_ERR("Something went wrong while reading RX FIFO");
			goto exit;
		}
		result[i] = tmp_val & 0xFF;
	}
exit:
	if (k_mutex_unlock(&kb900x_i2c_master_mutex)) {
		LOG_ERR("kb900x i2c master mutex unlock failed");
		ret = KB900X_E_MUX_UNLOCK_FAILED;
	}

	return ret;
}

/**
 * \brief Set the I2C master interface slave address
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[in] slave_address the target I2C slave address to set
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_i2c_master_set_slave_address(I2C_MSG *msg, uint8_t slave_address)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, KB900X_E_INVALID_ARG);

	kb900x_error_t ret = KB900X_E_OK;

	// Lock mutex
	uint8_t retry = 0;
	for (; retry < KB900X_MAX_RETRY; retry++) {
		if (k_mutex_lock(&kb900x_i2c_master_mutex, K_MSEC(KB900X_MUTEX_LOCK_MS))) {
			k_msleep(10);
		} else {
			break;
		}
	}
	if (retry == KB900X_MAX_RETRY) {
		LOG_ERR("kb900x i2c master mutex lock failed");
		return KB900X_E_MUX_LOCK_FAILED;
	}

	ret = kb900x_i2c_master_enable(msg, false, false);
	if (ret != KB900X_E_OK) {
		LOG_ERR("Something went wrong while disabling the I2C master");
		goto exit;
	}
	const uint32_t payload = slave_address;
	ret = kb900x_register_io.write(msg, kb900x_ee_IC_TAR, payload);
	if (ret != KB900X_E_OK) {
		LOG_ERR("Something went wrong while writing the field to select slave address");
		goto exit;
	}
	ret = kb900x_i2c_master_enable(msg, true, false);
	if (ret != KB900X_E_OK) {
		LOG_ERR("Something went wrong while enabling the I2C master");
		goto exit;
	}
exit:
	if (k_mutex_unlock(&kb900x_i2c_master_mutex)) {
		LOG_ERR("kb900x i2c master mutex unlock failed");
		ret = KB900X_E_MUX_UNLOCK_FAILED;
	}

	return ret;
}

/**
 * \brief Enable/Disable the KB900X I2C master interface
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[in] enable true to enable, false to disable
 * \param[in] block_fifo true to lock the FIFO for writing else false
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_i2c_master_enable(I2C_MSG *msg, bool enable, bool block_fifo)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, KB900X_E_INVALID_ARG);

	uint8_t retry = 0;
	kb900x_error_t ret = KB900X_E_OK;

	// Lock mutex
	for (; retry < KB900X_MAX_RETRY; retry++) {
		if (k_mutex_lock(&kb900x_i2c_master_mutex, K_MSEC(KB900X_MUTEX_LOCK_MS))) {
			k_msleep(10);
		} else {
			break;
		}
	}
	if (retry == KB900X_MAX_RETRY) {
		LOG_ERR("kb900x i2c master mutex lock failed");
		return KB900X_E_MUX_LOCK_FAILED;
	}

	const uint32_t value = ((uint32_t)(block_fifo) << 2) | (uint32_t)(enable);
	ret = kb900x_register_io.write(msg, kb900x_ee_IC_ENABLE, value);
	if (ret != KB900X_E_OK) {
		LOG_ERR("Something went wrong while writing the field to enable/disable the I2C master");
		goto exit;
	}
exit:
	if (k_mutex_unlock(&kb900x_i2c_master_mutex)) {
		LOG_ERR("kb900x i2c master mutex unlock failed");
		ret = KB900X_E_MUX_UNLOCK_FAILED;
	}

	return ret;
}

/**
 * \brief Wait for the I2C master interface to finish the current transaction
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_i2c_master_wait_for_inactivity(I2C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, KB900X_E_INVALID_ARG);

	uint32_t retry = 0;
	kb900x_error_t ret = KB900X_E_OK;

	// Lock mutex
	for (; retry < KB900X_MAX_RETRY; retry++) {
		if (k_mutex_lock(&kb900x_i2c_master_mutex, K_MSEC(KB900X_MUTEX_LOCK_MS))) {
			k_msleep(10);
		} else {
			break;
		}
	}
	if (retry == KB900X_MAX_RETRY) {
		LOG_ERR("kb900x i2c master mutex lock failed");
		return KB900X_E_MUX_LOCK_FAILED;
	}

	const size_t nb_retry = 1000;
	uint32_t result;
	const uint32_t mask = 0x20;
	retry = 0;
	for (; retry < nb_retry; retry++) {
		ret = kb900x_register_io.read(msg, kb900x_ee_IC_STATUS, &result);
		if (ret != KB900X_E_OK) {
			LOG_ERR("Something went wrong while reading the activity register");
			goto exit;
		}
		if ((result & mask) == 0) {
			goto exit;
		}
	}
	LOG_ERR("TIMEOUT while waiting for inactivity!");
	ret = KB900X_E_TIMEOUT;
exit:
	if (k_mutex_unlock(&kb900x_i2c_master_mutex)) {
		LOG_ERR("kb900x i2c master mutex unlock failed");
		ret = KB900X_E_MUX_UNLOCK_FAILED;
	}

	return ret;
}

/**
 * \brief Check the I2C interface status
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[in] slave_address the slave_address to communicate with
 * \param[in] data the data that we sent on the bus, for logging purpose
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_i2c_master_check_status(I2C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, KB900X_E_INVALID_ARG);

	uint8_t retry = 0;
	kb900x_error_t ret = KB900X_E_OK;

	// Lock mutex
	for (; retry < KB900X_MAX_RETRY; retry++) {
		if (k_mutex_lock(&kb900x_i2c_master_mutex, K_MSEC(KB900X_MUTEX_LOCK_MS))) {
			k_msleep(10);
		} else {
			break;
		}
	}
	if (retry == KB900X_MAX_RETRY) {
		LOG_ERR("kb900x i2c master mutex lock failed");
		return KB900X_E_MUX_LOCK_FAILED;
	}

	uint32_t result;
	const uint32_t mask = 0x40;
	ret = kb900x_register_io.read(msg, kb900x_ee_IC_RAW_INTR_STAT, &result);
	if (ret != KB900X_E_OK) {
		LOG_ERR("Something went wrong while reading the status register");
		goto exit;
	}
	if ((result & mask) != 0) {
		// Read and print abort source
		uint32_t tmp;
		kb900x_register_io.read(msg, kb900x_ee_IC_TX_ABRT_SOURCE, &tmp);
		LOG_ERR("TX ABORT detected, source reg value: 0x%08X", tmp);
		// Clear TX_ABRT interrupt
		kb900x_register_io.read(msg, kb900x_tx_abrt_clr, &tmp);
		ret = KB900X_E_I2C_ERROR;
		goto exit;
	}

exit:
	if (k_mutex_unlock(&kb900x_i2c_master_mutex)) {
		LOG_ERR("kb900x i2c master mutex unlock failed");
		ret = KB900X_E_MUX_UNLOCK_FAILED;
	}

	return ret;
}

/**
 * \brief Write a field (part of a 32 bits register)
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[in] addr the register address
 * \param[in] field_width the width of the field to write
 * \param[in] field_lsb the least significant bit of the field to write
 * \param[in] value the value of the field to write
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_write_field(I2C_MSG *msg, uint32_t addr, uint8_t field_width,
				  uint8_t field_lsb, uint32_t value)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, KB900X_E_INVALID_ARG);

	uint8_t retry = 0;
	kb900x_error_t ret = KB900X_E_OK;

	// Lock mutex
	for (; retry < KB900X_MAX_RETRY; retry++) {
		if (k_mutex_lock(&kb900x_i2c_master_mutex, K_MSEC(KB900X_MUTEX_LOCK_MS))) {
			k_msleep(10);
		} else {
			break;
		}
	}
	if (retry == KB900X_MAX_RETRY) {
		LOG_ERR("kb900x i2c master mutex lock failed");
		return KB900X_E_MUX_LOCK_FAILED;
	}

	uint32_t reg_val;
	ret = kb900x_register_io.read(msg, addr, &reg_val);
	if (ret != KB900X_E_OK) {
		LOG_ERR("Something went wrong while reading the register");
		goto exit;
	}

	uint32_t mask = (1 << field_width) - 1;

	// Set field's bits to 0
	reg_val &= ~(mask << field_lsb);

	// Set value to field's bits
	reg_val |= (value & mask) << field_lsb;
	ret = kb900x_register_io.write(msg, addr, reg_val);
	if (ret != KB900X_E_OK) {
		LOG_ERR("Something went wrong while writing the register");
		goto exit;
	}
exit:
	if (k_mutex_unlock(&kb900x_i2c_master_mutex)) {
		LOG_ERR("kb900x i2c master mutex unlock failed");
		ret = KB900X_E_MUX_UNLOCK_FAILED;
	}

	return ret;
}

/**
 * \brief Read a field (part of a 32 bits register)
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[in] addr the register address
 * \param[in] field_width the width of the field to read
 * \param[in] field_lsb the least significant bit of the field to read
 * \param[out] value a pointer to the uint32_t used to store the result
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_read_field(I2C_MSG *msg, uint32_t addr, uint8_t field_width,
				 uint8_t field_lsb, uint32_t *value)
{
	CHECK_NULL_ARG_WITH_RETURN(value, KB900X_E_INVALID_ARG);
	CHECK_NULL_ARG_WITH_RETURN(msg, KB900X_E_INVALID_ARG);

	uint8_t retry = 0;
	kb900x_error_t ret = KB900X_E_OK;

	// Lock mutex
	for (; retry < KB900X_MAX_RETRY; retry++) {
		if (k_mutex_lock(&kb900x_i2c_master_mutex, K_MSEC(KB900X_MUTEX_LOCK_MS))) {
			k_msleep(10);
		} else {
			break;
		}
	}
	if (retry == KB900X_MAX_RETRY) {
		LOG_ERR("kb900x i2c master mutex lock failed");
		return KB900X_E_MUX_LOCK_FAILED;
	}

	uint32_t result;
	ret = kb900x_register_io.read(msg, addr, &result);
	if (ret != KB900X_E_OK) {
		LOG_ERR("Something went wrong while reading field");
		goto exit;
	}
	uint32_t mask = (1 << field_width) - 1;
	*value = result >> field_lsb & mask;
exit:
	if (k_mutex_unlock(&kb900x_i2c_master_mutex)) {
		LOG_ERR("kb900x i2c master mutex unlock failed");
		ret = KB900X_E_MUX_UNLOCK_FAILED;
	}

	return ret;
}

/************** I2C MASTER SECTION END **************/

/************** EEPROM SECTION **************/

kb900x_error_t kb900x_flash_firmware(I2C_MSG *msg, uint32_t addr, uint8_t *payload,
				     size_t payload_size, kb900x_eeprom_config_t *config)
{
	CHECK_NULL_ARG_WITH_RETURN(payload, KB900X_E_INVALID_ARG);
	CHECK_NULL_ARG_WITH_RETURN(config, KB900X_E_INVALID_ARG);
	CHECK_NULL_ARG_WITH_RETURN(msg, KB900X_E_INVALID_ARG);

	const size_t QUARTER_SIZE_BYTES = 1 << 16; // If 16 bits addressing
	const size_t i2c_max_write_size = (config->page_size < KB900X_TX_FIFO_DEPTH) ?
						  (config->page_size) :
						  (KB900X_TX_FIFO_DEPTH);
	// Making sure the page size is at least 3 bytes (2 address bytes and one data byte)
	if (i2c_max_write_size <= 2) {
		LOG_ERR("Invalid page size");
		return KB900X_E_INVALID_ARG;
	}
	if ((addr + payload_size) > config->eeprom_size) {
		LOG_ERR("Write out of memory bounds! addr = 0x%08X length=%zu", addr, payload_size);
		return KB900X_E_INVALID_ARG;
	}
	// Init I2C master interface
	if (kb900x_i2c_master_init(msg, config->slave_addr)) {
		LOG_ERR("Failed to initialize I2C master");
		return KB900X_E_I2C_ERROR;
	}

	size_t bytes_written = 0;
	while (bytes_written < payload_size) {
		size_t global_addr = addr + bytes_written;
		uint8_t sa = config->slave_addr + (global_addr / QUARTER_SIZE_BYTES);
		uint16_t page_addr = global_addr % QUARTER_SIZE_BYTES;

		size_t write_size = ((i2c_max_write_size - 2) < (payload_size - bytes_written)) ?
					    (i2c_max_write_size - 2) :
					    (payload_size - bytes_written);

		if (global_addr / QUARTER_SIZE_BYTES <
		    (global_addr + write_size - 1) / QUARTER_SIZE_BYTES) {
			write_size = QUARTER_SIZE_BYTES - (global_addr % QUARTER_SIZE_BYTES);
		}
		if (global_addr / config->page_size <
		    (global_addr + write_size - 1) / config->page_size) {
			write_size = config->page_size - (global_addr % config->page_size);
		}

		uint8_t data_bytes[write_size];
		for (size_t i = 0; i < write_size; i++) {
			data_bytes[i] = payload[i + bytes_written];
		}
		uint8_t bytes_to_write[write_size + 2];
		const uint8_t mask = 0xFF;
		bytes_to_write[0] = page_addr >> BITS_IN_BYTE;
		bytes_to_write[1] = page_addr & mask;
		for (size_t i = 0; i < write_size; i++) {
			bytes_to_write[i + 2] = data_bytes[i];
		}
		kb900x_error_t ret =
			kb900x_i2c_master_write(msg, sa, bytes_to_write, write_size + 2, true);
		if (ret != KB900X_E_OK) {
			LOG_ERR("Failed to write to EEPROM");
			return ret;
		}

		// Sleep for transaction to complete in EEPROM
		k_msleep(config->write_cycle_time_ms);

		// check bytes written if first or second iteration
		// to assert working communication
		if (bytes_written <= write_size) {
			uint8_t result[write_size];
			ret = kb900x_i2c_master_read(msg, sa, bytes_to_write, 2, write_size, result,
						     true, false);
			if (ret != KB900X_E_OK) {
				LOG_ERR("Failed to read from EEPROM");
				return ret;
			}
			for (size_t i = 0; i < write_size; i++) {
				if (result[i] != data_bytes[i]) {
					LOG_ERR("Error while writing the firmware, does not match addr = 0x%02x%02x byte number %zu",
						bytes_to_write[0], bytes_to_write[1], i);
					// Buffer for logging
					const int nb_char_per_byte = 6; // "0x00 \0"
					char buf[write_size * nb_char_per_byte];
					buf[0] = '\0';
					for (size_t j = 0; j < write_size; j++) {
						char temp[nb_char_per_byte];
						snprintf(temp, sizeof(temp), "0x%02x ",
							 data_bytes[j]); // NOLINT
						strncat(buf, temp,
							sizeof(buf) - strlen(buf) - 1); // NOLINT
					}
					LOG_ERR("%s", log_strdup(buf));
					LOG_ERR(" VS ");
					buf[0] = '\0';
					for (size_t j = 0; j < write_size; j++) {
						char temp[nb_char_per_byte];
						snprintf(temp, sizeof(temp), "0x%02x ",
							 result[j]); // NOLINT
						strncat(buf, temp,
							sizeof(buf) - strlen(buf) - 1); // NOLINT
					}
					LOG_ERR("%s", log_strdup(buf));
					return KB900X_E_FW_WRITE_ERROR;
				}
			}
		}
		bytes_written += write_size;
	}

	return KB900X_E_OK;
}

uint8_t kb900x_pcie_retimer_fw_update(I2C_MSG *msg, uint32_t offset, uint16_t msg_len,
				      uint8_t *msg_buf, uint8_t flag)
{
	(void)flag;

// Default EEPROM configuration required since the existing function signature does not allow passing the configuration
#define KB900X_RISER_CARD_EEPROM_BASE_ADDR (0x50)
#define KB900X_RISER_CARD_EEPROM_WRITE_CYCLE_TIME_MS (10)
#define KB900X_RISER_CARD_EEPROM_SIZE (1 << 18) // 256 KiB
#define KB900X_RISER_CARD_EEPROM_PAGE_SIZE (256)
	kb900x_eeprom_config_t eeprom_config = {
		.slave_addr = KB900X_RISER_CARD_EEPROM_BASE_ADDR,
		.write_cycle_time_ms = KB900X_RISER_CARD_EEPROM_WRITE_CYCLE_TIME_MS,
		.eeprom_size = KB900X_RISER_CARD_EEPROM_SIZE,
		.page_size = KB900X_RISER_CARD_EEPROM_PAGE_SIZE,
	};

	// Call the actual function and translate between KB900X error code to OpenBIC (uint8_t) error code
	const kb900x_error_t status =
		kb900x_flash_firmware(msg, offset, msg_buf, msg_len, &eeprom_config);
	uint8_t ret = FWUPDATE_UPDATE_FAIL;
	switch (status) {
	case KB900X_E_OK:
		ret = FWUPDATE_SUCCESS;
		break;
	case KB900X_E_NOT_IMPLEMENTED:
		ret = FWUPDATE_NOT_SUPPORT;
		break;
	case KB900X_E_INVALID_ARG:
		ret = FWUPDATE_OVER_LENGTH;
		break;
	case KB900X_E_MUX_LOCK_FAILED:
	case KB900X_E_MUX_UNLOCK_FAILED:
	case KB900X_E_I2C_ERROR:
	case KB900X_E_CRC_ERROR:
	case KB900X_E_TIMEOUT:
	case KB900X_E_FW_WRITE_ERROR:
		ret = FWUPDATE_UPDATE_FAIL;
		break;
	default:
		ret = FWUPDATE_UPDATE_FAIL;
		break;
	}

	return ret;
}

kb900x_error_t kb900x_read_firmware(I2C_MSG *msg, uint32_t addr, size_t length, uint8_t *result,
				    kb900x_eeprom_config_t *config)
{
	CHECK_NULL_ARG_WITH_RETURN(result, KB900X_E_INVALID_ARG);
	CHECK_NULL_ARG_WITH_RETURN(config, KB900X_E_INVALID_ARG);
	CHECK_NULL_ARG_WITH_RETURN(msg, KB900X_E_INVALID_ARG);

	const size_t QUARTER_SIZE_BYTES = 1 << 16; // If 16 bits addressing
	const size_t i2c_max_read_size =
		config->page_size < KB900X_RX_FIFO_DEPTH ? config->page_size : KB900X_RX_FIFO_DEPTH;

	if ((addr + length) > config->eeprom_size) {
		LOG_ERR("Read out of memory bounds! addr = 0x%08X length=%zu", addr, length);
		return KB900X_E_INVALID_ARG;
	}
	// Init I2C master interface
	if (kb900x_i2c_master_init(msg, config->slave_addr)) {
		LOG_ERR("Failed to initialize I2C master");
		return KB900X_E_I2C_ERROR;
	}

	size_t bytes_read = 0;
	uint8_t read_buffer[i2c_max_read_size];
	const uint8_t mask = 0xFF;
	uint8_t current_address[2];
	// Start address
	current_address[0] = (addr >> BITS_IN_BYTE) & mask;
	current_address[1] = addr & mask;
	while (bytes_read < length) {
		size_t global_addr = addr + bytes_read;
		uint8_t sa = config->slave_addr + (global_addr / QUARTER_SIZE_BYTES);
		uint16_t page_addr = global_addr % QUARTER_SIZE_BYTES;

		size_t read_size = ((i2c_max_read_size) < (length - bytes_read)) ?
					   (i2c_max_read_size) :
					   (length - bytes_read);

		if (global_addr / QUARTER_SIZE_BYTES <
		    (global_addr + read_size - 1) / QUARTER_SIZE_BYTES) {
			read_size = QUARTER_SIZE_BYTES - (global_addr % QUARTER_SIZE_BYTES);
		}
		current_address[0] = page_addr >> BITS_IN_BYTE;
		current_address[1] = page_addr & mask;

		const kb900x_error_t ret = kb900x_i2c_master_read(
			msg, sa, current_address, 2, read_size, read_buffer, true, false);
		if (ret != KB900X_E_OK) {
			LOG_ERR("Something went wrong while reading from EEPROM");
			return ret;
		}

		for (size_t i = 0; i < read_size; i++) {
			result[i + bytes_read] = read_buffer[i];
		}
		bytes_read += read_size;
	}
	return KB900X_E_OK;
}

kb900x_error_t kb900x_check_firmware(I2C_MSG *msg, uint32_t offset, uint8_t *buffer,
				     uint32_t buffer_size, kb900x_eeprom_config_t *config)
{
	LOG_ERR("Not Yet Implemented");
	return KB900X_E_NOT_IMPLEMENTED;
}

/************** EEPROM SECTION END **************/

/************** CONNECTION MODE **************/

/** \brief Set the communication mode used by the SDK.
 *
 * Can be either KB900X_COMM_SMBUS or KB900X_COMM_RAW_I2C.
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[in] mode the connection mode (KB900X_COMM_SMBUS = 0 or KB900X_COMM_RAW_I2C = 1)
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_set_connection_mode(I2C_MSG *msg, kb900x_communication_mode_t mode)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, KB900X_E_INVALID_ARG);
	if (mode == KB900X_COMM_RAW_I2C) {
		kb900x_register_io.write = twi_write_register;
		kb900x_register_io.read = twi_read_register;
	} else if (mode == KB900X_COMM_SMBUS) {
		kb900x_register_io.write = smbus_write_register;
		kb900x_register_io.read = smbus_read_register;
	} else {
		LOG_ERR("Invalid connection mode");
		return KB900X_E_INVALID_ARG;
	}
	return KB900X_E_OK;
}

kb900x_error_t kb900x_get_connection_mode(kb900x_communication_mode_t *mode)
{
	CHECK_NULL_ARG_WITH_RETURN(mode, KB900X_E_INVALID_ARG);
	if (kb900x_register_io.write == twi_write_register &&
	    kb900x_register_io.read == twi_read_register) {
		*mode = KB900X_COMM_RAW_I2C;
	} else if (kb900x_register_io.write == smbus_write_register &&
		   kb900x_register_io.read == smbus_read_register) {
		*mode = KB900X_COMM_SMBUS;
	} else {
		LOG_ERR("Corrupted connection mode");
		return KB900X_E_INVALID_ARG;
	}
	return KB900X_E_OK;
}

kb900x_error_t kb900x_detect_connection_mode(I2C_MSG *msg, kb900x_communication_mode_t *mode)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, KB900X_E_INVALID_ARG);
	CHECK_NULL_ARG_WITH_RETURN(mode, KB900X_E_INVALID_ARG);
	kb900x_error_t ret = kb900x_set_connection_mode(msg, KB900X_COMM_SMBUS);
	if (ret) {
		return ret;
	}
	uint32_t rev_id = 0;
	ret = kb900x_read_register(msg, kb900x_cfg_top_revid, &rev_id);
	if (ret == KB900X_E_OK && (rev_id == KB900X_B0_REVID || rev_id == KB900X_B1_REVID)) {
		*mode = KB900X_COMM_SMBUS;
		return KB900X_E_OK;
	}
	ret = kb900x_set_connection_mode(msg, KB900X_COMM_RAW_I2C);
	if (ret) {
		return ret;
	}
	kb900x_read_register(msg, kb900x_cfg_top_revid, &rev_id);
	// Fix byteshifting
	const uint32_t byteshifting_val = 0xFF;
	const uint8_t offset = 24;
	const uint8_t max_retry = 4;
	uint8_t retry = 0;
	while (rev_id >> offset == byteshifting_val && retry < max_retry) {
		// Send ACK
		msg->tx_len = 0;
		msg->rx_len = 1;
		if (i2c_master_read(msg, KB900X_MAX_RETRY)) {
			LOG_ERR("Failed to send ACK");
			return KB900X_E_I2C_ERROR;
		}
		kb900x_read_register(msg, kb900x_cfg_top_revid, &rev_id);
		retry++;
	}
	if (retry == max_retry) {
		LOG_ERR("Failed to detect connection mode - No ACK");
		return KB900X_E_COMM;
	}
	if (ret == KB900X_E_OK && (rev_id == KB900X_B0_REVID || rev_id == KB900X_B1_REVID)) {
		*mode = KB900X_COMM_RAW_I2C;
		return KB900X_E_OK;
	}
	LOG_ERR("Failed to detect connection mode - no communication");
	return KB900X_E_COMM;
}

kb900x_error_t kb900x_enable_smbus(I2C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, KB900X_E_INVALID_ARG);
	bool is_ready;
	kb900x_error_t ret = kb900x_is_firmware_ready(msg, &is_ready);
	if (ret) {
		return ret;
	}
	// FIXME is it clear enough?
	if (!is_ready) {
		LOG_ERR("Firmware is not ready, if this issue persists consider updating the firmware");
		return KB900X_E_FW_NOT_READY;
	}
	ret = kb900x_write_field(msg, kb900x_smbus_mux, 1, 1, 1);
	if (ret) {
		LOG_ERR("Failed to enable SMBUS");
		return ret;
	}
	// Wait for the firmware to enable SMBus
	k_sleep(K_MSEC(600));

	ret = kb900x_set_connection_mode(msg, KB900X_COMM_SMBUS);
	return ret;
}

kb900x_error_t kb900x_init_smbus(I2C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, KB900X_E_INVALID_ARG);
	// Check communication mode
	kb900x_communication_mode_t mode;
	kb900x_error_t ret = kb900x_detect_connection_mode(msg, &mode);
	if (ret) {
		LOG_DBG("KB900x: Failed to detect connection mode : %d", ret);
		return ret;
	}
	if (mode == KB900X_COMM_RAW_I2C) {
		const uint8_t max_retry = 5;
		uint8_t retry = 0;
		kb900x_boot_entity_t entity;
		kb900x_boot_status_t status;
		while (retry < max_retry) {
			ret = kb900x_get_boot_status(msg, &entity, &status);
			if (ret) {
				LOG_DBG("KB900x: Failed to check firmware status : %d", ret);
				return ret;
			}
			if (status == KB900X_STATE_ERROR) {
				LOG_DBG("KB900x: Firmware boot is in error state in entity : %d",
					entity);
				return KB900X_E_FW_ERROR;
			}
			if (entity == KB900X_ENTITY_FW && status == KB900X_STATE_READY) {
				ret = kb900x_enable_smbus(msg);
				if (ret) {
					LOG_DBG("KB900x: Failed to enable SMBUS : %d", ret);
					return ret;
				}
				return KB900X_E_OK;
			}
			LOG_DBG("KB900x: Firmware is not ready - retrying in 300ms");
			// Retry if the firmware is not ready
			k_sleep(K_MSEC(300));
			retry++;
		}
		LOG_DBG("KB900x: Firmware is not responding after %d retries", max_retry);
		return KB900X_E_FW_ERROR;
	}
	// SMBus already initalized
	return KB900X_E_OK;
}

/************** CONNECTION MODE END **************/

kb900x_error_t kb900x_reset(I2C_MSG *msg)
{
	// Stub implementation, always fails
	LOG_ERR("Not Yet Implemented");
	return KB900X_E_NOT_IMPLEMENTED;
}

kb900x_error_t kb900x_is_firmware_ready(I2C_MSG *msg, bool *is_ready)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, KB900X_E_INVALID_ARG);
	CHECK_NULL_ARG_WITH_RETURN(is_ready, KB900X_E_INVALID_ARG);
	kb900x_boot_status_t status;
	kb900x_boot_entity_t entity;
	kb900x_error_t ret = kb900x_get_boot_status(msg, &entity, &status);
	if (ret) {
		return ret;
	}
	*is_ready = (status == KB900X_STATE_READY && entity == KB900X_ENTITY_FW);
	return KB900X_E_OK;
}

kb900x_error_t kb900x_get_boot_status(I2C_MSG *msg, kb900x_boot_entity_t *entity,
				      kb900x_boot_status_t *status)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, KB900X_E_INVALID_ARG);
	CHECK_NULL_ARG_WITH_RETURN(entity, KB900X_E_INVALID_ARG);
	CHECK_NULL_ARG_WITH_RETURN(status, KB900X_E_INVALID_ARG);
	uint32_t value;
	kb900x_error_t ret = kb900x_read_register(msg, kb900x_cpu_system, &value);
	if (ret) {
		return ret;
	}
	const uint32_t mask_28_27 = 0x18000000;
	const uint32_t offset_28_27 = 27;
	const uint32_t mask_30_29 = 0x60000000;
	const uint32_t offset_30_29 = 29;
	*status = (value & mask_28_27) >> offset_28_27;
	*entity = (value & mask_30_29) >> offset_30_29;
	return KB900X_E_OK;
}

kb900x_error_t kb900x_get_revid(I2C_MSG *msg, uint32_t *revid)
{
	// Stub implementation, always fails
	LOG_ERR("Not Yet Implemented");
	return KB900X_E_NOT_IMPLEMENTED;
}

kb900x_error_t kb900x_get_sds_addr(I2C_MSG *msg, uint32_t *sds_addr)
{
	// Stub implementation, always fails
	LOG_ERR("Not Yet Implemented");
	return KB900X_E_NOT_IMPLEMENTED;
}

kb900x_error_t kb900x_dump_phy_rpcs_registers_with_offset(I2C_MSG *msg,
							  kb900x_register_record_t *records,
							  size_t skip_num, size_t dump_num)
{
	// Stub implementation, always fails
	LOG_ERR("Not Yet Implemented");
	return KB900X_E_NOT_IMPLEMENTED;
}

uint8_t kb900x_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	const kb900x_init_arg *init_args = (const kb900x_init_arg *)cfg->init_args;
	if (!init_args->is_init) {
		LOG_ERR("KB900x: Device is not initialized");
		return SENSOR_UNSPECIFIED_ERROR;
	}

	I2C_MSG msg;
	memset(&msg, 0, sizeof(I2C_MSG));
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;

	kb900x_error_t ret;
	// NOTE: cfg->offset is a u16, but to ensure compatility of this version of OpenBIC with other vendors,
	// we must ensure cfg->offset is actually a u8. Since SMBus offsets are u16, we need to translate from
	// 'symbolic' cfg->offset to real SMBus offsets. For example, the real SMBus offset is hardcoded in
	// kb900x_get_temperature here.
	switch (cfg->offset) {
	case KB900X_CFG_OFFSET_TEMPERATURE: {
		float temperature = 0.0;
		ret = kb900x_get_temperature(&msg, &temperature);
		if (ret) {
			LOG_ERR("KB900x: Failed to read temperature : %d", ret);
			return ret;
		}
		sensor_val *sval = (sensor_val *)reading;
		memset(sval, 0, sizeof(*sval));
		sval->integer = (int)temperature;
		sval->fraction = (int)((temperature - sval->integer) * 1000);
		break;
	}
	default:
		LOG_ERR("KB900x: Unsupported offset : %d", cfg->offset);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	return SENSOR_READ_SUCCESS;
}

uint8_t kb900x_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(cfg->init_args, SENSOR_INIT_UNSPECIFIED_ERROR);

	I2C_MSG msg = { 0 };
	kb900x_init_arg *init_args = (kb900x_init_arg *)cfg->init_args;
	if (!init_args->is_init) {
		memset(&msg, 0, sizeof(I2C_MSG));
		msg.bus = cfg->port;
		msg.target_addr = cfg->target_addr;

		kb900x_error_t ret = kb900x_init_smbus(&msg);
		if (ret) {
			LOG_ERR("KB900x: Failed to initialize SMBUS : %d", ret);
			return SENSOR_FAIL_TO_ACCESS;
		}
	}
	init_args->is_init = true;
	cfg->read = kb900x_read;
	return SENSOR_INIT_SUCCESS;
}
