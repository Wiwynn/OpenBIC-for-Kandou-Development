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
#include "libutil.h"
#include "kb900x.h"

LOG_MODULE_REGISTER(dev_kb900x);
K_MUTEX_DEFINE(kb900x_mutex);

static uint8_t cal_crc8_pec(uint8_t *crc_list, uint8_t len)
{
	CHECK_NULL_ARG_WITH_RETURN(crc_list, 0);
	return crc8(crc_list, len, 0x07, 0x00, false);
}

static bool verify_crc8_pec(uint8_t *crc_list, I2C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(crc_list, false);
	CHECK_NULL_ARG_WITH_RETURN(msg, false);

	uint8_t pec_crc, crc_reslut;
	uint8_t bytecnt = msg->data[0];
	crc_list[0] = KB900X_SLAVE_ADDR << 1;
	crc_list[1] = I2C_READ_CCODE_END;
	crc_list[2] = (KB900X_SLAVE_ADDR << 1) + 1;
	memcpy(&(crc_list[3]), msg->data, bytecnt + 1);

	pec_crc = msg->data[bytecnt + 1]; // pec value

	crc_reslut = cal_crc8_pec(crc_list, 3 + bytecnt + 1);

	if (pec_crc != crc_reslut) {
		LOG_ERR("The read data pec_crc=0x%x is invalid, the right crc value=0x%x",
			pec_crc, crc_reslut);
		return false;
	}
	return true;
}

bool smbus_read(I2C_MSG *msg, uint16_t offsets)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);

	uint8_t retry = 0;
	bool ret = false;

	// Lock mutex
	for (; retry < MAX_RETRY; retry++) {
		if (k_mutex_lock(&kb900x_mutex, K_MSEC(KB900X_MUTEX_LOCK_MS))) {
			k_msleep(10);
		} else {
			break;
		}
	}
	if (retry == MAX_RETRY) {
		LOG_ERR("kb900x mutex lock failed");
		return ret;
	}

	msg->target_addr = KB900X_SLAVE_ADDR;
	msg->data[0] = I2C_READ_CCODE_START; // COMMAND CODE
	msg->data[1] = I2C_WRITE_BYTCNT; // byte count
	msg->data[2] = (uint8_t)(offsets & 0xFF); // lower offset
	msg->data[3] = (uint8_t)(offsets >> 8); // upper offset
	msg->tx_len = 5;

	// PEC signature
	uint8_t crc_list[msg->tx_len];
	crc_list[0] = KB900X_SLAVE_ADDR << 1;
	memcpy(&(crc_list[1]), msg->data, msg->tx_len - 1);
	msg->data[msg->tx_len - 1] = cal_crc8_pec(crc_list, msg->tx_len);

	// Write (Prepare read)
	if (i2c_master_write(msg, retry)) {
		LOG_ERR("Failed to write, 0x%X not set", offsets);
		goto exit;
	}

	// Read
	memset(msg->data, 0, I2C_BUFF_SIZE);
	msg->tx_len = 1;
	msg->rx_len = 8;
	msg->data[0] = I2C_READ_CCODE_END;
	if (i2c_master_read(msg, MAX_RETRY)) {
		LOG_ERR("Failed to read PCIE RETIMER addr 0x%X", offsets);
		goto exit;
	}

	// PEC validation
	if (!verify_crc8_pec(crc_list, msg)) {
		goto exit;
	}

	ret = true;

exit:
	if (k_mutex_unlock(&kb900x_mutex)) {
		LOG_ERR("kb900x mutex unlock failed");
	}
	return ret;
}

bool get_vendor_id(I2C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);
	return smbus_read(msg, OFFSET_GLOB_PARAM_REG_1);
}

bool get_fw_version(I2C_MSG *msg, uint8_t *version)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);
	CHECK_NULL_ARG_WITH_RETURN(version, false);

	if (smbus_read(msg, OFFSET_FW_VERSION)) {
		// Parse response
		uint8_t bytecnt = msg->data[0];
		for (uint8_t i = 0; i < (bytecnt - 2); i++) {
			version[i] = msg->data[bytecnt - i];
		}
		return true;
	}

	return false;
}

bool get_temperature(I2C_MSG *msg, float *temperature)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, false);
	CHECK_NULL_ARG_WITH_RETURN(temperature, false);

	if (smbus_read(msg, OFFSET_TEMPERATURE)) {
		// Parse response
		uint8_t bytecnt = msg->data[0];
		*temperature =
			((float)(msg->data[bytecnt - 3] + (msg->data[bytecnt - 2] << 8) +
      (msg->data[bytecnt - 1] << 16) + (msg->data[bytecnt] << 24)) /
      (float)(1 << FLOAT_PRECISION)) +
			ABSOLUTE_ZER0;
		return true;
	}

	return false;
}
