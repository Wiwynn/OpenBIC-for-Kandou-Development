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

#ifndef KB900_H
#define KB900_H

#include "hal_i2c.h"
#include "util_spi.h"
#include "sensor.h"

// Read SMBus command (2 bytes offset)
#define KB900X_CCODE_START_READ_FUNC0 0x82
#define KB900X_CCODE_END_READ_FUNC0 0x81
// Read SMBus register (4 bytes offset)
#define KB900X_CCODE_START_READ_FUNC2 0x8A
#define KB900X_CCODE_END_READ_FUNC2 0x89
// Write SMBus register (4 bytes offset)
#define KB900X_CCODE_START_END_WRITE_FUNC3 0x8F

#define KB900X_I2C_WRITE_BYTCNT 0x02

#define KB900X_SLAVE_ADDR 0x21
#define KB900X_MUTEX_LOCK_MS 1000

#define KB900X_B0_REVID 0x00000010
#define KB900X_B1_REVID 0x00000011

// Symbolic offsets for Zephyr's sensor_cfg->offset
#define KB900X_CFG_OFFSET_TEMPERATURE 0x00

// SMBus command offsets
#define KB900X_SMBUS_OFFSET_GLOB_PARAM_REG_1 0x0004
#define KB900X_SMBUS_OFFSET_FW_VERSION 0x0500
#define KB900X_SMBUS_OFFSET_TEMPERATURE 0x0510

#define KB900X_MAX_RETRY 3
#define KB900X_REGISTER_VALUE_WIDTH 4
#define KB900X_REGISTER_ADDRESS_WIDTH 4

#define ABSOLUTE_ZER0 (-273.15)
#define KB900X_FLOAT_PRECISION (16)
#define BITS_IN_BYTE (8)

#define KB900X_SIDE_A (0)
#define KB900X_SIDE_B (1)
#define KB9003_NUM_LANES (16)

/**
 * \brief Error codes for the KB900X driver.
 *
 * This enumeration defines the error codes that can be returned
 * by the KB900X driver functions.
 *
 * \note The error codes are negative values.
 */
typedef enum {
	KB900X_E_OK = 0, /**< No error */
	KB900X_E_NOT_IMPLEMENTED = -1, /**< Functionality not implemented */
	KB900X_E_INVALID_ARG = -2, /**< Invalid argument */
	KB900X_E_MUX_LOCK_FAILED = -3, /**< Mutex lock failed */
	KB900X_E_MUX_UNLOCK_FAILED = -4, /**< Mutex unlock failed */
	KB900X_E_I2C_ERROR = -5, /**< I2C error */
	KB900X_E_CRC_ERROR = -6, /**< CRC error */
	KB900X_E_TIMEOUT = -7, /**< I2C Timeout */
	KB900X_E_FW_WRITE_ERROR = -8, /**< Firmware write error */
	KB900X_E_COMM = -9, /**< Communication error */
	KB900X_E_FW_NOT_READY = -10, /**< Firmware not ready to switch to SMBus */
	KB900X_E_FW_ERROR = -11, /**< Firmware error */
} kb900x_error_t;

// Type and struct to inject different read/write methods (SMBus, raw I2C, mocked for unit tests)
typedef kb900x_error_t (*KB900X_WRITE_REGISTER_OPERATION)(I2C_MSG *msg, uint32_t address,
							  uint32_t value);
typedef kb900x_error_t (*KB900X_READ_REGISTER_OPERATION)(I2C_MSG *msg, uint32_t address,
							 uint32_t *value);

typedef struct {
	KB900X_WRITE_REGISTER_OPERATION write;
	KB900X_READ_REGISTER_OPERATION read;
} KB900X_REGISTER_IO;

extern KB900X_REGISTER_IO kb900x_register_io;

/**
 * \brief Configuration structure for EEPROM.
 *
 * This structure holds the configuration parameters required
 * for initializing and interacting with the EEPROM device.
 */
typedef struct {
	/**
         * \brief The I2C address of the EEPROM slave.
         *
         * This is the 7-bit I2C slave address used to communicate
         * with the EEPROM.
         */
	uint8_t slave_addr;

	/**
         * \brief The total size of the EEPROM in bytes.
         *
         * Specifies the total memory capacity of the EEPROM.
         */
	uint32_t eeprom_size;

	/**
         * \brief The page size of the EEPROM in bytes.
         *
         * Specifies the size of a page in the EEPROM. This value
         * is used to determine the maximum amount of data that can
         * be written in a single write operation.
         */
	uint32_t page_size;

	/**
         * \brief Write cycle time for EEPROM in milliseconds.
         *
         * Specifies the time needed to complete one write cycle to
         * the EEPROM in milliseconds. It is important to respect
         * this timing to ensure proper data writing.
         */
	unsigned write_cycle_time_ms;
} kb900x_eeprom_config_t;

/**
 * \brief Communication mode for the KB900X device.
 */
typedef enum {
	KB900X_COMM_RAW_I2C = 0,
	KB900X_COMM_SMBUS,
} kb900x_communication_mode_t;

/**
 * \brief Boot status of the KB900X device.
 */
typedef enum {
	KB900X_STATE_INIT = 0,
	KB900X_STATE_READY = 1,
	KB900X_STATE_ERROR = 2,
} kb900x_boot_status_t;

/**
 * \brief Firmware boot entity type.
 */
typedef enum {
	KB900X_ENTITY_BOOTROM = 0,
	KB900X_ENTITY_SBL = 1,
	KB900X_ENTITY_PHY_LOAD = 2,
	KB900X_ENTITY_FW = 3
} kb900x_boot_entity_t;

/**
 * \brief This struct is used to hold the firmware health (get_firmware_health)
 */
typedef union {
	struct {
		/* liveliness counter */
		uint32_t liveliness : 4;
		/* unused */
		uint32_t : 27;
		/* Firmware is initialized */
		uint32_t fw_is_initialized : 1;
	};
	uint32_t raw;
} kb900x_fw_health_t;

/**
 * \brief This struct is used to hold the status of a link (get_link_status)
 */
typedef union {
	struct {
		uint32_t link_info : 1;
		uint32_t l0_reached : 1;
		uint32_t link_number : 9;
		uint32_t rtssm_state : 5;
		uint32_t rtssm_speed : 3;
		uint32_t link_width : 5;
		/* unused */
		uint32_t : 8;
	};
	uint32_t raw;
} kb900x_link_status_t;

/**
 * \brief This struct stores a register address and value tuple, for register dumps.
 */
typedef struct {
	uint32_t address;
	uint32_t value;
} kb900x_register_record_t;

/************** I2C MASTER SECTION START **************/

#define KB900X_TX_FIFO_DEPTH (24)
#define KB900X_RX_FIFO_DEPTH (24)

// I2C Master interface registers
#define kb900x_ee_IC_CON 0xe0081000
#define kb900x_ee_IC_TAR 0xe0081004
#define kb900x_ee_IC_DATA_CMD 0xe0081010
#define kb900x_ee_IC_INTR_MASK 0xe0081030
#define kb900x_ee_IC_RAW_INTR_STAT 0xe0081034
#define kb900x_ee_IC_ENABLE 0xe008106c
#define kb900x_ee_IC_STATUS 0xe0081070
#define kb900x_ee_IC_TX_ABRT_SOURCE 0xe0081080
#define kb900x_cfg_top_vd_bump_0 0xe048018c
#define kb900x_cfg_top_vd_bump_1 0xe0480190

// Other registers
#define kb900x_cpu_periph_clk_gate_en                                                              \
	0xe009005c // I2C Master clock - Clock gate enables per peripheral
#define kb900x_cpu_system 0xe0090008
#define kb900x_tx_abrt_clr 0xe0081054 // Clear interrupts
#define kb900x_cfg_top_revid 0xe0480004
#define kb900x_smbus_mux 0xe0480008

// Global variable to store the selected slave address
extern uint8_t kb900x_eeprom_slave_addr;

/************** CONNECTION MODE **************/

/**
 * \brief Get the communication mode used by the SDK.
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[out] mode a pointer to write the communication mode to
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_get_connection_mode(kb900x_communication_mode_t *mode);

/** \brief Detect the connection mode and set it.
 *
 * This function detects the connection mode of the retimer and set it.
 * First trying with SMBus - If wrong PEC then raw I2C.
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[out] mode the communication mode
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_detect_connection_mode(I2C_MSG *msg, kb900x_communication_mode_t *mode);

/** \brief Enable SMBus connection mode by toggling the MUX between TWI and SMBus.
 *
 * \note This feature interract with the firmware to enable the SMBus connection mode.
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_enable_smbus(I2C_MSG *msg);

/** \brief Run the SMBus initialization sequence.
 *
 * \note This feature interract with the firmware to enable the SMBus connection mode.
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_init_smbus(I2C_MSG *msg);

/************** CONNECTION MODE END **************/

/************** I2C Master START **************/

/**
 * \brief Initialize the KB900X I2C master interface
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[in] slave_addr the default I2C slave address KB900X's I2C master interface should communicate with
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_i2c_master_init(I2C_MSG *msg, uint8_t slave_addr);

/************** I2C Master STOP **************/

/************** KB900X EEPROM READ/WRITE SECTION **************/

/**
 * \brief Read from the KB900X EEPROM.
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[in] addr the EEPROM address to start reading from
 * \param[in] length the number of bytes to read
 * \param[out] result a pointer to an array to store the read data
 * \param[in] config the EEPROM configuration, see kb900x_eeprom_config_t for more details
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_read_firmware(I2C_MSG *msg, uint32_t addr, size_t length, uint8_t *result,
				    kb900x_eeprom_config_t *config);

/** \brief Compare the content of the EEPROM with expected data.
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[in] offset the offset in the EEPROM to start reading from
 * \param[in] buffer the buffer containing the expected firmware, starting at `offset` offset in the EEPROM
 * \param[in] buffer_size the size of the buffer and of the data chunk to read and compare
 * \param[in] config the EEPROM config structure
 *
 * \return 0 if no error, else the error code
 */
kb900x_error_t kb900x_check_firmware(I2C_MSG *msg, uint32_t offset, uint8_t *buffer,
				     uint32_t buffer_size, kb900x_eeprom_config_t *config);

/** \brief Flash the firmware.
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[in] addr the EEPROM address to start writing to
 * \param[in] payload the data to write
 * \param[in] payload_size the payload length
 * \param[in] config the EEPROM configuration, see kb900x_eeprom_config_t for more details
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_flash_firmware(I2C_MSG *msg, uint32_t addr, uint8_t *payload,
				     size_t payload_size, kb900x_eeprom_config_t *config);

/**
 * \brief Write the KB900X firmware to the KB900X firmware EEPROM.
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[in] offset the offset in the EEPROM to start writing to
 * \param[in] msg_len the length of the data to write
 * \param[in] msg_buf the buffer containing the firmware to write
 * \param[in] flag unused
 *
 * \return 0 if no error, else the error code
 */
uint8_t kb900x_pcie_retimer_fw_update(I2C_MSG *msg, uint32_t offset, uint16_t msg_len,
				      uint8_t *msg_buf, uint8_t flag);

/************** KB900X EEPROM READ/WRITE SECTION END **************/

/************** KB900X SMBUS COMMANDS **************/

/**
 * \brief Get the KB900X vendor ID.
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *               `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[out] vendor_id a pointer to the integer used to store the vendor ID
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_get_vendor_id_with_err_code(I2C_MSG *msg, int *vendor_id);

/**
 * \brief Get the KB900X vendor ID.
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *               `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[out] vendor_id a pointer to the integer used to store the vendor ID
 *
 * \return true if successful, otherwise false. Error information will be printed in the logs.
 */
bool kb900x_get_vendor_id(I2C_MSG *msg, int *vendor_id);

/**
 * \brief Get the KB900X temperature.
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *               `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[out] temperature a pointer to the float used to store the temperature
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_get_temperature(I2C_MSG *msg, float *temperature);

/** \brief Get the temperature of a KB900X lane.
 *
 * This function gets the temperature of the sensor close to the provided lane.
 *
 * There are 16 sensors in total on the KB9003 (1 sensor for 2 lanes).
 *
 * The port parameter is used to select from which side of the retimer to get
 * the temperature from (0 = upstream, 1 = downstream).
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *               `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[in] port the port, 0 = A (upstream), anything else = B (downstream)
 * \param[in] lane the lane id, 0 to 15
 * \param[out] temperature pointer to the temperature[Celsius]
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_get_lane_temperature(I2C_MSG *msg, int port, int lane, float *temperature);

/**
 * \brief Get the KB900X firmware version.
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *               `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[out] version a pointer to the uint8_t array used to store the firmware version;
 *              the array must be at least 4 bytes long
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_get_fw_version_with_err_code(I2C_MSG *msg, uint8_t *version);

/**
 * \brief Get the KB900X firmware version.
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *               `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[out] version a pointer to the uint8_t array used to store the firmware version;
 *              the array must be at least 4 bytes long
 *
 * \return true if successful, otherwise false. Error information will be printed in the logs.
 */
bool kb900x_get_fw_version(I2C_MSG *msg, uint8_t *version);

/** \brief Get the firmware health.
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *               `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[out] firmware_health pointer to the firmware health
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_get_firmware_health(I2C_MSG *msg, kb900x_fw_health_t *firmware_health);

/** \brief Dump the RTSSM data area into a buffer.
 *
 * \note Only in SMBUS mode.
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *               `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[out] buffer a pointer to a u32 buffer of size >= (SIZEOF_SW_SHARED_DATA / 4)
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_get_rtssm_dump(I2C_MSG *msg, uint32_t *buffer);

/** \brief Get the status of a given link.
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *               `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[in] link_id the link id (0-7)
 * \param[out] link_status pointer to the lane status
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_get_link_status(I2C_MSG *msg, int link_id, kb900x_link_status_t *link_status);

/************** KB900X SMBUS COMMANDS END **************/

/************** KB900X DEBUG TOOLS **************/

/** \brief Reset KB900x.
 *
 * Restart KB900x firmware boot sequence.
 *
 * \note As the boot sequence takes up to 2 seconds to complete, it is recommended to wait at least 2 seconds before interracting with KB900x again.
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *               `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_reset(I2C_MSG *msg);

/**
 * \brief Get the boot status of KB900x firmware.
 *
 * This function gets and check the boot status of the KB900x firmware.
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *               `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[out] is_ready true if firmware ready to switch to SMBus otherwise false
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_is_firmware_ready(I2C_MSG *msg, bool *is_ready);

/**
 * \brief Get the boot status of KB900x firmware.
 *
 * This function gets and check the boot status of the KB900x firmware.
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *               `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[out] entity contains the boot entity
 * \param[out] status contains the boot status of the entity
 *
 * \return error code, E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_get_boot_status(I2C_MSG *msg, kb900x_boot_entity_t *entity,
				      kb900x_boot_status_t *status);

/**
 * \brief Read and return the rev ID of the KB900x chip.
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *               `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[out] revid a pointer for writing the result
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_get_revid(I2C_MSG *msg, uint32_t *revid);

/**
 * \brief Compute the SDS address based on the KB900x chip revid.
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *               `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[out] sds_addr a pointer for writing the result
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_get_sds_addr(I2C_MSG *msg, uint32_t *sds_addr);

/**
 * \brief Dump PHY and RPCS registers to a user-provided pointer,
 * but skip a number of registers first and then only dump a certain number of registers.
 *
 * \note Use this to dump all PHY/RPCS registers in several steps. Useful if
 *       if you cannot allocate enough memory for a complete register dump.
 *
 * \param[in] msg I2C_MSG structure to communicate with KB900X,
 *               `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
 * \param[out] records a non-NULL pointer where to write the dump records,
 *                     should point to a buffer large enough to hold the number
 *                     of records specified in dump_num
 * \param[in] skip_num the number of registers to skip before starting the dump
 * \param[in] dump_num the number of registers to dump
 *
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
kb900x_error_t kb900x_dump_phy_rpcs_registers_with_offset(I2C_MSG *msg,
							  kb900x_register_record_t *records,
							  size_t skip_num, size_t dump_num);

/************** KB900X DEBUG TOOLS END **************/

typedef struct _kb900x_init_arg_ {
	bool is_init;
} kb900x_init_arg;

/**
 * \brief Generic read function.
 *
 * \param[in] cfg a pointer to the sensor_cfg structure that has to contain 
 *      the I2C port, the I2C slave address and the offset that represents the
 *      information to read.
 * \param[out] reading a pointer to the integer (32 bits) used to store the data read
 * 
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
uint8_t kb900x_read(sensor_cfg *cfg, int *reading);

/**
 * \brief Initialize the KB900X driver.
 *
 * \param[in] cfg a pointer to the sensor_cfg structure that has to contain 
 *      the I2C port and the I2C slave address
 * 
 * \return error code, KB900X_E_OK if successful, otherwise an other error code
 */
uint8_t kb900x_init(sensor_cfg *cfg);

#endif
