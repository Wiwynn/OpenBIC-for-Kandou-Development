/*
 * Copyright (c) Kandou-AI.
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
#define KB900X_CCODE_START_READ_FUNC0 (0x82)
#define KB900X_CCODE_END_READ_FUNC0 (0x81)
// Read SMBus register (4 bytes offset)
#define KB900X_CCODE_START_READ_FUNC2 (0x8A)
#define KB900X_CCODE_END_READ_FUNC2 (0x89)
// Write SMBus register (4 bytes offset)
#define KB900X_CCODE_START_END_WRITE_FUNC3 (0x8F)

#define KB900X_I2C_WRITE_BYTCNT (0x02)

#define KB900X_SLAVE_ADDR (0x21)
#define KB900X_MUTEX_LOCK_MS (1000)

#define KB900X_B0_REVID (0x00000010)
#define KB900X_B1_REVID (0x00000011)

// Symbolic offsets for Zephyr's sensor_cfg->offset (kb900x_read function)
#define KB900X_CFG_OFFSET_TEMPERATURE (0x00)

// SMBus command offsets
#define KB900X_SMBUS_OFFSET_GLOB_PARAM_REG_1 (0x0004)
#define KB900X_SMBUS_OFFSET_FW_VERSION (0x0500)
#define KB900X_SMBUS_OFFSET_TEMPERATURE (0x0510)
#define KB900X_SMBUS_OFFSET_RTSSM_DUMP_REQ (0x05C4)
#define KB900X_SMBUS_OFFSET_RTSSM_DUMP_REQ_STATUS (0x05C8)
#define KB900X_SMBUS_OFFSET_RTSSM_DUMP_START_ADDR (0x05CC)
#define KB900X_SMBUS_OFFSET_RTSSM_DUMP_LENGTH (0x05D0)
#define KB900X_SMBUS_OFFSET_PRESET_REQ (0x05D4)
#define KB900X_SMBUS_OFFSET_PRESET_REQ_STATUS (0x05D8)
#define KB900X_SMBUS_OFFSET_PRESET_START_ADDR (0x05DC)
#define KB900X_SMBUS_OFFSET_PRESET_LENGTH (0x05E0)
#define KB900X_SMBUS_OFFSET_BIFURCATION_MODE_INFO (0x06C0)
#define KB900X_SMBUS_OFFSET_LINK_STATUS_GATHER (0x0560)
#define KB900X_SMBUS_OFFSET_LINK_STATUS_READY (0x0564)
#define KB900X_SMBUS_OFFSET_LINK_STATUS (0x0568)
#define KB900X_SMBUS_OFFSET_FW_HEALTH (0x05B0)

#define KB900X_MAX_RETRY (3)
#define KB900X_REQ_MAX_RETRY (10)
#define KB900X_REQ_STATUS_DELAY_MS (2)
#define KB900X_REGISTER_VALUE_WIDTH (4)
#define KB900X_REGISTER_ADDRESS_WIDTH (4)

#define ABSOLUTE_ZER0 (-273.15)
#define KB900X_FLOAT_PRECISION (16)
#define BITS_IN_BYTE (8)

#define KB900X_SIDE_A (0)
#define KB900X_SIDE_B (1)
#define KB9003_NUM_LANES (16)
#define KB9003_HW_RTSSM_LOGGERS (8)
#define KB900X_VENDOR_ID_LENGTH (7)

#define KB900X_TX_FIFO_DEPTH (24)
#define KB900X_RX_FIFO_DEPTH (24)

// I2C Master interface registers
#define kb900x_ee_IC_CON (0xe0081000)
#define kb900x_ee_IC_TAR (0xe0081004)
#define kb900x_ee_IC_DATA_CMD (0xe0081010)
#define kb900x_ee_IC_INTR_MASK (0xe0081030)
#define kb900x_ee_IC_RAW_INTR_STAT (0xe0081034)
#define kb900x_ee_IC_ENABLE (0xe008106c)
#define kb900x_ee_IC_STATUS (0xe0081070)
#define kb900x_ee_IC_TX_ABRT_SOURCE (0xe0081080)
#define kb900x_cfg_top_vd_bump_0 (0xe048018c)
#define kb900x_cfg_top_vd_bump_1 (0xe0480190)

// Other registers
#define kb900x_cpu_periph_clk_gate_en                                                              \
	(0xe009005c) // I2C Master clock - Clock gate enables per peripheral
#define kb900x_cpu_system (0xe0090008)
#define kb900x_tx_abrt_clr (0xe0081054) // Clear interrupts
#define kb900x_cfg_top_revid (0xe0480004)
#define kb900x_smbus_mux (0xe0480008)

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
	KB900X_E_TIMEOUT = -7, /**< I2C Master Timeout */
	KB900X_E_FW_WRITE_ERROR = -8, /**< Firmware write error */
	KB900X_E_COMM = -9, /**< Communication error */
	KB900X_E_FW_NOT_READY = -10, /**< Firmware not ready to switch to SMBus */
	KB900X_E_FW_ERROR = -11, /**< Firmware error */
	KB900X_E_INVALID_DATA = -12, /**< Invalid data */
	KB900X_E_OP_NOT_SUPPPORTED_FW =
		-13, /**< Current version of the firmware does not support this operation */
} kb900x_error_t;

/**
  * \brief Type for the write function (SMBus, I2C or mock)
  */
typedef kb900x_error_t (*KB900X_WRITE_REGISTER_OPERATION)(I2C_MSG *msg, uint32_t address,
							  uint32_t value);

/**
  * \brief Type for the read function (SMBus, I2C or mock)
  */
typedef kb900x_error_t (*KB900X_READ_REGISTER_OPERATION)(I2C_MSG *msg, uint32_t address,
							 uint32_t *value);

/**
  * \brief Communication abstraction structure
  *
  * This structure abstracts the I2C/SMBus write and read functions. This allows the
  * use of different I2C/SMBus drivers and also to mock the transactions.
  */
typedef struct {
	KB900X_WRITE_REGISTER_OPERATION write; /**< Pointer to the write function */
	KB900X_READ_REGISTER_OPERATION read; /**< Pointer to the read function */
} KB900X_REGISTER_IO;

extern KB900X_REGISTER_IO kb900x_register_io;

/**
  * \brief Vendor ID array
  */
extern const uint8_t KB900X_VENDOR_ID[KB900X_VENDOR_ID_LENGTH];

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
	KB900X_COMM_RAW_I2C = 0, /**< I2C */
	KB900X_COMM_SMBUS, /**< SMBus */
} kb900x_communication_mode_t;

/**
  * \brief Boot status of the KB900X device.
  */
typedef enum {
	KB900X_STATE_INIT = 0, /**< Initialization */
	KB900X_STATE_READY = 1, /**< Ready */
	KB900X_STATE_ERROR = 2, /**< Error occured */
} kb900x_boot_status_t;

/**
  * \brief Firmware boot entity type.
  */
typedef enum {
	KB900X_ENTITY_BOOTROM = 0, /**< ROM */
	KB900X_ENTITY_SBL = 1, /**< Secondary boot loader */
	KB900X_ENTITY_PHY_LOAD = 2, /**< Phy binary loader */
	KB900X_ENTITY_FW = 3 /**< FW */
} kb900x_boot_entity_t;

/**
  * \brief REQ status return type.
  */
typedef enum {
	KB900X_FEATURE_REQ_STATUS_NOT_SET = 0, /**< status_not set */
	KB900X_FEATURE_REQ_STATUS_SUCCESS, /**< Request succeeded */
	KB900X_FEATURE_REQ_STATUS_IN_PROGRESS, /**< Request in progress */
	KB900X_FEATURE_REQ_STATUS_FAILURE, /**< Request failed */
} kb900x_feature_req_status_t;

/**
  * \brief This struct is used to hold the firmware health (get_firmware_health)
  */
typedef union {
	struct {
		uint32_t liveliness : 4; /**< liveliness counter */
		uint32_t : 27; /**< unused */
		uint32_t fw_is_initialized : 1; /**< Firmware is initialized */
	};
	uint32_t raw; /**< Raw value */
} kb900x_fw_health_t;

/**
  * \brief This struct is used to hold the status of a link (get_link_status)
  */
typedef union {
	struct {
		uint32_t link_info : 1; /**< Link info (1 if link is valid) */
		uint32_t l0_reached : 1; /**< L0 reached (1 if L0 is reached) */
		uint32_t link_number : 9; /**< Link number (0-511) */
		uint32_t rtssm_state : 5; /**< RTSSM state (0-31) */
		uint32_t rtssm_speed : 3; /**< RTSSM speed (0-4, 7 if invalid) */
		uint32_t link_width : 5; /**< Link width (1-16) */
		uint32_t : 8; /**< unused */
	};
	uint32_t raw; /**< Raw value */
} kb900x_link_status_t;

/**
  * \brief This struct stores a register address and value tuple, for register dumps.
  */
typedef struct {
	uint32_t address; /**< Register address */
	uint32_t value; /**< Register value */
} kb900x_register_record_t;

/**
  * \brief This enum represents the different bifurcation settings.
  */
typedef enum {
	KB900X_BIFUR_MODE_x16 = 0x00,
	KB900X_BIFUR_MODE_x0x0x8 = 0x01,
	KB900X_BIFUR_MODE_x0x0x0x4 = 0x02,
	KB900X_BIFUR_MODE_x8x8 = 0x03,
	KB900X_BIFUR_MODE_x8x4x4 = 0x04,
	KB900X_BIFUR_MODE_x4x4x8 = 0x05,
	KB900X_BIFUR_MODE_x4x4x4x4 = 0x06,
	KB900X_BIFUR_MODE_x2x2x2x2x2x2x2x2 = 0x07,
	KB900X_BIFUR_MODE_x8x4x2x2 = 0x08,
	KB900X_BIFUR_MODE_x8x2x2x4 = 0x09,
	KB900X_BIFUR_MODE_x2x2x4x8 = 0x0A,
	KB900X_BIFUR_MODE_x4x2x2x8 = 0x0B,
	KB900X_BIFUR_MODE_x2x2x2x2x8 = 0x0C,
	KB900X_BIFUR_MODE_x8x2x2x2x2 = 0x0D,
	KB900X_BIFUR_MODE_x2x2x4x4x4 = 0x0E,
	KB900X_BIFUR_MODE_x4x2x2x4x4 = 0x0F,
	KB900X_BIFUR_MODE_x4x4x2x2x4 = 0x10,
	KB900X_BIFUR_MODE_x4x4x4x2x2 = 0x11,
	KB900X_BIFUR_MODE_x2x2x2x2x4x4 = 0x12,
	KB900X_BIFUR_MODE_x2x2x4x2x2x4 = 0x13,
	KB900X_BIFUR_MODE_x4x2x2x2x2x4 = 0x14,
	KB900X_BIFUR_MODE_x2x2x4x4x2x2 = 0x15,
	KB900X_BIFUR_MODE_x4x2x2x4x2x2 = 0x16,
	KB900X_BIFUR_MODE_x4x4x2x2x2x2 = 0x17,
	KB900X_BIFUR_MODE_x2x2x2x2x2x2x4 = 0x18,
	KB900X_BIFUR_MODE_x2x2x2x2x4x2x2 = 0x19,
	KB900X_BIFUR_MODE_x2x2x4x2x2x2x2 = 0x1A,
	KB900X_BIFUR_MODE_x4x2x2x2x2x2x2 = 0x1B,
	KB900X_BIFUR_MODE_x4x4 = 0x1C,
	KB900X_BIFUR_MODE_x2x2x4 = 0x1D,
	KB900X_BIFUR_MODE_x4x2x2 = 0x1E,
	KB900X_BIFUR_MODE_x2x2x2x2 = 0x1F,
	KB900X_BIFUR_MODE_x2x2 = 0x20,

	// maximum number of modes
	KB900X_BIFUR_MODE_COUNT,
} kb900x_bifurcation_mode_t;

/**
  * \brief Retimer init struct
  */
typedef struct {
	bool is_init; /**< True if initialized */
} kb900x_init_arg;

/**
  * \brief Type representing a single entry in a log.
  *
  * \note The delta parameter must be interpreted using the delta_to_ms function.
  */
typedef union {
	struct {
		uint16_t rtssm : 6; /**< The RTSSM state */
		uint16_t data_rate : 3; /**< The data rate in range 0 to 4 (invalid =7, PCIe Gen 5 = 4, PCIe Gen4 = 3, ...)*/
		uint16_t delta : 7; /**< The time delta */
	};
	uint16_t raw; /**< Raw value */
} kb900x_hw_rtssm_entry_t;

/**
  * \brief Type representing a log map info.
  */
typedef union {
	struct {
		uint32_t tile_id : 8; /**< The tile ID */
		uint32_t rpcs_id : 8; /**< The RPCS ID */
		uint32_t rtssm_cfg_id : 8; /**< The RTSSM config ID */
		uint32_t curr_pos : 4; /**< The current position */
		uint32_t era : 4; /**< The era */
	};
	uint32_t raw; /**< Raw value */
} kb900x_hw_rtssm_log_map_info_t;

/**
  * \brief Struct representing all RTSSM entries of a single RPCS
  */
typedef struct {
	kb900x_hw_rtssm_log_map_info_t log_map_info; /**< Log map information */
	kb900x_hw_rtssm_entry_t entries[32]; /**< Array of RTSSM entries */
} kb900x_hw_rtssm_log_t;

/**
  * \brief Struct representing all the RTSSM logs on a device
  */
typedef struct {
	kb900x_hw_rtssm_log_t logs[8]; /**< Array of RTSSM logs */
} kb900x_hw_rtssm_logs_t;

/**
  * \brief The preset config structure
  */
typedef struct {
	uint8_t lane_id; /**< The lane ID in range 0 to 15 */
	uint8_t data_rate; /**< The data rate in range 0 to 4 (PCIe Gen 5 = 4, PCIe Gen4 = 3, ...) */
	uint8_t pad[2]; /**< Unused */
} kb900x_preset_config_t;

/**
  * \brief The preset config structure
  *
  * It contains the different requested presets.
  */
typedef struct {
	uint8_t rt_rc; /**< Local upstream preset (retimer -> root complex) in range 0 to 10 */
	uint8_t rt_ep; /**< Local downstream preset (retimer -> endpoint) in range 0 to 10 */
	uint8_t rc_rt; /**< Partner upstream preset (root compex -> retimer) in range 0 to 10 */
	uint8_t ep_rt; /**< Partner downstream preset (endpoint -> retimer) in range 0 to 10 */
} kb900x_preset_data_t;

/**
  * \brief The presets for a single lane structure
  */
typedef struct {
	kb900x_preset_config_t config; /**< Preset config */
	kb900x_preset_data_t data; /**< Config info */
} kb900x_presets_t;

/**
  * \brief The presets for all lanes structure
  */
typedef struct {
	kb900x_presets_t lanes[KB9003_NUM_LANES]; /**< Array of presets for each lane */
} kb900x_all_presets_t;

// Global variable to store the selected slave address
extern uint8_t kb900x_eeprom_slave_addr;

/**
  * \brief Detect the communication mode and set it.
  *
  * This function detects the communication mode of the retimer and set it.
  *
  * It starts by reading a register (read only) by sending a SMBus command to the retimer.
  *
  * If the communication fails due to a wrong PEC (Parity Error Check), it falls back to using raw I2C.
  *
  * The same register is then read again but this time using raw I2C. If the second read succeeds,
  * it means that the retimer is in I2C mode and the communication works as expected.
  *
  * \note This function can be used when you want to ensure that your application is communicating with the retimer in the correct mode.
  *
  * \param[in] msg I2C_MSG structure to communicate with KB900X.
  *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
  * \param[out] mode the communication mode
  *
  * \return error code, KB900X_E_OK if successful, otherwise an other error code
  */
kb900x_error_t kb900x_detect_communication_mode(I2C_MSG *msg, kb900x_communication_mode_t *mode);

/**
  * \brief Enable SMBus communication mode by toggling the MUX between TWI and SMBus.
  *
  * This function first checks if the retimer is ready by reading the FW status. If the retimer is not ready, it returns an error.
  * If the retimer is ready, it proceeds to enable the SMBus communication mode.
  * It will send a specific command to the retimer to toggle the SMBus/I2C MUX.
  *
  * As the retimer has a delay after switching the MUX, the function waits for a certain amount of time before returning.
  *
  * \note This feature interracts with the firmware to enable the SMBus communication mode.
  *
  * \param[in] msg I2C_MSG structure to communicate with KB900X.
  *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
  *
  * \return error code, KB900X_E_OK if successful, otherwise an other error code
  */
kb900x_error_t kb900x_enable_smbus(I2C_MSG *msg);

/**
  * \brief Run the SMBus initialization sequence.
  *
  * This function will start by detecting the retimer communication mode.
  *
  * If the communication mode is SMBus, it returns. If the communication is I2C, it will proceed to check the boot status
  *  and enable the SMBus communication mode.
  *
  * \note This feature interract with the firmware to enable the SMBus communication mode.
  *
  * \param[in] msg I2C_MSG structure to communicate with KB900X.
  *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
  *
  * \return error code, KB900X_E_OK if successful, otherwise an other error code
  */
kb900x_error_t kb900x_init_smbus(I2C_MSG *msg);

/**
  * \brief Initialize the KB900X I2C master interface
  *
  * \param[in] msg I2C_MSG structure to communicate with KB900X.
  *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
  * \param[in] slave_addr the default I2C slave address KB900X's I2C master interface should communicate with
  *
  * \return error code, KB900X_E_OK if successful, otherwise an other error code
  */
kb900x_error_t kb900x_i2c_master_init(I2C_MSG *msg, uint8_t slave_addr);

/**
  * \brief Read from the KB900X EEPROM.
  *
  * \param[in] msg I2C_MSG structure to communicate with KB900X.
  *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
  * \param[in] addr the EEPROM address to start reading from
  * \param[in] length the number of bytes to read
  * \param[out] result a pointer to an array to store the read data
  * \param[in] config the EEPROM configuration, see kb900x_eeprom_config_t for more details
  *
  * \return error code, KB900X_E_OK if successful, otherwise an other error code
  */
kb900x_error_t kb900x_read_firmware(I2C_MSG *msg, uint32_t addr, size_t length, uint8_t *result,
				    const kb900x_eeprom_config_t *config);

/**
  * \brief Compare the content of the EEPROM with expected data.
  *
  * \param[in] msg I2C_MSG structure to communicate with KB900X.
  *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
  * \param[in] offset the offset in the EEPROM to start reading from
  * \param[in] buffer the buffer containing the expected firmware, starting at `offset` offset in the EEPROM
  * \param[in] buffer_size the size of the buffer and of the data chunk to read and compare
  * \param[in] config the EEPROM config structure
  *
  * \return 0 if no error, else the error code
  */
kb900x_error_t kb900x_check_firmware(I2C_MSG *msg, uint32_t offset, uint8_t *buffer,
				     uint32_t buffer_size, const kb900x_eeprom_config_t *config);

/**
  * \brief Flash the firmware.
  *
  * This function writes the provided payload to the specified EEPROM address.
  *
  * An EEPROM configuration parameter has to be provided
  *
  * \param[in] msg I2C_MSG structure to communicate with KB900X.
  *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
  * \param[in] addr the EEPROM address to start writing to
  * \param[in] payload the data to write
  * \param[in] payload_size the payload length
  * \param[in] config the EEPROM configuration, see kb900x_eeprom_config_t for more details
  *
  * \return error code, KB900X_E_OK if successful, otherwise an other error code
  */
kb900x_error_t kb900x_flash_firmware(I2C_MSG *msg, uint32_t addr, uint8_t *payload,
				     size_t payload_size, const kb900x_eeprom_config_t *config);

/**
  * \brief Get the KB900X temperature.
  *
  * This function reads the temperature on all the lanes. It finds the maximum temperature and returns it.
  *
  * The temperature is returned as a float value in Celsius.
  * If there was an error during the communication or while parsing the temperature, the function will return an error code.
  *
  * \param[in] msg I2C_MSG structure to communicate with KB900X.
  *               `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
  * \param[out] temperature a pointer to the float used to store the temperature
  *
  * \return error code, KB900X_E_OK if successful, otherwise an other error code
  */
kb900x_error_t kb900x_get_temperature(I2C_MSG *msg, float *temperature);

/**
  * \brief Get the temperature of a KB900X lane.
  *
  * This function gets the temperature of the sensor close to the provided lane.
  *
  * There are 16 sensors in total on the KB9003 (1 sensor for 2 lanes).
  *
  * The port parameter is used to select from which side of the retimer to get
  * the temperature from (0 = upstream, 1 = downstream).
  *
  * \param[in] msg I2C_MSG structure to communicate with KB900X.
  *               `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
  * \param[in] port the port, 0 = A (upstream), anything else = B (downstream)
  * \param[in] lane the lane id, 0 to 15
  * \param[out] temperature pointer to the temperature[Celsius]
  *
  * \return error code, KB900X_E_OK if successful, otherwise an other error code
  */
kb900x_error_t kb900x_get_lane_temperature(I2C_MSG *msg, int port, int lane, float *temperature);

/**
  * \brief Dump the RTSSM data area into a buffer.
  *
  * This function reads the HW RTSSM logs from the 8 loggers and stores them in the provided logs structure.
  *
  * See \ref kb900x_hw_rtssm_logs_t for more information about the logs.
  *
  * \param[in] msg I2C_MSG structure to communicate with KB900X.
  *               `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
  * \param[out] logs a pointer to the HW RTSSM logs
  *
  * \return error code, KB900X_E_OK if successful, otherwise an other error code
  */
kb900x_error_t kb900x_get_hw_rtssm_logs(I2C_MSG *msg, kb900x_hw_rtssm_logs_t *logs);

/**
  * \brief Read the TX presets.
  *
  * This function reads the TX presets from the KB900x device and stores them in the provided `presets` parameter.
  * The function returns an error code indicating whether the operation was successful or not.
  *
  * See \ref kb900x_all_presets_t for more information on the structure that holds the presets.
  *
  * \param[in] msg I2C_MSG structure to communicate with KB900X.
  *               `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
  * \param[out] logs a pointer to kb900x_all_presets_t
  *
  * \return error code, KB900X_E_OK if successful, otherwise an other error code
  */
kb900x_error_t kb900x_get_tx_presets(I2C_MSG *msg, kb900x_all_presets_t *presets);

/**
  * \brief Get the firmware health.
  *
  * This function reads the firmware health status from the KB900x device. The result is stored in the provided
  * `firmware_health` parameter. The function returns an error code indicating whether the operation was successful or not.
  *
  * The firmware health status includes information such as a liveliness counter or a FW initialized flag.
  *
  * See \ref kb900x_fw_health_t for more details.
  *
  * \param[in] msg I2C_MSG structure to communicate with KB900X.
  *               `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
  * \param[out] firmware_health pointer to the firmware health
  *
  * \return error code, KB900X_E_OK if successful, otherwise an other error code
  */
kb900x_error_t kb900x_get_firmware_health(I2C_MSG *msg, kb900x_fw_health_t *firmware_health);

/**
  * \brief Reset KB900x.
  *
  * Restart KB900x firmware boot sequence.
  *
  * \note As the boot sequence takes some time to complete, it is recommended to wait at least 2 seconds before
  * interracting with KB900x again.
  *
  * \param[in] msg I2C_MSG structure to communicate with KB900X.
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
  * \param[in] msg I2C_MSG structure to communicate with KB900X.
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
  * The entity indicates from which part of the boot process the status is retrieved.
  *
  * The entity can be one of the following:
  * - KB900X_ENTITY_BOOTROM: Indicates the status is retrieved from the ROM
  * - KB900X_ENTITY_SB: Indicates the status is retrieved from the Secondary boot loader
  * - KB900X_ENTITY_PHY_LOAD: Indicates the status is retrieved from the Phy binary loader
  * - KB900X_ENTITY_FW: Indicates the status is retrieved from the FW
  *
  * The status indicates whether the boot process is in progress, ready or failed.
  *
  * If the entity is KB900X_ENTITY_FW and the status is KB900X_STATE_READY, then the firmware is ready to switch to SMBus.
  *
  * \param[in] msg I2C_MSG structure to communicate with KB900X.
  *               `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
  * \param[out] entity contains the boot entity
  * \param[out] status contains the boot status of the entity
  *
  * \return error code, E_OK if successful, otherwise an other error code
  */
kb900x_error_t kb900x_get_boot_status(I2C_MSG *msg, kb900x_boot_entity_t *entity,
				      kb900x_boot_status_t *status);

/**
  * \brief Get the link status for a given link id.
  *
  * This function retrieves the current status of a link. The link status includes information such as the link speed (gen),
  * link width, and if L0 is reached.
  *
  * The link_id parameter specifies which link's status should be retrieved.
  * The maximum number of links can be obtained using kb900x_get_max_nb_links().
  *
  * The link_status parameter points to a structure where the link status will be stored.
  *
  * \note The link_status structure should be properly initialized before calling this function.
  * This function assumes that the link_id is valid and within the range of the maximum number of links.
  *
  * \param[in] msg I2C_MSG structure to communicate with KB900X.
  *               `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
  * \param[in] link_id the link id, kb900x_get_max_nb_links() can be used to get the maximum number of links
  *
  * \param[out] link_status a pointer to the link status structure
  *
  * \return error code, KB900X_E_OK if successful, otherwise an other error code
  */
kb900x_error_t kb900x_get_link_status(I2C_MSG *msg, int link_id, kb900x_link_status_t *link_status);

/**
  * \brief Dump PHY and RPCS registers to a user-provided pointer,
  * but skip a number of registers first and then only dump a certain number of registers.
  *
  * \note Use this to dump all PHY/RPCS registers in several steps. Useful if
  *       if you cannot allocate enough memory for a complete register dump.
  *
  * \param[in] msg I2C_MSG structure to communicate with KB900X.
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

//////////////////////////////////////////////////////////////////////////////////
// OpenBIC specific functions
//////////////////////////////////////////////////////////////////////////////////

/**
  * \brief Initialize the KB900X driver.
  *
  * \param[in] cfg a pointer to the sensor_cfg structure that has to contain
  *      the I2C port and the I2C slave address
  *
  * \return error code, KB900X_E_OK if successful, otherwise an other error code
  */
uint8_t kb900x_init(sensor_cfg *cfg);

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
  * \brief Get the KB900X firmware version.
  *
  * \param[in] msg I2C_MSG structure to communicate with KB900X.
  *               `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
  * \param[out] version a pointer to the uint8_t array used to store the firmware version;
  *              the array must be at least 4 bytes long
  *
  * \return true if successful, otherwise false. Error information will be printed in the logs.
  */
bool kb900x_get_fw_version(I2C_MSG *msg, uint8_t *version);

/**
  * \brief Get the KB900X vendor ID.
  *
  * \param[in] msg I2C_MSG structure to communicate with KB900X.
  *               `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
  * \param[out] vendor_id a pointer to the integer used to store the vendor ID
  *
  * \return true if successful, otherwise false. Error information will be printed in the logs.
  */
bool kb900x_get_vendor_id(I2C_MSG *msg, int *vendor_id);

/**
  * \brief Write the KB900X firmware to the KB900X firmware EEPROM.
  *
  * \param[in] msg I2C_MSG structure to communicate with KB900X.
  *                `msg->target_addr` and `msg->bus` must be set by the caller to point to KB900X
  * \param[in] offset the offset in the EEPROM to start writing to
  * \param[in] msg_len the length of the data to write
  * \param[in] msg_buf the buffer containing the firmware to write
  * \param[in] flag unused
  *
  * \return 0 if no error, else the error code
  */
uint8_t kb900x_pcie_retimer_fw_update(I2C_MSG *msg, uint32_t offset, uint32_t msg_len,
				      uint8_t *msg_buf, uint8_t flag);

#endif
