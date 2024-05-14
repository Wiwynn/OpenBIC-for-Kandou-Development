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

#define I2C_READ_CCODE_START 0x82
#define I2C_READ_CCODE_END 0x81
#define I2C_WRITE_BYTCNT 0x02

#define KB900X_SLAVE_ADDR 0x21
#define KB900X_MUTEX_LOCK_MS 1000

#define OFFSET_GLOB_PARAM_REG_1 0x0004
#define OFFSET_FW_VERSION 0x0500
#define OFFSET_TEMPERATURE 0x0510

#define MAX_RETRY 3

#define ABSOLUTE_ZER0 (-273.15)
#define FLOAT_PRECISION (16)

bool get_vendor_id(I2C_MSG *msg);
bool get_fw_version(I2C_MSG *msg, uint8_t *version);
bool get_temperature(I2C_MSG *msg, float *temperature);

#endif
