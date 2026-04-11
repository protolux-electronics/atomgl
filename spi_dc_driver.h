/*
 * This file is part of AtomGL.
 *
 * Copyright 2026 Davide Bettio <davide@uninstall.it>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SPI_DC_DRIVER_H_
#define _SPI_DC_DRIVER_H_

#include <stddef.h>
#include <stdint.h>

#include "spi_display.h"

struct SPIDCBus
{
    struct SPIDisplay spi_disp;
    int dc_gpio;
};

void spi_dc_write_data(struct SPIDCBus *bus, uint32_t data);
void spi_dc_write_command(struct SPIDCBus *bus, uint8_t cmd);
void spi_dc_write_cmd_data(struct SPIDCBus *bus, uint8_t cmd, const uint8_t *data, size_t data_len);
void spi_dc_write_data_n(struct SPIDCBus *bus, const uint8_t *data, size_t data_len);

#endif
