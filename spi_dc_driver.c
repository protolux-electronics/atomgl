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

#include "spi_dc_driver.h"

#include <freertos/FreeRTOS.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>

void spi_dc_write_data(struct SPIDCBus *bus, uint32_t data)
{
    spi_device_acquire_bus(bus->spi_disp.handle, portMAX_DELAY);
    spi_display_write(&bus->spi_disp, 8, data);
    spi_device_release_bus(bus->spi_disp.handle);
}

void spi_dc_write_command(struct SPIDCBus *bus, uint8_t cmd)
{
    gpio_set_level(bus->dc_gpio, 0);
    spi_dc_write_data(bus, cmd);
    gpio_set_level(bus->dc_gpio, 1);
}

void spi_dc_write_cmd_data(struct SPIDCBus *bus, uint8_t cmd, const uint8_t *data, size_t data_len)
{
    spi_dc_write_command(bus, cmd);
    for (int i = 0; i < data_len; i++) {
        spi_dc_write_data(bus, data[i]);
    }
}

void spi_dc_write_data_n(struct SPIDCBus *bus, const uint8_t *data, size_t data_len)
{
    for (size_t i = 0; i < data_len; i++) {
        spi_dc_write_data(bus, data[i]);
    }
}
