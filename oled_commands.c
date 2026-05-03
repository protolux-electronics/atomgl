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

#include "oled_commands.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// SSD13xx I2C control byte (Co=0, D/C#=0): all subsequent bytes in
// this transaction are interpreted as command bytes (commands and
// their inline parameters).
#define OLED_CTRL_CMD_STREAM 0x00

void oled_execute_init_seq(i2c_port_t i2c_num, uint8_t i2c_addr,
    const uint8_t *seq, size_t seq_len)
{
    const uint8_t *end = seq + seq_len;
    while (seq < end) {
        uint8_t cmd_byte = *seq++;
        uint8_t flags_len = *seq++;
        uint8_t len = flags_len & 0x7F;

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, OLED_CTRL_CMD_STREAM, true);
        i2c_master_write_byte(cmd, cmd_byte, true);
        for (uint8_t i = 0; i < len; i++) {
            i2c_master_write_byte(cmd, seq[i], true);
        }
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(i2c_num, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        seq += len;

        if (flags_len & OLED_INIT_SEQ_DELAY) {
            vTaskDelay(*seq++ / portTICK_PERIOD_MS);
        }
    }
}

// --- Built-in init sequences ---
//
// Transcribed mechanically from the original display_init() body in
// ssd1306_display_driver.c.  _Static_assert on sizeof guards against
// miscounted data bytes.

// clang-format off

// SSD1306 / SH1106 minimal init.  The post-reset defaults are mostly
// correct for these controllers; the only adjustments needed are
// charge-pump enable and the segment-remap / COM-scan flip used by
// most modules.
const uint8_t oled_init_seq_ssd1306[] = {
    0x8D, 1,  0x14,                    // SET_CHARGE_PUMP + enable
    0xA1, 0,                           // SET_SEGMENT_REMAP (column 127 -> SEG0)
    0xC8, 0,                           // SET_COM_SCAN_MODE (remapped)
};
_Static_assert(sizeof(oled_init_seq_ssd1306) == 7,
    "oled_init_seq_ssd1306: miscounted bytes");
const size_t oled_init_seq_ssd1306_len = sizeof(oled_init_seq_ssd1306);

// SSD1315 full init.  Derived from the u8g2 project (BSD-2-Clause)
// per the comment in the original driver.  Standard hardware
// initialization commands defined by the Solomon Systech SSD1315
// datasheet.
const uint8_t oled_init_seq_ssd1315[] = {
    0xAE, 0,                           // Display OFF
    0xD5, 1,  0x80,                    // Display Clock Divide Ratio (0x80 standard)
    0xA8, 1,  0x3F,                    // Multiplex Ratio (64 MUX)
    0xD3, 1,  0x00,                    // Display Offset (none)
    0x40, 0,                           // Display Start Line = 0
    0x8D, 1,  0x14,                    // Charge Pump (enable)
    0xA1, 0,                           // Segment Remap
    0xC8, 0,                           // COM Scan Mode
    0xDA, 1,  0x12,                    // COM Pins Hardware Configuration (alternative)
    0x81, 1,  0xCF,                    // Contrast Control (high contrast per u8x8)
    0xD9, 1,  0xF1,                    // Pre-charge Period (required for 400 kHz stability)
    0xDB, 1,  0x40,                    // VCOMH Deselect Level (~0.77x VCC)
    0xA4, 0,                           // Resume to RAM content display
    0xA6, 0,                           // Normal Display (not inverted)
    0xAD, 1,  0x10,                    // Internal IREF Setting
};
_Static_assert(sizeof(oled_init_seq_ssd1315) == 39,
    "oled_init_seq_ssd1315: miscounted bytes");
const size_t oled_init_seq_ssd1315_len = sizeof(oled_init_seq_ssd1315);

// clang-format on

// --- Per-controller descriptors ---

const struct OLEDDesc oled_desc_ssd1306 = {
    .name                       = "Solomon Systech SSD1306",
    .native_width               = 128,
    .native_height              = 64,
    .i2c_address                = 0x3C,

    .init_seq                   = oled_init_seq_ssd1306,
    .init_seq_len               = sizeof(oled_init_seq_ssd1306),

    .column_reset_per_page      = false,
    .scanline_prefix_pad_bytes  = 0,
};

const struct OLEDDesc oled_desc_ssd1315 = {
    .name                       = "Solomon Systech SSD1315",
    .native_width               = 128,
    .native_height              = 64,
    .i2c_address                = 0x3C,

    .init_seq                   = oled_init_seq_ssd1315,
    .init_seq_len               = sizeof(oled_init_seq_ssd1315),

    .column_reset_per_page      = true,
    .scanline_prefix_pad_bytes  = 0,
};

const struct OLEDDesc oled_desc_sh1106 = {
    .name                       = "Sino Wealth SH1106",
    .native_width               = 128,
    .native_height              = 64,
    .i2c_address                = 0x3C,

    // SH1106 shares the SSD1306 minimal init.
    .init_seq                   = oled_init_seq_ssd1306,
    .init_seq_len               = sizeof(oled_init_seq_ssd1306),

    .column_reset_per_page      = true,
    .scanline_prefix_pad_bytes  = 2,
};
