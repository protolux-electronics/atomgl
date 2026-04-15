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

#ifndef _OLED_COMMANDS_H_
#define _OLED_COMMANDS_H_

#include <stddef.h>
#include <stdint.h>

#include <driver/i2c.h>

// --- Init sequence byte-array format ---
//
// Each entry:  [CMD] [FLAGS_LEN] [DATA_0 ... DATA_N] [DELAY_MS]
//   CMD:        OLED command byte
//   FLAGS_LEN:  bits 6:0 = data byte count (0-127)
//               bit 7    = delay flag (DELAY_MS byte follows data)
//   DELAY_MS:   delay in milliseconds (0-255), present only if flag set
//
// The sequence is length-bounded: callers pass seq_len and the
// executor walks the buffer until exhausted.  The format
// intentionally matches epaper_commands.h so the codebase converges
// on one init-sequence convention; current OLED init steps don't
// need delays, but future variants can opt in without a format
// change.
//
// Driver-controlled finalization (the optional "invert" command and
// the unconditional "display ON") is NOT encoded in init sequences;
// it stays in the driver because it depends on a runtime opt.

#define OLED_INIT_SEQ_DELAY 0x80

// Execute an init sequence over an I2C bus.  Owns the transaction
// boundary: emits one i2c_cmd_link transaction per init step so that
// the optional inter-step delay (vTaskDelay) actually waits between
// commands rather than being a no-op inside a queued cmd_link.  Uses
// the SSD13xx command-stream control byte (Co=0, D/C#=0) so a
// command and its parameters can ride together in one transaction.
void oled_execute_init_seq(i2c_port_t i2c_num, uint8_t i2c_addr,
    const uint8_t *seq, size_t seq_len);

// Built-in init sequences.  SSD1306 and SH1106 share the minimal
// 4-step charge-pump/remap init; SSD1315 has its own full reset
// sequence derived from u8g2.  Each array is paired with a size_t
// constant giving its length; callers pass both to
// oled_execute_init_seq().
extern const uint8_t oled_init_seq_ssd1306[];
extern const size_t oled_init_seq_ssd1306_len;
extern const uint8_t oled_init_seq_ssd1315[];
extern const size_t oled_init_seq_ssd1315_len;

#endif
