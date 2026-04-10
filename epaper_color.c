/*
 * This file is part of AtomGL.
 *
 * Copyright 2022-2026 Davide Bettio <davide@uninstall.it>
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

#include "epaper_color.h"

#include <limits.h>
#include <math.h>
#include <stdint.h>

const uint8_t epaper_acep_palette[7][3] = {
    { 0x00, 0x00, 0x00 },
    { 0xFF, 0xFF, 0xFF },
    { 0x00, 0xFF, 0x00 },
    { 0x00, 0x00, 0xFF },
    { 0xFF, 0x00, 0x00 },
    { 0xFF, 0xFF, 0x00 },
    { 0xFF, 0x80, 0x00 }
};

const uint8_t epaper_gdep073e01_palette[7][3] = {
    { 0x19, 0x1E, 0x21 },
    { 0xE8, 0xE8, 0xE8 },
    { 0xEF, 0xDE, 0x44 },
    { 0xB2, 0x13, 0x18 },
    { 0xE8, 0xE8, 0xE8 },
    { 0x21, 0x57, 0xBA },
    { 0x12, 0x5F, 0x20 }
};

static inline float square(float p)
{
    return p * p;
}

uint8_t epaper_dither_acep7(int x, int y, uint8_t r, uint8_t g, uint8_t b,
    const uint8_t palette[][3], int palette_size)
{
    const uint8_t m[4][4] = {
        { 0, 8, 2, 10 },
        { 12, 4, 14, 6 },
        { 3, 11, 1, 9 },
        { 15, 7, 13, 5 }
    };

    // following r parameters have been found using standard deviation
    // that gives a decent result
    int r1 = r + roundf(92.0 * ((float) m[x % 4][y % 4] * 0.0625 - 0.5));
    int g1 = g + roundf(85.0 * ((float) m[x % 4][y % 4] * 0.0625 - 0.5));
    int b1 = b + roundf(65.0 * ((float) m[x % 4][y % 4] * 0.0625 - 0.5));

    float min = INT_MAX;
    int min_index = 0;

    for (int i = 0; i < palette_size; i++) {
        int r2 = palette[i][0];
        int g2 = palette[i][1];
        int b2 = palette[i][2];

#ifdef NO_WEIGHTS
        float d = square((r2 - r1)) + square((g2 - g1)) + square((b2 - b1));
#else
        float d = square((r2 - r1) * 0.30) + square((g2 - g1) * 0.59) + square((b2 - b1) * 0.11);
#endif

        if (d < min) {
            min = d;
            min_index = i;
        }
    }

    return min_index;
}
