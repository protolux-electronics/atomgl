/*
 * This file is part of AtomGL.
 *
 * Copyright 2020-2026 Davide Bettio <davide@uninstall.it>
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

#include "dcs_lcd_commands.h"

#include <stdlib.h>

#include <freertos/FreeRTOS.h>

#include <driver/spi_master.h>
#include <esp_heap_caps.h>

void dcs_lcd_set_paint_area(struct SPIDCBus *bus, const struct DCSLCDScreen *screen,
    int x, int y, int width, int height)
{
    x += screen->x_offset;
    y += screen->y_offset;

    spi_dc_writecommand(bus, DCS_LCD_CASET);
    spi_device_acquire_bus(bus->spi_disp.handle, portMAX_DELAY);
    spi_display_write(&bus->spi_disp, 32, (x << 16) | ((x + width) - 1));
    spi_device_release_bus(bus->spi_disp.handle);

    spi_dc_writecommand(bus, DCS_LCD_PASET);
    spi_device_acquire_bus(bus->spi_disp.handle, portMAX_DELAY);
    spi_display_write(&bus->spi_disp, 32, (y << 16) | ((y + height) - 1));
    spi_device_release_bus(bus->spi_disp.handle);
}

void dcs_lcd_draw_buffer(struct SPIDCBus *bus, const struct DCSLCDScreen *screen,
    int pixel_bytes, int x, int y, int width, int height, const void *imgdata)
{
    const uint16_t *data = imgdata;

    dcs_lcd_set_paint_area(bus, screen, x, y, width, height);

    spi_dc_writecommand(bus, DCS_LCD_RAMWR);

    int dest_size = width * height;
    int chunks = dest_size / 1024;

    spi_device_acquire_bus(bus->spi_disp.handle, portMAX_DELAY);

    if (pixel_bytes == 2) {
        int buf_pixel_size = (dest_size > 1024) ? 1024 : dest_size;
        uint16_t *tmpbuf = heap_caps_malloc(buf_pixel_size * sizeof(uint16_t), MALLOC_CAP_DMA);

        for (int i = 0; i < chunks; i++) {
            const uint16_t *data_b = data + 1024 * i;
            for (int j = 0; j < 1024; j++) {
                tmpbuf[j] = SPI_SWAP_DATA_TX(data_b[j], 16);
            }
            spi_display_dmawrite(&bus->spi_disp, 1024 * sizeof(uint16_t), tmpbuf);
        }

        int last_chunk_size = dest_size - chunks * 1024;
        if (last_chunk_size) {
            const uint16_t *data_b = data + chunks * 1024;
            for (int j = 0; j < last_chunk_size; j++) {
                tmpbuf[j] = SPI_SWAP_DATA_TX(data_b[j], 16);
            }
            spi_display_dmawrite(&bus->spi_disp, last_chunk_size * sizeof(uint16_t), tmpbuf);
        }

        free(tmpbuf);

    } else {
        // ILI9488: RGB565 -> RGB888 (3 bytes/pixel).
        const int chunk_pixels = 512;
        uint8_t *tmpbuf = heap_caps_malloc(chunk_pixels * 3, MALLOC_CAP_DMA);

        int i = 0;
        while (i < dest_size) {
            int n = (dest_size - i > chunk_pixels) ? chunk_pixels : (dest_size - i);

            for (int j = 0; j < n; j++) {
                uint16_t px = data[i + j];
                uint8_t r5 = (px >> 11) & 0x1F;
                uint8_t g6 = (px >> 5) & 0x3F;
                uint8_t b5 = (px >> 0) & 0x1F;

                uint8_t r8 = (r5 << 3) | (r5 >> 2);
                uint8_t g8 = (g6 << 2) | (g6 >> 4);
                uint8_t b8 = (b5 << 3) | (b5 >> 2);

                tmpbuf[j * 3 + 0] = r8;
                tmpbuf[j * 3 + 1] = g8;
                tmpbuf[j * 3 + 2] = b8;
            }

            spi_display_dmawrite(&bus->spi_disp, n * 3, tmpbuf);
            i += n;
        }

        free(tmpbuf);
    }

    spi_device_release_bus(bus->spi_disp.handle);
}
