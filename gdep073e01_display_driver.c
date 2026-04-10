/*
 * This file is part of AtomGL.
 *
 * Copyright 2025 Davide Bettio <davide@uninstall.it>
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

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <freertos/task.h>

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include <sys/time.h>

#include <defaultatoms.h>
#include <interop.h>
#include <mailbox.h>
#include <port.h>
#include <term.h>

#include <esp32_sys.h>

#define DISPLAY_WIDTH 800
#define DISPLAY_HEIGHT 480

#include "display_items.h"
#include "display_message.h"
#include "display_task.h"
#include "display_common.h"
#include "draw_common.h"
#include "font_data.h"
#include "image_helpers.h"
#include "spi_display.h"

#define PSR         0x00
#define PWRR        0x01
#define POF         0x02
#define POFS        0x03
#define PON         0x04
#define BTST1       0x05
#define BTST2       0x06
#define DSLP        0x07
#define BTST3       0x08
#define DTM         0x10
#define DRF         0x12
#define PLL         0x30
#define CDI         0x50
#define TCON        0x60
#define TRES        0x61
#define REV         0x70
#define VDCS        0x82
#define T_VDCS      0x84
#define PWS         0xE3

#define CHAR_WIDTH 8

#define REPORT_UNEXPECTED_MSGS 0
#define CHECK_OVERFLOW 1
#define SELF_TEST 0

static const char *TAG = "gdep073e01";

static void clear_screen(Context *ctx, int color);

struct SPI
{
    struct SPIDisplay spi_disp;

    int busy_gpio;
    int dc_gpio;
    int reset_gpio;

    Context *ctx;

    int count_to_refresh;
    uint64_t last_refresh;

    struct DisplayTaskArgs display_args;
};

#define SPI_FROM_CTX(ctx) \
    CONTAINER_OF((struct DisplayTaskArgs *) (ctx)->platform_data, struct SPI, display_args)

static inline float square(float p)
{
    return p * p;
}

static uint8_t dither_acep7(int x, int y, uint8_t r, uint8_t g, uint8_t b)
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

    // values found by trial and error
    // they try to get closer to real colors than pure saturated RGB colors
    uint8_t colors[7][3] = {
        { 0x00, 0x00, 0x00 },
        { 0xFF, 0xFF, 0xFF },
        { 0xFF, 0xFF, 0x00 },
        { 0xFF, 0x00, 0x00 },
        { 0x10, 0x10, 0x10 },
        { 0x00, 0x00, 0xFF },
        { 0x00, 0xFF, 0x00 }
    };

    float min = INT_MAX;
    int min_index = 0;

    for (int i = 0; i < 7; i++) {
        int r2 = colors[i][0];
        int g2 = colors[i][1];
        int b2 = colors[i][2];

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

static void writecommand(struct SPI *spi, uint8_t cmd)
{
    gpio_set_level(spi->dc_gpio, 0);
    spi_display_write(&spi->spi_disp, 8, cmd);
    gpio_set_level(spi->dc_gpio, 1);
}

static void writedata(struct SPI *spi, uint8_t data)
{
    gpio_set_level(spi->dc_gpio, 1);
    spi_display_write(&spi->spi_disp, 8, data);
}

static void writedatan(struct SPI *spi, uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        writedata(spi, data[i]);
    }
}

static void display_reset(struct SPI *spi)
{
    gpio_set_level(spi->reset_gpio, 0);
    vTaskDelay(100);
    gpio_set_level(spi->reset_gpio, 1);
}

static void wait_busy_level(struct SPI *spi, int level)
{
    while (gpio_get_level(spi->busy_gpio) != level) {
        vTaskDelay(100);
    }
}

static inline void draw_pixel_x(uint8_t *line_buf, int xpos, uint8_t c)
{
#if CHECK_OVERFLOW
    if (xpos > DISPLAY_WIDTH) {
        fprintf(stderr, "buf ovf!\n");
        return;
    }
#endif

    if ((xpos & 1) == 0) {
        line_buf[xpos / 2] = (line_buf[xpos / 2] & 0xF) | (c << 4);
    } else {
        line_buf[xpos / 2] = (line_buf[xpos / 2] & 0xF0) | c;
    }
}

static int draw_image_x(uint8_t *line_buf, int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int y = item->y;

    int bgcolor_r;
    int bgcolor_g;
    int bgcolor_b;
    bool visible_bg;
    if (item->brcolor != 0) {
        bgcolor_r = (item->brcolor >> 24) & 0xFF;
        bgcolor_g = (item->brcolor >> 16) & 0xFF;
        bgcolor_b = (item->brcolor >> 8) & 0xFF;
        visible_bg = true;
    } else {
        bgcolor_r = 0;
        bgcolor_g = 0;
        bgcolor_b = 0;
        visible_bg = false;
    }

    int width = item->width;
    const char *data = item->data.image_data.pix;

    int drawn_pixels = 0;

    uint32_t *pixels = ((uint32_t *) data) + (ypos - y) * width + (xpos - x);

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    for (int j = xpos - x; j < width; j++) {
        uint32_t img_pixel = READ_32_UNALIGNED(pixels);
        if ((*pixels >> 24) & 0xFF) {
            uint8_t r = img_pixel >> 24;
            uint8_t g = (img_pixel >> 16) & 0xFF;
            uint8_t b = (img_pixel >> 8) & 0xFF;

            uint8_t c = dither_acep7(xpos + drawn_pixels, ypos, r, g, b);
            draw_pixel_x(line_buf, xpos + drawn_pixels, c);

        } else if (visible_bg) {
            uint8_t c = dither_acep7(xpos + drawn_pixels, ypos, bgcolor_r, bgcolor_g, bgcolor_b);
            draw_pixel_x(line_buf, xpos + drawn_pixels, c);

        } else {
            return drawn_pixels;
        }
        drawn_pixels++;
        pixels++;
    }

    return drawn_pixels;
}

static int draw_scaled_cropped_img_x(uint8_t *line_buf, int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int y = item->y;

    int bgcolor_r;
    int bgcolor_g;
    int bgcolor_b;
    bool visible_bg;
    if (item->brcolor != 0) {
        bgcolor_r = (item->brcolor >> 24) & 0xFF;
        bgcolor_g = (item->brcolor >> 16) & 0xFF;
        bgcolor_b = (item->brcolor >> 8) & 0xFF;
        visible_bg = true;
    } else {
        bgcolor_r = 0;
        bgcolor_g = 0;
        bgcolor_b = 0;
        visible_bg = false;
    }

    int width = item->width;
    const char *data = item->data.image_data_with_size.pix;

    int drawn_pixels = 0;

    int y_scale = item->y_scale;
    int x_scale = item->x_scale;
    int img_width = item->data.image_data_with_size.width;

    int source_x = item->source_x;
    int source_y = item->source_y;

    uint32_t *pixels = ((uint32_t *) data) + (source_y + ((ypos - y) / y_scale)) * img_width + source_x + ((xpos - x) / x_scale);

    if (source_x + (width / x_scale) > img_width) {
        width = (img_width - source_x) * x_scale;
    }

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    for (int j = xpos - x; j < width; j++) {
        uint32_t img_pixel = READ_32_UNALIGNED(pixels);
        if ((*pixels >> 24) & 0xFF) {
            uint8_t r = img_pixel >> 24;
            uint8_t g = (img_pixel >> 16) & 0xFF;
            uint8_t b = (img_pixel >> 8) & 0xFF;

            uint8_t c = dither_acep7(xpos + drawn_pixels, ypos, r, g, b);
            draw_pixel_x(line_buf, xpos + drawn_pixels, c);

        } else if (visible_bg) {
            uint8_t c = dither_acep7(xpos + drawn_pixels, ypos, bgcolor_r, bgcolor_g, bgcolor_b);
            draw_pixel_x(line_buf, xpos + drawn_pixels, c);

        } else {
            return drawn_pixels;
        }
        drawn_pixels++;
        pixels = ((uint32_t *) data) + (source_y + ((ypos - y) / y_scale)) * img_width + source_x + (j / x_scale);
    }

    return drawn_pixels;
}

static int draw_rect_x(uint8_t *line_buf, int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int width = item->width;

    uint8_t r = (item->brcolor >> 24) & 0xFF;
    uint8_t g = (item->brcolor >> 16) & 0xFF;
    uint8_t b = (item->brcolor >> 8) & 0xFF;

    int drawn_pixels = 0;

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    for (int j = xpos - x; j < width; j++) {
        uint8_t c = dither_acep7(xpos + drawn_pixels, ypos, r, g, b);
        draw_pixel_x(line_buf, xpos + drawn_pixels, c);
        drawn_pixels++;
    }

    return drawn_pixels;
}

static int draw_text_x(uint8_t *line_buf, int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int y = item->y;
    bool visible_bg;

    int fgcolor_r = (item->data.text_data.fgcolor >> 24) & 0xFF;
    int fgcolor_g = (item->data.text_data.fgcolor >> 16) & 0xFF;
    int fgcolor_b = (item->data.text_data.fgcolor >> 8) & 0xFF;

    int bgcolor_r;
    int bgcolor_g;
    int bgcolor_b;

    if (item->brcolor != 0) {
        bgcolor_r = (item->brcolor >> 24) & 0xFF;
        bgcolor_g = (item->brcolor >> 16) & 0xFF;
        bgcolor_b = (item->brcolor >> 8) & 0xFF;
        visible_bg = true;
    } else {
        bgcolor_r = 0;
        bgcolor_g = 0;
        bgcolor_b = 0;
        visible_bg = false;
    }

    char *text = (char *) item->data.text_data.text;

    int width = item->width;

    int drawn_pixels = 0;

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    for (int j = xpos - x; j < width; j++) {
        int char_index = j / CHAR_WIDTH;
        char c = text[char_index];
        unsigned const char *glyph = fontdata + ((unsigned char) c) * 16;

        unsigned char row = glyph[ypos - y];

        bool opaque;
        int k = j % CHAR_WIDTH;
        if (row & (1 << (7 - k))) {
            opaque = true;
        } else {
            opaque = false;
        }

        if (opaque) {
            uint8_t c = dither_acep7(xpos + drawn_pixels, ypos, fgcolor_r, fgcolor_g, fgcolor_b);
            draw_pixel_x(line_buf, xpos + drawn_pixels, c);

        } else if (visible_bg) {
            uint8_t c = dither_acep7(xpos + drawn_pixels, ypos, bgcolor_r, bgcolor_g, bgcolor_b);
            draw_pixel_x(line_buf, xpos + drawn_pixels, c);

        } else {
            return drawn_pixels;
        }
        drawn_pixels++;
    }

    return drawn_pixels;
}

static void wait_some_time(Context *ctx)
{
    struct SPI *spi = SPI_FROM_CTX(ctx);

    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t now = tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL);
    uint64_t delta = now - spi->last_refresh;
    if (delta < 2000) {
        // Wait 2 seconds before allowing a new refresh
        // this is not on datasheets, but without this the screen will not update.
        vTaskDelay((2000 - delta) / portTICK_PERIOD_MS);
    }
}

static void update_last_refresh_ts(Context *ctx)
{
    struct SPI *spi = SPI_FROM_CTX(ctx);

    struct timeval tv;
    gettimeofday(&tv, NULL);
    spi->last_refresh = tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL);
}

static void maybe_refresh(Context *ctx)
{
#if 0
    struct SPI *spi = SPI_FROM_CTX(ctx);

    spi->count_to_refresh--;
    if (spi->count_to_refresh <= 0) {
        // 7 is the special "clear screen color"
        clear_screen(ctx, 7);
        update_last_refresh_ts(ctx);
        spi->count_to_refresh = 5;
    }
#endif
}

static void do_update(Context *ctx, term display_list)
{
    maybe_refresh(ctx);
    // it looks like we need to wait some time
    // let's use 2 seconds
    wait_some_time(ctx);

    int proper;
    int len = term_list_length(display_list, &proper);

    BaseDisplayItem *items = malloc(sizeof(BaseDisplayItem) * len);

    term t = display_list;
    for (int i = 0; i < len; i++) {
        init_item(&items[i], term_get_list_head(t), ctx);
        t = term_get_list_tail(t);
    }

    int screen_width = DISPLAY_WIDTH;
    int screen_height = DISPLAY_HEIGHT;
    struct SPI *spi = SPI_FROM_CTX(ctx);

    struct SPIDisplay *spi_disp = &spi->spi_disp;
    spi_device_acquire_bus(spi_disp->handle, portMAX_DELAY);

#if 0
    // resolution command
    writecommand(spi, 0x61);
    writedata(spi, 0x02);
    writedata(spi, 0x58);
    writedata(spi, 0x01);
    writedata(spi, 0xC0);
#endif

    // update command
    writecommand(spi, 0x10);

    gpio_set_level(spi->dc_gpio, 1);

    uint8_t *buf = heap_caps_malloc(DISPLAY_WIDTH / 2, MALLOC_CAP_DMA);
    memset(buf, 0x11, DISPLAY_WIDTH / 2);

    bool transaction_in_progress = false;

    for (int ypos = 0; ypos < screen_height; ypos++) {
        if (transaction_in_progress) {
            spi_transaction_t *trans = NULL;
            spi_device_get_trans_result(spi->spi_disp.handle, &trans, portMAX_DELAY);
        }

        int xpos = 0;
        while (xpos < screen_width) {
            int drawn_pixels = draw_x(buf, xpos, ypos, items, len);
            xpos += drawn_pixels;
        }

        spi_display_dmawrite(&spi->spi_disp, DISPLAY_WIDTH / 2, buf);
        transaction_in_progress = true;
    }

    if (transaction_in_progress) {
        spi_transaction_t *trans = NULL;
        spi_device_get_trans_result(spi->spi_disp.handle, &trans, portMAX_DELAY);
    }

    free(buf);

    // not sure if we should add 0x11, which is end of data command or not

    // power on command
    writecommand(spi, 0x04);
    wait_busy_level(spi, 1);

    // refresh command
    writecommand(spi, 0x12);
    uint8_t refresh_data[] = {0x00};
    writedatan(spi, refresh_data, sizeof(refresh_data));
    wait_busy_level(spi, 1);

    // power off command
    writecommand(spi, 0x02);
    spi_device_release_bus(spi_disp->handle);
    wait_busy_level(spi, 1);

    destroy_items(items, len);

    update_last_refresh_ts(ctx);
}

static void process_message(Message *message, Context *ctx)
{
    GenMessage gen_message;
    if (UNLIKELY(port_parse_gen_message(message->message, &gen_message) != GenCallMessage)) {
        fprintf(stderr, "Received invalid message.");
        AVM_ABORT();
    }

    term req = gen_message.req;
    if (UNLIKELY(!term_is_tuple(req) || term_get_tuple_arity(req) < 1)) {
        AVM_ABORT();
    }
    term cmd = term_get_tuple_element(req, 0);

    if (cmd == context_make_atom(ctx, "\x6"
                                      "update")) {

        term display_list = term_get_tuple_element(req, 1);
        do_update(ctx, display_list);

    } else if (cmd == globalcontext_make_atom(ctx->global, "\xA" "load_image")) {
        handle_load_image(req, gen_message.ref, gen_message.pid, ctx);
        return;

    } else {
#if REPORT_UNEXPECTED_MSGS
        fprintf(stderr, "display: ");
        term_display(stderr, req, ctx);
        fprintf(stderr, "\n");
#endif
    }

    BEGIN_WITH_STACK_HEAP(TUPLE_SIZE(2) + REF_SIZE, heap);
    term return_tuple = term_alloc_tuple(2, &heap);
    term_put_tuple_element(return_tuple, 0, gen_message.ref);
    term_put_tuple_element(return_tuple, 1, OK_ATOM);

    send_message(gen_message.pid, return_tuple, ctx->global);
    END_WITH_STACK_HEAP(heap, ctx->global);
}

static void clear_screen(Context *ctx, int color)
{
    struct SPI *spi = SPI_FROM_CTX(ctx);

    uint8_t *buf = heap_caps_malloc(DISPLAY_WIDTH / 2, MALLOC_CAP_DMA);

    struct SPIDisplay *spi_disp = &spi->spi_disp;
    spi_device_acquire_bus(spi_disp->handle, portMAX_DELAY);

#if 0
    writecommand(spi, 0x61);
    writedata(spi, 0x02);
    writedata(spi, 0x58);
    writedata(spi, 0x01);
    writedata(spi, 0xC0);
#endif
    writecommand(spi, 0x10);

    gpio_set_level(spi->dc_gpio, 1);

    bool transaction_in_progress = false;

    for (int i = 0; i < DISPLAY_HEIGHT; i++) {
        if (transaction_in_progress) {
            spi_transaction_t *trans = NULL;
            spi_device_get_trans_result(spi->spi_disp.handle, &trans, portMAX_DELAY);
        }

        // let's ensure a memset otherwise we might generate odd artifacts
        memset(buf, color | (color << 4), DISPLAY_WIDTH / 2);
        spi_display_dmawrite(spi_disp, DISPLAY_WIDTH / 2, buf);
        transaction_in_progress = true;
    }

    if (transaction_in_progress) {
        spi_transaction_t *trans = NULL;
        spi_device_get_trans_result(spi->spi_disp.handle, &trans, portMAX_DELAY);
    }

    writecommand(spi, 0x04);
    wait_busy_level(spi, 1);
    writecommand(spi, 0x12);
    uint8_t refresh_data[] = {0x00};
    writedatan(spi, refresh_data, sizeof(refresh_data));
    wait_busy_level(spi, 1);
    writecommand(spi, 0x02);
    spi_device_release_bus(spi_disp->handle);
    wait_busy_level(spi, 1);
}

static void display_spi_init(Context *ctx, term opts)
{
    struct SPI *spi = malloc(sizeof(struct SPI));
    // TODO check here

    struct SPIDisplayConfig spi_config;
    spi_display_init_config(&spi_config);
    spi_config.clock_speed_hz = 4000000;
    spi_display_parse_config(&spi_config, opts, ctx->global);
    spi_display_init(&spi->spi_disp, &spi_config);

    bool ok = display_common_gpio_from_opts(opts, ATOM_STR("\x4", "busy"), &spi->busy_gpio, ctx->global);
    ok = ok && display_common_gpio_from_opts(opts, ATOM_STR("\x2", "dc"), &spi->dc_gpio, ctx->global);
    ok = ok && display_common_gpio_from_opts(opts, ATOM_STR("\x5", "reset"), &spi->reset_gpio, ctx->global);
    if (UNLIKELY(!ok)) {
        ESP_LOGE(TAG, "Failed init: invalid display GPIOs.");
        return;
    }

    gpio_set_direction(spi->reset_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(spi->reset_gpio, 1);
    gpio_set_direction(spi->dc_gpio, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(spi->dc_gpio, GPIO_PULLUP_ENABLE);
    gpio_set_direction(spi->busy_gpio, GPIO_MODE_INPUT);
    gpio_set_pull_mode(spi->busy_gpio, GPIO_PULLUP_ENABLE);
    gpio_set_level(spi->dc_gpio, 0);

    esp_err_t ret = spi_device_acquire_bus(spi->spi_disp.handle, portMAX_DELAY);
    ESP_ERROR_CHECK(ret);
    display_reset(spi);

    wait_busy_level(spi, 1);

    writecommand(spi, 0xAA);
    uint8_t psr1_data[] = {0x49, 0x55, 0x20, 0x08, 0x09, 0x18};
    writedatan(spi, psr1_data, sizeof(psr1_data));
    wait_busy_level(spi, 1);

    writecommand(spi, PWRR);
    uint8_t pwrr_data[] = {0x3F};
    writedatan(spi, pwrr_data, sizeof(pwrr_data));
    wait_busy_level(spi, 1);

    writecommand(spi, PSR);
    uint8_t psr_data[] = {0x5F, 0x69};
    writedatan(spi, psr_data, sizeof(psr_data));
    wait_busy_level(spi, 1);

    writecommand(spi, POFS);
    uint8_t pofs_data[] = {0x00, 0x54, 0x00, 0x44};
    writedatan(spi, pofs_data, sizeof(pofs_data));
    wait_busy_level(spi, 1);

    writecommand(spi, BTST1);
    uint8_t btst1_data[] = {0x40, 0x1F, 0x1F, 0x2C};
    writedatan(spi, btst1_data, sizeof(btst1_data));
    wait_busy_level(spi, 1);

    writecommand(spi, BTST2);
    uint8_t btst2_data[] = {0x6F, 0x1F, 0x17, 0x49};
    writedatan(spi, btst2_data, sizeof(btst2_data));
    wait_busy_level(spi, 1);

    writecommand(spi, BTST3);
    uint8_t btst3_data[] = {0x6F, 0x1F, 0x1F, 0x22};
    writedatan(spi, btst3_data, sizeof(btst3_data));
    wait_busy_level(spi, 1);

    writecommand(spi, PLL);
    uint8_t pll_data[] = {0x00};
    writedatan(spi, pll_data, sizeof(pll_data));
    wait_busy_level(spi, 1);

    writecommand(spi, CDI);
    uint8_t cdi_data[] = {0x3F};
    writedatan(spi, cdi_data, sizeof(cdi_data));
    wait_busy_level(spi, 1);

    writecommand(spi, TCON);
    uint8_t tcon_data[] = {0x02, 0x00};
    writedatan(spi, tcon_data, sizeof(tcon_data));
    wait_busy_level(spi, 1);

    writecommand(spi, TRES);
    uint8_t tres_data[] = {0x03, 0x20, 0x01, 0xe0};
    writedatan(spi, tres_data, sizeof(tres_data));
    wait_busy_level(spi, 1);

    writecommand(spi, T_VDCS);
    uint8_t vdcs_data[] = {0x01};
    writedatan(spi, vdcs_data, sizeof(vdcs_data));
    wait_busy_level(spi, 1);

    writecommand(spi, PWS);
    uint8_t pws_data[] = {0x2F};
    writedatan(spi, pws_data, sizeof(pws_data));
    wait_busy_level(spi, 1);

    // PON
    writecommand(spi, 0x04);
    wait_busy_level(spi, 1);

    spi_device_release_bus(spi->spi_disp.handle);

    ctx->platform_data = &spi->display_args;

    spi->ctx = ctx;

    update_last_refresh_ts(ctx);
    spi->count_to_refresh = 0;

#if SELF_TEST
    for (int i = 0; i < 8; i++) {
        fprintf(stderr, "color: %i\n", i);
        clear_screen(ctx, i);
        vTaskDelay(30000 / portTICK_PERIOD_MS);
    }
    clear_screen(ctx, 1);

    while (1)
        ;
#else
    spi->display_args.messages_queue = xQueueCreate(32, sizeof(Message *));
    spi->display_args.process_message_fn = process_message;
    spi->display_args.ctx = ctx;
    xTaskCreate(display_process_messages, "display", 10000, &spi->display_args, 1, NULL);
#endif
}

Context *gdep073e01_display_driver_create_port(GlobalContext *global, term opts)
{
    Context *ctx = context_new(global);
    ctx->native_handler = display_driver_consume_mailbox;
    display_spi_init(ctx, opts);
    return ctx;
}
