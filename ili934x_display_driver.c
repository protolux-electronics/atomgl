/*
 * This file is part of AtomGL.
 *
 * Copyright 2020-2022 Davide Bettio <davide@uninstall.it>
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

#include "display_driver.h"

#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <esp_heap_caps.h>

#include <atom.h>
#include <bif.h>
#include <context.h>
#include <debug.h>
#include <defaultatoms.h>
#include <globalcontext.h>
#include <interop.h>
#include <mailbox.h>
#include <module.h>
#include <port.h>
#include <sys.h>
#include <term.h>
#include <utils.h>

#include <esp32_sys.h>

#include <trace.h>

#include "backlight_gpio.h"
#include "dcs_lcd_color.h"
#include "display_common.h"
#include "display_items.h"
#include "display_message.h"
#include "display_task.h"
#include "image_helpers.h"
#include "spi_dc_driver.h"
#include "spi_display.h"

#define SPI_CLOCK_HZ 27000000
#define SPI_MODE 0

#define CHAR_WIDTH 8

#define ILI9341_SLPIN 0x10
#define ILI9341_SLPOUT 0x11
#define ILI9341_PTLON 0x12
#define ILI9341_NORON 0x13

#define ILI9341_INVOFF 0x20
#define ILI9341_INVON 0x21
#define ILI9341_GAMMASET 0x26
#define ILI9341_DISPOFF 0x28
#define ILI9341_DISPON 0x29

#define ILI9341_PTLAR 0x30
#define ILI9341_VSCRDEF 0x33
#define ILI9341_MADCTL 0x36
#define ILI9341_VSCRSADD 0x37
#define ILI9341_PIXFMT 0x3A

#define ILI9341_FRMCTR1 0xB1
#define ILI9341_FRMCTR2 0xB2
#define ILI9341_FRMCTR3 0xB3
#define ILI9341_INVCTR 0xB4
#define ILI9341_DFUNCTR 0xB6

#define ILI9341_PWCTR1 0xC0
#define ILI9341_PWCTR2 0xC1
#define ILI9341_PWCTR3 0xC2
#define ILI9341_PWCTR4 0xC3
#define ILI9341_PWCTR5 0xC4
#define ILI9341_VMCTR1 0xC5
#define ILI9341_VMCTR2 0xC7

#define ILI9341_GMCTRP1 0xE0
#define ILI9341_GMCTRN1 0xE1

#define TFT_SWRST 0x01
#define TFT_CASET 0x2A
#define TFT_PASET 0x2B
#define TFT_RAMWR 0x2C

#define TFT_MADCTL 0x36
#define TFT_MAD_MY 0x80
#define TFT_MAD_MX 0x40
#define TFT_MAD_MV 0x20
#define TFT_MAD_BGR 0x08

#define TFT_INVOFF 0x20
#define TFT_INVON 0x21

#include "font_data.h"

static const char *TAG = "ili934x_display_driver";

struct SPI
{
    struct SPIDCBus bus;
    int reset_gpio;

    avm_int_t rotation;

    Context *ctx;

    struct DisplayTaskArgs display_args;
};

#define SPI_FROM_CTX(ctx) \
    CONTAINER_OF((struct DisplayTaskArgs *) (ctx)->platform_data, struct SPI, display_args)

// This struct is just for compatibility reasons with the SDL display driver
// so it is possible to easily copy & paste code from there.
struct Screen
{
    int w;
    int h;
    uint16_t *pixels;
    uint16_t *pixels_out;
};

static struct Screen *screen;

static void display_init(Context *ctx, term opts);
static void display_init42c(struct SPI *spi);
static void display_init41(struct SPI *spi);

static inline void set_screen_paint_area(struct SPI *spi, int x, int y, int width, int height)
{
    spi_dc_writecommand(&spi->bus, TFT_CASET);
    spi_device_acquire_bus(spi->bus.spi_disp.handle, portMAX_DELAY);
    spi_display_write(&spi->bus.spi_disp, 32, (x << 16) | ((x + width) - 1));
    spi_device_release_bus(spi->bus.spi_disp.handle);

    spi_dc_writecommand(&spi->bus, TFT_PASET);
    spi_device_acquire_bus(spi->bus.spi_disp.handle, portMAX_DELAY);
    spi_display_write(&spi->bus.spi_disp, 32, (y << 16) | ((y + height) - 1));
    spi_device_release_bus(spi->bus.spi_disp.handle);
}

static int draw_image_x(int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int y = item->y;

    uint16_t bgcolor = 0;
    bool visible_bg;
    if (item->brcolor != 0) {
        bgcolor = rgba8888_color_to_rgb565(item->brcolor);
        visible_bg = true;
    } else {
        visible_bg = false;
    }

    int width = item->width;
    const char *data = item->data.image_data.pix;

    int drawn_pixels = 0;

    uint32_t *pixels = ((uint32_t *) data) + (ypos - y) * width + (xpos - x);
    uint16_t *pixmem16 = (uint16_t *) (((uint8_t *) screen->pixels) + xpos * sizeof(uint16_t));

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    for (int j = xpos - x; j < width; j++) {
        uint32_t img_pixel = READ_32_UNALIGNED(pixels);
        uint8_t alpha = rgba8888_get_alpha(img_pixel);
        if (alpha == 0xFF) {
            uint16_t color = uint32_color_to_surface(img_pixel);
            pixmem16[drawn_pixels] = color;
        } else if (visible_bg) {
            uint16_t color = rgba8888_color_to_rgb565(img_pixel);
            uint16_t blended = alpha_blend_rgb565(color, bgcolor, alpha);
            pixmem16[drawn_pixels] = rgb565_color_to_surface(blended);
        } else {
            return drawn_pixels;
        }
        drawn_pixels++;
        pixels++;
    }

    return drawn_pixels;
}

static int draw_scaled_cropped_img_x(int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int y = item->y;

    uint16_t bgcolor = 0;
    bool visible_bg;
    if (item->brcolor != 0) {
        bgcolor = rgba8888_color_to_rgb565(item->brcolor);
        visible_bg = true;
    } else {
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
    uint16_t *pixmem16 = (uint16_t *) (((uint8_t *) screen->pixels) + xpos * sizeof(uint16_t));

    if (source_x + (width / x_scale) > img_width) {
        width = (img_width - source_x) * x_scale;
    }

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    for (int j = xpos - x; j < width; j++) {
        uint32_t img_pixel = READ_32_UNALIGNED(pixels);
        uint8_t alpha = rgba8888_get_alpha(img_pixel);
        if (alpha == 0xFF) {
            uint16_t color = uint32_color_to_surface(img_pixel);
            pixmem16[drawn_pixels] = color;
        } else if (visible_bg) {
            uint16_t color = rgba8888_color_to_rgb565(img_pixel);
            uint16_t blended = alpha_blend_rgb565(color, bgcolor, alpha);
            pixmem16[drawn_pixels] = rgb565_color_to_surface(blended);
        } else {
            return drawn_pixels;
        }
        drawn_pixels++;
        // TODO: optimize here
        pixels = ((uint32_t *) data) + (source_y + ((ypos - y) / y_scale)) * img_width + source_x + (j / x_scale);
    }

    return drawn_pixels;
}

static int draw_rect_x(int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int width = item->width;
    uint16_t color = uint32_color_to_surface(item->brcolor);

    int drawn_pixels = 0;

    uint16_t *pixmem16 = (uint16_t *) (((uint8_t *) screen->pixels) + xpos * sizeof(uint16_t));

    if (width > xpos - x + max_line_len) {
        width = xpos - x + max_line_len;
    }

    for (int j = xpos - x; j < width; j++) {
        pixmem16[drawn_pixels] = color;
        drawn_pixels++;
    }

    return drawn_pixels;
}

static int draw_text_x(int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int x = item->x;
    int y = item->y;
    uint16_t fgcolor = uint32_color_to_surface(item->data.text_data.fgcolor);
    uint16_t bgcolor;
    bool visible_bg;
    if (item->brcolor != 0) {
        bgcolor = uint32_color_to_surface(item->brcolor);
        visible_bg = true;
    } else {
        visible_bg = false;
    }

    char *text = (char *) item->data.text_data.text;

    int width = item->width;

    int drawn_pixels = 0;

    uint16_t *pixmem32 = (uint16_t *) (((uint8_t *) screen->pixels) + xpos * sizeof(uint16_t));

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
            pixmem32[drawn_pixels] = fgcolor;
        } else if (visible_bg) {
            pixmem32[drawn_pixels] = bgcolor;
        } else {
            return drawn_pixels;
        }
        drawn_pixels++;
    }

    return drawn_pixels;
}

static int find_max_line_len(BaseDisplayItem *items, int count, int xpos, int ypos)
{
    int line_len = screen->w - xpos;

    for (int i = 0; i < count; i++) {
        BaseDisplayItem *item = &items[i];

        if ((xpos < item->x) && (ypos >= item->y) && (ypos < item->y + item->height)) {
            int len_to_item = item->x - xpos;
            line_len = (line_len > len_to_item) ? len_to_item : line_len;
        }
    }

    return line_len;
}

static int draw_x(int xpos, int ypos, BaseDisplayItem *items, int items_count)
{
    bool below = false;

    for (int i = 0; i < items_count; i++) {
        BaseDisplayItem *item = &items[i];
        if ((xpos < item->x) || (xpos >= item->x + item->width) || (ypos < item->y) || (ypos >= item->y + item->height)) {
            continue;
        }

        int max_line_len = below ? 1 : find_max_line_len(items, i, xpos, ypos);

        int drawn_pixels = 0;
        switch (items[i].primitive) {
            case Image:
                drawn_pixels = draw_image_x(xpos, ypos, max_line_len, item);
                break;

            case Rect:
                drawn_pixels = draw_rect_x(xpos, ypos, max_line_len, item);
                break;

            case ScaledCroppedImage:
                drawn_pixels = draw_scaled_cropped_img_x(xpos, ypos, max_line_len, item);
                break;

            case Text:
                drawn_pixels = draw_text_x(xpos, ypos, max_line_len, item);
                break;
            default: {
                fprintf(stderr, "unexpected display list command.\n");
            }
        }

        if (drawn_pixels != 0) {
            return drawn_pixels;
        }

        below = true;
    }

    return 1;
}

static void do_update(Context *ctx, term display_list)
{
    int proper;
    int len = term_list_length(display_list, &proper);

    BaseDisplayItem *items = malloc(sizeof(BaseDisplayItem) * len);

    term t = display_list;
    for (int i = 0; i < len; i++) {
        init_item(&items[i], term_get_list_head(t), ctx);
        t = term_get_list_tail(t);
    }

    int screen_width = screen->w;
    int screen_height = screen->h;
    struct SPI *spi = SPI_FROM_CTX(ctx);

    set_screen_paint_area(spi, 0, 0, screen_width, screen_height);
    spi_dc_writecommand(&spi->bus, TFT_RAMWR);
    spi_device_acquire_bus(spi->bus.spi_disp.handle, portMAX_DELAY);

    bool transaction_in_progress = false;

    for (int ypos = 0; ypos < screen_height; ypos++) {
        int xpos = 0;
        while (xpos < screen_width) {
            int drawn_pixels = draw_x(xpos, ypos, items, len);
            xpos += drawn_pixels;
        }

        if (transaction_in_progress) {
            spi_transaction_t *trans;
            // I did a quick measurement, and most of the time is spent waiting for DMA transaction
            // eg. 23 us spent in draw_x, 188 us spent in spi_device_get_trans_result
            spi_device_get_trans_result(spi->bus.spi_disp.handle, &trans, portMAX_DELAY);
        }

        //NEW CODE
        void *tmp = screen->pixels;
        screen->pixels = screen->pixels_out;
        screen->pixels_out = tmp;
        spi_display_dmawrite(&spi->bus.spi_disp, screen_width * sizeof(uint16_t), screen->pixels_out);
        transaction_in_progress = true;
    }

    if (transaction_in_progress) {
        spi_transaction_t *trans;
        spi_device_get_trans_result(spi->bus.spi_disp.handle, &trans, portMAX_DELAY);
    }

    spi_device_release_bus(spi->bus.spi_disp.handle);

    destroy_items(items, len);
}

void draw_buffer(struct SPI *spi, int x, int y, int width, int height, const void *imgdata)
{
    const uint16_t *data = imgdata;

    set_screen_paint_area(spi, x, y, width, height);

    spi_dc_writecommand(&spi->bus, TFT_RAMWR);

    int dest_size = width * height;
    int buf_pixel_size = (dest_size > 1024) ? 1024 : dest_size;

    int chunks = dest_size / 1024;

    uint16_t *tmpbuf = heap_caps_malloc(buf_pixel_size * sizeof(uint16_t), MALLOC_CAP_DMA);

    spi_device_acquire_bus(spi->bus.spi_disp.handle, portMAX_DELAY);
    for (int i = 0; i < chunks; i++) {
        const uint16_t *data_b = data + 1024 * i;
        for (int j = 0; j < 1024; j++) {
            tmpbuf[j] = SPI_SWAP_DATA_TX(data_b[j], 16);
        }
        spi_display_dmawrite(&spi->bus.spi_disp, buf_pixel_size * sizeof(uint16_t), tmpbuf);
    }
    int last_chunk_size = dest_size - chunks * 1024;
    if (last_chunk_size) {
        const uint16_t *data_b = data + chunks * 1024;
        for (int j = 0; j < 1024; j++) {
            tmpbuf[j] = SPI_SWAP_DATA_TX(data_b[j], 16);
        }
        spi_display_dmawrite(&spi->bus.spi_disp, last_chunk_size * sizeof(uint16_t), tmpbuf);
    }
    spi_device_release_bus(spi->bus.spi_disp.handle);

    free(tmpbuf);
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

    struct SPI *spi = SPI_FROM_CTX(ctx);

    if (cmd == context_make_atom(ctx, "\x6"
                                      "update")) {
        term display_list = term_get_tuple_element(req, 1);
        do_update(ctx, display_list);

    } else if (cmd == context_make_atom(ctx, "\xB"
                                             "draw_buffer")) {
        int x = term_to_int(term_get_tuple_element(req, 1));
        int y = term_to_int(term_get_tuple_element(req, 2));
        int width = term_to_int(term_get_tuple_element(req, 3));
        int height = term_to_int(term_get_tuple_element(req, 4));
        unsigned long addr_low = term_to_int(term_get_tuple_element(req, 5));
        unsigned long addr_high = term_to_int(term_get_tuple_element(req, 6));

        const void *data = (const void *) ((addr_low | (addr_high << 16)));

        draw_buffer(spi, x, y, width, height, data);

        // draw_buffer is a kind of cast, no need to reply
        return;

    } else if (cmd == globalcontext_make_atom(ctx->global, "\xA" "load_image")) {
        handle_load_image(req, gen_message.ref, gen_message.pid, ctx);
        return;

    } else {
        fprintf(stderr, "display: ");
        term_display(stderr, req, ctx);
        fprintf(stderr, "\n");
    }

    BEGIN_WITH_STACK_HEAP(TUPLE_SIZE(2) + REF_SIZE, heap);
    term return_tuple = term_alloc_tuple(2, &heap);
    term_put_tuple_element(return_tuple, 0, gen_message.ref);
    term_put_tuple_element(return_tuple, 1, OK_ATOM);

    send_message(gen_message.pid, return_tuple, ctx->global);
    END_WITH_STACK_HEAP(heap, ctx->global);
}

static void set_rotation(struct SPI *spi, int rotation)
{
    if (rotation == 1) {
        spi_dc_writecommand(&spi->bus, TFT_MADCTL);
        spi_dc_writedata(&spi->bus, TFT_MAD_BGR | TFT_MAD_MY | TFT_MAD_MV);
    }
}

Context *ili934x_display_create_port(GlobalContext *global, term opts)
{
    Context *ctx = context_new(global);
    ctx->native_handler = display_driver_consume_mailbox;
    display_init(ctx, opts);
    return ctx;
}

static void display_init(Context *ctx, term opts)
{
    screen = malloc(sizeof(struct Screen));
    // FIXME: hardcoded width and height
    screen->w = 320;
    screen->h = 240;
    screen->pixels = heap_caps_malloc(screen->w * sizeof(uint16_t), MALLOC_CAP_DMA);
    screen->pixels_out = heap_caps_malloc(screen->w * sizeof(uint16_t), MALLOC_CAP_DMA);

    struct SPI *spi = malloc(sizeof(struct SPI));

    spi->display_args.messages_queue = xQueueCreate(32, sizeof(Message *));
    spi->display_args.process_message_fn = process_message;
    spi->display_args.ctx = ctx;
    ctx->platform_data = &spi->display_args;

    spi->ctx = ctx;

    struct SPIDisplayConfig spi_config;
    spi_display_init_config(&spi_config);
    spi_config.mode = SPI_MODE;
    spi_config.clock_speed_hz = SPI_CLOCK_HZ;
    spi_display_parse_config(&spi_config, opts, ctx->global);
    spi_display_init(&spi->bus.spi_disp, &spi_config);

    bool ok = display_common_gpio_from_opts(opts, ATOM_STR("\x2", "dc"), &spi->bus.dc_gpio, ctx->global);
    ok = ok && display_common_gpio_from_opts(opts, ATOM_STR("\x5", "reset"), &spi->reset_gpio, ctx->global);

    term compat_value_term = interop_kv_get_value_default(opts, ATOM_STR("\xA", "compatible"), term_nil(), ctx->global);
    int str_ok;
    char *compat_string = interop_term_to_string(compat_value_term, &str_ok);
    bool enable_ili93442c = false;
    if (str_ok && compat_string) {
        enable_ili93442c = !strcmp(compat_string, "ilitek,ili9342c");
        free(compat_string);
    } else {
        ok = false;
    }

    term rotation = interop_kv_get_value_default(opts, ATOM_STR("\x8", "rotation"), term_from_int(0), ctx->global);
    ok = ok && term_is_integer(rotation);
    spi->rotation = term_to_int(rotation);

    term invon = interop_kv_get_value_default(opts, ATOM_STR("\x10", "enable_tft_invon"), FALSE_ATOM, ctx->global);
    ok = ok && ((invon == TRUE_ATOM) || (invon == FALSE_ATOM));
    bool enable_tft_invon = (invon == TRUE_ATOM);

    if (UNLIKELY(!ok)) {
        ESP_LOGE(TAG, "Failed init: invalid display parameters.");
        return;
    }

    // Reset
    spi_device_acquire_bus(spi->bus.spi_disp.handle, portMAX_DELAY);
    gpio_set_direction(spi->reset_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(spi->reset_gpio, 1);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gpio_set_level(spi->reset_gpio, 0);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gpio_set_level(spi->reset_gpio, 1);
    spi_device_release_bus(spi->bus.spi_disp.handle);

    gpio_set_direction(spi->bus.dc_gpio, GPIO_MODE_OUTPUT);

    spi_dc_writecommand(&spi->bus, TFT_SWRST);

    vTaskDelay(5 / portTICK_PERIOD_MS);

    if (enable_ili93442c) {
        display_init42c(spi);
    } else {
        display_init41(spi);
    }

    spi_dc_writecommand(&spi->bus, ILI9341_SLPOUT);

    vTaskDelay(120 / portTICK_PERIOD_MS);

    spi_dc_writecommand(&spi->bus, ILI9341_DISPON);

    if (enable_tft_invon) {
        spi_dc_writecommand(&spi->bus, TFT_INVON);
    }

    set_rotation(spi, spi->rotation);

    struct BacklightGPIOConfig backlight_config;
    backlight_gpio_init_config(&backlight_config);
    backlight_gpio_parse_config(&backlight_config, opts, ctx->global);
    backlight_gpio_init(&backlight_config);

    xTaskCreate(display_process_messages, "display", 10000, &spi->display_args, 1, NULL);
}

static void display_init41(struct SPI *spi)
{
    spi_dc_writecommand(&spi->bus, 0xEF);
    spi_dc_writedata(&spi->bus, 0x03);
    spi_dc_writedata(&spi->bus, 0x80);
    spi_dc_writedata(&spi->bus, 0x02);

    spi_dc_writecommand(&spi->bus, 0xCF);
    spi_dc_writedata(&spi->bus, 0x00);
    spi_dc_writedata(&spi->bus, 0xC1);
    spi_dc_writedata(&spi->bus, 0x30);

    spi_dc_writecommand(&spi->bus, 0xED);
    spi_dc_writedata(&spi->bus, 0x64);
    spi_dc_writedata(&spi->bus, 0x03);
    spi_dc_writedata(&spi->bus, 0x12);
    spi_dc_writedata(&spi->bus, 0x81);

    spi_dc_writecommand(&spi->bus, 0xE8);
    spi_dc_writedata(&spi->bus, 0x85);
    spi_dc_writedata(&spi->bus, 0x00);
    spi_dc_writedata(&spi->bus, 0x78);

    spi_dc_writecommand(&spi->bus, 0xCB);
    spi_dc_writedata(&spi->bus, 0x39);
    spi_dc_writedata(&spi->bus, 0x2C);
    spi_dc_writedata(&spi->bus, 0x00);
    spi_dc_writedata(&spi->bus, 0x34);
    spi_dc_writedata(&spi->bus, 0x02);

    spi_dc_writecommand(&spi->bus, 0xF7);
    spi_dc_writedata(&spi->bus, 0x20);

    spi_dc_writecommand(&spi->bus, 0xEA);
    spi_dc_writedata(&spi->bus, 0x00);
    spi_dc_writedata(&spi->bus, 0x00);

    spi_dc_writecommand(&spi->bus, ILI9341_PWCTR1);
    spi_dc_writedata(&spi->bus, 0x23);

    spi_dc_writecommand(&spi->bus, ILI9341_PWCTR2);
    spi_dc_writedata(&spi->bus, 0x10);

    spi_dc_writecommand(&spi->bus, ILI9341_VMCTR1);
    spi_dc_writedata(&spi->bus, 0x3E);
    spi_dc_writedata(&spi->bus, 0x28);

    spi_dc_writecommand(&spi->bus, ILI9341_VMCTR2);
    spi_dc_writedata(&spi->bus, 0x86);

    spi_dc_writecommand(&spi->bus, ILI9341_MADCTL);
    spi_dc_writedata(&spi->bus, 0x08);

    spi_dc_writecommand(&spi->bus, ILI9341_PIXFMT);
    spi_dc_writedata(&spi->bus, 0x55);

    spi_dc_writecommand(&spi->bus, ILI9341_FRMCTR1);
    spi_dc_writedata(&spi->bus, 0x00);
    spi_dc_writedata(&spi->bus, 0x13);

    spi_dc_writecommand(&spi->bus, ILI9341_DFUNCTR);
    spi_dc_writedata(&spi->bus, 0x0A);
    spi_dc_writedata(&spi->bus, 0xA2);
    spi_dc_writedata(&spi->bus, 0x27);

    spi_dc_writecommand(&spi->bus, 0xF2);
    spi_dc_writedata(&spi->bus, 0x00);

    spi_dc_writecommand(&spi->bus, ILI9341_GAMMASET);
    spi_dc_writedata(&spi->bus, 0x01);

    spi_dc_writecommand(&spi->bus, ILI9341_GMCTRP1);
    spi_dc_writedata(&spi->bus, 0x0F);
    spi_dc_writedata(&spi->bus, 0x31);
    spi_dc_writedata(&spi->bus, 0x2B);
    spi_dc_writedata(&spi->bus, 0x0C);
    spi_dc_writedata(&spi->bus, 0x0E);
    spi_dc_writedata(&spi->bus, 0x08);
    spi_dc_writedata(&spi->bus, 0x4E);
    spi_dc_writedata(&spi->bus, 0xF1);
    spi_dc_writedata(&spi->bus, 0x37);
    spi_dc_writedata(&spi->bus, 0x07);
    spi_dc_writedata(&spi->bus, 0x10);
    spi_dc_writedata(&spi->bus, 0x03);
    spi_dc_writedata(&spi->bus, 0x0E);
    spi_dc_writedata(&spi->bus, 0x09);
    spi_dc_writedata(&spi->bus, 0x00);

    spi_dc_writecommand(&spi->bus, ILI9341_GMCTRN1);
    spi_dc_writedata(&spi->bus, 0x00);
    spi_dc_writedata(&spi->bus, 0x0E);
    spi_dc_writedata(&spi->bus, 0x14);
    spi_dc_writedata(&spi->bus, 0x03);
    spi_dc_writedata(&spi->bus, 0x11);
    spi_dc_writedata(&spi->bus, 0x07);
    spi_dc_writedata(&spi->bus, 0x31);
    spi_dc_writedata(&spi->bus, 0xC1);
    spi_dc_writedata(&spi->bus, 0x48);
    spi_dc_writedata(&spi->bus, 0x08);
    spi_dc_writedata(&spi->bus, 0x0F);
    spi_dc_writedata(&spi->bus, 0x0C);
    spi_dc_writedata(&spi->bus, 0x31);
    spi_dc_writedata(&spi->bus, 0x36);
    spi_dc_writedata(&spi->bus, 0x0F);
}

static void display_init42c(struct SPI *spi)
{
    spi_dc_writecommand(&spi->bus, 0xC8);
    spi_dc_writedata(&spi->bus, 0xFF);
    spi_dc_writedata(&spi->bus, 0x93);
    spi_dc_writedata(&spi->bus, 0x42);

    spi_dc_writecommand(&spi->bus, ILI9341_PWCTR1);
    spi_dc_writedata(&spi->bus, 0x12);
    spi_dc_writedata(&spi->bus, 0x12);

    spi_dc_writecommand(&spi->bus, ILI9341_PWCTR2);
    spi_dc_writedata(&spi->bus, 0x03);

    spi_dc_writecommand(&spi->bus, 0xB0);
    spi_dc_writedata(&spi->bus, 0xE0);

    spi_dc_writecommand(&spi->bus, 0xF6);
    spi_dc_writedata(&spi->bus, 0x00);
    spi_dc_writedata(&spi->bus, 0x01);
    spi_dc_writedata(&spi->bus, 0x01);

    spi_dc_writecommand(&spi->bus, ILI9341_MADCTL);
    spi_dc_writedata(&spi->bus, TFT_MAD_MY | TFT_MAD_MV);

    spi_dc_writecommand(&spi->bus, ILI9341_PIXFMT);
    spi_dc_writedata(&spi->bus, 0x55);

    spi_dc_writecommand(&spi->bus, ILI9341_DFUNCTR);
    spi_dc_writedata(&spi->bus, 0x08);
    spi_dc_writedata(&spi->bus, 0x82);
    spi_dc_writedata(&spi->bus, 0x27);

    spi_dc_writecommand(&spi->bus, ILI9341_GMCTRP1);
    spi_dc_writedata(&spi->bus, 0x00);
    spi_dc_writedata(&spi->bus, 0x0C);
    spi_dc_writedata(&spi->bus, 0x11);
    spi_dc_writedata(&spi->bus, 0x04);
    spi_dc_writedata(&spi->bus, 0x11);
    spi_dc_writedata(&spi->bus, 0x08);
    spi_dc_writedata(&spi->bus, 0x37);
    spi_dc_writedata(&spi->bus, 0x89);
    spi_dc_writedata(&spi->bus, 0x4C);
    spi_dc_writedata(&spi->bus, 0x06);
    spi_dc_writedata(&spi->bus, 0x0C);
    spi_dc_writedata(&spi->bus, 0x0A);
    spi_dc_writedata(&spi->bus, 0x2E);
    spi_dc_writedata(&spi->bus, 0x34);
    spi_dc_writedata(&spi->bus, 0x0F);

    spi_dc_writecommand(&spi->bus, ILI9341_GMCTRN1);
    spi_dc_writedata(&spi->bus, 0x00);
    spi_dc_writedata(&spi->bus, 0x0B);
    spi_dc_writedata(&spi->bus, 0x11);
    spi_dc_writedata(&spi->bus, 0x05);
    spi_dc_writedata(&spi->bus, 0x13);
    spi_dc_writedata(&spi->bus, 0x09);
    spi_dc_writedata(&spi->bus, 0x33);
    spi_dc_writedata(&spi->bus, 0x67);
    spi_dc_writedata(&spi->bus, 0x48);
    spi_dc_writedata(&spi->bus, 0x07);
    spi_dc_writedata(&spi->bus, 0x0E);
    spi_dc_writedata(&spi->bus, 0x0B);
    spi_dc_writedata(&spi->bus, 0x2E);
    spi_dc_writedata(&spi->bus, 0x33);
    spi_dc_writedata(&spi->bus, 0x0F);
}
