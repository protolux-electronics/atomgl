/*
 * This file is part of AtomGL.
 *
 * Copyright 2020-2022 Davide Bettio <davide@uninstall.it>
 * Copyright 2025 Masatoshi Nishiguchi
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

/* Based on ili934x_display_driver.c. */

#include "display_driver.h"

#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_heap_caps.h>
#include <esp_log.h>

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
#include "dcs_lcd_commands.h"
#include "dcs_lcd_draw.h"
#include "dcs_lcd_screen.h"
#include "display_common.h"
#include "display_items.h"
#include "display_message.h"
#include "display_task.h"
#include "image_helpers.h"
#include "spi_dc_driver.h"
#include "spi_display.h"

#define SPI_CLOCK_HZ 27000000
#define SPI_MODE 0

#define ILI948X_IFMODE 0xB0
#define ILI948X_FRMCTR1 0xB1
#define ILI948X_INVCTR 0xB4
#define ILI948X_DFUNCTR 0xB6
#define ILI948X_ETMOD 0xB7
#define ILI948X_PWRCTR1 0xC0
#define ILI948X_PWRCTR2 0xC1
#define ILI948X_PWRCTR3 0xC2
#define ILI948X_VMCTR1 0xC5
#define ILI948X_HS_LANES_CTRL 0xBE
#define ILI948X_IMAGE_FUNCTION 0xE9
#define ILI948X_PGAMCTRL 0xE0
#define ILI948X_NGAMCTRL 0xE1
#define ILI948X_DGAMCTRL 0xE2
#define ILI948X_ADJCTRL3 0xF7

#define ILI948X_TFTWIDTH 320
#define ILI948X_TFTHEIGHT 480

#include "font_data.h"

static const char *TAG = "ili948x_display_driver";

struct DCSLCDDriver
{
    struct SPIDCBus bus;
    int reset_gpio;

    avm_int_t rotation;
    bool is_ili9488;

    bool madctl_bgr;

    Context *ctx;

    struct DisplayTaskArgs display_args;
};

#define DCS_LCD_DRIVER_FROM_CTX(ctx) \
    CONTAINER_OF((struct DisplayTaskArgs *) (ctx)->platform_data, struct DCSLCDDriver, display_args)

static struct DCSLCDScreen *screen;

// ILI9488 scanline conversion: RGB565 -> RGB888 bytes.
static inline void rgb565_swapped_line_to_rgb888(uint8_t *dst, const uint16_t *src_swapped, int n_pixels)
{
    for (int i = 0; i < n_pixels; i++) {
        uint16_t px = (uint16_t) SPI_SWAP_DATA_TX(src_swapped[i], 16);

        uint8_t r5 = (px >> 11) & 0x1F;
        uint8_t g6 = (px >> 5) & 0x3F;
        uint8_t b5 = (px >> 0) & 0x1F;

        uint8_t r8 = (r5 << 3) | (r5 >> 2);
        uint8_t g8 = (g6 << 2) | (g6 >> 4);
        uint8_t b8 = (b5 << 3) | (b5 >> 2);

        dst[i * 3 + 0] = r8;
        dst[i * 3 + 1] = g8;
        dst[i * 3 + 2] = b8;
    }
}

static void display_init(Context *ctx, term opts);

static void display_init_9486(struct DCSLCDDriver *driver);
static void display_init_9488(struct DCSLCDDriver *driver);

static void do_update(Context *ctx, term display_list)
{
    int proper;
    int len = term_list_length(display_list, &proper);

    BaseDisplayItem *items = malloc(sizeof(BaseDisplayItem) * len);

    term t = display_list;
    for (int i = 0; i < len; i++) {
        display_items_init_item(&items[i], term_get_list_head(t), ctx);
        t = term_get_list_tail(t);
    }

    int screen_width = screen->w;
    int screen_height = screen->h;
    struct DCSLCDDriver *driver = DCS_LCD_DRIVER_FROM_CTX(ctx);

    dcs_lcd_set_paint_area(&driver->bus, screen, 0, 0, screen_width, screen_height);
    spi_dc_write_command(&driver->bus, DCS_LCD_RAMWR);
    spi_device_acquire_bus(driver->bus.spi_disp.handle, portMAX_DELAY);

    bool transaction_in_progress = false;

    for (int ypos = 0; ypos < screen_height; ypos++) {
        int xpos = 0;
        while (xpos < screen_width) {
            int drawn_pixels = dcs_lcd_draw_x(screen, xpos, ypos, items, len);
            xpos += drawn_pixels;
        }

        if (transaction_in_progress) {
            spi_transaction_t *trans;
            spi_device_get_trans_result(driver->bus.spi_disp.handle, &trans, portMAX_DELAY);
        }

        // Swap scanline buffers.
        void *tmp = screen->pixels;
        screen->pixels = screen->pixels_out;
        screen->pixels_out = tmp;

        if (!driver->is_ili9488) {
            spi_display_dma_write(&driver->bus.spi_disp, screen_width * sizeof(uint16_t), screen->pixels_out);
        } else {
            void *tmpb = screen->bytes;
            screen->bytes = screen->bytes_out;
            screen->bytes_out = tmpb;

            rgb565_swapped_line_to_rgb888(screen->bytes_out, screen->pixels_out, screen_width);
            spi_display_dma_write(&driver->bus.spi_disp, screen_width * 3, screen->bytes_out);
        }

        transaction_in_progress = true;
    }

    if (transaction_in_progress) {
        spi_transaction_t *trans;
        spi_device_get_trans_result(driver->bus.spi_disp.handle, &trans, portMAX_DELAY);
    }

    spi_device_release_bus(driver->bus.spi_disp.handle);

    display_items_delete(items, len);
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

    struct DCSLCDDriver *driver = DCS_LCD_DRIVER_FROM_CTX(ctx);

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

        dcs_lcd_draw_buffer(&driver->bus, screen, driver->is_ili9488 ? 3 : 2, x, y, width, height, data);

        // draw_buffer is fire-and-forget.
        return;

    } else if (cmd == globalcontext_make_atom(ctx->global, "\xA"
                                                           "load_image")) {
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

    display_message_send(gen_message.pid, return_tuple, ctx->global);
    END_WITH_STACK_HEAP(heap, ctx->global);
}

static void set_rotation(struct DCSLCDDriver *driver, int rotation)
{
    uint8_t madctl = 0;

    if (driver->madctl_bgr) {
        madctl |= DCS_LCD_MAD_BGR;
    }

    switch (rotation & 3) {
        case 0:
            madctl |= DCS_LCD_MAD_MX;
            break;

        case 1:
            madctl |= DCS_LCD_MAD_MV;
            break;

        case 2:
            madctl |= DCS_LCD_MAD_MY;
            break;

        case 3:
            madctl |= DCS_LCD_MAD_MX | DCS_LCD_MAD_MY | DCS_LCD_MAD_MV;
            break;
    }

    spi_dc_write_command(&driver->bus, DCS_LCD_MADCTL);
    spi_dc_write_data(&driver->bus, madctl);
}

Context *ili948x_display_create_port(GlobalContext *global, term opts)
{
    Context *ctx = context_new(global);
    ctx->native_handler = display_task_consume_mailbox;
    display_init(ctx, opts);
    return ctx;
}

static void display_init(Context *ctx, term opts)
{
    screen = calloc(1, sizeof(struct DCSLCDScreen));

    struct DCSLCDDriver *driver = malloc(sizeof(struct DCSLCDDriver));

    driver->display_args.messages_queue = xQueueCreate(32, sizeof(Message *));
    driver->display_args.process_message_fn = process_message;
    driver->display_args.ctx = ctx;
    ctx->platform_data = &driver->display_args;

    driver->ctx = ctx;

    struct SPIDisplayConfig spi_config;
    spi_display_init_config(&spi_config);
    spi_config.mode = SPI_MODE;
    spi_config.clock_speed_hz = SPI_CLOCK_HZ;
    spi_display_parse_config(&spi_config, opts, ctx->global);
    spi_display_init(&driver->bus.spi_disp, &spi_config);

    bool ok = display_common_gpio_from_opts(opts, ATOM_STR("\x2", "dc"), &driver->bus.dc_gpio, ctx->global);
    ok = ok && display_common_gpio_from_opts(opts, ATOM_STR("\x5", "reset"), &driver->reset_gpio, ctx->global);

    term compat_value_term = interop_kv_get_value_default(opts, ATOM_STR("\xA", "compatible"), term_nil(), ctx->global);
    int str_ok;
    char *compat_string = interop_term_to_string(compat_value_term, &str_ok);

    bool is_ili9486 = false;
    bool is_ili9488 = false;
    if (str_ok && compat_string) {
        is_ili9486 = !strcmp(compat_string, "ilitek,ili9486");
        is_ili9488 = !strcmp(compat_string, "ilitek,ili9488");
        free(compat_string);
    } else {
        ok = false;
    }

    if (!is_ili9486 && !is_ili9488) {
        ok = false;
    }
    driver->is_ili9488 = is_ili9488;

    // color_order: rgb|bgr (default: bgr)
    term color_order_term = interop_kv_get_value_default(opts, ATOM_STR("\xB", "color_order"), term_nil(), ctx->global);

    if (term_is_nil(color_order_term)) {
        driver->madctl_bgr = true;
    } else if (term_is_atom(color_order_term)) {
        if (color_order_term == context_make_atom(ctx, "\x3"
                                                       "rgb")) {
            driver->madctl_bgr = false;
        } else if (color_order_term == context_make_atom(ctx, "\x3"
                                                              "bgr")) {
            driver->madctl_bgr = true;
        } else {
            ok = false;
        }
    } else {
        ok = false;
    }

    term rotation = interop_kv_get_value_default(opts, ATOM_STR("\x8", "rotation"), term_from_int(0), ctx->global);
    ok = ok && term_is_integer(rotation);
    driver->rotation = term_to_int(rotation);

    term invon = interop_kv_get_value_default(opts, ATOM_STR("\x10", "enable_tft_invon"), FALSE_ATOM, ctx->global);
    ok = ok && ((invon == TRUE_ATOM) || (invon == FALSE_ATOM));
    bool enable_tft_invon = (invon == TRUE_ATOM);

    if (UNLIKELY(!ok)) {
        ESP_LOGE(TAG, "Failed init: invalid display parameters.");
        return;
    }

    // Swap w/h for 90/270.
    if (driver->rotation & 1) {
        screen->w = ILI948X_TFTHEIGHT;
        screen->h = ILI948X_TFTWIDTH;
    } else {
        screen->w = ILI948X_TFTWIDTH;
        screen->h = ILI948X_TFTHEIGHT;
    }

    screen->pixels = heap_caps_malloc(screen->w * sizeof(uint16_t), MALLOC_CAP_DMA);
    screen->pixels_out = heap_caps_malloc(screen->w * sizeof(uint16_t), MALLOC_CAP_DMA);

    if (driver->is_ili9488) {
        screen->bytes = heap_caps_malloc(screen->w * 3, MALLOC_CAP_DMA);
        screen->bytes_out = heap_caps_malloc(screen->w * 3, MALLOC_CAP_DMA);
    } else {
        screen->bytes = NULL;
        screen->bytes_out = NULL;
    }

    // Reset.
    spi_device_acquire_bus(driver->bus.spi_disp.handle, portMAX_DELAY);
    gpio_set_direction(driver->reset_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(driver->reset_gpio, 1);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gpio_set_level(driver->reset_gpio, 0);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    gpio_set_level(driver->reset_gpio, 1);
    spi_device_release_bus(driver->bus.spi_disp.handle);

    gpio_set_direction(driver->bus.dc_gpio, GPIO_MODE_OUTPUT);

    spi_dc_write_command(&driver->bus, DCS_LCD_SWRESET);

    vTaskDelay(5 / portTICK_PERIOD_MS);

    if (driver->is_ili9488) {
        display_init_9488(driver);
    } else {
        display_init_9486(driver);
    }

    spi_dc_write_command(&driver->bus, DCS_LCD_SLPOUT);

    vTaskDelay(120 / portTICK_PERIOD_MS);

    spi_dc_write_command(&driver->bus, DCS_LCD_DISPON);

    if (enable_tft_invon) {
        spi_dc_write_command(&driver->bus, DCS_LCD_INVON);
    } else {
        spi_dc_write_command(&driver->bus, DCS_LCD_INVOFF);
    }

    set_rotation(driver, driver->rotation);

    struct BacklightGPIOConfig backlight_config;
    backlight_gpio_init_config(&backlight_config);
    backlight_gpio_parse_config(&backlight_config, opts, ctx->global);
    backlight_gpio_init(&backlight_config);

    xTaskCreate(display_task_process_messages, "display", 10000, &driver->display_args, 1, NULL);
}

static void display_init_9486(struct DCSLCDDriver *driver)
{
    spi_dc_write_command(&driver->bus, ILI948X_IFMODE);
    spi_dc_write_data(&driver->bus, 0x00);

    spi_dc_write_command(&driver->bus, DCS_LCD_COLMOD);
    spi_dc_write_data(&driver->bus, 0x55);

    spi_dc_write_command(&driver->bus, ILI948X_PWRCTR3);
    spi_dc_write_data(&driver->bus, 0x44);

    spi_dc_write_command(&driver->bus, ILI948X_VMCTR1);
    spi_dc_write_data(&driver->bus, 0x00);
    spi_dc_write_data(&driver->bus, 0x00);
    spi_dc_write_data(&driver->bus, 0x00);
    spi_dc_write_data(&driver->bus, 0x00);

    spi_dc_write_command(&driver->bus, ILI948X_PGAMCTRL);
    spi_dc_write_data(&driver->bus, 0x0f);
    spi_dc_write_data(&driver->bus, 0x1f);
    spi_dc_write_data(&driver->bus, 0x1c);
    spi_dc_write_data(&driver->bus, 0x0c);
    spi_dc_write_data(&driver->bus, 0x0f);
    spi_dc_write_data(&driver->bus, 0x08);
    spi_dc_write_data(&driver->bus, 0x48);
    spi_dc_write_data(&driver->bus, 0x98);
    spi_dc_write_data(&driver->bus, 0x37);
    spi_dc_write_data(&driver->bus, 0x0a);
    spi_dc_write_data(&driver->bus, 0x13);
    spi_dc_write_data(&driver->bus, 0x04);
    spi_dc_write_data(&driver->bus, 0x11);
    spi_dc_write_data(&driver->bus, 0x0d);
    spi_dc_write_data(&driver->bus, 0x00);

    spi_dc_write_command(&driver->bus, ILI948X_NGAMCTRL);
    spi_dc_write_data(&driver->bus, 0x0f);
    spi_dc_write_data(&driver->bus, 0x32);
    spi_dc_write_data(&driver->bus, 0x2e);
    spi_dc_write_data(&driver->bus, 0x0b);
    spi_dc_write_data(&driver->bus, 0x0d);
    spi_dc_write_data(&driver->bus, 0x05);
    spi_dc_write_data(&driver->bus, 0x47);
    spi_dc_write_data(&driver->bus, 0x75);
    spi_dc_write_data(&driver->bus, 0x37);
    spi_dc_write_data(&driver->bus, 0x06);
    spi_dc_write_data(&driver->bus, 0x10);
    spi_dc_write_data(&driver->bus, 0x03);
    spi_dc_write_data(&driver->bus, 0x24);
    spi_dc_write_data(&driver->bus, 0x20);
    spi_dc_write_data(&driver->bus, 0x00);

    spi_dc_write_command(&driver->bus, ILI948X_DGAMCTRL);
    spi_dc_write_data(&driver->bus, 0x0f);
    spi_dc_write_data(&driver->bus, 0x32);
    spi_dc_write_data(&driver->bus, 0x2e);
    spi_dc_write_data(&driver->bus, 0x0b);
    spi_dc_write_data(&driver->bus, 0x0d);
    spi_dc_write_data(&driver->bus, 0x05);
    spi_dc_write_data(&driver->bus, 0x47);
    spi_dc_write_data(&driver->bus, 0x75);
    spi_dc_write_data(&driver->bus, 0x37);
    spi_dc_write_data(&driver->bus, 0x06);
    spi_dc_write_data(&driver->bus, 0x10);
    spi_dc_write_data(&driver->bus, 0x03);
    spi_dc_write_data(&driver->bus, 0x24);
    spi_dc_write_data(&driver->bus, 0x20);
    spi_dc_write_data(&driver->bus, 0x00);
}

static void display_init_9488(struct DCSLCDDriver *driver)
{
    // ILI9488: RGB666 over SPI (3 bytes/pixel).
    spi_dc_write_command(&driver->bus, ILI948X_IFMODE);
    spi_dc_write_data(&driver->bus, 0x00);

    spi_dc_write_command(&driver->bus, ILI948X_ADJCTRL3);
    spi_dc_write_data(&driver->bus, 0xA9);
    spi_dc_write_data(&driver->bus, 0x51);
    spi_dc_write_data(&driver->bus, 0x2C);
    spi_dc_write_data(&driver->bus, 0x82);

    spi_dc_write_command(&driver->bus, ILI948X_PWRCTR1);
    spi_dc_write_data(&driver->bus, 0x11);
    spi_dc_write_data(&driver->bus, 0x09);

    spi_dc_write_command(&driver->bus, ILI948X_PWRCTR2);
    spi_dc_write_data(&driver->bus, 0x41);

    spi_dc_write_command(&driver->bus, ILI948X_VMCTR1);
    spi_dc_write_data(&driver->bus, 0x00);
    spi_dc_write_data(&driver->bus, 0x0A);
    spi_dc_write_data(&driver->bus, 0x80);

    spi_dc_write_command(&driver->bus, ILI948X_FRMCTR1);
    spi_dc_write_data(&driver->bus, 0xB0);
    spi_dc_write_data(&driver->bus, 0x11);

    spi_dc_write_command(&driver->bus, ILI948X_INVCTR);
    spi_dc_write_data(&driver->bus, 0x02);

    spi_dc_write_command(&driver->bus, ILI948X_DFUNCTR);
    spi_dc_write_data(&driver->bus, 0x02);
    spi_dc_write_data(&driver->bus, 0x02);

    spi_dc_write_command(&driver->bus, ILI948X_ETMOD);
    spi_dc_write_data(&driver->bus, 0xC6);

    spi_dc_write_command(&driver->bus, ILI948X_HS_LANES_CTRL);
    spi_dc_write_data(&driver->bus, 0x00);
    spi_dc_write_data(&driver->bus, 0x04);

    spi_dc_write_command(&driver->bus, ILI948X_IMAGE_FUNCTION);
    spi_dc_write_data(&driver->bus, 0x00);

    spi_dc_write_command(&driver->bus, DCS_LCD_COLMOD);
    spi_dc_write_data(&driver->bus, 0x66);

    spi_dc_write_command(&driver->bus, ILI948X_PGAMCTRL);
    spi_dc_write_data(&driver->bus, 0x00);
    spi_dc_write_data(&driver->bus, 0x07);
    spi_dc_write_data(&driver->bus, 0x10);
    spi_dc_write_data(&driver->bus, 0x09);
    spi_dc_write_data(&driver->bus, 0x17);
    spi_dc_write_data(&driver->bus, 0x0B);
    spi_dc_write_data(&driver->bus, 0x41);
    spi_dc_write_data(&driver->bus, 0x89);
    spi_dc_write_data(&driver->bus, 0x4B);
    spi_dc_write_data(&driver->bus, 0x0A);
    spi_dc_write_data(&driver->bus, 0x0C);
    spi_dc_write_data(&driver->bus, 0x0E);
    spi_dc_write_data(&driver->bus, 0x18);
    spi_dc_write_data(&driver->bus, 0x1B);
    spi_dc_write_data(&driver->bus, 0x0F);

    spi_dc_write_command(&driver->bus, ILI948X_NGAMCTRL);
    spi_dc_write_data(&driver->bus, 0x00);
    spi_dc_write_data(&driver->bus, 0x17);
    spi_dc_write_data(&driver->bus, 0x1A);
    spi_dc_write_data(&driver->bus, 0x04);
    spi_dc_write_data(&driver->bus, 0x0E);
    spi_dc_write_data(&driver->bus, 0x06);
    spi_dc_write_data(&driver->bus, 0x2F);
    spi_dc_write_data(&driver->bus, 0x45);
    spi_dc_write_data(&driver->bus, 0x43);
    spi_dc_write_data(&driver->bus, 0x02);
    spi_dc_write_data(&driver->bus, 0x0A);
    spi_dc_write_data(&driver->bus, 0x09);
    spi_dc_write_data(&driver->bus, 0x32);
    spi_dc_write_data(&driver->bus, 0x36);
    spi_dc_write_data(&driver->bus, 0x0F);
}
