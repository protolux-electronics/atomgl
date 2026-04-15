/*
 * This file is part of AtomGL.
 *
 * Copyright 2024 Davide Bettio <davide@uninstall.it>
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

#include <string.h>

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/task.h>

#include <context.h>
#include <defaultatoms.h>
#include <globalcontext.h>
#include <interop.h>
#include <mailbox.h>
#include <port.h>
#include <term.h>
#include <utils.h>

#include <i2c_driver.h>

#include "display_common.h"
#include "display_message.h"
#include "display_task.h"
#include "image_helpers.h"
#include "oled_commands.h"

#define TAG "SSD1306"

#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define PAGE_HEIGHT 8
#define PAGES_NUM 8

#define I2C_ADDRESS 0x3C

#define CTRL_BYTE_CMD_SINGLE 0x80
#define CTRL_BYTE_CMD_STREAM 0x00
#define CTRL_BYTE_DATA_STREAM 0x40

#define CMD_DISPLAY_INVERTED 0xA7
#define CMD_DISPLAY_ON 0xAF

typedef enum
{
    DisplayTypeSsd1306,
    DisplayTypeSsd1315,
    DisplayTypeSh1106,
} display_type_t;

struct OLEDDriver
{
    term i2c_host;
    display_type_t type;
    Context *ctx;

    struct DisplayTaskArgs display_args;
};

#define OLED_DRIVER_FROM_CTX(ctx) \
    CONTAINER_OF((struct DisplayTaskArgs *) (ctx)->platform_data, struct OLEDDriver, display_args)

static struct MonoScreen *mono_screen;

#include "font_data.h"
#include "display_items.h"
#include "mono_draw.h"

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

    int screen_width = DISPLAY_WIDTH;
    int screen_height = DISPLAY_HEIGHT;
    struct OLEDDriver *driver = OLED_DRIVER_FROM_CTX(ctx);

    int memsize = (DISPLAY_WIDTH * (PAGE_HEIGHT + 1)) / sizeof(uint8_t);
    uint8_t *buf = malloc(memsize);
    memset(buf, 0, memsize);

    i2c_port_t i2c_num;
    if (i2c_driver_acquire(driver->i2c_host, &i2c_num, ctx->global) != I2CAcquireOk) {
        fprintf(stderr, "Invalid I2C peripheral\n");
        return;
    }

    for (int ypos = 0; ypos < screen_height; ypos++) {
        int xpos = 0;
        while (xpos < screen_width) {
            int drawn_pixels = mono_draw_x(mono_screen, buf, xpos, ypos, items, len);
            xpos += drawn_pixels;
        }

        uint8_t *out_buf = buf + (DISPLAY_WIDTH / 8);
        for (int i = 0; i < DISPLAY_WIDTH; i++) {
            out_buf[i] |= ((buf[i / 8] >> (i % 8)) & 1) << (ypos % 8);
        }

        if ((ypos % PAGE_HEIGHT) == (PAGE_HEIGHT - 1)) {
            i2c_cmd_handle_t cmd;
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

            i2c_master_write_byte(cmd, CTRL_BYTE_CMD_SINGLE, true);
            i2c_master_write_byte(cmd, 0xB0 | ypos / 8, true);

            if (driver->type == DisplayTypeSh1106 || driver->type == DisplayTypeSsd1315) {
                // SSD1315 and SH1106 require explicit column address reset
                i2c_master_write_byte(cmd, CTRL_BYTE_CMD_SINGLE, true);
                i2c_master_write_byte(cmd, 0x00, true);
                i2c_master_write_byte(cmd, CTRL_BYTE_CMD_SINGLE, true);
                i2c_master_write_byte(cmd, 0x10, true);
            }
            i2c_master_write_byte(cmd, CTRL_BYTE_DATA_STREAM, true);


            if (driver->type == DisplayTypeSh1106) {
                // add 2 empty pages on sh1106 since it can have up to 132 pixels
                // and 128 pixel screen starts at (2, 0)
                i2c_master_write_byte(cmd, 0, true);
                i2c_master_write_byte(cmd, 0, true);
            }

            for (uint8_t j = 0; j < DISPLAY_WIDTH; j++) {
                i2c_master_write_byte(cmd, out_buf[j], true);
            }

            // no need to send the last 2 page, the position will be set on next line again
            // if (driver->type == DisplayTypeSh1106) {
            //    i2c_master_write_byte(cmd, 0, true);
            //    i2c_master_write_byte(cmd, 0, true);
            // }

            i2c_master_stop(cmd);
            i2c_master_cmd_begin(i2c_num, cmd, 100 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);

            memset(buf, 0, memsize);
        }
    }

    i2c_driver_release(driver->i2c_host, ctx->global);

    free(buf);
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

    if (cmd == context_make_atom(ctx, "\x6"
                                      "update")) {
        term display_list = term_get_tuple_element(req, 1);
        do_update(ctx, display_list);

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

    display_message_send(gen_message.pid, return_tuple, ctx->global);
    END_WITH_STACK_HEAP(heap, ctx->global);
}

static void display_init(Context *ctx, term opts)
{
    GlobalContext *glb = ctx->global;

    term i2c_host
        = interop_kv_get_value_default(opts, ATOM_STR("\x8", "i2c_host"), term_invalid_term(), glb);
    if (i2c_host == term_invalid_term()) {
        ESP_LOGE(TAG, "Missing i2c_host config option.");
        return;
    }

    bool invert = interop_kv_get_value(opts, ATOM_STR("\x6", "invert"), glb) == TRUE_ATOM;

    mono_screen = calloc(1, sizeof(struct MonoScreen));
    mono_screen->w = DISPLAY_WIDTH;
    mono_screen->h = DISPLAY_HEIGHT;

    struct OLEDDriver *driver = malloc(sizeof(struct OLEDDriver));

    driver->display_args.messages_queue = xQueueCreate(32, sizeof(Message *));
    driver->display_args.process_message_fn = process_message;
    driver->display_args.ctx = ctx;
    ctx->platform_data = &driver->display_args;

    driver->ctx = ctx;
    driver->type = DisplayTypeSsd1306; // Default to SSD1306

    term compat_value_term = interop_kv_get_value_default(opts, ATOM_STR("\xA", "compatible"), term_nil(), ctx->global);
    int str_ok;
    char *compat_string = interop_term_to_string(compat_value_term, &str_ok);

    if (!(str_ok && compat_string)) {
        ESP_LOGE(TAG, "No Compatible Device Found.");
        return;
    }

    if (!strcmp(compat_string, "sino-wealth,sh1106")) {
        driver->type = DisplayTypeSh1106;
    } else if (!strcmp(compat_string, "solomon-systech,ssd1315")) {
        driver->type = DisplayTypeSsd1315;
    }

    free(compat_string);

    int reset_gpio;
    if (!display_common_gpio_from_opts(opts, ATOM_STR("\x5", "reset"), &reset_gpio, glb)) {
        ESP_LOGI(TAG, "Reset GPIO not configured.");
    } else {
        gpio_set_direction(reset_gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(reset_gpio, 0);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_set_level(reset_gpio, 1);
    }

    i2c_port_t i2c_num;
    if (i2c_driver_acquire(i2c_host, &i2c_num, glb) != I2CAcquireOk) {
        fprintf(stderr, "Invalid I2C peripheral\n");
        return;
    }
    driver->i2c_host = i2c_host;

    const uint8_t *init_seq;
    size_t init_seq_len;
    if (driver->type == DisplayTypeSsd1315) {
        init_seq = oled_init_seq_ssd1315;
        init_seq_len = oled_init_seq_ssd1315_len;
    } else {
        init_seq = oled_init_seq_ssd1306;
        init_seq_len = oled_init_seq_ssd1306_len;
    }
    oled_execute_init_seq(i2c_num, I2C_ADDRESS, init_seq, init_seq_len);

    // Driver-controlled finalization: optional invert, then display ON.
    // These depend on a runtime opt and are not panel data, so they
    // stay outside the per-variant init array.
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, CTRL_BYTE_CMD_STREAM, true);
    if (invert) {
        i2c_master_write_byte(cmd, CMD_DISPLAY_INVERTED, true);
    }
    i2c_master_write_byte(cmd, CMD_DISPLAY_ON, true);
    i2c_master_stop(cmd);

    esp_err_t res = i2c_master_cmd_begin(i2c_num, cmd, 50 / portTICK_PERIOD_MS);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "ssd1306/ssd1315 OLED configuration failed. error: 0x%.2X", res);
    } else {
        xTaskCreate(display_task_process_messages, "display", 10000, &driver->display_args, 1, NULL);
    }

    i2c_cmd_link_delete(cmd);
    i2c_driver_release(i2c_host, glb);
}

Context *ssd1306_display_create_port(GlobalContext *global, term opts)
{
    Context *ctx = context_new(global);
    ctx->native_handler = display_task_consume_mailbox;
    display_init(ctx, opts);

    return ctx;
}
