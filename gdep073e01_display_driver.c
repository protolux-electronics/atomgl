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
#include "epaper_color.h"
#include "epaper_draw.h"
#include "epaper_screen.h"
#include "image_helpers.h"
#include "spi_dc_driver.h"
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

#define REPORT_UNEXPECTED_MSGS 0
#define SELF_TEST 0

static const char *TAG = "gdep073e01";

static void clear_screen(Context *ctx, int color);

struct SPI
{
    struct SPIDCBus bus;

    int busy_gpio;
    int reset_gpio;

    Context *ctx;

    int count_to_refresh;
    uint64_t last_refresh;

    struct DisplayTaskArgs display_args;
};

#define SPI_FROM_CTX(ctx) \
    CONTAINER_OF((struct DisplayTaskArgs *) (ctx)->platform_data, struct SPI, display_args)

static struct EpaperScreen *screen;

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
        display_items_init_item(&items[i], term_get_list_head(t), ctx);
        t = term_get_list_tail(t);
    }

    int screen_width = DISPLAY_WIDTH;
    int screen_height = DISPLAY_HEIGHT;
    struct SPI *spi = SPI_FROM_CTX(ctx);

#if 0
    // resolution command
    spi_dc_write_command(&spi->bus, 0x61);
    spi_dc_write_data(&spi->bus, 0x02);
    spi_dc_write_data(&spi->bus, 0x58);
    spi_dc_write_data(&spi->bus, 0x01);
    spi_dc_write_data(&spi->bus, 0xC0);
#endif

    // update command
    spi_dc_write_command(&spi->bus, 0x10);

    uint8_t *buf = heap_caps_malloc(DISPLAY_WIDTH / 2, MALLOC_CAP_DMA);
    memset(buf, 0x11, DISPLAY_WIDTH / 2);

    bool transaction_in_progress = false;

    spi_device_acquire_bus(spi->bus.spi_disp.handle, portMAX_DELAY);

    for (int ypos = 0; ypos < screen_height; ypos++) {
        if (transaction_in_progress) {
            spi_transaction_t *trans = NULL;
            spi_device_get_trans_result(spi->bus.spi_disp.handle, &trans, portMAX_DELAY);
        }

        int xpos = 0;
        while (xpos < screen_width) {
            int drawn_pixels = epaper_draw_x(screen, buf, xpos, ypos, items, len);
            xpos += drawn_pixels;
        }

        spi_display_dma_write(&spi->bus.spi_disp, DISPLAY_WIDTH / 2, buf);
        transaction_in_progress = true;
    }

    if (transaction_in_progress) {
        spi_transaction_t *trans = NULL;
        spi_device_get_trans_result(spi->bus.spi_disp.handle, &trans, portMAX_DELAY);
    }

    spi_device_release_bus(spi->bus.spi_disp.handle);

    free(buf);

    // not sure if we should add 0x11, which is end of data command or not

    // power on command
    spi_dc_write_command(&spi->bus, 0x04);
    wait_busy_level(spi, 1);

    // refresh command
    spi_dc_write_command(&spi->bus, 0x12);
    uint8_t refresh_data[] = {0x00};
    spi_dc_write_data_n(&spi->bus, refresh_data, sizeof(refresh_data));
    wait_busy_level(spi, 1);

    // power off command
    spi_dc_write_command(&spi->bus, 0x02);
    wait_busy_level(spi, 1);

    display_items_delete(items, len);

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

    display_message_send(gen_message.pid, return_tuple, ctx->global);
    END_WITH_STACK_HEAP(heap, ctx->global);
}

static void clear_screen(Context *ctx, int color)
{
    struct SPI *spi = SPI_FROM_CTX(ctx);

    uint8_t *buf = heap_caps_malloc(DISPLAY_WIDTH / 2, MALLOC_CAP_DMA);

#if 0
    spi_dc_write_command(&spi->bus, 0x61);
    spi_dc_write_data(&spi->bus, 0x02);
    spi_dc_write_data(&spi->bus, 0x58);
    spi_dc_write_data(&spi->bus, 0x01);
    spi_dc_write_data(&spi->bus, 0xC0);
#endif
    spi_dc_write_command(&spi->bus, 0x10);

    bool transaction_in_progress = false;

    spi_device_acquire_bus(spi->bus.spi_disp.handle, portMAX_DELAY);

    for (int i = 0; i < DISPLAY_HEIGHT; i++) {
        if (transaction_in_progress) {
            spi_transaction_t *trans = NULL;
            spi_device_get_trans_result(spi->bus.spi_disp.handle, &trans, portMAX_DELAY);
        }

        // let's ensure a memset otherwise we might generate odd artifacts
        memset(buf, color | (color << 4), DISPLAY_WIDTH / 2);
        spi_display_dma_write(&spi->bus.spi_disp, DISPLAY_WIDTH / 2, buf);
        transaction_in_progress = true;
    }

    if (transaction_in_progress) {
        spi_transaction_t *trans = NULL;
        spi_device_get_trans_result(spi->bus.spi_disp.handle, &trans, portMAX_DELAY);
    }

    spi_device_release_bus(spi->bus.spi_disp.handle);

    free(buf);

    spi_dc_write_command(&spi->bus, 0x04);
    wait_busy_level(spi, 1);
    spi_dc_write_command(&spi->bus, 0x12);
    uint8_t refresh_data[] = {0x00};
    spi_dc_write_data_n(&spi->bus, refresh_data, sizeof(refresh_data));
    wait_busy_level(spi, 1);
    spi_dc_write_command(&spi->bus, 0x02);
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
    spi_display_init(&spi->bus.spi_disp, &spi_config);

    bool ok = display_common_gpio_from_opts(opts, ATOM_STR("\x4", "busy"), &spi->busy_gpio, ctx->global);
    ok = ok && display_common_gpio_from_opts(opts, ATOM_STR("\x2", "dc"), &spi->bus.dc_gpio, ctx->global);
    ok = ok && display_common_gpio_from_opts(opts, ATOM_STR("\x5", "reset"), &spi->reset_gpio, ctx->global);
    if (UNLIKELY(!ok)) {
        ESP_LOGE(TAG, "Failed init: invalid display GPIOs.");
        return;
    }

    gpio_set_direction(spi->reset_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(spi->reset_gpio, 1);
    gpio_set_direction(spi->bus.dc_gpio, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(spi->bus.dc_gpio, GPIO_PULLUP_ENABLE);
    gpio_set_direction(spi->busy_gpio, GPIO_MODE_INPUT);
    gpio_set_pull_mode(spi->busy_gpio, GPIO_PULLUP_ENABLE);
    gpio_set_level(spi->bus.dc_gpio, 0);

    display_reset(spi);

    wait_busy_level(spi, 1);

    spi_dc_write_command(&spi->bus, 0xAA);
    uint8_t psr1_data[] = {0x49, 0x55, 0x20, 0x08, 0x09, 0x18};
    spi_dc_write_data_n(&spi->bus, psr1_data, sizeof(psr1_data));
    wait_busy_level(spi, 1);

    spi_dc_write_command(&spi->bus, PWRR);
    uint8_t pwrr_data[] = {0x3F};
    spi_dc_write_data_n(&spi->bus, pwrr_data, sizeof(pwrr_data));
    wait_busy_level(spi, 1);

    spi_dc_write_command(&spi->bus, PSR);
    uint8_t psr_data[] = {0x5F, 0x69};
    spi_dc_write_data_n(&spi->bus, psr_data, sizeof(psr_data));
    wait_busy_level(spi, 1);

    spi_dc_write_command(&spi->bus, POFS);
    uint8_t pofs_data[] = {0x00, 0x54, 0x00, 0x44};
    spi_dc_write_data_n(&spi->bus, pofs_data, sizeof(pofs_data));
    wait_busy_level(spi, 1);

    spi_dc_write_command(&spi->bus, BTST1);
    uint8_t btst1_data[] = {0x40, 0x1F, 0x1F, 0x2C};
    spi_dc_write_data_n(&spi->bus, btst1_data, sizeof(btst1_data));
    wait_busy_level(spi, 1);

    spi_dc_write_command(&spi->bus, BTST2);
    uint8_t btst2_data[] = {0x6F, 0x1F, 0x17, 0x49};
    spi_dc_write_data_n(&spi->bus, btst2_data, sizeof(btst2_data));
    wait_busy_level(spi, 1);

    spi_dc_write_command(&spi->bus, BTST3);
    uint8_t btst3_data[] = {0x6F, 0x1F, 0x1F, 0x22};
    spi_dc_write_data_n(&spi->bus, btst3_data, sizeof(btst3_data));
    wait_busy_level(spi, 1);

    spi_dc_write_command(&spi->bus, PLL);
    uint8_t pll_data[] = {0x00};
    spi_dc_write_data_n(&spi->bus, pll_data, sizeof(pll_data));
    wait_busy_level(spi, 1);

    spi_dc_write_command(&spi->bus, CDI);
    uint8_t cdi_data[] = {0x3F};
    spi_dc_write_data_n(&spi->bus, cdi_data, sizeof(cdi_data));
    wait_busy_level(spi, 1);

    spi_dc_write_command(&spi->bus, TCON);
    uint8_t tcon_data[] = {0x02, 0x00};
    spi_dc_write_data_n(&spi->bus, tcon_data, sizeof(tcon_data));
    wait_busy_level(spi, 1);

    spi_dc_write_command(&spi->bus, TRES);
    uint8_t tres_data[] = {0x03, 0x20, 0x01, 0xe0};
    spi_dc_write_data_n(&spi->bus, tres_data, sizeof(tres_data));
    wait_busy_level(spi, 1);

    spi_dc_write_command(&spi->bus, T_VDCS);
    uint8_t vdcs_data[] = {0x01};
    spi_dc_write_data_n(&spi->bus, vdcs_data, sizeof(vdcs_data));
    wait_busy_level(spi, 1);

    spi_dc_write_command(&spi->bus, PWS);
    uint8_t pws_data[] = {0x2F};
    spi_dc_write_data_n(&spi->bus, pws_data, sizeof(pws_data));
    wait_busy_level(spi, 1);

    // PON
    spi_dc_write_command(&spi->bus, 0x04);
    wait_busy_level(spi, 1);

    ctx->platform_data = &spi->display_args;

    spi->ctx = ctx;

    screen = calloc(1, sizeof(struct EpaperScreen));
    screen->w = DISPLAY_WIDTH;
    screen->h = DISPLAY_HEIGHT;
    screen->palette = epaper_gdep073e01_palette;
    screen->palette_size = 7;

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
    xTaskCreate(display_task_process_messages, "display", 10000, &spi->display_args, 1, NULL);
#endif
}

Context *gdep073e01_display_driver_create_port(GlobalContext *global, term opts)
{
    Context *ctx = context_new(global);
    ctx->native_handler = display_task_consume_mailbox;
    display_spi_init(ctx, opts);
    return ctx;
}
