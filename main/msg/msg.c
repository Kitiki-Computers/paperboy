//
// Modos Smooth Graphics
// Copyright 2026 Wenting Zhang
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// System includes
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// Driver includes
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "rom/ets_sys.h"
#include <driver/gpio.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
// App includes
#include "audio.h"
#include "paperboy_config.h"
#include "msg.h"

//#define TIME_PRINT

// Private
static const char *TAG = "msg";

static volatile bool dma_done = false;
static esp_lcd_i80_bus_handle_t i80_bus_handle = NULL;
static esp_lcd_panel_io_handle_t panel_io_handle = NULL;
static volatile uint32_t s_vsync_count = 0;

/* Core-1 load measurement (debug only — feeds the profiler/perf-HUD). The msg
 * task drives the e-paper field-sequentially, computing each row's waveform then
 * busy-spinning (msg_send_row) on the prior row's DMA. We split each field into
 * "busy" (compute) and "idle" (DMA spin) us. Compiled out in a release build so
 * the per-spin esp_timer_get_time() reads on Core 1 cost nothing. Written by the
 * Core-1 task, read by the Core-0 profiler — single-word volatile is atomic on
 * Xtensa LX7. */
#if defined(PAPERBOY_DEBUG) && PAPERBOY_DEBUG
static volatile uint32_t s_msg_busy_us = 0;   /* last field: compute us       */
static volatile uint32_t s_msg_idle_us = 0;   /* last field: DMA-spin us       */
static volatile uint32_t s_msg_field_us = 0;  /* last field: total driving us  */
static uint32_t s_field_spin_us = 0;          /* per-field accumulator (C1)    */
#endif

// Triple buffer:
// prev-buffer: Previous, for partial update use
// front-buffer: Current image, being sent out
// back-buffer: For application to render into
static uint8_t *fb[2];
static volatile int front_buffer;
static uint8_t *statebuf;
static uint8_t *img_src;
static volatile bool img_req;
static volatile bool img_to_white;
static volatile bool flip_req;
static volatile bool enable_video;
static TaskHandle_t flip_waiter_task;

// Ping-pong DMA buffer
static uint8_t *dma_buf[2];
static int dma_front_buffer;

// Push value lookup table
static uint8_t push_lut_w[16];
static uint8_t push_lut_b[16];

static bool dma_done_callback(esp_lcd_panel_io_handle_t panel_io,
        esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
    dma_done = true;
    return false;
}

static void msg_power_on() {
    // Enable power
    gpio_set_level(EPD_PWR_EN_PIN, 1);
    ets_delay_us(100);
    gpio_set_level(EPD_BST_EN_PIN, 1);
    ets_delay_us(100);
    // Set default IO states
    gpio_set_level(EPD_GDCK_PIN, 1);
    gpio_set_level(EPD_GDSP_PIN, 1);
}

static void msg_power_off() {
    // Reset IO levels
    gpio_set_level(EPD_GDCK_PIN, 0);
    gpio_set_level(EPD_GDSP_PIN, 0);
    // Disable power
    gpio_set_level(EPD_BST_EN_PIN, 0);
    ets_delay_us(100);
    gpio_set_level(EPD_PWR_EN_PIN, 0);
    ets_delay_us(100);
}

/* Wait for the previous row's DMA to finish, but never forever: a healthy
 * e-paper row transfer completes in ~12 us, so if dma_done hasn't arrived in
 * ~1.5 ms the trans-done interrupt was missed and that row is wedged (this can
 * happen when a flash write on Core 0 stalls the other CPU and masks the i80
 * ISR, desyncing dma_done from the hardware for one field). Bounding the spin
 * is what keeps a wedge from freezing the device: with the old 50 ms bound a
 * single desynced field of ~510 rows stalled the panel for ~25 s; at 1.5 ms the
 * worst case is <1 s and the field self-resyncs at the next frame. 1.5 ms is
 * ~100x the real transfer time, so a healthy transfer never trips it. esp_timer
 * is read only every 16384 spins so the common fast path stays cheap. */
static inline void IRAM_ATTR dma_wait(void) {
    uint32_t n = 0;
    int64_t t0 = 0;
    while (!dma_done) {
        if ((++n & 0x3FFFu) == 0u) {
            int64_t now = esp_timer_get_time();
            if (t0 == 0) t0 = now;
            else if (now - t0 > 1500) { dma_done = true; break; }
        }
    }
}

static void IRAM_ATTR msg_send_row(uint8_t *data) {
    // Wait if last line hasn't finished
#if defined(PAPERBOY_DEBUG) && PAPERBOY_DEBUG
    if (!dma_done) {
        int64_t _t = esp_timer_get_time();
        dma_wait();
        s_field_spin_us += (uint32_t)(esp_timer_get_time() - _t);
    }
#else
    dma_wait();
#endif

    gpio_set_level(EPD_GDCK_PIN, 0);
    gpio_set_level(EPD_SDLE_PIN, 1);
    gpio_set_level(EPD_SDLE_PIN, 0);
    gpio_set_level(EPD_GDCK_PIN, 1);

    esp_lcd_panel_io_tx_color(panel_io_handle, -1, data,
        (EPD_WIDTH / 4) + EPD_LINE_PAD);

    dma_done = false;
}

static void IRAM_ATTR msg_update_task(void *arg) {
    int field_counter = 0;
#ifdef TIME_PRINT
    bool printed = false;
    bool img_printed = false;
    uint32_t last_print = 0;
#endif
    while (true) {
#if defined(PAPERBOY_DEBUG) && PAPERBOY_DEBUG
        int64_t frame_start_us = esp_timer_get_time();
        s_field_spin_us = 0;   // reset per-field DMA-spin accumulator
#endif

        // JUST KEEP CRANKING
        s_vsync_count++;
        if (flip_req) {
            front_buffer = !front_buffer;
            flip_req = false;
            if (flip_waiter_task != NULL) {
                xTaskNotifyGive(flip_waiter_task);
                flip_waiter_task = NULL;
            }
        }
        uint8_t *cur_buf = fb[front_buffer];

#ifdef TIME_PRINT
        uint32_t start = esp_timer_get_time();
        if ((start - last_print) > 1000000) {
            printed = false;
        }
#endif

        // Frame start sequence
        gpio_set_level(EPD_GDCK_PIN, 1);
        ets_delay_us(7);
        gpio_set_level(EPD_GDSP_PIN, 0);
        ets_delay_us(10);
        gpio_set_level(EPD_GDCK_PIN, 0);
        gpio_set_level(EPD_GDCK_PIN, 1);
        ets_delay_us(8);
        gpio_set_level(EPD_GDSP_PIN, 1);
        ets_delay_us(10);
        gpio_set_level(EPD_GDCK_PIN, 0);
        for (int i = 0; i < 2; i++) {
            gpio_set_level(EPD_GDCK_PIN, 1);
            gpio_set_level(EPD_GDCK_PIN, 0);
        }

        if (img_req) {
            // Send the image for fixed amount of frames
            // Send lines
            // if (printed && !img_printed) {
            //     ESP_LOGI(TAG, "In image mode");
            //     img_printed = true;
            //     printed = false;
            // }
            uint8_t *rdptr = img_src;
            uint8_t *lutptr = img_to_white ? push_lut_w : push_lut_b;
            for (int i = 0; i < EPD_HEIGHT; i++) {
                uint8_t *wrptr = dma_buf[dma_front_buffer];
                for (int j = 0; j < EPD_WIDTH / 8; j++) {
                    uint8_t rd = *rdptr++;
                    *wrptr++ = lutptr[rd >> 4];
                    *wrptr++ = lutptr[rd & 0xf];
                }
                msg_send_row(dma_buf[dma_front_buffer]);
                dma_front_buffer = !dma_front_buffer;
            }
            field_counter++;
            if (field_counter == EPD_IMAGE_FIELDS) {
                field_counter = 0;
                img_req = false;
            }
        }
        else if (enable_video) {
            // Video mode
            // if (!printed) {
            //     ESP_LOGI(TAG, "In video mode");
            // }
            // 30 dummy lines
            // Ensure both buffers are empty. We will not touch areas outside
            // of the window
            memset(dma_buf[0], 0, EPD_WIDTH / 4 + EPD_LINE_PAD);
            memset(dma_buf[1], 0, EPD_WIDTH / 4 + EPD_LINE_PAD);
            for (int i = 0; i < (30 - 2); i++) {
                msg_send_row(dma_buf[0]);
            }
            // Active video runs from line 30-510
            dma_front_buffer = 0;

            uint8_t *stptr = statebuf;
            uint8_t *rdptr = cur_buf;
            for (int i = 0; i < 480; i++) {
                // Each line is repeated 3 times
                // We divide the processing time across 3 lines
                // DMA buffer flipping only happens every 3 lines
                // For each line, we only process pixel 432 to 432+432
                // And each time we will only process 432/3=144 pixels
                uint8_t *wrptr = dma_buf[!dma_front_buffer];
                wrptr += 432/4; // Always skip first 432 pixels
                wrptr += (i % 3) * (144/4); // Skip processed pixels
                for (int j = 0; j < 144/8; j++) {
                    // Do 8 pixels each iteration
                    // In state byte: each nibble counts from 0 to 3,
                    // 4 means fully driven, so it should output NOP
                    // The color is encoded in the LSBs:
                    // count1[2:0], count0[2:0], color1, color0
                    // This allows easily know which pixel needs driving
                    // By XOR, we can know if we need to change direction
                    uint8_t incoming_pixels = *rdptr++;
                    for (int k = 0; k < 2; k++) {
                        // 2 outputs
                        uint8_t out = 0;
                        for (int l = 0; l < 2; l++) {
                            uint8_t state;
                            out <<= 4;
                            state = *stptr;
                            uint8_t driving_dir = incoming_pixels >> 6;
                            uint8_t pixel_diff = (state ^ driving_dir) & 0x3;
                            const uint8_t RESET_CNTR_MASK[4] = {
                                0xfc, // 00 all same
                                0xe0, // 01 lower one different
                                0x1c, // 10, higher one different
                                0x00, // 11, both different
                            };
                            state &= RESET_CNTR_MASK[pixel_diff];
                            state |= driving_dir;
                            // Due to the design, 0 in need_driving means it should be driven
                            // Handle output
                            out |= (state & 0x80) ? (0x0) : (driving_dir & 0x2) ? 0x4 : 0x8;
                            out |= (state & 0x10) ? (0x0) : (driving_dir & 0x1) ? 0x1 : 0x2;
                            // Update state 
                            uint8_t cntr_incr = ((~state) >> 2) & 0x24;
                            state = state + cntr_incr;
                            *stptr++ = state;
                            incoming_pixels <<= 2;
                        }
                        *wrptr++ = out;
                    }
                }
                if ((i % 3) == 2) {
                    // Flip buffer every 3 lines
                    dma_front_buffer = !dma_front_buffer;
                    // At the 3rd iteration, it has been rendered, start sending the new line
                }
                msg_send_row(dma_buf[dma_front_buffer]);
            }

            // By this time, the last line has only been sent out once. Repeat it two more times
            msg_send_row(dma_buf[dma_front_buffer]);
            msg_send_row(dma_buf[dma_front_buffer]);

            // Another 30 dummy lines
            memset(dma_buf[0], 0, EPD_WIDTH / 4 + EPD_LINE_PAD);
            for (int i = 0; i < 30; i++) {
                msg_send_row(dma_buf[0]);
            }
        }
        else {
            // Send dummy lines
            memset(dma_buf[0], 0, EPD_WIDTH / 4 + EPD_LINE_PAD);
            for (int i = 0; i < EPD_HEIGHT; i++) {
                msg_send_row(dma_buf[0]);
            }
        }

        // Send additional dummy line
        memset(dma_buf[0], 0, EPD_WIDTH / 4 + EPD_LINE_PAD);
        msg_send_row(dma_buf[0]);
#if defined(PAPERBOY_DEBUG) && PAPERBOY_DEBUG
        if (!dma_done) {
            int64_t _t = esp_timer_get_time();
            dma_wait();
            s_field_spin_us += (uint32_t)(esp_timer_get_time() - _t);
        }
        // Record this field's Core-1 load split (busy compute vs idle DMA spin).
        {
            uint32_t field_us = (uint32_t)(esp_timer_get_time() - frame_start_us);
            s_msg_field_us = field_us;
            s_msg_idle_us  = s_field_spin_us;
            s_msg_busy_us  = (field_us > s_field_spin_us)
                           ? (field_us - s_field_spin_us) : 0u;
        }
#else
        dma_wait();   // wait for the last row's DMA (bounded)
#endif

#ifdef TIME_PRINT
        uint32_t end = esp_timer_get_time();

        if (!printed) {
            ESP_LOGI(TAG, "Frame time: %lu us", end - start);
            last_print = end;
            printed = true;
        }
#endif

        audio_service_frame();

        // int64_t frame_elapsed_us = esp_timer_get_time() - frame_start_us;
        // if (FRAME_TARGET_US > 0 && frame_elapsed_us < FRAME_TARGET_US) {
        //     ets_delay_us((uint32_t)(FRAME_TARGET_US - frame_elapsed_us));
        // }
    }
}

// Public
void msg_init(void) {
    // Initialize IO
    gpio_config_t cfg = {
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };

    cfg.pin_bit_mask =
        (1ull << EPD_PWR_EN_PIN) |
        (1ull << EPD_BST_EN_PIN) |
        (1ull << EPD_SDCE_PIN) |
        (1ull << EPD_SDLE_PIN) |
        (1ull << EPD_SDCK_PIN) |
        (1ull << EPD_GDSP_PIN) |
        (1ull << EPD_GDCK_PIN);
    ESP_ERROR_CHECK(gpio_config(&cfg));

    // Initialize bus IO
    esp_lcd_i80_bus_config_t i80_bus_config = {
        .dc_gpio_num = (gpio_num_t)49, // dummy pin
        .wr_gpio_num = (gpio_num_t)EPD_SDCK_PIN,
        .clk_src = LCD_CLK_SRC_PLL160M,
        .data_gpio_nums = {
            (gpio_num_t)EPD_D0_PIN,
            (gpio_num_t)EPD_D1_PIN,
            (gpio_num_t)EPD_D2_PIN,
            (gpio_num_t)EPD_D3_PIN,
            (gpio_num_t)EPD_D4_PIN,
            (gpio_num_t)EPD_D5_PIN,
            (gpio_num_t)EPD_D6_PIN,
            (gpio_num_t)EPD_D7_PIN
        },
        .bus_width = EPD_BUSW,
        .max_transfer_bytes = 256, // Max 1024 pixels wide panel 
        .dma_burst_size = 32,
    };
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&i80_bus_config, &i80_bus_handle));

    esp_lcd_panel_io_i80_config_t panel_io_config = {
        .cs_gpio_num = (gpio_num_t)EPD_SDCE_PIN,
        .pclk_hz = EPD_XCK,
        .trans_queue_depth = 4,
        .on_color_trans_done = dma_done_callback,
        .user_ctx = NULL, // debug
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
        .flags = {
            .cs_active_high = 0,
            .reverse_color_bits = 0,
            .swap_color_bytes = 0,
            .pclk_active_neg = 0,
            .pclk_idle_low = 0,
        },
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus_handle, &panel_io_config,
        &panel_io_handle));
    
    // Allocate buffers
    for (int i = 0; i < 2; i++) {
        fb[i] = (uint8_t *)heap_caps_aligned_calloc(16, 1,
                EPD_VIDEO_FB_SIZE, MALLOC_CAP_INTERNAL);
    }
    statebuf = (uint8_t *)heap_caps_aligned_calloc(16, 1,
                EPD_VIDEO_WIDTH * EPD_VIDEO_HEIGHT / 2, MALLOC_CAP_INTERNAL);

    for (int i = 0; i < 2; i++) {
        dma_buf[i] = (uint8_t *)heap_caps_aligned_alloc(16,
                EPD_WIDTH / 4 + EPD_LINE_PAD + 16, MALLOC_CAP_DMA);
    }

    // Generate LUTs
    for (int i = 0; i < 16; i++) {
        push_lut_b[i] = 0;
        push_lut_w[i] = 0;
        for (int j = 0; j < 4; j++) {
            if ((i >> j) & 0x01) {
                push_lut_b[i] |= (0x1 << (j * 2));
                push_lut_w[i] |= (0x2 << (j * 2));
            }
        }
    }
}

void msg_get_load(uint32_t *busy_us, uint32_t *idle_us, uint32_t *field_us) {
#if defined(PAPERBOY_DEBUG) && PAPERBOY_DEBUG
    if (busy_us)  *busy_us  = s_msg_busy_us;
    if (idle_us)  *idle_us  = s_msg_idle_us;
    if (field_us) *field_us = s_msg_field_us;
#else
    if (busy_us)  *busy_us  = 0;
    if (idle_us)  *idle_us  = 0;
    if (field_us) *field_us = 0;
#endif
}

void msg_display_image(uint8_t *img, bool to_white) {
    img_src = img;
    img_to_white = to_white;
    img_req = true;
}

static TaskHandle_t s_msg_task = NULL;

void msg_start(void) {
    dma_done = true;
    enable_video = false;
    flip_waiter_task = NULL;
    msg_power_on();
    BaseType_t result = xTaskCreatePinnedToCore(msg_update_task, "msg", 8192,
            NULL, MSG_TASK_PRIORITY, &s_msg_task, 1);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task");
    }
}

/* Suspend / resume the e-paper field driver. Used to quiesce Core 1 around a
 * flash write so the flash driver doesn't repeatedly stall a busy, max-priority,
 * interrupt-disabling task. The e-paper is bistable, so the image persists while
 * paused. */
void msg_pause(void) {
    if (s_msg_task) vTaskSuspend(s_msg_task);
}
void msg_resume(void) {
    if (s_msg_task) { dma_done = true; vTaskResume(s_msg_task); }
}

void msg_enable_video(bool en) {
    enable_video = en;
}

uint8_t *msg_flip(void) {
    TaskHandle_t self = xTaskGetCurrentTaskHandle();

    // Drop stale notifications, if any, before waiting for the next VSYNC.
    (void)ulTaskNotifyTake(pdTRUE, 0);
    flip_waiter_task = self;
    flip_req = true;
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    return fb[!front_buffer];
}

uint8_t *msg_flip_nowait(void) {
    // Return the back-buffer immediately; the EPD keeps showing the current
    // front-buffer (last submitted frame).  No buffer swap is performed.
    return fb[!front_buffer];
}

uint32_t msg_get_vsync_count(void) {
    return s_vsync_count;
}
