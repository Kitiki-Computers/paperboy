/* On-device UART debug console — see debug_console.h.
 *
 * Transport: USB-Serial-JTAG (the native-USB CDC, /dev/ttyACM0). The driver
 * is installed and stdio is routed through it so both blocking RX and printf
 * TX work. Boot logs before dbg_console_init() use the default console path.
 *
 * Line protocol (host -> device), one command per '\n'-terminated line:
 *   b <hex>        set held button mask (GB_BTN_* layout) until changed
 *   t <hex> <n>    tap: hold mask for n frames, then release
 *   f              dump the next frame's GB framebuffer
 *   s              print frame-time / FPS stats
 *   z              reset stats (worst-case trackers)
 *
 * Device -> host framebuffer dump (markers let the host skip interleaved logs):
 *   <<FB 160 144>>
 *   <11520 hex chars = 5760 bytes, 4 px/byte, MSB-first pairs, shade 0..3>
 *   <<FBEND>>
 */
#if defined(PAPERBOY_DEBUG_CONSOLE) && PAPERBOY_DEBUG_CONSOLE

#include "debug_console.h"
#include "gbemu.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"

#define GB_W 160
#define GB_H 144

static volatile uint8_t  s_btn = 0;
static volatile int      s_tap_frames = 0;
static volatile bool     s_dump_req = false;
static volatile bool     s_stats_req = false;
static volatile bool     s_reset_req = false;

/* Frame-time stats (updated in dbg_console_after_frame, main-loop context). */
static int64_t  s_last_us = 0;
static uint32_t s_frames = 0;
static uint32_t s_worst_us = 0;     /* slowest single frame -> worst-case fps */
static uint64_t s_sum_us = 0;
static int64_t  s_window_start_us = 0;
static uint32_t s_window_frames = 0;
static double   s_min_window_fps = 1e9;

static char s_hex[GB_W * GB_H / 4 * 2 + 1];   /* 11520 + NUL */

static void dump_fb(const uint8_t *fb);   /* fwd decl */

static void parse_line(char *line) {
    /* strip CR/LF */
    char *p = line;
    while (*p) { if (*p == '\r' || *p == '\n') { *p = 0; break; } p++; }
    if (line[0] == 0) return;
    char cmd = line[0];
    const char *arg = line + 1;
    while (*arg == ' ') arg++;
    if (cmd == 'b') {
        s_btn = (uint8_t)strtoul(arg, NULL, 16);
        s_tap_frames = 0;
    } else if (cmd == 't') {
        char *end = NULL;
        unsigned mask = strtoul(arg, &end, 16);
        int n = (end && *end) ? atoi(end) : 8;
        if (n <= 0) n = 8;
        s_btn = (uint8_t)mask;
        s_tap_frames = n;
    } else if (cmd == 'f') {
        /* Dump directly from the RX task so it works even when the render
         * loop is stalled/thrashing (the very state we want to inspect). */
        dump_fb(paperboy_gb_debug_lcd());
    } else if (cmd == 's') {
        s_stats_req = true;
    } else if (cmd == 'z') {
        s_reset_req = true;
    } else if (cmd == 'v') {
        /* Report the panel field (VSYNC) counter + wall clock so the host can
         * compute the e-paper refresh/field rate independent of emulation fps. */
        extern uint32_t msg_get_vsync_count(void);
        printf("[vsync] count=%lu us=%lld\n",
               (unsigned long)msg_get_vsync_count(),
               (long long)esp_timer_get_time());
        fflush(stdout);
    } else if (cmd == 'j') {
        /* Report JIT counters (need DEBUG=1 build) + cycles, to compute
         * blocks-compiled/sec etc. and detect recompilation thrashing. */
        paperboy_gb_stats_t st;
        if (paperboy_gb_get_stats(&st)) {
            printf("[jit] cyc=%llu blkcomp=%llu bbchit=%llu bbcmiss=%llu blkexec=%llu chmiss=%llu smcinv=%llu arst=%llu bankflip=%llu smcflush=%llu interp=%llu arena=%lu/%lu us=%lld\n",
                   (unsigned long long)st.emulated_cycles,
                   (unsigned long long)st.blocks_compiled,
                   (unsigned long long)st.bbc_hits,
                   (unsigned long long)st.bbc_misses,
                   (unsigned long long)st.blocks_executed,
                   (unsigned long long)st.chain_misses,
                   (unsigned long long)st.smc_invalidations,
                   (unsigned long long)st.arena_resets,
                   (unsigned long long)st.bank_flips,
                   (unsigned long long)st.smc_flushes,
                   (unsigned long long)st.interp_steps,
                   (unsigned long)st.arena_used,
                   (unsigned long)st.arena_bytes,
                   (long long)esp_timer_get_time());
            fflush(stdout);
        }
    }
}

static void rx_task(void *arg) {
    (void)arg;
    char line[64]; int li = 0;
    for (;;) {
        uint8_t c;
        int n = usb_serial_jtag_read_bytes(&c, 1, portMAX_DELAY);
        if (n <= 0) continue;
        if (c == '\n' || c == '\r') {
            line[li] = 0;
            if (li > 0) parse_line(line);
            li = 0;
        } else if (li < (int)sizeof(line) - 1) {
            line[li++] = (char)c;
        }
    }
}

void dbg_console_init(void) {
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.tx_buffer_size = 4096;
    cfg.rx_buffer_size = 1024;
    if (usb_serial_jtag_driver_install(&cfg) == ESP_OK) {
        usb_serial_jtag_vfs_use_driver();
    }
    s_last_us = esp_timer_get_time();
    s_window_start_us = s_last_us;
    xTaskCreatePinnedToCore(rx_task, "dbg_rx", 3072, NULL,
                            tskIDLE_PRIORITY + 1, NULL, 0 /* Core 0 */);
    printf("\n[dbg] console ready: b/t/f/s/z\n");
}

uint8_t dbg_console_button_mask(void) { return s_btn; }

static void dump_fb(const uint8_t *fb) {
    static const char hexd[] = "0123456789ABCDEF";
    int o = 0;
    /* pack 4 pixels/byte: bits [7:6]=px0 .. [1:0]=px3, then hex it */
    for (int i = 0; i < GB_W * GB_H; i += 4) {
        uint8_t b = (uint8_t)(((fb[i] & 3) << 6) | ((fb[i+1] & 3) << 4) |
                              ((fb[i+2] & 3) << 2) | (fb[i+3] & 3));
        s_hex[o++] = hexd[b >> 4];
        s_hex[o++] = hexd[b & 0xF];
    }
    s_hex[o] = 0;
    printf("<<FB %d %d>>\n", GB_W, GB_H);
    fwrite(s_hex, 1, (size_t)o, stdout);
    printf("\n<<FBEND>>\n");
    fflush(stdout);
}

void dbg_console_after_frame(const uint8_t *gb_fb) {
#if defined(PAPERBOY_AUTOTEST) && PAPERBOY_AUTOTEST
    /* Fully-scripted, output-only self test (no flaky console RX): once the
     * title is up, press Start, then dump the framebuffer well after the level
     * has loaded+settled. Verifies the SML Start->level transition on-device. */
    static uint32_t af = 0; af++;
    if (af == 150) { s_btn = GB_BTN_START; printf("[autotest] press Start @frame %lu\n", (unsigned long)af); }
    else if (af == 165) { s_btn = 0; }
    else if (af == 600) { printf("[autotest] capture @frame %lu\n", (unsigned long)af); dump_fb(gb_fb); }
#endif
    int64_t now = esp_timer_get_time();
    uint32_t dt = (uint32_t)(now - s_last_us);
    s_last_us = now;

    if (s_reset_req) {
        s_worst_us = 0; s_sum_us = 0; s_frames = 0;
        s_window_start_us = now; s_window_frames = 0; s_min_window_fps = 1e9;
        s_reset_req = false;
        dt = 0;
    }
    if (dt > 0) {
        s_frames++; s_sum_us += dt;
        if (dt > s_worst_us) s_worst_us = dt;
        s_window_frames++;
        if (now - s_window_start_us >= 1000000) {
            double fps = s_window_frames * 1e6 / (double)(now - s_window_start_us);
            if (fps < s_min_window_fps) s_min_window_fps = fps;
            s_window_start_us = now; s_window_frames = 0;
        }
    }

    /* tap auto-release */
    if (s_tap_frames > 0) {
        if (--s_tap_frames == 0) s_btn = 0;
    }

    if (s_stats_req) {
        double avg_fps = s_frames ? (s_frames * 1e6 / (double)s_sum_us) : 0.0;
        double worst_fps = s_worst_us ? (1e6 / (double)s_worst_us) : 0.0;
        printf("[stats] frames=%lu avg_fps=%.1f worst_frame=%lu us (worst_inst_fps=%.1f) min_1s_fps=%.1f\n",
               (unsigned long)s_frames, avg_fps, (unsigned long)s_worst_us, worst_fps,
               (s_min_window_fps < 1e8) ? s_min_window_fps : 0.0);
        fflush(stdout);
        s_stats_req = false;
    }
    if (s_dump_req) {
        if (gb_fb) dump_fb(gb_fb);
        else printf("<<FB ERR no-fb>>\n");
        s_dump_req = false;
    }
}

#endif /* PAPERBOY_DEBUG_CONSOLE */
