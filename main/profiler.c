#include "profiler.h"

#include <string.h>
#include <inttypes.h>
#include <stdatomic.h>
#include "esp_log.h"
#include "esp_timer.h"

#include "paperboy_config.h"
#include "msg.h"

static const char *TAG = "prof";

/* Timestamp (µs) taken at profiler_begin(). */
static int64_t s_t0[PROF_PHASES];

/* Per-frame µs accumulators (reset each frame). */
static int64_t s_frame_us[PROF_PHASES];

/* Per-second µs accumulators and counters. */
static int64_t  s_sec_us[PROF_PHASES];
static uint32_t s_sec_frames;
static uint32_t s_sec_skipped;
static int64_t  s_sec_start_us; /* esp_timer_get_time() at start of interval */

/*
 * Audio task accumulator — written by the audio task, read+reset by the
 * emulator task in profiler_frame_end().  Uses an atomic uint32_t so that
 * the single-word load/store on Xtensa LX7 is naturally race-free.
 * Max value per second: ~60 calls × ~16 ms = ~960 000 µs < UINT32_MAX.
 */
static _Atomic uint32_t s_audio_sec_us = 0;

/* Worst-case (max) per-frame µs over the current interval, for the on-screen
 * HUD. Reset every interval alongside s_sec_us. */
static int64_t s_max_us[PROF_PHASES];
static int64_t s_max_cpu_us;
static int64_t s_max_dt_us;       /* longest full main-loop frame (→ min fps) */
static int64_t s_prev_end_us;     /* timestamp of the previous frame_end */

/* Latest published HUD snapshot (computed once per interval, read every frame
 * from the same task, so no locking needed). */
static profiler_hud_t s_hud;

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

void profiler_get_hud(profiler_hud_t *out)
{
    if (out) *out = s_hud;
}

void profiler_begin(int phase)
{
    s_t0[phase] = esp_timer_get_time();
}

void profiler_end(int phase)
{
    s_frame_us[phase] += esp_timer_get_time() - s_t0[phase];
}

void profiler_audio_add(int64_t elapsed_us)
{
    /* Saturate to uint32_t max to avoid wrap confusion. */
    uint32_t add = (elapsed_us > 0 && elapsed_us < 0xFFFFFFFF)
                   ? (uint32_t)elapsed_us : 0u;
    atomic_fetch_add_explicit(&s_audio_sec_us, add, memory_order_relaxed);
}

void profiler_frame_end(bool skipped)
{
    int64_t now_us = esp_timer_get_time();

    /* Worst-case tracking for the HUD — must read s_frame_us BEFORE it is
     * zeroed below. cpu is derived (frame - lcd - ppu); track its per-frame
     * max directly (max of the difference, not difference of the maxima). */
    int64_t cpu_us = s_frame_us[PROF_FRAME] - s_frame_us[PROF_LCD] - s_frame_us[PROF_PPU];
    if (cpu_us < 0) cpu_us = 0;
    for (int i = 0; i < PROF_PHASES; i++) {
        if (s_frame_us[i] > s_max_us[i]) s_max_us[i] = s_frame_us[i];
    }
    if (cpu_us > s_max_cpu_us) s_max_cpu_us = cpu_us;
    if (s_prev_end_us != 0) {
        int64_t dt = now_us - s_prev_end_us;   /* full main-loop frame time */
        if (dt > s_max_dt_us) s_max_dt_us = dt;
    }
    s_prev_end_us = now_us;

    /* Accumulate this frame's data into per-second buckets. */
    for (int i = 0; i < PROF_PHASES; i++) {
        s_sec_us[i] += s_frame_us[i];
    }
    memset(s_frame_us, 0, sizeof(s_frame_us));
    s_sec_frames++;
    if (skipped) {
        s_sec_skipped++;
    }

    /* Initialise the timer on the first call. */
    if (s_sec_start_us == 0) {
        s_sec_start_us = now_us;
        return;
    }

    /* Print once per second. */
    if ((now_us - s_sec_start_us) < 1000000) {
        return;
    }

    /* Convert accumulated µs → milliseconds per frame. */
    float inv_frames = (s_sec_frames > 0) ? (1.0f / (float)s_sec_frames) : 0.0f;

    float ms_frame = (float)s_sec_us[PROF_FRAME] * inv_frames * 0.001f;
    float ms_ppu   = (float)s_sec_us[PROF_PPU]   * inv_frames * 0.001f;
    float ms_lcd   = (float)s_sec_us[PROF_LCD]   * inv_frames * 0.001f;
    float ms_flip  = (float)s_sec_us[PROF_FLIP]  * inv_frames * 0.001f;
    float ms_touch = (float)s_sec_us[PROF_TOUCH] * inv_frames * 0.001f;
    float ms_cpu   = ms_frame - ms_lcd - ms_ppu; /* pure CPU stepping */

    /* Drain the audio accumulator atomically. */
    uint32_t audio_us = atomic_exchange_explicit(&s_audio_sec_us, 0u,
                                                 memory_order_relaxed);
    float ms_audio = (float)audio_us * inv_frames * 0.001f;

    float ms_total = ms_frame + ms_touch;

    /* Core-1 e-paper load for the most recent field (busy compute vs idle DMA
     * spin). idle is the per-field headroom available to offloaded work. */
    uint32_t c1_busy_us = 0, c1_idle_us = 0, c1_field_us = 0;
    msg_get_load(&c1_busy_us, &c1_idle_us, &c1_field_us);
    float ms_c1_busy = (float)c1_busy_us * 0.001f;
    float ms_c1_idle = (float)c1_idle_us * 0.001f;

    const float budget_ms = 17.7f;
    float budget_pct = 100.0f * ms_total / budget_ms;

    /* Publish the worst-case HUD snapshot for this interval. */
    float interval_us = (float)(now_us - s_sec_start_us);
    s_hud.valid    = true;
    s_hud.gen++;
    s_hud.fps_cur  = (interval_us > 0.0f) ? ((float)s_sec_frames * 1.0e6f / interval_us) : 0.0f;
    s_hud.fps_min  = (s_max_dt_us > 0)    ? (1.0e6f / (float)s_max_dt_us) : 0.0f;
    s_hud.frame_ms = (float)s_max_us[PROF_FRAME] * 0.001f;
    s_hud.cpu_ms   = (float)s_max_cpu_us         * 0.001f;
    s_hud.ppu_ms   = (float)s_max_us[PROF_PPU]   * 0.001f;
    s_hud.lcd_ms   = (float)s_max_us[PROF_LCD]   * 0.001f;
    s_hud.flip_ms  = (float)s_max_us[PROF_FLIP]  * 0.001f;
    s_hud.touch_ms = (float)s_max_us[PROF_TOUCH] * 0.001f;
    s_hud.audio_ms = ms_audio;       /* average over the interval */
    s_hud.msg_busy_ms = ms_c1_busy;
    s_hud.msg_idle_ms = ms_c1_idle;
    s_hud.frames   = s_sec_frames;
    s_hud.skipped  = s_sec_skipped;

    ESP_LOGI(TAG, "%" PRIu32 " fr (%" PRIu32 " skip) "
             "| cpu %4.1fms  ppu %4.1fms  lcd %4.1fms  flip %4.1fms  touch %4.1fms  audio %4.1fms "
             "| c1 busy %4.1f idle %4.1f (field %4.1f) "
             "| total %4.1fms / %.1fms budget (%4.1f%%)",
             s_sec_frames, s_sec_skipped,
             ms_cpu, ms_ppu, ms_lcd, ms_flip, ms_touch, ms_audio,
             ms_c1_busy, ms_c1_idle, (float)c1_field_us * 0.001f,
             ms_total, budget_ms, budget_pct);

    /* Reset for next interval. */
    memset(s_sec_us, 0, sizeof(s_sec_us));
    memset(s_max_us, 0, sizeof(s_max_us));
    s_max_cpu_us  = 0;
    s_max_dt_us   = 0;
    s_sec_frames  = 0;
    s_sec_skipped = 0;
    s_sec_start_us = now_us;
}
