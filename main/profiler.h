#pragma once

/*
 * Lightweight frame-time profiler for the Paperboy emulation loop.
 *
 * Usage
 * -----
 *   PROF_BEGIN(PROF_FLIP);
 *   msg_flip();
 *   PROF_END(PROF_FLIP);
 *
 *   // At the end of every emulated frame (rendered or skipped):
 *   profiler_frame_end(skip_render);
 *
 * Phases
 * ------
 *   PROF_FRAME  – total paperboy_gb_run_frame() wall time (cpu + lcd callbacks)
 *   PROF_LCD    – lcd_draw_line_cb() accumulated over all 144 lines in a frame
 *   PROF_FLIP   – msg_flip() / msg_flip_nowait() wait time
 *   PROF_TOUCH  – tp_read_buttons() time
 *
 * Audio task (higher-priority, same core): call profiler_audio_add() with the
 * elapsed µs for each synthesis + ring-fill iteration.  Uses a separate
 * atomic accumulator so no locking is needed between the two tasks.
 *
 * The "pure CPU emulation" time is derived as FRAME − LCD and shown in the
 * report.  Timing uses esp_timer_get_time() (µs resolution, ~1–2 µs call
 * overhead).
 *
 * Output (once per second via ESP_LOGI, tag "prof"):
 *   42 fr (8 skip) | cpu 9.80ms  lcd 3.12ms  flip 2.40ms  touch 0.08ms  audio 1.20ms
 *                  | total 16.60ms / 16.67ms budget (99.6%)
 */

#include <stdbool.h>
#include <stdint.h>

#define PROF_FRAME  0
#define PROF_PPU    1
#define PROF_LCD    2
#define PROF_FLIP   3
#define PROF_TOUCH  4
#define PROF_PHASES 5

/* Record the start of a measurement for the given phase. */
void profiler_begin(int phase);

/* Accumulate elapsed cycles for the given phase since profiler_begin(). */
void profiler_end(int phase);

/*
 * Call once per emulated frame (after PROF_FRAME has been ended).
 * Accumulates per-frame data; prints a summary after ~1 second of frames.
 * skipped: true when the frame's LCD rendering was suppressed (frame-skip).
 */
void profiler_frame_end(bool skipped);

/*
 * Called from the audio task to accumulate audio synthesis CPU time.
 * Thread-safe: uses an atomic add internally.
 * elapsed_us: wall-clock µs spent in synthesis + ring-fill (not DMA wait).
 */
void profiler_audio_add(int64_t elapsed_us);

/*
 * Live HUD snapshot, refreshed once per second inside profiler_frame_end().
 * All *_ms fields are the WORST (max) single-frame value seen over the last
 * interval (except audio_ms, which is the interval average — audio is timed
 * cross-core and accumulated per second). Times are milliseconds.
 */
typedef struct {
    bool     valid;     /* false until the first interval has elapsed */
    uint32_t gen;       /* bumps each publish (~1 Hz); for change detection */
    float    fps_cur;   /* average fps over the interval */
    float    fps_min;   /* worst single-frame fps (1e6 / longest frame) */
    float    frame_ms;  /* worst full paperboy_gb_run_frame() ms */
    float    cpu_ms;    /* worst pure-CPU ms (frame - lcd - ppu) */
    float    ppu_ms;    /* worst PPU ms */
    float    lcd_ms;    /* worst LCD/blit ms */
    float    flip_ms;   /* worst flip / VSYNC-wait ms */
    float    touch_ms;  /* worst touch ms */
    float    audio_ms;  /* average audio (APU synth) ms */
    uint32_t frames;    /* frames in the interval */
    uint32_t skipped;   /* skipped frames in the interval */
} profiler_hud_t;

/* Copy the latest published HUD snapshot. Cheap; call every frame. */
void profiler_get_hud(profiler_hud_t *out);

/* Convenience macros */
#define PROF_BEGIN(phase)  profiler_begin(phase)
#define PROF_END(phase)    profiler_end(phase)
