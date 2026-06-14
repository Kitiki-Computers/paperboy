#include <stdlib.h>
#include <string.h>

#include "esp_log.h"

#include "audio.h"
#include "profiler.h"

#include "gbemu_internal.h"

#include "cpu_state.h"
#include "dispatcher.h"
#include "memory.h"
#include "ppu.h"
#include "ppu_thread.h"

static const char *TAG = "gbjit_pb";

static cpu_state s_cpu;
static mmu s_mmu;
static gbjit_dispatcher s_dispatcher;
static bool s_dispatcher_ready;
static bool s_ready;
static char s_last_error[128];

#define GB_FRAME_CYCLES 70224u

static void set_last_error(const char *msg)
{
    strlcpy(s_last_error, msg, sizeof(s_last_error));
}

static bool gbjit_is_rom_supported(const uint8_t *rom, size_t rom_size)
{
    uint8_t cart_type;
    uint8_t cgb_flag;

    if (rom == NULL || rom_size < 0x150 || rom_size > ROM_SIZE_MAX) {
        return false;
    }

    cart_type = rom[0x147];
    cgb_flag = rom[0x143];

    if (cgb_flag == 0xC0) {
        return false;
    }

    switch (cart_type) {
    case 0x00:
    case 0x01:
    case 0x02:
    case 0x03:
        return true;
    default:
        return false;
    }
}

static uint8_t io_read_cb(void *ctx, uint16_t addr, uint8_t value)
{
    (void)ctx;

    if (addr >= 0xFF10u && addr <= 0xFF3Fu) {
        return audio_apu_read(addr);
    }

    return value;
}

static void io_write_cb(void *ctx, uint16_t addr, uint8_t value)
{
    (void)ctx;

    if (addr >= 0xFF10u && addr <= 0xFF3Fu) {
        audio_apu_write(addr, value);
    }
}

static bool gbjit_init_backend(const uint8_t *rom, size_t rom_size)
{
    if (!gbjit_is_rom_supported(rom, rom_size)) {
        set_last_error("gbjit only supports DMG ROM/MBC1 carts up to 256KB");
        return false;
    }

    ppu_offload_thread_stop();
    ppu_thread_stop();
    if (s_dispatcher_ready) {
        gbjit_dispatcher_shutdown(&s_dispatcher);
        s_dispatcher_ready = false;
    }
    mmu_destroy(&s_mmu);

    memset(&s_cpu, 0, sizeof(s_cpu));
    memset(&s_mmu, 0, sizeof(s_mmu));
    memset(&s_dispatcher, 0, sizeof(s_dispatcher));

    gb_mmu_init(&s_mmu);
    if (!mmu_load_rom(&s_mmu, rom, rom_size)) {
        set_last_error("mmu_load_rom failed");
        return false;
    }

    s_mmu.io_read_cb = io_read_cb;
    s_mmu.io_write_cb = io_write_cb;
    s_mmu.io_cb_ctx = NULL;
    mmu_set_joypad_state(&s_mmu, 0);

    cpu_reset(&s_cpu, &s_mmu);
    audio_init();

    if (!gbjit_dispatcher_init(&s_dispatcher, &s_cpu)) {
        audio_deinit();
        set_last_error("gbjit_dispatcher_init failed");
        return false;
    }
    s_dispatcher_ready = true;

    /* Match the known-good HTIT-WB32LAF_V3.2 board build: disable the
     * static-successor prefetcher. gbjit_dispatcher_init() defaults it ON,
     * but the HTIT app (boards/.../main/app_main.c) explicitly turns it OFF.
     * With it on, the dispatcher eagerly compiles fall-through successor
     * blocks; on the SML title->level transition that's a burst of
     * compiles that (a) spikes one frame to ~190 ms (visible freeze) and
     * (b) churns the code cache enough that native execution mis-branches
     * (jumps into the bonus-game handler / locks up). The lazy chain-miss
     * path still compiles blocks on demand. The host Xtensa simulator
     * tolerates the prefetch path, so this only shows on real hardware. */
    s_dispatcher.prefetch_enabled = false;

#if defined(PAPERBOY_GB_FORCE_INTERP) && PAPERBOY_GB_FORCE_INTERP
    /* Debug toggle: run every step through the reference interpreter
     * (sm83_step) instead of compiling/executing native JIT blocks. Same
     * dispatcher loop, same PPU/timer ticking — isolates JIT-codegen /
     * native-execution bugs (e.g. instruction-cache staleness) from the
     * shared core. Enabled via -DPAPERBOY_GB_FORCE_INTERP=1. */
    s_dispatcher.interp_fallback = true;
    ESP_LOGW(TAG, "gbjit: FORCE_INTERP enabled (no native JIT)");
#endif

    ppu_thread_start(&s_cpu);
    ppu_offload_thread_start();   /* no-op unless GBJIT_PPU_OFFLOAD_THREAD */

    s_ready = true;
    set_last_error("ok");
    ESP_LOGI(TAG, "gbjit initialized");
    return true;
}

static void gbjit_set_buttons(uint8_t pressed_mask)
{
    if (!s_ready) {
        return;
    }

    mmu_set_joypad_state(&s_mmu, pressed_mask);
}

const uint8_t *paperboy_gb_debug_lcd(void)
{
    return (s_ready && s_dispatcher_ready) ? s_mmu.framebuffer : NULL;
}

static bool gbjit_run_frame(uint8_t *fb, bool skip_render)
{
    uint32_t start_frame;

    if (!s_ready || !s_dispatcher_ready) {
        set_last_error("gbjit not initialized");
        return false;
    }

#if GBJIT_PPU_OFFLOAD_THREAD
    /* Pipelined: the render of the *previous* frame was dispatched to the
     * second core at its VBlank and ran concurrently with this thread's
     * idle time (the e-paper VSYNC wait). Block until it's complete, then
     * blit it. This surfaces frames with one frame of latency, but keeps
     * the ~11.5 ms scanline render off this core's critical path. */
    ppu_offload_render_wait();
    if (!skip_render && fb != NULL) {
        PROF_BEGIN(PROF_LCD);
        for (uint_fast8_t line = 0; line < GB_LCD_HEIGHT; line++) {
            const uint8_t *src = &s_mmu.framebuffer[(size_t)line * GB_LCD_WIDTH];
            paperboy_gb_blit_line(fb, src, line);
        }
        PROF_END(PROF_LCD);
    }
#endif

    start_frame = s_mmu.frame_seq;
    do {
        gbjit_dispatcher_run_until(&s_dispatcher, s_cpu.cycles + GB_FRAME_CYCLES);
        if (s_cpu.stopped) {
            set_last_error("cpu stopped");
            return false;
        }
    } while (s_ready && s_mmu.frame_seq == start_frame);

#if !GBJIT_PPU_OFFLOAD_THREAD
    /* Synchronous render: the scanlines were drawn during emulation (at each
     * mode3->0, or batched at VBlank under GBJIT_PPU_OFFLOAD), so the GB
     * framebuffer holds this frame now. */
    if (!skip_render && fb != NULL) {
        PROF_BEGIN(PROF_LCD);
        for (uint_fast8_t line = 0; line < GB_LCD_HEIGHT; line++) {
            const uint8_t *src = &s_mmu.framebuffer[(size_t)line * GB_LCD_WIDTH];
            paperboy_gb_blit_line(fb, src, line);
        }
        PROF_END(PROF_LCD);
    }
#endif

    return true;
}

static bool gbjit_has_persist(void)
{
    return false;
}

static size_t gbjit_persist_size(void)
{
    return 0;
}

static bool gbjit_persist_is_dirty(void)
{
    return false;
}

static bool gbjit_persist_export(uint8_t *dst, size_t dst_size, uint32_t timestamp)
{
    (void)dst;
    (void)dst_size;
    (void)timestamp;
    set_last_error("persist not implemented for gbjit");
    return false;
}

static bool gbjit_persist_import(const uint8_t *src, size_t src_size, uint32_t current_timestamp)
{
    (void)src;
    (void)src_size;
    (void)current_timestamp;
    set_last_error("persist not implemented for gbjit");
    return false;
}

static void gbjit_persist_mark_clean(void)
{
}

static size_t gbjit_state_size(void)
{
    return 0;
}

static bool gbjit_state_export(uint8_t *dst, size_t dst_size)
{
    (void)dst;
    (void)dst_size;
    set_last_error("save states not implemented for gbjit");
    return false;
}

static bool gbjit_state_import(const uint8_t *src, size_t src_size)
{
    (void)src;
    (void)src_size;
    set_last_error("save states not implemented for gbjit");
    return false;
}

static bool gbjit_is_ready(void)
{
    return s_ready;
}

static const char *gbjit_last_error(void)
{
    return s_last_error;
}

static bool gbjit_get_stats(paperboy_gb_stats_t *stats)
{
    if (stats == NULL) {
        return false;
    }

    memset(stats, 0, sizeof(*stats));
    stats->has_emulated_cycles = true;
    stats->has_jit_counters = true;
    stats->emulated_cycles = s_cpu.cycles;
    stats->frame_count = s_mmu.frame_seq;
    stats->blocks_compiled = s_dispatcher.blocks_compiled;
    stats->blocks_executed = s_dispatcher.blocks_executed;
    stats->chain_hits = s_dispatcher.chain_hits;
    stats->chain_misses = s_dispatcher.chain_misses;
    stats->smc_invalidations = s_dispatcher.smc_invalidations;
    stats->arena_resets = s_dispatcher.arena_resets;
    stats->bank_flips = s_dispatcher.bank_flips;
    stats->smc_flushes = s_dispatcher.smc_flushes;
    stats->bbc_hits = s_dispatcher.bbc_hits;
    stats->bbc_misses = s_dispatcher.bbc_misses;
    stats->interp_steps = s_dispatcher.interp_steps;
    stats->arena_used = s_dispatcher.cc.used;
    stats->arena_bytes = s_dispatcher.cc.cap;
    return true;
}

const paperboy_gb_backend_ops_t *paperboy_gb_gbjit_backend(void)
{
    static const paperboy_gb_backend_ops_t ops = {
        .name = "gbjit",
        .is_rom_supported = gbjit_is_rom_supported,
        .init = gbjit_init_backend,
        .set_buttons = gbjit_set_buttons,
        .run_frame = gbjit_run_frame,
        .has_persist = gbjit_has_persist,
        .persist_size = gbjit_persist_size,
        .persist_is_dirty = gbjit_persist_is_dirty,
        .persist_export = gbjit_persist_export,
        .persist_import = gbjit_persist_import,
        .persist_mark_clean = gbjit_persist_mark_clean,
        .state_size = gbjit_state_size,
        .state_export = gbjit_state_export,
        .state_import = gbjit_state_import,
        .is_ready = gbjit_is_ready,
        .last_error = gbjit_last_error,
        .get_stats = gbjit_get_stats,
    };

    return &ops;
}
