# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Release-build defaults for this board. Each is a CMake variable so a
# user can still override at configure time:
#   idf.py -DGBJIT_HALT_STEP_CYCLES=4 build
# These are the values that benchmark/results/board_option_sweep.md
# documents as the steady-state-best for SML on the Heltec V3 OLED:
#   sync PPU (correct BG)
#   1024-cycle halt step (large halt-loop strides on HALT-heavy ROMs)
#   crop the per-line render to the OLED's centre 128×64 window
#   OLED I²C blits at native bandwidth (no artificial cap)
#   chain-batching 4 blocks per outer iter (default already)
#
# QEMU virtual-LCD builds (-DGBJIT_QEMU_LCD=1) override the crop
# defaults to render the full 160×144 GB frame — the host viewer
# shows the whole screen, not just the OLED's centre window.
# GBJIT_PPU_ASYNC is intentionally NOT defaulted or propagated here.
# The source uses `#ifdef GBJIT_PPU_ASYNC` (definedness, not value), so
# exporting `GBJIT_PPU_ASYNC=0` project-wide would still select the
# async path. The gbjit component's own CMakeLists handles it correctly
# — it only defines the macro (as =1) when the value is truthy, so the
# default and an explicit `-DGBJIT_PPU_ASYNC=0` both mean synchronous.
if(NOT DEFINED GBJIT_HALT_STEP_CYCLES)
    set(GBJIT_HALT_STEP_CYCLES 1024)
endif()
# JIT codecache arena. The 64 KB core default starves larger games:
# an on-board arena sweep with Tetris showed a hard cliff — 46 fps at
# <=88 KB (working set spills the bump allocator; the overflow blocks
# fall back to the interpreter permanently) jumping to ~145 fps at
# 96 KB once the whole working set is JIT-resident. 96 KB is the knee.
if(NOT DEFINED GBJIT_ARENA_KB)
    # 128KB is the largest IRAM arena that still leaves room for the e-ink
    # statebuf (~67KB internal) after WiFi is disabled + cold .bss is parked in
    # PSRAM (EXT_RAM_BSS). At 128KB SML's whole working set is resident, so the
    # PSRAM block-byte-cache rehydration thrash is essentially gone
    # (~17.6k -> ~200 rehydrations; action cpu ~77ms -> ~12ms; min_1s 12.7 ->
    # 22.8 fps). 160KB overflows internal SRAM and crashes the e-ink task.
    set(GBJIT_ARENA_KB 128)
endif()
# Full-frame render for the virtual LCD.
if(NOT DEFINED GBJIT_PPU_DRAW_MIN_LY)
    set(GBJIT_PPU_DRAW_MIN_LY 0)
endif()
if(NOT DEFINED GBJIT_PPU_DRAW_MAX_LY)
    set(GBJIT_PPU_DRAW_MAX_LY 144)
endif()
if(NOT DEFINED GBJIT_PPU_DRAW_MIN_X)
    set(GBJIT_PPU_DRAW_MIN_X 0)
endif()
if(NOT DEFINED GBJIT_PPU_DRAW_MAX_X)
    set(GBJIT_PPU_DRAW_MAX_X 160)
endif()
if(NOT DEFINED GBJIT_JIT_EVICT)
    set(GBJIT_JIT_EVICT 1)
endif()
# Skip the per-block sm83_service_interrupts/ppu_tick call when nothing is due
# (inlined early-return predicate). Behavior-preserving; parity-verified.
if(NOT DEFINED GBJIT_DISPATCHER_SERVICE_SHORTCUT)
    set(GBJIT_DISPATCHER_SERVICE_SHORTCUT 1)
endif()
# PPU render offload to a second core. DEFAULT OFF on this board: the
# M5Paper's e-paper panel needs continuous field-sequential driving, so the
# "msg" task (main/msg/msg.c) runs a never-yielding refresh loop at
# configMAX_PRIORITIES-1 pinned to Core 1. Core 1 is therefore 100% committed
# and a render task at any lower priority is starved (the render never
# completes and the emulation core deadlocks in ppu_offload_render_wait).
# The offload implementation is host-verified bit-exact and kept gated for a
# board that DOES have a spare core; enable with -DGBJIT_PPU_OFFLOAD_THREAD=1
# only on such hardware. See [[paperboy-jit-perf]].
if(NOT DEFINED GBJIT_PPU_OFFLOAD_THREAD)
    set(GBJIT_PPU_OFFLOAD_THREAD 0)
endif()

# Propagate JIT/PPU compile-time toggles project-wide. Flags that
# affect the mmu struct layout (e.g. GBJIT_FRAMEBUFFER_DOUBLE_BUFFER)
# MUST be visible to every translation unit that includes
# core/memory.h — gbjit, main, anything else — or different TUs see
# different `mmu` sizes and crash on first cross-component access.
# Pass any subset on the command line, e.g.
#   idf.py -DGBJIT_FRAMEBUFFER_DOUBLE_BUFFER=1 -DGBJIT_CHAIN_PREDICTOR_WAYS=1 build
foreach(_opt
        GBJIT_CHAIN_PREDICTOR_WAYS
        GBJIT_INLINE_JP_HL
        GBJIT_INLINE_ADD_HL_RR
        GBJIT_DISPATCHER_HALT_INNER_LOOP
        GBJIT_HALT_STEP_CYCLES
        GBJIT_FRAMEBUFFER_DOUBLE_BUFFER
        GBJIT_PPU_FAST_BG_RENDERER
        GBJIT_PPU_SKIP_DRAW
        GBJIT_PPU_DRAW_MIN_LY
        GBJIT_PPU_DRAW_MAX_LY
        GBJIT_PPU_DRAW_MIN_X
        GBJIT_PPU_DRAW_MAX_X
        GBJIT_ARENA_KB
        GBJIT_JIT_EVICT
        GBJIT_PPU_OFFLOAD
        GBJIT_PPU_OFFLOAD_THREAD
        GBJIT_HTIT_OLED_ENABLE
        GBJIT_HTIT_OLED_MIN_INTERVAL_MS
        GBJIT_DISPATCHER_SERVICE_SHORTCUT
        GBJIT_DISPATCHER_CHAIN_BATCH
        GBJIT_DISPATCHER_SELFLOOP
        GBJIT_SELFLOOP_MAX
        GBJIT_IO_POLL_HALT)
    if(DEFINED ${_opt})
        idf_build_set_property(COMPILE_DEFINITIONS
            "${_opt}=${${_opt}}" APPEND)
    endif()
endforeach()

project(paperboy)
