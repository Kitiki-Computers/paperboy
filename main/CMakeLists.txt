set(PAPERBOY_AUDIO_PCM_IMPL "audio_pdm.c" CACHE STRING
    "PCM audio backend source file (audio_pdm.c or audio_pwm.c)")

idf_component_register(
    SRCS
        "main.c"
        "ui.c"
        "msg/msg.c"
        "gbemu.c"
        "gbemu_api.c"
        "gbemu_jit.c"
        "audio.c"
        "${PAPERBOY_AUDIO_PCM_IMPL}"
        "audio_poly_pwm.c"
        "minigb_apu/minigb_apu.c"
        "touch.c"
        "profiler.c"
        "battery.c"
        "debug_console.c"
        "lockstep_diff.c"
        "gbjit-xtensa/core/memory.c"
        "gbjit-xtensa/core/ppu.c"
        "gbjit-xtensa/core/ppu_thread.c"
        "gbjit-xtensa/core/sm83_decoder.c"
        "gbjit-xtensa/core/sm83_interp.c"
        "gbjit-xtensa/jit/emit_xtensa.c"
        "gbjit-xtensa/jit/codecache.c"
        "gbjit-xtensa/jit/codegen.c"
        "gbjit-xtensa/jit/dispatcher.c"
        "gbjit-xtensa/port/esp32s3/components/gbjit/jit_trampolines.S"
    INCLUDE_DIRS "." "msg" "minigb_apu" "gbjit-xtensa/include" "gbjit-xtensa/core" "gbjit-xtensa/jit"
    REQUIRES driver esp_driver_sdspi esp_driver_sdmmc fatfs esp_timer esp_lcd esp_driver_ledc esp_driver_i2s esp_driver_i2c esp_adc esp_driver_usb_serial_jtag freertos heap
)

# Audio APU format: 16-bit signed integer
target_compile_definitions(${COMPONENT_LIB} PRIVATE MINIGB_APU_AUDIO_FORMAT_S16SYS)

# The newer xtensa-esp-elf gcc 15 toolchain promotes several benign warnings
# (e.g. -Wtype-limits on config-gated `unsigned >= 0` comparisons) that ESP-IDF
# turns into errors via -Werror. This predates the toolchain bump; keep these as
# plain warnings for the project's own component so the firmware still builds.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error)

# Optional on-device autoboot: embed a ROM and boot it directly (no SD card /
# no touch picker). Opt-in only — normal builds are unaffected.
#   idf.py -DPAPERBOY_AUTOBOOT_ROM=autoboot_rom.gb build
# The file (relative to this dir) is embedded as _binary_<name>_gb_start/_end.
if(DEFINED PAPERBOY_AUTOBOOT_ROM)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE PAPERBOY_AUTOBOOT_EMBEDDED=1)
    target_add_binary_data(${COMPONENT_LIB} "${PAPERBOY_AUTOBOOT_ROM}" BINARY)
endif()

# Opt-in debug toggles (idf.py -D<NAME>=1 build):
#   PAPERBOY_DEBUG_CONSOLE  USB-Serial-JTAG console: inject input, dump GB
#                           framebuffer, report FPS/worst-case timing.
#   PAPERBOY_GB_FORCE_INTERP  run the reference interpreter instead of native
#                           JIT (isolates JIT-codegen / icache-staleness bugs).
if(PAPERBOY_DEBUG_CONSOLE)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE PAPERBOY_DEBUG_CONSOLE=1)
endif()
if(PAPERBOY_GB_FORCE_INTERP)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE PAPERBOY_GB_FORCE_INTERP=1)
endif()
if(PAPERBOY_LOCKSTEP_DIFF)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE PAPERBOY_LOCKSTEP_DIFF=1)
endif()
if(PAPERBOY_AUTOTEST)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE PAPERBOY_AUTOTEST=1)
endif()
# Diagnostic toggles for the device-only JIT dispatcher fast paths (root-causing
# the SML chain-batch interrupt-deferral bug).
if(DEFINED PAPERBOY_CHAIN_BATCH)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE GBJIT_DISPATCHER_CHAIN_BATCH=${PAPERBOY_CHAIN_BATCH})
endif()
if(DEFINED PAPERBOY_SELFLOOP)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE GBJIT_DISPATCHER_SELFLOOP=${PAPERBOY_SELFLOOP})
endif()

# Match the known-good HTIT-WB32LAF_V3.2 board build: compile the gbjit-xtensa
# core + JIT at -O3 (the dispatcher busy-loop and codegen are the hot path;
# IDF defaults these to -O2 via CONFIG_COMPILER_OPTIMIZATION_PERF).
set_source_files_properties(
    gbjit-xtensa/core/memory.c
    gbjit-xtensa/core/ppu.c
    gbjit-xtensa/core/ppu_thread.c
    gbjit-xtensa/core/sm83_decoder.c
    gbjit-xtensa/core/sm83_interp.c
    gbjit-xtensa/jit/emit_xtensa.c
    gbjit-xtensa/jit/codecache.c
    gbjit-xtensa/jit/codegen.c
    gbjit-xtensa/jit/dispatcher.c
    PROPERTIES COMPILE_FLAGS "-O3")

# if ((IDF_VERSION_MAJOR GREATER_EQUAL 5) AND (${IDF_TARGET} MATCHES "^esp32$|esp32s2|esp32s3"))
#     target_compile_options(${COMPONENT_LIB} PRIVATE -fjump-tables -ftree-switch-conversion)
# endif ()

# Compile the emulator translation unit at -O3.
# walnut_cgb.h is included here and contains the entire SM83 CPU decoder;
# the extra optimisation (better register allocation, unrolling) typically
# saves 5–15% on the hot loop.
# target_compile_definitions(${COMPONENT_LIB} PRIVATE DEBUG=0 GBJIT_DISPATCHER_CHAIN_BATCH=4)

set_source_files_properties(gbemu.c gbemu_jit.c PROPERTIES COMPILE_FLAGS "-O3")
