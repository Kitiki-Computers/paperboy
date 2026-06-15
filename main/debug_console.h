#pragma once
#include <stdint.h>
#include <stdbool.h>

/* On-device UART (USB-Serial-JTAG) debug console.
 *
 * Opt-in via -DPAPERBOY_DEBUG_CONSOLE=1. Lets a host driver inject joypad
 * input, capture the GB framebuffer, and read FPS / worst-case timing over
 * the same cable used for flashing. See debug_console.c for the protocol. */

#if defined(PAPERBOY_DEBUG_CONSOLE) && PAPERBOY_DEBUG_CONSOLE

void    dbg_console_init(void);            /* install USB-JTAG driver + RX task */
uint8_t dbg_console_button_mask(void);     /* held debug buttons, OR into input */
/* Call once per emulated frame from the render loop. gb_fb = 160x144 shade
 * buffer (0..3) or NULL. Services pending dump / stats / tap-countdown and
 * accumulates frame-time stats. */
void    dbg_console_after_frame(const uint8_t *gb_fb);

#else
static inline void    dbg_console_init(void) {}
static inline uint8_t dbg_console_button_mask(void) { return 0; }
static inline void    dbg_console_after_frame(const uint8_t *gb_fb) { (void)gb_fb; }
#endif
