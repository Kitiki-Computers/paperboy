#pragma once
#include <stdint.h>
#include <stddef.h>

#if defined(PAPERBOY_LOCKSTEP_DIFF) && PAPERBOY_LOCKSTEP_DIFF
/* Run the on-device interp-vs-JIT lockstep differential on `rom` and report
 * the first post-Start divergence over the console. Does not return until the
 * diff completes (then the caller should idle). */
void lockstep_diff_run(const uint8_t *rom, size_t rom_size);
#endif
