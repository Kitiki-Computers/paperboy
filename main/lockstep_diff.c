/* On-device lockstep interp-vs-JIT differential (PAPERBOY_LOCKSTEP_DIFF=1).
 *
 * Runs the reference interpreter and the native JIT side-by-side on the same
 * ROM, injects a scripted Start press, and reports the first block where the
 * native JIT's CPU state diverges from the interpreter — i.e. the exact point
 * the on-device JIT mis-executes (the host Xtensa simulator does not). Output
 * goes to the USB-Serial-JTAG console. This is the device port of
 * tests/test_stepwise_diff.c.
 */
#if defined(PAPERBOY_LOCKSTEP_DIFF) && PAPERBOY_LOCKSTEP_DIFF

#include "lockstep_diff.h"
#include "cpu_state.h"
#include "memory.h"
#include "sm83_interp.h"
#include "dispatcher.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>

static mmu m_i, m_j;
static cpu_state cpu_i, cpu_j;
static gbjit_dispatcher disp;

#define START_CYC   4500000ull      /* inject Start ~frame 64 (title up) */
#define START_REL   4700000ull      /* release Start */
#define END_CYC     7000000ull      /* run to ~frame 100 */
#define GB_BTN_START 0x08

static void print_state(const char *who, const cpu_state *c, const mmu *m) {
    printf("  %-6s A=%02X F=%02X BC=%04X DE=%04X HL=%04X SP=%04X PC=%04X cyc=%llu "
           "| LY=%02X STAT=%02X LCDC=%02X IF=%02X IE=%02X ime=%d halt=%d\n",
           who, c->a, c->f, c->bc, c->de, c->hl, c->sp, c->pc,
           (unsigned long long)c->cycles,
           m->io[0x44], m->io[0x41], m->io[0x40], m->io[0x0F], m->ie,
           c->ime, c->halted);
}

void lockstep_diff_run(const uint8_t *rom, size_t rom_size) {
    printf("\n==== LOCKSTEP interp-vs-JIT diff (rom=%u bytes) ====\n", (unsigned)rom_size);
    gb_mmu_init(&m_i); mmu_load_rom(&m_i, rom, rom_size); cpu_reset(&cpu_i, &m_i);
    gb_mmu_init(&m_j); mmu_load_rom(&m_j, rom, rom_size); cpu_reset(&cpu_j, &m_j);
    if (!gbjit_dispatcher_init(&disp, &cpu_j)) { printf("dispatcher init failed\n"); return; }
    disp.prefetch_enabled = false;

    bool started = false, released = false;
    int post_start_divs = 0;
    unsigned long blocks = 0;

    while (cpu_j.cycles < END_CYC) {
        /* scripted Start (both sides identically) */
        if (!started && cpu_j.cycles >= START_CYC) {
            mmu_set_joypad_state(&m_i, GB_BTN_START);
            mmu_set_joypad_state(&m_j, GB_BTN_START);
            started = true;
            printf("[Start pressed @cyc=%llu]\n", (unsigned long long)cpu_j.cycles);
        }
        if (started && !released && cpu_j.cycles >= START_REL) {
            mmu_set_joypad_state(&m_i, 0);
            mmu_set_joypad_state(&m_j, 0);
            released = true;
        }

        u64 before = cpu_j.cycles;
        u16 pc_before = cpu_j.pc;
        u8  op_before = mmu_read8(cpu_j.mmu, pc_before);
        gbjit_dispatcher_run_until(&disp, before + 1);   /* >= 1 block */
        u64 delta = cpu_j.cycles - before;
        u64 target = cpu_i.cycles + delta;
        while (cpu_i.cycles < target) sm83_step(&cpu_i);
        blocks++;

        bool diff = (cpu_i.a!=cpu_j.a)||(cpu_i.f!=cpu_j.f)||(cpu_i.bc!=cpu_j.bc)||
                    (cpu_i.de!=cpu_j.de)||(cpu_i.hl!=cpu_j.hl)||(cpu_i.sp!=cpu_j.sp)||
                    (cpu_i.pc!=cpu_j.pc);
        if (diff) {
            bool post = (before >= START_CYC);
            if (!post) {
                /* benign pre-Start block-granular divergence — resync silently */
                u8 *rs = m_j.rom; u32 cap = m_j.rom_capacity;
                m_j = m_i; m_j.rom = rs; m_j.rom_capacity = cap; m_j.cpu = &cpu_j;
                cpu_j = cpu_i; cpu_j.mmu = &m_j;
                continue;
            }
            /* POST-Start divergence — the bug. Report and let it run free. */
            post_start_divs++;
            printf("\n*** DIVERGENCE #%d after Start (block @PC=%04X op=%02X, jit cyc=%llu) ***\n",
                   post_start_divs, pc_before, op_before, (unsigned long long)before);
            print_state("interp", &cpu_i, &m_i);
            print_state("jit",    &cpu_j, &m_j);
            if (post_start_divs >= 8) {
                printf("==== stopping after %d post-Start divergences ====\n", post_start_divs);
                return;
            }
            /* do NOT resync — watch the JIT run away from the interp */
        }
        if ((blocks & 0xFFFFF) == 0)
            printf("[..%lu blocks, cyc=%llu]\n", blocks, (unsigned long long)cpu_j.cycles);
    }
    printf("==== lockstep done: %lu blocks, cyc=%llu, post-start divs=%d ====\n",
           blocks, (unsigned long long)cpu_j.cycles, post_start_divs);
}

#endif /* PAPERBOY_LOCKSTEP_DIFF */
