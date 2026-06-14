/* On-device interp-vs-JIT diagnostics (PAPERBOY_LOCKSTEP_DIFF=1).
 *
 * Two phases, both driven by a scripted Start press at a fixed GB cycle so the
 * run is fully deterministic and matches the host runner (--press 4500000 0x08
 * --dump-fb 24500000):
 *
 *   Phase A (framebuffer repro): run the reference interpreter standalone to
 *   END_CYC and dump its framebuffer, then the native JIT standalone to END_CYC
 *   and dump its framebuffer. The host (simulator, no register windows) renders
 *   World 1-1 for BOTH engines; if the device JIT renders something else, the
 *   bug is in the real-silicon JIT path the simulator can't model.
 *
 *   Phase B (runaway finder): run interp and JIT in cycle-aligned lockstep.
 *   Benign block-granular interrupt-timing skews diverge for a few blocks then
 *   RECONVERGE; a real codegen bug diverges and never reconverges. Report the
 *   first NON-reconverging ("runaway") divergence — that is the bug — together
 *   with a WRAM diff so the wrong value/branch can be pinpointed.
 *
 * Output (framebuffers + report) goes to the USB-Serial-JTAG console.
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
#include <stddef.h>

static mmu m_i, m_j;
static cpu_state cpu_i, cpu_j;
static gbjit_dispatcher disp;

#define START_CYC   4500000ull      /* inject Start ~frame 64 (title up) */
#define START_REL   5000000ull      /* release Start (matches host runner) */
#define FB_CYC      24500000ull     /* dump framebuffer here (matches host) */
#define END_CYC     12000000ull     /* runaway-finder budget (past the branch) */
#define GB_BTN_START 0x08

#define GB_W 160
#define GB_H 144
static char s_hex[GB_W * GB_H / 4 * 2 + 1];

static void dump_fb(const char *tag, const u8 *fb) {
    static const char hexd[] = "0123456789ABCDEF";
    int o = 0;
    for (int i = 0; i < GB_W * GB_H; i += 4) {
        u8 b = (u8)(((fb[i] & 3) << 6) | ((fb[i+1] & 3) << 4) |
                    ((fb[i+2] & 3) << 2) | (fb[i+3] & 3));
        s_hex[o++] = hexd[b >> 4];
        s_hex[o++] = hexd[b & 0xF];
    }
    s_hex[o] = 0;
    printf("<<FB-%s %d %d>>\n", tag, GB_W, GB_H);
    fwrite(s_hex, 1, (size_t)o, stdout);
    printf("\n<<FBEND>>\n");
    fflush(stdout);
}

/* Run one engine standalone to `end`, scripted Start, then dump its fb.
 * jit_mode: 0=interp, 1=jit default, 2=jit no_cache (no chaining, recompile
 * each block), 3=jit one-block-per-run_until (keeps cache+chaining but returns
 * to the dispatcher caller every block, resetting native window state). */
static void run_and_dump_mode(const char *tag, int jit_mode,
                              const uint8_t *rom, size_t rom_size) {
    bool use_jit = jit_mode != 0;
    mmu *m = use_jit ? &m_j : &m_i;
    cpu_state *c = use_jit ? &cpu_j : &cpu_i;
    gb_mmu_init(m); mmu_load_rom(m, rom, rom_size); cpu_reset(c, m);
    if (use_jit) {
        if (!gbjit_dispatcher_init(&disp, c)) { printf("disp init failed\n"); return; }
        disp.prefetch_enabled = false;
        if (jit_mode == 2) disp.no_cache = true;
    }
    bool started = false, released = false;
    while (c->cycles < FB_CYC) {
        if (!started && c->cycles >= START_CYC) {
            mmu_set_joypad_state(m, GB_BTN_START); started = true;
        }
        if (started && !released && c->cycles >= START_REL) {
            mmu_set_joypad_state(m, 0); released = true;
        }
        u64 next = (jit_mode == 3) ? c->cycles + 1 : c->cycles + 2048;
        if (next > FB_CYC) next = FB_CYC;
        if (use_jit) gbjit_dispatcher_run_until(&disp, next);
        else         sm83_run_until(c, next);
    }
    printf("[%s done cyc=%llu pc=%04X]\n", tag, (unsigned long long)c->cycles, c->pc);
    dump_fb(tag, m->framebuffer);
    if (use_jit) gbjit_dispatcher_shutdown(&disp);
}
static void run_and_dump(const char *tag, bool use_jit,
                         const uint8_t *rom, size_t rom_size) {
    run_and_dump_mode(tag, use_jit ? 1 : 0, rom, rom_size);
}

/* Run the native JIT standalone to `cyc` (Start scripted) and dump WRAM
 * ($C000..$DFFF) as hex over the console for host-golden byte diffing. */
static char s_wram_hex[0x2000 * 2 + 1];
static void run_jit_dump_wram(const uint8_t *rom, size_t rom_size, u64 cyc) {
    static const char hexd[] = "0123456789ABCDEF";
    gb_mmu_init(&m_j); mmu_load_rom(&m_j, rom, rom_size); cpu_reset(&cpu_j, &m_j);
    if (!gbjit_dispatcher_init(&disp, &cpu_j)) { printf("disp init failed\n"); return; }
    disp.prefetch_enabled = false;
    bool started=false, released=false;
    while (cpu_j.cycles < cyc) {
        if (!started && cpu_j.cycles >= START_CYC) { mmu_set_joypad_state(&m_j, GB_BTN_START); started=true; }
        if (started && !released && cpu_j.cycles >= START_REL) { mmu_set_joypad_state(&m_j, 0); released=true; }
        u64 nx = cpu_j.cycles + 2048; if (nx > cyc) nx = cyc;
        gbjit_dispatcher_run_until(&disp, nx);
    }
    int o = 0;
    for (int a = 0xC000; a <= 0xDFFF; a++) {
        u8 v = mmu_read8(&m_j, (u16)a);
        s_wram_hex[o++] = hexd[v >> 4]; s_wram_hex[o++] = hexd[v & 0xF];
    }
    s_wram_hex[o] = 0;
    printf("<<WRAM %llu pc=%04X>>\n", (unsigned long long)cyc, cpu_j.pc);
    fwrite(s_wram_hex, 1, (size_t)o, stdout);
    printf("\n<<WRAMEND>>\n"); fflush(stdout);
    gbjit_dispatcher_shutdown(&disp);
}

static void print_state(const char *who, const cpu_state *c, const mmu *m) {
    printf("  %-6s A=%02X F=%02X BC=%04X DE=%04X HL=%04X SP=%04X PC=%04X cyc=%llu "
           "| LY=%02X STAT=%02X LCDC=%02X IF=%02X IE=%02X ime=%d halt=%d\n",
           who, c->a, c->f, c->bc, c->de, c->hl, c->sp, c->pc,
           (unsigned long long)c->cycles,
           m->io[0x44], m->io[0x41], m->io[0x40], m->io[0x0F], m->ie,
           c->ime, c->halted);
}

static bool cpu_equal(void) {
    return cpu_i.a==cpu_j.a && cpu_i.f==cpu_j.f && cpu_i.bc==cpu_j.bc &&
           cpu_i.de==cpu_j.de && cpu_i.hl==cpu_j.hl && cpu_i.sp==cpu_j.sp &&
           cpu_i.pc==cpu_j.pc;
}

/* Phase C: frame-granular WRAM/OAM/IO diff. Robust to HALT/interrupt phase
 * skew (both engines process the same VBlanks within a frame), so a non-zero,
 * GROWING diff is real corruption, not timing noise. Reports the first frame
 * where memory diverges plus the offending bytes — the value that drives the
 * wrong (bonus-game) branch. */
#define FRAME_CYC 70224ull
/* Count WRAM bytes that differ, EXCLUDING stale stack slots below both SPs
 * (those carry benign popped-frame residue that differs only by interrupt
 * phase) and reporting the lowest differing address (the most meaningful,
 * since SML's level/mode state lives low in WRAM while the stack is at the
 * top). */
static u32 wram_diff_lo(int *first) {
    u32 cnt = 0; *first = -1;
    u16 sp_lo = (cpu_i.sp < cpu_j.sp ? cpu_i.sp : cpu_j.sp);
    for (int a = 0xC000; a <= 0xDFFF; a++) {
        if (a >= 0xCF00 && (u16)a < sp_lo) continue;   /* stale stack residue */
        u8 vi = mmu_read8(&m_i, (u16)a), vj = mmu_read8(&m_j, (u16)a);
        if (vi != vj) { if (*first < 0) *first = a; cnt++; }
    }
    return cnt;
}
static void frame_diff_finder(const uint8_t *rom, size_t rom_size) {
    printf("\n==== FRAME DIFF FINDER (interp vs JIT) ====\n");
    gb_mmu_init(&m_i); mmu_load_rom(&m_i, rom, rom_size); cpu_reset(&cpu_i, &m_i);
    gb_mmu_init(&m_j); mmu_load_rom(&m_j, rom, rom_size); cpu_reset(&cpu_j, &m_j);
    if (!gbjit_dispatcher_init(&disp, &cpu_j)) { printf("disp init failed\n"); return; }
    disp.prefetch_enabled = false;
    bool started=false, released=false;
    int reported = 0;
    for (u64 target = FRAME_CYC; target <= 25000000ull; target += FRAME_CYC) {
        if (!started && target >= START_CYC) {
            mmu_set_joypad_state(&m_i, GB_BTN_START);
            mmu_set_joypad_state(&m_j, GB_BTN_START);
            started = true;
            printf("[Start @target=%llu]\n", (unsigned long long)target);
        }
        if (started && !released && target >= START_REL) {
            mmu_set_joypad_state(&m_i, 0);
            mmu_set_joypad_state(&m_j, 0);
            released = true;
        }
        sm83_run_until(&cpu_i, target);
        gbjit_dispatcher_run_until(&disp, target);
        int f; u32 cnt = wram_diff_lo(&f);
        /* skip tiny benign (counter/phase) noise; report real divergence */
        if (cnt > 6) {
            printf("frame@cyc~%llu: WRAM diff cnt=%u lowest=%04X  "
                   "iPC=%04X jPC=%04X iSP=%04X jSP=%04X\n",
                   (unsigned long long)target, cnt, f,
                   cpu_i.pc, cpu_j.pc, cpu_i.sp, cpu_j.sp);
            int shown = 0;
            for (int a = (f<0?0xC000:f); a <= 0xDFFF && shown < 20; a++) {
                u8 vi = mmu_read8(&m_i,(u16)a), vj = mmu_read8(&m_j,(u16)a);
                if (vi != vj) { printf("  [%04X] i=%02X j=%02X\n", a, vi, vj); shown++; }
            }
            if (++reported >= 3) {
                printf("==== stopping after %d diverged frames ====\n", reported);
                gbjit_dispatcher_shutdown(&disp);
                return;
            }
        }
    }
    printf("==== frame finder: no WRAM divergence(>6) through 25M cyc ====\n");
    gbjit_dispatcher_shutdown(&disp);
}

/* Phase D: resync-each-block pinpoint. Run both to RESYNC_CYC, then for each
 * JIT block: snapshot interp==jit, run one JIT block, advance interp to the
 * same cycle, compare. Full resync after every block eliminates accumulated
 * interrupt-phase drift, so the FIRST block whose JIT output differs from the
 * interpreter's (same input state, same cycle delta) is the mis-translated
 * block. Reports its PC/op and both result states. */
/* Copy interp mmu+cpu into the jit's, EXCLUDING the write-only framebuffer
 * (the CPU never reads it, so it can't affect execution) — keeps the per-block
 * resync cheap enough to run over the whole boot->Start range. */
static void copy_i_to_j(void) {
    u8 *rs = m_j.rom; u32 cap = m_j.rom_capacity;
    size_t head = offsetof(mmu, framebuffer);
    memcpy(&m_j, &m_i, head);
    size_t tail_off = offsetof(mmu, framebuffer) + sizeof(m_i.framebuffer);
    memcpy((u8*)&m_j + tail_off, (u8*)&m_i + tail_off, sizeof(mmu) - tail_off);
    m_j.rom = rs; m_j.rom_capacity = cap; m_j.cpu = &cpu_j;
    cpu_j = cpu_i; cpu_j.mmu = &m_j;
}
static void block_pinpoint(const uint8_t *rom, size_t rom_size,
                           u64 resync_cyc, u64 end_cyc) {
    (void)resync_cyc;
    printf("\n==== BLOCK PINPOINT [boot..%llu] ====\n",
           (unsigned long long)end_cyc);
    gb_mmu_init(&m_i); mmu_load_rom(&m_i, rom, rom_size); cpu_reset(&cpu_i, &m_i);
    gb_mmu_init(&m_j); mmu_load_rom(&m_j, rom, rom_size); cpu_reset(&cpu_j, &m_j);
    if (!gbjit_dispatcher_init(&disp, &cpu_j)) { printf("disp init failed\n"); return; }
    disp.prefetch_enabled = false;
    bool started=false, released=false;
    int reports = 0;
    while (cpu_i.cycles < end_cyc) {
        if (!started && cpu_i.cycles >= START_CYC) { mmu_set_joypad_state(&m_i, GB_BTN_START); started=true; }
        if (started && !released && cpu_i.cycles >= START_REL) { mmu_set_joypad_state(&m_i, 0); released=true; }
        copy_i_to_j();                         /* jit := interp (identical) */
        u16 pc_before = cpu_j.pc;
        u8  op_before = mmu_read8(&m_j, pc_before);
        if (cpu_j.halted) {                    /* skip halt-wait uniformly */
            sm83_step(&cpu_i);
            continue;
        }
        u64 c0 = cpu_j.cycles;
        gbjit_dispatcher_run_until(&disp, c0 + 1);   /* exactly one block */
        u64 jdelta = cpu_j.cycles - c0;
        u64 itarget = cpu_i.cycles + jdelta;
        while (cpu_i.cycles < itarget) sm83_step(&cpu_i);
        bool eq = cpu_i.a==cpu_j.a && cpu_i.f==cpu_j.f && cpu_i.bc==cpu_j.bc &&
                  cpu_i.de==cpu_j.de && cpu_i.hl==cpu_j.hl && cpu_i.sp==cpu_j.sp &&
                  cpu_i.pc==cpu_j.pc;
        if (!eq) {
            /* ignore pure interrupt-phase difference: interp entered an ISR
             * (pc in 40/48/50/58/60 or ime just cleared) that the jit will take
             * at the next boundary. Heuristic: if only PC/SP/ime differ AND the
             * data regs (A,BC,DE,HL) all match, treat as benign and resync. */
            bool data_eq = cpu_i.a==cpu_j.a && cpu_i.bc==cpu_j.bc &&
                           cpu_i.de==cpu_j.de && cpu_i.hl==cpu_j.hl;
            bool isr_like = data_eq && (cpu_i.pc & 0xFFC7) == 0x0040 && cpu_i.pc != cpu_j.pc;
            if (isr_like) continue;            /* benign; next iter resyncs */
            printf("\n*** BLOCK MISMATCH @PC=%04X op=%02X (cyc=%llu, jdelta=%llu) ***\n",
                   pc_before, op_before, (unsigned long long)c0, (unsigned long long)jdelta);
            print_state("interp", &cpu_i, &m_i);
            print_state("jit   ", &cpu_j, &m_j);
            if (++reports >= 14) { printf("==== stop after %d ====\n", reports);
                                  gbjit_dispatcher_shutdown(&disp); return; }
        }
    }
    printf("==== pinpoint: %d mismatches, ran to cyc=%llu ====\n",
           reports, (unsigned long long)cpu_i.cycles);
    gbjit_dispatcher_shutdown(&disp);
}

/* Phase B: cycle-aligned lockstep; report first non-reconverging divergence. */
static void runaway_finder(const uint8_t *rom, size_t rom_size) {
    printf("\n==== RUNAWAY FINDER (interp vs JIT) ====\n");
    gb_mmu_init(&m_i); mmu_load_rom(&m_i, rom, rom_size); cpu_reset(&cpu_i, &m_i);
    gb_mmu_init(&m_j); mmu_load_rom(&m_j, rom, rom_size); cpu_reset(&cpu_j, &m_j);
    if (!gbjit_dispatcher_init(&disp, &cpu_j)) { printf("disp init failed\n"); return; }
    disp.prefetch_enabled = false;

    bool started = false, released = false;
    unsigned long blocks = 0;
    /* divergence tracking */
    bool diverged = false;
    unsigned long since_div = 0;
    u16 div_pc = 0; u8 div_op = 0; u64 div_cyc = 0;
    cpu_state div_i, div_j;       /* snapshots at first divergence */
    const unsigned long RECONV_LIMIT = 4000; /* blocks before declaring runaway */

    while (cpu_j.cycles < END_CYC) {
        if (!started && cpu_j.cycles >= START_CYC) {
            mmu_set_joypad_state(&m_i, GB_BTN_START);
            mmu_set_joypad_state(&m_j, GB_BTN_START);
            started = true;
            printf("[Start @cyc=%llu]\n", (unsigned long long)cpu_j.cycles);
        }
        if (started && !released && cpu_j.cycles >= START_REL) {
            mmu_set_joypad_state(&m_i, 0);
            mmu_set_joypad_state(&m_j, 0);
            released = true;
        }

        u64 before = cpu_j.cycles;
        u16 pc_before = cpu_j.pc;
        u8  op_before = mmu_read8(cpu_j.mmu, pc_before);
        gbjit_dispatcher_run_until(&disp, before + 1);  /* >= 1 block */
        u64 delta = cpu_j.cycles - before;
        u64 target = cpu_i.cycles + delta;
        while (cpu_i.cycles < target) sm83_step(&cpu_i);
        blocks++;

        bool eq = cpu_equal();
        if (!diverged) {
            if (!eq) {
                /* before Start: benign block-granular skew -> resync silently */
                if (before < START_CYC) {
                    u8 *rs = m_j.rom; u32 cap = m_j.rom_capacity;
                    m_j = m_i; m_j.rom = rs; m_j.rom_capacity = cap; m_j.cpu = &cpu_j;
                    cpu_j = cpu_i; cpu_j.mmu = &m_j;
                    continue;
                }
                /* first post-Start divergence: start watching for reconvergence */
                diverged = true; since_div = 0;
                div_pc = pc_before; div_op = op_before; div_cyc = before;
                div_i = cpu_i; div_j = cpu_j;
            }
        } else {
            since_div++;
            if (eq) {
                /* reconverged -> it was a benign timing skew; clear and go on */
                diverged = false;
            } else if (since_div >= RECONV_LIMIT) {
                /* RUNAWAY: never reconverged. Report. */
                printf("\n*** RUNAWAY divergence (no reconverge in %lu blocks) ***\n",
                       RECONV_LIMIT);
                printf("first diverged at block@PC=%04X op=%02X cyc=%llu:\n",
                       div_pc, div_op, (unsigned long long)div_cyc);
                print_state("  interp@div", &div_i, &m_i);
                print_state("  jit@div   ", &div_j, &m_j);
                printf("now (after %lu more blocks):\n", since_div);
                print_state("  interp@now", &cpu_i, &m_i);
                print_state("  jit@now   ", &cpu_j, &m_j);
                /* WRAM diff (C000-DFFF): first/last differing byte + count */
                int first = -1, last = -1, cnt = 0;
                for (int a = 0xC000; a <= 0xDFFF; a++) {
                    u8 vi = mmu_read8(&m_i, (u16)a), vj = mmu_read8(&m_j, (u16)a);
                    if (vi != vj) { if (first < 0) first = a; last = a; cnt++; }
                }
                printf("WRAM diffs: count=%d range=%04X..%04X\n", cnt, first, last);
                for (int a = (first<0?0:first); a <= last && a >= 0; a++) {
                    u8 vi = mmu_read8(&m_i, (u16)a), vj = mmu_read8(&m_j, (u16)a);
                    if (vi != vj)
                        printf("  WRAM[%04X] interp=%02X jit=%02X\n", a, vi, vj);
                }
                /* HRAM diff (FF80-FFFE) */
                for (int a = 0xFF80; a <= 0xFFFE; a++) {
                    u8 vi = mmu_read8(&m_i, (u16)a), vj = mmu_read8(&m_j, (u16)a);
                    if (vi != vj) printf("  HRAM[%04X] interp=%02X jit=%02X\n", a, vi, vj);
                }
                fflush(stdout);
                gbjit_dispatcher_shutdown(&disp);
                return;
            }
        }
        if ((blocks & 0x1FFFFF) == 0)
            printf("[..%lu blocks cyc=%llu]\n", blocks, (unsigned long long)cpu_j.cycles);
    }
    printf("==== runaway finder: no runaway in %lu blocks (cyc=%llu) ====\n",
           blocks, (unsigned long long)cpu_j.cycles);
    gbjit_dispatcher_shutdown(&disp);
}

void lockstep_diff_run(const uint8_t *rom, size_t rom_size) {
    printf("\n==== ON-DEVICE DIAG (rom=%u bytes) ====\n", (unsigned)rom_size);
    /* Phase A confirmed the repro (interp->1-1, jit->bonus); skip it now and go
     * straight to the runaway finder so it isn't starved by the two slow 24.5M
     * framebuffer renders. Re-enable run_and_dump() to re-capture frames. */
    (void)run_and_dump;
    (void)run_and_dump_mode;
    (void)runaway_finder;
    (void)frame_diff_finder;
    (void)block_pinpoint;
    (void)run_jit_dump_wram;
    /* FIX VERIFICATION: dump device JIT framebuffer (and C0D7) at 24.5M after
     * Start. Expect World 1-1 (was BONUS GAME) and C0D7 matching golden. */
    run_jit_dump_wram(rom, rom_size, 4700000ull);   /* C0D7 check */
    run_and_dump_mode("JITFIX", 1, rom, rom_size);  /* framebuffer */
    printf("==== DIAG COMPLETE ====\n");
    fflush(stdout);
}

#endif /* PAPERBOY_LOCKSTEP_DIFF */
