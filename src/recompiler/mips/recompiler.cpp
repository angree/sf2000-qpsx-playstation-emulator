/*
 * Mips-to-mips recompiler for pcsx4all
 *
 * Copyright (c) 2009 Ulrich Hecht
 * Copyright (c) 2017 modified by Dmitry Smagin, Daniel Silsby
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <stddef.h>
#include "plugin_lib.h"
#include "psxcommon.h"
#include "psxhle.h"
#include "psxmem.h"
#include "psxhw.h"
#include "r3000a.h"
#include "gte.h"
#include "profiler.h"   /* v092: Profiler support */

/* For direct HW I/O */
#include "mdec.h"
#include "cdrom.h"
#include "gpu.h"

/* Standard console logging */
#define REC_LOG(...) printf("mipsrec: " __VA_ARGS__)
#ifndef REC_LOG
#define REC_LOG(...)
#endif

/* Verbose console logging (uncomment next line to enable) */
//#define REC_LOG_V REC_LOG
#ifndef REC_LOG_V
#define REC_LOG_V(...)
#endif

/* Use inlined-asm version of block dispatcher: */
/* QPSX_039: FIX - Clear emu_frame_complete BEFORE entering loop, exit after frame */
// QPSX_047: DISABLED for debugging
// #define ASM_EXECUTE_LOOP

extern "C" void xlog(const char *fmt, ...);

/* v192: Counters from rec_native.cpp.h for stats */
extern u32 native_used;
extern u32 dynarec_used;

/* ============== v194: DIFFERENTIAL TESTING (FIXED) ==============
 * Run BOTH dynarec and native for each block, compare results.
 * Dynarec is used for actual execution (game works).
 * Native runs in parallel for comparison (detects bugs).
 *
 * v194 FIXES:
 * - MUCH smaller buffers (was 6MB, now 256KB total)
 * - Only log on MISMATCH (not every comparison)
 * - Log instruction types when mismatch found
 */
#define DIFF_TEST_ENABLE 1   /* v225: ON - rozszerzony diff test (rejestry + pamięć) */
#define DIFF_TEST_LOG_LIMIT 20  /* Max mismatches to log */

#if DIFF_TEST_ENABLE
/* v194: SMALLER buffers - SF2000 has limited RAM! */
#define NATIVE_CODE_BUF_SIZE (128*1024)  /* 128KB for native code */
#define NATIVE_LUT_SIZE      (16*1024)   /* 16K entries = 64KB for LUT */

static u8 native_code_buffer[NATIVE_CODE_BUF_SIZE] __attribute__((aligned(16)));
static u32 native_code_guard[16] __attribute__((aligned(4)));  /* v238: Guard after buffer */
static u8* native_code_ptr = native_code_buffer;
static u8* native_code_end = native_code_buffer + NATIVE_CODE_BUF_SIZE;

/* v238: Overflow detection for native_code_buffer */
static void init_native_code_guard(void) {
    for (int i = 0; i < 16; i++) {
        native_code_guard[i] = 0xCAFEBABE;
    }
}

static bool check_native_code_overflow(void) {
    for (int i = 0; i < 16; i++) {
        if (native_code_guard[i] != 0xCAFEBABE) {
            xlog("!!! OVERFLOW in native_code_buffer! guard[%d]=0x%08X\n", i, native_code_guard[i]);
            return true;
        }
    }
    return false;
}

/* Simple hash LUT - wraps around if collision */
static void* native_block_lut[NATIVE_LUT_SIZE] __attribute__((aligned(4)));
static u32   native_block_pc[NATIVE_LUT_SIZE] __attribute__((aligned(4)));  /* Store PC for verification */

/* Stats for differential testing */
static u32 diff_test_count __attribute__((aligned(4))) = 0;
static u32 diff_mismatch_count __attribute__((aligned(4))) = 0;
static u32 diff_native_compiled __attribute__((aligned(4))) = 0;

/* Convert PC to LUT index (simple hash) */
static inline u32 pc_to_lut_idx(u32 pc) {
    return ((pc >> 2) ^ (pc >> 14)) & (NATIVE_LUT_SIZE - 1);
}

/* Register names for logging */
static const char* gpr_names[32] = {
    "r0", "at", "v0", "v1", "a0", "a1", "a2", "a3",
    "t0", "t1", "t2", "t3", "t4", "t5", "t6", "t7",
    "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7",
    "t8", "t9", "k0", "k1", "gp", "sp", "fp", "ra"
};

/* v194: Log block instructions on mismatch */
static void diff_log_block_instrs(u32 pc) {
    xlog("  Block instrs at %08X:\n", pc);
    for (int i = 0; i < 8; i++) {  /* Log up to 8 instructions */
        u32 addr = pc + i * 4;
        if (addr < 0x00200000 || (addr >= 0x80000000 && addr < 0x80200000)) {
            u32 op = *(u32*)PSXM(addr);
            u32 opcode = op >> 26;
            const char* name = "???";
            if (op == 0) name = "NOP";
            else if (opcode == 0x00) name = "ALU-R";
            else if (opcode >= 0x08 && opcode <= 0x0E) name = "ALU-I";
            else if (opcode == 0x0F) name = "LUI";
            else if (opcode >= 0x20 && opcode <= 0x26) name = "LOAD";
            else if (opcode >= 0x28 && opcode <= 0x2E) name = "STORE";
            else if (opcode == 0x04) name = "BEQ";
            else if (opcode == 0x05) name = "BNE";
            else if (opcode == 0x06) name = "BLEZ";
            else if (opcode == 0x07) name = "BGTZ";
            else if (opcode == 0x02) name = "J";
            else if (opcode == 0x03) name = "JAL";
            xlog("    [%d] %08X: %s\n", i, op, name);
        }
    }
}

/* v225: ROZSZERZONY DIFF TEST
 * Porównuje: GPR, PC, lo, hi, cycle + memory checksum
 *
 * v210: SKIP $at (register 1) - dynarec uses it as scratch
 */

/* v225: Memory checksum counter */
static u32 diff_mem_mismatch_count __attribute__((aligned(4))) = 0;

/* v225: Simple XOR checksum for memory region */
static u32 mem_checksum(u8* mem, u32 size) {
    u32 sum = 0;
    u32* p = (u32*)mem;
    u32 words = size / 4;
    for (u32 i = 0; i < words; i++) {
        sum ^= p[i];
    }
    return sum;
}

/* v225: Extended comparison - GPR + lo + hi + cycle
 * Memory comparison done separately in main loop
 */
static bool diff_test_compare(u32 pc, psxRegisters* before, psxRegisters* dynarec, psxRegisters* native) {
    bool mismatch = false;
    bool gpr_mismatch = false;
    bool lohi_mismatch = false;
    bool pc_mismatch = false;

    /* Check GPR - SKIP $at (reg 1), it's scratch! */
    for (int i = 2; i < 32; i++) {
        if (dynarec->GPR.r[i] != native->GPR.r[i]) {
            gpr_mismatch = true;
            mismatch = true;
            break;
        }
    }

    /* Check PC */
    if (dynarec->pc != native->pc) {
        pc_mismatch = true;
        mismatch = true;
    }

    /* v225: Check lo/hi (MULT/DIV results) */
    if (dynarec->GPR.n.lo != native->GPR.n.lo ||
        dynarec->GPR.n.hi != native->GPR.n.hi) {
        lohi_mismatch = true;
        mismatch = true;
    }

    if (!mismatch) {
        diff_test_count++;
        return false;  /* Match - native produced same result as dynarec */
    }

    /* MISMATCH FOUND - log FULL state for first 10 mismatches */
    if (diff_mismatch_count >= 10) {
        diff_mismatch_count++;
        diff_test_count++;
        return true;
    }

    xlog("\n=== MISMATCH #%d at PC=%08X ===\n", diff_mismatch_count + 1, pc);
    if (gpr_mismatch) xlog("  TYPE: GPR\n");
    if (pc_mismatch) xlog("  TYPE: PC\n");
    if (lohi_mismatch) xlog("  TYPE: LO/HI\n");

    diff_log_block_instrs(pc);

    /* Log BEFORE state */
    xlog("  --- BEFORE ---\n");
    xlog("  v0=%08X v1=%08X a0=%08X a1=%08X\n",
         before->GPR.r[2], before->GPR.r[3], before->GPR.r[4], before->GPR.r[5]);
    xlog("  lo=%08X hi=%08X\n", before->GPR.n.lo, before->GPR.n.hi);

    /* Log DYNAREC result */
    xlog("  --- DYNAREC ---\n");
    xlog("  v0=%08X v1=%08X a0=%08X a1=%08X\n",
         dynarec->GPR.r[2], dynarec->GPR.r[3], dynarec->GPR.r[4], dynarec->GPR.r[5]);
    xlog("  lo=%08X hi=%08X pc=%08X\n", dynarec->GPR.n.lo, dynarec->GPR.n.hi, dynarec->pc);

    /* Log NATIVE result */
    xlog("  --- NATIVE ---\n");
    xlog("  v0=%08X v1=%08X a0=%08X a1=%08X\n",
         native->GPR.r[2], native->GPR.r[3], native->GPR.r[4], native->GPR.r[5]);
    xlog("  lo=%08X hi=%08X pc=%08X\n", native->GPR.n.lo, native->GPR.n.hi, native->pc);

    /* Log which registers DIFFER */
    xlog("  --- DIFFS ---\n");
    for (int i = 1; i < 32; i++) {
        if (dynarec->GPR.r[i] != native->GPR.r[i]) {
            xlog("  $%s: dyn=%08X nat=%08X\n",
                 gpr_names[i], dynarec->GPR.r[i], native->GPR.r[i]);
        }
    }
    if (lohi_mismatch) {
        xlog("  LO: dyn=%08X nat=%08X\n", dynarec->GPR.n.lo, native->GPR.n.lo);
        xlog("  HI: dyn=%08X nat=%08X\n", dynarec->GPR.n.hi, native->GPR.n.hi);
    }
    if (pc_mismatch) {
        xlog("  PC: dyn=%08X nat=%08X\n", dynarec->pc, native->pc);
    }

    diff_mismatch_count++;
    diff_test_count++;

    if (diff_mismatch_count >= 10) {
        xlog("=== GPR log limit reached ===\n");
    }
    return true;  /* Mismatch */
}
#endif /* DIFF_TEST_ENABLE */

/* ============== v237: NATIVE DEBUG RING BUFFER ==============
 * Captures last N native block executions for crash debugging.
 * When native mode freezes, we can see what blocks ran before crash.
 */
#define NATIVE_DEBUG_RING 1          /* v238: RE-ENABLED with overflow guards */
#define NATIVE_DEBUG_RING_SIZE 50    /* Last 50 blocks */

#if NATIVE_DEBUG_RING
struct NativeDebugEntry {
    u32 pc_before;      /* PC before execution */
    u32 pc_after;       /* PC after execution */
    u32 cycle_before;   /* Cycle count before */
    u32 cycle_after;    /* Cycle count after */
    u32 v0_before;      /* $v0 before */
    u32 v0_after;       /* $v0 after */
    u32 v1_before;      /* $v1 before */
    u32 v1_after;       /* $v1 after */
    u32 sp_before;      /* $sp before */
    u32 ra_before;      /* $ra before */
    u32 block_ptr;      /* Pointer to block code */
    u32 is_native;      /* 1=native, 0=dynarec */
};

static NativeDebugEntry native_debug_ring[NATIVE_DEBUG_RING_SIZE] __attribute__((aligned(4)));
static u32 native_debug_ring_idx __attribute__((aligned(4))) = 0;
static u32 native_debug_ring_count __attribute__((aligned(4))) = 0;
static bool native_debug_dumped __attribute__((aligned(4))) = false;

static void native_debug_ring_add(u32 pc_before, u32 block_ptr, bool is_native) {
    NativeDebugEntry* e = &native_debug_ring[native_debug_ring_idx];
    e->pc_before = pc_before;
    e->cycle_before = psxRegs.cycle;
    e->v0_before = psxRegs.GPR.n.v0;
    e->v1_before = psxRegs.GPR.n.v1;
    e->sp_before = psxRegs.GPR.n.sp;
    e->ra_before = psxRegs.GPR.n.ra;
    e->block_ptr = block_ptr;
    e->is_native = is_native ? 1 : 0;
    /* After fields filled by native_debug_ring_complete() */
}

static void native_debug_ring_complete(void) {
    NativeDebugEntry* e = &native_debug_ring[native_debug_ring_idx];
    e->pc_after = psxRegs.pc;
    e->cycle_after = psxRegs.cycle;
    e->v0_after = psxRegs.GPR.n.v0;
    e->v1_after = psxRegs.GPR.n.v1;
    /* Advance ring */
    native_debug_ring_idx = (native_debug_ring_idx + 1) % NATIVE_DEBUG_RING_SIZE;
    if (native_debug_ring_count < NATIVE_DEBUG_RING_SIZE)
        native_debug_ring_count++;
}

static void native_debug_ring_dump(void) {
    if (native_debug_dumped) return;
    native_debug_dumped = true;

    xlog("\n=== NATIVE DEBUG RING DUMP (last %d blocks) ===\n", native_debug_ring_count);

    /* Start from oldest entry */
    u32 start_idx = (native_debug_ring_count < NATIVE_DEBUG_RING_SIZE)
                  ? 0
                  : native_debug_ring_idx;

    for (u32 i = 0; i < native_debug_ring_count; i++) {
        u32 idx = (start_idx + i) % NATIVE_DEBUG_RING_SIZE;
        NativeDebugEntry* e = &native_debug_ring[idx];

        xlog("[%02d] %s PC:%08X->%08X cyc:%d->%d (+%d)\n",
             i, e->is_native ? "NAT" : "DYN",
             e->pc_before, e->pc_after,
             e->cycle_before, e->cycle_after,
             e->cycle_after - e->cycle_before);
        xlog("     v0:%08X->%08X v1:%08X->%08X sp:%08X ra:%08X ptr:%08X\n",
             e->v0_before, e->v0_after,
             e->v1_before, e->v1_after,
             e->sp_before, e->ra_before, e->block_ptr);
    }
    xlog("=== END RING DUMP ===\n");
}
#endif /* NATIVE_DEBUG_RING */

/* v217: These counters are needed in ALL modes (not just DIFF_TEST) */
/* v200: Track native execution vs dynarec fallback (extern for display) */
u32 native_exec_count __attribute__((aligned(4))) = 0;
u32 dynarec_fallback_count __attribute__((aligned(4))) = 0;

/* v205: COMPLETE rejection tracking - EVERY instruction type! */
u32 nat_blocks_attempted __attribute__((aligned(4))) = 0;
/* Pre-loop rejections: */
u32 nat_rej_remap __attribute__((aligned(4))) = 0;    /* $sp/$fp/$ra */
u32 nat_rej_k0k1 __attribute__((aligned(4))) = 0;     /* $k0/$k1 */
u32 nat_rej_garbage __attribute__((aligned(4))) = 0;  /* garbage block */
/* Jump/branch: */
u32 nat_rej_j __attribute__((aligned(4))) = 0;        /* J */
u32 nat_rej_jal __attribute__((aligned(4))) = 0;      /* JAL */
u32 nat_rej_jr __attribute__((aligned(4))) = 0;       /* JR */
u32 nat_rej_jalr __attribute__((aligned(4))) = 0;     /* JALR */
u32 nat_rej_bxx __attribute__((aligned(4))) = 0;      /* branches */
u32 nat_rej_regimm __attribute__((aligned(4))) = 0;   /* BLTZ/BGEZ */
u32 nat_rej_delay __attribute__((aligned(4))) = 0;    /* non-NOP delay */
/* Coprocessor: */
u32 nat_rej_cop0 __attribute__((aligned(4))) = 0;     /* MFC0/MTC0/RFE */
u32 nat_rej_gte __attribute__((aligned(4))) = 0;      /* COP2/GTE ops */
u32 nat_rej_lwc2 __attribute__((aligned(4))) = 0;     /* LWC2 */
u32 nat_rej_swc2 __attribute__((aligned(4))) = 0;     /* SWC2 */
/* Unaligned load/store: */
u32 nat_rej_lwl __attribute__((aligned(4))) = 0;      /* LWL */
u32 nat_rej_lwr __attribute__((aligned(4))) = 0;      /* LWR */
u32 nat_rej_swl __attribute__((aligned(4))) = 0;      /* SWL */
u32 nat_rej_swr __attribute__((aligned(4))) = 0;      /* SWR */
/* Other: */
u32 nat_rej_syscall __attribute__((aligned(4))) = 0;  /* SYSCALL */
u32 nat_rej_break __attribute__((aligned(4))) = 0;    /* BREAK */
u32 nat_rej_other __attribute__((aligned(4))) = 0;    /* unknown */

/* v218: Buffer for native compilation attempts in production mode
 * Dynarec compiles to main buffer first, then we try native to this temp buffer.
 * If native succeeds, we copy it to main buffer (overwrites dynarec).
 * If native fails, dynarec code remains in main buffer untouched.
 */
#define NATIVE_TRY_BUF_SIZE (32*1024)  /* v242: 32KB for single block (was 16KB - too small!) */
static u8 native_try_buffer[NATIVE_TRY_BUF_SIZE] __attribute__((aligned(16)));

/* v238: OVERFLOW DETECTION - guard patterns after buffers */
#define GUARD_PATTERN 0xDEADBEEF
#define GUARD_SIZE 64  /* 64 bytes = 16 words */
static u32 native_try_guard[GUARD_SIZE/4] __attribute__((aligned(4)));

static void init_overflow_guards(void) {
    for (int i = 0; i < GUARD_SIZE/4; i++) {
        native_try_guard[i] = GUARD_PATTERN;
    }
}

static bool check_native_try_overflow(void) {
    for (int i = 0; i < GUARD_SIZE/4; i++) {
        if (native_try_guard[i] != GUARD_PATTERN) {
            xlog("!!! OVERFLOW DETECTED in native_try_buffer! guard[%d]=0x%08X\n",
                 i, native_try_guard[i]);
            return true;
        }
    }
    return false;
}

/* v139: g_native_cycles global is no longer needed!
 * Native blocks now set $v1 = cycle count AFTER emit_save_gprs(),
 * so recFunc's normal dispatcher adds $v1 to psxRegs.cycle.
 * This is the same mechanism dynarec blocks use.
 */

/* v130: EXECUTION DEBUGGING - smarter logging */
/* Only log when PC CHANGES (new block), not every iteration of loops */
/* v192: DISABLED - was killing performance by forcing C dispatch */
#define NATIVE_EXEC_DEBUG 0
#if NATIVE_EXEC_DEBUG
/* v151: Aligned for SF2000 stability */
static u32 native_exec_log_count __attribute__((aligned(4))) = 0;
static u32 native_last_pc __attribute__((aligned(4))) = 0;
static u32 native_loop_count __attribute__((aligned(4))) = 0;
#define NATIVE_EXEC_LOG_LIMIT 100        /* Log first N unique PC transitions */
#endif

/* Scan for and skip useless code in PS1 executable: */
#define USE_CODE_DISCARD

/* If HLE emulated BIOS is not in use, blocks return to dispatch loop directly */
#define USE_DIRECT_BLOCK_RETURN_JUMPS

/* If a block jumps backwards to the top of itself, use fast dispatch path.
 *  Every block's recompiled start address is saved before entry. To
 *  return to the dispatch loop, it jumps to a shorter version of dispatch
 *  loop that skips code table lookups and code invalidation checks.
 * This could *theoretically* cause a problem  with poorly-behaved
 *  self-modifying code, but so far no issues have been found.
 * NOTE: Option only has effect if USE_DIRECT_BLOCK_RETURN_JUMPS is enabled,
 *  which itself only has effect when HLE emulated BIOS is not in use.
 */
#define USE_DIRECT_FASTPATH_BLOCK_RETURN_JUMPS

/* Const propagation is applied to addresses */
#define USE_CONST_ADDRESSES

/* Const propagation is extended to optimize 'fuzzy' non-const addresses */
#define USE_CONST_FUZZY_ADDRESSES

/* Generate inline memory access or call psxMemRead/Write C functions */
#define USE_DIRECT_MEM_ACCESS

/* Virtual memory mapping options: */
#if defined(SHMEM_MIRRORING) || defined(TMPFS_MIRRORING)
	/* 2MB of PSX RAM (psxM) is now mapped+mirrored virtually, much like
	 *  a real PS1. We can skip mirror-region checks, the 'Einhander' game
	 *  fix. We can skip masking RAM addresses. We also map 0x1fxx_xxxx
	 *  regions (psxP,psxH) into this space, inlining scratchpad accesses.
	 *
	 * IMPORTANT: Don't enable if 'USE_DIRECT_MEM_ACCESS' isn't also enabled.
	 *            There'd be no benefit to a virtual mapping: all mem access
	 *            would be through indirect psxMemWrite/psxMemRead C funcs.
	 *            Even worse, we support a mapping starting at address 0, and
	 *            that is not compatible with psxMemWrite/psxMemRead funcs
	 *            because of how they handle NULL addresses in their LUTs.
	 */
	#ifdef USE_DIRECT_MEM_ACCESS
		#define USE_VIRTUAL_PSXMEM_MAPPING
	#else
		#warning "USE_DIRECT_MEM_ACCESS is undefined! Dynarec will emit slower C memory accesses."
	#endif

	/* Prefer virtually mapped/mirrored code block pointer array over using
	 *  psxRecLUT[]. This removes a layer of indirection from PC->block-ptr lookups,
	 *  allows faster block dispatch loops, and reduces cache/TLB pressure.
	 */
	#define USE_VIRTUAL_RECRAM_MAPPING

#else
	#warning "Neither SHMEM_MIRRORING nor TMPFS_MIRRORING are defined! Dynarec will use slower block dispatch loop and emit slower memory accesses. Check your Makefile!"
#endif // defined(SHMEM_MIRRORING) || defined(TMPFS_MIRRORING)

//#define WITH_DISASM
//#define DEBUGG printf

#include "mem_mapping.h"

/* Bit vector indicating which PS1 RAM pages contain the start of blocks.
 *  Used to determine when code invalidation in recClear() can be skipped.
 */
static u8 code_pages[0x200000/4096/8];

/* Pointers to the recompiled blocks go here. psxRecLUT[] uses upper 16 bits of
 *  a PC value as an index to lookup a block pointer stored in recRAM/recROM.
 */
static s8 *recRAM;
static s8 *recROM;
static uptr psxRecLUT[0x10000];

#undef PC_REC
#undef PC_REC8
#undef PC_REC16
#undef PC_REC32
#define PC_REC(x)	((uptr)psxRecLUT[(x) >> 16] + (((x) & 0xffff) * (REC_RAM_PTR_SIZE / 4)))
#define PC_REC8(x)	(*(u8 *)PC_REC(x))
#define PC_REC16(x)	(*(u16*)PC_REC(x))
#define PC_REC32(x)	(*(u32*)PC_REC(x))
/* Version of PC_REC() that uses faster virtual block ptr mapping */
#define PC_REC_MMAP(x)	(REC_RAM_VADDR | (((x) & 0x00ffffff) * (REC_RAM_PTR_SIZE / 4)))

#include "mips_codegen.h"
#include "disasm.h"
#include "host_asm.h"


/* Const-propagation data and functions */
typedef struct {
	u32  constval;
	bool is_const;

	bool is_fuzzy_ram_addr;        /* GPR is not known-const, but at least known
	                                   to be address somewhere in RAM? */
	bool is_fuzzy_nonram_addr;     /* GPR is not known-const, but at least known
	                                   to be address somewhere outside RAM? */
	bool is_fuzzy_scratchpad_addr; /* GPR is not known-const, but at least known
	                                   to be address somewhere in 1KB scratcpad? */
} iRegisters;
static iRegisters iRegs[32];
static inline void ResetConsts()
{
	memset(&iRegs, 0, sizeof(iRegs));
	iRegs[0].is_const = true;  // $r0 is always zero val
}
static inline bool IsConst(const u32 reg)  { return iRegs[reg].is_const; }
static inline u32  GetConst(const u32 reg) { return iRegs[reg].constval; }
static inline void SetUndef(const u32 reg)
{
	if (reg) {
		iRegs[reg].is_const = false;
		iRegs[reg].is_fuzzy_ram_addr        = false;
		iRegs[reg].is_fuzzy_nonram_addr     = false;
		iRegs[reg].is_fuzzy_scratchpad_addr = false;
	}
}
static inline void SetConst(const u32 reg, const u32 val)
{
	if (reg) {
		iRegs[reg].constval = val;
		iRegs[reg].is_const = true;
		iRegs[reg].is_fuzzy_ram_addr        = false;
		iRegs[reg].is_fuzzy_nonram_addr     = false;
		iRegs[reg].is_fuzzy_scratchpad_addr = false;
	}
}
static inline void SetFuzzyRamAddr(const u32 reg)        { iRegs[reg].is_fuzzy_ram_addr = true; }
static inline bool IsFuzzyRamAddr(const u32 reg)         { return iRegs[reg].is_fuzzy_ram_addr; }
static inline void SetFuzzyNonramAddr(const u32 reg)     { iRegs[reg].is_fuzzy_nonram_addr = true; }
static inline bool IsFuzzyNonramAddr(const u32 reg)      { return iRegs[reg].is_fuzzy_nonram_addr; }
static inline void SetFuzzyScratchpadAddr(const u32 reg) { iRegs[reg].is_fuzzy_scratchpad_addr = true; }
static inline bool IsFuzzyScratchpadAddr(const u32 reg)  { return iRegs[reg].is_fuzzy_scratchpad_addr; }


/* Code cache buffer
 *  Keep this statically allocated! This keeps it close to the
 *  .text section, so C code and recompiled code lie in the same 256MiB region.
 *  MIPS JAL/J opcodes in recompiled code that jump to C code require this.
 *  Dynamic allocation would get an anonymous mmap'ing, locating the recompiled
 *  code *far* too high in virtual address space.
 */
/* SF2000: Increased to 8MB - device has 128MB RAM, larger cache = less stutter */
#ifdef SF2000
#define RECMEM_SIZE         (8 * 1024 * 1024)
#else
#define RECMEM_SIZE         (12 * 1024 * 1024)
#endif
#define RECMEM_SIZE_MAX     (RECMEM_SIZE-(256*1024))
static u8 recMemBase[RECMEM_SIZE] __attribute__((aligned(4)));

u32        *recMem;                /* Where does next emitted opcode in block go? */
static u32 *recMemStart;           /* Where did first emitted opcode in block go? */
static u32 pc;                     /* Recompiler pc */
static u32 oldpc;                  /* Recompiler pc at start of block */
u32 cycle_multiplier = 0x200;      /* Cycle advance per emulated instruction
                                      Default is 0x200 == 2.00 (24.8 fixed-pt) */

/* See comments in recExecute() regarding direct block returns */
static uptr block_ret_addr;                /* Non-zero when blocks are using direct return jumps */
static uptr block_fast_ret_addr;           /* Non-zero when blocks are using direct return jumps &
                                              dispatch loop fastpath is enabled */

static bool psx_mem_mapped;                /* PS1 RAM mmap'd+mirrored at fixed address? (psxM) */
static bool rec_mem_mapped;                /* Code ptr arrays mmap'd+mirrored at fixed address? (recRAM,recROM) */

/* Flags used during a recompilation phase */
static bool branch;                        /* Current instruction lies in a BD slot? */
static bool end_block;                     /* Has recompilation phase ended? */
static bool skip_emitting_next_mflo;       /* Was a MULT/MULTU converted to 3-op MUL? See rec_mdu.cpp.h */
static bool emit_code_invalidations;       /* Emit code invalidation for store instructions? */

/* v063: External SMC check disable flag (from libretro config) */
extern int qpsx_nosmccheck;
/* v112: Cycle batching level (0=OFF, 1=2x, 2=4x, 3=8x) */
extern int g_opt_cycle_batch;
/* v113: Hot Block Cache level (0=OFF, 1-7 = 256 to 1M entries) */
extern int g_opt_block_cache;
/* v114: Native Mode (0=OFF, 1=STATS, 2=FAST) */
extern int g_opt_native_mode;

/***************************************************************************
 * v113: Hot Block Cache - Fast dispatch cache for frequently executed blocks
 *
 * Structure: Direct-mapped cache indexed by hash of PC
 * Entry: 8 bytes (4 byte PC + 4 byte code pointer)
 * Hash: (PC >> 2) & mask
 *
 * Size levels (entries): 0=OFF, 1=256, 2=1K, 3=4K, 4=16K, 5=64K, 6=256K, 7=1M
 * Memory usage: entries * 8 bytes = 2KB to 8MB
 ***************************************************************************/
typedef struct {
    u32 pc;         /* PSX PC address */
    void *code;     /* Pointer to compiled code */
} HotBlockEntry;

/* v151: Aligned for SF2000 stability */
static HotBlockEntry *hot_block_cache __attribute__((aligned(4))) = NULL;
static u32 hot_block_mask __attribute__((aligned(4))) = 0;          /* (entries - 1) for hash masking */
static int hot_block_level __attribute__((aligned(4))) = 0;         /* Current cache level */

/* Size table: level -> entry count (must be power of 2) */
static const u32 hbc_sizes[8] = {
    0,          /* 0: OFF */
    256,        /* 1: 256 entries = 2KB */
    1024,       /* 2: 1K entries = 8KB */
    4096,       /* 3: 4K entries = 32KB */
    16384,      /* 4: 16K entries = 128KB */
    65536,      /* 5: 64K entries = 512KB */
    262144,     /* 6: 256K entries = 2MB */
    1048576     /* 7: 1M entries = 8MB */
};

/* Initialize/resize Hot Block Cache based on g_opt_block_cache */
static void hbc_init(void)
{
    int level = g_opt_block_cache;
    if (level < 0 || level > 7) level = 0;

    /* No change needed? */
    if (level == hot_block_level && hot_block_cache != NULL)
        return;

    /* Free old cache */
    if (hot_block_cache != NULL) {
        free(hot_block_cache);
        hot_block_cache = NULL;
        hot_block_mask = 0;
    }

    hot_block_level = level;

    /* OFF? */
    if (level == 0)
        return;

    /* Allocate new cache */
    u32 entries = hbc_sizes[level];
    hot_block_cache = (HotBlockEntry*)calloc(entries, sizeof(HotBlockEntry));
    if (hot_block_cache == NULL) {
        REC_LOG("HBC: Failed to allocate %u entries!\n", entries);
        hot_block_level = 0;
        return;
    }

    hot_block_mask = entries - 1;
    REC_LOG("HBC: Allocated %u entries (%u KB)\n", entries, (entries * 8) / 1024);
}

/* Clear Hot Block Cache (call on code cache flush) */
static void hbc_clear(void)
{
    if (hot_block_cache != NULL && hot_block_mask > 0) {
        memset(hot_block_cache, 0, (hot_block_mask + 1) * sizeof(HotBlockEntry));
    }
}

/* Inline lookup - returns code ptr or NULL if miss */
static inline void* hbc_lookup(u32 pc)
{
    if (hot_block_cache == NULL) return NULL;
    u32 idx = (pc >> 2) & hot_block_mask;
    if (hot_block_cache[idx].pc == pc)
        return hot_block_cache[idx].code;
    return NULL;
}

/* Inline store - update cache entry */
static inline void hbc_store(u32 pc, void *code)
{
    if (hot_block_cache == NULL) return;
    u32 idx = (pc >> 2) & hot_block_mask;
    hot_block_cache[idx].pc = pc;
    hot_block_cache[idx].code = code;
}

/***************************************************************************
 * v114: Native Mode - MIPS I to MIPS32 direct execution statistics
 *
 * PSX (R3000A) uses MIPS I instruction set which is 100% binary compatible
 * with SF2000's MIPS32. The dynarec ALREADY emits identical instructions!
 * The overhead is in register management, not instruction encoding.
 *
 * Native-capable instructions (run identically on both CPUs):
 *   - ALU R-type: ADD, ADDU, SUB, SUBU, AND, OR, XOR, NOR, SLT, SLTU
 *   - Shifts: SLL, SRL, SRA, SLLV, SRLV, SRAV
 *   - Immediate: ADDIU, ADDI, ANDI, ORI, XORI, SLTI, SLTIU, LUI
 *   - Multiply/Divide: MULT, MULTU, DIV, DIVU, MFHI, MFLO, MTHI, MTLO
 *   - Branches: BEQ, BNE, BGTZ, BLEZ, BLTZ, BGEZ, BLTZAL, BGEZAL
 *   - Jumps: J, JAL, JR, JALR
 *
 * Non-native instructions (need emulation/translation):
 *   - Load/Store: LB, LBU, LH, LHU, LW, LWL, LWR, SB, SH, SW, SWL, SWR
 *     (PSX addresses ≠ host addresses, no MMU for translation)
 *   - Coprocessors: COP0 (system), COP2 (GTE) - not available on host
 *   - SYSCALL, BREAK - trigger exceptions
 *
 * STATS mode: Counts native vs emulated instructions (for analysis)
 * FAST mode: Skips const propagation overhead for simple ALU operations
 * SEMI mode: v115 - POPS-inspired semi-native execution (skips ALL const prop)
 ***************************************************************************/

/* Native mode statistics (visible to profiler) */
typedef struct {
    u32 native_alu;         /* ALU R-type (ADD, SUB, AND, OR, etc.) */
    u32 native_shift;       /* Shifts (SLL, SRL, SRA) */
    u32 native_imm;         /* Immediate ops (ADDIU, ANDI, ORI, etc.) */
    u32 native_muldiv;      /* Multiply/Divide */
    u32 native_branch;      /* Branches (BEQ, BNE, etc.) */
    u32 native_jump;        /* Jumps (J, JAL, JR) */
    u32 emulated_load;      /* Load instructions (need addr translation) */
    u32 emulated_store;     /* Store instructions (need addr translation) */
    u32 emulated_cop0;      /* COP0 instructions */
    u32 emulated_cop2;      /* COP2/GTE instructions */
    u32 emulated_syscall;   /* SYSCALL/BREAK */
    u32 emulated_other;     /* Other non-native */
    /* v115: Block classification stats (for SEMI mode analysis) */
    u32 blocks_total;       /* Total blocks compiled */
    u32 blocks_pure_alu;    /* "Pure" blocks (no load/store/COP) - could run semi-natively */
    /* v118: Native execution stats */
    u32 blocks_native;      /* Blocks actually compiled with native translator */
} NativeModeStats;

/* v151: Aligned for SF2000 stability */
static NativeModeStats native_stats __attribute__((aligned(4)));
static NativeModeStats native_stats_prev __attribute__((aligned(4))); /* For delta calculation */

/* Get pointer to stats (for profiler display) */
extern "C" const NativeModeStats* native_mode_get_stats(void)
{
    return &native_stats;
}

/* v117: Stats are now CUMULATIVE - show totals since game start, not per-frame.
 * This is because compilation only happens once per code block. After initial
 * compilation, blocks run from cache and no new stats are generated.
 * Cumulative stats show the actual composition of compiled code.
 */
extern "C" void native_stats_reset_frame(void)
{
    /* v117: No longer reset stats - they're cumulative now */
    (void)native_stats_prev; /* Unused but kept for potential future use */
}

/* Get native instruction ratio (0-100%) - CUMULATIVE since game start */
extern "C" int native_mode_get_ratio(void)
{
    /* v117: Use native_stats directly (cumulative), not native_stats_prev */
    u32 native = native_stats.native_alu + native_stats.native_shift +
                 native_stats.native_imm + native_stats.native_muldiv +
                 native_stats.native_branch + native_stats.native_jump;
    u32 emulated = native_stats.emulated_load + native_stats.emulated_store +
                   native_stats.emulated_cop0 + native_stats.emulated_cop2 +
                   native_stats.emulated_syscall + native_stats.emulated_other;
    u32 total = native + emulated;
    if (total == 0) return 0;
    return (native * 100) / total;
}

/* v115: Get pure block ratio (0-100%) - blocks that could run semi-natively */
extern "C" int native_mode_get_pure_block_ratio(void)
{
    /* v117: Use native_stats directly (cumulative) */
    if (native_stats.blocks_total == 0) return 0;
    return (native_stats.blocks_pure_alu * 100) / native_stats.blocks_total;
}

/* Inline stat tracking macros (only active in STATS mode) */
#define NATIVE_STAT_ALU()       do { if (g_opt_native_mode == 1) native_stats.native_alu++; } while(0)
#define NATIVE_STAT_SHIFT()     do { if (g_opt_native_mode == 1) native_stats.native_shift++; } while(0)
#define NATIVE_STAT_IMM()       do { if (g_opt_native_mode == 1) native_stats.native_imm++; } while(0)
#define NATIVE_STAT_MULDIV()    do { if (g_opt_native_mode == 1) native_stats.native_muldiv++; } while(0)
#define NATIVE_STAT_BRANCH()    do { if (g_opt_native_mode == 1) native_stats.native_branch++; } while(0)
#define NATIVE_STAT_JUMP()      do { if (g_opt_native_mode == 1) native_stats.native_jump++; } while(0)
#define NATIVE_STAT_LOAD()      do { if (g_opt_native_mode == 1) native_stats.emulated_load++; } while(0)
#define NATIVE_STAT_STORE()     do { if (g_opt_native_mode == 1) native_stats.emulated_store++; } while(0)
#define NATIVE_STAT_COP0()      do { if (g_opt_native_mode == 1) native_stats.emulated_cop0++; } while(0)
#define NATIVE_STAT_COP2()      do { if (g_opt_native_mode == 1) native_stats.emulated_cop2++; } while(0)
#define NATIVE_STAT_SYSCALL()   do { if (g_opt_native_mode == 1) native_stats.emulated_syscall++; } while(0)
#define NATIVE_STAT_OTHER()     do { if (g_opt_native_mode == 1) native_stats.emulated_other++; } while(0)

static bool flush_code_on_dma3_exe_load;   /* Flush code cache when psxDma3() detects EXE load? */

/* Flags/vals used to cache common values in temp regs in emitted code */
static bool lsu_tmp_cache_valid;           /* LSU vals are cached in $at,$v1. See rec_lsu.cpp.h */
static bool host_v0_reg_is_const;          /* PCs are cached in $v0. See rec_bcu.cpp.h */
static u32  host_v0_reg_constval;
static bool host_ra_reg_has_block_retaddr; /* Indirect-return address is cached in $ra. */


#ifdef WITH_DISASM
char	disasm_buffer[512];
#endif

#include "regcache.h"

static void recReset();
static void recRecompile();
/* v101: recClear needs C linkage for psxmem_asm.S */
extern "C" void recClear(u32 Addr, u32 Size);
static void recNotify(int note, void *data);

extern void (*recBSC[64])();
extern void (*recSPC[64])();
extern void (*recREG[32])();
extern void (*recCP0[32])();
extern void (*recCP2[64])();
extern void (*recCP2BSC[32])();


#ifdef WITH_DISASM

#define make_stub_label(name) \
 { (void *)name, (char*)#name }

disasm_label stub_labels[] =
{
  make_stub_label(gteMFC2),
  make_stub_label(gteMTC2),
  make_stub_label(gteLWC2),
  make_stub_label(gteSWC2),
  make_stub_label(gteRTPS),
  make_stub_label(gteOP),
  make_stub_label(gteNCLIP),
  make_stub_label(gteDPCS),
  make_stub_label(gteINTPL),
  make_stub_label(gteMVMVA),
  make_stub_label(gteNCDS),
  make_stub_label(gteNCDT),
  make_stub_label(gteCDP),
  make_stub_label(gteNCCS),
  make_stub_label(gteCC),
  make_stub_label(gteNCS),
  make_stub_label(gteNCT),
  make_stub_label(gteSQR),
  make_stub_label(gteDCPL),
  make_stub_label(gteDPCT),
  make_stub_label(gteAVSZ3),
  make_stub_label(gteAVSZ4),
  make_stub_label(gteRTPT),
  make_stub_label(gteGPF),
  make_stub_label(gteGPL),
  make_stub_label(gteNCCT),
  make_stub_label(psxMemRead8),
  make_stub_label(psxMemRead16),
  make_stub_label(psxMemRead32),
  make_stub_label(psxMemWrite8),
  make_stub_label(psxMemWrite16),
  make_stub_label(psxMemWrite32),
  make_stub_label(psxHwRead8),
  make_stub_label(psxHwRead16),
  make_stub_label(psxHwRead32),
  make_stub_label(psxHwWrite8),
  make_stub_label(psxHwWrite16),
  make_stub_label(psxHwWrite32),
  make_stub_label(psxException),
  // Direct HW I/O:
  make_stub_label(cdrRead0),
  make_stub_label(cdrRead1),
  make_stub_label(cdrRead2),
  make_stub_label(cdrRead3),
  make_stub_label(cdrWrite0),
  make_stub_label(cdrWrite1),
  make_stub_label(cdrWrite2),
  make_stub_label(cdrWrite3),
  make_stub_label(GPU_writeData),
  make_stub_label(mdecRead0),
  make_stub_label(mdecRead1),
  make_stub_label(mdecWrite0),
  make_stub_label(mdecWrite1),
  make_stub_label(psxRcntRcount),
  make_stub_label(psxRcntRmode),
  make_stub_label(psxRcntRtarget),
  make_stub_label(psxRcntWcount),
  make_stub_label(psxRcntWmode),
  make_stub_label(psxRcntWtarget),
  make_stub_label(sioRead8),
  make_stub_label(sioRead16),
  make_stub_label(sioRead32),
  make_stub_label(sioReadBaud16),
  make_stub_label(sioReadCtrl16),
  make_stub_label(sioReadMode16),
  make_stub_label(sioReadStat16),
  make_stub_label(sioWrite8),
  make_stub_label(sioWrite16),
  make_stub_label(sioWrite32),
  make_stub_label(sioWriteBaud16),
  make_stub_label(sioWriteCtrl16),
  make_stub_label(sioWriteMode16),
  make_stub_label(SPU_writeRegister),
};

const u32 num_stub_labels = sizeof(stub_labels) / sizeof(disasm_label);

#define DISASM_INIT() \
do { \
	printf("Block PC %x (MIPS) -> %p\n", pc, recMemStart); \
} while (0)

#define DISASM_PSX(_PC_) \
do { \
	u32 opcode = *(u32 *)((char *)PSXM(_PC_)); \
	disasm_mips_instruction(opcode, disasm_buffer, _PC_, 0, 0); \
	printf("%08x: %08x %s\n", _PC_, opcode, disasm_buffer); \
} while (0)

#define DISASM_HOST() \
do { \
	printf("\n"); \
	u8 *tr_ptr = (u8*)recMemStart; \
	for (; (u32)tr_ptr < (u32)recMem; tr_ptr += 4) { \
		u32 opcode = *(u32*)tr_ptr; \
		disasm_mips_instruction(opcode, disasm_buffer, \
					(u32)tr_ptr, stub_labels, \
					num_stub_labels); \
		printf("%08x: %s\t(0x%08x)\n", \
			(u32)tr_ptr, disasm_buffer, opcode); \
	} \
	printf("\n"); \
} while (0)

#define DISASM_MSG printf

#else

#define DISASM_PSX(_PC_)
#define DISASM_HOST()
#define DISASM_INIT()
#define DISASM_MSG(...)

#endif


#include "opcodes.h"

#if !defined(HAVE_MIPS32R2_CACHE_OPS) && !defined(SF2000)
#include <sys/cachectl.h>
#endif

static inline void clear_insn_cache(void *start, void *end, int flags)
{
#ifdef HAVE_MIPS32R2_CACHE_OPS
	// MIPS32r2 added fine-grained usermode cache flush ability (yes, please!)
	MIPS32R2_MakeCodeVisible(start, (char *)end - (char *)start);
#elif defined(SF2000)
	// SF2000 bare metal: use GCC builtin which calls _flush_cache() provided
	// by the multicore framework. This uses MIPS cache instructions directly
	// (Index_Writeback_Inv_D + Index_Invalidate_I) since SF2000's MIPS32
	// doesn't have the synci instruction from MIPS32r2.
	__builtin___clear_cache((char *)start, (char *)end);
	/* v149: HEISENBUG FIX - add full memory barrier after cache flush
	 * Ensures all cache operations complete before CPU fetches new code.
	 * This fixes the timing-dependent crash found in v148.
	 */
	__sync_synchronize();
#else
	// Use Linux system call (ends up flushing entire cache)
	#ifdef DYNAREC_SKIP_DCACHE_FLUSH
		// Faster, but only works if host's ICACHE pulls from DCACHE, not RAM
		int cache_to_flush = ICACHE;
	#else
		// Slower, but most compatible (flush both caches)
		int cache_to_flush = BCACHE; // ICACHE|DCACHE
	#endif

	cacheflush(start, (char *)end - (char *)start, cache_to_flush);
#endif
}


/* Set default recompilation options, and any per-game settings */
static void rec_set_options()
{
	// Default options
	emit_code_invalidations = true;
	flush_code_on_dma3_exe_load = false;

	// v063: Check for user-requested SMC check disable
	if (qpsx_nosmccheck) {
		REC_LOG("SMC checks disabled by user config (nosmccheck=1)\n");
		emit_code_invalidations = false;
	}

	// Per-game options
	// -> Use case-insensitive comparisons! Some CDs have lowercase CdromId.

	// 'Studio 33' game workarounds (other Studio 33 games seem to be OK)
	//  See comments in recNotify(), psxDma3().
	if (strncasecmp(CdromId, "SCES03886", 9) == 0  ||  // Formula 1 Arcade
	    strncasecmp(CdromId, "SLUS00870", 9) == 0  ||  // Formula 1 '99  NTSC US
	    strncasecmp(CdromId, "SCPS10101", 9) == 0  ||  // Formula 1 '99  NTSC J (untested)
	    strncasecmp(CdromId, "SCES01979", 9) == 0  ||  // Formula 1 '99  PAL  E (requires .SBI subchannel file)
	    strncasecmp(CdromId, "SLES01979", 9) == 0  ||  // Formula 1 '99  PAL  E (unknown revision, couldn't test)
	    strncasecmp(CdromId, "SCES03404", 9) == 0  ||  // Formula 1 2001 PAL  E,Fi (fixes broken AI/controls)
	    strncasecmp(CdromId, "SCES03423", 9) == 0)     // Formula 1 2001 PAL  Fr,G (fixes broken AI/controls)
	{
		REC_LOG("Using Icache workarounds for trouble games 'Formula One 99/2001/etc'.\n");
		emit_code_invalidations = false;
		flush_code_on_dma3_exe_load = true;
	}
}


static void recRecompile()
{
	PROFILE_START(PROF_CPU_COMPILE);

	// Notify plugin_lib that we're recompiling (affects frameskip timing)
	pl_dynarec_notify();

	if (((uptr)recMem - (uptr)recMemBase) >= RECMEM_SIZE_MAX ) {
		REC_LOG("Code cache size limit exceeded: flushing code cache.\n");
		recReset();
	}

	recMemStart = recMem;

	regReset();

	PC_REC32(psxRegs.pc) = (u32)recMem;
	oldpc = pc = psxRegs.pc;

	// If 'pc' is in PS1 RAM, mark the page of RAM as containing the start of
	//  a block. For the range check, bit 27 is interpreted as a sign bit.
	if ((s32)(pc << 4) >= 0) {
		u32 masked_pc = pc & 0x1fffff;
		code_pages[masked_pc/4096/8] |= (1 << ((masked_pc/4096) & 7));
	}

	DISASM_INIT();

	rec_recompile_start();

	// Reset const-propagation
	ResetConsts();

	// Flag indicates when recompilation should stop
	end_block = false;

	// See convertMultiplyTo3Op() and recMFLO() in rec_mdu.cpp.h
	skip_emitting_next_mflo = false;

	// Flag indicates when values are cached by load/store emitters in $at,$v1
	lsu_tmp_cache_valid = false;

	// Flag indicates when a PC value is cached in $v0. All dispatch loops set
	//  $v0 to block start PC before entry. See rec_bcu.cpp.h
	host_v0_reg_is_const = true;
	host_v0_reg_constval = psxRegs.pc;

	// Flag indicates when $ra holds block return address. This is only used
	//  by blocks returning indirectly. The indirect-return dispatch loops
	//  set $ra before block entry. See rec_recompile_end_part1().
	host_ra_reg_has_block_retaddr = (block_ret_addr == 0);

	// Number of discardable instructions we are currently skipping
	int discard_cnt = 0;

	/* v115: Block classification for SEMI mode
	 * Track if this block is "pure" (only native-capable instructions)
	 * Pure blocks could potentially run semi-natively like POPS
	 */
	bool block_is_pure = true;  /* Assume pure until we hit a non-native instruction */

	/* v118: Native block compilation (SEMI mode level 3)
	 * Try to compile the block natively - bypasses normal dynarec entirely.
	 * Native translator does its own analysis and handles:
	 * - ALU/shift/branch/jump: byte-for-byte copy
	 * - Load/store: address translation + original op
	 * - Returns false if block uses forbidden registers or COP instructions
	 */
	bool native_compiled = false;

	/* v155: BLOCK DIAGNOSTIC LOGGING - log first 20 blocks */
	static int block_log_count = 0;

	/* v218: Save start PC for native compilation (needed in ALL modes now) */
	u32 block_start_pc = pc;
	u32* recMemBlockStart = recMem;  /* Save recMem position before dynarec */

#if DIFF_TEST_ENABLE
	/* v195: DIFF_TEST - save start PC for later native compilation */
	u32 diff_test_start_pc = pc;
#endif

	/* v218: REMOVED - don't try native first with required_count=0!
	 * Native needs dynarec's instruction count to work correctly.
	 * Dynarec will run first, then we try native with exact count.
	 */

	do {
		// Flag indicates if next instruction lies in a BD slot
		branch = false;

		psxRegs.code = OPCODE_AT(pc);

#ifdef USE_CODE_DISCARD
		// If we are not already skipping past discardable code, scan
		//  for PS1 code sequence we can discard.
		if (discard_cnt == 0) {
			int discard_type = 0;
			discard_cnt = rec_discard_scan(pc, &discard_type);
			if (discard_cnt > 0)
				DISASM_MSG(" ->BEGIN code discard: %s\n", rec_discard_type_str(discard_type));
		}
#endif

		DISASM_PSX(pc);
		pc += 4;

#ifdef USE_CODE_DISCARD
		// Skip next discardable instruction in any sequence found.
		if (discard_cnt > 0) {
			--discard_cnt;
			if (discard_cnt == 0)
				DISASM_MSG(" ->END code discard.\n");
			continue;
		}
#endif

		// Recompile next instruction.

		/* v117: Native Mode - track instruction categories & block purity
		 * FIXED: Now tracks stats in ALL native modes, not just STATS mode!
		 * Stats are cumulative (not reset per frame) to show meaningful numbers.
		 */
		if (g_opt_native_mode >= 1) {
			u32 op = psxRegs.code >> 26;
			u32 funct = psxRegs.code & 0x3f;
			bool is_native = false;  /* Does this instruction affect block purity? */

			switch (op) {
				case 0x00: /* SPECIAL */
					switch (funct) {
						/* Native ALU R-type */
						case 0x20: case 0x21: /* ADD, ADDU */
						case 0x22: case 0x23: /* SUB, SUBU */
						case 0x24: case 0x25: /* AND, OR */
						case 0x26: case 0x27: /* XOR, NOR */
						case 0x2A: case 0x2B: /* SLT, SLTU */
							native_stats.native_alu++;
							is_native = true;
							break;
						/* Native shifts */
						case 0x00: case 0x02: case 0x03: /* SLL, SRL, SRA */
						case 0x04: case 0x06: case 0x07: /* SLLV, SRLV, SRAV */
							native_stats.native_shift++;
							is_native = true;
							break;
						/* Native multiply/divide */
						case 0x18: case 0x19: /* MULT, MULTU */
						case 0x1A: case 0x1B: /* DIV, DIVU */
						case 0x10: case 0x11: /* MFHI, MTHI */
						case 0x12: case 0x13: /* MFLO, MTLO */
							native_stats.native_muldiv++;
							is_native = true;
							break;
						/* Native jumps */
						case 0x08: case 0x09: /* JR, JALR */
							native_stats.native_jump++;
							is_native = true;
							break;
						/* SYSCALL, BREAK - need emulation */
						case 0x0C: case 0x0D:
							native_stats.emulated_syscall++;
							block_is_pure = false;
							break;
						default:
							native_stats.emulated_other++;
							block_is_pure = false;
							break;
					}
					break;
				case 0x01: /* REGIMM - branches */
					native_stats.native_branch++;
					is_native = true;
					break;
				case 0x02: case 0x03: /* J, JAL */
					native_stats.native_jump++;
					is_native = true;
					break;
				case 0x04: case 0x05: /* BEQ, BNE */
				case 0x06: case 0x07: /* BLEZ, BGTZ */
					native_stats.native_branch++;
					is_native = true;
					break;
				/* Native immediate ops */
				case 0x08: case 0x09: /* ADDI, ADDIU */
				case 0x0A: case 0x0B: /* SLTI, SLTIU */
				case 0x0C: case 0x0D: /* ANDI, ORI */
				case 0x0E: case 0x0F: /* XORI, LUI */
					native_stats.native_imm++;
					is_native = true;
					break;
				/* COP0 - need emulation */
				case 0x10:
					native_stats.emulated_cop0++;
					block_is_pure = false;
					break;
				/* COP2/GTE - need emulation */
				case 0x12:
					native_stats.emulated_cop2++;
					block_is_pure = false;
					break;
				/* Load instructions - need address translation */
				case 0x20: case 0x21: case 0x22: case 0x23: /* LB, LH, LWL, LW */
				case 0x24: case 0x25: case 0x26: /* LBU, LHU, LWR */
				case 0x32: /* LWC2 */
					native_stats.emulated_load++;
					block_is_pure = false;
					break;
				/* Store instructions - need address translation */
				case 0x28: case 0x29: case 0x2A: case 0x2B: /* SB, SH, SWL, SW */
				case 0x2E: /* SWR */
				case 0x3A: /* SWC2 */
					native_stats.emulated_store++;
					block_is_pure = false;
					break;
				default:
					native_stats.emulated_other++;
					block_is_pure = false;
					break;
			}
			(void)is_native; /* Suppress unused warning - used for future direct exec */
		}

		recBSC[psxRegs.code>>26]();
		regUpdate();
	} while (!end_block);

	/* v236: MODE 3 = NATIVE EXECUTION
	 * Try native compilation, use native if successful, fallback to dynarec
	 */
	if (g_opt_native_mode == 3) {
		/* v245: Skip native if all commands disabled in menu */
		extern int native_any_cmd_enabled;
		if (!native_any_cmd_enabled) {
			dynarec_used++;
			goto native_block_done;
		}

		/* Calculate dynarec instruction count */
		int dynarec_instr_count = (pc - block_start_pc) / 4;

		/* v242: Limit instruction count to prevent overflow
		 * Each instruction can generate ~50 bytes worst case
		 * 32KB buffer / 50 bytes = ~640 instructions max
		 * Use 500 as safe limit */
		#define MODE3_MAX_INSTRS 500
		if (dynarec_instr_count > MODE3_MAX_INSTRS) {
			/* Block too large for native, use dynarec */
			dynarec_used++;
			goto native_block_done;
		}

		/* Save current recMem position (end of dynarec code) */
		u32* recMemAfterDynarec = recMem;
		u32 dynarec_code_size = ((u8*)recMem - (u8*)recMemBlockStart);
		(void)dynarec_code_size;

		/* v242: Re-init guards before each attempt to ensure clean detection */
		init_overflow_guards();

		/* Try native compilation to temp buffer */
		recMem = (u32*)native_try_buffer;
		u32* native_start = recMem;

		if (native_compile_block(block_start_pc, dynarec_instr_count)) {
			/* Native succeeded! Add epilogue */
			rec_recompile_end_part1();
			rec_recompile_end_part2(false);

			u32 native_code_size = ((u8*)recMem - (u8*)native_start);

			/* v238: Check for overflow BEFORE using the code */
			if (check_native_try_overflow()) {
				xlog("!!! MODE 3: native_try_buffer OVERFLOW! size=%d, max=%d\n",
				     native_code_size, NATIVE_TRY_BUF_SIZE);
				xlog("!!! Block PC=%08X, instr_count=%d\n", block_start_pc, dynarec_instr_count);
				init_overflow_guards();  /* Reset guards */
				recMem = recMemAfterDynarec;
				dynarec_used++;
			}
			/* Copy native code to main buffer (overwrite dynarec) */
			else if (native_code_size <= NATIVE_TRY_BUF_SIZE) {
				memcpy(recMemBlockStart, native_try_buffer, native_code_size);
				recMem = (u32*)((u8*)recMemBlockStart + native_code_size);
				native_compiled = true;
				/* v244: native_used already incremented in native_compile_block() */

#ifdef __mips__
				__builtin___clear_cache((char*)recMemBlockStart, (char*)recMem);
				__sync_synchronize();
#endif
			} else {
				xlog("!!! MODE 3: native too big! size=%d, max=%d\n",
				     native_code_size, NATIVE_TRY_BUF_SIZE);
				recMem = recMemAfterDynarec;
				dynarec_used++;
			}
		} else {
			recMem = recMemAfterDynarec;
			dynarec_used++;
		}
	}

#if DIFF_TEST_ENABLE
	/* v236: MODE 4 = DIFF_TEST
	 * Compile native to separate buffer for comparison during execution
	 */
	if (g_opt_native_mode == 4) {
		/* Calculate dynarec instruction count */
		int dynarec_instr_count = (pc - diff_test_start_pc) / 4;

		u32 lut_idx = pc_to_lut_idx(diff_test_start_pc);

		/* v240: Like v235 - just stop compiling when buffer full, no reset
		 * Check: slot empty AND buffer has space (at least 8KB for safety) */
		if (native_block_lut[lut_idx] == NULL &&
		    (native_code_ptr + 8192) < native_code_end)
		{
			/* Save current recMem position */
			u32* saved_recMem = recMem;

			/* Point recMem to native code buffer */
			recMem = (u32*)native_code_ptr;
			u32* native_start = recMem;

			/* Try native compilation with EXACT instruction count */
			if (native_compile_block(diff_test_start_pc, dynarec_instr_count)) {
				/* Native succeeded! */
				rec_recompile_end_part1();
				rec_recompile_end_part2(false);

				u32 native_code_size = ((u8*)recMem - (u8*)native_start);

				/* v238: Check bounds AFTER compilation */
				if ((u8*)recMem > native_code_end || check_native_code_overflow()) {
					xlog("!!! MODE 4: native_code_buffer OVERFLOW! size=%d\n", native_code_size);
					xlog("!!! recMem=%08X, end=%08X, PC=%08X, instrs=%d\n",
					     (u32)recMem, (u32)native_code_end, diff_test_start_pc, dynarec_instr_count);
					init_native_code_guard();  /* Reset guard */
					/* Don't store in LUT - block is corrupted */
				} else {
					/* Store in lookup table WITH PC for verification */
					native_block_lut[lut_idx] = native_start;
					native_block_pc[lut_idx] = diff_test_start_pc;
					diff_native_compiled++;

					/* Advance native buffer pointer */
					native_code_ptr = (u8*)recMem;

					/* Flush cache for native code */
#ifdef __mips__
					__builtin___clear_cache((char*)native_start, (char*)recMem);
					__sync_synchronize();
#endif
				}
			}

			/* Restore recMem for any later operations */
			recMem = saved_recMem;
		}
	}
#endif

	/* v115: Update block classification stats (for SEMI mode analysis) */
	if (g_opt_native_mode >= 1) {
		native_stats.blocks_total++;
		if (block_is_pure)
			native_stats.blocks_pure_alu++;
	}

native_block_done:
	/* v118: Track if block was natively compiled */
	if (native_compiled) {
		native_stats.blocks_total++;
		native_stats.blocks_pure_alu++;
	}

	DISASM_HOST();
	clear_insn_cache(recMemStart, recMem, 0);

	PROFILE_END(PROF_CPU_COMPILE);
}


static int recInit()
{
	REC_LOG("Initializing\n");

	recMem = (u32*)recMemBase;

	// Init code buffer, to allocate the RAM we need in advance. Filling with
	//  all-1's should force an exception on any accidental non-code execution.
	{
		size_t fill_size = RECMEM_SIZE_MAX + 0x1000;  // 4KB past 'maximum' we'd ever use.
		if (fill_size > RECMEM_SIZE)
			fill_size = RECMEM_SIZE;
		memset(recMemBase, 0xff, fill_size);
	}

	/* v238: Initialize overflow guards */
	init_overflow_guards();
#if DIFF_TEST_ENABLE
	init_native_code_guard();
#endif

	// The tables recRAM and recROM hold block code pointers for all valid PC
	//  values for a PS1 program, after masking away banking and/or mirroring.
	//  Originally, the dynarec used a LUT, psxRecLUT[], to accomplish this
	//  masking, which added a second layer of indirection to code lookups.
	//  Instead, we now virtually map/mirror recRAM and map recROM.
	//  This offloads the indirection onto the host's TLB.
	//  NOTE: psxRecLUT[] still is used inside recExecuteBlock(): no sense in
	//        making two versions of a function that hardly gets any real use.
#ifdef USE_VIRTUAL_RECRAM_MAPPING
	if (!rec_mem_mapped && !recRAM && !recROM) {
		if (rec_mmap_rec_mem() >= 0) {
			rec_mem_mapped = true;
			recRAM = (s8*)REC_RAM_VADDR;
			recROM = (s8*)REC_ROM_VADDR;
		}
	}
#endif

	if (!rec_mem_mapped) {
		// Oops, somehow the virtual mapping/mirroring failed. We'll allocate
		//  recRAM/recROM here and use slower versions of recExecute*() that
		//  use psxRecLUT[] to indirectly access recRAM/recROM.
		recRAM = (s8*)malloc(REC_RAM_SIZE);
		recROM = (s8*)malloc(REC_ROM_SIZE);
		printf("WARNING: Recompiler is using slower non-virtual block ptr lookups.\n");
	}

	recReset();

	if (recRAM == NULL || recROM == NULL || recMemBase == NULL || psxRecLUT == NULL) {
		printf("Error allocating memory\n"); return -1;
	}

	for (int i = 0; i < 0x80; i++)
		psxRecLUT[i + 0x0000] = (uptr)recRAM + (((i & 0x1f) << 16) * (REC_RAM_PTR_SIZE/4));

	memcpy(&psxRecLUT[0x8000], psxRecLUT, 0x80 * sizeof(psxRecLUT[0]));
	memcpy(&psxRecLUT[0xa000], psxRecLUT, 0x80 * sizeof(psxRecLUT[0]));

	for (int i = 0; i < 0x08; i++)
		psxRecLUT[i + 0xbfc0] = (uptr)recROM + ((i << 16) * (REC_RAM_PTR_SIZE/4));

	// Map/mirror PSX RAM, other regions, i.e. psxM, psxP, psxH
	// NOTE: if mapping fails or isn't enabled at compile-time, PSX mem will be
	//       allocated using traditional methods in psxmem.cpp
#ifdef USE_VIRTUAL_PSXMEM_MAPPING
	if (!psx_mem_mapped)
		psx_mem_mapped = (rec_mmap_psx_mem() >= 0);
#endif

	if (!psx_mem_mapped)
		printf("WARNING: Recompiler is emitting slower non-virtual mem access code.\n");

	return 0;
}


static void recShutdown()
{
	REC_LOG("Shutting down\n");

	if (psx_mem_mapped)
		rec_munmap_psx_mem();
	if (rec_mem_mapped)
		rec_munmap_rec_mem();
	psx_mem_mapped = rec_mem_mapped = false;
}


/* It seems there's no way to tell GCC that something is being called inside
 * asm() blocks and GCC doesn't bother to save temporaries to stack.
 * That's why we have two options:
 * 1. Call recompiled blocks via recFunc() trap which is strictly noinline and
 * saves registers $s[0-7], $fp and $ra on each call, or
 * 2. Code recExecute() and recExecuteBlock() entirely in assembler taking into
 * account that no registers except $ra are saved in recompiled blocks and
 * thus put all temporaries to stack. In this case $s[0-7], $fp and $ra are saved
 * in recExecute() and recExecuteBlock() only once.
 *
 * IMPORTANT: Functions containing inline ASM should have attribute 'noinline'.
 *            Crashes at callsites can occur otherwise, at least with GCC 4.xx.
 */
#ifndef ASM_EXECUTE_LOOP
__attribute__((noinline)) static void recFunc(void *fn)
{
	/* This magic code calls fn address saving registers $s[0-7], $fp and $ra. */
	/*                                                                         */
	/* Focus here is on clarity, not speed. This is the bare minimum needed to */
	/*  call blocks from within C code, which is:                              */
	/* Blocks expect $fp to be set to &psxRegs.                                */
	/* Blocks expect return address to be stored at 16($sp) and in $ra.        */
	/* Blocks expect $v0 to contain psxRegs.pc value (helps addr generation).  */
	/* Stack should have 16 bytes free at 0($sp) for use by called functions.  */
	/* Stack should be 8-byte aligned to satisfy MIPS ABI.                     */
	/*                                                                         */
	/* Blocks return these values, which are handled here:                     */
	/*  $v0 is new value for psxRegs.pc                                        */
	/*  $v1 is number of cycles to increment psxRegs.cycle by                  */
	__asm__ __volatile__ (
		"addiu  $sp, $sp, -24                   \n"
		"la     $fp, %[psxRegs]                 \n" // $fp = &psxRegs
		"lw     $v0, %[psxRegs_pc_off]($fp)     \n" // Blocks expect $v0 to contain PC val on entry
		"la     $ra, block_return_addr%=        \n" // Load $ra with block_return_addr
		"sw     $ra, 16($sp)                    \n" // Put 'block_return_addr' on stack
		"jr     %[fn]                           \n" // Execute block

		"block_return_addr%=:                   \n"
		"sw     $v0, %[psxRegs_pc_off]($fp)     \n" // psxRegs.pc = $v0
		"lw     $t1, %[psxRegs_cycle_off]($fp)  \n" //
		"addu   $t1, $t1, $v1                   \n" //
		"sw     $t1, %[psxRegs_cycle_off]($fp)  \n" // psxRegs.cycle += $v1
		"addiu  $sp, $sp, 24                    \n"
		: // Output
		: // Input
		  [fn]                   "r" (fn),
		  [psxRegs]              "i" (&psxRegs),
		  [psxRegs_pc_off]       "i" (off(pc)),   // Offset of psxRegs.pc in psxRegs
		  [psxRegs_cycle_off]    "i" (off(cycle)) // Offset of psxRegs.cycle in psxRegs
		: // Clobber - No need to list anything but 'saved' regs
		  "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7", "fp", "ra", "memory"
	);
}
#endif


/* Execute blocks starting at psxRegs.pc
 * Blocks return indirectly, to address stored at 16($sp)
 * Block pointers are looked up using psxRecLUT[].
 * Called only from recExecute(), see notes there.
 *
 * IMPORTANT: Functions containing inline ASM should have attribute 'noinline'.
 *            Crashes at callsites can occur otherwise, at least with GCC 4.xx.
 */
__attribute__((noinline)) void recExecute_indirect_return_lut()
{
	/* v141: Removed verbose entry logging */

	// Set block_ret_addr to 0, so generated code uses indirect returns
	block_ret_addr = block_fast_ret_addr = 0;

	// QPSX_039: Frame complete flag - must return after VBlank for libretro
	extern volatile int emu_frame_complete;

#ifndef ASM_EXECUTE_LOOP
	emu_frame_complete = 0;

	/* v112: Cycle batching - check less frequently for speed */
	/* batch_mask: 0=every block, 1=every 2, 3=every 4, 7=every 8 */
	const u32 batch_mask = (1 << g_opt_cycle_batch) - 1;  /* 0, 1, 3, 7 */
	u32 batch_counter = 0;

	for (;;) {
		u32 pc = psxRegs.pc;
		void *code;

		/* v113: Hot Block Cache - fast path for cached blocks */
		code = hbc_lookup(pc);
		if (code == NULL) {
			/* Cache miss - normal lookup */
			u32 *p = (u32*)PC_REC(pc);
			if (*p == 0)
				recRecompile();
			code = (void*)*p;
			/* Store in cache for next time */
			hbc_store(pc, code);
		}

#if NATIVE_EXEC_DEBUG
		/* v132: Enhanced logging - track loops even after log limit */
		bool is_new_pc = (pc != native_last_pc);

		if (is_new_pc) {
			/* Log exit from previous loop if there was one */
			if (native_loop_count > 1 && native_exec_log_count < NATIVE_EXEC_LOG_LIMIT) {
				xlog("EXEC: (loop x%d)", native_loop_count);
			}
			native_loop_count = 0;
			if (native_exec_log_count < NATIVE_EXEC_LOG_LIMIT) {
				xlog("EXEC: [%d] >> PC=%08X ptr=%p", native_exec_log_count, pc, code);
			}
		} else {
			/* v132: Periodic logging for infinite loops */
			if (native_loop_count > 0 && (native_loop_count % 50000) == 0) {
				xlog("EXEC: STILL LOOPING PC=%08X count=%u", pc, native_loop_count);
			}
		}
#endif

		recFunc(code);

		/* v139: Cycle counting now works via $v1!
		 * Native blocks set $v1 = count AFTER emit_save_gprs(),
		 * and recFunc's dispatcher adds $v1 to psxRegs.cycle.
		 * No extra handling needed - same as dynarec blocks!
		 */

#if NATIVE_EXEC_DEBUG
		if (is_new_pc && native_exec_log_count < NATIVE_EXEC_LOG_LIMIT) {
			xlog("EXEC: [%d] << PC=%08X -> %08X", native_exec_log_count, pc, psxRegs.pc);
			native_exec_log_count++;
		}
		native_loop_count++;
		native_last_pc = pc;
#endif

		/* v138: FORCE psxBranchTest on EVERY block for debugging!
		 * This ensures interrupts are checked even if native blocks
		 * don't accumulate enough cycles.
		 */
		if (psxRegs.cycle >= psxRegs.io_cycle_counter) {
			/* v192: DISP logging disabled for performance */
			psxBranchTest();
		}

		// QPSX_039: Return after frame for libretro input/video
		if (emu_frame_complete) {
			/* v141: Removed verbose frame return logging */
			emu_frame_complete = 0;
			return;
		}
	}
#else
// QPSX_039: Clear flag BEFORE entering loop - this was the bug in v37!
emu_frame_complete = 0;

__asm__ __volatile__ (
// NOTE: <BD> indicates an instruction in a branch-delay slot
".set push                                    \n"
".set noreorder                               \n"

// $fp/$s8 remains set to &psxRegs across all calls to blocks
"la    $fp, %[psxRegs]                        \n"

// Set up our own stack frame. Should have 8-byte alignment, and have 16 bytes
// empty at 0($sp) for use by functions called from within recompiled code.
".equ  frame_size,                  24        \n"
".equ  f_off_temp_var1,             20        \n"
".equ  f_off_block_ret_addr,        16        \n" // NOTE: blocks assume this is at 16($sp)!
"addiu $sp, $sp, -frame_size                  \n"

// Store const block return address at fixed location in stack frame
"la    $t0, loop%=                            \n"
"sw    $t0, f_off_block_ret_addr($sp)         \n"

// Load $v0 once with psxRegs.pc, blocks will assign new value when returning
"lw    $v0, %[psxRegs_pc_off]($fp)            \n"

// Load $v1 once with zero. It is the # of cycles psxRegs.cycle should be
// incremented by when a block returns.
"move  $v1, $0                                \n"

// Align loop on cache-line boundary
".balign 32                                   \n"

////////////////////////////
//       LOOP CODE:       //
////////////////////////////

// NOTE: Blocks return to top of loop.
// NOTE: Loop expects following values to be set:
// $v0 = new value for psxRegs.pc
// $v1 = # of cycles to increment psxRegs.cycle by

// The loop pseudocode is this, interleaving ops to reduce load stalls:
//
// loop:
// $t2 = psxRecLUT[pxsRegs.pc >> 16] + (psxRegs.pc & 0xffff)
// $t0 = *($t2)
// psxRegs.cycle += $v1
// if (psxRegs.cycle >= psxRegs.io_cycle_counter)
//    goto call_psxBranchTest;
// psxRegs.pc = $v0
// if ($t0 == 0)
//    goto recompile_block;
// $ra = block return address
// goto $t0;
// /* Code at addr $t0 will run and return having set $v0 to new psxRegs.pc */
// /*  value and $v1 to the number of cycles to increment psxRegs.cycle by. */

// Infinite loop, blocks return here
"loop%=:                                      \n"
"lw    $t3, %[psxRegs_cycle_off]($fp)         \n" // $t3 = psxRegs.cycle
"lui   $t1, %%hi(%[psxRecLUT])                \n"
"srl   $t2, $v0, 16                           \n"
"sll   $t2, $t2, 2                            \n" // sizeof() psxRecLUT[] elements is 4
"addu  $t1, $t1, $t2                          \n"
"lw    $t1, %%lo(%[psxRecLUT])($t1)           \n" // $t1 = psxRecLUT[psxRegs.pc >> 16]
"lw    $t4, %[psxRegs_io_cycle_ctr_off]($fp)  \n" // $t4 = psxRegs.io_cycle_counter
"addu  $t3, $t3, $v1                          \n" // $t3 = psxRegs.cycle + $v1
"andi  $t0, $v0, 0xffff                       \n"
"addu  $t2, $t0, $t1                          \n"
"lw    $t0, 0($t2)                            \n" // $t0 = address of start of block code, or
                                                  //       or 0 if block needs recompilation
                                                  // IMPORTANT: leave block ptr in $t2, it gets
                                                  // saved & re-used if recRecompile() is called.


// Must call psxBranchTest() when psxRegs.cycle >= psxRegs.io_cycle_counter
"sltu  $t4, $t3, $t4                          \n"
"beqz  $t4, call_psxBranchTest%=              \n"
"sw    $t3, %[psxRegs_cycle_off]($fp)         \n" // <BD> IMPORTANT: store new psxRegs.cycle val,
                                                  //  whether or not we are branching here

// Recompile block, if necessary
"beqz  $t0, recompile_block%=                 \n"
"sw    $v0, %[psxRegs_pc_off]($fp)            \n" // <BD> Use BD slot to store new psxRegs.pc val

// Execute already-compiled block. It will return at top of loop.
"execute_block%=:                             \n"
"jr    $t0                                    \n"
"lw    $ra, 16($sp)                           \n" // <BD> Load block return address

////////////////////////////
//     NON-LOOP CODE:     //
////////////////////////////

// Call psxBranchTest() and go back to top of loop
"call_psxBranchTest%=:                        \n"
"jal   %[psxBranchTest]                       \n"
"sw    $v0, %[psxRegs_pc_off]($fp)            \n" // <BD> Use BD slot to store new psxRegs.pc val,
                                                  //  as psxBranchTest() might issue an exception.
// QPSX_039: Check emu_frame_complete flag - exit if frame is done
"lui   $t5, %%hi(%[emu_frame_complete])       \n"
"lw    $t6, %%lo(%[emu_frame_complete])($t5)  \n"
"bnez  $t6, exit%=                            \n" // Exit loop if frame complete
"nop                                          \n"
"lw    $v0, %[psxRegs_pc_off]($fp)            \n" // After psxBranchTest() returns, load psxRegs.pc
                                                  //  back into $v0, which could be different than
                                                  //  before the call if an exception was issued.
"b     loop%=                                 \n" // Go back to top to process psxRegs.pc again..
"move  $v1, $0                                \n" // <BD> ..using BD slot to set $v1 to 0, since
                                                  //  psxRegs.cycle shouldn't be incremented again.

// Recompile block and return to normal codepath.
"recompile_block%=:                           \n"
"jal   %[recRecompile]                        \n"
"sw    $t2, f_off_temp_var1($sp)              \n" // <BD> Save block ptr across call
"lw    $t2, f_off_temp_var1($sp)              \n" // Restore block ptr upon return
"lw    $v0, %[psxRegs_pc_off]($fp)            \n" // Blocks expect $v0 to contain PC val on entry
"b     execute_block%=                        \n" // Resume normal code path, but first we must..
"lw    $t0, 0($t2)                            \n" // <BD> ..load $t0 with ptr to block code

// QPSX_039: Exit point - frame complete, return to libretro
"exit%=:                                      \n"
"addiu $sp, $sp, frame_size                   \n"
".set pop                                     \n"

: // Output
: // Input
  [psxRegs]                    "i" (&psxRegs),
  [psxRegs_pc_off]             "i" (off(pc)),
  [psxRegs_cycle_off]          "i" (off(cycle)),
  [psxRegs_io_cycle_ctr_off]   "i" (off(io_cycle_counter)),
  [recRecompile]               "i" (&recRecompile),
  [psxBranchTest]              "i" (&psxBranchTest),
  [psxRecLUT]                  "i" (psxRecLUT),
  [emu_frame_complete]         "i" (&emu_frame_complete)
: // Clobber - No need to list anything but 'saved' regs
  "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7", "fp", "ra", "memory"
);
#endif
}


/* Execute blocks starting at psxRegs.pc
 * Blocks return indirectly, to address stored at 16($sp)
 * Block pointers are looked up using virtual address mapping of recRAM/recROM.
 * Called only from recExecute(), see notes there.
 *
 * IMPORTANT: Functions containing inline ASM should have attribute 'noinline'.
 *            Crashes at callsites can occur otherwise, at least with GCC 4.xx.
 */
__attribute__((noinline)) static void recExecute_indirect_return_mmap()
{
	// Set block_ret_addr to 0, so generated code uses indirect returns
	block_ret_addr = block_fast_ret_addr = 0;

	// QPSX_039: Frame complete flag - must return after VBlank for libretro
	extern volatile int emu_frame_complete;

#ifndef ASM_EXECUTE_LOOP
	emu_frame_complete = 0;
	for (;;) {
		u32 *p = (u32*)PC_REC(psxRegs.pc);
		if (*p == 0)
			recRecompile();

		recFunc((void *)*p);

		/* v139: Cycles handled via $v1 in recFunc */

		if (psxRegs.cycle >= psxRegs.io_cycle_counter)
			psxBranchTest();

		// QPSX_039: Return after frame for libretro input/video
		if (emu_frame_complete) {
			emu_frame_complete = 0;
			return;
		}
	}
#else
// QPSX_039: Clear flag BEFORE entering loop - this was the bug in v37!
emu_frame_complete = 0;

__asm__ __volatile__ (
// NOTE: <BD> indicates an instruction in a branch-delay slot
".set push                                    \n"
".set noreorder                               \n"

// $fp/$s8 remains set to &psxRegs across all calls to blocks
"la    $fp, %[psxRegs]                        \n"

// Set up our own stack frame. Should have 8-byte alignment, and have 16 bytes
// empty at 0($sp) for use by functions called from within recompiled code.
".equ  frame_size,                  24        \n"
".equ  f_off_temp_var1,             20        \n"
".equ  f_off_block_ret_addr,        16        \n" // NOTE: blocks assume this is at 16($sp)!
"addiu $sp, $sp, -frame_size                  \n"

// Store const block return address at fixed location in stack frame
"la    $t0, loop%=                            \n"
"sw    $t0, f_off_block_ret_addr($sp)         \n"

// Load $v0 once with psxRegs.pc, blocks will assign new value when returning
"lw    $v0, %[psxRegs_pc_off]($fp)            \n"

// Load $v1 once with zero. It is the # of cycles psxRegs.cycle should be
// incremented by when a block returns.
"move  $v1, $0                                \n"

// Align loop on cache-line boundary
".balign 32                                   \n"

////////////////////////////
//       LOOP CODE:       //
////////////////////////////

// NOTE: Blocks return to top of loop.
// NOTE: Loop expects following values to be set:
// $v0 = new value for psxRegs.pc
// $v1 = # of cycles to increment psxRegs.cycle by

// The loop pseudocode is this, interleaving ops to reduce load stalls:
//
// loop:
// $t2 = REC_RAM_VADDR | ($v0 & 0x00ffffff)
// $t0 = *($t2)
// psxRegs.cycle += $v1
// if (psxRegs.cycle >= psxRegs.io_cycle_counter)
//    goto call_psxBranchTest;
// psxRegs.pc = $v0
// if ($t0 == 0)
//    goto recompile_block;
// $ra = block return address
// goto $t0;
// /* Code at addr $t0 will run and return having set $v0 to new psxRegs.pc */
// /*  value and $v1 to the number of cycles to increment psxRegs.cycle by. */

// Infinite loop, blocks return here
"loop%=:                                      \n"
"lw    $t3, %[psxRegs_cycle_off]($fp)         \n" // $t3 = psxRegs.cycle
"lw    $t4, %[psxRegs_io_cycle_ctr_off]($fp)  \n" // $t4 = psxRegs.io_cycle_counter

// The block ptrs are mapped virtually to address space allowing lower
//  24 bits of PS1 PC address to lookup start of any RAM or ROM code block.
"lui   $t2, %[REC_RAM_VADDR_UPPER]            \n"
#ifdef HAVE_MIPS32R2_EXT_INS
"ins   $t2, $v0, 0, 24                        \n"
#else
"sll   $t1, $v0, 8                            \n"
"srl   $t1, $t1, 8                            \n"
"or    $t2, $t2, $t1                          \n"
#endif
"lw    $t0, 0($t2)                            \n" // $t0 = address of start of block code, or
                                                  //       or 0 if block needs recompilation
                                                  // IMPORTANT: leave block ptr in $t2, it gets
                                                  // saved & re-used if recRecompile() is called.

"addu  $t3, $t3, $v1                          \n" // $t3 = psxRegs.cycle + $v1

// Must call psxBranchTest() when psxRegs.cycle >= psxRegs.io_cycle_counter
"sltu  $t4, $t3, $t4                          \n"
"beqz  $t4, call_psxBranchTest%=              \n"
"sw    $t3, %[psxRegs_cycle_off]($fp)         \n" // <BD> IMPORTANT: store new psxRegs.cycle val,
                                                  //  whether or not we are branching here

// Recompile block, if necessary
"beqz  $t0, recompile_block%=                 \n"
"sw    $v0, %[psxRegs_pc_off]($fp)            \n" // <BD> Use BD slot to store new psxRegs.pc val

// Execute already-compiled block. It will return at top of loop.
"execute_block%=:                             \n"
"jr    $t0                                    \n"
"lw    $ra, 16($sp)                           \n" // <BD> Load block return address

////////////////////////////
//     NON-LOOP CODE:     //
////////////////////////////

// Call psxBranchTest() and go back to top of loop
"call_psxBranchTest%=:                        \n"
"jal   %[psxBranchTest]                       \n"
"sw    $v0, %[psxRegs_pc_off]($fp)            \n" // <BD> Use BD slot to store new psxRegs.pc val,
                                                  //  as psxBranchTest() might issue an exception.
// QPSX_039: Check emu_frame_complete flag - exit if frame is done
"lui   $t5, %%hi(%[emu_frame_complete])       \n"
"lw    $t6, %%lo(%[emu_frame_complete])($t5)  \n"
"bnez  $t6, exit%=                            \n" // Exit loop if frame complete
"nop                                          \n"
"lw    $v0, %[psxRegs_pc_off]($fp)            \n" // After psxBranchTest() returns, load psxRegs.pc
                                                  //  back into $v0, which could be different than
                                                  //  before the call if an exception was issued.
"b     loop%=                                 \n" // Go back to top to process psxRegs.pc again..
"move  $v1, $0                                \n" // <BD> ..using BD slot to set $v1 to 0, since
                                                  //  psxRegs.cycle shouldn't be incremented again.

// Recompile block and return to normal codepath.
"recompile_block%=:                           \n"
"jal   %[recRecompile]                        \n"
"sw    $t2, f_off_temp_var1($sp)              \n" // <BD> Save block ptr across call
"lw    $t2, f_off_temp_var1($sp)              \n" // Restore block ptr upon return
"lw    $v0, %[psxRegs_pc_off]($fp)            \n" // Blocks expect $v0 to contain PC val on entry
"b     execute_block%=                        \n" // Resume normal code path, but first we must..
"lw    $t0, 0($t2)                            \n" // <BD> ..load $t0 with ptr to block code

// QPSX_039: Exit point - frame complete, return to libretro
"exit%=:                                      \n"
"addiu $sp, $sp, frame_size                   \n"
".set pop                                     \n"

: // Output
: // Input
  [psxRegs]                    "i" (&psxRegs),
  [psxRegs_pc_off]             "i" (off(pc)),
  [psxRegs_cycle_off]          "i" (off(cycle)),
  [psxRegs_io_cycle_ctr_off]   "i" (off(io_cycle_counter)),
  [recRecompile]               "i" (&recRecompile),
  [psxBranchTest]              "i" (&psxBranchTest),
  [REC_RAM_VADDR_UPPER]        "i" (REC_RAM_VADDR >> 16),
  [emu_frame_complete]         "i" (&emu_frame_complete)
: // Clobber - No need to list anything but 'saved' regs
  "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7", "fp", "ra", "memory"
);
#endif
}


/* Execute blocks starting at psxRegs.pc
 * Blocks return directly (not indirectly via stack address var).
 * Block pointers are looked up using psxRecLUT[].
 * Called only from recExecute(), see notes there.

 * IMPORTANT: Functions containing inline ASM should have attribute 'noinline'.
 *            Crashes at callsites can occur otherwise, at least with GCC 4.xx.
 */
__attribute__((noinline)) static void recExecute_direct_return_lut()
{
__asm__ __volatile__ (
// NOTE: <BD> indicates an instruction in a branch-delay slot
".set push                                    \n"
".set noreorder                               \n"

// Set up our own stack frame. Should have 8-byte alignment, and have 16 bytes
// empty at 0($sp) for use by functions called from within recompiled code.
".equ  frame_size,                        32  \n"
".equ  f_off_branchtest_fastpath_ra,      28  \n"
".equ  f_off_temp_var1,                   24  \n"
".equ  f_off_block_start_addr,            16  \n"
"addiu $sp, $sp, -frame_size                  \n"

#ifdef USE_DIRECT_FASTPATH_BLOCK_RETURN_JUMPS
// Set the global var 'block_fast_ret_addr'.
"la    $t0, fastpath_loop%=                   \n"
"lui   $t1, %%hi(%[block_fast_ret_addr])      \n"
"sw    $t0, %%lo(%[block_fast_ret_addr])($t1) \n"

// We need a stack var to hold a return address for a 'fastpath' call to
//  psxBranchTest. This helps reduce size of code through sharing.
"la    $t0, branchtest_fastpath_retaddr%=     \n"
"sw    $t0, f_off_branchtest_fastpath_ra($sp) \n"
#endif

// Set the global var 'block_ret_addr'.
"la    $t0, loop%=                            \n"
"lui   $t1, %%hi(%[block_ret_addr])           \n"
"sw    $t0, %%lo(%[block_ret_addr])($t1)      \n"

// $fp/$s8 remains set to &psxRegs across all calls to blocks
"la    $fp, %[psxRegs]                        \n"

// Load $v0 once with psxRegs.pc, blocks will assign new value when returning
"lw    $v0, %[psxRegs_pc_off]($fp)            \n"

// Load $v1 once with zero. It is the # of cycles psxRegs.cycle should be
// incremented by when a block returns.
"move  $v1, $0                                \n"

// Align loop on cache-line boundary
".balign 32                                   \n"

////////////////////////////
//       LOOP CODE:       //
////////////////////////////

// NOTE: Blocks return to top of loop.
// NOTE: Loop expects following values to be set:
// $v0 = new value for psxRegs.pc
// $v1 = # of cycles to increment psxRegs.cycle by

// The loop pseudocode is this, interleaving ops to reduce load stalls:
//
// loop:
// $t2 = psxRecLUT[pxsRegs.pc >> 16] + (psxRegs.pc & 0xffff)
// $t0 = *($t2)
// psxRegs.cycle += $v1
// if (psxRegs.cycle >= psxRegs.io_cycle_counter)
//    goto call_psxBranchTest;
// psxRegs.pc = $v0
// if ($t0 == 0)
//    goto recompile_block;
// tmp_block_start_addr = $t0
// goto $t0;
// /* Code at addr $t0 will run and return having set $v0 to new psxRegs.pc */
// /*  value and $v1 to the number of cycles to increment psxRegs.cycle by. */
// /* If block branches back to its beginning, it will return to the        */
// /*  fastpath version of loop that skips looking up a code pointer.       */

// Infinite loop, blocks return here
"loop%=:                                      \n"
"lw    $t3, %[psxRegs_cycle_off]($fp)         \n" // $t3 = psxRegs.cycle
"lui   $t1, %%hi(%[psxRecLUT])                \n"
"srl   $t2, $v0, 16                           \n"
"sll   $t2, $t2, 2                            \n" // sizeof() psxRecLUT[] elements is 4
"addu  $t1, $t1, $t2                          \n"
"lw    $t1, %%lo(%[psxRecLUT])($t1)           \n" // $t1 = psxRecLUT[psxRegs.pc >> 16]
"lw    $t4, %[psxRegs_io_cycle_ctr_off]($fp)  \n" // $t4 = psxRegs.io_cycle_counter
"addu  $t3, $t3, $v1                          \n" // $t3 = psxRegs.cycle + $v1
"andi  $t0, $v0, 0xffff                       \n"
"addu  $t2, $t0, $t1                          \n"
"lw    $t0, 0($t2)                            \n" // $t0 = address of start of block code, or
                                                  //       or 0 if block needs recompilation
                                                  // IMPORTANT: leave block ptr in $t2, it gets
                                                  // saved & re-used if recRecompile() is called.


// Must call psxBranchTest() when psxRegs.cycle >= psxRegs.io_cycle_counter
"sltu  $t4, $t3, $t4                          \n"
"beqz  $t4, call_psxBranchTest%=              \n"
"sw    $t3, %[psxRegs_cycle_off]($fp)         \n" // <BD> IMPORTANT: store new psxRegs.cycle val,
                                                  //  whether or not we are branching here

// Recompile block, if necessary
"beqz  $t0, recompile_block%=                 \n"
"sw    $v0, %[psxRegs_pc_off]($fp)            \n" // <BD> Use BD slot to store new psxRegs.pc val

// Execute already-compiled block. It returns to top of 'fastpath' loop if it
//  jumps to its own beginning PC. Otherwise, it returns to top of main loop.
"execute_block%=:                             \n"
"jr    $t0                                    \n"
#ifdef USE_DIRECT_FASTPATH_BLOCK_RETURN_JUMPS
"sw    $t0, f_off_block_start_addr($sp)       \n" // <BD> Save code address, in case block jumps back
                                                  //      to its beginning and takes 'fastpath'
#else
"nop                                          \n" // <BD>
#endif

////////////////////////////
//   BRANCH-TEST CODE:    //
////////////////////////////

// Call psxBranchTest() and go back to top of loop
"call_psxBranchTest%=:                        \n"
"jal   %[psxBranchTest]                       \n"
"sw    $v0, %[psxRegs_pc_off]($fp)            \n" // <BD> Store new psxRegs.pc val before calling C
"branchtest_fastpath_retaddr%=:               \n" // Next 3 instructions shared with 'fastpath' code..
"lw    $v0, %[psxRegs_pc_off]($fp)            \n" // After psxBranchTest() returns, load psxRegs.pc
                                                  //  back into $v0, which could be different than
                                                  //  before the call if an exception was issued.
"b     loop%=                                 \n" // Go back to top to process new psxRegs.pc value..
"move  $v1, $0                                \n" // <BD> ..using BD slot to set $v1 to 0, since
                                                  //  psxRegs.cycle shouldn't be incremented again.

#ifdef USE_DIRECT_FASTPATH_BLOCK_RETURN_JUMPS
////////////////////////////
//   FASTPATH LOOP CODE:  //
////////////////////////////

// When a block's new PC (jump target) is the same as its own beginning PC,
//  i.e. it branches back to its own top, it will return to this 'fast path'
//  version of above loop. It assumes that the block is already compiled and
//  unmodified, saving cycles versus the general loop. The main loop was
//  careful to save the block start address at location in stack frame.
//  NOTE: Blocks returning this way don't bother to set $v0 to any PC value.
"fastpath_loop%=:                             \n"
"lw    $t3, %[psxRegs_cycle_off]($fp)         \n" // $t3 = psxRegs.cycle
"lw    $t4, %[psxRegs_io_cycle_ctr_off]($fp)  \n" // $t4 = psxRegs.io_cycle_counter
"lw    $t0, f_off_block_start_addr($sp)       \n" // Load block code addr saved in main loop
"addu  $t3, $t3, $v1                          \n" // $t3 = psxRegs.cycle + $v1

// Must call psxBranchTest() when psxRegs.cycle >= psxRegs.io_cycle_counter
"sltu  $t4, $t3, $t4                          \n"
"beqz  $t4, call_psxBranchTest_fastpath%=     \n"
"sw    $t3, %[psxRegs_cycle_off]($fp)         \n" // <BD> IMPORTANT: store new psxRegs.cycle val,
                                                  //  whether or not we are branching here

// Execute already-compiled block. It returns to top of 'fastpath' loop if it
//  jumps to its own beginning PC. Otherwise, it returns to top of main loop.
"jr    $t0                                    \n"
"lw    $v0, %[psxRegs_pc_off]($fp)            \n" // <BD> Blocks expect $v0 to contain PC val on entry

////////////////////////////
//   BRANCH-TEST CODE:    //
////////////////////////////

// Call psxBranchTest(). We don't have a new PC in $v0 to store, so call it
//  directly, and have it return to code shared with main 'call_psxBranchTest'.
"call_psxBranchTest_fastpath%=:               \n"
"j     %[psxBranchTest]                       \n"
"lw    $ra, f_off_branchtest_fastpath_ra($sp) \n" // <BD>

#endif // USE_DIRECT_FASTPATH_BLOCK_RETURN_JUMPS

////////////////////////////
// RECOMPILE BLOCK CODE:  //
////////////////////////////

// Recompile block and return to normal codepath.
"recompile_block%=:                           \n"
"jal   %[recRecompile]                        \n"
"sw    $t2, f_off_temp_var1($sp)              \n" // <BD> Save block ptr across call
"lw    $t2, f_off_temp_var1($sp)              \n" // Restore block ptr upon return
"lw    $v0, %[psxRegs_pc_off]($fp)            \n" // Blocks expect $v0 to contain PC val on entry
"b     execute_block%=                        \n" // Resume normal code path, but first we must..
"lw    $t0, 0($t2)                            \n" // <BD> ..load $t0 with ptr to block code


// Destroy stack frame, exiting inlined ASM block
// NOTE: During block executions, we'd never reach this point because the block
//  dispatch loop is currently infinite. This could change in the future.
// TODO: Could add a way to reset a game or load a new game from within
//  the running emulator by setting a global boolean, resetting
//  psxRegs.io_cycle_counter to 0, and checking if it's been set before
//  calling pxsBranchTest() here. If set, you must jump here to
//  exit the loop, to ensure that stack frame is adjusted before return!
"exit%=:                                      \n"
"addiu $sp, $sp, frame_size                   \n"
".set pop                                     \n"

: // Output
: // Input
  [block_ret_addr]             "i" (&block_ret_addr),
  [block_fast_ret_addr]        "i" (&block_fast_ret_addr),
  [psxRegs]                    "i" (&psxRegs),
  [psxRegs_pc_off]             "i" (off(pc)),
  [psxRegs_cycle_off]          "i" (off(cycle)),
  [psxRegs_io_cycle_ctr_off]   "i" (off(io_cycle_counter)),
  [recRecompile]               "i" (&recRecompile),
  [psxBranchTest]              "i" (&psxBranchTest),
  [psxRecLUT]                  "i" (psxRecLUT)
: // Clobber - No need to list anything but 'saved' regs
  "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7", "fp", "ra", "memory"
);
}


/* Execute blocks starting at psxRegs.pc
 * Blocks return directly (not indirectly via stack address var).
 * Block pointers are looked up using virtual address mapping of recRAM/recROM.
 * Called only from recExecute(), see notes there.
 *
 * IMPORTANT: Functions containing inline ASM should have attribute 'noinline'.
 *            Crashes at callsites can occur otherwise, at least with GCC 4.xx.
 */
__attribute__((noinline)) static void recExecute_direct_return_mmap()
{
__asm__ __volatile__ (
// NOTE: <BD> indicates an instruction in a branch-delay slot
".set push                                    \n"
".set noreorder                               \n"

// Set up our own stack frame. Should have 8-byte alignment, and have 16 bytes
// empty at 0($sp) for use by functions called from within recompiled code.
".equ  frame_size,                        32  \n"
".equ  f_off_branchtest_fastpath_ra,      28  \n"
".equ  f_off_temp_var1,                   24  \n"
".equ  f_off_block_start_addr,            16  \n"
"addiu $sp, $sp, -frame_size                  \n"

#ifdef USE_DIRECT_FASTPATH_BLOCK_RETURN_JUMPS
// Set the global var 'block_fast_ret_addr'.
"la    $t0, fastpath_loop%=                   \n"
"lui   $t1, %%hi(%[block_fast_ret_addr])      \n"
"sw    $t0, %%lo(%[block_fast_ret_addr])($t1) \n"

// We need a stack var to hold a return address for a 'fastpath' call to
//  psxBranchTest. This helps reduce size of code through sharing.
"la    $t0, branchtest_fastpath_retaddr%=     \n"
"sw    $t0, f_off_branchtest_fastpath_ra($sp) \n"
#endif

// Set the global var 'block_ret_addr'.
"la    $t0, loop%=                            \n"
"lui   $t1, %%hi(%[block_ret_addr])           \n"
"sw    $t0, %%lo(%[block_ret_addr])($t1)      \n"

// $fp/$s8 remains set to &psxRegs across all calls to blocks
"la    $fp, %[psxRegs]                        \n"

// Load $v0 once with psxRegs.pc, blocks will assign new value when returning
"lw    $v0, %[psxRegs_pc_off]($fp)            \n"

// Load $v1 once with zero. It is the # of cycles psxRegs.cycle should be
// incremented by when a block returns.
"move  $v1, $0                                \n"

// Align loop on cache-line boundary
".balign 32                                   \n"

////////////////////////////
//       LOOP CODE:       //
////////////////////////////

// NOTE: Blocks return to top of loop.
// NOTE: Loop expects following values to be set:
// $v0 = new value for psxRegs.pc
// $v1 = # of cycles to increment psxRegs.cycle by

// The loop pseudocode is this, interleaving ops to reduce load stalls:
//
// loop:
// $t2 = REC_RAM_VADDR | ($v0 & 0x00ffffff)
// $t0 = *($t2)
// psxRegs.cycle += $v1
// if (psxRegs.cycle >= psxRegs.io_cycle_counter)
//    goto call_psxBranchTest;
// psxRegs.pc = $v0
// if ($t0 == 0)
//    goto recompile_block;
// tmp_block_start_addr = $t0
// goto $t0;
// /* Code at addr $t0 will run and return having set $v0 to new psxRegs.pc */
// /*  value and $v1 to the number of cycles to increment psxRegs.cycle by. */
// /* If block branches back to its beginning, it will return to the        */
// /*  fastpath version of loop that skips looking up a code pointer.       */

// Infinite loop, blocks return here
"loop%=:                                      \n"
"lw    $t3, %[psxRegs_cycle_off]($fp)         \n" // $t3 = psxRegs.cycle
"lw    $t4, %[psxRegs_io_cycle_ctr_off]($fp)  \n" // $t4 = psxRegs.io_cycle_counter

// The block ptrs are mapped virtually to address space allowing lower
//  24 bits of PS1 PC address to lookup start of any RAM or ROM code block.
"lui   $t2, %[REC_RAM_VADDR_UPPER]            \n"
#ifdef HAVE_MIPS32R2_EXT_INS
"ins   $t2, $v0, 0, 24                        \n"
#else
"sll   $t1, $v0, 8                            \n"
"srl   $t1, $t1, 8                            \n"
"or    $t2, $t2, $t1                          \n"
#endif
"lw    $t0, 0($t2)                            \n" // $t0 = address of start of block code, or
                                                  //       or 0 if block needs recompilation
                                                  // IMPORTANT: leave block ptr in $t2, it gets
                                                  // saved & re-used if recRecompile() is called.

"addu  $t3, $t3, $v1                          \n" // $t3 = psxRegs.cycle + $v1

// Must call psxBranchTest() when psxRegs.cycle >= psxRegs.io_cycle_counter
"sltu  $t4, $t3, $t4                          \n"
"beqz  $t4, call_psxBranchTest%=              \n"
"sw    $t3, %[psxRegs_cycle_off]($fp)         \n" // <BD> IMPORTANT: store new psxRegs.cycle val,
                                                  //  whether or not we are branching here

// Recompile block, if necessary
"beqz  $t0, recompile_block%=                 \n"
"sw    $v0, %[psxRegs_pc_off]($fp)            \n" // <BD> Use BD slot to store new psxRegs.pc val

// Execute already-compiled block. It returns to top of 'fastpath' loop if it
//  jumps to its own beginning PC. Otherwise, it returns to top of main loop.
"execute_block%=:                             \n"
"jr    $t0                                    \n"
#ifdef USE_DIRECT_FASTPATH_BLOCK_RETURN_JUMPS
"sw    $t0, f_off_block_start_addr($sp)       \n" // <BD> Save code address, in case block jumps back
                                                  //      to its beginning and takes 'fastpath'
#else
"nop                                          \n" // <BD>
#endif

////////////////////////////
//   BRANCH-TEST CODE:    //
////////////////////////////

// Call psxBranchTest() and go back to top of loop
"call_psxBranchTest%=:                        \n"
"jal   %[psxBranchTest]                       \n"
"sw    $v0, %[psxRegs_pc_off]($fp)            \n" // <BD> Store new psxRegs.pc val before calling C
"branchtest_fastpath_retaddr%=:               \n" // Next 3 instructions shared with 'fastpath' code..
"lw    $v0, %[psxRegs_pc_off]($fp)            \n" // After psxBranchTest() returns, load psxRegs.pc
                                                  //  back into $v0, which could be different than
                                                  //  before the call if an exception was issued.
"b     loop%=                                 \n" // Go back to top to process new psxRegs.pc value..
"move  $v1, $0                                \n" // <BD> ..using BD slot to set $v1 to 0, since
                                                  //  psxRegs.cycle shouldn't be incremented again.

#ifdef USE_DIRECT_FASTPATH_BLOCK_RETURN_JUMPS
////////////////////////////
//   FASTPATH LOOP CODE:  //
////////////////////////////

// When a block's new PC (jump target) is the same as its own beginning PC,
//  i.e. it branches back to its own top, it will return to this 'fast path'
//  version of above loop. It assumes that the block is already compiled and
//  unmodified, saving cycles versus the general loop. The main loop was
//  careful to save the block start address at location in stack frame.
//  NOTE: Blocks returning this way don't bother to set $v0 to any PC value.
"fastpath_loop%=:                             \n"
"lw    $t3, %[psxRegs_cycle_off]($fp)         \n" // $t3 = psxRegs.cycle
"lw    $t4, %[psxRegs_io_cycle_ctr_off]($fp)  \n" // $t4 = psxRegs.io_cycle_counter
"lw    $t0, f_off_block_start_addr($sp)       \n" // Load block code addr saved in main loop
"addu  $t3, $t3, $v1                          \n" // $t3 = psxRegs.cycle + $v1

// Must call psxBranchTest() when psxRegs.cycle >= psxRegs.io_cycle_counter
"sltu  $t4, $t3, $t4                          \n"
"beqz  $t4, call_psxBranchTest_fastpath%=     \n"
"sw    $t3, %[psxRegs_cycle_off]($fp)         \n" // <BD> IMPORTANT: store new psxRegs.cycle val,
                                                  //  whether or not we are branching here

// Execute already-compiled block. It returns to top of 'fastpath' loop if it
//  jumps to its own beginning PC. Otherwise, it returns to top of main loop.
"jr    $t0                                    \n"
"lw    $v0, %[psxRegs_pc_off]($fp)            \n" // <BD> Blocks expect $v0 to contain PC val on entry

////////////////////////////
//   BRANCH-TEST CODE:    //
////////////////////////////

// Call psxBranchTest(). We don't have a new PC in $v0 to store, so call it
//  directly, and have it return to code shared with main 'call_psxBranchTest'.
"call_psxBranchTest_fastpath%=:               \n"
"j     %[psxBranchTest]                       \n"
"lw    $ra, f_off_branchtest_fastpath_ra($sp) \n" // <BD>

#endif // USE_DIRECT_FASTPATH_BLOCK_RETURN_JUMPS

////////////////////////////
// RECOMPILE BLOCK CODE:  //
////////////////////////////

// Recompile block and return to normal codepath.
"recompile_block%=:                           \n"
"jal   %[recRecompile]                        \n"
"sw    $t2, f_off_temp_var1($sp)              \n" // <BD> Save block ptr across call
"lw    $t2, f_off_temp_var1($sp)              \n" // Restore block ptr upon return
"lw    $v0, %[psxRegs_pc_off]($fp)            \n" // Blocks expect $v0 to contain PC val on entry
"b     execute_block%=                        \n" // Resume normal code path, but first we must..
"lw    $t0, 0($t2)                            \n" // <BD> ..load $t0 with ptr to block code


// Destroy stack frame, exiting inlined ASM block
// NOTE: During block executions, we'd never reach this point because the block
//  dispatch loop is currently infinite. This could change in the future.
// TODO: Could add a way to reset a game or load a new game from within
//  the running emulator by setting a global boolean, resetting
//  psxRegs.io_cycle_counter to 0, and checking if it's been set before
//  calling pxsBranchTest() here. If set, you must jump here to
//  exit the loop, to ensure that stack frame is adjusted before return!
"exit%=:                                      \n"
"addiu $sp, $sp, frame_size                   \n"
".set pop                                     \n"

: // Output
: // Input
  [block_ret_addr]             "i" (&block_ret_addr),
  [block_fast_ret_addr]        "i" (&block_fast_ret_addr),
  [psxRegs]                    "i" (&psxRegs),
  [psxRegs_pc_off]             "i" (off(pc)),
  [psxRegs_cycle_off]          "i" (off(cycle)),
  [psxRegs_io_cycle_ctr_off]   "i" (off(io_cycle_counter)),
  [recRecompile]               "i" (&recRecompile),
  [psxBranchTest]              "i" (&psxBranchTest),
  [REC_RAM_VADDR_UPPER]        "i" (REC_RAM_VADDR >> 16)
: // Clobber - No need to list anything but 'saved' regs
  "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7", "fp", "ra", "memory"
);
}


/* Execute blocks starting at psxRegs.pc until 'target_pc' is reached.
 * Blocks return indirectly, to address stored at 16($sp)
 *
 * IMPORTANT: Functions containing inline ASM should have attribute 'noinline'.
 *            Crashes at callsites can occur otherwise, at least with GCC 4.xx.
 */
__attribute__((noinline)) static void recExecuteBlock(unsigned target_pc)
{
	// Set block_ret_addr to 0, so generated code uses indirect returns
	block_ret_addr = block_fast_ret_addr = 0;

#ifndef ASM_EXECUTE_LOOP
	static unsigned int block_count = 0;
	static unsigned int last_pc = 0;
	static unsigned int branch_test_count = 0;
	do {
		u32 *p = (u32*)PC_REC(psxRegs.pc);

		if (*p == 0)
			recRecompile();

		/* v236: Unified native mode switching
		 * mode 3 = NATIVE: run native block (if available), fallback to dynarec
		 * mode 4 = DIFF_TEST: run dynarec, compare with native, log mismatches
		 * other = pure dynarec
		 */
#if DIFF_TEST_ENABLE
		if (g_opt_native_mode == 4) {
			/* MODE 4: DIFF_TEST - dynarec main, compare with native */
			u32 block_pc = psxRegs.pc;

			static psxRegisters regs_before __attribute__((aligned(4)));
			static psxRegisters regs_dynarec __attribute__((aligned(4)));
			memcpy(&regs_before, &psxRegs, sizeof(psxRegisters));

			/* Run DYNAREC (main execution) */
			recFunc((void *)*p);
			dynarec_fallback_count++;

			memcpy(&regs_dynarec, &psxRegs, sizeof(psxRegisters));

			/* Check if we have native block for this PC */
			u32 lut_idx = pc_to_lut_idx(block_pc);
			void* native_block = NULL;
			if (lut_idx < NATIVE_LUT_SIZE &&
			    native_block_lut[lut_idx] != NULL &&
			    native_block_pc[lut_idx] == block_pc)
			{
				native_block = native_block_lut[lut_idx];
			}

			if (native_block) {
				/* Restore state, run native, compare */
				memcpy(&psxRegs, &regs_before, sizeof(psxRegisters));
				recFunc(native_block);
				native_exec_count++;

				bool mismatch = diff_test_compare(block_pc, &regs_before, &regs_dynarec, &psxRegs);
				(void)mismatch;

				/* ALWAYS restore dynarec result */
				memcpy(&psxRegs, &regs_dynarec, sizeof(psxRegisters));
			}
		} else
#endif
		{
			/* MODE 3 or other: just run the block (native or dynarec) */
#if NATIVE_DEBUG_RING
			if (g_opt_native_mode == 3) {
				native_debug_ring_add(psxRegs.pc, (u32)*p, true);
			}
#endif
			recFunc((void *)*p);
#if NATIVE_DEBUG_RING
			if (g_opt_native_mode == 3) {
				native_debug_ring_complete();
			}
#endif
		}

		/* v139: Cycles handled via $v1 in recFunc */

		if (psxRegs.cycle >= psxRegs.io_cycle_counter) {
#if NATIVE_DEBUG_RING
			/* v237: Dump ring buffer at each psxBranchTest for debugging */
			if (g_opt_native_mode == 3 && branch_test_count >= 8) {
				native_debug_ring_dump();
			}
			branch_test_count++;
#endif
			psxBranchTest();
			// QPSX_047: Exit loop when in BIOS mode after interrupt
			if (target_pc == 0) break;
		}

		block_count++;
	} while (psxRegs.pc != target_pc);
#else
__asm__ __volatile__ (
// NOTE: <BD> indicates an instruction in a branch-delay slot
".set push                                    \n"
".set noreorder                               \n"

// $fp/$s8 remains set to &psxRegs across all calls to blocks
"la     $fp, %[psxRegs]                       \n"

// Set up our own stack frame. Should have 8-byte alignment, and have 16 bytes
// empty at 0($sp) for use by functions called from within recompiled code.
".equ  frame_size,                  32        \n"
".equ  f_off_target_pc,             24        \n"
".equ  f_off_temp_var1,             20        \n"
".equ  f_off_block_ret_addr,        16        \n" // NOTE: blocks assume this is at 16($sp)!
"addiu $sp, $sp, -frame_size                  \n"

// Store const copy of 'target_pc' parameter in stack frame
"sw    %[target_pc], f_off_target_pc($sp)     \n"

// Store const block return address at fixed location in stack frame
"la    $t0, return_from_block%=               \n"
"sw    $t0, f_off_block_ret_addr($sp)         \n"

// Load $v0 once with psxRegs.pc, blocks will assign new value when returning
"lw    $v0, %[psxRegs_pc_off]($fp)            \n"

// Load $v1 once with zero. It is the # of cycles psxRegs.cycle should be
// incremented by when a block returns.
"move  $v1, $0                                \n"

// Align loop on cache-line boundary
".balign 32                                   \n"

////////////////////////////
//       LOOP CODE:       //
////////////////////////////

// NOTE: Loop expects following values to be set:
// $v0 = new value for psxRegs.pc
// $v1 = # of cycles to increment psxRegs.cycle by

// The loop pseudocode is this, interleaving ops to reduce load stalls:
//
// loop:
// $t2 = psxRecLUT[pxsRegs.pc >> 16] + (psxRegs.pc & 0xffff)
// $t0 = *($t2)
// psxRegs.cycle += $v1
// if (psxRegs.cycle >= psxRegs.io_cycle_counter)
//    goto call_psxBranchTest;
// if ($t0 == 0)
//    goto recompile_block;
// $ra = block return address
// goto $t0;
// /* Code at addr $t0 will run and return having set $v0 to new psxRegs.pc */
// /*  value and $v1 to the number of cycles to increment psxRegs.cycle by. */
// psxRegs.pc = $v0
// if (psxRegs.pc == target_pc)
//    goto exit;
// else
//    goto loop;

// Loop until psxRegs.pc == target_pc
"loop%=:                                      \n"
"lw    $t3, %[psxRegs_cycle_off]($fp)         \n" // $t3 = psxRegs.cycle
"lui   $t1, %%hi(%[psxRecLUT])                \n"
"srl   $t2, $v0, 16                           \n"
"sll   $t2, $t2, 2                            \n" // sizeof() psxRecLUT[] elements is 4
"addu  $t1, $t1, $t2                          \n"
"lw    $t1, %%lo(%[psxRecLUT])($t1)           \n" // $t1 = psxRecLUT[psxRegs.pc >> 16]
"lw    $t4, %[psxRegs_io_cycle_ctr_off]($fp)  \n" // $t4 = psxRegs.io_cycle_counter
"addu  $t3, $t3, $v1                          \n" // $t3 = new psxRegs.cycle val
"andi  $t0, $v0, 0xffff                       \n"
"addu  $t2, $t0, $t1                          \n"
"lw    $t0, 0($t2)                            \n" // $t0 now points to beginning of recompiled
                                                  //  block, or is 0 if block needs compiling

// Must call psxBranchTest() when psxRegs.cycle >= psxRegs.io_cycle_counter
"sltu  $t4, $t3, $t4                          \n"
"beqz  $t4, call_psxBranchTest%=              \n"
"sw    $t3, %[psxRegs_cycle_off]($fp)         \n" // <BD> IMPORTANT: store new psxRegs.cycle val,
                                                  //  whether or not we are branching here

// Recompile block, if necessary
"beqz  $t0, recompile_block%=                 \n"
"nop                                          \n" // <BD>

"execute_block%=:                             \n"
"jalr  $ra, $t0                               \n" // Block is returning here, so safe to
"nop                                          \n" // <BD> set $ra using 'jalr'

"return_from_block%=:                         \n"
// Return point for all executed blocks, which will have set:
// $v0 to new value for psxRegs.pc
// $v1 to the # of cycles to increment psxRegs.cycle by

// QPSX_046: Check if target_pc has been reached
// Special case: when target_pc == 0, exit when PC enters RAM (0x80xxxxxx)
// This is needed for BIOS bootstrap which should stop when kernel is loaded
"lw    $t0, f_off_target_pc($sp)              \n" // $t0 = target_pc
"sw    $v0, %[psxRegs_pc_off]($fp)            \n" // Store new psxRegs.pc val first
"bnez  $t0, check_exact_pc%=                  \n" // if target_pc != 0, check exact match
"nop                                          \n" // <BD>
// target_pc == 0: Check if PC entered RAM region 0x80xxxxxx (BIOS end)
"lui   $t1, 0xff80                            \n" // $t1 = 0xff800000
"and   $t1, $v0, $t1                          \n" // $t1 = PC & 0xff800000
"lui   $t2, 0x8000                            \n" // $t2 = 0x80000000
"bne   $t1, $t2, loop%=                       \n" // if not in RAM region, continue loop
"nop                                          \n" // <BD>
"b     target_reached%=                       \n" // PC in RAM - exit!
"nop                                          \n" // <BD>
"check_exact_pc%=:                            \n"
"bne   $v0, $t0, loop%=                       \n" // Loop if target_pc hasn't been reached
"nop                                          \n" // <BD>
"target_reached%=:                            \n"

// Before cleanup/exit, ensure psxRegs.cycle is incremented by block's $v1 retval
"lw    $t3, %[psxRegs_cycle_off]($fp)         \n" // $t3 = psxRegs.cycle
"addu  $t3, $t3, $v1                          \n"
"b     exit%=                                 \n" // Goto cleanup/exit code, using BD slot..
"sw    $t3, %[psxRegs_cycle_off]($fp)         \n" // <BD> ..to store new psxRegs.cycle val

////////////////////////////
//     NON-LOOP CODE:     //
////////////////////////////

// Call psxBranchTest() and go back to top of loop
"call_psxBranchTest%=:                        \n"
"jal   %[psxBranchTest]                       \n"
"nop                                          \n" // <BD>
"lw    $v0, %[psxRegs_pc_off]($fp)            \n" // After psxBranchTest() returns, load psxRegs.pc
                                                  //  back into $v0, which could be different than
                                                  //  before the call if an exception was issued.
"b     loop%=                                 \n" // Go back to top to process psxRegs.pc again..
"move  $v1, $0                                \n" // <BD> ..using BD slot to set $v1 to 0, since
                                                  //  psxRegs.cycle shouldn't be incremented again.

// Recompile block and return to normal codepath
"recompile_block%=:                           \n"
"jal   %[recRecompile]                        \n"
"sw    $t2, f_off_temp_var1($sp)              \n" // <BD> Save block ptr across call
"lw    $t2, f_off_temp_var1($sp)              \n" // Restore block ptr upon return
"lw    $v0, %[psxRegs_pc_off]($fp)            \n" // Blocks expect $v0 to contain PC val on entry
"b     execute_block%=                        \n" // Resume normal code path, but first we must..
"lw    $t0, 0($t2)                            \n" // <BD> ..load $t0 with ptr to block code

// Destroy stack frame, exiting inlined ASM block
"exit%=:                                      \n"
"addiu $sp, $sp, frame_size                   \n"
".set pop                                     \n"

: // Output
: // Input
  [target_pc]                  "r" (target_pc),
  [psxRegs]                    "i" (&psxRegs),
  [psxRegs_pc_off]             "i" (off(pc)),
  [psxRegs_cycle_off]          "i" (off(cycle)),
  [psxRegs_io_cycle_ctr_off]   "i" (off(io_cycle_counter)),
  [psxRecLUT]                  "i" (psxRecLUT),
  [recRecompile]               "i" (&recRecompile),
  [psxBranchTest]              "i" (&psxBranchTest)
: // Clobber - No need to list anything but 'saved' regs
  "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7", "fp", "ra", "memory"
);
#endif
}


/***************************************************************************
 * QPSX v100: Direct Block LUT Dispatch Loop
 *
 * OPTIMIZATION: Replaces 2-level psxRecLUT lookup with direct lookup.
 *
 * OLD (psxRecLUT - 8 instructions, 2 memory loads):
 *   lui   $t1, %hi(psxRecLUT)
 *   srl   $t2, $v0, 16
 *   sll   $t2, $t2, 2
 *   addu  $t1, $t1, $t2
 *   lw    $t1, %lo(psxRecLUT)($t1)    <- LOAD #1
 *   andi  $t0, $v0, 0xffff
 *   addu  $t2, $t0, $t1
 *   lw    $t0, 0($t2)                 <- LOAD #2
 *
 * NEW (Direct - 4 instructions, 1 memory load for RAM path):
 *   li    $t1, 0x1FFFFC               (or use pre-loaded $s2)
 *   and   $t2, $v0, $t1               (mask to 2MB, word-aligned)
 *   addu  $t2, $s0, $t2               ($s0 = recRAM base, pre-loaded)
 *   lw    $t0, 0($t2)                 <- ONLY 1 LOAD!
 *
 * SAVINGS: 4 instructions + 1 memory load per dispatch = ~10-15 cycles
 * With 100K+ dispatches/frame, this is 1-2ms improvement!
 *
 * ROM (0xBFCxxxxx) is handled with branch to separate path (rare after boot).
 ***************************************************************************/

/* v101: Direct Block LUT controlled by menu option (default OFF) */
extern int g_opt_direct_block_lut;  /* Defined in libretro-core.cpp */
/* v112: g_opt_cycle_batch declared near top of file */

__attribute__((noinline)) static void recExecute_direct_block_lut()
{
	// Set block_ret_addr to 0, so generated code uses indirect returns
	block_ret_addr = block_fast_ret_addr = 0;

	// QPSX_039: Frame complete flag - must return after VBlank for libretro
	extern volatile int emu_frame_complete;

#ifndef ASM_EXECUTE_LOOP
	/* C fallback - same as lut version but with direct lookup */
	emu_frame_complete = 0;

	/* v112: Cycle batching - check less frequently for speed */
	/* batch_mask: 0=every block, 1=every 2, 3=every 4, 7=every 8 */
	const u32 batch_mask = (1 << g_opt_cycle_batch) - 1;  /* 0, 1, 3, 7 */
	u32 batch_counter = 0;

	for (;;) {
		u32 pc = psxRegs.pc;
		void *code;

		/* v113: Hot Block Cache - fast path for cached blocks */
		code = hbc_lookup(pc);
		if (code == NULL) {
			/* Cache miss - direct lookup: RAM or ROM? */
			u32 *p;
			if ((pc >> 20) == 0xBFC) {
				/* ROM: 0xBFC00000-0xBFC7FFFF */
				p = (u32*)(recROM + (pc & 0x7FFFC));
			} else {
				/* RAM: All other addresses (0x00/0x80/0xA0 mirrors) */
				p = (u32*)(recRAM + (pc & 0x1FFFFC));
			}

			if (*p == 0)
				recRecompile();

			code = (void*)*p;
			/* Store in cache for next time */
			hbc_store(pc, code);
		}

		recFunc(code);

		/* v139: Cycles handled via $v1 in recFunc */

		/* v112: Only check cycles every N blocks based on batch level */
		if ((batch_counter++ & batch_mask) == 0) {
			if (psxRegs.cycle >= psxRegs.io_cycle_counter)
				psxBranchTest();
		}

		if (emu_frame_complete) {
			emu_frame_complete = 0;
			return;
		}
	}
#else
/* QPSX v100: Optimized ASM dispatch with Direct Block LUT */
emu_frame_complete = 0;

__asm__ __volatile__ (
".set push                                    \n"
".set noreorder                               \n"

/* Setup: $fp = &psxRegs (standard, kept across all blocks) */
"la    $fp, %[psxRegs]                        \n"

/* Stack frame */
".equ  frame_size,                  32        \n"
".equ  f_off_temp_var1,             28        \n"
".equ  f_off_recrom,                24        \n"
".equ  f_off_block_ret_addr,        16        \n"
"addiu $sp, $sp, -frame_size                  \n"

/* v100 OPTIMIZATION: Pre-load recRAM base into $s0 */
"la    $s0, %[recRAM]                         \n"
"lw    $s0, 0($s0)                            \n"  /* $s0 = recRAM pointer */

/* v100: Pre-load recROM base (store on stack for ROM fallback) */
"la    $t0, %[recROM]                         \n"
"lw    $t0, 0($t0)                            \n"
"sw    $t0, f_off_recrom($sp)                 \n"

/* v100: Pre-load masks into saved regs for speed */
"li    $s1, 0x1FFFFC                          \n"  /* RAM mask (2MB, word-aligned) */
"li    $s2, 0x7FFFC                           \n"  /* ROM mask (512KB, word-aligned) */
"li    $s3, 0xBFC                             \n"  /* ROM upper bits check */

/* Store block return address */
"la    $t0, loop_dbl%=                        \n"
"sw    $t0, f_off_block_ret_addr($sp)         \n"

/* Load initial PC */
"lw    $v0, %[psxRegs_pc_off]($fp)            \n"
"move  $v1, $0                                \n"

/* Align loop on cache line for better I-cache performance */
".balign 32                                   \n"

/*==========================================================================
 * MAIN DISPATCH LOOP - DIRECT BLOCK LUT (v100)
 *
 * Pseudocode:
 *   loop:
 *     if ((pc >> 20) == 0xBFC) goto rom_lookup;  // Rare
 *     block_ptr = *(recRAM + (pc & 0x1FFFFC));   // Direct RAM lookup!
 *     psxRegs.cycle += cycles;
 *     if (cycle >= io_cycle_counter) goto branch_test;
 *     if (block_ptr == 0) goto recompile;
 *     execute(block_ptr);
 *==========================================================================*/

"loop_dbl%=:                                  \n"

/* v100: Direct lookup - check for ROM first (upper 12 bits == 0xBFC) */
"srl   $t3, $v0, 20                           \n"  /* Upper 12 bits of PC */
"beq   $t3, $s3, rom_lookup_dbl%=             \n"  /* If 0xBFC, goto ROM path */
"lw    $t4, %[psxRegs_cycle_off]($fp)         \n"  /* <BD> Load cycle (interleaved) */

/*----------------------------------------------------------------------
 * RAM PATH (99%+ of lookups) - OPTIMIZED DIRECT LOOKUP
 * Only 3 instructions + 1 load (vs 7 instructions + 2 loads before!)
 *----------------------------------------------------------------------*/
"and   $t2, $v0, $s1                          \n"  /* $t2 = pc & 0x1FFFFC */
"addu  $t2, $s0, $t2                          \n"  /* $t2 = recRAM + offset */
"lw    $t0, 0($t2)                            \n"  /* $t0 = block_ptr (ONLY 1 LOAD!) */

"ram_continue_dbl%=:                          \n"
/* Cycle counting (interleaved with lookup to hide load latency) */
"lw    $t5, %[psxRegs_io_cycle_ctr_off]($fp)  \n"  /* $t5 = io_cycle_counter */
"addu  $t4, $t4, $v1                          \n"  /* $t4 = cycle + $v1 */

/* Branch test check */
"sltu  $t5, $t4, $t5                          \n"
"beqz  $t5, call_psxBranchTest_dbl%=          \n"
"sw    $t4, %[psxRegs_cycle_off]($fp)         \n"  /* <BD> Store updated cycle */

/* Recompile if block not yet compiled */
"beqz  $t0, recompile_block_dbl%=             \n"
"sw    $v0, %[psxRegs_pc_off]($fp)            \n"  /* <BD> Store new PC */

/* Execute block - it returns to loop_dbl with new $v0, $v1 */
"execute_block_dbl%=:                         \n"
"jr    $t0                                    \n"
"lw    $ra, f_off_block_ret_addr($sp)         \n"  /* <BD> Load return address */

/*----------------------------------------------------------------------
 * ROM PATH (rare - only BIOS, called <0.1% of time after boot)
 * Uses separate lookup from recROM table
 *----------------------------------------------------------------------*/
"rom_lookup_dbl%=:                            \n"
"and   $t2, $v0, $s2                          \n"  /* $t2 = pc & 0x7FFFC */
"lw    $t6, f_off_recrom($sp)                 \n"  /* $t6 = recROM base */
"addu  $t2, $t6, $t2                          \n"  /* $t2 = recROM + offset */
"b     ram_continue_dbl%=                     \n"
"lw    $t0, 0($t2)                            \n"  /* <BD> Load block ptr */

/*----------------------------------------------------------------------
 * NON-LOOP CODE
 *----------------------------------------------------------------------*/

/* psxBranchTest() call */
"call_psxBranchTest_dbl%=:                    \n"
"jal   %[psxBranchTest]                       \n"
"sw    $v0, %[psxRegs_pc_off]($fp)            \n"  /* <BD> Store PC (exception might change it) */

/* Check frame complete flag */
"lui   $t5, %%hi(%[emu_frame_complete])       \n"
"lw    $t6, %%lo(%[emu_frame_complete])($t5)  \n"
"bnez  $t6, exit_dbl%=                        \n"  /* Exit if frame done */
"nop                                          \n"

/* Reload PC and continue */
"lw    $v0, %[psxRegs_pc_off]($fp)            \n"
"b     loop_dbl%=                             \n"
"move  $v1, $0                                \n"  /* <BD> Reset cycle delta */

/* Block recompilation */
"recompile_block_dbl%=:                       \n"
"jal   %[recRecompile]                        \n"
"sw    $t2, f_off_temp_var1($sp)              \n"  /* <BD> Save block ptr addr */
"lw    $t2, f_off_temp_var1($sp)              \n"  /* Restore ptr addr */
"lw    $v0, %[psxRegs_pc_off]($fp)            \n"  /* Reload PC (blocks expect this) */
"b     execute_block_dbl%=                    \n"
"lw    $t0, 0($t2)                            \n"  /* <BD> Load compiled block ptr */

/* Exit - frame complete */
"exit_dbl%=:                                  \n"
"addiu $sp, $sp, frame_size                   \n"
".set pop                                     \n"

: /* Output */
: /* Input */
  [psxRegs]                    "i" (&psxRegs),
  [psxRegs_pc_off]             "i" (off(pc)),
  [psxRegs_cycle_off]          "i" (off(cycle)),
  [psxRegs_io_cycle_ctr_off]   "i" (off(io_cycle_counter)),
  [recRecompile]               "i" (&recRecompile),
  [psxBranchTest]              "i" (&psxBranchTest),
  [recRAM]                     "i" (&recRAM),
  [recROM]                     "i" (&recROM),
  [emu_frame_complete]         "i" (&emu_frame_complete)
: /* Clobber */
  "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7", "fp", "ra", "memory"
);
#endif
}


static void recExecute()
{
	PROFILE_START(PROF_CPU_TOTAL);

	// QPSX_039: Only reset code cache on first call
	// Prevents slow frame execution caused by clearing cache every frame
	static bool rec_initialized = false;
	if (!rec_initialized) {
		// Clear code cache so that all emitted code from this point forward uses
		//  the correct block-return method. This also clears out any now-dead code
		//  emitted during BIOS startup. Non-dead BIOS code gets recompiled fresh.
		recReset();
		rec_initialized = true;
	}

	// By default, emit code that returns to dispatch loop indirectly, using
	//  address kept on stack. This return method is safe for use with both
	//  HLE BIOS and real BIOS. In fact, HLE BIOS requires indirect return
	//  jumps: During main execution, i.e. recExecute(), HLE BIOS calls
	//  recExecuteBlock() for certain 'softcalls', so in effect, two separate
	//  dispatch loops can be active, even simultaneously! Therefore, blocks
	//  under HLE BIOS cannot know at compile-time where to return to.
	//
	// To force indirect return code, set 'block_ret_addr' to 0 here.
	bool use_indirect_return_dispatch_loop = true;
	block_ret_addr = block_fast_ret_addr = 0;

#if defined(ASM_EXECUTE_LOOP) && defined(USE_DIRECT_BLOCK_RETURN_JUMPS)
	// When using a real BIOS, recExecuteBlock() is only ever called for
	//  initial BIOS startup, and when a certain PC is reached, it returns.
	//  Soon after that, this function is called to begin real execution.
	//  At this point, blocks will always be returning to the same address.
	//  We now emit faster, denser code that returns via direct jumps.
	//
	// NOTE: On entry, direct-return versions of dispatch loops set vars
	//       'block_ret_addr' and 'block_fast_ret_addr' to correct value.
	// IMPORTANT: All existing code needs to have been cleared at this point.
	use_indirect_return_dispatch_loop = Config.HLE;
#endif

	/*
	 * QPSX v101: Direct Block LUT controlled by menu option (default OFF)
	 * This eliminates the 2-level psxRecLUT lookup, saving ~10-15 cycles per dispatch.
	 * Falls back to standard lut dispatch if rec_mem_mapped (virtual mapping active).
	 *
	 * v129: DISABLED when NATIVE_EXEC_DEBUG is enabled - forces C dispatch for logging
	 */
#ifdef SF2000
#if !NATIVE_EXEC_DEBUG
	if (g_opt_direct_block_lut && !rec_mem_mapped) {
		recExecute_direct_block_lut();
		PROFILE_END(PROF_CPU_TOTAL);
		return;
	}
#else
	/* v130: Log once, not every call */
	static bool debug_msg_shown = false;
	if (!debug_msg_shown) {
		xlog("EXEC: v130 DEBUG - using C dispatch for logging");
		debug_msg_shown = true;
	}
#endif
#endif

	if (use_indirect_return_dispatch_loop) {
		if (rec_mem_mapped)
			recExecute_indirect_return_mmap();
		else
			recExecute_indirect_return_lut();
	} else {
		if (rec_mem_mapped)
			recExecute_direct_return_mmap();
		else
			recExecute_direct_return_lut();
	}

	PROFILE_END(PROF_CPU_TOTAL);
}


/* Invalidate 'Size' code block pointers at word-aligned PS1 address 'Addr'.
 * v101: extern "C" for psxmem_asm.S linkage */
extern "C" void recClear(u32 Addr, u32 Size)
{
	const u32 masked_ram_addr = Addr & 0x1ffffc;

	// Check if the page(s) of PS1 RAM that 'Addr','Size' target contain the
	//  start of any blocks. If not, invalidation would have no effect and is
	//  skipped. This eliminates 99% of large unnecessary invalidations that
	//  occur when many games stream CD data in-game.
	u32 page = masked_ram_addr/4096;
	u32 end_page = ((masked_ram_addr + (Size-1)*4)/4096) + 1;
	bool has_code = false;
	do {
		u32 pflag = 1 << (page & 7);  // Each byte in code_pages[] represents 8 pages
		has_code = code_pages[page/8] & pflag;
	} while ((++page != end_page) && !has_code);

	// NOTE: If PS1 mem is mapped/mirrored, make PC_REC_MMAP use the same virtual
	//       mirror region the game is already using, reducing TLB pressure.
	uptr dst_base = !rec_mem_mapped ? (uptr)recRAM : (uptr)PC_REC_MMAP(Addr & ~0x1fffff);

	if (has_code) {
		void *dst = (void*)(dst_base + (masked_ram_addr * REC_RAM_PTR_SIZE/4));
		memset(dst, 0, Size*REC_RAM_PTR_SIZE);
		/* v113: Clear Hot Block Cache when code is invalidated */
		hbc_clear();
	}
}


/* Notification from emulator. */
void recNotify(int note, void *data __attribute__((unused)))
{
	switch (note)
	{
		/* R3000ACPU_NOTIFY_CACHE_ISOLATED,
		 * R3000ACPU_NOTIFY_CACHE_UNISOLATED
		 *  Sent from psxMemWrite32_CacheCtrlPort(). Also see notes there.
		 */
		case R3000ACPU_NOTIFY_CACHE_ISOLATED:
			/*  There's no need to do anything here:
			 * psxMemWrite32_CacheCtrlPort() has backed up lower 64KB PS1 RAM,
			 * allowing stores in emitted code to skip checking if cache
			 * is isolated before writing to RAM (the old 'writeok' check).
			 */
			REC_LOG_V("R3000ACPU_NOTIFY_CACHE_ISOLATED\n");
			break;
		case R3000ACPU_NOTIFY_CACHE_UNISOLATED:
			/*  Flush entire code cache, game has loaded new code:
			 * BIOS or routine has finished invalidating cache lines.
			 * psxMemWrite32_CacheCtrlPort() has restored lower 64KB PS1 RAM.
			 *  Using this coarse invalidation method fixes some games that
			 * previously needed hacks in recClear(), 'Buster Bros. Collection'.
			 * It's also part of a fix/hack for certain games that did Icache
			 * trickery (see DMA3 stuff further below).
			 * TODO: With this new method, it should eventually be possible to
			 *  provide an option to allow emitted code to skip most code
			 *  invalidations after stores, boosting speed.
			 */
			recClear(0, 0x200000/4);
			REC_LOG_V("R3000ACPU_NOTIFY_CACHE_UNISOLATED\n");
			break;

		/* Sent from psxDma3(). Also see notes there. */
		case R3000ACPU_NOTIFY_DMA3_EXE_LOAD:
			/* Game has begun reading from a CDROM file whose first sector is
			 * a standard 'PS-X EXE' header. Part of a hack by senquack to fix:
			 *  'Formula One Arcade' (crash on load)
			 *  'Formula One 99'     (crash on load) (NOTE: PAL version requires .SBI file)
			 *  'Formula One 2001'   (in-game controls, AI broken.. even the PS3
			 *                        supposedly has trouble emulating this.)
			 *
			 *  The workaround is enabled on a per-game basis (using CdromId).
			 *
			 *  These games do Icache trickery and merely doing the usual
			 * code-flush when Icache is unisolated is not enough. The fix is
			 * admittedly just a lucky hack, and requires these things:
			 *  1.)  As usual, invalidate code whenever emu calls recClear(),
			 *      typically after a DMA transfer.
			 *      *However*, emit no code invalidations whatsoever.
			 *      Fixes the in-game AI/controls in 'Formula One 2001'.
			 *  2.)  As usual, flush code cache when Icache is unisolated.
			 *      *However*, also flush cache when psxDma3() notifies us here.
			 *      This fixes crashes.
			 */
			if (flush_code_on_dma3_exe_load) {
				recClear(0, 0x200000/4);
				REC_LOG_V("R3000ACPU_NOTIFY_DMA3_EXE_LOAD .. Flushing dynarec cache\n");
			} else {
				REC_LOG_V("R3000ACPU_NOTIFY_DMA3_EXE_LOAD\n");
			}
			break;

		default:
			break;
	}
}


static void recReset()
{
	memset(code_pages, 0, sizeof(code_pages));
	memset(recRAM, 0, REC_RAM_SIZE);
	memset(recROM, 0, REC_ROM_SIZE);

	recMem = (u32*)recMemBase;

	regReset();

	// Set default recompilation options and any per-game options
	rec_set_options();

	/* v113: Initialize/update Hot Block Cache based on current settings */
	hbc_init();

	/* v117: Reset cumulative native mode stats when code cache is cleared */
	memset(&native_stats, 0, sizeof(native_stats));
}


R3000Acpu psxRec =
{
	recInit,
	recReset,
	recExecute,
	recExecuteBlock,
	recClear,
	recNotify,
	recShutdown
};
