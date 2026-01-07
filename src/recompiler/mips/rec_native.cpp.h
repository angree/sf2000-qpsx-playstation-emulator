/*
 * QPSX v250 - Native Block Translator
 *
 * v246: FIX collision detection for HOST_SP_REMAP!
 *   BUG: v233 changed HOST_SP_REMAP from $at(1) to $gp(28), but collision
 *        detection still checked for $at! If PSX code used BOTH $sp AND $gp,
 *        emit_load_gprs() would overwrite PSX $gp with PSX $sp value.
 *        This caused crashes with pure ALU blocks (LUI+ADDIU+ADDI).
 *   FIX: Check bit 28 ($gp) instead of bit 1 ($at) for sp collision.
 *
 * v237 - Native Block Translator (NATIVE + DIFF_TEST MODES)
 *
 * v237: FIX cycle_multiplier not applied in native blocks!
 *   BUG: Native epilogue used raw instruction count for $v1.
 *        Dynarec uses ADJUST_CLOCK(count) which applies cycle_multiplier.
 *        With cycle_multiplier=0x200 (2x), native returned HALF the cycles!
 *        This caused mode 3 (NATIVE) to freeze - events never fired.
 *   FIX: Use ADJUST_CLOCK(count) in native epilogue, same as dynarec.
 *
 * v235: FIX hardware address LOAD clobbering dst register!
 *   BUG: v230 used dst as temp for physical address computation.
 *        When hardware skip triggered, dst had physical address, not data!
 *        Caused $v0/$v1 mismatches with values like 0x238, 0x35A.
 *   FIX: Never modify dst before RAM check. Use $k1 + psxRegs.code as temp.
 *        Hardware path: dst unchanged (keeps emit_load_gprs value)
 *        RAM path: dst gets inline-loaded data
 *
 * v231 - Native Block Translator (DIFF_TEST MODE)
 *
 * v231: PRIORITY 2 BUG FIXES - 2 additional issues resolved:
 *
 *   FIX #2: BLTZAL/BGEZAL link to HOST register (line ~1536)
 *     BUG: Link address (PC+8) was written ONLY to psxRegs.GPR[31] (memory),
 *          but NOT to host register $t9 (HOST_RA_REMAP).
 *          If subsequent instruction reads $ra, it gets STALE value from host!
 *     FIX: Use LI32(HOST_RA_REMAP, ra_value) then SW to memory.
 *          Now both host register AND memory have correct link address.
 *     Sources: MIPS IV ISA Manual, PSX-SPX, MIPS Instruction Set (Harvard)
 *
 *   FIX #3: Reject LOAD/STORE in JR delay slot (line ~1147)
 *     BUG: JR delay slot handling emitted LOAD/STORE "as-is" without
 *          address translation (mask + psxM base + HW address check).
 *          This caused incorrect memory access or crashes.
 *     FIX: Reject blocks where JR delay slot contains LOAD (0x20-0x26),
 *          STORE (0x28-0x2E), or any branch/jump instruction.
 *     Sources: Wikipedia Delay Slot, jsgroth PSX CPU, MIPS ISA
 *
 * v230: CRITICAL BUG FIXES - 4 Priority 1 issues resolved:
 *
 *   FIX #1: $k0 restore after STORE (line ~1303)
 *     BUG: STORE used MOV($k0, src) to save value, clobbering psxM pointer.
 *          Subsequent LOADs used wrong base address!
 *     FIX: LW($k0, psxRegs.psxM) after STORE completes.
 *     Sources: emit_cop2_trap/lwc2/swc2 patterns
 *
 *   FIX #5: Reject MULT/DIV/MFHI/MFLO/MTHI/MTLO (line ~1116)
 *     BUG: These fell through to "Valid ALU R-type" and emitted as native.
 *          HI/LO registers are NOT synced with psxRegs.GPR[32/33]!
 *     FIX: Reject blocks containing funct 0x10-0x1B.
 *     Sources: Raymond Chen "The Old New Thing", PSX-SPX, chortle.ccsu.edu
 *
 *   FIX #6: JR target from HOST register (line ~1071)
 *     BUG: Old code loaded JR target from psxRegs.GPR[rs] AFTER delay slot.
 *          If rs was modified earlier in block, psxRegs had STALE value!
 *     FIX: Save HOST register to psxRegs.code BEFORE delay slot,
 *          then load from psxRegs.code AFTER delay slot.
 *     Sources: PSX-SPX, Raymond Chen, Univ. Delaware
 *
 *   FIX #8: Hardware address check for LOAD (line ~1197)
 *     BUG: Inline LOAD used mask 0x1FFFFF which WRAPPED hardware addresses:
 *          0x1F801810 (GPU) became RAM[0x1810], 0xBFC00000 (BIOS) → RAM[1.75MB]!
 *     FIX: Strip segment bits (& 0x1FFFFFFF), check if physical >= 0x200000.
 *          If hardware address, skip inline load (DIFF_TEST will catch mismatch).
 *     Sources: PSX-SPX Memory Map, PSX-SPX I/O Map, jsgroth blog
 *
 * v214 - Original version (details below)
 *
 * v214: FIX ALL BRANCHES - USE required_count FOR branch_at_end!
 *       - BUG in v213: BEQ/BNE/BLEZ/BGTZ/REGIMM used (i == count - 2) for branch_at_end.
 *         But 'count' was from analysis, not from required_count!
 *         Native block ended at wrong position when branch was in middle.
 *       - FIX: Use required_count for branch_at_end when required_count > 0:
 *         bool branch_at_end = (required_count > 0)
 *                            ? (i >= required_count - 2)
 *                            : (i == count - 2);
 *         Now ALL control flow (JR/J/BEQ/BNE/BLEZ/BGTZ/REGIMM) uses same logic.
 *
 * v213: FIX JR/J IN MIDDLE OF BLOCK - CHECK required_count!
 *       - JR/J now checks required_count, emits NOP if in middle.
 *
 * v212: FIX JR DELAY SLOT - EMIT REAL INSTRUCTION!
 *       - Delay slot now properly emits with register remapping.
 *
 * v211: FIX JR HANDLING - IMPLEMENT, DON'T REJECT!
 *       - JR now ALWAYS terminates block (when at end).
 *         Load target address from psxRegs.GPR[rs] (memory), store to psxRegs.pc.
 *
 * v210: FIX REGIMM HANDLING + $at COMPARISON!
 *       - BUG in v209: REGIMM (BLTZ/BGEZ) at end of block had NO delay slot processing!
 *         When REGIMM was at position count-1, loop ended without processing delay slot.
 *       - FIX: Add branch_at_end check for REGIMM, same as BEQ/BNE/BLEZ/BGTZ.
 *         Now REGIMM at end of block is fully handled with condition evaluation.
 *       - ALSO: diff_test_compare now SKIPS $at (register 1) in comparison.
 *         Dynarec uses $at as internal scratch, native doesn't - false mismatches!
 *         Per MIPS convention, $at is assembler temp and NOT preserved.
 *
 * v209: Remap sp/fp/ra to at/t8/t9 (caused more mismatches due to $at scratch issue)
 * v208: Tried remapping s0-s7 but caused collisions
 *
 * v207: TRUE DIFF_TEST - dynarec main, native parallel for comparison
 * v206: SP/FP/RA REMAPPING + J/JR SUPPORT!
 *
 * v205: Added comprehensive rejection statistics tracking
 * v201: LOAD/STORE ENABLED!
 *       - Native handles: ALU, LOAD (LB/LH/LW/LBU/LHU), STORE (SB/SH/SW)
 *       - Dynarec fallback: COP, branches with non-NOP delay slots
 *       - NATIVE_ALU_ONLY=0, NATIVE_MAX_BLOCK_SIZE=32
 *
 * v200: Single execution (native OR dynarec, not both)
 * v199: ALU-only mode (LOAD/STORE rejected) - too few blocks
 *
 * v193: DIFFERENTIAL TESTING MODE (used in v195-v198)
 *       - Compile BOTH native and dynarec for each block
 *       - Run dynarec for actual execution (game works)
 *       - Run native in parallel, compare results
 *       - Log any mismatches to find native bugs
 *       - Supports: ALU, loads (LB/LH/LW/LBU/LHU), stores (SB/SH/SW),
 *         branches (BEQ/BNE/BLEZ/BGTZ with NOP delay slots)
 *
 * v139: FIX cycle counting - set $v1 = count AFTER emit_save_gprs
 *       v137-138 bug: recFunc adds $v1 to cycle, but native blocks left
 *       PSX $v1 (garbage like 0x80200000) in host $v1, corrupting cycle!
 *       FIX: Set host $v1 = count after PSX regs are saved to memory.
 * v138: Debug logging revealed the bug
 * v137: Store cycles to g_native_cycles (didn't work - recFunc still adds $v1)
 * v136: Skip cycle counting - test basic execution works (WORKS!)
 * v135: Return cycles in $v1 BEFORE save (Exception 5 - corrupts PSX state!)
 * v134: Direct cycle update (Exception 4 - wrong $s8 usage)
 * v133: Reject blocks that START with SYSCALL/BREAK (infinite loop bug)
 * v131: Skip blocks using $sp/$fp/$ra (remapped registers)
 * v128: J-type register detection fix, SYSCALL/BREAK handling
 * v127: Cache flush after generating code
 * v126: Branches capture condition BEFORE delay slot
 * v125: JR/JALR capture target BEFORE delay slot
 * v124: Blocks without jump/branch set PC explicitly
 * v123: Delay slot executes BEFORE branch/jump
 *
 * REGISTER MAPPING:
 * - PSX $0      → Host $0 (hardwired zero)
 * - PSX $1-$25  → Host $1-$25 (direct)
 * - PSX $26-$27 → forbidden (we use $k0/$k1)
 * - PSX $28     → Host $28 (direct)
 * - PSX $29($sp)→ Host $s0($16) ← REMAPPED!
 * - PSX $30($fp)→ Host $s1($17) ← REMAPPED!
 * - PSX $31($ra)→ Host $s2($18) ← REMAPPED!
 *
 * HOST RESERVED:
 * - $k0 ($26) = psxM base pointer
 * - $k1 ($27) = temp for address translation / PC calc
 * - $s8 ($30) = PERM_REG_1 (psxRegs pointer)
 * - $sp ($29) = host stack
 * - $ra ($31) = host return
 */

#ifndef REC_NATIVE_V128_H
#define REC_NATIVE_V128_H

/* ============== v154 MASTER CONTROL FLAGS ============== */
/* v154: FIX v153 bug - validate PC BEFORE calling PSXM()!
 * v153 crashed because ALU-only check called PSXM(pc) before validating PC.
 * v154 moves PC validation before any PSXM() access.
 */
#define NATIVE_MASTER_ENABLE    1   /* 0=pure dynarec, 1=native enabled */
#define NATIVE_ALU_ONLY         0   /* v201: ENABLE load/store! (was 1 in v199-200) */
#define NATIVE_MAX_BLOCK_SIZE   32  /* v201: bigger blocks now that LOAD/STORE works */
#define NATIVE_SKIP_REMAP       0   /* v206: NOW HANDLING sp/fp/ra via remapping! */

/* v192: Counters for tracking native vs dynarec usage (NOT static - extern in recompiler.cpp) */
u32 native_used __attribute__((aligned(4))) = 0;
u32 dynarec_used __attribute__((aligned(4))) = 0;
static u32 stats_log_counter __attribute__((aligned(4))) = 0;
static u32 stats_logged __attribute__((aligned(4))) = 0;
#define STATS_LOG_INTERVAL 500000  /* Log every 500K blocks */

/* ============== XLOG DIAGNOSTICS ============== */
extern "C" void xlog(const char *fmt, ...);

/* v244: Menu toggle check - returns 1 if instruction enabled for native, 0 for dynarec */
extern "C" int native_cmd_check_enabled(u32 psx_opcode);

/* v219: psxMemWrite functions already declared in psxmem.h (included via recompiler.cpp) */

/* Diagnostic log level:
 * 0 = OFF
 * 1 = Block entry/exit only
 * 2 = + Phase info
 * 3 = + Each instruction
 */
/* v149: Disable NAT logs like v148 to test if memory barrier alone fixes crash */
#define NATIVE_LOG_LEVEL 2    /* v128: Normal verbosity */
#define NATIVE_LOG_LIMIT 100  /* v128: More blocks now that SYSCALL fixed */

/* v151: All globals aligned for SF2000 bare-metal stability */
static int native_log_count __attribute__((aligned(4))) = 0;

/* v149: NAT logs disabled - testing if __sync_synchronize() fixes heisenbug */
#if 0
#define NLOG1(fmt, ...) do { if (NATIVE_LOG_LEVEL >= 1 && native_log_count < NATIVE_LOG_LIMIT) xlog("NAT1: " fmt "\n", ##__VA_ARGS__); } while(0)
#define NLOG2(fmt, ...) do { if (NATIVE_LOG_LEVEL >= 2 && native_log_count < NATIVE_LOG_LIMIT) xlog("NAT2: " fmt "\n", ##__VA_ARGS__); } while(0)
#define NLOG3(fmt, ...) do { if (NATIVE_LOG_LEVEL >= 3 && native_log_count < NATIVE_LOG_LIMIT) xlog("NAT3: " fmt "\n", ##__VA_ARGS__); } while(0)
#else
#define NLOG1(fmt, ...) do {} while(0)
#define NLOG2(fmt, ...) do {} while(0)
#define NLOG3(fmt, ...) do {} while(0)
#endif

/* ============== DEBUG COUNTERS (v151: aligned for SF2000) ============== */
static u32 native_blocks_total __attribute__((aligned(4))) = 0;
static u32 native_blocks_compiled __attribute__((aligned(4))) = 0;
static u32 native_blocks_skipped_k0k1 __attribute__((aligned(4))) = 0;
static u32 native_instr_alu_copy __attribute__((aligned(4))) = 0;
static u32 native_instr_alu_remapped __attribute__((aligned(4))) = 0;
static u32 native_instr_loadstore __attribute__((aligned(4))) = 0;
static u32 native_instr_branch __attribute__((aligned(4))) = 0;
static u32 native_instr_jump __attribute__((aligned(4))) = 0;
static u32 native_instr_cop_trap __attribute__((aligned(4))) = 0;

/* v224: Store diagnostics - runtime counters for debugging */
u32 native_store_total __attribute__((aligned(4))) = 0;
u32 native_store_ram __attribute__((aligned(4))) = 0;
u32 native_store_hw __attribute__((aligned(4))) = 0;
u32 native_store_last_hw_addr __attribute__((aligned(4))) = 0;

/* v229: VALIDATION-ONLY mode - compare native value with dynarec result, DON'T write!
 *
 * How it works:
 * 1. Dynarec already executed and wrote to memory
 * 2. Native computes address + value but doesn't write
 * 3. We READ what dynarec wrote (current memory)
 * 4. COMPARE with what native would write
 * 5. If different = MISMATCH (log it!)
 * 6. DON'T write - keep dynarec's result (game works)
 *
 * This finds EXACTLY which STORE is buggy without 2MB checksum overhead!
 */
static u32 native_store_mismatch_count __attribute__((aligned(4))) = 0;
#define NATIVE_STORE_MISMATCH_LOG_LIMIT 20

static void psxMemWrite8_validate(u32 addr, u8 native_val) {
    /* Read what dynarec wrote */
    u8 dynarec_val = psxMemRead8(addr);

    /* Compare */
    if (native_val != dynarec_val) {
        if (native_store_mismatch_count < NATIVE_STORE_MISMATCH_LOG_LIMIT) {
            xlog("STORE8 MISMATCH @%08X: dyn=%02X nat=%02X\n", addr, dynarec_val, native_val);
            native_store_mismatch_count++;
        }
    }
    /* DON'T write - keep dynarec's value */
}

static void psxMemWrite16_validate(u32 addr, u16 native_val) {
    /* Read what dynarec wrote */
    u16 dynarec_val = psxMemRead16(addr);

    /* Compare */
    if (native_val != dynarec_val) {
        if (native_store_mismatch_count < NATIVE_STORE_MISMATCH_LOG_LIMIT) {
            xlog("STORE16 MISMATCH @%08X: dyn=%04X nat=%04X\n", addr, dynarec_val, native_val);
            native_store_mismatch_count++;
        }
    }
    /* DON'T write - keep dynarec's value */
}

static void psxMemWrite32_validate(u32 addr, u32 native_val) {
    /* Read what dynarec wrote */
    u32 dynarec_val = psxMemRead32(addr);

    /* Compare */
    if (native_val != dynarec_val) {
        if (native_store_mismatch_count < NATIVE_STORE_MISMATCH_LOG_LIMIT) {
            xlog("STORE32 MISMATCH @%08X: dyn=%08X nat=%08X\n", addr, dynarec_val, native_val);
            native_store_mismatch_count++;
        }
    }
    /* DON'T write - keep dynarec's value */
}

/* v205: Extern counters from recompiler.cpp - full rejection tracking */
extern u32 nat_blocks_attempted;
extern u32 nat_rej_remap, nat_rej_k0k1, nat_rej_garbage;
extern u32 nat_rej_j, nat_rej_jal, nat_rej_jr, nat_rej_jalr;
extern u32 nat_rej_bxx, nat_rej_regimm, nat_rej_delay;
extern u32 nat_rej_cop0, nat_rej_gte, nat_rej_lwc2, nat_rej_swc2;
extern u32 nat_rej_lwl, nat_rej_lwr, nat_rej_swl, nat_rej_swr;
extern u32 nat_rej_syscall, nat_rej_break, nat_rej_other;

/* v139: g_native_cycles global is no longer needed!
 * Native epilogue now sets $v1 = count AFTER emit_save_gprs(),
 * so recFunc's dispatcher adds $v1 to psxRegs.cycle.
 * This is the same mechanism used by dynarec blocks.
 */

/* ============== REGISTER DEFINITIONS ============== */
#define MIPSREG_K0 26
#define MIPSREG_K1 27

/* v233: MINIMAL PSX to Host register remapping - ONLY sp/fp/ra!
 *
 * Problem in v208: Remapping s0→s3, s1→s4, s2→s5 caused collisions when
 * PSX code used BOTH s0 AND s3 (both ended up in host s3).
 * Function prologues use sp,ra,s0,s1,s2,s3,s4 together - impossible to avoid!
 *
 * Solution v233: ONLY remap sp/fp/ra to registers PSX rarely uses:
 *   PSX $sp (29) → host $gp (28)  - v233: CHANGED from $at! $gp unused in bare-metal
 *   PSX $fp (30) → host $t8 (24)  - temp, not used in function prologues
 *   PSX $ra (31) → host $t9 (25)  - temp, not used in function prologues
 *
 * v233 FIX: Previously sp→at caused collision because PSX code ALSO uses $at
 * directly (e.g., LUI $at, imm). Now using $gp which is not used by:
 *   - PSX code (reserved, rarely used)
 *   - Native code (uses $k0/$k1 for scratch)
 *   - C runtime (bare-metal with -mno-abicalls -fno-pic)
 *
 * Collision detection: Reject blocks that use remapped target regs:
 *   - gp(28)  → collision with sp remap
 *   - t8(24)  → collision with fp remap
 *   - t9(25)  → collision with ra remap
 */
#define PSX_SP 29
#define PSX_FP 30
#define PSX_RA 31

/*
 * v233: CHANGED HOST_SP_REMAP from $at (1) to $gp (28)!
 *
 * BUG FIX: PSX code uses $at directly (e.g., LUI $at, 0x801D).
 * If HOST_SP_REMAP=1, then BOTH PSX $at AND PSX $sp map to HOST $at!
 * This causes value corruption and eventually crashes.
 *
 * Solution: Use $gp (28) which is:
 *   - NOT used by C runtime in bare-metal (-mno-abicalls -fno-pic)
 *   - NOT used by native code ($k0/$k1 are used instead)
 *   - Rarely used by PSX code (reserved for global pointer)
 */
#define HOST_SP_REMAP 28  /* PSX $sp (29) → host $gp (28) - CHANGED v233! */
#define HOST_FP_REMAP 24  /* PSX $fp (30) → host $t8 (24) */
#define HOST_RA_REMAP 25  /* PSX $ra (31) → host $t9 (25) */

/* v233: Skip if uses $k0/$k1 (native temps) OR $gp (now used for PSX $sp) */
#define FORBIDDEN_MASK ((1u << 26) | (1u << 27) | (1u << 28))

/* v209: Only 3 remapped registers now (sp/fp/ra) */
#define REMAP_MASK ((1u << 29) | (1u << 30) | (1u << 31))

#define NATIVE_MAX_BLOCK 128

/* ============== OPCODE HELPERS ============== */
#define OP_CODE(x)  ((x) >> 26)
#define OP_RS(x)    (((x) >> 21) & 0x1F)
#define OP_RT(x)    (((x) >> 16) & 0x1F)
#define OP_RD(x)    (((x) >> 11) & 0x1F)
#define OP_SHAMT(x) (((x) >> 6) & 0x1F)
#define OP_FUNCT(x) ((x) & 0x3F)
#define OP_IMM(x)   ((s16)((x) & 0xFFFF))
#define OP_TARGET(x) ((x) & 0x03FFFFFF)

/* v233: Remap PSX register to host register - ONLY sp/fp/ra remapped! */
static inline u32 remap_reg(u32 reg) {
    switch (reg) {
        case PSX_SP: return HOST_SP_REMAP;  /* 29 → 28 ($gp) - v233 CHANGED! */
        case PSX_FP: return HOST_FP_REMAP;  /* 30 → 24 ($t8) */
        case PSX_RA: return HOST_RA_REMAP;  /* 31 → 25 ($t9) */
        default:     return reg;  /* All others stay in natural registers! */
    }
}

/* Check if opcode uses forbidden ($k0/$k1) registers */
static bool uses_forbidden_regs(u32 opcode) {
    u32 op = OP_CODE(opcode);
    u32 rs = OP_RS(opcode);
    u32 rt = OP_RT(opcode);
    u32 rd = OP_RD(opcode);

    u32 used = 0;

    switch (op) {
        case 0x00: /* SPECIAL */
            used = (1u << rs) | (1u << rt) | (1u << rd);
            break;
        case 0x01: /* REGIMM */
        case 0x04: case 0x05: case 0x06: case 0x07: /* branches */
            used = (1u << rs) | (1u << rt);
            break;
        /* v128: J-type instructions have NO register fields!
         * The 26-bit target address was being misinterpreted as rs/rt/rd.
         * J uses no registers, JAL writes $ra (31) but that's never $k0/$k1.
         */
        case 0x02: /* J */
            used = 0;  /* No registers */
            break;
        case 0x03: /* JAL */
            used = (1u << 31);  /* Writes $ra, but $ra is not forbidden */
            break;
        case 0x08: case 0x09: case 0x0A: case 0x0B:
        case 0x0C: case 0x0D: case 0x0E: case 0x0F:
            used = (1u << rs) | (1u << rt);
            break;
        case 0x20: case 0x21: case 0x22: case 0x23:
        case 0x24: case 0x25: case 0x26:
        case 0x28: case 0x29: case 0x2A: case 0x2B: case 0x2E:
            used = (1u << rs) | (1u << rt);
            break;
        default:
            used = (1u << rs) | (1u << rt) | (1u << rd);
    }

    return (used & FORBIDDEN_MASK) != 0;
}

/* Check if opcode uses remapped registers */
static bool uses_remapped_regs(u32 opcode) {
    u32 op = OP_CODE(opcode);
    u32 rs = OP_RS(opcode);
    u32 rt = OP_RT(opcode);
    u32 rd = OP_RD(opcode);

    u32 used = (1u << rs) | (1u << rt) | (1u << rd);
    return (used & REMAP_MASK) != 0;
}

/* Check if branch/jump */
static inline bool is_branch(u32 opcode) {
    u32 op = OP_CODE(opcode);
    if (op == 0x01) return true; /* REGIMM */
    if (op >= 0x04 && op <= 0x07) return true; /* BEQ, BNE, BLEZ, BGTZ */
    return false;
}

static inline bool is_jump(u32 opcode) {
    u32 op = OP_CODE(opcode);
    u32 funct = OP_FUNCT(opcode);
    if (op == 0x02 || op == 0x03) return true; /* J, JAL */
    if (op == 0x00 && (funct == 0x08 || funct == 0x09)) return true; /* JR, JALR */
    return false;
}

static inline bool is_load(u32 opcode) {
    u32 op = OP_CODE(opcode);
    return (op >= 0x20 && op <= 0x26);
}

static inline bool is_store(u32 opcode) {
    u32 op = OP_CODE(opcode);
    return (op >= 0x28 && op <= 0x2E);
}

static inline bool is_cop(u32 opcode) {
    u32 op = OP_CODE(opcode);
    return (op == 0x10 || op == 0x12 || op == 0x32 || op == 0x3A);
}

/* v153: Check if opcode is PURE ALU (safe for native execution)
 * Accepts ONLY:
 * - NOP (0x00000000)
 * - R-type ALU: ADD, ADDU, SUB, SUBU, AND, OR, XOR, NOR, SLT, SLTU
 * - R-type shifts: SLL, SRL, SRA, SLLV, SRLV, SRAV
 * - I-type ALU: ADDI, ADDIU, SLTI, SLTIU, ANDI, ORI, XORI, LUI
 * Rejects ALL: loads, stores, branches, jumps, COP0/COP2, MULT, DIV, etc.
 */
static inline bool is_pure_alu(u32 opcode) {
    /* NOP is always safe */
    if (opcode == 0x00000000) return true;

    u32 op = OP_CODE(opcode);
    u32 funct = OP_FUNCT(opcode);

    if (op == 0x00) {
        /* R-type: check funct field */
        switch (funct) {
            /* Shifts (immediate) */
            case 0x00: /* SLL */
            case 0x02: /* SRL */
            case 0x03: /* SRA */
            /* Shifts (variable) */
            case 0x04: /* SLLV */
            case 0x06: /* SRLV */
            case 0x07: /* SRAV */
            /* ALU */
            case 0x20: /* ADD */
            case 0x21: /* ADDU */
            case 0x22: /* SUB */
            case 0x23: /* SUBU */
            case 0x24: /* AND */
            case 0x25: /* OR */
            case 0x26: /* XOR */
            case 0x27: /* NOR */
            case 0x2A: /* SLT */
            case 0x2B: /* SLTU */
                return true;
            default:
                /* JR, JALR, SYSCALL, BREAK, MULT, DIV, MFHI, MFLO, etc. - REJECT */
                return false;
        }
    }

    /* I-type ALU */
    switch (op) {
        case 0x08: /* ADDI */
        case 0x09: /* ADDIU */
        case 0x0A: /* SLTI */
        case 0x0B: /* SLTIU */
        case 0x0C: /* ANDI */
        case 0x0D: /* ORI */
        case 0x0E: /* XORI */
        case 0x0F: /* LUI */
            return true;
        default:
            /* Loads, stores, branches, jumps, COPx - REJECT */
            return false;
    }
}

/* ============== EMIT HELPERS ============== */

/* Remap registers in R-type opcode */
static u32 remap_r_type(u32 opcode) {
    u32 rs = remap_reg(OP_RS(opcode));
    u32 rt = remap_reg(OP_RT(opcode));
    u32 rd = remap_reg(OP_RD(opcode));
    u32 shamt = OP_SHAMT(opcode);
    u32 funct = OP_FUNCT(opcode);
    return (rs << 21) | (rt << 16) | (rd << 11) | (shamt << 6) | funct;
}

/* Remap registers in I-type opcode */
static u32 remap_i_type(u32 opcode) {
    u32 op = OP_CODE(opcode);
    u32 rs = remap_reg(OP_RS(opcode));
    u32 rt = remap_reg(OP_RT(opcode));
    u32 imm = opcode & 0xFFFF;
    return (op << 26) | (rs << 21) | (rt << 16) | imm;
}

/* v209: Save GPRs to psxRegs - ONLY sp/fp/ra remapped now */
static void emit_save_gprs(u32 mask) {
    NLOG3("emit_save_gprs mask=0x%08X", mask);
    /* PSX regs 1-28 go directly to same host reg (no remapping for s0-s7!) */
    for (int r = 1; r <= 28; r++) {
        if (mask & (1u << r)) {
            SW(r, PERM_REG_1, offGPR(r));
        }
    }

    /* v233: Only 3 remapped registers (sp/fp/ra) */
    if (mask & (1u << 29)) SW(HOST_SP_REMAP, PERM_REG_1, offGPR(29));  /* host $gp → PSX $sp (v233) */
    if (mask & (1u << 30)) SW(HOST_FP_REMAP, PERM_REG_1, offGPR(30));  /* host $t8 → PSX $fp */
    if (mask & (1u << 31)) SW(HOST_RA_REMAP, PERM_REG_1, offGPR(31));  /* host $t9 → PSX $ra */
}

/* v233: Load GPRs from psxRegs - ONLY sp/fp/ra remapped now */
static void emit_load_gprs(u32 mask) {
    NLOG3("emit_load_gprs mask=0x%08X", mask);
    /* PSX regs 1-28 go directly to same host reg (no remapping for s0-s7!) */
    for (int r = 1; r <= 28; r++) {
        if (mask & (1u << r)) {
            LW(r, PERM_REG_1, offGPR(r));
        }
    }

    /* v233: Only 3 remapped registers (sp/fp/ra) */
    if (mask & (1u << 29)) LW(HOST_SP_REMAP, PERM_REG_1, offGPR(29));  /* PSX $sp → host $gp (v233) */
    if (mask & (1u << 30)) LW(HOST_FP_REMAP, PERM_REG_1, offGPR(30));  /* PSX $fp → host $t8 */
    if (mask & (1u << 31)) LW(HOST_RA_REMAP, PERM_REG_1, offGPR(31));  /* PSX $ra → host $t9 */
}

/* Address translation: $k1 = psxM + (reg & 0x1FFFFF) */
static void emit_addr_xlat(u32 reg) {
    u32 host_reg = remap_reg(reg);
    SLL(MIPSREG_K1, host_reg, 11);
    SRL(MIPSREG_K1, MIPSREG_K1, 11);
    ADDU(MIPSREG_K1, MIPSREG_K0, MIPSREG_K1);
}

/* Emit load with address translation */
static void emit_load(u32 opcode) {
    u32 op = OP_CODE(opcode);
    u32 rs = OP_RS(opcode);
    u32 rt = remap_reg(OP_RT(opcode));
    s16 imm = OP_IMM(opcode);

    NLOG3("emit_load op=%02X rs=%d rt=%d->%d imm=%d", op, rs, OP_RT(opcode), rt, imm);

    emit_addr_xlat(rs);

    switch (op) {
        case 0x20: LB(rt, MIPSREG_K1, imm); break;
        case 0x21: LH(rt, MIPSREG_K1, imm); break;
        case 0x22: LWL(rt, MIPSREG_K1, imm); break;
        case 0x23: LW(rt, MIPSREG_K1, imm); break;
        case 0x24: LBU(rt, MIPSREG_K1, imm); break;
        case 0x25: LHU(rt, MIPSREG_K1, imm); break;
        case 0x26: LWR(rt, MIPSREG_K1, imm); break;
    }
    native_instr_loadstore++;
}

/* Emit store with address translation */
static void emit_store(u32 opcode) {
    u32 op = OP_CODE(opcode);
    u32 rs = OP_RS(opcode);
    u32 rt = remap_reg(OP_RT(opcode));
    s16 imm = OP_IMM(opcode);

    NLOG3("emit_store op=%02X rs=%d rt=%d->%d imm=%d", op, rs, OP_RT(opcode), rt, imm);

    emit_addr_xlat(rs);

    switch (op) {
        case 0x28: SB(rt, MIPSREG_K1, imm); break;
        case 0x29: SH(rt, MIPSREG_K1, imm); break;
        case 0x2A: SWL(rt, MIPSREG_K1, imm); break;
        case 0x2B: SW(rt, MIPSREG_K1, imm); break;
        case 0x2E: SWR(rt, MIPSREG_K1, imm); break;
    }
    native_instr_loadstore++;
}

/* v126: Emit branch - PART 1: Save condition result BEFORE delay slot
 * Computes whether branch should be taken and saves result (0 or 1) to psxRegs.code
 * This must be called BEFORE the delay slot executes!
 *
 * For link branches (BLTZAL/BGEZAL), also sets $ra = PC + 8 BEFORE delay slot.
 */
static void emit_branch_save_condition(u32 opcode, u32 current_pc) {
    u32 op = OP_CODE(opcode);
    u32 rs = remap_reg(OP_RS(opcode));
    u32 rt = remap_reg(OP_RT(opcode));

    NLOG2("emit_branch_save op=%02X psx_rs=%d psx_rt=%d host_rs=%d host_rt=%d",
          op, OP_RS(opcode), OP_RT(opcode), rs, rt);

    /* Compute branch condition and save to psxRegs.code:
     * code = 1 if branch should be taken, 0 otherwise
     */

    switch (op) {
        case 0x04: /* BEQ: taken if rs == rt */
            /* XOR $k1, rs, rt -> k1=0 if equal */
            /* SLTIU $k1, $k1, 1 -> k1=1 if (xor < 1), i.e., if xor==0, i.e., if equal */
            XOR(MIPSREG_K1, rs, rt);
            SLTIU(MIPSREG_K1, MIPSREG_K1, 1);
            break;

        case 0x05: /* BNE: taken if rs != rt */
            /* XOR $k1, rs, rt -> k1!=0 if not equal */
            /* SLTU $k1, $zero, $k1 -> k1=1 if (0 < xor), i.e., if xor!=0, i.e., if not equal */
            XOR(MIPSREG_K1, rs, rt);
            SLTU(MIPSREG_K1, 0, MIPSREG_K1);
            break;

        case 0x06: /* BLEZ: taken if rs <= 0 (signed) */
            /* SLT $k1, $zero, rs -> k1=1 if (0 < rs), i.e., if rs > 0 */
            /* XORI $k1, $k1, 1 -> k1=1 if rs <= 0 */
            SLT(MIPSREG_K1, 0, rs);
            XORI(MIPSREG_K1, MIPSREG_K1, 1);
            break;

        case 0x07: /* BGTZ: taken if rs > 0 (signed) */
            /* SLT $k1, $zero, rs -> k1=1 if (0 < rs), i.e., if rs > 0 */
            SLT(MIPSREG_K1, 0, rs);
            break;

        case 0x01: /* REGIMM */
            {
                u32 regimm = OP_RT(opcode);

                if (regimm == 0x00) { /* BLTZ: taken if rs < 0 */
                    /* SLT $k1, rs, $zero -> k1=1 if (rs < 0) */
                    SLT(MIPSREG_K1, rs, 0);
                }
                else if (regimm == 0x01) { /* BGEZ: taken if rs >= 0 */
                    /* SLT $k1, rs, $zero -> k1=1 if (rs < 0) */
                    /* XORI $k1, $k1, 1 -> k1=1 if rs >= 0 */
                    SLT(MIPSREG_K1, rs, 0);
                    XORI(MIPSREG_K1, MIPSREG_K1, 1);
                }
                else if (regimm == 0x10) { /* BLTZAL: taken if rs < 0, link $ra = PC + 8 */
                    /* Link BEFORE delay slot! */
                    LI32(HOST_RA_REMAP, current_pc + 8);
                    SLT(MIPSREG_K1, rs, 0);
                }
                else if (regimm == 0x11) { /* BGEZAL: taken if rs >= 0, link $ra = PC + 8 */
                    /* Link BEFORE delay slot! */
                    LI32(HOST_RA_REMAP, current_pc + 8);
                    SLT(MIPSREG_K1, rs, 0);
                    XORI(MIPSREG_K1, MIPSREG_K1, 1);
                }
            }
            break;
    }

    /* Save condition result to psxRegs.code */
    SW(MIPSREG_K1, PERM_REG_1, off(code));
    NLOG2("emit_branch_save: condition saved to psxRegs.code");
}

/* v126: Emit branch - PART 2: Use saved condition AFTER delay slot
 * Loads the pre-computed branch result from psxRegs.code and decides PC
 */
static void emit_branch_use_saved(u32 opcode, u32 current_pc, u32 *end_label) {
    s16 offset = OP_IMM(opcode);
    u32 target_pc = current_pc + 4 + (offset << 2);
    u32 next_pc = current_pc + 8; /* PC after delay slot */

    NLOG2("emit_branch_use_saved target=0x%08X next=0x%08X", target_pc, next_pc);

    /* Load saved condition from psxRegs.code
     * If code != 0, branch is taken
     */
    LW(MIPSREG_K1, PERM_REG_1, off(code));

    /* If NOT taken (k1 == 0), skip to not-taken code */
    u32 *patch_not_taken = (u32*)recMem;
    BEQ(MIPSREG_K1, 0, 0);
    NOP();

    /* TAKEN: set PC = target, jump to block end */
    LI32(MIPSREG_K1, target_pc);
    SW(MIPSREG_K1, PERM_REG_1, off(pc));

    /* Jump to block end (will be patched) */
    *end_label = (u32)recMem;
    B(0);
    NOP();

    /* NOT TAKEN: patch the conditional branch */
    fixup_branch(patch_not_taken);

    /* Set PC = next */
    LI32(MIPSREG_K1, next_pc);
    SW(MIPSREG_K1, PERM_REG_1, off(pc));

    native_instr_branch++;
}

/* Emit J/JAL */
static void emit_jump(u32 opcode, u32 current_pc) {
    u32 op = OP_CODE(opcode);
    u32 target = (current_pc & 0xF0000000) | (OP_TARGET(opcode) << 2);

    NLOG2("emit_jump op=%02X target=0x%08X", op, target);

    if (op == 0x03) { /* JAL - link */
        LI32(HOST_RA_REMAP, current_pc + 8);
    }

    LI32(MIPSREG_K1, target);
    SW(MIPSREG_K1, PERM_REG_1, off(pc));

    native_instr_jump++;
}

/* Emit JR/JALR - PART 1: Save jump target BEFORE delay slot
 * v125: Must capture rs BEFORE delay slot can modify it!
 * Saves to psxRegs.code (not $k1, because delay slot load/store uses $k1)
 */
static void emit_jump_reg_save_target(u32 opcode, u32 current_pc) {
    u32 funct = OP_FUNCT(opcode);
    u32 rs = remap_reg(OP_RS(opcode));
    u32 rd = remap_reg(OP_RD(opcode));

    NLOG2("emit_jump_reg_save funct=%02X psx_rs=%d psx_rd=%d host_rs=%d -> saving to psxRegs.code",
          funct, OP_RS(opcode), OP_RD(opcode), rs);

    if (funct == 0x09) { /* JALR - link: set rd = PC+8 BEFORE delay slot */
        LI32(rd, current_pc + 8);
    }

    /* Save jump target to psxRegs.code (safe from delay slot's $k1 usage) */
    SW(rs, PERM_REG_1, off(code));
}

/* Emit JR/JALR - PART 2: Store saved target to PC AFTER delay slot
 * v125: Load from psxRegs.code (where we saved it before delay slot)
 */
static void emit_jump_reg_store_pc(void) {
    NLOG2("emit_jump_reg_store: PC = psxRegs.code (pre-saved target)");

    /* Load saved target from psxRegs.code and store to PC */
    LW(MIPSREG_K1, PERM_REG_1, off(code));
    SW(MIPSREG_K1, PERM_REG_1, off(pc));

    native_instr_jump++;
}

/* Emit COP2 trap */
static void emit_cop2_trap(u32 opcode, u32 live_mask) {
    u32 rs = OP_RS(opcode);
    u32 funct = OP_FUNCT(opcode);

    NLOG3("emit_cop2_trap rs=%02X funct=%02X", rs, funct);

    emit_save_gprs(live_mask);

    LI32(MIPSREG_A0, opcode);
    SW(MIPSREG_A0, PERM_REG_1, off(code));

    void *func = NULL;
    bool needs_arg = false;

    if (rs == 0x00) func = (void*)gteMFC2;
    else if (rs == 0x02) func = (void*)gteCFC2;
    else if (rs == 0x04) func = (void*)gteMTC2;
    else if (rs == 0x06) func = (void*)gteCTC2;
    else {
        switch (funct) {
            case 0x01: func = (void*)gteRTPS; break;
            case 0x06: func = (void*)gteNCLIP; break;
            case 0x0C: needs_arg = true; func = (void*)gteOP; break;
            case 0x10: needs_arg = true; func = (void*)gteDPCS; break;
            case 0x11: needs_arg = true; func = (void*)gteINTPL; break;
            case 0x12: needs_arg = true; func = (void*)gteMVMVA; break;
            case 0x13: func = (void*)gteNCDS; break;
            case 0x14: func = (void*)gteCDP; break;
            case 0x16: func = (void*)gteNCDT; break;
            case 0x1B: func = (void*)gteNCCS; break;
            case 0x1C: func = (void*)gteCC; break;
            case 0x1E: func = (void*)gteNCS; break;
            case 0x20: func = (void*)gteNCT; break;
            case 0x28: needs_arg = true; func = (void*)gteSQR; break;
            case 0x29: needs_arg = true; func = (void*)gteDCPL; break;
            case 0x2A: func = (void*)gteDPCT; break;
            case 0x2D: func = (void*)gteAVSZ3; break;
            case 0x2E: func = (void*)gteAVSZ4; break;
            case 0x30: func = (void*)gteRTPT; break;
            case 0x3D: needs_arg = true; func = (void*)gteGPF; break;
            case 0x3E: needs_arg = true; func = (void*)gteGPL; break;
            case 0x3F: func = (void*)gteNCCT; break;
        }
    }

    if (func) {
        if (needs_arg) {
            JAL(func);
            LI16(MIPSREG_A0, (u16)(opcode >> 10));
        } else {
            JAL(func);
            NOP();
        }
    }

    emit_load_gprs(live_mask);
    LW(MIPSREG_K0, PERM_REG_1, off(psxM));

    native_instr_cop_trap++;
}

/* Emit LWC2/SWC2 */
static void emit_lwc2(u32 opcode, u32 live_mask) {
    NLOG3("emit_lwc2 opcode=0x%08X", opcode);
    emit_save_gprs(live_mask);
    LI32(MIPSREG_A0, opcode);
    SW(MIPSREG_A0, PERM_REG_1, off(code));
    JAL(gteLWC2);
    NOP();
    emit_load_gprs(live_mask);
    LW(MIPSREG_K0, PERM_REG_1, off(psxM));
    native_instr_cop_trap++;
}

static void emit_swc2(u32 opcode, u32 live_mask) {
    NLOG3("emit_swc2 opcode=0x%08X", opcode);
    emit_save_gprs(live_mask);
    LI32(MIPSREG_A0, opcode);
    SW(MIPSREG_A0, PERM_REG_1, off(code));
    JAL(gteSWC2);
    NOP();
    emit_load_gprs(live_mask);
    LW(MIPSREG_K0, PERM_REG_1, off(psxM));
    native_instr_cop_trap++;
}

/* Emit COP0 inline */
static void emit_cop0(u32 opcode) {
    u32 rs = OP_RS(opcode);
    u32 rt = remap_reg(OP_RT(opcode));
    u32 rd = OP_RD(opcode);

    NLOG3("emit_cop0 rs=%02X rt=%d rd=%d", rs, rt, rd);

    switch (rs) {
        case 0x00: /* MFC0 */
            if (OP_RT(opcode) != 0) {
                LW(rt, PERM_REG_1, offCP0(rd));
            }
            break;
        case 0x04: /* MTC0 */
            if (OP_RT(opcode) != 0) {
                SW(rt, PERM_REG_1, offCP0(rd));
            } else {
                SW(0, PERM_REG_1, offCP0(rd));
            }
            break;
        case 0x10: /* RFE */
            LW(TEMP_1, PERM_REG_1, offCP0(12));
            SRL(TEMP_2, TEMP_1, 2);
            ANDI(TEMP_2, TEMP_2, 0x0F);
            ANDI(TEMP_1, TEMP_1, 0xFFF0);
            OR(TEMP_1, TEMP_1, TEMP_2);
            SW(TEMP_1, PERM_REG_1, offCP0(12));
            break;
    }
    native_instr_cop_trap++;
}

/* v123: Helper to emit a single instruction (for delay slots) */
static void emit_single_instruction(u32 opcode, u32 all_used) {
    u32 op = OP_CODE(opcode);

    if (op == 0x10) {
        emit_cop0(opcode);
    }
    else if (op == 0x12) {
        emit_cop2_trap(opcode, all_used);
    }
    else if (op == 0x32) {
        emit_lwc2(opcode, all_used);
    }
    else if (op == 0x3A) {
        emit_swc2(opcode, all_used);
    }
    else if (is_load(opcode)) {
        emit_load(opcode);
    }
    else if (is_store(opcode)) {
        emit_store(opcode);
    }
    else {
        /* ALU - remap and emit */
        if (uses_remapped_regs(opcode)) {
            if (op == 0x00) {
                write32(remap_r_type(opcode));
            } else {
                write32(remap_i_type(opcode));
            }
            native_instr_alu_remapped++;
        } else {
            write32(opcode);
            native_instr_alu_copy++;
        }
    }
}

/* ============== MAIN COMPILER ============== */

static bool native_compile_block(u32 start_pc, int required_count) {
    /* v247: ADDED - Log every native block attempt for debugging */
    xlog("NAT_TRY: PC=%08X cnt=%d\n", start_pc, required_count);

    /* v195: FIXED - now uses required_count from dynarec!
     * When required_count > 0, we compile EXACTLY that many instructions.
     * When required_count == 0, we use our own analysis (stops at branches).
     */
    static u32 attempt_count __attribute__((aligned(4))) = 0;
    attempt_count++;

    /* PC validation */
    bool pc_valid = (start_pc >= 0x80000000 && start_pc < 0x80200000) ||
                    (start_pc >= 0xBFC00000 && start_pc < 0xBFC80000) ||
                    (start_pc >= 0xA0000000 && start_pc < 0xA0200000) ||
                    (start_pc < 0x00200000);

    if (!pc_valid) {
        return false;
    }

    native_blocks_total++; nat_blocks_attempted++;

    /* v133: Reject blocks that START with SYSCALL/BREAK */
    u32 first_opcode = *(u32*)PSXM(start_pc);
    if (first_opcode == 0x0000000C) {  /* SYSCALL */
        return false;
    }
    if ((first_opcode & 0xFC00003F) == 0x0000000D) {  /* BREAK */
        return false;
    }

    /* === PHASE 1: ANALYZE === */
    u32 read_mask = 0, write_mask = 0;
    bool has_forbidden = false;
    int count = 0;
    bool in_delay = false;
    bool has_control_flow = false;

    /* v195: Determine max instructions to analyze */
    int max_instructions = (required_count > 0) ? required_count : NATIVE_MAX_BLOCK;

    u32 pc = start_pc;
    for (int i = 0; i < max_instructions; i++) {
        u32 opcode = *(u32*)PSXM(pc);

        /* v244: Check if instruction is enabled in menu - reject block if disabled */
        if (!native_cmd_check_enabled(opcode)) {
            return false;  /* Use dynarec for this block */
        }

        if (uses_forbidden_regs(opcode)) {
            has_forbidden = true;
        }

        /* Track register usage */
        u32 op = OP_CODE(opcode);
        u32 rs = OP_RS(opcode);
        u32 rt = OP_RT(opcode);
        u32 rd = OP_RD(opcode);
        u32 funct = OP_FUNCT(opcode);

        switch (op) {
            case 0x00:
                read_mask |= (1u << rs) | (1u << rt);
                write_mask |= (1u << rd);
                break;
            case 0x01: /* REGIMM */
                read_mask |= (1u << rs);
                if ((rt & 0x10)) write_mask |= (1u << 31);
                break;
            case 0x02: break; /* J */
            case 0x03: write_mask |= (1u << 31); break; /* JAL */
            case 0x04: case 0x05:
                read_mask |= (1u << rs) | (1u << rt);
                break;
            case 0x06: case 0x07:
                read_mask |= (1u << rs);
                break;
            case 0x08: case 0x09: case 0x0A: case 0x0B:
            case 0x0C: case 0x0D: case 0x0E:
                read_mask |= (1u << rs);
                write_mask |= (1u << rt);
                break;
            case 0x0F:
                write_mask |= (1u << rt);
                break;
            case 0x20: case 0x21: case 0x22: case 0x23:
            case 0x24: case 0x25: case 0x26:
                read_mask |= (1u << rs);
                write_mask |= (1u << rt);
                break;
            case 0x28: case 0x29: case 0x2A: case 0x2B: case 0x2E:
                read_mask |= (1u << rs) | (1u << rt);
                break;
            case 0x10: /* COP0 */
            case 0x12: /* COP2 */
                if (rs == 0x00 || rs == 0x02) write_mask |= (1u << rt);
                else if (rs == 0x04 || rs == 0x06) read_mask |= (1u << rt);
                break;
            case 0x32: case 0x3A:
                read_mask |= (1u << rs);
                break;
        }

        /* v215: SYSCALL ends the block (needs exception), BREAK is NOP */
        if (op == 0x00 && funct == 0x0C) {
            has_control_flow = true;
            break;
        }
        /* BREAK (funct=0x0D) is NOP in dynarec, so we continue */

        count++;
        pc += 4;

        /* v195: Only stop at branches if NO required_count given */
        if (required_count == 0) {
            if (in_delay) {
                has_control_flow = true;
                break;
            }
            if (is_branch(opcode) || is_jump(opcode)) in_delay = true;
        }
    }

    /* Skip blocks using $k0/$k1 */
    if (has_forbidden) {
        native_blocks_skipped_k0k1++; nat_rej_k0k1++;
        return false;
    }

    /* Clean masks */
    read_mask &= ~(1u << 0);
    write_mask &= ~(1u << 0);
    u32 all_used = read_mask | write_mask;

    /* v246: SP/FP/RA remapping collision detection
     * v233 changed mapping to: 29→28($gp), 30→24($t8), 31→25($t9)
     * Must reject blocks using BOTH a PSX reg AND its host target.
     */

    /* v246: Collision detection - fixed for new HOST_SP_REMAP=28 */
    bool collision = false;
    if ((all_used & (1u << 29)) && (all_used & (1u << 28))) collision = true;  /* sp->gp(28) - v246 FIX! */
    if ((all_used & (1u << 30)) && (all_used & (1u << 24))) collision = true;  /* fp→t8, uses t8 */
    if ((all_used & (1u << 31)) && (all_used & (1u << 25))) collision = true;  /* ra→t9, uses t9 */
    if (collision) {
        nat_rej_remap++;  /* Reuse existing counter for collision rejections */
        return false;
    }

    /* Detect garbage blocks */
    if (all_used == 0 && count > 2) { nat_rej_garbage++;
        return false;
    }

    /* v248: Log block with FULL opcodes for debugging */
    xlog("NAT_OK: PC=%08X cnt=%d regs=0x%08X\n", start_pc, count, all_used);
    /* Log ALL instructions in block with full details */
    for (int dbg_i = 0; dbg_i < count && dbg_i < 8; dbg_i++) {
        u32 dbg_opc = *(u32*)PSXM(start_pc + dbg_i * 4);
        u32 dbg_op = dbg_opc >> 26;
        u32 dbg_funct = dbg_opc & 0x3F;
        u32 dbg_rs = (dbg_opc >> 21) & 0x1F;
        u32 dbg_rt = (dbg_opc >> 16) & 0x1F;
        u32 dbg_rd = (dbg_opc >> 11) & 0x1F;
        if (dbg_op == 0) {
            xlog("  [%d] %08X=%08X R-type rs=%d rt=%d rd=%d funct=%02X\n",
                 dbg_i, start_pc + dbg_i * 4, dbg_opc, dbg_rs, dbg_rt, dbg_rd, dbg_funct);
        } else {
            xlog("  [%d] %08X=%08X op=%02X rs=%d rt=%d imm=%04X\n",
                 dbg_i, start_pc + dbg_i * 4, dbg_opc, dbg_op, dbg_rs, dbg_rt, dbg_opc & 0xFFFF);
        }
    }
    if (count > 8) xlog("  ... +%d more\n", count - 8);

    /* === v162: PHASE 2: PROLOGUE === */
    u32* code_start = recMem;

    LW(MIPSREG_K0, PERM_REG_1, off(psxM));
    emit_load_gprs(all_used);

    /* === PHASE 3: BODY === */
    /* v192: NOP + LUI + ALU + ALL LOADS + SW (logging disabled) */
    pc = start_pc;
    u32 end_labels[NATIVE_MAX_BLOCK];
    int num_end_labels = 0;
    bool block_sets_pc = false;

    for (int i = 0; i < count; i++) {
        u32 opcode = *(u32*)PSXM(pc);
        u32 op = opcode >> 26;
        u32 funct = opcode & 0x3F;

        if (opcode == 0) {
            /* NOP - just copy */
            write32(opcode);
        } else if (op == 0x0F) {
            /* LUI rt, imm - needs register remapping! */
            /* v194 FIX: LUI is I-type, must remap rt */
            if (uses_remapped_regs(opcode)) {
                write32(remap_i_type(opcode));
            } else {
                write32(opcode);
            }
        } else if (op == 0x00) {
            /* R-type: check funct */
            if (funct == 0x08) {
                /* v213: JR (jump register) - same logic as branches!
                 *
                 * v213 FIX: Check required_count like branches do!
                 * If JR is in MIDDLE of block (dynarec continued past it),
                 * emit NOP instead of jumping (dynarec skipped this JR).
                 * Only handle JR fully when it's at END of block.
                 *
                 * This fixes MISMATCH where native block was shorter than dynarec!
                 */
                u32 ds = *(u32*)PSXM(pc + 4);
                bool jr_at_end = (required_count > 0)
                               ? (i >= required_count - 2)  /* JR + delay = last 2 */
                               : true;  /* No required_count = always ends block */

                if (required_count > 0 && !jr_at_end) {
                    /* JR in MIDDLE - dynarec continued, so NOT taken. Emit NOP. */
                    write32(0);
                    pc += 4;  /* v216: Must increment pc before continue! */
                    continue;
                } else {
                    /* JR at END of block - full handling */
                    u32 rs_idx = OP_RS(opcode);

                    /* v230 FIX #6: Capture JR target from HOST register BEFORE delay slot!
                     *
                     * BUG: Old code loaded target from psxRegs.GPR[rs] AFTER delay slot.
                     * If rs was modified earlier in the native block, the HOST register
                     * has the correct value, but psxRegs.GPR[rs] has the STALE value
                     * (not yet saved back to memory).
                     *
                     * Example of buggy behavior:
                     *   ADDIU $ra, $zero, 0x1234   // HOST $t9 = 0x1234, psxRegs.GPR[31] = old
                     *   JR $ra                     // Should jump to 0x1234
                     *   NOP
                     *   // Old code: LW $k1, psxRegs.GPR[31] → loads OLD value!
                     *
                     * Verified in sources:
                     * - PSX-SPX: JR jumps to address in register rs
                     * - Raymond Chen: delay slot executes BEFORE branch takes effect
                     * - Univ. Delaware: "PC = contents of rs"
                     *
                     * FIX: Save HOST register to psxRegs.code BEFORE delay slot,
                     * then load from psxRegs.code AFTER delay slot.
                     */
                    u32 host_rs = remap_reg(rs_idx);

                    /* BEFORE delay slot: Save target address to psxRegs.code */
                    SW(host_rs, PERM_REG_1, off(code));

                    /* v212: Emit delay slot PROPERLY (not just NOP!) */
                    if (ds == 0) {
                        write32(0);  /* NOP delay slot */
                    } else {
                        /* Emit delay slot instruction with proper remapping */
                        u32 ds_op = ds >> 26;
                        if (ds_op == 0) {
                            /* R-type ALU */
                            if (uses_remapped_regs(ds)) {
                                write32(remap_r_type(ds));
                            } else {
                                write32(ds);
                            }
                        } else if (ds_op == 0x0F || (ds_op >= 0x08 && ds_op <= 0x0E)) {
                            /* I-type: LUI or ALU */
                            if (uses_remapped_regs(ds)) {
                                write32(remap_i_type(ds));
                            } else {
                                write32(ds);
                            }
                        } else {
                            /* v231 FIX #3: Reject LOAD/STORE/BRANCH/JUMP in JR delay slot!
                             *
                             * BUG: Previous code emitted these instructions "as-is" without
                             * any address translation. Native LOAD/STORE requires:
                             * - Address masking (& 0x1FFFFF for RAM)
                             * - psxM base addition
                             * - Hardware address detection
                             *
                             * MIPS Delay Slot Rules (verified in sources):
                             * - LOAD/STORE: Technically legal, but require full translation
                             * - BRANCH/JUMP: ILLEGAL! Causes UNDEFINED behavior
                             *
                             * Opcodes to reject:
                             *   0x01: REGIMM (BLTZ/BGEZ/BLTZAL/BGEZAL)
                             *   0x02-0x03: J/JAL
                             *   0x04-0x07: BEQ/BNE/BLEZ/BGTZ
                             *   0x20-0x26: LB/LH/LWL/LW/LBU/LHU/LWR
                             *   0x28-0x2E: SB/SH/SWL/SW/SWR
                             *   0x30-0x3E: LWCz/SWCz (coprocessor load/store)
                             *
                             * Sources:
                             * - Wikipedia "Delay slot": branch in delay slot = UNDEFINED
                             * - jsgroth blog: PS1 CPU load delay slot behavior
                             * - MIPS IV ISA: load/store need address calculation
                             */
                            bool reject_ds = false;

                            /* Check for LOAD instructions (0x20-0x26, 0x30-0x32) */
                            if ((ds_op >= 0x20 && ds_op <= 0x26) ||
                                (ds_op >= 0x30 && ds_op <= 0x32)) {
                                reject_ds = true;
                            }
                            /* Check for STORE instructions (0x28-0x2E, 0x38-0x3A) */
                            else if ((ds_op >= 0x28 && ds_op <= 0x2E) ||
                                     (ds_op >= 0x38 && ds_op <= 0x3A)) {
                                reject_ds = true;
                            }
                            /* Check for BRANCH instructions (0x01, 0x04-0x07) */
                            else if (ds_op == 0x01 || (ds_op >= 0x04 && ds_op <= 0x07)) {
                                reject_ds = true;
                            }
                            /* Check for JUMP instructions (0x02-0x03) */
                            else if (ds_op == 0x02 || ds_op == 0x03) {
                                reject_ds = true;
                            }
                            /* Check for COP instructions (0x10, 0x11, 0x12, 0x13) */
                            else if (ds_op >= 0x10 && ds_op <= 0x13) {
                                reject_ds = true;
                            }

                            if (reject_ds) {
                                /* Reject block - let dynarec handle */
                                nat_rej_delay++;
                                recMem = code_start;
                                return false;
                            }

                            /* Safe "other" instruction - emit as-is */
                            write32(ds);
                        }
                    }

                    /* AFTER delay slot: Load saved target and store to psxRegs.pc */
                    LW(MIPSREG_K1, PERM_REG_1, off(code));
                    SW(MIPSREG_K1, PERM_REG_1, off(pc));

                    /* Block ends at JR */
                    count = i + 2;  /* JR + delay slot */
                    block_sets_pc = true;
                    goto emit_epilogue;
                }
            } else if (funct == 0x09 || funct == 0x0C) {
                /* JALR, SYSCALL - reject (need special handling) */
                if(funct==0x09)nat_rej_jalr++; else nat_rej_syscall++;
                recMem = code_start;
                return false;
            } else if (funct == 0x0D) {
                /* v215: BREAK = NOP (dynarec recBREAK() is empty function) */
                write32(0);
                pc += 4;  /* v216: Must increment pc before continue! */
                continue;
            } else if (funct >= 0x10 && funct <= 0x1B) {
                /* v230 FIX #5: Reject MULT/DIV/MFHI/MFLO/MTHI/MTLO!
                 *
                 * BUG: These instructions were falling through to "Valid ALU R-type"
                 * and being emitted as native MIPS. This is WRONG because:
                 *
                 * MFHI (0x10), MTHI (0x11), MFLO (0x12), MTLO (0x13):
                 *   - Access HI/LO registers which are NOT synced with psxRegs.GPR[32/33]
                 *   - MTHI/MTLO have "temperamental" behavior - first write destroys BOTH
                 *   - Source: Raymond Chen "The Old New Thing" (MS blog)
                 *
                 * MULT (0x18), MULTU (0x19), DIV (0x1A), DIVU (0x1B):
                 *   - Write to HI/LO registers which are NOT saved to psxRegs
                 *   - CPU stalls if MFHI/MFLO called before completion
                 *   - Source: PSX-SPX cpuspecifications
                 *
                 * Funct codes:
                 *   0x10 = MFHI, 0x11 = MTHI, 0x12 = MFLO, 0x13 = MTLO
                 *   0x18 = MULT, 0x19 = MULTU, 0x1A = DIV, 0x1B = DIVU
                 *
                 * Note: 0x14-0x17 are unused/reserved, but reject them too for safety.
                 */
                nat_rej_other++;  /* Count MULT/DIV/MFHI/MFLO/MTHI/MTLO rejections */
                recMem = code_start;
                return false;
            }
            /* Valid ALU R-type - copy with possible remap */
            if (uses_remapped_regs(opcode)) {
                write32(remap_r_type(opcode));
            } else {
                write32(opcode);
            }
        } else if (op == 0x08 || op == 0x09 || op == 0x0A || op == 0x0B ||
                   op == 0x0C || op == 0x0D || op == 0x0E) {
            /* I-type ALU: ADDI, ADDIU, SLTI, SLTIU, ANDI, ORI, XORI */
            if (uses_remapped_regs(opcode)) {
                write32(remap_i_type(opcode));
            } else {
                write32(opcode);
            }
        } else if (op == 0x20 || op == 0x21 || op == 0x23 || op == 0x24 || op == 0x25) {
            /* v199 FIX: REJECT loads when NATIVE_ALU_ONLY=1 */
#if NATIVE_ALU_ONLY
            recMem = code_start;
            return false;  /* Let dynarec handle this block */
#endif
            /* v191: All basic loads - LB/LH/LW/LBU/LHU */
            u32 rt = OP_RT(opcode);
            u32 rs = OP_RS(opcode);
            s16 imm = OP_IMM(opcode);

            if (rt == 0) {
                /* Load to $zero - NOP */
                write32(0);
            } else {
                /* v235 FIX: Hardware address check WITHOUT clobbering dst!
                 *
                 * v230 BUG: Used dst as temp for physical address computation.
                 * When hardware address detected (skip path), dst remained with
                 * physical address value instead of loaded data!
                 *
                 * In DIFF_TEST mode:
                 * - emit_load_gprs loaded regs_before into host registers
                 * - If hardware skip, dst should keep that value (unchanged)
                 * - Old code: dst = physical address = WRONG!
                 *
                 * v235 FIX: Never modify dst before we're sure it's RAM.
                 * Use $k1 for ALL temp calculations, store to psxRegs.code if needed.
                 * If hardware address: dst unchanged (keeps emit_load_gprs value)
                 * If RAM address: inline load into dst
                 *
                 * PSX Memory Map (physical = PSX_addr & 0x1FFFFFFF):
                 *   0x00000000-0x001FFFFF: Main RAM (2MB) - inline OK
                 *   0x1F800000+: Scratchpad/Hardware I/O - NOT inline
                 *   0x1FC00000+: BIOS ROM - NOT inline
                 *
                 * Check: if (physical >> 21) != 0, it's not RAM (>= 2MB)
                 */
                u32 host_rs = remap_reg(rs);
                u32 dst = remap_reg(rt);

                /* Step 1: Compute full PSX address */
                ADDIU(MIPSREG_K1, host_rs, imm);  /* $k1 = base + imm */

                /* Step 2: Strip segment bits (& 0x1FFFFFFF) */
                SLL(MIPSREG_K1, MIPSREG_K1, 3);
                SRL(MIPSREG_K1, MIPSREG_K1, 3);   /* $k1 = physical address */

                /* Step 3: Save physical to psxRegs.code (temp storage) */
                SW(MIPSREG_K1, PERM_REG_1, off(code));

                /* Step 4: Check if RAM: (physical >> 21) == 0 means < 0x200000 */
                SRL(MIPSREG_K1, MIPSREG_K1, 21);

                /* Step 5: If NOT RAM (k1 != 0), skip load - dst unchanged! */
                u32 *patch_skip = (u32*)recMem;
                BNE(MIPSREG_K1, 0, 0);            /* if hardware, skip */
                NOP();                             /* delay slot */

                /* Step 6: RAM path - inline load */
                LW(MIPSREG_K1, PERM_REG_1, off(code));  /* restore physical */
                ADDU(MIPSREG_K1, MIPSREG_K0, MIPSREG_K1);  /* $k1 = psxM + physical */

                switch (op) {
                    case 0x20: LB(dst, MIPSREG_K1, 0); break;   /* LB */
                    case 0x21: LH(dst, MIPSREG_K1, 0); break;   /* LH */
                    case 0x23: LW(dst, MIPSREG_K1, 0); break;   /* LW */
                    case 0x24: LBU(dst, MIPSREG_K1, 0); break;  /* LBU */
                    case 0x25: LHU(dst, MIPSREG_K1, 0); break;  /* LHU */
                }

                /* Step 7: Patch skip branch to here
                 * Hardware path: dst unchanged (has value from emit_load_gprs)
                 * RAM path: dst has loaded data
                 */
                fixup_branch(patch_skip);
            }
        } else if (op == 0x28 || op == 0x29 || op == 0x2B) {
            /* v229: VALIDATION-ONLY STORE - compare native value with dynarec result!
             *
             * How it works:
             * - Dynarec already executed and wrote to memory
             * - Native computes address+value, calls psxMemWriteXX_validate
             * - Validate functions READ memory (dynarec's result) and COMPARE
             * - If different = MISMATCH logged with exact address/values
             * - Functions DON'T write - memory stays with dynarec's value
             *
             * This finds the EXACT buggy STORE without 2MB checksum overhead!
             */
            /* v223: FIXED JAL to psxMemWrite!
             *
             * v220 BUGS that caused instant freeze:
             * 1. $ra NOT SAVED - JAL clobbers $ra, breaks return to dispatcher!
             * 2. Value clobbering - if src=$a0, it gets overwritten when setting args
             *    Example: SW $a0, 0($a1)
             *    - MOV $a0, $k1      → $a0 = address (clobbers value!)
             *    - MOV $a1, $a0      → $a1 = address (WRONG! should be value)
             *
             * v223 FIXES:
             * 1. Save value to $k0 FIRST, before ANY register modification
             * 2. Save $ra along with other caller-saved registers
             * 3. Set up arguments from $k0/$k1 (kernel regs, safe from clobbering)
             *
             * psxMemWrite handles address translation internally (0x80xxxxxx etc.)
             */
            u32 rt = OP_RT(opcode);
            u32 rs = OP_RS(opcode);
            s16 imm = OP_IMM(opcode);

            u32 host_rs = remap_reg(rs);
            u32 src = remap_reg(rt);

            /* v223 FIX #1: Save VALUE to $k0 FIRST, before anything!
             * This prevents clobbering if src is $a0, $a1, or any caller-saved reg.
             */
            MOV(MIPSREG_K0, src);  /* $k0 = value (SAFE!) */

            /* Compute address to $k1 (PSX virtual address, e.g. 0x80012345) */
            ADDIU(MIPSREG_K1, host_rs, imm);  /* $k1 = address */

            /* v223 FIX #2: Save caller-saved registers INCLUDING $ra!
             * Stack: 72 bytes (17 regs + 4 padding for 8-byte alignment)
             * Without saving $ra, the JR $ra at block end jumps to wrong place!
             */
            ADDIU(MIPSREG_SP, MIPSREG_SP, -72);
            SW(2, MIPSREG_SP, 0);   /* $v0 */
            SW(3, MIPSREG_SP, 4);   /* $v1 */
            SW(4, MIPSREG_SP, 8);   /* $a0 */
            SW(5, MIPSREG_SP, 12);  /* $a1 */
            SW(6, MIPSREG_SP, 16);  /* $a2 */
            SW(7, MIPSREG_SP, 20);  /* $a3 */
            SW(8, MIPSREG_SP, 24);  /* $t0 */
            SW(9, MIPSREG_SP, 28);  /* $t1 */
            SW(10, MIPSREG_SP, 32); /* $t2 */
            SW(11, MIPSREG_SP, 36); /* $t3 */
            SW(12, MIPSREG_SP, 40); /* $t4 */
            SW(13, MIPSREG_SP, 44); /* $t5 */
            SW(14, MIPSREG_SP, 48); /* $t6 */
            SW(15, MIPSREG_SP, 52); /* $t7 */
            SW(24, MIPSREG_SP, 56); /* $t8 */
            SW(25, MIPSREG_SP, 60); /* $t9 */
            SW(31, MIPSREG_SP, 64); /* $ra - CRITICAL FIX! */

            /* v223 FIX #3: Set up arguments from SAFE $k0/$k1
             * $k0/$k1 are kernel-reserved, won't be touched by save sequence
             */
            MOV(MIPSREG_A0, MIPSREG_K1);  /* $a0 = address (from $k1) */
            MOV(MIPSREG_A1, MIPSREG_K0);  /* $a1 = value (from $k0) */

            /* v224: Use JALR instead of JAL!
             * JAL has 256MB region limitation (uses PC[31:28] + 28-bit addr).
             * If native code and psxMemWrite are in different 256MB regions,
             * JAL jumps to wrong address!
             * JALR uses full 32-bit address from register - no limitation.
             *
             * Load function address into $t0 (already saved on stack).
             */
            /* v229: Use validation wrappers - compare native vs dynarec values */
            switch (op) {
                case 0x28:  /* SB */
                    LI32(8, (u32)psxMemWrite8_validate);   /* $t0 = validate func */
                    JALR_RA(8);                            /* JALR $ra, $t0 */
                    NOP();
                    break;
                case 0x29:  /* SH */
                    LI32(8, (u32)psxMemWrite16_validate);  /* $t0 = validate func */
                    JALR_RA(8);                            /* JALR $ra, $t0 */
                    NOP();
                    break;
                case 0x2B:  /* SW */
                    LI32(8, (u32)psxMemWrite32_validate);  /* $t0 = validate func */
                    JALR_RA(8);                            /* JALR $ra, $t0 */
                    NOP();
                    break;
            }

            /* Restore caller-saved registers INCLUDING $ra */
            LW(2, MIPSREG_SP, 0);   /* $v0 */
            LW(3, MIPSREG_SP, 4);   /* $v1 */
            LW(4, MIPSREG_SP, 8);   /* $a0 */
            LW(5, MIPSREG_SP, 12);  /* $a1 */
            LW(6, MIPSREG_SP, 16);  /* $a2 */
            LW(7, MIPSREG_SP, 20);  /* $a3 */
            LW(8, MIPSREG_SP, 24);  /* $t0 */
            LW(9, MIPSREG_SP, 28);  /* $t1 */
            LW(10, MIPSREG_SP, 32); /* $t2 */
            LW(11, MIPSREG_SP, 36); /* $t3 */
            LW(12, MIPSREG_SP, 40); /* $t4 */
            LW(13, MIPSREG_SP, 44); /* $t5 */
            LW(14, MIPSREG_SP, 48); /* $t6 */
            LW(15, MIPSREG_SP, 52); /* $t7 */
            LW(24, MIPSREG_SP, 56); /* $t8 */
            LW(25, MIPSREG_SP, 60); /* $t9 */
            LW(31, MIPSREG_SP, 64); /* $ra - CRITICAL FIX! */
            ADDIU(MIPSREG_SP, MIPSREG_SP, 72);

            /* v230 FIX #1: Restore $k0 after STORE!
             *
             * BUG: Line 1208 uses MOV(MIPSREG_K0, src) to save store value,
             * which OVERWRITES $k0 (psxM base pointer).
             * After STORE completes, $k0 still contains the STORE VALUE,
             * not psxM pointer. Any subsequent LOAD would use wrong base!
             *
             * Verified in 3 sources:
             * - emit_cop2_trap() restores $k0 after function calls
             * - emit_lwc2/swc2() restore $k0 after function calls
             * - PSX-SPX: psxM is base pointer for all RAM access
             *
             * FIX: Reload $k0 = psxM after stack restore.
             */
            LW(MIPSREG_K0, PERM_REG_1, off(psxM));

        } else if (op == 0x04 || op == 0x05 || op == 0x06 || op == 0x07) {
            /* v214 FIX: Branches - BEQ/BNE/BLEZ/BGTZ
             *
             * v214: Use required_count for branch_at_end check!
             * Same fix as JR/J - when required_count > 0, use it to determine
             * if branch is at end of the block dynarec compiled.
             */
            u32 ds = *(u32*)PSXM(pc + 4);
            bool branch_at_end = (required_count > 0)
                               ? (i >= required_count - 2)
                               : (i == count - 2);

            if (required_count > 0 && !branch_at_end) {
                /* Branch in MIDDLE - dynarec continued, so NOT taken. Emit NOP. */
                write32(0);
                pc += 4;  /* v216: Must increment pc before continue! */
                continue;
            } else {
                /* Normal mode: actually handle the branch */

                /* For now, only accept NOP delay slots for safety */
                if (ds != 0) {
                    /* Non-NOP delay slot */ nat_rej_delay++;
                    recMem = code_start;
                    return false;
                }

                u32 rs = remap_reg(OP_RS(opcode));
                u32 rt = remap_reg(OP_RT(opcode));
                s16 offset = OP_IMM(opcode);
                u32 target_pc = pc + 4 + (offset << 2);
                u32 next_pc = pc + 8;  /* After delay slot */

                /* Compute condition result in $k1:
                 * $k1 = 1 if branch taken, 0 if not taken
                 */
                switch (op) {
                    case 0x04: /* BEQ: taken if rs == rt */
                        XOR(MIPSREG_K1, rs, rt);
                        SLTIU(MIPSREG_K1, MIPSREG_K1, 1);  /* k1=1 if xor==0 */
                        break;
                    case 0x05: /* BNE: taken if rs != rt */
                        XOR(MIPSREG_K1, rs, rt);
                        SLTU(MIPSREG_K1, 0, MIPSREG_K1);   /* k1=1 if xor!=0 */
                        break;
                    case 0x06: /* BLEZ: taken if rs <= 0 (signed) */
                        SLT(MIPSREG_K1, 0, rs);            /* k1=1 if 0<rs */
                        XORI(MIPSREG_K1, MIPSREG_K1, 1);   /* k1=1 if rs<=0 */
                        break;
                    case 0x07: /* BGTZ: taken if rs > 0 (signed) */
                        SLT(MIPSREG_K1, 0, rs);            /* k1=1 if rs>0 */
                        break;
                }

                /* Emit delay slot (NOP) */
                write32(0);

                /* If NOT taken (k1 == 0), skip to not-taken path */
                u32 *patch_not_taken = (u32*)recMem;
                BEQ(MIPSREG_K1, 0, 0);  /* Placeholder, will patch */
                NOP();

                /* TAKEN: set PC = target */
                LI32(MIPSREG_K1, target_pc);
                SW(MIPSREG_K1, PERM_REG_1, off(pc));

                /* Jump to epilogue */
                end_labels[num_end_labels++] = (u32)recMem;
                B(0);  /* Placeholder, will patch */
                NOP();

                /* NOT TAKEN: patch the BEQ above to point here */
                fixup_branch(patch_not_taken);

                /* NOT TAKEN: set PC = next_pc */
                LI32(MIPSREG_K1, next_pc);
                SW(MIPSREG_K1, PERM_REG_1, off(pc));

                /* Block ends with branch */
                count = i + 2;  /* Include branch + delay slot */
                block_sets_pc = true;
                goto emit_epilogue;
            }

        } else if (op == 0x01) {
            /* v214: REGIMM conditional branches (BLTZ, BGEZ, BLTZAL, BGEZAL)
             *
             * v214: Use required_count for branch_at_end check!
             * Same fix as JR/J/BEQ/BNE - when required_count > 0, use it.
             *
             * IMPORTANT: BLTZAL/BGEZAL ALWAYS set $ra = PC+8, even if not taken!
             */
            u32 rt_field = OP_RT(opcode);
            u32 ds = *(u32*)PSXM(pc + 4);
            bool branch_at_end = (required_count > 0)
                               ? (i >= required_count - 2)
                               : (i == count - 2);

            /* Handle link variants - $ra = PC+8 regardless of taken/not-taken
             *
             * v231 FIX #2: BLTZAL/BGEZAL link to HOST register!
             *
             * BUG: Previous code wrote link address (PC+8) ONLY to psxRegs.GPR[31]
             * (memory), but NOT to host register $t9 (HOST_RA_REMAP).
             * If subsequent instructions in this block read $ra, they use the
             * HOST register $t9, which still has the OLD value!
             *
             * Example of buggy behavior:
             *   BGEZAL $t0, label      ; Should set $ra = PC+8
             *   NOP
             *   MOV $t1, $ra           ; Native uses HOST $t9 = OLD value!
             *
             * MIPS Specification (verified in 3 sources):
             * - MIPS IV ISA Manual: "place address of instruction following
             *   delay slot into link register ($ra)"
             * - PSX-SPX: BLTZAL/BGEZAL "save return address in $ra"
             * - MIPS Instruction Set (Harvard): "$ra unconditionally modified"
             *
             * FIX: Load link address into HOST_RA_REMAP ($t9), then store
             * to psxRegs.GPR[31]. Now both are synchronized!
             */
            if (rt_field == 0x10 || rt_field == 0x11) {  /* BLTZAL or BGEZAL */
                u32 ra_value = pc + 8;
                /* v231: Update HOST register first! */
                LI32(HOST_RA_REMAP, ra_value);           /* HOST $t9 = PC+8 */
                SW(HOST_RA_REMAP, PERM_REG_1, offGPR(31)); /* psxRegs.$ra = PC+8 */
            }

            if (required_count > 0 && !branch_at_end) {
                /* Branch in MIDDLE - dynarec continued, so NOT taken. Emit NOP. */
                write32(0);
                pc += 4;  /* v216: Must increment pc before continue! */
                continue;
            } else {
                /* Branch at END or normal mode - fully handle the branch */

                /* For now, only accept NOP delay slots for safety */
                if (ds != 0) {
                    nat_rej_delay++;
                    recMem = code_start;
                    return false;
                }

                u32 rs = remap_reg(OP_RS(opcode));
                s16 offset = OP_IMM(opcode);
                u32 target_pc = pc + 4 + (offset << 2);
                u32 next_pc = pc + 8;  /* After delay slot */

                /* Compute condition in $k1: 1 if taken, 0 if not */
                if (rt_field == 0x00 || rt_field == 0x10) {
                    /* BLTZ / BLTZAL: taken if rs < 0 */
                    SLT(MIPSREG_K1, rs, 0);
                } else {
                    /* BGEZ / BGEZAL: taken if rs >= 0 */
                    SLT(MIPSREG_K1, rs, 0);
                    XORI(MIPSREG_K1, MIPSREG_K1, 1);
                }

                /* Emit delay slot (NOP) */
                write32(0);

                /* If NOT taken (k1 == 0), skip to not-taken path */
                u32 *patch_not_taken = (u32*)recMem;
                BEQ(MIPSREG_K1, 0, 0);  /* Placeholder */
                NOP();

                /* TAKEN: set PC = target */
                LI32(MIPSREG_K1, target_pc);
                SW(MIPSREG_K1, PERM_REG_1, off(pc));

                /* Jump to epilogue */
                end_labels[num_end_labels++] = (u32)recMem;
                B(0);
                NOP();

                /* NOT TAKEN: patch the BEQ */
                fixup_branch(patch_not_taken);

                /* NOT TAKEN: set PC = next_pc */
                LI32(MIPSREG_K1, next_pc);
                SW(MIPSREG_K1, PERM_REG_1, off(pc));

                /* Block ends with branch */
                count = i + 2;
                block_sets_pc = true;
                goto emit_epilogue;
            }
        } else if (op == 0x02) {
            /* v213: J (unconditional jump) - same logic as branches/JR!
             * Check required_count - if J is in MIDDLE, emit NOP.
             */
            u32 ds = *(u32*)PSXM(pc + 4);
            bool j_at_end = (required_count > 0)
                          ? (i >= required_count - 2)
                          : true;

            if (required_count > 0 && !j_at_end) {
                /* J in MIDDLE - dynarec continued, so NOT taken. Emit NOP. */
                write32(0);
                pc += 4;  /* v216: Must increment pc before continue! */
                continue;
            } else {
                /* J at END of block - full handling */
                if (ds != 0) { nat_rej_delay++; recMem = code_start; return false; }
                u32 target = (opcode & 0x03FFFFFF) << 2;
                u32 target_pc = (pc & 0xF0000000) | target;
                write32(0);  /* emit delay slot as NOP */
                LI32(MIPSREG_K1, target_pc);
                SW(MIPSREG_K1, PERM_REG_1, off(pc));
                count = i + 2;  /* Include J + delay slot */
                block_sets_pc = true;
                goto emit_epilogue;
            }
        } else {
            /* Unsupported opcode - reject and let dynarec handle */
            /* v205: detailed */ if(op==0x03)nat_rej_jal++; else if(op==0x10)nat_rej_cop0++; else if(op==0x12)nat_rej_gte++; else if(op==0x32)nat_rej_lwc2++; else if(op==0x3A)nat_rej_swc2++; else if(op==0x22)nat_rej_lwl++; else if(op==0x26)nat_rej_lwr++; else if(op==0x2A)nat_rej_swl++; else if(op==0x2E)nat_rej_swr++; else if(op==0x01)nat_rej_regimm++; else nat_rej_other++;
            recMem = code_start;
            return false;
        }

        pc += 4;
    }

emit_epilogue:

    (void)end_labels;
    (void)num_end_labels;

    /* === PHASE 4: EPILOGUE === */
    for (int i = 0; i < num_end_labels; i++) {
        fixup_branch((u32*)end_labels[i]);
    }

    emit_save_gprs(write_mask);

    /* Set $v1 = cycle count for recFunc
     * v237 FIX: Must use ADJUST_CLOCK like dynarec does!
     * Without this, native blocks return half the cycles (cycle_multiplier=0x200).
     */
    u32 adjusted_cycles = ADJUST_CLOCK(count);
    if (adjusted_cycles <= 0xFFFF) {
        ORI(MIPSREG_V1, 0, adjusted_cycles);
    } else {
        LI32(MIPSREG_V1, adjusted_cycles);
    }

    /* Set fallthrough PC if block doesn't end with branch/jump */
    if (!block_sets_pc) {
        LI32(MIPSREG_K1, pc);
        SW(MIPSREG_K1, PERM_REG_1, off(pc));
    }

    /* Return to dispatcher */
    LW(MIPSREG_V0, PERM_REG_1, off(pc));
    JR(MIPSREG_RA);
    NOP();

    /* v166: FULL NATIVE EXECUTION! */
    native_blocks_compiled++;
    native_used++;

    /* v247: Log successful native compilation */
    xlog("NAT_DONE: PC=%08X size=%d bytes\n", start_pc, (u32)((u8*)recMem - (u8*)code_start));

    /* v192: Stats now shown in profiler overlay (no xlog) */

    /* Cache flush - CRITICAL on MIPS!
     * D-cache and I-cache are separate. After writing code to memory,
     * we must flush D-cache and invalidate I-cache.
     */
    u32 code_size = (u32)((u8*)recMem - (u8*)code_start);
#ifdef __mips__
    __builtin___clear_cache((char*)code_start, (char*)recMem);
    __sync_synchronize();
#endif

    (void)has_control_flow;
    (void)code_size;

    /* NO recMem reset - we're using this code! */
    return true;

#if 0  /* ===== DISABLED - OLD CODE ===== */

    native_blocks_total++; nat_blocks_attempted++;
    native_log_count++;

    NLOG1("=== NATIVE BLOCK #%d PC=0x%08X ===", native_blocks_total, start_pc);

    /* v154: PC already validated above */

    /* v133: CRITICAL - reject blocks that START with SYSCALL/BREAK!
     * If block starts at SYSCALL, we'd create empty block (0 instructions)
     * with fallthrough PC = start PC = infinite loop!
     * Dynarec handles these properly with exception mechanism.
     */
    u32 first_opcode = *(u32*)PSXM(start_pc);
    if (first_opcode == 0x0000000C) {  /* SYSCALL */
        NLOG1("=== REJECT (starts with SYSCALL) PC=0x%08X ===", start_pc);
        return false;
    }
    if ((first_opcode & 0xFC00003F) == 0x0000000D) {  /* BREAK */
        NLOG1("=== REJECT (starts with BREAK) PC=0x%08X ===", start_pc);
        return false;
    }

    /* === PHASE 1: ANALYZE === */
    NLOG2("PHASE 1: ANALYZE");

    u32 read_mask = 0, write_mask = 0;
    bool has_forbidden = false;
    int count = 0;
    bool in_delay = false;
    bool has_control_flow = false;  /* v124: track if block ends with branch/jump */

    u32 pc = start_pc;
    for (int i = 0; i < NATIVE_MAX_BLOCK; i++) {
        u32 opcode = *(u32*)PSXM(pc);

        NLOG3("  [%d] PC=0x%08X OP=0x%08X", i, pc, opcode);

        if (uses_forbidden_regs(opcode)) {
            has_forbidden = true;
            NLOG2("  -> FORBIDDEN REG at PC=0x%08X", pc);
        }

        /* Track register usage */
        u32 op = OP_CODE(opcode);
        u32 rs = OP_RS(opcode);
        u32 rt = OP_RT(opcode);
        u32 rd = OP_RD(opcode);
        u32 funct = OP_FUNCT(opcode);

        switch (op) {
            case 0x00:
                read_mask |= (1u << rs) | (1u << rt);
                write_mask |= (1u << rd);
                break;
            case 0x01: /* REGIMM */
                read_mask |= (1u << rs);
                if ((rt & 0x10)) write_mask |= (1u << 31); /* link */
                break;
            case 0x02: break; /* J */
            case 0x03: write_mask |= (1u << 31); break; /* JAL */
            case 0x04: case 0x05:
                read_mask |= (1u << rs) | (1u << rt);
                break;
            case 0x06: case 0x07:
                read_mask |= (1u << rs);
                break;
            case 0x08: case 0x09: case 0x0A: case 0x0B:
            case 0x0C: case 0x0D: case 0x0E:
                read_mask |= (1u << rs);
                write_mask |= (1u << rt);
                break;
            case 0x0F:
                write_mask |= (1u << rt);
                break;
            case 0x20: case 0x21: case 0x22: case 0x23:
            case 0x24: case 0x25: case 0x26:
                read_mask |= (1u << rs);
                write_mask |= (1u << rt);
                break;
            case 0x28: case 0x29: case 0x2A: case 0x2B: case 0x2E:
                read_mask |= (1u << rs) | (1u << rt);
                break;
            case 0x10: /* COP0 */
            case 0x12: /* COP2 */
                if (rs == 0x00 || rs == 0x02) write_mask |= (1u << rt);
                else if (rs == 0x04 || rs == 0x06) read_mask |= (1u << rt);
                break;
            case 0x32: case 0x3A:
                read_mask |= (1u << rs);
                break;
        }

        /* v128: SYSCALL and BREAK must end the block!
         * These instructions trigger exceptions and cannot be executed natively.
         * We end the block BEFORE the SYSCALL/BREAK so dynarec handles it.
         */
        if (op == 0x00 && (funct == 0x0C || funct == 0x0D)) {
            NLOG2("  -> SYSCALL/BREAK at PC=0x%08X - ending block here", pc);
            /* Don't include this instruction - let dynarec handle it */
            has_control_flow = true;  /* Block ends here */
            break;
        }

        count++;
        pc += 4;

        if (in_delay) {
            has_control_flow = true;  /* v124: block ends with control flow */
            break;
        }
        if (is_branch(opcode) || is_jump(opcode)) in_delay = true;
    }

    NLOG2("  count=%d read=0x%08X write=0x%08X forbidden=%d ctl_flow=%d",
          count, read_mask, write_mask, has_forbidden, has_control_flow);

    /* Only skip for $k0/$k1 */
    if (has_forbidden) {
        native_blocks_skipped_k0k1++; nat_rej_k0k1++;
        NLOG1("=== SKIP (k0/k1) ===");
        return false;
    }

    /* Clean masks */
    read_mask &= ~(1u << 0);
    write_mask &= ~(1u << 0);
    u32 all_used = read_mask | write_mask;

    /* v131: TEMPORARY - Skip blocks that need register remapping
     * $sp(29), $fp(30), $ra(31) are remapped to $s0, $s1, $s2
     * But this corrupts host $ra which we need to return to dispatcher!
     * Skip these blocks until we fix the remapping properly.
     */
    const u32 REMAPPED_MASK = (1u << 29) | (1u << 30) | (1u << 31);
    if (all_used & REMAPPED_MASK) { nat_rej_remap++;
        NLOG1("=== SKIP (needs remapping: sp/fp/ra) ===");
        return false;  /* Let dynarec handle this block */
    }

    /* v125: Detect garbage blocks - real code ALWAYS uses some registers!
     * If all_used=0 after analysis, block is probably NOPs or garbage.
     */
    if (all_used == 0 && count > 2) { nat_rej_garbage++;
        NLOG1("=== REJECT GARBAGE BLOCK (no registers used in %d instrs) ===", count);
        return false;
    }

    /* === PHASE 2: PROLOGUE === */
    NLOG2("PHASE 2: PROLOGUE all_used=0x%08X", all_used);

    /* v127: Save code start address for cache flush later */
    u32* code_start = recMem;

    LW(MIPSREG_K0, PERM_REG_1, off(psxM));
    emit_load_gprs(all_used);

    /* === PHASE 3: BODY === */
    NLOG2("PHASE 3: BODY (%d instructions)", count);

    pc = start_pc;
    u32 end_labels[NATIVE_MAX_BLOCK];
    int num_end_labels = 0;
    bool block_sets_pc = false;  /* v124: track if any instruction sets PC */

    for (int i = 0; i < count; ) {  /* v123: no i++ here, manual control */
        u32 opcode = *(u32*)PSXM(pc);
        u32 op = OP_CODE(opcode);

        NLOG3("BODY[%d] PC=0x%08X OP=0x%08X op=%02X", i, pc, opcode, op);

        /* v126: Branch/Jump handling with proper delay slot semantics
         * CRITICAL: Condition/target must be captured BEFORE delay slot!
         */
        if (is_branch(opcode) || is_jump(opcode)) {
            /* Get delay slot opcode */
            u32 ds_opcode = *(u32*)PSXM(pc + 4);
            NLOG3("  -> DELAY SLOT at PC=0x%08X OP=0x%08X", pc + 4, ds_opcode);

            /* PART 1: Save condition/target BEFORE delay slot */
            bool is_jr_jalr = (op == 0x00) && (OP_FUNCT(opcode) == 0x08 || OP_FUNCT(opcode) == 0x09);

            if (is_branch(opcode)) {
                /* v126: Branches - save condition result BEFORE delay slot */
                NLOG3("  -> v126: BRANCH - saving condition BEFORE delay slot");
                emit_branch_save_condition(opcode, pc);
            }
            else if (is_jr_jalr) {
                /* v125: JR/JALR - save jump target BEFORE delay slot */
                NLOG3("  -> v125: JR/JALR - saving target BEFORE delay slot");
                emit_jump_reg_save_target(opcode, pc);
            }

            /* PART 2: Execute delay slot instruction */
            emit_single_instruction(ds_opcode, all_used);

            /* PART 3: Use saved condition/target AFTER delay slot */
            if (is_branch(opcode)) {
                NLOG3("  -> BRANCH using saved condition");
                emit_branch_use_saved(opcode, pc, &end_labels[num_end_labels++]);
                block_sets_pc = true;
            } else {
                NLOG3("  -> JUMP");
                if (op == 0x02 || op == 0x03) {
                    /* J/JAL - target is immediate, not affected by delay slot */
                    emit_jump(opcode, pc);
                } else {
                    /* JR/JALR - target was already saved, now store to PC */
                    emit_jump_reg_store_pc();
                }
                block_sets_pc = true;
            }

            /* Skip both branch and delay slot */
            i += 2;
            pc += 8;
            continue;
        }

        /* Non-branch/jump instructions */
        if (op == 0x10) {
            NLOG3("  -> COP0");
            emit_cop0(opcode);
        }
        else if (op == 0x12) {
            NLOG3("  -> COP2");
            emit_cop2_trap(opcode, all_used);
        }
        else if (op == 0x32) {
            NLOG3("  -> LWC2");
            emit_lwc2(opcode, all_used);
        }
        else if (op == 0x3A) {
            NLOG3("  -> SWC2");
            emit_swc2(opcode, all_used);
        }
        else if (is_load(opcode)) {
            NLOG3("  -> LOAD");
            emit_load(opcode);
        }
        else if (is_store(opcode)) {
            NLOG3("  -> STORE");
            emit_store(opcode);
        }
        else {
            /* ALU - remap and emit */
            if (uses_remapped_regs(opcode)) {
                NLOG3("  -> ALU REMAP");
                if (op == 0x00) {
                    write32(remap_r_type(opcode));
                } else {
                    write32(remap_i_type(opcode));
                }
                native_instr_alu_remapped++;
            } else {
                NLOG3("  -> ALU COPY");
                write32(opcode);
                native_instr_alu_copy++;
            }
        }

        i++;
        pc += 4;
    }

    /* === PHASE 4: EPILOGUE === */
    NLOG2("PHASE 4: EPILOGUE num_end_labels=%d block_sets_pc=%d", num_end_labels, block_sets_pc);

    /* Patch end labels */
    for (int i = 0; i < num_end_labels; i++) {
        NLOG3("  patching end_label[%d]=0x%08X", i, end_labels[i]);
        fixup_branch((u32*)end_labels[i]);
    }

    emit_save_gprs(write_mask);

    /* v139: Set $v1 = cycle count for recFunc dispatcher
     *
     * BUG IN v137-v138: We stored cycles to g_native_cycles global, but recFunc
     * also adds $v1 to cycle. For native blocks, $v1 contains whatever PSX code
     * left there (garbage like 0x80200000), corrupting psxRegs.cycle!
     *
     * FIX: Set host $v1 = count AFTER saving PSX registers.
     * emit_save_gprs() already saved PSX $v1 to memory, so we can safely
     * overwrite host $v1 with the cycle count.
     *
     * recFunc will then correctly add $v1 to psxRegs.cycle.
     */
    /* v237 FIX: Must use ADJUST_CLOCK like dynarec does! */
    u32 adjusted_cycles_old = ADJUST_CLOCK(count);
    NLOG2("  v139: setting $v1=%d for recFunc (adjusted)", adjusted_cycles_old);

    /* Emit: ORI $v1, $zero, adjusted_cycles (or LI for larger values) */
    if (adjusted_cycles_old <= 0xFFFF) {
        ORI(MIPSREG_V1, 0, adjusted_cycles_old);
    } else {
        LI32(MIPSREG_V1, adjusted_cycles_old);
    }

    /* v124: CRITICAL FIX - if block doesn't end with branch/jump, set PC explicitly!
     * 'pc' now contains start_pc + count*4 (address after last instruction)
     * This is the fall-through PC.
     */
    if (!block_sets_pc) {
        NLOG2("  v124: setting fallthrough PC=0x%08X", pc);
        LI32(MIPSREG_K1, pc);
        SW(MIPSREG_K1, PERM_REG_1, off(pc));
    }

    /* Return PC in $v0 and jump back to dispatcher */
    LW(MIPSREG_V0, PERM_REG_1, off(pc));
    NLOG2("  returning to dispatcher with PC from psxRegs");
    JR(MIPSREG_RA);   /* v122: CRITICAL FIX - return to dispatcher! */
    NOP();            /* delay slot */

    native_blocks_compiled++;
    native_stats.blocks_native++;
    native_used++;  /* v153: track for stats */

    /* v127: CRITICAL - Flush caches after writing native code!
     * On MIPS, D-cache and I-cache are separate. After writing code
     * to memory (which goes to D-cache), we must:
     * 1. Write back D-cache to RAM
     * 2. Invalidate I-cache so CPU fetches new instructions
     * Without this, CPU executes garbage from stale I-cache!
     */
    u32 code_size = (u32)((recMem - code_start) * sizeof(u32));
    NLOG1("=== COMPILED OK (psx_instrs=%d, host_bytes=%d) ===", count, code_size);

#ifdef __mips__
    /* Use GCC builtin for cache coherency */
    __builtin___clear_cache((char*)code_start, (char*)recMem);
    /* v149: HEISENBUG FIX - add full memory barrier after cache flush
     * Without this, CPU may execute stale instructions from I-cache
     * before D-cache writeback completes. This was masked by logging
     * delays in v141 but caused crashes in v148 without logs.
     */
    __sync_synchronize();
    NLOG2("  cache flush: %p - %p (%d bytes)", (void*)code_start, (void*)recMem, code_size);
#endif

    return true;
#endif  /* ===== END DISABLED FOR v159 BISECTION TEST ===== */
}

/* Statistics getters */
extern "C" u32 native_get_blocks_total(void) { return native_blocks_total; }
extern "C" u32 native_get_blocks_compiled(void) { return native_blocks_compiled; }
extern "C" u32 native_get_blocks_skipped_conflict(void) { return native_blocks_skipped_k0k1; }
extern "C" u32 native_get_blocks_skipped_cop(void) { return 0; }
extern "C" u32 native_get_instr_native(void) { return native_instr_alu_copy + native_instr_alu_remapped; }
extern "C" u32 native_get_instr_loadstore(void) { return native_instr_loadstore; }
extern "C" u32 native_get_instr_remapped(void) { return native_instr_alu_remapped; }
extern "C" u32 native_get_instr_branch(void) { return native_instr_branch; }
extern "C" u32 native_get_instr_jump(void) { return native_instr_jump; }

#endif /* REC_NATIVE_V127_H */
