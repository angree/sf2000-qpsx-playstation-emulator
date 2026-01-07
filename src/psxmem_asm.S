/***************************************************************************
 *   QPSX v101 - MIPS32 Assembly Memory Functions
 *
 *   Hand-optimized assembly for SF2000 (MIPS32 918MHz, Little-Endian)
 *
 *   v101: FIXED - restores code invalidation logic from v099
 *         The v100 version that removed recClear caused vector corruption!
 *
 *   Functions:
 *     - psxMemWrite32_asm(u32 mem, u32 value)
 *     - psxMemWrite16_asm(u32 mem, u16 value)
 *     - psxMemWrite8_asm(u32 mem, u8 value)
 *     - psxMemRead32_asm(u32 mem) -> u32
 *     - psxMemRead16_asm(u32 mem) -> u16
 *     - psxMemRead8_asm(u32 mem) -> u8
 *
 *   External symbols needed:
 *     - psxMemWLUT: Write LUT table (64K entries, each 4 bytes)
 *     - psxMemRLUT: Read LUT table
 *     - psxH: Hardware registers base
 *     - psxHwWrite32/16/8: HW write functions (extern "C" in psxhw.h)
 *     - psxHwRead32/16/8: HW read functions (extern "C" in psxhw.h)
 *     - g_opt_skip_code_inv: Skip code invalidation flag
 *     - recClear: Code invalidation function (extern "C" in recompiler.cpp)
 *
 ***************************************************************************/

#if defined(SF2000) || defined(__mips__)

    .set noreorder
    .set noat

    .text
    .align 4

/*--------------------------------------------------------------------------
 * External symbols
 *--------------------------------------------------------------------------*/
    .extern psxMemWLUT
    .extern psxMemRLUT
    .extern psxH
    .extern psxHwWrite32
    .extern psxHwWrite16
    .extern psxHwWrite8
    .extern psxHwRead32
    .extern psxHwRead16
    .extern psxHwRead8
    .extern g_opt_skip_code_inv
    .extern recClear

/*==========================================================================
 * psxMemWrite32_asm - Write 32-bit value to PSX memory
 *
 * Arguments:
 *   $a0 = mem (PSX address)
 *   $a1 = value (32-bit value to write)
 *==========================================================================*/
    .global psxMemWrite32_asm
    .ent psxMemWrite32_asm
    .type psxMemWrite32_asm, @function

psxMemWrite32_asm:
    /* Extract high 16 bits: t = mem >> 16 */
    srl     $t0, $a0, 16            /* $t0 = t = mem >> 16 */

    /* Check for HW region: 0x1f80, 0x9f80, 0xbf80 */
    andi    $t1, $t0, 0x1F80        /* $t1 = t & 0x1F80 */
    li      $t2, 0x1F80
    beq     $t1, $t2, .Lhw_write32  /* if HW region, branch */
    andi    $t3, $a0, 0xFFFF        /* $t3 = m = mem & 0xFFFF (delay slot) */

    /*----------------------------------------------------------------------
     * RAM Fast Path (most common case)
     * Load LUT pointer: p = psxMemWLUT[t]
     *----------------------------------------------------------------------*/
    la      $t4, psxMemWLUT         /* $t4 = &psxMemWLUT */
    sll     $t1, $t0, 2             /* $t1 = t * 4 (pointer size) */
    addu    $t4, $t4, $t1           /* $t4 = &psxMemWLUT[t] */
    lw      $t4, 0($t4)             /* $t4 = p = psxMemWLUT[t] */

    beqz    $t4, .Lnull_ptr32       /* if p == NULL, handle error */
    nop

    /* Direct RAM store - THE HOT PATH */
    addu    $t5, $t4, $t3           /* $t5 = p + m */
    sw      $a1, 0($t5)             /* *(u32*)(p + m) = value */

    /*----------------------------------------------------------------------
     * v101: Code invalidation (if not skipped)
     * Check g_opt_skip_code_inv flag - RESTORED from v099!
     *----------------------------------------------------------------------*/
    la      $t0, g_opt_skip_code_inv
    lw      $t0, 0($t0)             /* $t0 = g_opt_skip_code_inv */
    bnez    $t0, .Ldone32           /* if skip enabled, return */
    nop

    /* Call recClear(mem, 1) for code invalidation */
    addiu   $sp, $sp, -8
    sw      $ra, 4($sp)

    /* recClear(u32 addr, u32 size) - $a0 already has mem */
    li      $a1, 1                  /* size = 1 */
    la      $t0, recClear
    jalr    $t0
    nop

    lw      $ra, 4($sp)
    addiu   $sp, $sp, 8

.Ldone32:
    jr      $ra
    nop

    /*----------------------------------------------------------------------
     * HW Region handler
     *----------------------------------------------------------------------*/
.Lhw_write32:
    /* Check if scratchpad: m < 0x400 */
    sltiu   $t1, $t3, 0x400
    beqz    $t1, .Lhw_io_write32
    nop

    /* Scratchpad write: psxH[m] = value */
    la      $t4, psxH
    lw      $t4, 0($t4)             /* $t4 = psxH base */
    addu    $t4, $t4, $t3           /* $t4 = psxH + m */
    sw      $a1, 0($t4)             /* store to scratchpad */
    jr      $ra
    nop

.Lhw_io_write32:
    /* Hardware I/O: call psxHwWrite32(mem, value) */
    la      $t0, psxHwWrite32
    jr      $t0                     /* tail call optimization */
    nop

.Lnull_ptr32:
    /* NULL pointer - check for cache control port 0xfffe0130 */
    li      $t1, 0xfffe0130
    bne     $a0, $t1, .Ldone32
    nop

    /* Cache control port - just return for now */
    jr      $ra
    nop

    .end psxMemWrite32_asm
    .size psxMemWrite32_asm, .-psxMemWrite32_asm


/*==========================================================================
 * psxMemWrite16_asm - Write 16-bit value to PSX memory
 *==========================================================================*/
    .global psxMemWrite16_asm
    .ent psxMemWrite16_asm
    .type psxMemWrite16_asm, @function

psxMemWrite16_asm:
    srl     $t0, $a0, 16            /* $t0 = t = mem >> 16 */

    /* Check for HW region */
    andi    $t1, $t0, 0x1F80
    li      $t2, 0x1F80
    beq     $t1, $t2, .Lhw_write16
    andi    $t3, $a0, 0xFFFF        /* $t3 = m (delay slot) */

    /* RAM path */
    la      $t4, psxMemWLUT
    sll     $t1, $t0, 2
    addu    $t4, $t4, $t1
    lw      $t4, 0($t4)             /* $t4 = p = psxMemWLUT[t] */

    beqz    $t4, .Ldone16
    nop

    addu    $t5, $t4, $t3           /* $t5 = p + m */
    sh      $a1, 0($t5)             /* *(u16*)(p + m) = value */

    /* v101: Code invalidation check - RESTORED */
    la      $t0, g_opt_skip_code_inv
    lw      $t0, 0($t0)
    bnez    $t0, .Ldone16
    nop

    /* Align address for Clear: mem & ~3 */
    addiu   $sp, $sp, -8
    sw      $ra, 4($sp)

    li      $t1, -4                 /* 0xFFFFFFFC */
    and     $a0, $a0, $t1           /* $a0 = mem & ~3 */
    li      $a1, 1
    la      $t0, recClear
    jalr    $t0
    nop

    lw      $ra, 4($sp)
    addiu   $sp, $sp, 8

.Ldone16:
    jr      $ra
    nop

.Lhw_write16:
    sltiu   $t1, $t3, 0x400
    beqz    $t1, .Lhw_io_write16
    nop

    /* Scratchpad */
    la      $t4, psxH
    lw      $t4, 0($t4)
    addu    $t4, $t4, $t3
    sh      $a1, 0($t4)
    jr      $ra
    nop

.Lhw_io_write16:
    la      $t0, psxHwWrite16
    jr      $t0
    nop

    .end psxMemWrite16_asm
    .size psxMemWrite16_asm, .-psxMemWrite16_asm


/*==========================================================================
 * psxMemWrite8_asm - Write 8-bit value to PSX memory
 *==========================================================================*/
    .global psxMemWrite8_asm
    .ent psxMemWrite8_asm
    .type psxMemWrite8_asm, @function

psxMemWrite8_asm:
    srl     $t0, $a0, 16            /* $t0 = t = mem >> 16 */

    /* Check for HW region */
    andi    $t1, $t0, 0x1F80
    li      $t2, 0x1F80
    beq     $t1, $t2, .Lhw_write8
    andi    $t3, $a0, 0xFFFF        /* $t3 = m (delay slot) */

    /* RAM path */
    la      $t4, psxMemWLUT
    sll     $t1, $t0, 2
    addu    $t4, $t4, $t1
    lw      $t4, 0($t4)             /* $t4 = p = psxMemWLUT[t] */

    beqz    $t4, .Ldone8
    nop

    addu    $t5, $t4, $t3           /* $t5 = p + m */
    sb      $a1, 0($t5)             /* *(u8*)(p + m) = value */

    /* v101: Code invalidation check - RESTORED */
    la      $t0, g_opt_skip_code_inv
    lw      $t0, 0($t0)
    bnez    $t0, .Ldone8
    nop

    /* Align address for Clear: mem & ~3 */
    addiu   $sp, $sp, -8
    sw      $ra, 4($sp)

    li      $t1, -4
    and     $a0, $a0, $t1
    li      $a1, 1
    la      $t0, recClear
    jalr    $t0
    nop

    lw      $ra, 4($sp)
    addiu   $sp, $sp, 8

.Ldone8:
    jr      $ra
    nop

.Lhw_write8:
    sltiu   $t1, $t3, 0x400
    beqz    $t1, .Lhw_io_write8
    nop

    /* Scratchpad */
    la      $t4, psxH
    lw      $t4, 0($t4)
    addu    $t4, $t4, $t3
    sb      $a1, 0($t4)
    jr      $ra
    nop

.Lhw_io_write8:
    la      $t0, psxHwWrite8
    jr      $t0
    nop

    .end psxMemWrite8_asm
    .size psxMemWrite8_asm, .-psxMemWrite8_asm


/*==========================================================================
 * psxMemRead32_asm - Read 32-bit value from PSX memory
 *==========================================================================*/
    .global psxMemRead32_asm
    .ent psxMemRead32_asm
    .type psxMemRead32_asm, @function

psxMemRead32_asm:
    srl     $t0, $a0, 16            /* $t0 = t = mem >> 16 */

    /* Check for HW region */
    andi    $t1, $t0, 0x1F80
    li      $t2, 0x1F80
    beq     $t1, $t2, .Lhw_read32
    andi    $t3, $a0, 0xFFFF        /* $t3 = m (delay slot) */

    /* RAM/ROM path - use Read LUT */
    la      $t4, psxMemRLUT
    sll     $t1, $t0, 2
    addu    $t4, $t4, $t1
    lw      $t4, 0($t4)             /* $t4 = p = psxMemRLUT[t] */

    beqz    $t4, .Lread_null32
    nop

    /* Direct memory read - THE HOT PATH */
    addu    $t5, $t4, $t3           /* $t5 = p + m */
    lw      $v0, 0($t5)             /* $v0 = *(u32*)(p + m) */
    jr      $ra
    nop

.Lhw_read32:
    /* Check if scratchpad: m < 0x400 */
    sltiu   $t1, $t3, 0x400
    beqz    $t1, .Lhw_io_read32
    nop

    /* Scratchpad read */
    la      $t4, psxH
    lw      $t4, 0($t4)
    addu    $t4, $t4, $t3
    lw      $v0, 0($t4)
    jr      $ra
    nop

.Lhw_io_read32:
    la      $t0, psxHwRead32
    jr      $t0                     /* tail call */
    nop

.Lread_null32:
    li      $v0, 0
    jr      $ra
    nop

    .end psxMemRead32_asm
    .size psxMemRead32_asm, .-psxMemRead32_asm


/*==========================================================================
 * psxMemRead16_asm - Read 16-bit value from PSX memory
 *==========================================================================*/
    .global psxMemRead16_asm
    .ent psxMemRead16_asm
    .type psxMemRead16_asm, @function

psxMemRead16_asm:
    srl     $t0, $a0, 16
    andi    $t1, $t0, 0x1F80
    li      $t2, 0x1F80
    beq     $t1, $t2, .Lhw_read16
    andi    $t3, $a0, 0xFFFF

    la      $t4, psxMemRLUT
    sll     $t1, $t0, 2
    addu    $t4, $t4, $t1
    lw      $t4, 0($t4)

    beqz    $t4, .Lread_null16
    nop

    addu    $t5, $t4, $t3
    lhu     $v0, 0($t5)
    jr      $ra
    nop

.Lhw_read16:
    sltiu   $t1, $t3, 0x400
    beqz    $t1, .Lhw_io_read16
    nop

    la      $t4, psxH
    lw      $t4, 0($t4)
    addu    $t4, $t4, $t3
    lhu     $v0, 0($t4)
    jr      $ra
    nop

.Lhw_io_read16:
    la      $t0, psxHwRead16
    jr      $t0
    nop

.Lread_null16:
    li      $v0, 0
    jr      $ra
    nop

    .end psxMemRead16_asm
    .size psxMemRead16_asm, .-psxMemRead16_asm


/*==========================================================================
 * psxMemRead8_asm - Read 8-bit value from PSX memory
 *==========================================================================*/
    .global psxMemRead8_asm
    .ent psxMemRead8_asm
    .type psxMemRead8_asm, @function

psxMemRead8_asm:
    srl     $t0, $a0, 16
    andi    $t1, $t0, 0x1F80
    li      $t2, 0x1F80
    beq     $t1, $t2, .Lhw_read8
    andi    $t3, $a0, 0xFFFF

    la      $t4, psxMemRLUT
    sll     $t1, $t0, 2
    addu    $t4, $t4, $t1
    lw      $t4, 0($t4)

    beqz    $t4, .Lread_null8
    nop

    addu    $t5, $t4, $t3
    lbu     $v0, 0($t5)
    jr      $ra
    nop

.Lhw_read8:
    sltiu   $t1, $t3, 0x400
    beqz    $t1, .Lhw_io_read8
    nop

    la      $t4, psxH
    lw      $t4, 0($t4)
    addu    $t4, $t4, $t3
    lbu     $v0, 0($t4)
    jr      $ra
    nop

.Lhw_io_read8:
    la      $t0, psxHwRead8
    jr      $t0
    nop

.Lread_null8:
    li      $v0, 0
    jr      $ra
    nop

    .end psxMemRead8_asm
    .size psxMemRead8_asm, .-psxMemRead8_asm


#endif /* SF2000 || __mips__ */
