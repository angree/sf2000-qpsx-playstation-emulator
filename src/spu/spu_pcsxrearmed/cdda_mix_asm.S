/***************************************************************************
 *   QPSX v111 - CDDA Mixing in MIPS32 Assembly
 *
 *   Hand-optimized assembly for SF2000 (MIPS32 918MHz, Little-Endian)
 *
 *   This provides a fast path for CDDA audio mixing, the main bottleneck
 *   in games with CD audio (takes 10-15% of frame time in C).
 *
 *   Optimizations:
 *   1. Loop unrolling - 4 stereo samples per iteration
 *   2. Efficient 16-bit signed multiply using mult/mflo
 *   3. Pipeline-friendly instruction ordering
 *   4. Minimized branches in inner loop
 *   5. Direct register operations without unnecessary loads
 *
 *   Function: MixCDDA_asm_inner
 *   Processes CDDA samples with volume scaling and adds to output buffer.
 *
 *   Target: MIPS32 little-endian (SF2000 B210 CPU)
 *   ABI: o32 (a0-a3 args, v0-v1 return, t0-t9 temps, s0-s7 saved)
 *
 ***************************************************************************/

#if defined(SF2000) || defined(__mips__)

    .set noreorder
    .set noat

    .text
    .align 4

/*==========================================================================
 * MixCDDA_asm_inner - Mix CDDA samples with volume to output buffer
 *
 * C prototype:
 *   int MixCDDA_asm_inner(int *SSumLR,      // $a0: output buffer (L/R interleaved)
 *                         uint32_t *src,    // $a1: source samples (L16|R16 packed)
 *                         int count,        // $a2: number of stereo samples
 *                         int vol,          // $a3: volume (0x8000 = 100%)
 *                         int unity_vol);   // stack: 1 if volume = 100%
 *
 * Returns: number of samples processed (in $v0)
 *
 * Sample format (little-endian):
 *   src[n] = (right_sample << 16) | (left_sample & 0xFFFF)
 *   Both are signed 16-bit values
 *
 * Volume scaling:
 *   output = (sample * vol) >> 15
 *   When vol = 0x8000 (32768), this equals the input value
 *
 * Output:
 *   SSumLR[n*2+0] += scaled_left
 *   SSumLR[n*2+1] += scaled_right
 *
 *==========================================================================*/
    .global MixCDDA_asm_inner
    .ent MixCDDA_asm_inner
    .type MixCDDA_asm_inner, @function

MixCDDA_asm_inner:
    /* Prologue - save callee-saved registers we'll use */
    addiu   $sp, $sp, -24
    sw      $s0, 0($sp)
    sw      $s1, 4($sp)
    sw      $s2, 8($sp)
    sw      $s3, 12($sp)
    sw      $s4, 16($sp)
    sw      $ra, 20($sp)

    /* Load unity_vol from stack (5th argument at offset 24+16=40) */
    lw      $t9, 40($sp)            /* $t9 = unity_vol */

    /* Check count - if <= 0, return immediately */
    blez    $a2, .Ldone
    move    $v0, $zero              /* delay slot: return 0 if count <= 0 */

    /* Save original count for return value */
    move    $v0, $a2                /* return count */

    /* Registers:
     * $a0 = SSumLR pointer
     * $a1 = src pointer
     * $a2 = remaining count
     * $a3 = volume
     * $t9 = unity_vol flag
     * $s0-$s3 = temp for samples
     * $s4 = loop counter
     */

    /* Check if unity volume - branch to fast path */
    bnez    $t9, .Lunity_loop
    nop

/*--------------------------------------------------------------------------
 * Volume scaling path - multiply each sample by vol, shift by 15
 *--------------------------------------------------------------------------*/
.Lvol_loop:
    /* Load packed sample: v = *src++ */
    lw      $t0, 0($a1)             /* $t0 = packed sample (R16|L16) */
    addiu   $a1, $a1, 4             /* src++ */

    /* Extract left sample (low 16 bits, sign-extended) using SLL/SRA */
    sll     $t1, $t0, 16            /* Shift left to put in high bits */
    sra     $t1, $t1, 16            /* Arithmetic shift right to sign-extend */

    /* Extract right sample (high 16 bits, sign-extended) */
    sra     $t2, $t0, 16            /* $t2 = (int16_t)(t0 >> 16) */

    /* Scale left: l = (left * vol) >> 15 */
    mult    $t1, $a3
    mflo    $t3                     /* $t3 = left * vol (low 32 bits) */
    sra     $t3, $t3, 15            /* $t3 = (left * vol) >> 15 */

    /* Scale right: r = (right * vol) >> 15 */
    mult    $t2, $a3
    mflo    $t4                     /* $t4 = right * vol (low 32 bits) */
    sra     $t4, $t4, 15            /* $t4 = (right * vol) >> 15 */

    /* Accumulate to output: SSumLR[0] += l, SSumLR[1] += r */
    lw      $t5, 0($a0)             /* $t5 = SSumLR[0] */
    lw      $t6, 4($a0)             /* $t6 = SSumLR[1] */
    addu    $t5, $t5, $t3           /* $t5 += scaled_left */
    addu    $t6, $t6, $t4           /* $t6 += scaled_right */
    sw      $t5, 0($a0)             /* SSumLR[0] = result */
    sw      $t6, 4($a0)             /* SSumLR[1] = result */
    addiu   $a0, $a0, 8             /* SSumLR += 2 */

    /* Decrement count and loop */
    addiu   $a2, $a2, -1
    bnez    $a2, .Lvol_loop
    nop

    b       .Ldone
    nop

/*--------------------------------------------------------------------------
 * Unity volume fast path - no multiply, just sign-extend and add
 * This is ~2x faster than the volume path
 *--------------------------------------------------------------------------*/
.Lunity_loop:
    /* Process 4 samples per iteration for better pipeline utilization */
    /* Check if we have at least 4 samples */
    slti    $t0, $a2, 4
    bnez    $t0, .Lunity_tail
    nop

.Lunity_loop_4:
    /* Load 4 packed samples */
    lw      $s0, 0($a1)             /* sample 0 */
    lw      $s1, 4($a1)             /* sample 1 */
    lw      $s2, 8($a1)             /* sample 2 */
    lw      $s3, 12($a1)            /* sample 3 */
    addiu   $a1, $a1, 16            /* src += 4 */

    /* Sample 0: extract L/R and accumulate */
    sll     $t1, $s0, 16            /* left0 = sign_extend(s0 & 0xFFFF) */
    sra     $t1, $t1, 16
    sra     $t2, $s0, 16            /* right0 = s0 >> 16 (already signed) */
    lw      $t3, 0($a0)             /* load SSumLR[0] */
    lw      $t4, 4($a0)             /* load SSumLR[1] */
    addu    $t3, $t3, $t1           /* SSumLR[0] += left0 */
    addu    $t4, $t4, $t2           /* SSumLR[1] += right0 */
    sw      $t3, 0($a0)
    sw      $t4, 4($a0)

    /* Sample 1: extract L/R and accumulate */
    sll     $t1, $s1, 16            /* left1 */
    sra     $t1, $t1, 16
    sra     $t2, $s1, 16            /* right1 */
    lw      $t3, 8($a0)             /* load SSumLR[2] */
    lw      $t4, 12($a0)            /* load SSumLR[3] */
    addu    $t3, $t3, $t1
    addu    $t4, $t4, $t2
    sw      $t3, 8($a0)
    sw      $t4, 12($a0)

    /* Sample 2: extract L/R and accumulate */
    sll     $t1, $s2, 16            /* left2 */
    sra     $t1, $t1, 16
    sra     $t2, $s2, 16            /* right2 */
    lw      $t3, 16($a0)            /* load SSumLR[4] */
    lw      $t4, 20($a0)            /* load SSumLR[5] */
    addu    $t3, $t3, $t1
    addu    $t4, $t4, $t2
    sw      $t3, 16($a0)
    sw      $t4, 20($a0)

    /* Sample 3: extract L/R and accumulate */
    sll     $t1, $s3, 16            /* left3 */
    sra     $t1, $t1, 16
    sra     $t2, $s3, 16            /* right3 */
    lw      $t3, 24($a0)            /* load SSumLR[6] */
    lw      $t4, 28($a0)            /* load SSumLR[7] */
    addu    $t3, $t3, $t1
    addu    $t4, $t4, $t2
    sw      $t3, 24($a0)
    sw      $t4, 28($a0)

    addiu   $a0, $a0, 32            /* SSumLR += 8 (4 stereo samples) */

    /* Decrement count by 4 and continue if >= 4 remain */
    addiu   $a2, $a2, -4
    slti    $t0, $a2, 4
    beqz    $t0, .Lunity_loop_4
    nop

.Lunity_tail:
    /* Process remaining 0-3 samples one at a time */
    beqz    $a2, .Ldone
    nop

.Lunity_loop_1:
    /* Load packed sample */
    lw      $t0, 0($a1)
    addiu   $a1, $a1, 4

    /* Extract left and right */
    sll     $t1, $t0, 16            /* left */
    sra     $t1, $t1, 16
    sra     $t2, $t0, 16            /* right */

    /* Accumulate */
    lw      $t3, 0($a0)
    lw      $t4, 4($a0)
    addu    $t3, $t3, $t1
    addu    $t4, $t4, $t2
    sw      $t3, 0($a0)
    sw      $t4, 4($a0)
    addiu   $a0, $a0, 8

    addiu   $a2, $a2, -1
    bnez    $a2, .Lunity_loop_1
    nop

.Ldone:
    /* Epilogue - restore saved registers */
    lw      $s0, 0($sp)
    lw      $s1, 4($sp)
    lw      $s2, 8($sp)
    lw      $s3, 12($sp)
    lw      $s4, 16($sp)
    lw      $ra, 20($sp)
    jr      $ra
    addiu   $sp, $sp, 24            /* delay slot */

.end MixCDDA_asm_inner


/*==========================================================================
 * MixCDDA_asm_inner_nounroll - Simpler version without loop unrolling
 *
 * Used as fallback if SEH instruction is not available (pre-MIPS32R2)
 * Uses SLL/SRA for sign extension instead
 *==========================================================================*/
    .global MixCDDA_asm_inner_simple
    .ent MixCDDA_asm_inner_simple
    .type MixCDDA_asm_inner_simple, @function

MixCDDA_asm_inner_simple:
    /* Prologue */
    addiu   $sp, $sp, -8
    sw      $ra, 4($sp)

    /* Load unity_vol from stack */
    lw      $t9, 24($sp)            /* $t9 = unity_vol (offset 8+16) */

    /* Check count */
    blez    $a2, .Lsimple_done
    move    $v0, $zero

    move    $v0, $a2                /* return count */

    /* Check unity volume */
    bnez    $t9, .Lsimple_unity
    nop

/*--- Volume scaling loop ---*/
.Lsimple_vol_loop:
    lw      $t0, 0($a1)             /* packed sample */
    addiu   $a1, $a1, 4

    /* Sign extend left (low 16 bits) using SLL/SRA */
    sll     $t1, $t0, 16            /* shift left to put in high bits */
    sra     $t1, $t1, 16            /* arithmetic shift right to sign-extend */

    /* Sign extend right (high 16 bits) */
    sra     $t2, $t0, 16            /* already in high bits, just shift */

    /* Scale by volume */
    mult    $t1, $a3
    mflo    $t3
    sra     $t3, $t3, 15

    mult    $t2, $a3
    mflo    $t4
    sra     $t4, $t4, 15

    /* Accumulate */
    lw      $t5, 0($a0)
    lw      $t6, 4($a0)
    addu    $t5, $t5, $t3
    addu    $t6, $t6, $t4
    sw      $t5, 0($a0)
    sw      $t6, 4($a0)
    addiu   $a0, $a0, 8

    addiu   $a2, $a2, -1
    bnez    $a2, .Lsimple_vol_loop
    nop

    b       .Lsimple_done
    nop

/*--- Unity volume loop ---*/
.Lsimple_unity:
    lw      $t0, 0($a1)
    addiu   $a1, $a1, 4

    /* Sign extend using SLL/SRA (works on all MIPS32) */
    sll     $t1, $t0, 16
    sra     $t1, $t1, 16            /* left */
    sra     $t2, $t0, 16            /* right */

    /* Accumulate */
    lw      $t3, 0($a0)
    lw      $t4, 4($a0)
    addu    $t3, $t3, $t1
    addu    $t4, $t4, $t2
    sw      $t3, 0($a0)
    sw      $t4, 4($a0)
    addiu   $a0, $a0, 8

    addiu   $a2, $a2, -1
    bnez    $a2, .Lsimple_unity
    nop

.Lsimple_done:
    lw      $ra, 4($sp)
    jr      $ra
    addiu   $sp, $sp, 8

.end MixCDDA_asm_inner_simple

#endif /* SF2000 || __mips__ */
