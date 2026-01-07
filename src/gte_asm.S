/***************************************************************************
 *   QPSX v095 - GTE Functions in MIPS32 Assembly
 *
 *   Hand-written assembly for performance-critical GTE operations.
 *   These are optional replacements for the C++ versions.
 *
 *   Target: MIPS32 little-endian (SF2000 B210 CPU)
 *   ABI: o32 (a0-a3 args, v0-v1 return, t0-t9 temps, s0-s7 saved)
 *
 *   Offsets in psxRegs structure:
 *     GPR:  0     (34 regs * 4 = 136 bytes)
 *     CP0:  136   (32 regs * 4 = 128 bytes)
 *     CP2D: 264   (32 regs * 4 = 128 bytes) - GTE Data registers
 *     CP2C: 392   (32 regs * 4 = 128 bytes) - GTE Control registers
 *
 ***************************************************************************/

.set noreorder
.set noat

/* psxRegs offsets */
#define OFF_CP2D        264
#define OFF_CP2C        392

/* CP2D register offsets (from psxRegs base) */
#define OFF_SXY0        (OFF_CP2D + 12*4)       /* 312: SX0=lo, SY0=hi */
#define OFF_SXY1        (OFF_CP2D + 13*4)       /* 316: SX1=lo, SY1=hi */
#define OFF_SXY2        (OFF_CP2D + 14*4)       /* 320: SX2=lo, SY2=hi */
#define OFF_MAC0        (OFF_CP2D + 24*4)       /* 360 */

/* CP2C register offsets */
#define OFF_FLAG        (OFF_CP2C + 31*4)       /* 516 */

.text

/***************************************************************************
 * ASM Block 1: NCLIP (Normal Clipping)
 *
 * Computes 2D cross product for backface culling:
 *   MAC0 = SX0*(SY1-SY2) + SX1*(SY2-SY0) + SX2*(SY0-SY1)
 *
 * This is a simplified version WITHOUT overflow flag checking.
 * Use with g_opt_gte_noflags=1 for best results.
 *
 * Input: psxRegs.CP2D[12-14] contain SXY0, SXY1, SXY2
 * Output: psxRegs.CP2D[24] = MAC0
 *         psxRegs.CP2C[31] = FLAG (cleared to 0)
 ***************************************************************************/
.globl gte_NCLIP_asm
.ent gte_NCLIP_asm
gte_NCLIP_asm:
    /* Get psxRegs base address into $t0 */
    la      $t0, psxRegs

    /* Clear FLAG register (no overflow checking in ASM version) */
    sw      $zero, OFF_FLAG($t0)

    /* Load SY values (sign-extended from high halfwords) */
    lh      $t1, OFF_SXY0+2($t0)        /* SY0 */
    lh      $t2, OFF_SXY1+2($t0)        /* SY1 */
    lh      $t3, OFF_SXY2+2($t0)        /* SY2 */

    /* Load SX values (sign-extended from low halfwords) */
    lh      $t4, OFF_SXY0($t0)          /* SX0 */
    lh      $t5, OFF_SXY1($t0)          /* SX1 */
    lh      $t6, OFF_SXY2($t0)          /* SX2 */

    /* Compute differences */
    subu    $t7, $t2, $t3               /* diff0 = SY1 - SY2 */
    subu    $t8, $t3, $t1               /* diff1 = SY2 - SY0 */
    subu    $t9, $t1, $t2               /* diff2 = SY0 - SY1 */

    /* Multiply and accumulate: result = SX0*diff0 + SX1*diff1 + SX2*diff2 */
    /* Using 32-bit arithmetic (sufficient for typical screen coords) */

    mult    $t4, $t7                    /* SX0 * diff0 */
    mflo    $v0                         /* result = product0 */

    mult    $t5, $t8                    /* SX1 * diff1 */
    mflo    $v1                         /* temp = product1 */
    addu    $v0, $v0, $v1               /* result += product1 */

    mult    $t6, $t9                    /* SX2 * diff2 */
    mflo    $v1                         /* temp = product2 */
    addu    $v0, $v0, $v1               /* result += product2 */

    /* Store result to MAC0 */
    sw      $v0, OFF_MAC0($t0)

    jr      $ra
    nop                                 /* delay slot */

.end gte_NCLIP_asm


/***************************************************************************
 * ASM Block 2: AVSZ3 (Average of 3 Z values)
 *
 * Computes: MAC0 = ZSF3 * (SZ1 + SZ2 + SZ3)
 *           OTZ = MAC0 >> 12 (clamped to 0-65535)
 *
 * Used for ordering table depth sorting.
 ***************************************************************************/

/* Additional offsets for AVSZ3 */
#define OFF_SZ1         (OFF_CP2D + 17*4)       /* 332 */
#define OFF_SZ2         (OFF_CP2D + 18*4)       /* 336 */
#define OFF_SZ3         (OFF_CP2D + 19*4)       /* 340 */
#define OFF_OTZ         (OFF_CP2D + 7*4)        /* 292 */
#define OFF_ZSF3        (OFF_CP2C + 29*4)       /* 508 */

.globl gte_AVSZ3_asm
.ent gte_AVSZ3_asm
gte_AVSZ3_asm:
    /* Get psxRegs base address */
    la      $t0, psxRegs

    /* Clear FLAG */
    sw      $zero, OFF_FLAG($t0)

    /* Load SZ1, SZ2, SZ3 (unsigned 16-bit from low halfwords) */
    lhu     $t1, OFF_SZ1($t0)           /* SZ1 */
    lhu     $t2, OFF_SZ2($t0)           /* SZ2 */
    lhu     $t3, OFF_SZ3($t0)           /* SZ3 */

    /* Load ZSF3 (signed 16-bit) */
    lh      $t4, OFF_ZSF3($t0)          /* ZSF3 */

    /* Sum = SZ1 + SZ2 + SZ3 */
    addu    $t5, $t1, $t2
    addu    $t5, $t5, $t3

    /* MAC0 = ZSF3 * Sum */
    mult    $t4, $t5
    mflo    $v0

    /* Store MAC0 */
    sw      $v0, OFF_MAC0($t0)

    /* OTZ = MAC0 >> 12, clamped to 0-65535 */
    sra     $v1, $v0, 12                /* v1 = MAC0 >> 12 */

    /* Clamp to 0 */
    bgez    $v1, 1f
    move    $t1, $zero                  /* delay: t1 = 0 */
    move    $v1, $zero                  /* if negative, clamp to 0 */
1:
    /* Clamp to 65535 */
    li      $t1, 65535
    sltu    $t2, $t1, $v1               /* t2 = (65535 < v1) */
    beqz    $t2, 2f
    nop
    move    $v1, $t1                    /* if > 65535, clamp to 65535 */
2:
    /* Store OTZ (low halfword only) */
    sh      $v1, OFF_OTZ($t0)

    jr      $ra
    nop

.end gte_AVSZ3_asm


/* End of gte_asm.S */
