/***************************************************************************
 *   QPSX v091 - MIPS32 Assembly Optimizations for GPU UNAI
 *
 *   Copyright (C) 2024 QPSX Team
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   Target: MIPS32 Release 2 (SF2000 HCSEMI B210)
 *   Features used: MOVN/MOVZ (branchless), EXT/INS (bit fields)
 ***************************************************************************/

    .set    noreorder
    .set    mips32r2

    .text

/*=============================================================================
 * gpuLightingTXT_ASM - MIPS32 optimized texture lighting
 *=============================================================================
 * Applies 5-bit lighting to bgr555 texture color using branchless clamping.
 *
 * Input:  a0 = uSrc (u16, bgr555 format: -bbbbbgggggrrrrr)
 *         a1 = r5 (u8, red light 0-31, 15=neutral)
 *         a2 = g5 (u8, green light 0-31, 15=neutral)
 *         a3 = b5 (u8, blue light 0-31, 15=neutral)
 * Output: v0 = lit pixel (u16, bgr555)
 *
 * Formula: out = (component * light) >> 4, clamped to 31
 *=============================================================================*/
    .globl  gpuLightingTXT_ASM
    .ent    gpuLightingTXT_ASM
gpuLightingTXT_ASM:
    /* Fast path: neutral lighting (15,15,15) = no change */
    li      $t0, 15
    bne     $a1, $t0, .L_light_do
    nop
    bne     $a2, $t0, .L_light_do
    nop
    bne     $a3, $t0, .L_light_do
    move    $v0, $a0              /* delay slot: prepare return */
    jr      $ra
    nop

.L_light_do:
    /* Extract RGB components using EXT (MIPS32R2) */
    /* uSrc format: -bbbbbgggggrrrrr (bits 0-4=R, 5-9=G, 10-14=B) */
    ext     $t0, $a0, 0, 5        /* t0 = R (bits 0-4) */
    ext     $t1, $a0, 5, 5        /* t1 = G (bits 5-9) */
    ext     $t2, $a0, 10, 5       /* t2 = B (bits 10-14) */

    /* Multiply: component * light */
    mul     $t0, $t0, $a1         /* t0 = R * r5 */
    mul     $t1, $t1, $a2         /* t1 = G * g5 */
    mul     $t2, $t2, $a3         /* t2 = B * b5 */

    /* Shift right by 4 (divide by 16) */
    srl     $t0, $t0, 4
    srl     $t1, $t1, 4
    srl     $t2, $t2, 4

    /* Branchless clamp to 31 using MOVZ */
    /* if (val >= 32) val = 31 */
    li      $t3, 31               /* t3 = max value */

    slti    $t4, $t0, 32          /* t4 = (R < 32) ? 1 : 0 */
    movz    $t0, $t3, $t4         /* if (t4 == 0) R = 31 */

    slti    $t4, $t1, 32          /* t4 = (G < 32) ? 1 : 0 */
    movz    $t1, $t3, $t4         /* if (t4 == 0) G = 31 */

    slti    $t4, $t2, 32          /* t4 = (B < 32) ? 1 : 0 */
    movz    $t2, $t3, $t4         /* if (t4 == 0) B = 31 */

    /* Pack result: R | (G << 5) | (B << 10) */
    sll     $t1, $t1, 5           /* G << 5 */
    sll     $t2, $t2, 10          /* B << 10 */
    or      $v0, $t0, $t1         /* R | G */
    or      $v0, $v0, $t2         /* | B */

    jr      $ra
    nop

    .end    gpuLightingTXT_ASM


/*=============================================================================
 * gpuLightingTXTGouraud_ASM - MIPS32 optimized Gouraud texture lighting
 *=============================================================================
 * Input:  a0 = uSrc (u16, bgr555)
 *         a1 = gCol (u32, packed gouraud: rrrrrrrrXXXggggggggXXXbbbbbbbbXX)
 * Output: v0 = lit pixel (u16, bgr555)
 *=============================================================================*/
    .globl  gpuLightingTXTGouraud_ASM
    .ent    gpuLightingTXTGouraud_ASM
gpuLightingTXTGouraud_ASM:
    /* Extract light values from packed gouraud color */
    srl     $t5, $a1, 27          /* r5 = gCol >> 27 */
    ext     $t6, $a1, 16, 5       /* g5 = (gCol >> 16) & 0x1F */
    ext     $t7, $a1, 5, 5        /* b5 = (gCol >> 5) & 0x1F */

    /* Fast path: neutral lighting (15,15,15) */
    li      $t0, 15
    bne     $t5, $t0, .L_gouraud_do
    nop
    bne     $t6, $t0, .L_gouraud_do
    nop
    bne     $t7, $t0, .L_gouraud_do
    move    $v0, $a0
    jr      $ra
    nop

.L_gouraud_do:
    /* Extract texture RGB */
    ext     $t0, $a0, 0, 5        /* R */
    ext     $t1, $a0, 5, 5        /* G */
    ext     $t2, $a0, 10, 5       /* B */

    /* Multiply */
    mul     $t0, $t0, $t5         /* R * r5 */
    mul     $t1, $t1, $t6         /* G * g5 */
    mul     $t2, $t2, $t7         /* B * b5 */

    /* Shift */
    srl     $t0, $t0, 4
    srl     $t1, $t1, 4
    srl     $t2, $t2, 4

    /* Branchless clamp */
    li      $t3, 31
    slti    $t4, $t0, 32
    movz    $t0, $t3, $t4
    slti    $t4, $t1, 32
    movz    $t1, $t3, $t4
    slti    $t4, $t2, 32
    movz    $t2, $t3, $t4

    /* Pack */
    sll     $t1, $t1, 5
    sll     $t2, $t2, 10
    or      $v0, $t0, $t1
    or      $v0, $v0, $t2

    jr      $ra
    nop

    .end    gpuLightingTXTGouraud_ASM


/*=============================================================================
 * gpuBlendingMode0_ASM - 50% blend (0.5*Back + 0.5*Forward)
 *=============================================================================
 * Input:  a0 = uSrc (foreground)
 *         a1 = uDst (background)
 * Output: v0 = blended pixel
 *=============================================================================*/
    .globl  gpuBlendingMode0_ASM
    .ent    gpuBlendingMode0_ASM
gpuBlendingMode0_ASM:
    /* Simple 50% blend: ((uDst & 0x7bde) + (uSrc & 0x7bde)) >> 1 */
    li      $t0, 0x7bde
    and     $t1, $a0, $t0         /* uSrc & 0x7bde */
    and     $t2, $a1, $t0         /* uDst & 0x7bde */
    addu    $t3, $t1, $t2         /* sum */
    srl     $v0, $t3, 1           /* >> 1 */
    jr      $ra
    nop

    .end    gpuBlendingMode0_ASM


/*=============================================================================
 * gpuBlendingMode1_ASM - Additive blend (1.0*Back + 1.0*Forward) with saturation
 *=============================================================================
 * Input:  a0 = uSrc (foreground)
 *         a1 = uDst (background)
 * Output: v0 = blended pixel (saturated)
 *=============================================================================*/
    .globl  gpuBlendingMode1_ASM
    .ent    gpuBlendingMode1_ASM
gpuBlendingMode1_ASM:
    /* Extract and add RGB components */

    /* Red channel */
    ext     $t0, $a0, 0, 5        /* src.R */
    ext     $t1, $a1, 0, 5        /* dst.R */
    addu    $t0, $t0, $t1         /* R = src.R + dst.R */

    /* Green channel */
    ext     $t1, $a0, 5, 5        /* src.G */
    ext     $t2, $a1, 5, 5        /* dst.G */
    addu    $t1, $t1, $t2         /* G = src.G + dst.G */

    /* Blue channel */
    ext     $t2, $a0, 10, 5       /* src.B */
    ext     $t3, $a1, 10, 5       /* dst.B */
    addu    $t2, $t2, $t3         /* B = src.B + dst.B */

    /* Branchless saturate to 31 */
    li      $t4, 31

    slti    $t5, $t0, 32
    movz    $t0, $t4, $t5         /* R = min(R, 31) */

    slti    $t5, $t1, 32
    movz    $t1, $t4, $t5         /* G = min(G, 31) */

    slti    $t5, $t2, 32
    movz    $t2, $t4, $t5         /* B = min(B, 31) */

    /* Pack result */
    sll     $t1, $t1, 5
    sll     $t2, $t2, 10
    or      $v0, $t0, $t1
    or      $v0, $v0, $t2

    jr      $ra
    nop

    .end    gpuBlendingMode1_ASM


/*=============================================================================
 * gpuBlendingMode2_ASM - Subtractive blend (1.0*Back - 1.0*Forward)
 *=============================================================================
 * Input:  a0 = uSrc (foreground, to subtract)
 *         a1 = uDst (background)
 * Output: v0 = blended pixel (clamped to 0)
 *=============================================================================*/
    .globl  gpuBlendingMode2_ASM
    .ent    gpuBlendingMode2_ASM
gpuBlendingMode2_ASM:
    /* Extract RGB from both pixels */
    ext     $t0, $a1, 0, 5        /* dst.R */
    ext     $t1, $a0, 0, 5        /* src.R */
    subu    $t0, $t0, $t1         /* R = dst.R - src.R */

    ext     $t1, $a1, 5, 5        /* dst.G */
    ext     $t2, $a0, 5, 5        /* src.G */
    subu    $t1, $t1, $t2         /* G = dst.G - src.G */

    ext     $t2, $a1, 10, 5       /* dst.B */
    ext     $t3, $a0, 10, 5       /* src.B */
    subu    $t2, $t2, $t3         /* B = dst.B - src.B */

    /* Branchless clamp to 0 for negative values */
    /* if (x < 0) x = 0  using: x = x & ~(x >> 31) */
    sra     $t3, $t0, 31          /* t3 = (R < 0) ? -1 : 0 */
    not     $t3, $t3              /* t3 = (R < 0) ? 0 : -1 */
    and     $t0, $t0, $t3         /* R = (R < 0) ? 0 : R */

    sra     $t3, $t1, 31
    not     $t3, $t3
    and     $t1, $t1, $t3         /* G clamped */

    sra     $t3, $t2, 31
    not     $t3, $t3
    and     $t2, $t2, $t3         /* B clamped */

    /* Pack result */
    sll     $t1, $t1, 5
    sll     $t2, $t2, 10
    or      $v0, $t0, $t1
    or      $v0, $v0, $t2

    jr      $ra
    nop

    .end    gpuBlendingMode2_ASM


/*=============================================================================
 * gpuBlendingMode3_ASM - Additive 25% (1.0*Back + 0.25*Forward)
 *=============================================================================
 * Input:  a0 = uSrc (foreground)
 *         a1 = uDst (background)
 * Output: v0 = blended pixel
 *=============================================================================*/
    .globl  gpuBlendingMode3_ASM
    .ent    gpuBlendingMode3_ASM
gpuBlendingMode3_ASM:
    /* Extract src RGB and divide by 4 */
    ext     $t0, $a0, 0, 5        /* src.R */
    ext     $t1, $a0, 5, 5        /* src.G */
    ext     $t2, $a0, 10, 5       /* src.B */
    srl     $t0, $t0, 2           /* src.R / 4 */
    srl     $t1, $t1, 2           /* src.G / 4 */
    srl     $t2, $t2, 2           /* src.B / 4 */

    /* Extract dst RGB and add */
    ext     $t3, $a1, 0, 5        /* dst.R */
    ext     $t4, $a1, 5, 5        /* dst.G */
    ext     $t5, $a1, 10, 5       /* dst.B */
    addu    $t0, $t0, $t3         /* R */
    addu    $t1, $t1, $t4         /* G */
    addu    $t2, $t2, $t5         /* B */

    /* Branchless saturate */
    li      $t6, 31
    slti    $t7, $t0, 32
    movz    $t0, $t6, $t7
    slti    $t7, $t1, 32
    movz    $t1, $t6, $t7
    slti    $t7, $t2, 32
    movz    $t2, $t6, $t7

    /* Pack */
    sll     $t1, $t1, 5
    sll     $t2, $t2, 10
    or      $v0, $t0, $t1
    or      $v0, $v0, $t2

    jr      $ra
    nop

    .end    gpuBlendingMode3_ASM


/*=============================================================================
 * gpuPixelPrefetch_ASM - Prefetch texture data
 *=============================================================================
 * Input:  a0 = texture pointer
 *         a1 = offset (bytes ahead to prefetch)
 * Output: none (advisory hint to cache)
 *=============================================================================*/
    .globl  gpuPixelPrefetch_ASM
    .ent    gpuPixelPrefetch_ASM
gpuPixelPrefetch_ASM:
    addu    $t0, $a0, $a1
    pref    0, 0($t0)             /* hint 0 = load */
    jr      $ra
    nop

    .end    gpuPixelPrefetch_ASM

