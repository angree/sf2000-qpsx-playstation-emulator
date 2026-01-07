/***************************************************************************
 *   QPSX v091 - MIPS32 Assembly Functions Header
 *
 *   Declarations for MIPS32 assembly optimized GPU functions.
 *   These are only available when compiled for SF2000/MIPS32.
 ***************************************************************************/

#ifndef GPU_INNER_MIPS32_H
#define GPU_INNER_MIPS32_H

#include "gpu_unai.h"

/*
 * QPSX v091: MIPS32 Assembly GPU Optimizations
 *
 * These functions provide branchless implementations using:
 * - MOVN/MOVZ: Conditional moves for clamping without branches
 * - EXT/INS: Bit field extraction/insertion for RGB565
 * - PREF: Cache prefetch hints for texture data
 *
 * All functions are optional - controlled via menu toggles.
 * When disabled, falls back to C implementations.
 */

#if defined(SF2000) || defined(__mips__)

#ifdef __cplusplus
extern "C" {
#endif

/*
 * gpuLightingTXT_ASM - Apply lighting to textured pixel
 *
 * @param uSrc: Source pixel (bgr555 format)
 * @param r5: Red light value (0-31, 15=neutral)
 * @param g5: Green light value (0-31, 15=neutral)
 * @param b5: Blue light value (0-31, 15=neutral)
 * @return: Lit pixel (bgr555)
 *
 * Uses branchless MOVN/MOVZ for clamping.
 * ~20-30% faster than C version with branches.
 */
u16 gpuLightingTXT_ASM(u16 uSrc, u8 r5, u8 g5, u8 b5);

/*
 * gpuLightingTXTGouraud_ASM - Apply Gouraud lighting to textured pixel
 *
 * @param uSrc: Source pixel (bgr555)
 * @param gCol: Packed Gouraud color (rrrrrrrrXXXggggggggXXXbbbbbbbbXX)
 * @return: Lit pixel (bgr555)
 */
u16 gpuLightingTXTGouraud_ASM(u16 uSrc, u32 gCol);

/*
 * gpuBlendingMode0_ASM - 50% blend (0.5*Back + 0.5*Forward)
 *
 * @param uSrc: Foreground pixel
 * @param uDst: Background pixel
 * @return: Blended pixel
 */
u16 gpuBlendingMode0_ASM(u16 uSrc, u16 uDst);

/*
 * gpuBlendingMode1_ASM - Additive blend (1.0*Back + 1.0*Forward)
 *
 * @param uSrc: Foreground pixel
 * @param uDst: Background pixel
 * @return: Blended pixel (saturated to 31 per channel)
 *
 * Uses branchless saturation via MOVZ.
 */
u16 gpuBlendingMode1_ASM(u16 uSrc, u16 uDst);

/*
 * gpuBlendingMode2_ASM - Subtractive blend (1.0*Back - 1.0*Forward)
 *
 * @param uSrc: Foreground pixel (to subtract)
 * @param uDst: Background pixel
 * @return: Blended pixel (clamped to 0 per channel)
 *
 * Uses branchless clamping via arithmetic.
 */
u16 gpuBlendingMode2_ASM(u16 uSrc, u16 uDst);

/*
 * gpuBlendingMode3_ASM - Additive 25% (1.0*Back + 0.25*Forward)
 *
 * @param uSrc: Foreground pixel
 * @param uDst: Background pixel
 * @return: Blended pixel
 */
u16 gpuBlendingMode3_ASM(u16 uSrc, u16 uDst);

/*
 * gpuPixelPrefetch_ASM - Prefetch texture data into cache
 *
 * @param ptr: Texture pointer
 * @param offset: Bytes ahead to prefetch
 *
 * Advisory hint - may be ignored by CPU.
 * Use in inner loops: prefetch 64-128 bytes ahead.
 */
void gpuPixelPrefetch_ASM(const void* ptr, u32 offset);

#ifdef __cplusplus
}
#endif

/*
 * Template wrapper for ASM blending
 * Calls appropriate ASM function based on BLENDMODE
 */
template <int BLENDMODE>
static inline u16 gpuBlending_ASM(u16 uSrc, u16 uDst)
{
    if (BLENDMODE == 0) return gpuBlendingMode0_ASM(uSrc, uDst);
    if (BLENDMODE == 1) return gpuBlendingMode1_ASM(uSrc, uDst);
    if (BLENDMODE == 2) return gpuBlendingMode2_ASM(uSrc, uDst);
    if (BLENDMODE == 3) return gpuBlendingMode3_ASM(uSrc, uDst);
    return uSrc;
}

#else /* !SF2000 && !__mips__ */

/* Fallback stubs for non-MIPS platforms (compilation only) */
#define gpuLightingTXT_ASM(uSrc, r5, g5, b5)     gpuLightingTXT_Fast(uSrc, r5, g5, b5)
#define gpuLightingTXTGouraud_ASM(uSrc, gCol)    gpuLightingTXTGouraud_Fast(uSrc, gCol)
#define gpuBlendingMode0_ASM(uSrc, uDst)         gpuBlendingFast_Mode0(uSrc, uDst)
#define gpuBlendingMode1_ASM(uSrc, uDst)         gpuBlendingFast_Mode1(uSrc, uDst)
#define gpuBlendingMode2_ASM(uSrc, uDst)         gpuBlendingFast_Mode2(uSrc, uDst)
#define gpuBlendingMode3_ASM(uSrc, uDst)         gpuBlendingFast_Mode3(uSrc, uDst)
#define gpuPixelPrefetch_ASM(ptr, offset)        ((void)0)

template <int BLENDMODE>
static inline u16 gpuBlending_ASM(u16 uSrc, u16 uDst)
{
    return gpuBlending_Fast<BLENDMODE>(uSrc, uDst);
}

#endif /* SF2000 || __mips__ */

#endif /* GPU_INNER_MIPS32_H */
