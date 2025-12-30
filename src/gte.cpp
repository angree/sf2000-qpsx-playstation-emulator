/*  PCSX-Revolution - PS Emulator for Nintendo Wii
 *  Copyright (C) 2009-2010  PCSX-Revolution Dev Team
 *
 *  PCSX-Revolution is free software: you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation, either version 2 of the
 *  License, or (at your option) any later version.
 *
 *  PCSX-Revolution is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with PCSX-Revolution. If not, see <http://www.gnu.org/licenses/>.
 */

/*
* GTE functions.
*/

#include "gte.h"
#include "psxmem.h"

// MIPS platforms have hardware divider, faster than 64KB LUT + UNR algo
#if defined(__mips__)
#define GTE_USE_NATIVE_DIVIDE
#endif

/*
 * QPSX v087: Optimized RTPT implementation
 * When enabled, uses loop unrolling and pre-loaded rotation matrix.
 * This avoids repeated memory accesses and enables better register allocation.
 */
#if defined(__mips__) || defined(SF2000)
#define GTE_USE_FAST_RTPT
#endif

/* External config flag for enabling/disabling fast RTPT (default: disabled) */
extern int g_opt_rtpt_unroll;

/*
 * QPSX v088: Two new GTE optimizations (EXPERIMENTAL - RISK!)
 *
 * 1. opt_gte_unrdiv - Use UNR (Unsigned Newton-Raphson) division
 *    Instead of hardware DIV (~36 cycles), use table lookup + 2 multiply iterations.
 *    Based on PSX hardware algorithm from psx-spx documentation.
 *    May or may not be faster depending on cache behavior.
 *
 * 2. opt_gte_noflags - Skip overflow checking in A1/A2/A3/F macros
 *    !!WARNING!! This can break games that read GTE flags!
 *    Most games don't care about overflow flags, but some do.
 *    Based on PCSX-ReARMed "FLAGLESS" optimization.
 */
extern int g_opt_gte_unrdiv;   /* UNR division (default: OFF) */
extern int g_opt_gte_noflags;  /* Skip overflow flags (default: OFF) - RISK! */

// (This is a backported optimization from PCSX Rearmed -senquack)
// On slower platforms, bounds-check is disabled on some calculations,
//  for now only some involving far colors. The overflow results from these
//  would likely never be used and just waste cycles. Overflow flags are
//  still set for other calculations.
#if !(defined(__arm__) || defined(__mips__))
#define PARANOID_OVERFLOW_CHECKING
#endif

#define VX(n) (n < 3 ? psxRegs.CP2D.p[n << 1].sw.l : psxRegs.CP2D.p[9].sw.l)
#define VY(n) (n < 3 ? psxRegs.CP2D.p[n << 1].sw.h : psxRegs.CP2D.p[10].sw.l)
#define VZ(n) (n < 3 ? psxRegs.CP2D.p[(n << 1) + 1].sw.l : psxRegs.CP2D.p[11].sw.l)
#define MX11(n) (n < 3 ? psxRegs.CP2C.p[(n << 3)].sw.l : 0)
#define MX12(n) (n < 3 ? psxRegs.CP2C.p[(n << 3)].sw.h : 0)
#define MX13(n) (n < 3 ? psxRegs.CP2C.p[(n << 3) + 1].sw.l : 0)
#define MX21(n) (n < 3 ? psxRegs.CP2C.p[(n << 3) + 1].sw.h : 0)
#define MX22(n) (n < 3 ? psxRegs.CP2C.p[(n << 3) + 2].sw.l : 0)
#define MX23(n) (n < 3 ? psxRegs.CP2C.p[(n << 3) + 2].sw.h : 0)
#define MX31(n) (n < 3 ? psxRegs.CP2C.p[(n << 3) + 3].sw.l : 0)
#define MX32(n) (n < 3 ? psxRegs.CP2C.p[(n << 3) + 3].sw.h : 0)
#define MX33(n) (n < 3 ? psxRegs.CP2C.p[(n << 3) + 4].sw.l : 0)
#define CV1(n) (n < 3 ? (s32)psxRegs.CP2C.r[(n << 3) + 5] : 0)
#define CV2(n) (n < 3 ? (s32)psxRegs.CP2C.r[(n << 3) + 6] : 0)
#define CV3(n) (n < 3 ? (s32)psxRegs.CP2C.r[(n << 3) + 7] : 0)

#define fSX(n) ((psxRegs.CP2D.p)[((n) + 12)].sw.l)
#define fSY(n) ((psxRegs.CP2D.p)[((n) + 12)].sw.h)
#define fSZ(n) ((psxRegs.CP2D.p)[((n) + 17)].w.l) /* (n == 0) => SZ1; */

#define gteVXY0 (psxRegs.CP2D.r[0])
#define gteVX0  (psxRegs.CP2D.p[0].sw.l)
#define gteVY0  (psxRegs.CP2D.p[0].sw.h)
#define gteVZ0  (psxRegs.CP2D.p[1].sw.l)
#define gteVXY1 (psxRegs.CP2D.r[2])
#define gteVX1  (psxRegs.CP2D.p[2].sw.l)
#define gteVY1  (psxRegs.CP2D.p[2].sw.h)
#define gteVZ1  (psxRegs.CP2D.p[3].sw.l)
#define gteVXY2 (psxRegs.CP2D.r[4])
#define gteVX2  (psxRegs.CP2D.p[4].sw.l)
#define gteVY2  (psxRegs.CP2D.p[4].sw.h)
#define gteVZ2  (psxRegs.CP2D.p[5].sw.l)
#define gteRGB  (psxRegs.CP2D.r[6])
#define gteR    (psxRegs.CP2D.p[6].b.l)
#define gteG    (psxRegs.CP2D.p[6].b.h)
#define gteB    (psxRegs.CP2D.p[6].b.h2)
#define gteCODE (psxRegs.CP2D.p[6].b.h3)
#define gteOTZ  (psxRegs.CP2D.p[7].w.l)
#define gteIR0  (psxRegs.CP2D.p[8].sw.l)
#define gteIR1  (psxRegs.CP2D.p[9].sw.l)
#define gteIR2  (psxRegs.CP2D.p[10].sw.l)
#define gteIR3  (psxRegs.CP2D.p[11].sw.l)
#define gteSXY0 (psxRegs.CP2D.r[12])
#define gteSX0  (psxRegs.CP2D.p[12].sw.l)
#define gteSY0  (psxRegs.CP2D.p[12].sw.h)
#define gteSXY1 (psxRegs.CP2D.r[13])
#define gteSX1  (psxRegs.CP2D.p[13].sw.l)
#define gteSY1  (psxRegs.CP2D.p[13].sw.h)
#define gteSXY2 (psxRegs.CP2D.r[14])
#define gteSX2  (psxRegs.CP2D.p[14].sw.l)
#define gteSY2  (psxRegs.CP2D.p[14].sw.h)
#define gteSXYP (psxRegs.CP2D.r[15])
#define gteSXP  (psxRegs.CP2D.p[15].sw.l)
#define gteSYP  (psxRegs.CP2D.p[15].sw.h)
#define gteSZ0  (psxRegs.CP2D.p[16].w.l)
#define gteSZ1  (psxRegs.CP2D.p[17].w.l)
#define gteSZ2  (psxRegs.CP2D.p[18].w.l)
#define gteSZ3  (psxRegs.CP2D.p[19].w.l)
#define gteRGB0  (psxRegs.CP2D.r[20])
#define gteR0    (psxRegs.CP2D.p[20].b.l)
#define gteG0    (psxRegs.CP2D.p[20].b.h)
#define gteB0    (psxRegs.CP2D.p[20].b.h2)
#define gteCODE0 (psxRegs.CP2D.p[20].b.h3)
#define gteRGB1  (psxRegs.CP2D.r[21])
#define gteR1    (psxRegs.CP2D.p[21].b.l)
#define gteG1    (psxRegs.CP2D.p[21].b.h)
#define gteB1    (psxRegs.CP2D.p[21].b.h2)
#define gteCODE1 (psxRegs.CP2D.p[21].b.h3)
#define gteRGB2  (psxRegs.CP2D.r[22])
#define gteR2    (psxRegs.CP2D.p[22].b.l)
#define gteG2    (psxRegs.CP2D.p[22].b.h)
#define gteB2    (psxRegs.CP2D.p[22].b.h2)
#define gteCODE2 (psxRegs.CP2D.p[22].b.h3)
#define gteRES1  (psxRegs.CP2D.r[23])
#define gteMAC0  (((s32 *)psxRegs.CP2D.r)[24])
#define gteMAC1  (((s32 *)psxRegs.CP2D.r)[25])
#define gteMAC2  (((s32 *)psxRegs.CP2D.r)[26])
#define gteMAC3  (((s32 *)psxRegs.CP2D.r)[27])
#define gteIRGB  (psxRegs.CP2D.r[28])
#define gteORGB  (psxRegs.CP2D.r[29])
#define gteLZCS  (psxRegs.CP2D.r[30])
#define gteLZCR  (psxRegs.CP2D.r[31])

#define gteR11R12 (((s32 *)psxRegs.CP2C.r)[0])
#define gteR22R23 (((s32 *)psxRegs.CP2C.r)[2])
#define gteR11 (psxRegs.CP2C.p[0].sw.l)
#define gteR12 (psxRegs.CP2C.p[0].sw.h)
#define gteR13 (psxRegs.CP2C.p[1].sw.l)
#define gteR21 (psxRegs.CP2C.p[1].sw.h)
#define gteR22 (psxRegs.CP2C.p[2].sw.l)
#define gteR23 (psxRegs.CP2C.p[2].sw.h)
#define gteR31 (psxRegs.CP2C.p[3].sw.l)
#define gteR32 (psxRegs.CP2C.p[3].sw.h)
#define gteR33 (psxRegs.CP2C.p[4].sw.l)
#define gteTRX (((s32 *)psxRegs.CP2C.r)[5])
#define gteTRY (((s32 *)psxRegs.CP2C.r)[6])
#define gteTRZ (((s32 *)psxRegs.CP2C.r)[7])
#define gteL11 (psxRegs.CP2C.p[8].sw.l)
#define gteL12 (psxRegs.CP2C.p[8].sw.h)
#define gteL13 (psxRegs.CP2C.p[9].sw.l)
#define gteL21 (psxRegs.CP2C.p[9].sw.h)
#define gteL22 (psxRegs.CP2C.p[10].sw.l)
#define gteL23 (psxRegs.CP2C.p[10].sw.h)
#define gteL31 (psxRegs.CP2C.p[11].sw.l)
#define gteL32 (psxRegs.CP2C.p[11].sw.h)
#define gteL33 (psxRegs.CP2C.p[12].sw.l)
#define gteRBK (((s32 *)psxRegs.CP2C.r)[13])
#define gteGBK (((s32 *)psxRegs.CP2C.r)[14])
#define gteBBK (((s32 *)psxRegs.CP2C.r)[15])
#define gteLR1 (psxRegs.CP2C.p[16].sw.l)
#define gteLR2 (psxRegs.CP2C.p[16].sw.h)
#define gteLR3 (psxRegs.CP2C.p[17].sw.l)
#define gteLG1 (psxRegs.CP2C.p[17].sw.h)
#define gteLG2 (psxRegs.CP2C.p[18].sw.l)
#define gteLG3 (psxRegs.CP2C.p[18].sw.h)
#define gteLB1 (psxRegs.CP2C.p[19].sw.l)
#define gteLB2 (psxRegs.CP2C.p[19].sw.h)
#define gteLB3 (psxRegs.CP2C.p[20].sw.l)
#define gteRFC (((s32 *)psxRegs.CP2C.r)[21])
#define gteGFC (((s32 *)psxRegs.CP2C.r)[22])
#define gteBFC (((s32 *)psxRegs.CP2C.r)[23])
#define gteOFX (((s32 *)psxRegs.CP2C.r)[24])
#define gteOFY (((s32 *)psxRegs.CP2C.r)[25])

// senquack - gteH register is u16, not s16, and used in GTE that way.
//  HOWEVER when read back by CPU using CFC2, it will be incorrectly
//  sign-extended by bug in original hardware, according to Nocash docs
//  GTE section 'Screen Offset and Distance'. The emulator does this
//  sign extension when it is loaded to GTE by CTC2.
//#define gteH   (psxRegs.CP2C.p[26].sw.l)
#define gteH   (psxRegs.CP2C.p[26].w.l)

#define gteDQA (psxRegs.CP2C.p[27].sw.l)
#define gteDQB (((s32 *)psxRegs.CP2C.r)[28])
#define gteZSF3 (psxRegs.CP2C.p[29].sw.l)
#define gteZSF4 (psxRegs.CP2C.p[30].sw.l)
#define gteFLAG (psxRegs.CP2C.r[31])

// Some GTE instructions encode various parameters in their 32-bit opcode.
//  Rather than passing the opcode value through the psxRegisters 'opcode'
//  field, we pass it to GTE funcs as an argument *pre-shifted* right by 10
//  places. This allows dynarecs to emit faster calls to GTE functions.
//  These helper macros interpret this pre-shifted opcode argument.
#define GTE_SF(op) ((op >>  9)  & 1)
#define GTE_MX(op) ((op >>  7)  & 3)
#define GTE_V(op)  ((op >>  5)  & 3)
#define GTE_CV(op) ((op >>  3)  & 3)
#define GTE_LM(op) ((op >>  0)  & 1)

/*
 * QPSX v088: BOUNDS and LIM with optional flag skip
 *
 * When g_opt_gte_noflags is set, overflow flag computation is skipped.
 * !!WARNING!! This can break games that rely on reading GTE flags!
 * Most 3D games don't check these flags, but some might.
 *
 * senquack note: Don't try to optimize return value to s32 like PCSX Rearmed
 * did here - it's why as of Nov. 2016, PC build has gfx glitches in 'Driver'
 */
INLINE s64 BOUNDS(s64 n_value, s64 n_max, int n_maxflag, s64 n_min, int n_minflag) {
	/* v088: Skip flag computation if noflags optimization enabled */
	if (!g_opt_gte_noflags) {
		if (n_value > n_max) {
			gteFLAG |= n_maxflag;
		} else if (n_value < n_min) {
			gteFLAG |= n_minflag;
		}
	}
	return n_value;
}

INLINE s32 LIM(s32 value, s32 max, s32 min, u32 flag) {
	s32 ret = value;
	if (value > max) {
		/* v088: Still clamp, but optionally skip flag */
		if (!g_opt_gte_noflags) gteFLAG |= flag;
		ret = max;
	} else if (value < min) {
		if (!g_opt_gte_noflags) gteFLAG |= flag;
		ret = min;
	}
	return ret;
}

#define A1(a) BOUNDS((a), 0x7fffffff, (1 << 30), -(s64)0x80000000, (1 << 31) | (1 << 27))
#define A2(a) BOUNDS((a), 0x7fffffff, (1 << 29), -(s64)0x80000000, (1 << 31) | (1 << 26))
#define A3(a) BOUNDS((a), 0x7fffffff, (1 << 28), -(s64)0x80000000, (1 << 31) | (1 << 25))
#define limB1(a, l) LIM((a), 0x7fff, -0x8000 * !l, (1 << 31) | (1 << 24))
#define limB2(a, l) LIM((a), 0x7fff, -0x8000 * !l, (1 << 31) | (1 << 23))
#define limB3(a, l) LIM((a), 0x7fff, -0x8000 * !l, (1 << 22) )
#define limC1(a) LIM((a), 0x00ff, 0x0000, (1 << 21) )
#define limC2(a) LIM((a), 0x00ff, 0x0000, (1 << 20) )
#define limC3(a) LIM((a), 0x00ff, 0x0000, (1 << 19) )
#define limD(a) LIM((a), 0xffff, 0x0000, (1 << 31) | (1 << 18))

INLINE u32 limE(u32 result) {
	if (result > 0x1ffff) {
		gteFLAG |= (1 << 31) | (1 << 17);
		return 0x1ffff;
	}

	return result;
}

#define F(a) BOUNDS((a), 0x7fffffff, (1 << 31) | (1 << 16), -(s64)0x80000000, (1 << 31) | (1 << 15))
#define limG1(a) LIM((a), 0x3ff, -0x400, (1 << 31) | (1 << 14))
#define limG2(a) LIM((a), 0x3ff, -0x400, (1 << 31) | (1 << 13))
//Fix for Valkyrie Profile crash loading world map
// (PCSX Rearmed commit 7384197d8a5fd20a4d94f3517a6462f7fe86dd4c
//  'seems to work, unverified value')
//#define limH(a) LIM((a), 0xfff, 0x000, (1 << 12))
#define limH(a) LIM((a), 0x1000, 0x0000, (1 << 12))


#ifdef PARANOID_OVERFLOW_CHECKING
#define A1U A1
#define A2U A2
#define A3U A3
#else
// Any calculation explicitly using these forms of A1/A2/A3 indicates
//  these checks are very unlikely to be useful would just waste cycles
#define A1U(x) (x)
#define A2U(x) (x)
#define A3U(x) (x)
#endif

//senquack - n param should be unsigned (will be 'gteH' reg which is u16)
#ifdef GTE_USE_NATIVE_DIVIDE

/*
 * UNR Division Lookup Table (257 entries) - QPSX v088
 * Generated from formula: unr_table[i] = min(0, (0x40000/(i+0x100)+1)/2 - 0x101)
 * This matches the original PSX GTE hardware implementation.
 */
static const u8 unr_table[257] = {
    0xff, 0xfd, 0xfb, 0xf9, 0xf7, 0xf5, 0xf3, 0xf1,
    0xef, 0xee, 0xec, 0xea, 0xe8, 0xe6, 0xe4, 0xe3,
    0xe1, 0xdf, 0xdd, 0xdc, 0xda, 0xd8, 0xd6, 0xd5,
    0xd3, 0xd1, 0xd0, 0xce, 0xcd, 0xcb, 0xc9, 0xc8,
    0xc6, 0xc5, 0xc3, 0xc1, 0xc0, 0xbe, 0xbd, 0xbb,
    0xba, 0xb8, 0xb7, 0xb5, 0xb4, 0xb2, 0xb1, 0xb0,
    0xae, 0xad, 0xab, 0xaa, 0xa9, 0xa7, 0xa6, 0xa4,
    0xa3, 0xa2, 0xa0, 0x9f, 0x9e, 0x9c, 0x9b, 0x9a,
    0x99, 0x97, 0x96, 0x95, 0x94, 0x92, 0x91, 0x90,
    0x8f, 0x8d, 0x8c, 0x8b, 0x8a, 0x89, 0x87, 0x86,
    0x85, 0x84, 0x83, 0x82, 0x81, 0x7f, 0x7e, 0x7d,
    0x7c, 0x7b, 0x7a, 0x79, 0x78, 0x77, 0x75, 0x74,
    0x73, 0x72, 0x71, 0x70, 0x6f, 0x6e, 0x6d, 0x6c,
    0x6b, 0x6a, 0x69, 0x68, 0x67, 0x66, 0x65, 0x64,
    0x63, 0x62, 0x61, 0x60, 0x5f, 0x5e, 0x5d, 0x5d,
    0x5c, 0x5b, 0x5a, 0x59, 0x58, 0x57, 0x56, 0x55,
    0x54, 0x53, 0x53, 0x52, 0x51, 0x50, 0x4f, 0x4e,
    0x4d, 0x4d, 0x4c, 0x4b, 0x4a, 0x49, 0x49, 0x48,
    0x47, 0x46, 0x45, 0x45, 0x44, 0x43, 0x42, 0x42,
    0x41, 0x40, 0x3f, 0x3f, 0x3e, 0x3d, 0x3c, 0x3c,
    0x3b, 0x3a, 0x39, 0x39, 0x38, 0x37, 0x37, 0x36,
    0x35, 0x35, 0x34, 0x33, 0x33, 0x32, 0x31, 0x31,
    0x30, 0x2f, 0x2f, 0x2e, 0x2d, 0x2d, 0x2c, 0x2c,
    0x2b, 0x2a, 0x2a, 0x29, 0x28, 0x28, 0x27, 0x27,
    0x26, 0x25, 0x25, 0x24, 0x24, 0x23, 0x23, 0x22,
    0x21, 0x21, 0x20, 0x20, 0x1f, 0x1f, 0x1e, 0x1e,
    0x1d, 0x1d, 0x1c, 0x1c, 0x1b, 0x1a, 0x1a, 0x19,
    0x19, 0x18, 0x18, 0x17, 0x17, 0x16, 0x16, 0x15,
    0x15, 0x15, 0x14, 0x14, 0x13, 0x13, 0x12, 0x12,
    0x11, 0x11, 0x10, 0x10, 0x0f, 0x0f, 0x0f, 0x0e,
    0x0e, 0x0d, 0x0d, 0x0c, 0x0c, 0x0c, 0x0b, 0x0b,
    0x0a, 0x0a, 0x0a, 0x09, 0x09, 0x08, 0x08, 0x08,
    0x07  /* index 256 */
};

/*
 * UNR Division - matches PSX hardware algorithm (QPSX v088)
 * From psx-spx: https://psx-spx.consoledev.net/geometrytransformationenginegte/
 */
INLINE u32 DIVIDE_UNR(u16 n, u16 d) {
    if (n < d * 2) {
        /* Count leading zeros of d (16-bit value) */
        int z = __builtin_clz((u32)d) - 16;  /* clz for 32-bit, adjust for 16-bit */
        u32 n_shifted = (u32)n << z;
        u32 d_shifted = (u32)d << z;

        /* Table lookup: index = (d_shifted - 0x7fc0) >> 7 */
        u32 idx = (d_shifted - 0x7fc0) >> 7;
        if (idx > 256) idx = 256;
        u32 u = unr_table[idx] + 0x101;

        /* Two Newton-Raphson iterations */
        u32 d1 = ((0x2000080 - (d_shifted * u)) >> 8);
        u32 d2 = ((0x80 + (d1 * u)) >> 8);

        /* Final result */
        u32 result = (((u64)n_shifted * d2) + 0x8000) >> 16;
        if (result > 0x1ffff) result = 0x1ffff;
        return result;
    }
    return 0x1ffff;  /* Overflow */
}

INLINE u32 DIVIDE(u16 n, u16 d) {
	/*
	 * QPSX v088: Optionally use UNR (Newton-Raphson) division
	 * UNR uses lookup table + 2 multiply iterations instead of hardware DIV.
	 * Hardware DIV on MIPS32 is ~36 cycles, UNR may be faster with good cache.
	 * Enable with g_opt_gte_unrdiv option.
	 */
	if (g_opt_gte_unrdiv) {
		return DIVIDE_UNR(n, d);
	}
	/* Original native divide */
	if (n < d * 2) {
		return ((u32)n << 16) / d;
	}
	return 0xffffffff;
}
#else
#include "gte_divide.h"
#endif // GTE_USE_NATIVE_DIVIDE

//senquack - Applied fixes from PCSX Rearmed 7384197d8a5fd20a4d94f3517a6462f7fe86dd4c
// Case 28 now falls through to case 29, and don't return 0 for case 30
// Fixes main menu freeze in 'Lego Racers'
u32 gtecalcMFC2(int reg) {
	switch(reg) {
		case 1:
		case 3:
		case 5:
		case 8:
		case 9:
		case 10:
		case 11:
			psxRegs.CP2D.r[reg] = (s32)psxRegs.CP2D.p[reg].sw.l;
			break;

		case 7:
		case 16:
		case 17:
		case 18:
		case 19:
			psxRegs.CP2D.r[reg] = (u32)psxRegs.CP2D.p[reg].w.l;
			break;

		case 15:
			psxRegs.CP2D.r[reg] = gteSXY2;
			break;

		case 28:
		case 29:
			psxRegs.CP2D.r[reg] = LIM(gteIR1 >> 7, 0x1f, 0, 0) |
									(LIM(gteIR2 >> 7, 0x1f, 0, 0) << 5) |
									(LIM(gteIR3 >> 7, 0x1f, 0, 0) << 10);
			break;
	}
	return psxRegs.CP2D.r[reg];
}

//senquack - Applied fixes from PCSX Rearmed 7384197d8a5fd20a4d94f3517a6462f7fe86dd4c
// Don't block writing to regs 7,29 despite Nocash listing them as read-only.
// Fixes disappearing elements in 'Motor Toon' game series.
void gtecalcMTC2(u32 value, int reg) {
	switch (reg) {
		case 15:
			gteSXY0 = gteSXY1;
			gteSXY1 = gteSXY2;
			gteSXY2 = value;
			gteSXYP = value;
			break;

		case 28:
			gteIRGB = value;
			gteIR1 = (value & 0x1f) << 7;
			gteIR2 = (value & 0x3e0) << 2;
			gteIR3 = (value & 0x7c00) >> 3;
			break;

		case 30:
			{
				int a;
				gteLZCS = value;

				a = gteLZCS;
				if (a > 0) {
					int i;
					for (i = 31; (a & (1 << i)) == 0 && i >= 0; i--);
					gteLZCR = 31 - i;
				} else if (a < 0) {
					int i;
					a ^= 0xffffffff;
					for (i=31; (a & (1 << i)) == 0 && i >= 0; i--);
					gteLZCR = 31 - i;
				} else {
					gteLZCR = 32;
				}
			}
			break;

		case 31:
			return;

		default:
			psxRegs.CP2D.r[reg] = value;
	}
}

void gtecalcCTC2(u32 value, int reg) {
	switch (reg) {
		case 4:
		case 12:
		case 20:
		case 26:
		case 27:
		case 29:
		case 30:
			value = (s32)(s16)value;
			break;

		case 31:
			value = value & 0x7ffff000;
			if (value & 0x7f87e000) value |= 0x80000000;
			break;
	}

	psxRegs.CP2C.r[reg] = value;
}

void gteMFC2(void) {
	if (!_Rt_) return;
	psxRegs.GPR.r[_Rt_] = gtecalcMFC2(_Rd_);
}

void gteCFC2(void) {
	if (!_Rt_) return;
	psxRegs.GPR.r[_Rt_] = psxRegs.CP2C.r[_Rd_];
}

void gteMTC2(void) {
	gtecalcMTC2(psxRegs.GPR.r[_Rt_], _Rd_);
}

void gteCTC2(void) {
	gtecalcCTC2(psxRegs.GPR.r[_Rt_], _Rd_);
}

#define _oB_ (psxRegs.GPR.r[_Rs_] + _Imm_)

void gteLWC2(void) {
	gtecalcMTC2(psxMemRead32(_oB_), _Rt_);
}

void gteSWC2(void) {
	psxMemWrite32(_oB_, gtecalcMFC2(_Rt_));
}

void gteRTPS(void) {
	int quotient;

#ifdef GTE_LOG
	GTE_LOG("GTE RTPS\n");
#endif
	gteFLAG = 0;

	gteMAC1 = A1((((s64)gteTRX << 12) + (gteR11 * gteVX0) + (gteR12 * gteVY0) + (gteR13 * gteVZ0)) >> 12);
	gteMAC2 = A2((((s64)gteTRY << 12) + (gteR21 * gteVX0) + (gteR22 * gteVY0) + (gteR23 * gteVZ0)) >> 12);
	gteMAC3 = A3((((s64)gteTRZ << 12) + (gteR31 * gteVX0) + (gteR32 * gteVY0) + (gteR33 * gteVZ0)) >> 12);
	gteIR1 = limB1(gteMAC1, 0);
	gteIR2 = limB2(gteMAC2, 0);
	gteIR3 = limB3(gteMAC3, 0);
	gteSZ0 = gteSZ1;
	gteSZ1 = gteSZ2;
	gteSZ2 = gteSZ3;
	gteSZ3 = limD(gteMAC3);
	quotient = limE(DIVIDE(gteH, gteSZ3));
	gteSXY0 = gteSXY1;
	gteSXY1 = gteSXY2;
	gteSX2 = limG1(F((s64)gteOFX + ((s64)gteIR1 * quotient)) >> 16);
	gteSY2 = limG2(F((s64)gteOFY + ((s64)gteIR2 * quotient)) >> 16);

	//senquack - Fix glitched drawing of road surface in 'Burning Road'..
	// behavior now matches Mednafen. This also preserves the fix by Shalma
	// from prior commit f916013 for missing elements in 'Legacy of Kain:
	// Soul Reaver' (missing green plasma balls in first level).
	s64 tmp = (s64)gteDQB + ((s64)gteDQA * quotient);
	gteMAC0 = F(tmp);
	gteIR0 = limH(tmp >> 12);
}

/*
 * QPSX v087: Optimized RTPT with loop unrolling and pre-loaded constants
 *
 * Original RTPT loops 3 times, reloading rotation matrix elements each iteration.
 * This version:
 * 1. Pre-loads rotation matrix R11-R33 and translation TRX/TRY/TRZ into locals
 * 2. Pre-loads projection constants OFX, OFY, H, DQA, DQB
 * 3. Fully unrolls the loop for 3 vertices
 * 4. Uses direct vertex access (gteVX0/1/2) instead of VX() macro
 *
 * This enables GCC to keep values in registers across all 3 vertex transforms,
 * avoiding repeated memory loads of the same constants.
 */
#ifdef GTE_USE_FAST_RTPT
static void gteRTPT_fast(void) {
	int quotient;

	gteFLAG = 0;

	/* Pre-load rotation matrix (stays constant for all 3 vertices) */
	const s16 r11 = gteR11, r12 = gteR12, r13 = gteR13;
	const s16 r21 = gteR21, r22 = gteR22, r23 = gteR23;
	const s16 r31 = gteR31, r32 = gteR32, r33 = gteR33;

	/* Pre-load translation vector */
	const s32 trx = gteTRX, try_ = gteTRY, trz = gteTRZ;

	/* Pre-load projection constants */
	const s32 ofx = gteOFX, ofy = gteOFY;
	const u16 h = gteH;

	/* SZ FIFO shift */
	gteSZ0 = gteSZ3;

	/* ========== VERTEX 0 ========== */
	{
		const s32 vx = gteVX0, vy = gteVY0, vz = gteVZ0;

		gteMAC1 = A1((((s64)trx << 12) + (r11 * vx) + (r12 * vy) + (r13 * vz)) >> 12);
		gteMAC2 = A2((((s64)try_ << 12) + (r21 * vx) + (r22 * vy) + (r23 * vz)) >> 12);
		gteMAC3 = A3((((s64)trz << 12) + (r31 * vx) + (r32 * vy) + (r33 * vz)) >> 12);

		gteIR1 = limB1(gteMAC1, 0);
		gteIR2 = limB2(gteMAC2, 0);
		gteIR3 = limB3(gteMAC3, 0);

		gteSZ1 = limD(gteMAC3);
		quotient = limE(DIVIDE(h, gteSZ1));

		gteSX0 = limG1(F((s64)ofx + ((s64)gteIR1 * quotient)) >> 16);
		gteSY0 = limG2(F((s64)ofy + ((s64)gteIR2 * quotient)) >> 16);
	}

	/* ========== VERTEX 1 ========== */
	{
		const s32 vx = gteVX1, vy = gteVY1, vz = gteVZ1;

		gteMAC1 = A1((((s64)trx << 12) + (r11 * vx) + (r12 * vy) + (r13 * vz)) >> 12);
		gteMAC2 = A2((((s64)try_ << 12) + (r21 * vx) + (r22 * vy) + (r23 * vz)) >> 12);
		gteMAC3 = A3((((s64)trz << 12) + (r31 * vx) + (r32 * vy) + (r33 * vz)) >> 12);

		gteIR1 = limB1(gteMAC1, 0);
		gteIR2 = limB2(gteMAC2, 0);
		gteIR3 = limB3(gteMAC3, 0);

		gteSZ2 = limD(gteMAC3);
		quotient = limE(DIVIDE(h, gteSZ2));

		gteSX1 = limG1(F((s64)ofx + ((s64)gteIR1 * quotient)) >> 16);
		gteSY1 = limG2(F((s64)ofy + ((s64)gteIR2 * quotient)) >> 16);
	}

	/* ========== VERTEX 2 ========== */
	{
		const s32 vx = gteVX2, vy = gteVY2, vz = gteVZ2;

		gteMAC1 = A1((((s64)trx << 12) + (r11 * vx) + (r12 * vy) + (r13 * vz)) >> 12);
		gteMAC2 = A2((((s64)try_ << 12) + (r21 * vx) + (r22 * vy) + (r23 * vz)) >> 12);
		gteMAC3 = A3((((s64)trz << 12) + (r31 * vx) + (r32 * vy) + (r33 * vz)) >> 12);

		gteIR1 = limB1(gteMAC1, 0);
		gteIR2 = limB2(gteMAC2, 0);
		gteIR3 = limB3(gteMAC3, 0);

		gteSZ3 = limD(gteMAC3);
		quotient = limE(DIVIDE(h, gteSZ3));

		gteSX2 = limG1(F((s64)ofx + ((s64)gteIR1 * quotient)) >> 16);
		gteSY2 = limG2(F((s64)ofy + ((s64)gteIR2 * quotient)) >> 16);
	}

	/* Depth cueing (only needs last quotient) */
	const s64 tmp = (s64)gteDQB + ((s64)gteDQA * quotient);
	gteMAC0 = F(tmp);
	gteIR0 = limH(tmp >> 12);
}
#endif /* GTE_USE_FAST_RTPT */

/* Original loop-based RTPT (fallback) */
static void gteRTPT_loop(void) {
	int quotient;
	int v;
	s32 vx, vy, vz;

	gteFLAG = 0;

	gteSZ0 = gteSZ3;
	for (v = 0; v < 3; v++) {
		vx = VX(v);
		vy = VY(v);
		vz = VZ(v);
		gteMAC1 = A1((((s64)gteTRX << 12) + (gteR11 * vx) + (gteR12 * vy) + (gteR13 * vz)) >> 12);
		gteMAC2 = A2((((s64)gteTRY << 12) + (gteR21 * vx) + (gteR22 * vy) + (gteR23 * vz)) >> 12);
		gteMAC3 = A3((((s64)gteTRZ << 12) + (gteR31 * vx) + (gteR32 * vy) + (gteR33 * vz)) >> 12);
		gteIR1 = limB1(gteMAC1, 0);
		gteIR2 = limB2(gteMAC2, 0);
		gteIR3 = limB3(gteMAC3, 0);
		fSZ(v) = limD(gteMAC3);
		quotient = limE(DIVIDE(gteH, fSZ(v)));
		fSX(v) = limG1(F((s64)gteOFX + ((s64)gteIR1 * quotient)) >> 16);
		fSY(v) = limG2(F((s64)gteOFY + ((s64)gteIR2 * quotient)) >> 16);
	}

	// See note in gteRTPS()
	s64 tmp = (s64)gteDQB + ((s64)gteDQA * quotient);
	gteMAC0 = F(tmp);
	gteIR0 = limH(tmp >> 12);
}

/* Public RTPT - dispatches to fast or loop version based on config */
void gteRTPT(void) {
#ifdef GTE_LOG
	GTE_LOG("GTE RTPT\n");
#endif

#ifdef GTE_USE_FAST_RTPT
	if (g_opt_rtpt_unroll) {
		gteRTPT_fast();
		return;
	}
#endif
	gteRTPT_loop();
}

// NOTE: 'gteop' parameter is instruction opcode shifted right 10 places.
void gteMVMVA(u32 gteop) {
	int shift = 12 * GTE_SF(gteop);
	int mx = GTE_MX(gteop);
	int v = GTE_V(gteop);
	int cv = GTE_CV(gteop);
	int lm = GTE_LM(gteop);
	s32 vx = VX(v);
	s32 vy = VY(v);
	s32 vz = VZ(v);

#ifdef GTE_LOG
	GTE_LOG("GTE MVMVA\n");
#endif
	gteFLAG = 0;

	gteMAC1 = A1((((s64)CV1(cv) << 12) + (MX11(mx) * vx) + (MX12(mx) * vy) + (MX13(mx) * vz)) >> shift);
	gteMAC2 = A2((((s64)CV2(cv) << 12) + (MX21(mx) * vx) + (MX22(mx) * vy) + (MX23(mx) * vz)) >> shift);
	gteMAC3 = A3((((s64)CV3(cv) << 12) + (MX31(mx) * vx) + (MX32(mx) * vy) + (MX33(mx) * vz)) >> shift);

	gteIR1 = limB1(gteMAC1, lm);
	gteIR2 = limB2(gteMAC2, lm);
	gteIR3 = limB3(gteMAC3, lm);
}

void gteNCLIP(void) {
#ifdef GTE_LOG
	GTE_LOG("GTE NCLIP\n");
#endif
	gteFLAG = 0;

	gteMAC0 = F((s64)gteSX0 * (gteSY1 - gteSY2) +
				gteSX1 * (gteSY2 - gteSY0) +
				gteSX2 * (gteSY0 - gteSY1));
}

void gteAVSZ3(void) {
#ifdef GTE_LOG
	GTE_LOG("GTE AVSZ3\n");
#endif
	gteFLAG = 0;

	gteMAC0 = F((s64)gteZSF3 * (gteSZ1 + gteSZ2 + gteSZ3));
	gteOTZ = limD(gteMAC0 >> 12);
}

void gteAVSZ4(void) {
#ifdef GTE_LOG
	GTE_LOG("GTE AVSZ4\n");
#endif
	gteFLAG = 0;

	gteMAC0 = F((s64)gteZSF4 * (gteSZ0 + gteSZ1 + gteSZ2 + gteSZ3));
	gteOTZ = limD(gteMAC0 >> 12);
}

// NOTE: 'gteop' parameter is instruction opcode shifted right 10 places.
void gteSQR(u32 gteop) {
	int shift = 12 * GTE_SF(gteop);
	int lm = GTE_LM(gteop);

#ifdef GTE_LOG
	GTE_LOG("GTE SQR\n");
#endif
	gteFLAG = 0;

	gteMAC1 = (gteIR1 * gteIR1) >> shift;
	gteMAC2 = (gteIR2 * gteIR2) >> shift;
	gteMAC3 = (gteIR3 * gteIR3) >> shift;
	gteIR1 = limB1(gteMAC1, lm);
	gteIR2 = limB2(gteMAC2, lm);
	gteIR3 = limB3(gteMAC3, lm);
}

void gteNCCS(void) {
#ifdef GTE_LOG
	GTE_LOG("GTE NCCS\n");
#endif
	gteFLAG = 0;

	gteMAC1 = ((s64)(gteL11 * gteVX0) + (gteL12 * gteVY0) + (gteL13 * gteVZ0)) >> 12;
	gteMAC2 = ((s64)(gteL21 * gteVX0) + (gteL22 * gteVY0) + (gteL23 * gteVZ0)) >> 12;
	gteMAC3 = ((s64)(gteL31 * gteVX0) + (gteL32 * gteVY0) + (gteL33 * gteVZ0)) >> 12;
	gteIR1 = limB1(gteMAC1, 1);
	gteIR2 = limB2(gteMAC2, 1);
	gteIR3 = limB3(gteMAC3, 1);
	gteMAC1 = A1((((s64)gteRBK << 12) + (gteLR1 * gteIR1) + (gteLR2 * gteIR2) + (gteLR3 * gteIR3)) >> 12);
	gteMAC2 = A2((((s64)gteGBK << 12) + (gteLG1 * gteIR1) + (gteLG2 * gteIR2) + (gteLG3 * gteIR3)) >> 12);
	gteMAC3 = A3((((s64)gteBBK << 12) + (gteLB1 * gteIR1) + (gteLB2 * gteIR2) + (gteLB3 * gteIR3)) >> 12);
	gteIR1 = limB1(gteMAC1, 1);
	gteIR2 = limB2(gteMAC2, 1);
	gteIR3 = limB3(gteMAC3, 1);
	gteMAC1 = ((s32)gteR * gteIR1) >> 8;
	gteMAC2 = ((s32)gteG * gteIR2) >> 8;
	gteMAC3 = ((s32)gteB * gteIR3) >> 8;
	gteIR1 = gteMAC1;
	gteIR2 = gteMAC2;
	gteIR3 = gteMAC3;

	gteRGB0 = gteRGB1;
	gteRGB1 = gteRGB2;
	gteCODE2 = gteCODE;
	gteR2 = limC1(gteMAC1 >> 4);
	gteG2 = limC2(gteMAC2 >> 4);
	gteB2 = limC3(gteMAC3 >> 4);
}

void gteNCCT(void) {
	int v;
	s32 vx, vy, vz;

#ifdef GTE_LOG
	GTE_LOG("GTE NCCT\n");
#endif
	gteFLAG = 0;

	for (v = 0; v < 3; v++) {
		vx = VX(v);
		vy = VY(v);
		vz = VZ(v);
		gteMAC1 = ((s64)(gteL11 * vx) + (gteL12 * vy) + (gteL13 * vz)) >> 12;
		gteMAC2 = ((s64)(gteL21 * vx) + (gteL22 * vy) + (gteL23 * vz)) >> 12;
		gteMAC3 = ((s64)(gteL31 * vx) + (gteL32 * vy) + (gteL33 * vz)) >> 12;
		gteIR1 = limB1(gteMAC1, 1);
		gteIR2 = limB2(gteMAC2, 1);
		gteIR3 = limB3(gteMAC3, 1);
		gteMAC1 = A1((((s64)gteRBK << 12) + (gteLR1 * gteIR1) + (gteLR2 * gteIR2) + (gteLR3 * gteIR3)) >> 12);
		gteMAC2 = A2((((s64)gteGBK << 12) + (gteLG1 * gteIR1) + (gteLG2 * gteIR2) + (gteLG3 * gteIR3)) >> 12);
		gteMAC3 = A3((((s64)gteBBK << 12) + (gteLB1 * gteIR1) + (gteLB2 * gteIR2) + (gteLB3 * gteIR3)) >> 12);
		gteIR1 = limB1(gteMAC1, 1);
		gteIR2 = limB2(gteMAC2, 1);
		gteIR3 = limB3(gteMAC3, 1);
		gteMAC1 = ((s32)gteR * gteIR1) >> 8;
		gteMAC2 = ((s32)gteG * gteIR2) >> 8;
		gteMAC3 = ((s32)gteB * gteIR3) >> 8;

		gteRGB0 = gteRGB1;
		gteRGB1 = gteRGB2;
		gteCODE2 = gteCODE;
		gteR2 = limC1(gteMAC1 >> 4);
		gteG2 = limC2(gteMAC2 >> 4);
		gteB2 = limC3(gteMAC3 >> 4);
	}
	gteIR1 = gteMAC1;
	gteIR2 = gteMAC2;
	gteIR3 = gteMAC3;
}

void gteNCDS(void) {
#ifdef GTE_LOG
	GTE_LOG("GTE NCDS\n");
#endif
	gteFLAG = 0;

	gteMAC1 = ((s64)(gteL11 * gteVX0) + (gteL12 * gteVY0) + (gteL13 * gteVZ0)) >> 12;
	gteMAC2 = ((s64)(gteL21 * gteVX0) + (gteL22 * gteVY0) + (gteL23 * gteVZ0)) >> 12;
	gteMAC3 = ((s64)(gteL31 * gteVX0) + (gteL32 * gteVY0) + (gteL33 * gteVZ0)) >> 12;
	gteIR1 = limB1(gteMAC1, 1);
	gteIR2 = limB2(gteMAC2, 1);
	gteIR3 = limB3(gteMAC3, 1);
	gteMAC1 = A1((((s64)gteRBK << 12) + (gteLR1 * gteIR1) + (gteLR2 * gteIR2) + (gteLR3 * gteIR3)) >> 12);
	gteMAC2 = A2((((s64)gteGBK << 12) + (gteLG1 * gteIR1) + (gteLG2 * gteIR2) + (gteLG3 * gteIR3)) >> 12);
	gteMAC3 = A3((((s64)gteBBK << 12) + (gteLB1 * gteIR1) + (gteLB2 * gteIR2) + (gteLB3 * gteIR3)) >> 12);
	gteIR1 = limB1(gteMAC1, 1);
	gteIR2 = limB2(gteMAC2, 1);
	gteIR3 = limB3(gteMAC3, 1);
	gteMAC1 = (((gteR << 4) * gteIR1) + (gteIR0 * limB1(A1U((s64)gteRFC - ((gteR * gteIR1) >> 8)), 0))) >> 12;
	gteMAC2 = (((gteG << 4) * gteIR2) + (gteIR0 * limB2(A2U((s64)gteGFC - ((gteG * gteIR2) >> 8)), 0))) >> 12;
	gteMAC3 = (((gteB << 4) * gteIR3) + (gteIR0 * limB3(A3U((s64)gteBFC - ((gteB * gteIR3) >> 8)), 0))) >> 12;
	gteIR1 = limB1(gteMAC1, 1);
	gteIR2 = limB2(gteMAC2, 1);
	gteIR3 = limB3(gteMAC3, 1);

	gteRGB0 = gteRGB1;
	gteRGB1 = gteRGB2;
	gteCODE2 = gteCODE;
	gteR2 = limC1(gteMAC1 >> 4);
	gteG2 = limC2(gteMAC2 >> 4);
	gteB2 = limC3(gteMAC3 >> 4);
}

void gteNCDT(void) {
	int v;
	s32 vx, vy, vz;

#ifdef GTE_LOG
	GTE_LOG("GTE NCDT\n");
#endif
	gteFLAG = 0;

	for (v = 0; v < 3; v++) {
		vx = VX(v);
		vy = VY(v);
		vz = VZ(v);
		gteMAC1 = ((s64)(gteL11 * vx) + (gteL12 * vy) + (gteL13 * vz)) >> 12;
		gteMAC2 = ((s64)(gteL21 * vx) + (gteL22 * vy) + (gteL23 * vz)) >> 12;
		gteMAC3 = ((s64)(gteL31 * vx) + (gteL32 * vy) + (gteL33 * vz)) >> 12;
		gteIR1 = limB1(gteMAC1, 1);
		gteIR2 = limB2(gteMAC2, 1);
		gteIR3 = limB3(gteMAC3, 1);
		gteMAC1 = A1((((s64)gteRBK << 12) + (gteLR1 * gteIR1) + (gteLR2 * gteIR2) + (gteLR3 * gteIR3)) >> 12);
		gteMAC2 = A2((((s64)gteGBK << 12) + (gteLG1 * gteIR1) + (gteLG2 * gteIR2) + (gteLG3 * gteIR3)) >> 12);
		gteMAC3 = A3((((s64)gteBBK << 12) + (gteLB1 * gteIR1) + (gteLB2 * gteIR2) + (gteLB3 * gteIR3)) >> 12);
		gteIR1 = limB1(gteMAC1, 1);
		gteIR2 = limB2(gteMAC2, 1);
		gteIR3 = limB3(gteMAC3, 1);
		gteMAC1 = (((gteR << 4) * gteIR1) + (gteIR0 * limB1(A1U((s64)gteRFC - ((gteR * gteIR1) >> 8)), 0))) >> 12;
		gteMAC2 = (((gteG << 4) * gteIR2) + (gteIR0 * limB2(A2U((s64)gteGFC - ((gteG * gteIR2) >> 8)), 0))) >> 12;
		gteMAC3 = (((gteB << 4) * gteIR3) + (gteIR0 * limB3(A3U((s64)gteBFC - ((gteB * gteIR3) >> 8)), 0))) >> 12;

		gteRGB0 = gteRGB1;
		gteRGB1 = gteRGB2;
		gteCODE2 = gteCODE;
		gteR2 = limC1(gteMAC1 >> 4);
		gteG2 = limC2(gteMAC2 >> 4);
		gteB2 = limC3(gteMAC3 >> 4);
	}
	gteIR1 = limB1(gteMAC1, 1);
	gteIR2 = limB2(gteMAC2, 1);
	gteIR3 = limB3(gteMAC3, 1);
}

// NOTE: 'gteop' parameter is instruction opcode shifted right 10 places.
void gteOP(u32 gteop) {
	int shift = 12 * GTE_SF(gteop);
	int lm = GTE_LM(gteop);

#ifdef GTE_LOG
	GTE_LOG("GTE OP\n");
#endif
	gteFLAG = 0;

	gteMAC1 = ((gteR22 * gteIR3) - (gteR33 * gteIR2)) >> shift;
	gteMAC2 = ((gteR33 * gteIR1) - (gteR11 * gteIR3)) >> shift;
	gteMAC3 = ((gteR11 * gteIR2) - (gteR22 * gteIR1)) >> shift;
	gteIR1 = limB1(gteMAC1, lm);
	gteIR2 = limB2(gteMAC2, lm);
	gteIR3 = limB3(gteMAC3, lm);
}

// NOTE: 'gteop' parameter is instruction opcode shifted right 10 places.
void gteDCPL(u32 gteop) {
	int lm = GTE_LM(gteop);

	s32 RIR1 = ((s32)gteR * gteIR1) >> 8;
	s32 GIR2 = ((s32)gteG * gteIR2) >> 8;
	s32 BIR3 = ((s32)gteB * gteIR3) >> 8;

#ifdef GTE_LOG
	GTE_LOG("GTE DCPL\n");
#endif
	gteFLAG = 0;

	gteMAC1 = RIR1 + ((gteIR0 * limB1(A1U((s64)gteRFC - RIR1), 0)) >> 12);
	gteMAC2 = GIR2 + ((gteIR0 * limB1(A2U((s64)gteGFC - GIR2), 0)) >> 12);
	gteMAC3 = BIR3 + ((gteIR0 * limB1(A3U((s64)gteBFC - BIR3), 0)) >> 12);

	gteIR1 = limB1(gteMAC1, lm);
	gteIR2 = limB2(gteMAC2, lm);
	gteIR3 = limB3(gteMAC3, lm);

	gteRGB0 = gteRGB1;
	gteRGB1 = gteRGB2;
	gteCODE2 = gteCODE;
	gteR2 = limC1(gteMAC1 >> 4);
	gteG2 = limC2(gteMAC2 >> 4);
	gteB2 = limC3(gteMAC3 >> 4);
}

// NOTE: 'gteop' parameter is instruction opcode shifted right 10 places.
void gteGPF(u32 gteop) {
	int shift = 12 * GTE_SF(gteop);

#ifdef GTE_LOG
	GTE_LOG("GTE GPF\n");
#endif
	gteFLAG = 0;

	gteMAC1 = (gteIR0 * gteIR1) >> shift;
	gteMAC2 = (gteIR0 * gteIR2) >> shift;
	gteMAC3 = (gteIR0 * gteIR3) >> shift;
	gteIR1 = limB1(gteMAC1, 0);
	gteIR2 = limB2(gteMAC2, 0);
	gteIR3 = limB3(gteMAC3, 0);

	gteRGB0 = gteRGB1;
	gteRGB1 = gteRGB2;
	gteCODE2 = gteCODE;
	gteR2 = limC1(gteMAC1 >> 4);
	gteG2 = limC2(gteMAC2 >> 4);
	gteB2 = limC3(gteMAC3 >> 4);
}

// NOTE: 'gteop' parameter is instruction opcode shifted right 10 places.
void gteGPL(u32 gteop) {
	int shift = 12 * GTE_SF(gteop);

#ifdef GTE_LOG
	GTE_LOG("GTE GPL\n");
#endif
	gteFLAG = 0;

	gteMAC1 = A1((((s64)gteMAC1 << shift) + (gteIR0 * gteIR1)) >> shift);
	gteMAC2 = A2((((s64)gteMAC2 << shift) + (gteIR0 * gteIR2)) >> shift);
	gteMAC3 = A3((((s64)gteMAC3 << shift) + (gteIR0 * gteIR3)) >> shift);
	gteIR1 = limB1(gteMAC1, 0);
	gteIR2 = limB2(gteMAC2, 0);
	gteIR3 = limB3(gteMAC3, 0);

	gteRGB0 = gteRGB1;
	gteRGB1 = gteRGB2;
	gteCODE2 = gteCODE;
	gteR2 = limC1(gteMAC1 >> 4);
	gteG2 = limC2(gteMAC2 >> 4);
	gteB2 = limC3(gteMAC3 >> 4);
}

// NOTE: 'gteop' parameter is instruction opcode shifted right 10 places.
void gteDPCS(u32 gteop) {
	int shift = 12 * GTE_SF(gteop);

#ifdef GTE_LOG
	GTE_LOG("GTE DPCS\n");
#endif
	gteFLAG = 0;

	gteMAC1 = ((gteR << 16) + (gteIR0 * limB1(A1U(((s64)gteRFC - (gteR << 4)) << (12 - shift)), 0))) >> 12;
	gteMAC2 = ((gteG << 16) + (gteIR0 * limB2(A2U(((s64)gteGFC - (gteG << 4)) << (12 - shift)), 0))) >> 12;
	gteMAC3 = ((gteB << 16) + (gteIR0 * limB3(A3U(((s64)gteBFC - (gteB << 4)) << (12 - shift)), 0))) >> 12;

	gteIR1 = limB1(gteMAC1, 0);
	gteIR2 = limB2(gteMAC2, 0);
	gteIR3 = limB3(gteMAC3, 0);
	gteRGB0 = gteRGB1;
	gteRGB1 = gteRGB2;
	gteCODE2 = gteCODE;
	gteR2 = limC1(gteMAC1 >> 4);
	gteG2 = limC2(gteMAC2 >> 4);
	gteB2 = limC3(gteMAC3 >> 4);
}

void gteDPCT(void) {
	int v;

#ifdef GTE_LOG
	GTE_LOG("GTE DPCT\n");
#endif
	gteFLAG = 0;

	for (v = 0; v < 3; v++) {
		gteMAC1 = ((gteR0 << 16) + (gteIR0 * limB1(A1U((s64)gteRFC - (gteR0 << 4)), 0))) >> 12;
		gteMAC2 = ((gteG0 << 16) + (gteIR0 * limB1(A2U((s64)gteGFC - (gteG0 << 4)), 0))) >> 12;
		gteMAC3 = ((gteB0 << 16) + (gteIR0 * limB1(A3U((s64)gteBFC - (gteB0 << 4)), 0))) >> 12;

		gteRGB0 = gteRGB1;
		gteRGB1 = gteRGB2;
		gteCODE2 = gteCODE;
		gteR2 = limC1(gteMAC1 >> 4);
		gteG2 = limC2(gteMAC2 >> 4);
		gteB2 = limC3(gteMAC3 >> 4);
	}
	gteIR1 = limB1(gteMAC1, 0);
	gteIR2 = limB2(gteMAC2, 0);
	gteIR3 = limB3(gteMAC3, 0);
}

void gteNCS(void) {
#ifdef GTE_LOG
	GTE_LOG("GTE NCS\n");
#endif
	gteFLAG = 0;

	gteMAC1 = ((s64)(gteL11 * gteVX0) + (gteL12 * gteVY0) + (gteL13 * gteVZ0)) >> 12;
	gteMAC2 = ((s64)(gteL21 * gteVX0) + (gteL22 * gteVY0) + (gteL23 * gteVZ0)) >> 12;
	gteMAC3 = ((s64)(gteL31 * gteVX0) + (gteL32 * gteVY0) + (gteL33 * gteVZ0)) >> 12;
	gteIR1 = limB1(gteMAC1, 1);
	gteIR2 = limB2(gteMAC2, 1);
	gteIR3 = limB3(gteMAC3, 1);
	gteMAC1 = A1((((s64)gteRBK << 12) + (gteLR1 * gteIR1) + (gteLR2 * gteIR2) + (gteLR3 * gteIR3)) >> 12);
	gteMAC2 = A2((((s64)gteGBK << 12) + (gteLG1 * gteIR1) + (gteLG2 * gteIR2) + (gteLG3 * gteIR3)) >> 12);
	gteMAC3 = A3((((s64)gteBBK << 12) + (gteLB1 * gteIR1) + (gteLB2 * gteIR2) + (gteLB3 * gteIR3)) >> 12);
	gteIR1 = limB1(gteMAC1, 1);
	gteIR2 = limB2(gteMAC2, 1);
	gteIR3 = limB3(gteMAC3, 1);

	gteRGB0 = gteRGB1;
	gteRGB1 = gteRGB2;
	gteCODE2 = gteCODE;
	gteR2 = limC1(gteMAC1 >> 4);
	gteG2 = limC2(gteMAC2 >> 4);
	gteB2 = limC3(gteMAC3 >> 4);
}

void gteNCT(void) {
	int v;
	s32 vx, vy, vz;

#ifdef GTE_LOG
	GTE_LOG("GTE NCT\n");
#endif
	gteFLAG = 0;

	for (v = 0; v < 3; v++) {
		vx = VX(v);
		vy = VY(v);
		vz = VZ(v);
		gteMAC1 = ((s64)(gteL11 * vx) + (gteL12 * vy) + (gteL13 * vz)) >> 12;
		gteMAC2 = ((s64)(gteL21 * vx) + (gteL22 * vy) + (gteL23 * vz)) >> 12;
		gteMAC3 = ((s64)(gteL31 * vx) + (gteL32 * vy) + (gteL33 * vz)) >> 12;
		gteIR1 = limB1(gteMAC1, 1);
		gteIR2 = limB2(gteMAC2, 1);
		gteIR3 = limB3(gteMAC3, 1);
		gteMAC1 = A1((((s64)gteRBK << 12) + (gteLR1 * gteIR1) + (gteLR2 * gteIR2) + (gteLR3 * gteIR3)) >> 12);
		gteMAC2 = A2((((s64)gteGBK << 12) + (gteLG1 * gteIR1) + (gteLG2 * gteIR2) + (gteLG3 * gteIR3)) >> 12);
		gteMAC3 = A3((((s64)gteBBK << 12) + (gteLB1 * gteIR1) + (gteLB2 * gteIR2) + (gteLB3 * gteIR3)) >> 12);
		gteRGB0 = gteRGB1;
		gteRGB1 = gteRGB2;
		gteCODE2 = gteCODE;
		gteR2 = limC1(gteMAC1 >> 4);
		gteG2 = limC2(gteMAC2 >> 4);
		gteB2 = limC3(gteMAC3 >> 4);
	}
	gteIR1 = limB1(gteMAC1, 1);
	gteIR2 = limB2(gteMAC2, 1);
	gteIR3 = limB3(gteMAC3, 1);
}

void gteCC(void) {
#ifdef GTE_LOG
	GTE_LOG("GTE CC\n");
#endif
	gteFLAG = 0;

	gteMAC1 = A1((((s64)gteRBK << 12) + (gteLR1 * gteIR1) + (gteLR2 * gteIR2) + (gteLR3 * gteIR3)) >> 12);
	gteMAC2 = A2((((s64)gteGBK << 12) + (gteLG1 * gteIR1) + (gteLG2 * gteIR2) + (gteLG3 * gteIR3)) >> 12);
	gteMAC3 = A3((((s64)gteBBK << 12) + (gteLB1 * gteIR1) + (gteLB2 * gteIR2) + (gteLB3 * gteIR3)) >> 12);
	gteIR1 = limB1(gteMAC1, 1);
	gteIR2 = limB2(gteMAC2, 1);
	gteIR3 = limB3(gteMAC3, 1);
	gteMAC1 = ((s32)gteR * gteIR1) >> 8;
	gteMAC2 = ((s32)gteG * gteIR2) >> 8;
	gteMAC3 = ((s32)gteB * gteIR3) >> 8;
	gteIR1 = limB1(gteMAC1, 1);
	gteIR2 = limB2(gteMAC2, 1);
	gteIR3 = limB3(gteMAC3, 1);

	gteRGB0 = gteRGB1;
	gteRGB1 = gteRGB2;
	gteCODE2 = gteCODE;
	gteR2 = limC1(gteMAC1 >> 4);
	gteG2 = limC2(gteMAC2 >> 4);
	gteB2 = limC3(gteMAC3 >> 4);
}

// NOTE: 'gteop' parameter is instruction opcode shifted right 10 places.
void gteINTPL(u32 gteop) {
	int shift = 12 * GTE_SF(gteop);
	int lm = GTE_LM(gteop);

#ifdef GTE_LOG
	GTE_LOG("GTE INTPL\n");
#endif
	gteFLAG = 0;

	gteMAC1 = ((gteIR1 << 12) + (gteIR0 * limB1(A1U((s64)gteRFC - gteIR1), 0))) >> shift;
	gteMAC2 = ((gteIR2 << 12) + (gteIR0 * limB2(A2U((s64)gteGFC - gteIR2), 0))) >> shift;
	gteMAC3 = ((gteIR3 << 12) + (gteIR0 * limB3(A3U((s64)gteBFC - gteIR3), 0))) >> shift;
	gteIR1 = limB1(gteMAC1, lm);
	gteIR2 = limB2(gteMAC2, lm);
	gteIR3 = limB3(gteMAC3, lm);
	gteRGB0 = gteRGB1;
	gteRGB1 = gteRGB2;
	gteCODE2 = gteCODE;
	gteR2 = limC1(gteMAC1 >> 4);
	gteG2 = limC2(gteMAC2 >> 4);
	gteB2 = limC3(gteMAC3 >> 4);
}

void gteCDP(void) {
#ifdef GTE_LOG
	GTE_LOG("GTE CDP\n");
#endif
	gteFLAG = 0;

	gteMAC1 = A1((((s64)gteRBK << 12) + (gteLR1 * gteIR1) + (gteLR2 * gteIR2) + (gteLR3 * gteIR3)) >> 12);
	gteMAC2 = A2((((s64)gteGBK << 12) + (gteLG1 * gteIR1) + (gteLG2 * gteIR2) + (gteLG3 * gteIR3)) >> 12);
	gteMAC3 = A3((((s64)gteBBK << 12) + (gteLB1 * gteIR1) + (gteLB2 * gteIR2) + (gteLB3 * gteIR3)) >> 12);
	gteIR1 = limB1(gteMAC1, 1);
	gteIR2 = limB2(gteMAC2, 1);
	gteIR3 = limB3(gteMAC3, 1);
	gteMAC1 = (((gteR << 4) * gteIR1) + (gteIR0 * limB1(A1U((s64)gteRFC - ((gteR * gteIR1) >> 8)), 0))) >> 12;
	gteMAC2 = (((gteG << 4) * gteIR2) + (gteIR0 * limB2(A2U((s64)gteGFC - ((gteG * gteIR2) >> 8)), 0))) >> 12;
	gteMAC3 = (((gteB << 4) * gteIR3) + (gteIR0 * limB3(A3U((s64)gteBFC - ((gteB * gteIR3) >> 8)), 0))) >> 12;
	gteIR1 = limB1(gteMAC1, 1);
	gteIR2 = limB2(gteMAC2, 1);
	gteIR3 = limB3(gteMAC3, 1);

	gteRGB0 = gteRGB1;
	gteRGB1 = gteRGB2;
	gteCODE2 = gteCODE;
	gteR2 = limC1(gteMAC1 >> 4);
	gteG2 = limC2(gteMAC2 >> 4);
	gteB2 = limC3(gteMAC3 >> 4);
}
