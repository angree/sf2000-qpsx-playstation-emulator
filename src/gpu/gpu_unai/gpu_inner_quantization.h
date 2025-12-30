/***************************************************************************
*   Copyright (C) 2016 PCSX4ALL Team                                      *
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU General Public License as published by  *
*   the Free Software Foundation; either version 2 of the License, or     *
*   (at your option) any later version.                                   *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU General Public License for more details.                          *
*                                                                         *
*   You should have received a copy of the GNU General Public License     *
*   along with this program; if not, write to the                         *
*   Free Software Foundation, Inc.,                                       *
*   51 Franklin Street, Fifth Floor, Boston, MA 02111-1307 USA.           *
***************************************************************************/

#ifndef _OP_DITHER_H_
#define _OP_DITHER_H_

// SF2000: Precomputed dither matrix to avoid runtime division by 65
// Each value is: component | (component << 10) | (component << 20)
// where component = (((DitherMatrix[i] + 1) << 4) / 65) & ~1
static const u32 PrecomputedDitherMatrix[64] = {
	0x00000000, 0x00200808, 0x00080202, 0x00280a0a, 0x00000000, 0x00200808, 0x00080202, 0x00280a0a,
	0x00300c0c, 0x00100404, 0x00380e0e, 0x00180606, 0x00300c0c, 0x00100404, 0x00380e0e, 0x00180606,
	0x00080202, 0x00280a0a, 0x00000000, 0x00200808, 0x00080202, 0x00280a0a, 0x00000000, 0x00200808,
	0x00380e0e, 0x00180606, 0x00300c0c, 0x00100404, 0x00380e0e, 0x00180606, 0x00300c0c, 0x00100404,
	0x00000000, 0x00200808, 0x00080202, 0x00280a0a, 0x00000000, 0x00200808, 0x00080202, 0x00280a0a,
	0x00300c0c, 0x00100404, 0x00380e0e, 0x00180606, 0x00300c0c, 0x00100404, 0x00380e0e, 0x00180606,
	0x00080202, 0x00280a0a, 0x00000000, 0x00200808, 0x00080202, 0x00280a0a, 0x00000000, 0x00200808,
	0x00380e0e, 0x00180606, 0x00300c0c, 0x00100404, 0x00380e0e, 0x00180606, 0x00300c0c, 0x00100404
};

static void SetupDitheringConstants()
{
	// SF2000: Use precomputed table instead of runtime division
	// This avoids potential FPU instruction generation from /65
	for (int i = 0; i < 64; i++)
	{
		gpu_unai.DitherMatrix[i] = PrecomputedDitherMatrix[i];
	}
}

////////////////////////////////////////////////////////////////////////////////
// Convert padded u32 5.4:5.4:5.4 bgr fixed-pt triplet to final bgr555 color,
//  applying dithering if specified by template parameter.
//
// INPUT:
//     'uSrc24' input: 000bbbbbXXXX0gggggXXXX0rrrrrXXXX
//                     ^ bit 31
//       'pDst' is a pointer to destination framebuffer pixel, used
//         to determine which DitherMatrix[] entry to apply.
// RETURNS:
//         u16 output: 0bbbbbgggggrrrrr
//                     ^ bit 16
// Where 'X' are fixed-pt bits, '0' is zero-padding, and '-' is don't care
////////////////////////////////////////////////////////////////////////////////
template <int DITHER>
GPU_INLINE u16 gpuColorQuantization24(u32 uSrc24, const u16 *pDst)
{
	if (DITHER)
	{
		u16 fbpos  = (u32)(pDst - gpu_unai.vram);
		u16 offset = ((fbpos & (0x7 << 10)) >> 7) | (fbpos & 0x7);

		//clean overflow flags and add
		uSrc24 = (uSrc24 & 0x1FF7FDFF) + gpu_unai.DitherMatrix[offset];

		if (uSrc24 & (1<< 9)) uSrc24 |= (0x1FF    );
		if (uSrc24 & (1<<19)) uSrc24 |= (0x1FF<<10);
		if (uSrc24 & (1<<29)) uSrc24 |= (0x1FF<<20);
	}

	return ((uSrc24>> 4) & (0x1F    ))
	     | ((uSrc24>> 9) & (0x1F<<5 ))
	     | ((uSrc24>>14) & (0x1F<<10));
}

#endif //_OP_DITHER_H_
