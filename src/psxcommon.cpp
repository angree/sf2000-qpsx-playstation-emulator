/***************************************************************************
 *   Copyright (C) 2007 Ryan Schultz, PCSX-df Team, PCSX team              *
 *   schultz.ryan@gmail.com, http://rschultz.ath.cx/code.php               *
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
 *   51 Franklin Steet, Fifth Floor, Boston, MA 02111-1307 USA.            *
 ***************************************************************************/

#include "psxcommon.h"
#include "plugin_lib/plugin_lib.h"

/* QPSX_033: Frame complete flag - signals Execute() to return after VBlank */
volatile int emu_frame_complete = 0;

/* QPSX_033: Debug counter for EmuUpdate calls */
static int emu_update_count = 0;

/* SF2000 xlog for debugging */
#ifdef SF2000
extern "C" {
    extern void xlog(const char *fmt, ...);
}
#define EMU_LOG(fmt, ...) xlog("EMU: " fmt "\n", ##__VA_ARGS__)
#else
#define EMU_LOG(fmt, ...) do {} while(0)
#endif

void EmuUpdate()
{
	pl_frame_limit();

	// Update controls
	// NOTE: This is point of control transfer to frontend menu..
	//  Only allow re-entry to frontend when PS1 cache status is normal.
	//  We don't want to allow creation of savestates when cache is isolated.
	//  See cache control port comments in psxmem.cpp psxMemWrite32().
	if (psxRegs.writeok) {
		pad_update();
	}

	/* QPSX_033: Signal frame complete - Execute() will check this and return */
	emu_frame_complete = 1;

	/* Log first few VBlanks to confirm EmuUpdate is called */
	if (emu_update_count < 5) {
		EMU_LOG("EmuUpdate() called #%d - set emu_frame_complete=1", emu_update_count);
		emu_update_count++;
	}
}
