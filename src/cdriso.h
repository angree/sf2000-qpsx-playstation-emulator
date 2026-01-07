/***************************************************************************
 *   Copyright (C) 2007 PCSX-df Team                                       *
 *   Copyright (C) 2009 Wei Mingzhi                                        *
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

#ifndef CDRISO_H
#define CDRISO_H

/* Define CALLBACK if not already defined (for standalone headers) */
#ifndef CALLBACK
#define CALLBACK
#endif

void cdrIsoInit(void);
int cdrIsoActive(void);

// Callback func ptr allows frontend GUI to choose CD to load
extern void (CALLBACK *cdrIsoMultidiskCallback)(void);
extern unsigned int cdrIsoMultidiskCount;
extern unsigned int cdrIsoMultidiskSelect;

/*
 * v258: CDDA Conversion API
 * Convert/delete audio track versions on-device
 */

/* CDDA format constants (must match cdriso.cpp) */
#define CDDA_FMT_BIN      0
#define CDDA_FMT_BINWAV   1
#define CDDA_FMT_BINADPCM 2

/* Track info structure */
typedef struct {
	int track_num;
	int has_bin;
	int has_binwav;
	int has_binadpcm;
	int current_format;
	unsigned int bin_sectors;
	char bin_path[512];
} cdda_track_info_t;

/* Progress callback type */
typedef void (*cdda_progress_cb_t)(int track_idx, int total_tracks,
                                   int sector, int total_sectors,
                                   int total_pct);

/* Functions */
#ifdef __cplusplus
extern "C" {
#endif

int cdda_scan_tracks(void);
cdda_track_info_t* cdda_get_track_info(int idx);
int cdda_get_num_tracks(void);
void cdda_set_progress_callback(cdda_progress_cb_t cb);
int cdda_convert_to_binwav(int track_idx);
int cdda_convert_to_binadpcm(int track_idx);
int cdda_delete_version(int track_idx, int format);

/* v261: Incremental conversion API */
int cdda_start_convert_track(int track_idx, int format);
int cdda_convert_step(void);
void cdda_get_convert_progress(int *sector, int *total_sectors, int *track_idx);
void cdda_request_cancel(void);
int cdda_is_converting(void);

#ifdef __cplusplus
}
#endif

#endif
