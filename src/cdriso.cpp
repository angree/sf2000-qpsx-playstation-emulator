/***************************************************************************
 *   Copyright (C) 2007 PCSX-df Team                                       *
 *   Copyright (C) 2009 Wei Mingzhi                                        *
 *   Copyright (C) 2012 notaz                                              *
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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

// NOTE: Code here adapted from newer PCSX Rearmed/Reloaded code

#include "psxcommon.h"
#include "plugins.h"
#include "cdrom.h"
#include "cdriso.h"
#include "ppf.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <process.h>
#include <windows.h>

#define strcasecmp _stricmp
#define fseeko fseek
#define ftello ftell
#else // UNIX:
#if !defined(NO_THREADS) && !defined(SF2000)
#include <pthread.h>
#endif
#endif

#include <sys/time.h>
#ifndef SF2000
#include <unistd.h>
#endif
#include <errno.h>
#ifndef NO_ZLIB
#include <zlib.h>
#endif

#define OFF_T_MSB ((off_t)1 << (sizeof(off_t) * 8 - 1))

/* v257: Compressed CDDA support */
extern int g_opt_cdda_binsav;  /* Enable compressed CDDA (.binadpcm / .binwav) */

/*
 * v257: Compressed CDDA support
 *
 * Three formats supported (in priority order):
 *
 * 1. .binadpcm - IMA ADPCM 22kHz mono (16x smaller than original)
 *    - 152 bytes/sector = 4-byte header + 147 bytes ADPCM data + 1 pad
 *    - Header: predictor (int16), step_index (uint8), reserved (uint8)
 *    - 294 samples per sector (4 bits each)
 *
 * 2. .binwav - Raw PCM 22kHz mono (4x smaller than original)
 *    - 588 bytes/sector = 294 mono samples at 22kHz (16-bit)
 *
 * 3. .bin - Original 44.1kHz stereo
 *    - 2352 bytes/sector = 588 stereo sample pairs
 *
 * When reading, we upsample back to 44.1kHz stereo for the emulator.
 */

/* CDDA track format enum */
#define CDDA_FMT_BIN      0   /* Original .bin (44.1kHz stereo, 2352 bytes/sector) */
#define CDDA_FMT_BINWAV   1   /* .binwav (22kHz mono PCM, 588 bytes/sector) */
#define CDDA_FMT_BINADPCM 2   /* .binadpcm (22kHz mono IMA ADPCM, 152 bytes/sector) */

/* Sector sizes for each format */
#define BINWAV_SECTOR_SIZE   588   /* 294 samples * 2 bytes */
#define BINADPCM_SECTOR_SIZE 152   /* 4 header + 147 data + 1 pad */
#define BINADPCM_HEADER_SIZE 4
#define BINADPCM_DATA_SIZE   147   /* 294 nibbles = 147 bytes */

/*
 * IMA ADPCM tables
 */
static const int ima_step_table[89] = {
	7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
	19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
	50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
	130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
	337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
	876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
	2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
	5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
	15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

static const int ima_index_table[16] = {
	-1, -1, -1, -1, 2, 4, 6, 8,
	-1, -1, -1, -1, 2, 4, 6, 8
};

/* Decode one IMA ADPCM nibble */
static inline int16_t ima_decode_sample(int nibble, int16_t *predictor, int *step_idx) {
	int step = ima_step_table[*step_idx];
	int diff = step >> 3;
	if (nibble & 1) diff += step >> 2;
	if (nibble & 2) diff += step >> 1;
	if (nibble & 4) diff += step;
	if (nibble & 8) diff = -diff;

	int sample = *predictor + diff;
	if (sample > 32767) sample = 32767;
	if (sample < -32768) sample = -32768;
	*predictor = (int16_t)sample;

	*step_idx += ima_index_table[nibble];
	if (*step_idx < 0) *step_idx = 0;
	if (*step_idx > 88) *step_idx = 88;

	return (int16_t)sample;
}

/*
 * v258: IMA ADPCM encoder - encode one sample to nibble
 * Updates predictor and step_idx state
 */
static inline int ima_encode_sample(int16_t sample, int16_t *predictor, int *step_idx) {
	int step = ima_step_table[*step_idx];
	int diff = sample - *predictor;
	int nibble = 0;

	if (diff < 0) {
		nibble = 8;
		diff = -diff;
	}

	if (diff >= step) {
		nibble |= 4;
		diff -= step;
	}
	if (diff >= (step >> 1)) {
		nibble |= 2;
		diff -= (step >> 1);
	}
	if (diff >= (step >> 2)) {
		nibble |= 1;
	}

	/* Decode to update predictor (must match decoder exactly) */
	int decode_diff = step >> 3;
	if (nibble & 1) decode_diff += step >> 2;
	if (nibble & 2) decode_diff += step >> 1;
	if (nibble & 4) decode_diff += step;
	if (nibble & 8) decode_diff = -decode_diff;

	int new_pred = *predictor + decode_diff;
	if (new_pred > 32767) new_pred = 32767;
	if (new_pred < -32768) new_pred = -32768;
	*predictor = (int16_t)new_pred;

	*step_idx += ima_index_table[nibble];
	if (*step_idx < 0) *step_idx = 0;
	if (*step_idx > 88) *step_idx = 88;

	return nibble;
}

static FILE *cdHandle = NULL;
static FILE *cddaHandle = NULL;
static FILE *subHandle = NULL;

static boolean subChanMixed = FALSE;
static boolean subChanRaw = FALSE;
static boolean subChanMissing = FALSE;

static boolean multifile = FALSE;

static unsigned char cdbuffer[CD_FRAMESIZE_RAW];
static unsigned char subbuffer[SUB_FRAMESIZE];

static unsigned char sndbuffer[CD_FRAMESIZE_RAW * 10];

unsigned char *(*CDR_getBuffer)(void);

#define CDDA_FRAMETIME			(1000 * (sizeof(sndbuffer) / CD_FRAMESIZE_RAW) / 75)

#if defined(NO_THREADS) || defined(SF2000)
// No CDDA thread support on SF2000 bare metal
#elif defined(_WIN32)
static HANDLE threadid;
#else
static pthread_t threadid;
#endif
static unsigned int initial_offset = 0;
static boolean playing = FALSE;
static boolean cddaBigEndian = FALSE;

// cdda sectors in toc, byte offset in file
static unsigned int cdda_cur_sector;
static unsigned int cdda_first_sector;
static unsigned int cdda_file_offset;
/* Frame offset into CD image where pregap data would be found if it was there.
 * If a game seeks there we must *not* return subchannel data since it's
 * not in the CD image, so that cdrom code can fake subchannel data instead.
 * XXX: there could be multiple pregaps but PSX dumps only have one? */
static unsigned int pregapOffset;

#define cddaCurPos cdda_cur_sector

#ifndef NO_ZLIB
// compressed image stuff
typedef struct {
	unsigned char buff_raw[16][CD_FRAMESIZE_RAW];
	unsigned char buff_compressed[CD_FRAMESIZE_RAW * 16 + 100];
	off_t *index_table;
	unsigned int index_len;
	unsigned int block_shift;
	unsigned int current_block;
	unsigned int sector_in_blk;
} COMPR_IMG;

static COMPR_IMG *compr_img;
#endif

int (*cdimg_read_func)(FILE *f, unsigned int base, void *dest, int sector);

char* CDR__getDriveLetter(void);
long CDR__configure(void);
long CDR__test(void);
void CDR__about(void);
long CDR__setfilename(char *filename);
long CDR__getStatus(struct CdrStat *stat);

static void DecodeRawSubData(void);

typedef enum {
	DATA = 1,
	CDDA
} cd_type;

struct trackinfo {
	cd_type type;
	char start[3];		// MSF-format
	char length[3];		// MSF-format
	FILE *handle;		// for multi-track images CDDA
	unsigned int start_offset; // byte offset from start of above file (in original 2352-byte units)
	int cdda_format;	// v257: CDDA_FMT_BIN, CDDA_FMT_BINWAV, or CDDA_FMT_BINADPCM
	char filepath[MAXPATHLEN]; // v257: path to this track's file (for compressed format lookup)
};

#define MAXTRACKS 100 /* How many tracks can a CD hold? */

static int numtracks = 0;
static struct trackinfo ti[MAXTRACKS];

/*
 * v257: Try to upgrade CDDA tracks to compressed formats
 * Priority: .binadpcm (16x smaller) -> .binwav (4x smaller) -> .bin (original)
 */
static void upgrade_cdda_formats(void) {
	int i;
	static char alt_path[MAXPATHLEN];

	if (!g_opt_cdda_binsav) {
		return;
	}

	for (i = 1; i <= numtracks; i++) {
		/* Only process AUDIO tracks with valid handles and paths */
		if (ti[i].type != CDDA || ti[i].handle == NULL || ti[i].filepath[0] == '\0') {
			continue;
		}

		/* Find extension */
		strncpy(alt_path, ti[i].filepath, MAXPATHLEN - 12);
		alt_path[MAXPATHLEN - 12] = '\0';
		char* ext = strrchr(alt_path, '.');
		if (!ext || strcasecmp(ext, ".bin") != 0) {
			continue;
		}

		/* Try .binadpcm first (16x smaller, IMA ADPCM) */
		strcpy(ext, ".binadpcm");
		FILE* f = fopen(alt_path, "rb");
		if (f) {
			fclose(ti[i].handle);
			ti[i].handle = f;
			ti[i].cdda_format = CDDA_FMT_BINADPCM;
			printf("CDDA Track %d: Using .binadpcm (IMA ADPCM 22kHz)\n", i);
			continue;
		}

		/* Try .binwav second (4x smaller, raw PCM) */
		strcpy(ext, ".binwav");
		f = fopen(alt_path, "rb");
		if (f) {
			fclose(ti[i].handle);
			ti[i].handle = f;
			ti[i].cdda_format = CDDA_FMT_BINWAV;
			printf("CDDA Track %d: Using .binwav (PCM 22kHz)\n", i);
			continue;
		}

		/* Fall back to original .bin */
		ti[i].cdda_format = CDDA_FMT_BIN;
	}
}

static char IsoFile[MAXPATHLEN] = "";
static s64 cdOpenCaseTime = 0;

//-----------------------------------------------------------------------------
// Multi-CD image section (PSP Eboot .pbp files: see handlepbp() )
//-----------------------------------------------------------------------------
unsigned int cdrIsoMultidiskCount = 0;
unsigned int cdrIsoMultidiskSelect = 0;
//senquack - The frontend GUI can register a callback function that gets called
// when a multi-CD image is detected on load (Eboot .pbp format supports this),
// allowing user to select which CD to boot from. Important for games like
// Resident Evil 2 which allow different story arcs depending on boot CD.
// The callback function will return having set cdrIsoMultidiskSelect.
void (CALLBACK *cdrIsoMultidiskCallback)(void) = NULL;
//-----------------------------------------------------------------------------
// END Multi-CD image section
//-----------------------------------------------------------------------------

// for CD swap
int ReloadCdromPlugin()
{
	if (cdrIsoActive())
		CDR_shutdown();

	return CDR_init();
}

void SetIsoFile(const char *filename) {
	//Reset multi-CD count & selection when loading any ISO
	cdrIsoMultidiskCount = 0;
	cdrIsoMultidiskSelect = 0;

	if (filename == NULL) {
		IsoFile[0] = '\0';
		return;
	}
	strcpy(IsoFile, filename);
}

const char *GetIsoFile(void) {
	return IsoFile;
}

boolean UsingIso(void) {
	return (IsoFile[0] != '\0');
}

void SetCdOpenCaseTime(s64 time) {
	cdOpenCaseTime = time;
}

s64 GetCdOpenCaseTime(void)
{
	return cdOpenCaseTime;
}

// get a sector from a msf-array
static unsigned int msf2sec(char *msf) {
	return ((msf[0] * 60 + msf[1]) * 75) + msf[2];
}

static void sec2msf(unsigned int s, char *msf) {
	msf[0] = s / 75 / 60;
	s = s - msf[0] * 75 * 60;
	msf[1] = s / 75;
	s = s - msf[1] * 75;
	msf[2] = s;
}

// divide a string of xx:yy:zz into m, s, f
static void tok2msf(char *time, char *msf) {
	char *token;

	token = strtok(time, ":");
	if (token) {
		msf[0] = atoi(token);
	}
	else {
		msf[0] = 0;
	}

	token = strtok(NULL, ":");
	if (token) {
		msf[1] = atoi(token);
	}
	else {
		msf[1] = 0;
	}

	token = strtok(NULL, ":");
	if (token) {
		msf[2] = atoi(token);
	}
	else {
		msf[2] = 0;
	}
}

#ifndef _WIN32
static long GetTickCount(void) {
	static time_t		initial_time = 0;
	struct timeval		now;

	gettimeofday(&now, NULL);

	if (initial_time == 0) {
		initial_time = now.tv_sec;
	}

	return (now.tv_sec - initial_time) * 1000L + now.tv_usec / 1000L;
}
#endif

#if defined(NO_THREADS) || defined(SF2000)
// SF2000: No CDDA thread support (bare metal, no pthreads)
static void stopCDDA(void) { playing = FALSE; }
static void startCDDA(void) { playing = TRUE; /* CDDA disabled on SF2000 */ }
#else

#ifdef _WIN32
static void playthread(void *param)
#else
static void *playthread(void *param)
#endif
{
	long osleep, d, t, i, s;
	unsigned char	tmp;
	int ret = 0, sector_offs;

	t = GetTickCount();

	while (playing) {
		s = 0;
		for (i = 0; i < sizeof(sndbuffer) / CD_FRAMESIZE_RAW; i++) {
			sector_offs = cdda_cur_sector - cdda_first_sector;
			if (sector_offs < 0) {
				d = CD_FRAMESIZE_RAW;
				memset(sndbuffer + s, 0, d);
			}
			else {
				d = cdimg_read_func(cddaHandle, cdda_file_offset,
					sndbuffer + s, sector_offs);
				if (d < CD_FRAMESIZE_RAW)
					break;
			}

			s += d;
			cdda_cur_sector++;
		}

		if (s == 0) {
			playing = FALSE;
			initial_offset = 0;
			break;
		}

		if (!cdr.Muted && playing) {
			if (cddaBigEndian) {
				for (i = 0; i < s / 2; i++) {
					tmp = sndbuffer[i * 2];
					sndbuffer[i * 2] = sndbuffer[i * 2 + 1];
					sndbuffer[i * 2 + 1] = tmp;
				}
			}

			// can't do it yet due to readahead..
			//cdrAttenuate((short *)sndbuffer, s / 4, 1);
			do {
				ret = SPU_playCDDAchannel((short *)sndbuffer, s);
				if (ret == 0x7761)
					usleep(6 * 1000);
			} while (ret == 0x7761 && playing); // rearmed_wait
		}

		if (ret != 0x676f) { // !rearmed_go
			// do approx sleep
			long now;

			//senquack - disabled pcsxReARMed-specific stuff we don't have (yet)
			// HACK: stop feeding data while emu is paused
			//extern int stop;
			//while (stop && playing)
			//	usleep(10000);

			now = GetTickCount();
			osleep = t - now;
			if (osleep <= 0) {
				osleep = 1;
				t = now;
			}
			else if (osleep > CDDA_FRAMETIME) {
				osleep = CDDA_FRAMETIME;
				t = now;
			}

			usleep(osleep * 1000);
			t += CDDA_FRAMETIME;
		}

	}

#ifdef _WIN32
	_endthread();
#else
	pthread_exit(0);
	return NULL;
#endif
}

// stop the CDDA playback
static void stopCDDA() {
	if (!playing) {
		return;
	}

	playing = FALSE;
#ifdef _WIN32
	WaitForSingleObject(threadid, INFINITE);
#else
	pthread_join(threadid, NULL);
#endif
}

// start the CDDA playback
static void startCDDA(void) {
	if (playing) {
		stopCDDA();
	}

	playing = TRUE;

#ifdef _WIN32
	threadid = (HANDLE)_beginthread(playthread, 0, NULL);
#else
	pthread_create(&threadid, NULL, playthread, NULL);
#endif
}
#endif // NO_THREADS || SF2000

// this function tries to get the .toc file of the given .bin
// the necessary data is put into the ti (trackinformation)-array
static int parsetoc(const char *isofile) {
	char			tocname[MAXPATHLEN];
	FILE			*fi;
	char			linebuf[256], tmp[256], name[256];
	char			*token;
	char			time[20], time2[20];
	unsigned int	t, sector_offs, sector_size;
	unsigned int	current_zero_gap = 0;

	numtracks = 0;

	// copy name of the iso and change extension from .bin to .toc
	strncpy(tocname, isofile, sizeof(tocname));
	tocname[MAXPATHLEN - 1] = '\0';
	if (strlen(tocname) >= 4) {
		strcpy(tocname + strlen(tocname) - 4, ".toc");
	} else {
		return -1;
	}

	bool toc_named_as_cue = false;
	if ((fi = fopen(tocname, "r")) == NULL) {
		// try changing extension to .cue (to satisfy some stupid tutorials)
		strcpy(tocname + strlen(tocname) - 4, ".cue");

		if ((fi = fopen(tocname, "r")) == NULL) {
			// if filename is image.toc.bin, try removing .bin (for Brasero)
			strcpy(tocname, isofile);
			t = strlen(tocname);
			if (t >= 8 && strcmp(tocname + t - 8, ".toc.bin") == 0) {
				tocname[t - 4] = '\0';
				if ((fi = fopen(tocname, "r")) == NULL) {
					return -1;
				}
			} else {
				return -1;
			}
		} else {
			toc_named_as_cue = true;
		}
	}

	// check if it's really a TOC and not a CUE
	bool is_toc_file = false;
	while (fgets(linebuf, sizeof(linebuf), fi) != NULL) {
		if (strstr(linebuf, "TRACK") != NULL) {
			char* mode_substr = strstr(linebuf, "MODE");
			if (mode_substr != NULL &&
				(mode_substr[4] == '1' || mode_substr[4] == '2') &&
			    mode_substr[5] != '/') {
				// A line containing both the substrings "TRACK" and either
				//  "MODE1" or "MODE2" exists, and the mode string lacks a
				//  trailing slash, which would have indicated a CUE file.
				is_toc_file = true;

				if (toc_named_as_cue)
					printf("\nWarning: .CUE file is really a .TOC file (processing as TOC..)\n");
			}
		}
	}

	if (!is_toc_file) {
		fclose(fi);
		return -1;
	}

	fseek(fi, 0, SEEK_SET);
	memset(&ti, 0, sizeof(ti));
	cddaBigEndian = TRUE; // cdrdao uses big-endian for CD Audio
	sector_size = CD_FRAMESIZE_RAW;
	sector_offs = 2 * 75;

	// parse the .toc file
	while (fgets(linebuf, sizeof(linebuf), fi) != NULL) {
		// search for tracks
		strncpy(tmp, linebuf, sizeof(linebuf));
		token = strtok(tmp, " ");

		if (token == NULL) continue;

		if (!strcmp(token, "TRACK")) {
			sector_offs += current_zero_gap;
			current_zero_gap = 0;

			// get type of track
			token = strtok(NULL, " ");
			numtracks++;

			if (!strncmp(token, "MODE2_RAW", 9)) {
				ti[numtracks].type = DATA;
				sec2msf(2 * 75, ti[numtracks].start); // assume data track on 0:2:0

				// check if this image contains mixed subchannel data
				token = strtok(NULL, " ");
				if (token != NULL && !strncmp(token, "RW", 2)) {
					sector_size = CD_FRAMESIZE_RAW + SUB_FRAMESIZE;
					subChanMixed = TRUE;
					if (!strncmp(token, "RW_RAW", 6))
						subChanRaw = TRUE;
				}
			}
			else if (!strncmp(token, "AUDIO", 5)) {
				ti[numtracks].type = CDDA;
			}
		}
		else if (!strcmp(token, "DATAFILE")) {
			if (ti[numtracks].type == CDDA) {
				sscanf(linebuf, "DATAFILE \"%[^\"]\" #%d %8s", name, &t, time2);
				ti[numtracks].start_offset = t;
				t = t / sector_size + sector_offs;
				sec2msf(t, (char *)&ti[numtracks].start);
				tok2msf((char *)&time2, (char *)&ti[numtracks].length);
			}
			else {
				sscanf(linebuf, "DATAFILE \"%[^\"]\" %8s", name, time);
				tok2msf((char *)&time, (char *)&ti[numtracks].length);
			}
		}
		else if (!strcmp(token, "FILE")) {
			sscanf(linebuf, "FILE \"%[^\"]\" #%d %8s %8s", name, &t, time, time2);
			tok2msf((char *)&time, (char *)&ti[numtracks].start);
			t += msf2sec(ti[numtracks].start) * sector_size;
			ti[numtracks].start_offset = t;
			t = t / sector_size + sector_offs;
			sec2msf(t, (char *)&ti[numtracks].start);
			tok2msf((char *)&time2, (char *)&ti[numtracks].length);
		}
		else if (!strcmp(token, "ZERO") || !strcmp(token, "SILENCE")) {
			// skip unneeded optional fields
			while (token != NULL) {
				token = strtok(NULL, " ");
				if (strchr(token, ':') != NULL)
					break;
			}
			if (token != NULL) {
				tok2msf(token, tmp);
				current_zero_gap = msf2sec(tmp);
			}
			if (numtracks > 1) {
				t = ti[numtracks - 1].start_offset;
				t /= sector_size;
				pregapOffset = t + msf2sec(ti[numtracks - 1].length);
			}
		}
		else if (!strcmp(token, "START")) {
			token = strtok(NULL, " ");
			if (token != NULL && strchr(token, ':')) {
				tok2msf(token, tmp);
				t = msf2sec(tmp);
				ti[numtracks].start_offset += (t - current_zero_gap) * sector_size;
				t = msf2sec(ti[numtracks].start) + t;
				sec2msf(t, (char *)&ti[numtracks].start);
			}
		}
	}

	if (numtracks <= 0) goto error;

	fclose(fi);
	return 0;

error:
	printf("\nError reading .TOC file %s\n", tocname);
	fclose(fi);
	return -1;
}

// this function tries to get the .cue file of the given .bin
// the necessary data is put into the ti (trackinformation)-array
static int parsecue(const char *isofile) {
	char			cuename[MAXPATHLEN];
	char			filepath[MAXPATHLEN];
	char			*incue_fname;
	FILE			*fi;
	char			*token;
	char			time[20];
	char			*tmp;
	char			linebuf[256], tmpb[256], dummy[256];
	unsigned int	incue_max_len;
	unsigned int	t, file_len, mode, sector_offs;
	unsigned int	sector_size = 2352;

	numtracks = 0;

	// copy name of the iso and change extension from .bin to .cue
	strncpy(cuename, isofile, sizeof(cuename));
	cuename[MAXPATHLEN - 1] = '\0';
	if (strlen(cuename) >= 4) {
		strcpy(cuename + strlen(cuename) - 4, ".cue");
	} else {
		return -1;
	}

	if ((fi = fopen(cuename, "r")) == NULL) {
		return -1;
	}

	// Some stupid tutorials wrongly tell users to use cdrdao to rip a
	// "bin/cue" image, which is in fact a "bin/toc" image. So let's check
	// that...
	while (fgets(linebuf, sizeof(linebuf), fi) != NULL) {
		if (strstr(linebuf, "TRACK") != NULL) {
			char* mode_substr = strstr(linebuf, "MODE");
			if (mode_substr != NULL &&
			    (mode_substr[4] == '1' || mode_substr[4] == '2') &&
			    mode_substr[5] != '/') {
				// A line containing both the substrings "TRACK" and either
				//  "MODE1" or "MODE2" exists, and the mode string lacks a
				//  trailing slash, which indicates this is a .TOC file
				//  falsely named as a .CUE file.
				printf("\nWarning: .CUE file is really a .TOC file (processing as TOC..)\n");
				fclose(fi);
				return parsetoc(isofile);
			}
		}
	}
	fseek(fi, 0, SEEK_SET);

	// build a path for files referenced in .cue
	strncpy(filepath, cuename, sizeof(filepath));
	tmp = strrchr(filepath, '/');
	if (tmp == NULL)
		tmp = strrchr(filepath, '\\');
	if (tmp != NULL)
		tmp++;
	else
		tmp = filepath;
	*tmp = 0;
	filepath[sizeof(filepath) - 1] = 0;
	incue_fname = tmp;
	incue_max_len = sizeof(filepath) - (tmp - filepath) - 1;

	// SF2000: Build subfolder path for multi-BIN games
	// Structure: ROMS/qpsx/GameName.cue + ROMS/qpsx/GameName/Track01.bin
	static char subfolder_path[MAXPATHLEN];
	char *cue_basename;
	strncpy(subfolder_path, filepath, sizeof(subfolder_path));
	// Get CUE filename without path
	cue_basename = strrchr(cuename, '/');
	if (cue_basename == NULL)
		cue_basename = strrchr(cuename, '\\');
	if (cue_basename != NULL)
		cue_basename++;
	else
		cue_basename = cuename;
	// Append CUE basename (without .cue) as subfolder
	size_t sf_len = strlen(subfolder_path);
	size_t cb_len = strlen(cue_basename);
	if (cb_len >= 4 && sf_len + cb_len < sizeof(subfolder_path) - 2) {
		strncat(subfolder_path, cue_basename, cb_len - 4); // remove .cue
		strcat(subfolder_path, "/");
	}

	memset(&ti, 0, sizeof(ti));

	file_len = 0;
	sector_offs = 2 * 75;

	while (fgets(linebuf, sizeof(linebuf), fi) != NULL) {
		strncpy(dummy, linebuf, sizeof(linebuf));
		token = strtok(dummy, " ");

		if (token == NULL) {
			continue;
		}

		if (!strcmp(token, "TRACK")) {
			numtracks++;

			sector_size = 0;
			if (strstr(linebuf, "AUDIO") != NULL) {
				ti[numtracks].type = CDDA;
				sector_size = 2352;
			}
			else if (sscanf(linebuf, " TRACK %u MODE%u/%u", &t, &mode, &sector_size) == 3)
				ti[numtracks].type = DATA;
			else {
				printf(".cue: failed to parse TRACK\n");
				ti[numtracks].type = numtracks == 1 ? DATA : CDDA;
			}
			if (sector_size == 0)
				sector_size = 2352;
		}
		else if (!strcmp(token, "INDEX")) {
			if (sscanf(linebuf, " INDEX %02d %8s", &t, time) != 2)
				printf(".cue: failed to parse INDEX\n");
			tok2msf(time, (char *)&ti[numtracks].start);

			t = msf2sec(ti[numtracks].start);
			ti[numtracks].start_offset = t * sector_size;
			t += sector_offs;
			sec2msf(t, ti[numtracks].start);

			// default track length to file length
			t = file_len - ti[numtracks].start_offset / sector_size;
			sec2msf(t, ti[numtracks].length);

			if (numtracks > 1 && ti[numtracks].handle == NULL) {
				// this track uses the same file as the last,
				// start of this track is last track's end
				t = msf2sec(ti[numtracks].start) - msf2sec(ti[numtracks - 1].start);
				sec2msf(t, ti[numtracks - 1].length);
			}
			if (numtracks > 1 && pregapOffset == -1)
				pregapOffset = ti[numtracks].start_offset / sector_size;
		}
		else if (!strcmp(token, "PREGAP")) {
			if (sscanf(linebuf, " PREGAP %8s", time) == 1) {
				tok2msf(time, dummy);
				sector_offs += msf2sec(dummy);
			}
			pregapOffset = -1; // mark to fill track start_offset
		}
		else if (!strcmp(token, "FILE")) {
			t = sscanf(linebuf, " FILE \"%255[^\"]\"", tmpb);
			if (t != 1)
				sscanf(linebuf, " FILE %255s", tmpb);

			// absolute path?
			/* HACK: Assume always relative paths, needed for frontend */
			/*ti[numtracks + 1].handle = fopen(tmpb, "rb");
			if (ti[numtracks + 1].handle == NULL)*/ {
				// relative to .cue?
				tmp = strrchr(tmpb, '\\');
				if (tmp == NULL)
					tmp = strrchr(tmpb, '/');
				if (tmp != NULL)
					tmp++;
				else
					tmp = tmpb;
				strncpy(incue_fname, tmp, incue_max_len);
				ti[numtracks + 1].handle = fopen(filepath, "rb");

				// SF2000: Try subfolder if file not found in CUE directory
				if (ti[numtracks + 1].handle == NULL) {
					static char subfolder_file[MAXPATHLEN];
					snprintf(subfolder_file, sizeof(subfolder_file), "%s%s", subfolder_path, tmp);
					ti[numtracks + 1].handle = fopen(subfolder_file, "rb");
					if (ti[numtracks + 1].handle != NULL) {
						// Update filepath for later use (cdHandle)
						strncpy(filepath, subfolder_file, sizeof(filepath));
					}
				}
			}

			// update global offset if this is not first file in this .cue
			if (numtracks + 1 > 1) {
				multifile = 1;
				sector_offs += file_len;
			}

			file_len = 0;
			if (ti[numtracks + 1].handle == NULL) {
				printf(("\ncould not open: %s\n"), filepath);
				continue;
			}
			/* v257: Store filepath for compressed format lookup later */
			strncpy(ti[numtracks + 1].filepath, filepath, MAXPATHLEN - 1);
			ti[numtracks + 1].filepath[MAXPATHLEN - 1] = '\0';
			ti[numtracks + 1].cdda_format = CDDA_FMT_BIN;

			fseek(ti[numtracks + 1].handle, 0, SEEK_END);
			file_len = ftell(ti[numtracks + 1].handle) / 2352;

			if (numtracks == 0 && strlen(isofile) >= 4 &&
				strcmp(isofile + strlen(isofile) - 4, ".cue") == 0)
			{
				// user selected .cue as image file, use it's data track instead
				fclose(cdHandle);
				cdHandle = fopen(filepath, "rb");
			}
		}
	}

	if (numtracks <= 0) goto error;

	fclose(fi);
	return 0;

error:
	printf("\nError reading .CUE file %s\n", cuename);
	fclose(fi);
	return -1;
}

// this function tries to get the .ccd file of the given .img
// the necessary data is put into the ti (trackinformation)-array
static int parseccd(const char *isofile) {
	char			ccdname[MAXPATHLEN];
	FILE			*fi;
	char			linebuf[256];
	unsigned int	t;

	numtracks = 0;

	// copy name of the iso and change extension from .img to .ccd
	strncpy(ccdname, isofile, sizeof(ccdname));
	ccdname[MAXPATHLEN - 1] = '\0';
	if (strlen(ccdname) >= 4) {
		strcpy(ccdname + strlen(ccdname) - 4, ".ccd");
	} else {
		return -1;
	}

	if ((fi = fopen(ccdname, "r")) == NULL) {
		return -1;
	}

	memset(&ti, 0, sizeof(ti));

	while (fgets(linebuf, sizeof(linebuf), fi) != NULL) {
		if (!strncmp(linebuf, "[TRACK", 6)){
			numtracks++;
		}
		else if (!strncmp(linebuf, "MODE=", 5)) {
			sscanf(linebuf, "MODE=%d", &t);
			ti[numtracks].type = ((t == 0) ? CDDA : DATA);
		}
		else if (!strncmp(linebuf, "INDEX 1=", 8)) {
			sscanf(linebuf, "INDEX 1=%d", &t);
			sec2msf(t + 2 * 75, ti[numtracks].start);
			ti[numtracks].start_offset = t * 2352;

			// If we've already seen another track, this is its end
			if (numtracks > 1) {
				t = msf2sec(ti[numtracks].start) - msf2sec(ti[numtracks - 1].start);
				sec2msf(t, ti[numtracks - 1].length);
			}
		}
	}

	if (numtracks <= 0)
		goto error;

	// Fill out the last track's end based on size
	if (numtracks >= 1) {
		fseek(cdHandle, 0, SEEK_END);
		t = ftell(cdHandle) / 2352 - msf2sec(ti[numtracks].start) + 2 * 75;
		sec2msf(t, ti[numtracks].length);
	}

	fclose(fi);
	return 0;

error:
	printf("\nError reading .CCD file %s\n", ccdname);
	fclose(fi);
	return -1;
}

//Alcohol 120% 'Media Descriptor Image' (.mds/.mdf) format support
// this function tries to get the .mds file of the given .mdf
// the necessary data is put into the ti (trackinformation)-array
static int parsemds(const char *isofile) {
	char      mdsname[MAXPATHLEN];
	FILE      *fi;
	uint32_t  offset, extra_offset, l, i;
	uint16_t  s;
	int       c;

	numtracks = 0;

	// copy name of the iso and change extension from .mdf to .mds
	strncpy(mdsname, isofile, sizeof(mdsname));
	mdsname[MAXPATHLEN - 1] = '\0';
	if (strlen(mdsname) >= 4) {
		strcpy(mdsname + strlen(mdsname) - 4, ".mds");
	} else {
		return -1;
	}

	if ((fi = fopen(mdsname, "rb")) == NULL)
		return -1;

	memset(&ti, 0, sizeof(ti));

	// check if it's a valid mds file
	if (fread(&i, 4, 1, fi) != 1)
		goto error;
	i = SWAP32(i);
	if (i != 0x4944454D) {
		// not an valid mds file
		printf("\nError: %s is not a valid .MDS file\n", mdsname);
		goto error;
	}

	// get offset to session block
	if (fseek(fi, 0x50, SEEK_SET) == -1 ||
	    fread(&offset, 4, 1, fi) != 1)
		goto error;
	offset = SWAP32(offset);

	// get total number of tracks
	offset += 14;
	if (fseek(fi, offset, SEEK_SET) == -1 ||
	    fread(&s, 2, 1, fi) != 1)
		goto error;
	s = SWAP16(s);
	numtracks = s;

	// get offset to track blocks
	if (fseek(fi, 4, SEEK_CUR) == -1 ||
	    fread(&offset, 4, 1, fi) != 1)
		goto error;
	offset = SWAP32(offset);

	// skip lead-in data
	while (1) {
		if (fseek(fi, offset + 4, SEEK_SET) == -1 ||
			(c = fgetc(fi)) == EOF)
			goto error;
		if (c < 0xA0)
			break;
		offset += 0x50;
	}

	// check if the image contains mixed subchannel data
	if (fseek(fi, offset + 1, SEEK_SET) == -1 ||
	    fgetc(fi) == EOF)
		goto error;
	subChanMixed = subChanRaw = (c ? TRUE : FALSE);

	// read track data
	for (i = 1; i <= numtracks; i++) {
		if (fseek(fi, offset, SEEK_SET) == -1)
			goto error;

		// get the track type
		if ((c = fgetc(fi)) == EOF)
			goto error;
		ti[i].type = (c == 0xA9) ? CDDA : DATA;
		fseek(fi, 8, SEEK_CUR);

		// get the track starting point
		for (int j = 0; j <= 2; ++j) {
			if ((c = fgetc(fi)) == EOF)
				goto error;
			ti[i].start[j] = c;
		}

		if (fread(&extra_offset, 4, 1, fi) != 1)
			goto error;
		extra_offset = SWAP32(extra_offset);

		// get track start offset (in .mdf)
		if (fseek(fi, offset + 0x28, SEEK_SET) == -1 ||
		    fread(&l, 4, 1, fi) != 1)
			goto error;
		l = SWAP32(l);
		ti[i].start_offset = l;

		// get pregap
		if (fseek(fi, extra_offset, SEEK_SET) == -1 ||
		    fread(&l, 4, 1, fi) != 1)
			goto error;
		l = SWAP32(l);
		if (l != 0 && i > 1)
			pregapOffset = msf2sec(ti[i].start);

		// get the track length
		if (fread(&l, 4, 1, fi) != 1)
			goto error;
		l = SWAP32(l);
		sec2msf(l, ti[i].length);

		offset += 0x50;
	}

	if (numtracks == 0) goto error;

	fclose(fi);
	return 0;

error:
	printf("\nError reading .MDS file %s\n", mdsname);
	fclose(fi);
	return -1;
}

#ifdef NO_ZLIB
// Compressed image support disabled - no zlib on SF2000
static int handlepbp(const char *isofile) { (void)isofile; return -1; }
static int handlecbin(const char *isofile) { (void)isofile; return -1; }
#else

static int handlepbp(const char *isofile) {
	struct {
		unsigned int sig;
		unsigned int dontcare[8];
		unsigned int psar_offs;
	} pbp_hdr;
	struct {
		unsigned char type;
		unsigned char pad0;
		unsigned char track;
		char index0[3];
		char pad1;
		char index1[3];
	} toc_entry;
	struct {
		unsigned int offset;
		unsigned int size;
		unsigned int dontcare[6];
	} index_entry;
	char psar_sig[11];
	off_t psisoimg_offs, cdimg_base;
	unsigned int t, cd_length;
	unsigned int offsettab[8];
	const char *ext = NULL;
	int i, ret;

	if (strlen(isofile) >= 4)
		ext = isofile + strlen(isofile) - 4;
	if (ext == NULL || (strcmp(ext, ".pbp") != 0 && strcmp(ext, ".PBP") != 0))
		return -1;

	fseeko(cdHandle, 0, SEEK_SET);

	numtracks = 0;

	ret = fread(&pbp_hdr, 1, sizeof(pbp_hdr), cdHandle);
	if (ret != sizeof(pbp_hdr)) {
		printf("failed to read pbp\n");
		goto fail_io;
	}

	ret = fseeko(cdHandle, pbp_hdr.psar_offs, SEEK_SET);
	if (ret != 0) {
		printf("failed to seek to %x\n", pbp_hdr.psar_offs);
		goto fail_io;
	}

	psisoimg_offs = pbp_hdr.psar_offs;
	ret = fread(psar_sig, 1, sizeof(psar_sig), cdHandle);
	if (ret != sizeof(psar_sig)) {
		printf("failed to read sig (1)\n");
		goto fail_io;
	}
	psar_sig[10] = 0;
	if (strcmp(psar_sig, "PSTITLEIMG") == 0) {
		// multidisk image?
		ret = fseeko(cdHandle, pbp_hdr.psar_offs + 0x200, SEEK_SET);
		if (ret != 0) {
			printf("failed to seek to %x\n", pbp_hdr.psar_offs + 0x200);
			goto fail_io;
		}

		if (fread(&offsettab, 1, sizeof(offsettab), cdHandle) != sizeof(offsettab)) {
			printf("failed to read offsettab\n");
			goto fail_io;
		}

		for (i = 0; i < sizeof(offsettab) / sizeof(offsettab[0]); i++) {
			if (offsettab[i] == 0)
				break;
		}
		cdrIsoMultidiskCount = i;
		if (cdrIsoMultidiskCount == 0) {
			printf("ERROR: multidisk eboot has 0 images?\n");
			goto fail_io;
		}

		//senquack - New feature allows GUI front end to register callback
		// to allow user to pick CD to boot from before proceeding:
		if (cdrIsoMultidiskCallback) cdrIsoMultidiskCallback();

		if (cdrIsoMultidiskSelect >= cdrIsoMultidiskCount)
			cdrIsoMultidiskSelect = 0;

		psisoimg_offs += offsettab[cdrIsoMultidiskSelect];

		ret = fseeko(cdHandle, psisoimg_offs, SEEK_SET);
		if (ret != 0) {
			printf("failed to seek to %llx\n", (long long)psisoimg_offs);
			goto fail_io;
		}

		ret = fread(psar_sig, 1, sizeof(psar_sig), cdHandle);
		if (ret != sizeof(psar_sig)) {
			printf("failed to read sig (2)\n");
			goto fail_io;
		}
		psar_sig[10] = 0;
	}

	if (strcmp(psar_sig, "PSISOIMG00") != 0) {
		printf("bad psar_sig: %s\n", psar_sig);
		goto fail_io;
	}

	// seek to TOC
	ret = fseeko(cdHandle, psisoimg_offs + 0x800, SEEK_SET);
	if (ret != 0) {
		printf("failed to seek to %llx\n", (long long)psisoimg_offs + 0x800);
		goto fail_io;
	}

	// first 3 entries are special
	if (fseek(cdHandle, sizeof(toc_entry), SEEK_CUR) == -1 ||
	    fread(&toc_entry, 1, sizeof(toc_entry), cdHandle) != sizeof(toc_entry)) {
		printf("failed reading first toc entry\n");
		goto fail_io;
	}

	numtracks = btoi(toc_entry.index1[0]);

	if (fread(&toc_entry, 1, sizeof(toc_entry), cdHandle) != sizeof(toc_entry)) {
		printf("failed reading second toc entry\n");
		goto fail_io;
	}

	cd_length = btoi(toc_entry.index1[0]) * 60 * 75 +
		btoi(toc_entry.index1[1]) * 75 + btoi(toc_entry.index1[2]);

	for (i = 1; i <= numtracks; i++) {
		if (fread(&toc_entry, 1, sizeof(toc_entry), cdHandle) != sizeof(toc_entry)) {
			printf("failed reading toc entry for track %d\n", i);
			goto fail_io;
		}

		ti[i].type = (toc_entry.type == 1) ? CDDA : DATA;

		ti[i].start_offset = btoi(toc_entry.index0[0]) * 60 * 75 +
			btoi(toc_entry.index0[1]) * 75 + btoi(toc_entry.index0[2]);
		ti[i].start_offset *= 2352;
		ti[i].start[0] = btoi(toc_entry.index1[0]);
		ti[i].start[1] = btoi(toc_entry.index1[1]);
		ti[i].start[2] = btoi(toc_entry.index1[2]);

		if (i > 1) {
			t = msf2sec(ti[i].start) - msf2sec(ti[i - 1].start);
			sec2msf(t, ti[i - 1].length);
		}
	}
	t = cd_length - ti[numtracks].start_offset / 2352;
	sec2msf(t, ti[numtracks].length);

	// seek to ISO index
	ret = fseeko(cdHandle, psisoimg_offs + 0x4000, SEEK_SET);
	if (ret != 0) {
		printf("failed to seek to ISO index\n");
		goto fail_io;
	}

	compr_img = (COMPR_IMG *)calloc(1, sizeof(*compr_img));
	if (compr_img == NULL)
		goto fail_io;

	compr_img->block_shift = 4;
	compr_img->current_block = (unsigned int)-1;

	compr_img->index_len = (0x100000 - 0x4000) / sizeof(index_entry);
	compr_img->index_table = (off_t *)malloc((compr_img->index_len + 1) *
				 sizeof(compr_img->index_table[0]));
	if (compr_img->index_table == NULL)
		goto fail_io;

	cdimg_base = psisoimg_offs + 0x100000;
	for (i = 0; i < compr_img->index_len; i++) {
		ret = fread(&index_entry, 1, sizeof(index_entry), cdHandle);
		if (ret != sizeof(index_entry)) {
			printf("failed to read index_entry #%d\n", i);
			goto fail_index;
		}

		if (index_entry.size == 0)
			break;

		compr_img->index_table[i] = cdimg_base + index_entry.offset;
	}
	compr_img->index_table[i] = cdimg_base + index_entry.offset + index_entry.size;

	return 0;

fail_index:
	free(compr_img->index_table);
	compr_img->index_table = NULL;
fail_io:
	if (compr_img != NULL) {
		free(compr_img);
		compr_img = NULL;
	}
	return -1;
}

static int handlecbin(const char *isofile) {
	struct
	{
		char magic[4];
		unsigned int header_size;
		unsigned long long total_bytes;
		unsigned int block_size;
		unsigned char ver;		// 1
		unsigned char align;
		unsigned char rsv_06[2];
	} ciso_hdr;
	const char *ext = NULL;
	unsigned int *index_table = NULL;
	unsigned int index = 0, plain;
	int i, ret;

	if (strlen(isofile) >= 5)
		ext = isofile + strlen(isofile) - 5;
	if (ext == NULL || (strcasecmp(ext + 1, ".cbn") != 0 && strcasecmp(ext, ".cbin") != 0))
		return -1;

	fseek(cdHandle, 0, SEEK_SET);

	ret = fread(&ciso_hdr, 1, sizeof(ciso_hdr), cdHandle);
	if (ret != sizeof(ciso_hdr)) {
		printf("failed to read ciso header\n");
		return -1;
	}

	if (strncmp(ciso_hdr.magic, "CISO", 4) != 0 || ciso_hdr.total_bytes <= 0 || ciso_hdr.block_size <= 0) {
		printf("bad ciso header\n");
		return -1;
	}
	if (ciso_hdr.header_size != 0 && ciso_hdr.header_size != sizeof(ciso_hdr)) {
		ret = fseeko(cdHandle, ciso_hdr.header_size, SEEK_SET);
		if (ret != 0) {
			printf("failed to seek to %x\n", ciso_hdr.header_size);
			return -1;
		}
	}

	compr_img = (COMPR_IMG *)calloc(1, sizeof(*compr_img));
	if (compr_img == NULL)
		goto fail_io;

	compr_img->block_shift = 0;
	compr_img->current_block = (unsigned int)-1;

	compr_img->index_len = ciso_hdr.total_bytes / ciso_hdr.block_size;
	index_table = (unsigned int *)malloc((compr_img->index_len + 1) * sizeof(index_table[0]));
	if (index_table == NULL)
		goto fail_io;

	ret = fread(index_table, sizeof(index_table[0]), compr_img->index_len, cdHandle);
	if (ret != compr_img->index_len) {
		printf("failed to read index table\n");
		goto fail_index;
	}

	compr_img->index_table = (off_t *)malloc((compr_img->index_len + 1) *
				 sizeof(compr_img->index_table[0]));
	if (compr_img->index_table == NULL)
		goto fail_index;

	for (i = 0; i < compr_img->index_len + 1; i++) {
		index = index_table[i];
		plain = index & 0x80000000;
		index &= 0x7fffffff;
		compr_img->index_table[i] = (off_t)index << ciso_hdr.align;
		if (plain)
			compr_img->index_table[i] |= OFF_T_MSB;
	}

	return 0;

fail_index:
	free(index_table);
fail_io:
	if (compr_img != NULL) {
		free(compr_img);
		compr_img = NULL;
	}
	return -1;
}
#endif // !NO_ZLIB

// this function tries to get the .sub file of the given .img
static int opensubfile(const char *isoname) {
	char		subname[MAXPATHLEN];

	// copy name of the iso and change extension from .img to .sub
	strncpy(subname, isoname, sizeof(subname));
	subname[MAXPATHLEN - 1] = '\0';
	if (strlen(subname) >= 4) {
		strcpy(subname + strlen(subname) - 4, ".sub");
	}
	else {
		return -1;
	}

	subHandle = fopen(subname, "rb");
	if (subHandle == NULL) {
		return -1;
	}

	return 0;
}

static int opensbifile(const char *isoname) {
	char		sbiname[MAXPATHLEN];
	int		s;

	strncpy(sbiname, isoname, sizeof(sbiname));
	sbiname[MAXPATHLEN - 1] = '\0';
	if (strlen(sbiname) >= 4) {
		strcpy(sbiname + strlen(sbiname) - 4, ".sbi");
	}
	else {
		return -1;
	}

	fseek(cdHandle, 0, SEEK_END);
	s = ftell(cdHandle) / 2352;

	return LoadSBI(sbiname, s);
}

static int cdread_normal(FILE *f, unsigned int base, void *dest, int sector)
{
	if (fseek(f, base + sector * CD_FRAMESIZE_RAW, SEEK_SET) == -1)
		return -1;
	return fread(dest, 1, CD_FRAMESIZE_RAW, f);
}

static int cdread_sub_mixed(FILE *f, unsigned int base, void *dest, int sector)
{
	int ret;

	if (fseek(f, base + sector * (CD_FRAMESIZE_RAW + SUB_FRAMESIZE), SEEK_SET) == -1)
		return -1;
	ret = fread(dest, 1, CD_FRAMESIZE_RAW, f);

	if (fread(subbuffer, 1, SUB_FRAMESIZE, f) != SUB_FRAMESIZE) {
		printf("Error reading mixed subchannel info in cdread_sub_mixed()\n");
	} else {
		if (subChanRaw) DecodeRawSubData();
	}

	return ret;
}

#ifndef NO_ZLIB
static int uncompress2(void *out, unsigned long *out_size, void *in, unsigned long in_size)
{
	static z_stream z;
	int ret = 0;

	if (z.zalloc == NULL) {
		// XXX: one-time leak here..
		z.next_in = Z_NULL;
		z.avail_in = 0;
		z.zalloc = Z_NULL;
		z.zfree = Z_NULL;
		z.opaque = Z_NULL;
		ret = inflateInit2(&z, -15);
	}
	else
		ret = inflateReset(&z);
	if (ret != Z_OK)
		return ret;

	z.next_in = (Bytef *)in;
	z.avail_in = in_size;
	z.next_out = (Bytef *)out;
	z.avail_out = *out_size;

	ret = inflate(&z, Z_NO_FLUSH);
	//inflateEnd(&z);

	*out_size -= z.avail_out;
	return ret == 1 ? 0 : ret;
}

static int cdread_compressed(FILE *f, unsigned int base, void *dest, int sector)
{
	unsigned long cdbuffer_size, cdbuffer_size_expect;
	unsigned int size;
	int is_compressed;
	off_t start_byte;
	int ret, block;

	if (base)
		sector += base / 2352;

	block = sector >> compr_img->block_shift;
	compr_img->sector_in_blk = sector & ((1 << compr_img->block_shift) - 1);

	if (block == compr_img->current_block) {
		//printf("hit sect %d\n", sector);
		goto finish;
	}

	if (sector >= compr_img->index_len * 16) {
		printf("sector %d is past img end\n", sector);
		return -1;
	}

	start_byte = compr_img->index_table[block] & ~OFF_T_MSB;
	if (fseeko(cdHandle, start_byte, SEEK_SET) != 0) {
		printf("seek error for block %d at %llx: ",
			block, (long long)start_byte);
		perror(NULL);
		return -1;
	}

	is_compressed = !(compr_img->index_table[block] & OFF_T_MSB);
	size = (compr_img->index_table[block + 1] & ~OFF_T_MSB) - start_byte;
	if (size > sizeof(compr_img->buff_compressed)) {
		printf("block %d is too large: %u\n", block, size);
		return -1;
	}

	if (fread(is_compressed ? compr_img->buff_compressed : compr_img->buff_raw[0],
				1, size, cdHandle) != size) {
		printf("read error for block %d at %lx: ", block, start_byte);
		perror(NULL);
		return -1;
	}

	if (is_compressed) {
		cdbuffer_size_expect = sizeof(compr_img->buff_raw[0]) << compr_img->block_shift;
		cdbuffer_size = cdbuffer_size_expect;
		ret = uncompress2(compr_img->buff_raw[0], &cdbuffer_size, compr_img->buff_compressed, size);
		if (ret != 0) {
			printf("uncompress failed with %d for block %d, sector %d\n",
					ret, block, sector);
			return -1;
		}
		if (cdbuffer_size != cdbuffer_size_expect)
			printf("cdbuffer_size: %lu != %lu, sector %d\n", cdbuffer_size,
					cdbuffer_size_expect, sector);
	}

	// done at last!
	compr_img->current_block = block;

finish:
	if (dest != cdbuffer) // copy avoid HACK
		memcpy(dest, compr_img->buff_raw[compr_img->sector_in_blk],
			CD_FRAMESIZE_RAW);
	return CD_FRAMESIZE_RAW;
}
#endif // NO_ZLIB

static int cdread_2048(FILE *f, unsigned int base, void *dest, int sector)
{
	int ret;

	if (fseek(f, base + sector * 2048, SEEK_SET) == -1)
		return -1;

	ret = fread((char *)dest + 12 * 2, 1, 2048, f);

	// not really necessary, fake mode 2 header
	memset(cdbuffer, 0, 12 * 2);
	sec2msf(sector + 2 * 75, (char *)&cdbuffer[12]);
	cdbuffer[12 + 3] = 1;

	return ret;
}

#ifndef NO_ZLIB
static unsigned char *CDR_getBuffer_compr(void) {
	return compr_img->buff_raw[compr_img->sector_in_blk] + 12;
}
#endif

static unsigned char *CDR_getBuffer_norm(void) {
	return cdbuffer + 12;
}

static void PrintTracks(void) {
	int i;

	for (i = 1; i <= numtracks; i++) {
		printf(("Track %.2d (%s) - Start %.2d:%.2d:%.2d, Length %.2d:%.2d:%.2d\n"),
			i, (ti[i].type == DATA ? "DATA" : "AUDIO"),
			ti[i].start[0], ti[i].start[1], ti[i].start[2],
			ti[i].length[0], ti[i].length[1], ti[i].length[2]);
	}
}

// This function is invoked by the front-end when opening an CDR_
// file for playback
long CDR_open(void) {
	boolean isMode1CDR_ = FALSE;
	char alt_bin_filename[MAXPATHLEN];
	const char *bin_filename;

	if (cdHandle != NULL) {
		return 0; // it's already open
	}

	cdHandle = fopen(GetIsoFile(), "rb");
	if (cdHandle == NULL) {
		printf(("Could't open '%s' for reading: %s\n"),
			GetIsoFile(), strerror(errno));
		return -1;
	}

	printf("Loaded CD Image: %s", GetIsoFile());

	cddaBigEndian = FALSE;
	subChanMixed = FALSE;
	subChanRaw = FALSE;
	pregapOffset = 0;
	cdrIsoMultidiskCount = 1;
	multifile = 0;

	CDR_getBuffer = CDR_getBuffer_norm;
	cdimg_read_func = cdread_normal;

	if (parsetoc(GetIsoFile()) == 0) {
		printf("[+toc]");
	}
	else if (parseccd(GetIsoFile()) == 0) {
		printf("[+ccd]");
	}
	else if (parsemds(GetIsoFile()) == 0) {
		printf("[+mds]");
	}
	else if (parsecue(GetIsoFile()) == 0) {
		printf("[+cue]");
	}
#ifndef NO_ZLIB
	if (handlepbp(GetIsoFile()) == 0) {
		printf("[pbp]");
		CDR_getBuffer = CDR_getBuffer_compr;
		cdimg_read_func = cdread_compressed;
	}
	else if (handlecbin(GetIsoFile()) == 0) {
		printf("[cbin]");
		CDR_getBuffer = CDR_getBuffer_compr;
		cdimg_read_func = cdread_compressed;
	}
#endif

	if (!subChanMixed && opensubfile(GetIsoFile()) == 0) {
		printf("[+sub]");
	}
	if (opensbifile(GetIsoFile()) == 0) {
		printf("[+sbi]");
	}
	fseeko(cdHandle, 0, SEEK_END);

	// maybe user selected metadata file instead of main .bin ..
	bin_filename = GetIsoFile();
	if (ftello(cdHandle) < 2352 * 0x10) {
		static const char *exts[] = { ".bin", ".BIN", ".img", ".IMG" };
		FILE *tmpf = NULL;
		size_t i;
		char *p;

		strncpy(alt_bin_filename, bin_filename, sizeof(alt_bin_filename));
		alt_bin_filename[MAXPATHLEN - 1] = '\0';
		if (strlen(alt_bin_filename) >= 4) {
			p = alt_bin_filename + strlen(alt_bin_filename) - 4;
			for (i = 0; i < sizeof(exts) / sizeof(exts[0]); i++) {
				strcpy(p, exts[i]);
				tmpf = fopen(alt_bin_filename, "rb");
				if (tmpf != NULL)
					break;
			}
		}
		if (tmpf != NULL) {
			bin_filename = alt_bin_filename;
			fclose(cdHandle);
			cdHandle = tmpf;
			fseeko(cdHandle, 0, SEEK_END);
		}
	}

	// guess whether it is mode1/2048
	if (ftello(cdHandle) % 2048 == 0) {
		unsigned int modeTest = 0;
		fseek(cdHandle, 0, SEEK_SET);
		if (fread(&modeTest, 4, 1, cdHandle) == 1) {
			if (SWAP32(modeTest) != 0xffffff00) {
				printf("[2048]");
				isMode1CDR_ = TRUE;
			}
		}
	}
	fseek(cdHandle, 0, SEEK_SET);

	printf(".\n");

	if (cdrIsoMultidiskCount > 1)
		printf("Loading multi-CD image %d of %d.\n", cdrIsoMultidiskSelect+1, cdrIsoMultidiskCount);

	PrintTracks();

	if (subChanMixed)
		cdimg_read_func = cdread_sub_mixed;
	else if (isMode1CDR_)
		cdimg_read_func = cdread_2048;

	// make sure we have another handle open for cdda
	if (numtracks > 1 && ti[1].handle == NULL) {
		ti[1].handle = fopen(bin_filename, "rb");
		/* v255: Store path for potential .binsav lookup */
		strncpy(ti[1].filepath, bin_filename, MAXPATHLEN - 1);
		ti[1].filepath[MAXPATHLEN - 1] = '\0';
	}
	cdda_cur_sector = 0;
	cdda_file_offset = 0;

	/* v257: Try to upgrade CDDA tracks to compressed formats */
	upgrade_cdda_formats();

	return 0;
}

long CDR_close(void) {
	int i;

	if (cdHandle != NULL) {
		fclose(cdHandle);
		cdHandle = NULL;
	}
	if (subHandle != NULL) {
		fclose(subHandle);
		subHandle = NULL;
	}
	stopCDDA();
	cddaHandle = NULL;

#ifndef NO_ZLIB
	if (compr_img != NULL) {
		free(compr_img->index_table);
		free(compr_img);
		compr_img = NULL;
	}
#endif

	for (i = 1; i <= numtracks; i++) {
		if (ti[i].handle != NULL) {
			fclose(ti[i].handle);
			ti[i].handle = NULL;
		}
	}
	numtracks = 0;
	ti[1].type = (cd_type)0;
	UnloadSBI();
	memset(cdbuffer, 0, sizeof(cdbuffer));
	CDR_getBuffer = CDR_getBuffer_norm;

	return 0;
}

long CDR_init(void) {
	numtracks = 0;
	assert(cdHandle == NULL);
	assert(subHandle == NULL);

	return 0; // do nothing
}

long CDR_shutdown(void) {
	CDR_close();

	//senquack - Added:
	FreePPFCache();

	return 0;
}

// return Starting and Ending Track
// buffer:
//  byte 0 - start track
//  byte 1 - end track
long CDR_getTN(unsigned char *buffer) {
	buffer[0] = 1;

	if (numtracks > 0) {
		buffer[1] = numtracks;
	}
	else {
		buffer[1] = 1;
	}

	return 0;
}

// return Track Time
// buffer:
//  byte 0 - frame
//  byte 1 - second
//  byte 2 - minute
long CDR_getTD(unsigned char track, unsigned char *buffer) {
	if (track == 0) {
		unsigned int sect;
		unsigned char time[3];
		sect = msf2sec(ti[numtracks].start) + msf2sec(ti[numtracks].length);
		sec2msf(sect, (char *)time);
		buffer[2] = time[0];
		buffer[1] = time[1];
		buffer[0] = time[2];
	}
	else if (numtracks > 0 && track <= numtracks) {
		buffer[2] = ti[track].start[0];
		buffer[1] = ti[track].start[1];
		buffer[0] = ti[track].start[2];
	}
	else {
		buffer[2] = 0;
		buffer[1] = 2;
		buffer[0] = 0;
	}

	return 0;
}

// decode 'raw' subchannel data ripped by cdrdao
static void DecodeRawSubData(void) {
	unsigned char subQData[12];
	int i;

	memset(subQData, 0, sizeof(subQData));

	for (i = 0; i < 8 * 12; i++) {
		if (subbuffer[i] & (1 << 6)) { // only subchannel Q is needed
			subQData[i >> 3] |= (1 << (7 - (i & 7)));
		}
	}

	memcpy(&subbuffer[12], subQData, 12);
}

// read track
// time: byte 0 - minute; byte 1 - second; byte 2 - frame
// uses bcd format
long CDR_readTrack(unsigned char *time) {
	int sector = MSF2SECT(btoi(time[0]), btoi(time[1]), btoi(time[2]));
	long ret;

	if (cdHandle == NULL) {
		return -1;
	}

	if (pregapOffset) {
		subChanMissing = FALSE;
		if (sector >= pregapOffset) {
			sector -= 2 * 75;
			if (sector < pregapOffset)
				subChanMissing = TRUE;
		}
	}

	ret = cdimg_read_func(cdHandle, 0, cdbuffer, sector);
	if (ret < 0)
		return -1;

	if (subHandle != NULL) {
		if (fseek(subHandle, sector * SUB_FRAMESIZE, SEEK_SET) != -1 &&
		    fread(subbuffer, 1, SUB_FRAMESIZE, subHandle) == SUB_FRAMESIZE) {
			if (subChanRaw) DecodeRawSubData();
		} else {
			printf("Error reading subchannel info in CDR_readTrack()\n");
		}
	}

	return 0;
}

// plays cdda audio
// sector: byte 0 - minute; byte 1 - second; byte 2 - frame
// does NOT uses bcd format
long CDR_play(unsigned char *time) {
	unsigned int i;

	if (numtracks <= 1)
		return 0;

	// find the track
	cdda_cur_sector = msf2sec((char *)time);
	for (i = numtracks; i > 1; i--) {
		cdda_first_sector = msf2sec(ti[i].start);
		if (cdda_first_sector <= cdda_cur_sector + 2 * 75)
			break;
	}
	cdda_file_offset = ti[i].start_offset;

	// find the file that contains this track
	for (; i > 1; i--)
		if (ti[i].handle != NULL)
			break;

	cddaHandle = ti[i].handle;

	// Uncomment when SPU_playCDDAchannel is a func ptr again
	//if (SPU_playCDDAchannel != NULL)
		startCDDA();

	return 0;
}

// stops cdda audio
long CDR_stop(void) {
	stopCDDA();
	return 0;
}

// gets subchannel data
unsigned char* CDR_getBufferSub(void) {
	if ((subHandle != NULL || subChanMixed) && !subChanMissing) {
		return subbuffer;
	}

	return NULL;
}

long CDR_getStatus(struct CdrStat *stat) {
	u32 sect;

	//senquack - PCSX Rearmed cdriso.c code has this if/else cdOpenCaseTime logic
	// abstracted through a call here to a separate function CDR__getStatus() in
	// its plugins.c, but since we don't support multiple CD plugins, just do
	// the logic directly here. (This handles CD swapping status bit)
	//CDR__getStatus(stat);  // <<<< This function is essentially this vvvvv
	if (cdOpenCaseTime < 0 || cdOpenCaseTime > (s64)time(NULL))
		stat->Status = 0x10;
	else
		stat->Status = 0;


	if (playing) {
		stat->Type = 0x02;
		stat->Status |= 0x80;
	}
	else {
		// BIOS - boot ID (CD type)
		stat->Type = ti[1].type;
	}

	// relative -> absolute time
	sect = cddaCurPos;
	sec2msf(sect, (char *)stat->Time);

	return 0;
}

/*
 * v257: Read CDDA sector into buffer
 * Handles all three formats: .binadpcm, .binwav, .bin
 */
long CDR_readCDDA(unsigned char m, unsigned char s, unsigned char f, unsigned char *buffer) {
	unsigned char msf[3] = {m, s, f};
	unsigned int file, track, track_start = 0;
	int ret;

	cddaCurPos = msf2sec((char *)msf);

	// find current track index
	for (track = numtracks; ; track--) {
		track_start = msf2sec(ti[track].start);
		if (track_start <= cddaCurPos)
			break;
		if (track == 1)
			break;
	}

	// data tracks play silent
	if (ti[track].type != CDDA) {
		memset(buffer, 0, CD_FRAMESIZE_RAW);
		return 0;
	}

	file = 1;
	if (multifile) {
		// find the file that contains this track
		for (file = track; file > 1; file--)
			if (ti[file].handle != NULL)
				break;
	}

	/*
	 * v257: Handle compressed CDDA formats
	 */
	if (ti[file].cdda_format == CDDA_FMT_BINADPCM) {
		/*
		 * .binadpcm format: IMA ADPCM 22kHz mono (16x smaller)
		 * Sector: 4-byte header + 147 bytes ADPCM data + 1 pad = 152 bytes
		 * Header: predictor (int16), step_index (uint8), reserved (uint8)
		 * Output: 294 samples -> upsample to 588 stereo pairs (2352 bytes)
		 */
		static unsigned char adpcm_buf[BINADPCM_SECTOR_SIZE];
		unsigned int sector = cddaCurPos - track_start;
		short *out = (short *)buffer;
		int i;
		int16_t predictor;
		int step_idx;
		unsigned char *data;

		/* start_offset is in 2352-byte units, convert to 152-byte units */
		/* Ratio: 2352/152 = 15.47, but we use integer math: offset * 152 / 2352 */
		unsigned int adpcm_file_offset = (ti[track].start_offset * BINADPCM_SECTOR_SIZE) / CD_FRAMESIZE_RAW;

		if (fseek(ti[file].handle, adpcm_file_offset + sector * BINADPCM_SECTOR_SIZE, SEEK_SET) == -1) {
			memset(buffer, 0, CD_FRAMESIZE_RAW);
			return -1;
		}

		ret = fread(adpcm_buf, 1, BINADPCM_SECTOR_SIZE, ti[file].handle);
		if (ret != BINADPCM_SECTOR_SIZE) {
			memset(buffer, 0, CD_FRAMESIZE_RAW);
			return -1;
		}

		/* Read header */
		predictor = (int16_t)(adpcm_buf[0] | (adpcm_buf[1] << 8));
		step_idx = adpcm_buf[2];
		if (step_idx > 88) step_idx = 88;

		/* Decode ADPCM and upsample to 44kHz stereo */
		data = adpcm_buf + BINADPCM_HEADER_SIZE;
		for (i = 0; i < BINADPCM_DATA_SIZE; i++) {
			unsigned char byte = data[i];
			int16_t sample1, sample2;

			/* Two nibbles per byte, low nibble first */
			sample1 = ima_decode_sample(byte & 0x0F, &predictor, &step_idx);
			sample2 = ima_decode_sample((byte >> 4) & 0x0F, &predictor, &step_idx);

			/* Upsample: each 22kHz sample -> 2 samples at 44kHz, stereo */
			*out++ = sample1;  /* L */
			*out++ = sample1;  /* R */
			*out++ = sample1;  /* L (duplicate for 2x rate) */
			*out++ = sample1;  /* R */
			*out++ = sample2;  /* L */
			*out++ = sample2;  /* R */
			*out++ = sample2;  /* L (duplicate for 2x rate) */
			*out++ = sample2;  /* R */
		}

		return 0;
	}
	else if (ti[file].cdda_format == CDDA_FMT_BINWAV) {
		/*
		 * .binwav format: Raw PCM 22kHz mono (4x smaller)
		 * Read 588 bytes and upsample to 2352 bytes (44kHz stereo)
		 */
		static unsigned char binwav_buf[BINWAV_SECTOR_SIZE];
		unsigned int sector = cddaCurPos - track_start;
		short *out = (short *)buffer;
		short *in;
		int i;

		/* start_offset is in 2352-byte units, convert to 588-byte units (divide by 4) */
		unsigned int binwav_offset = ti[track].start_offset / 4;

		if (fseek(ti[file].handle, binwav_offset + sector * BINWAV_SECTOR_SIZE, SEEK_SET) == -1) {
			memset(buffer, 0, CD_FRAMESIZE_RAW);
			return -1;
		}

		ret = fread(binwav_buf, 1, BINWAV_SECTOR_SIZE, ti[file].handle);
		if (ret != BINWAV_SECTOR_SIZE) {
			memset(buffer, 0, CD_FRAMESIZE_RAW);
			return -1;
		}

		/* Upsample 22kHz mono -> 44kHz stereo */
		in = (short *)binwav_buf;
		for (i = 0; i < 294; i++) {
			short sample = in[i];
			*out++ = sample;  /* L */
			*out++ = sample;  /* R */
			*out++ = sample;  /* L (duplicate for 2x rate) */
			*out++ = sample;  /* R */
		}

		return 0;
	}

	/* Standard read for original .bin format (CDDA_FMT_BIN) */
	ret = cdimg_read_func(ti[file].handle, ti[track].start_offset,
		buffer, cddaCurPos - track_start);
	if (ret != CD_FRAMESIZE_RAW) {
		memset(buffer, 0, CD_FRAMESIZE_RAW);
		return -1;
	}

	if (cddaBigEndian) {
		int i;
		unsigned char tmp;

		for (i = 0; i < CD_FRAMESIZE_RAW / 2; i++) {
			tmp = buffer[i * 2];
			buffer[i * 2] = buffer[i * 2 + 1];
			buffer[i * 2 + 1] = tmp;
		}
	}

	return 0;
}

/*
 * v253: CDR_readCDDA_batch - Read multiple consecutive CDDA sectors efficiently
 *
 * For single-file uncompressed BIN images (most common), this reads all sectors
 * with a single fseek + fread instead of one per sector. This reduces SD card
 * I/O operations from N to 1, which is critical on slow storage like SF2000.
 *
 * For multifile/compressed images, falls back to individual reads.
 *
 * Parameters:
 *   m, s, f - MSF of first sector to read
 *   buffer - output buffer (must be at least count * CD_FRAMESIZE_RAW bytes)
 *   count - number of consecutive sectors to read
 *
 * Returns: 0 on success, -1 on error
 */
long CDR_readCDDA_batch(unsigned char m, unsigned char s, unsigned char f, unsigned char *buffer, int count) {
	unsigned char msf[3] = {m, s, f};
	unsigned int file, track, track_start = 0;
	int i, ret;
	unsigned int start_sector;

	if (count <= 0) return 0;

	start_sector = msf2sec((char *)msf);
	cddaCurPos = start_sector;

	/* Find current track */
	for (track = numtracks; ; track--) {
		track_start = msf2sec(ti[track].start);
		if (track_start <= start_sector)
			break;
		if (track == 1)
			break;
	}

	/* Data tracks play silent */
	if (ti[track].type != CDDA) {
		memset(buffer, 0, CD_FRAMESIZE_RAW * count);
		return 0;
	}

	file = 1;
	if (multifile) {
		/* Multifile: fall back to individual reads (tracks may span files) */
		for (i = 0; i < count; i++) {
			ret = CDR_readCDDA(m, s, f, buffer + i * CD_FRAMESIZE_RAW);
			if (ret < 0) return ret;
			/* Increment MSF */
			f++;
			if (f >= 75) { f = 0; s++; }
			if (s >= 60) { s = 0; m++; }
		}
		return 0;
	}

	/*
	 * v257: Handle compressed CDDA batch reads
	 */
	if (ti[file].cdda_format == CDDA_FMT_BINADPCM) {
		/* ADPCM needs sequential decoding (state-dependent), use individual reads */
		goto fallback;
	}
	else if (ti[file].cdda_format == CDDA_FMT_BINWAV) {
		/* .binwav: batch read and upsample (22kHz mono -> 44kHz stereo) */
		static unsigned char binwav_batch[BINWAV_SECTOR_SIZE * 32];  /* Max 32 sectors */
		unsigned int sector = start_sector - track_start;
		int binwav_bytes = BINWAV_SECTOR_SIZE * count;
		int j;
		/* start_offset is in 2352-byte units, convert to 588-byte units */
		unsigned int binwav_offset = ti[track].start_offset / 4;

		if (count > 32) count = 32;  /* Safety limit */

		/* Bulk read from .binwav (588 bytes per sector) */
		if (fseek(ti[file].handle, binwav_offset + sector * BINWAV_SECTOR_SIZE, SEEK_SET) == -1) {
			goto fallback;
		}

		ret = fread(binwav_batch, 1, binwav_bytes, ti[file].handle);
		if (ret != binwav_bytes) {
			goto fallback;
		}

		/* Upsample all sectors: 22kHz mono -> 44kHz stereo */
		for (j = 0; j < count; j++) {
			short *in = (short *)(binwav_batch + j * BINWAV_SECTOR_SIZE);
			short *out = (short *)(buffer + j * CD_FRAMESIZE_RAW);
			int k;
			for (k = 0; k < 294; k++) {
				short sample = in[k];
				*out++ = sample;
				*out++ = sample;
				*out++ = sample;
				*out++ = sample;
			}
		}

		return 0;
	}

	/*
	 * Single-file optimization: Check if cdimg_read_func is cdread_normal
	 * For uncompressed images, we can read all sectors with one fread
	 */
	{
		FILE *fh = ti[file].handle;
		unsigned int base = ti[track].start_offset;
		unsigned int sector = start_sector - track_start;
		int total_bytes = CD_FRAMESIZE_RAW * count;

		/* Try bulk read - fseek to first sector, fread all at once */
		if (fseek(fh, base + sector * CD_FRAMESIZE_RAW, SEEK_SET) == -1) {
			/* Fall back to individual reads */
			goto fallback;
		}

		ret = fread(buffer, 1, total_bytes, fh);
		if (ret != total_bytes) {
			/* Partial read - some sectors may have been read. Fall back. */
			goto fallback;
		}

		/* Success! Handle endianness if needed */
		if (cddaBigEndian) {
			unsigned char tmp;
			int j;
			for (j = 0; j < total_bytes / 2; j++) {
				tmp = buffer[j * 2];
				buffer[j * 2] = buffer[j * 2 + 1];
				buffer[j * 2 + 1] = tmp;
			}
		}

		return 0;
	}

fallback:
	/* Fallback: read sectors individually */
	for (i = 0; i < count; i++) {
		unsigned char cur_m = m, cur_s = s, cur_f = f;
		/* Calculate MSF for this sector */
		int frames = i;
		cur_f += frames;
		while (cur_f >= 75) { cur_f -= 75; cur_s++; }
		while (cur_s >= 60) { cur_s -= 60; cur_m++; }

		ret = CDR_readCDDA(cur_m, cur_s, cur_f, buffer + i * CD_FRAMESIZE_RAW);
		if (ret < 0) return ret;
	}
	return 0;
}

void cdrIsoInit(void) {
	numtracks = 0;
}

int cdrIsoActive(void) {
	return (cdHandle != NULL);
}

/*
 * ============================================================================
 * v258: CDDA Conversion Functions
 * Convert audio tracks from .bin to .binwav/.binadpcm on-device
 * ============================================================================
 */

/* SF2000 firmware filesystem functions */
extern "C" int fs_open(const char *path, int flags, int perms);
extern "C" ssize_t fs_write(int fd, const void *buf, size_t count);
extern "C" ssize_t fs_read(int fd, void *buf, size_t count);
extern "C" int fs_close(int fd);
extern "C" int fs_sync(const char *path);
extern "C" int64_t fs_lseek(int fd, int64_t offset, int whence);

/* Firmware file flags */
#define FS_O_RDONLY 0x0000
#define FS_O_WRONLY 0x0001
#define FS_O_RDWR   0x0002
#define FS_O_CREAT  0x0100
#define FS_O_TRUNC  0x0200

/* Seek modes */
#define FS_SEEK_SET 0
#define FS_SEEK_CUR 1
#define FS_SEEK_END 2

/* Track version info for conversion menu - typedef in cdriso.h */
static cdda_track_info_t cdda_conv_tracks[MAXTRACKS];
static int cdda_conv_num_tracks = 0;

/* Conversion progress callback - typedef in cdriso.h */
static cdda_progress_cb_t cdda_progress_callback = NULL;

void cdda_set_progress_callback(cdda_progress_cb_t cb) {
	cdda_progress_callback = cb;
}

/*
 * v261: Incremental conversion state
 * Allows converting N sectors per frame so UI can update
 */
#define CDDA_SECTORS_PER_STEP 50  /* Convert this many sectors per frame */

static int cdda_incr_active = 0;        /* Incremental conversion in progress */
static int cdda_incr_format = 0;        /* 1=binwav, 2=binadpcm */
static int cdda_incr_track_idx = -1;    /* Current track index */
static int cdda_incr_fd_in = -1;        /* Input file descriptor */
static int cdda_incr_fd_out = -1;       /* Output file descriptor */
static unsigned int cdda_incr_sector = 0;        /* Current sector */
static unsigned int cdda_incr_total_sectors = 0; /* Total sectors in track */
static char cdda_incr_out_path[512];    /* Output path (for delete on cancel) */

/* ADPCM encoder state (preserved between calls) */
static int16_t cdda_incr_predictor = 0;
static int cdda_incr_step_idx = 0;

/* Cancel flag - set by UI, checked by converter */
static volatile int cdda_cancel_flag = 0;

/* Set cancel flag from UI */
void cdda_request_cancel(void) {
	cdda_cancel_flag = 1;
}

/* Check if conversion is active */
int cdda_is_converting(void) {
	return cdda_incr_active;
}

/* Check if file exists and has meaningful size (>7 bytes)
 * v265: Files truncated to ≤7 bytes are treated as deleted
 */
static int file_exists(const char *path) {
	int fd = fs_open(path, FS_O_RDONLY, 0);
	if (fd < 0) return 0;
	int64_t size = fs_lseek(fd, 0, FS_SEEK_END);
	fs_close(fd);
	return (size > 7) ? 1 : 0;
}

/* Get file size using fs_lseek */
static int64_t get_file_size(const char *path) {
	int fd = fs_open(path, FS_O_RDONLY, 0);
	if (fd < 0) return -1;
	int64_t size = fs_lseek(fd, 0, FS_SEEK_END);
	fs_close(fd);
	return size;
}

/*
 * Scan loaded game for CDDA tracks and their available versions
 * Returns number of audio tracks found
 */
int cdda_scan_tracks(void) {
	int i;
	cdda_conv_num_tracks = 0;

	if (!cdHandle || numtracks <= 0) {
		return 0;
	}

	for (i = 1; i <= numtracks; i++) {
		if (ti[i].type != CDDA) continue;
		if (ti[i].filepath[0] == '\0') continue;

		cdda_track_info_t *info = &cdda_conv_tracks[cdda_conv_num_tracks];
		info->track_num = i;
		strncpy(info->bin_path, ti[i].filepath, MAXPATHLEN - 1);
		info->bin_path[MAXPATHLEN - 1] = '\0';

		/* Check which versions exist */
		info->has_bin = file_exists(info->bin_path);

		/* Build .binwav path */
		char alt_path[MAXPATHLEN];
		strncpy(alt_path, info->bin_path, MAXPATHLEN - 12);
		char *ext = strrchr(alt_path, '.');
		if (ext) {
			strcpy(ext, ".binwav");
			info->has_binwav = file_exists(alt_path);

			strcpy(ext, ".binadpcm");
			info->has_binadpcm = file_exists(alt_path);
		} else {
			info->has_binwav = 0;
			info->has_binadpcm = 0;
		}

		/* Get sector count from .bin file */
		if (info->has_bin) {
			int64_t size = get_file_size(info->bin_path);
			info->bin_sectors = (size > 0) ? (unsigned int)(size / CD_FRAMESIZE_RAW) : 0;
		} else {
			info->bin_sectors = 0;
		}

		info->current_format = ti[i].cdda_format;
		cdda_conv_num_tracks++;
	}

	return cdda_conv_num_tracks;
}

/* Get track info by index (0-based) */
cdda_track_info_t* cdda_get_track_info(int idx) {
	if (idx < 0 || idx >= cdda_conv_num_tracks) return NULL;
	return &cdda_conv_tracks[idx];
}

int cdda_get_num_tracks(void) {
	return cdda_conv_num_tracks;
}

/*
 * Convert one track from .bin to .binwav (22kHz mono PCM)
 * Returns: 0=success, -1=error, -2=cancelled
 */
int cdda_convert_to_binwav(int track_idx) {
	if (track_idx < 0 || track_idx >= cdda_conv_num_tracks) return -1;

	cdda_track_info_t *info = &cdda_conv_tracks[track_idx];
	if (!info->has_bin) return -1;

	/* Build output path */
	char out_path[MAXPATHLEN];
	strncpy(out_path, info->bin_path, MAXPATHLEN - 12);
	char *ext = strrchr(out_path, '.');
	if (!ext) return -1;
	strcpy(ext, ".binwav");

	/* Open input file */
	int fd_in = fs_open(info->bin_path, FS_O_RDONLY, 0);
	if (fd_in < 0) return -1;

	/* Open output file */
	int fd_out = fs_open(out_path, FS_O_WRONLY | FS_O_CREAT | FS_O_TRUNC, 0666);
	if (fd_out < 0) {
		fs_close(fd_in);
		return -1;
	}

	/* Buffers */
	static unsigned char in_buf[CD_FRAMESIZE_RAW];
	static unsigned char out_buf[BINWAV_SECTOR_SIZE];

	unsigned int total_sectors = info->bin_sectors;
	unsigned int sector;

	for (sector = 0; sector < total_sectors; sector++) {
		/* Read one sector from .bin */
		ssize_t rd = fs_read(fd_in, in_buf, CD_FRAMESIZE_RAW);
		if (rd != CD_FRAMESIZE_RAW) {
			fs_close(fd_in);
			fs_close(fd_out);
			return -1;
		}

		/* Downsample 44.1kHz stereo -> 22kHz mono */
		int16_t *in_samples = (int16_t *)in_buf;
		int16_t *out_samples = (int16_t *)out_buf;

		for (int j = 0; j < 294; j++) {
			int src_idx = j * 4;  /* 4 samples (L1,R1,L2,R2) -> 1 mono sample */
			int l1 = in_samples[src_idx];
			int r1 = in_samples[src_idx + 1];
			int l2 = in_samples[src_idx + 2];
			int r2 = in_samples[src_idx + 3];
			int mono = (l1 + r1 + l2 + r2) / 4;
			if (mono > 32767) mono = 32767;
			if (mono < -32768) mono = -32768;
			out_samples[j] = (int16_t)mono;
		}

		/* Write output sector */
		ssize_t wr = fs_write(fd_out, out_buf, BINWAV_SECTOR_SIZE);
		if (wr != BINWAV_SECTOR_SIZE) {
			fs_close(fd_in);
			fs_close(fd_out);
			return -1;
		}

		/* Progress callback */
		if (cdda_progress_callback && (sector % 100 == 0 || sector == total_sectors - 1)) {
			int total_pct = (cdda_conv_num_tracks > 0) ?
				((track_idx * 100 + (sector * 100 / total_sectors)) / cdda_conv_num_tracks) : 0;
			cdda_progress_callback(track_idx, cdda_conv_num_tracks,
			                       sector, total_sectors, total_pct);
		}
	}

	fs_close(fd_in);
	fs_close(fd_out);
	fs_sync(out_path);

	info->has_binwav = 1;
	return 0;
}

/*
 * Convert one track from .bin to .binadpcm (22kHz mono IMA ADPCM)
 * Returns: 0=success, -1=error
 */
int cdda_convert_to_binadpcm(int track_idx) {
	if (track_idx < 0 || track_idx >= cdda_conv_num_tracks) return -1;

	cdda_track_info_t *info = &cdda_conv_tracks[track_idx];
	if (!info->has_bin) return -1;

	/* Build output path */
	char out_path[MAXPATHLEN];
	strncpy(out_path, info->bin_path, MAXPATHLEN - 12);
	char *ext = strrchr(out_path, '.');
	if (!ext) return -1;
	strcpy(ext, ".binadpcm");

	/* Open input file */
	int fd_in = fs_open(info->bin_path, FS_O_RDONLY, 0);
	if (fd_in < 0) return -1;

	/* Open output file */
	int fd_out = fs_open(out_path, FS_O_WRONLY | FS_O_CREAT | FS_O_TRUNC, 0666);
	if (fd_out < 0) {
		fs_close(fd_in);
		return -1;
	}

	/* Buffers */
	static unsigned char in_buf[CD_FRAMESIZE_RAW];
	static unsigned char out_buf[BINADPCM_SECTOR_SIZE];
	static int16_t sample_buf[294];

	unsigned int total_sectors = info->bin_sectors;
	unsigned int sector;

	/* ADPCM encoder state - reset at start of file */
	int16_t predictor = 0;
	int step_idx = 0;

	for (sector = 0; sector < total_sectors; sector++) {
		/* Read one sector from .bin */
		ssize_t rd = fs_read(fd_in, in_buf, CD_FRAMESIZE_RAW);
		if (rd != CD_FRAMESIZE_RAW) {
			fs_close(fd_in);
			fs_close(fd_out);
			return -1;
		}

		/* Downsample 44.1kHz stereo -> 22kHz mono into sample buffer */
		int16_t *in_samples = (int16_t *)in_buf;

		for (int j = 0; j < 294; j++) {
			int src_idx = j * 4;
			int l1 = in_samples[src_idx];
			int r1 = in_samples[src_idx + 1];
			int l2 = in_samples[src_idx + 2];
			int r2 = in_samples[src_idx + 3];
			int mono = (l1 + r1 + l2 + r2) / 4;
			if (mono > 32767) mono = 32767;
			if (mono < -32768) mono = -32768;
			sample_buf[j] = (int16_t)mono;
		}

		/* Write ADPCM header: predictor (int16), step_index (uint8), pad (uint8) */
		out_buf[0] = predictor & 0xFF;
		out_buf[1] = (predictor >> 8) & 0xFF;
		out_buf[2] = (unsigned char)step_idx;
		out_buf[3] = 0;  /* padding */

		/* Encode 294 samples to 147 bytes (2 nibbles per byte) */
		for (int j = 0; j < 147; j++) {
			int nibble1 = ima_encode_sample(sample_buf[j * 2], &predictor, &step_idx);
			int nibble2 = ima_encode_sample(sample_buf[j * 2 + 1], &predictor, &step_idx);
			out_buf[4 + j] = (unsigned char)((nibble2 << 4) | nibble1);
		}

		/* Padding byte */
		out_buf[151] = 0;

		/* Write output sector */
		ssize_t wr = fs_write(fd_out, out_buf, BINADPCM_SECTOR_SIZE);
		if (wr != BINADPCM_SECTOR_SIZE) {
			fs_close(fd_in);
			fs_close(fd_out);
			return -1;
		}

		/* Progress callback */
		if (cdda_progress_callback && (sector % 100 == 0 || sector == total_sectors - 1)) {
			int total_pct = (cdda_conv_num_tracks > 0) ?
				((track_idx * 100 + (sector * 100 / total_sectors)) / cdda_conv_num_tracks) : 0;
			cdda_progress_callback(track_idx, cdda_conv_num_tracks,
			                       sector, total_sectors, total_pct);
		}
	}

	fs_close(fd_in);
	fs_close(fd_out);
	fs_sync(out_path);

	info->has_binadpcm = 1;
	return 0;
}

/*
 * Delete a track version (truncate to 0 bytes - no fs_unlink in firmware)
 * format: CDDA_FMT_BIN, CDDA_FMT_BINWAV, or CDDA_FMT_BINADPCM
 * Returns: 0=success, -1=error, -2=last version (can't delete)
 */
int cdda_delete_version(int track_idx, int format) {
	if (track_idx < 0 || track_idx >= cdda_conv_num_tracks) return -1;

	cdda_track_info_t *info = &cdda_conv_tracks[track_idx];

	/* Count how many versions exist */
	int version_count = 0;
	if (info->has_bin) version_count++;
	if (info->has_binwav) version_count++;
	if (info->has_binadpcm) version_count++;

	/* Can't delete the last remaining version */
	if (version_count <= 1) return -2;

	/* Build path for the version to delete */
	char del_path[MAXPATHLEN];
	strncpy(del_path, info->bin_path, MAXPATHLEN - 12);
	char *ext = strrchr(del_path, '.');
	if (!ext) return -1;

	switch (format) {
		case CDDA_FMT_BIN:
			if (!info->has_bin) return -1;
			/* Keep .bin extension */
			break;
		case CDDA_FMT_BINWAV:
			if (!info->has_binwav) return -1;
			strcpy(ext, ".binwav");
			break;
		case CDDA_FMT_BINADPCM:
			if (!info->has_binadpcm) return -1;
			strcpy(ext, ".binadpcm");
			break;
		default:
			return -1;
	}

	/* "Delete" by truncating to 0 bytes
	 * v265: fopen("w") truncates, fflush forces write, file becomes 0 bytes
	 * If filesystem needs actual write, we write single null byte (≤7 = deleted)
	 */
	FILE *fp = fopen(del_path, "w");
	if (!fp) return -1;
	fputc('X', fp);  /* Write single byte to force truncation */
	fflush(fp);
	fclose(fp);
	fs_sync(del_path);

	/* Update track info */
	switch (format) {
		case CDDA_FMT_BIN: info->has_bin = 0; break;
		case CDDA_FMT_BINWAV: info->has_binwav = 0; break;
		case CDDA_FMT_BINADPCM: info->has_binadpcm = 0; break;
	}

	return 0;
}

/*
 * v261: Incremental conversion API
 * Call cdda_start_convert_track() once, then cdda_convert_step() repeatedly
 * until it returns 0 (done) or negative (error/cancel)
 */

/* Start converting a track - opens files, initializes state */
int cdda_start_convert_track(int track_idx, int format) {
	if (cdda_incr_active) return -1;  /* Already converting */
	if (track_idx < 0 || track_idx >= cdda_conv_num_tracks) return -1;
	if (format != 1 && format != 2) return -1;  /* 1=binwav, 2=binadpcm */

	cdda_track_info_t *info = &cdda_conv_tracks[track_idx];
	if (!info->has_bin) return -1;

	/* Build output path */
	strncpy(cdda_incr_out_path, info->bin_path, sizeof(cdda_incr_out_path) - 12);
	char *ext = strrchr(cdda_incr_out_path, '.');
	if (!ext) return -1;
	strcpy(ext, (format == 1) ? ".binwav" : ".binadpcm");

	/* Open input file */
	cdda_incr_fd_in = fs_open(info->bin_path, FS_O_RDONLY, 0);
	if (cdda_incr_fd_in < 0) return -1;

	/* Open output file */
	cdda_incr_fd_out = fs_open(cdda_incr_out_path, FS_O_WRONLY | FS_O_CREAT | FS_O_TRUNC, 0666);
	if (cdda_incr_fd_out < 0) {
		fs_close(cdda_incr_fd_in);
		cdda_incr_fd_in = -1;
		return -1;
	}

	/* Initialize state */
	cdda_incr_active = 1;
	cdda_incr_format = format;
	cdda_incr_track_idx = track_idx;
	cdda_incr_sector = 0;
	cdda_incr_total_sectors = info->bin_sectors;
	cdda_incr_predictor = 0;
	cdda_incr_step_idx = 0;
	cdda_cancel_flag = 0;

	return 0;
}

/*
 * Convert next batch of sectors
 * Returns: 1 = more work to do, 0 = done, -1 = error, -2 = cancelled
 */
int cdda_convert_step(void) {
	if (!cdda_incr_active) return -1;

	/* Check cancel flag */
	if (cdda_cancel_flag) {
		/* Close files and delete incomplete output */
		fs_close(cdda_incr_fd_in);
		fs_close(cdda_incr_fd_out);
		cdda_incr_fd_in = -1;
		cdda_incr_fd_out = -1;
		
		/* v270: "Delete" by writing single byte (<=7 bytes = deleted)
		 * MUST write data - SF2000 needs actual write to truncate file
		 */
		FILE *fp = fopen(cdda_incr_out_path, "w");
		if (fp) {
			fputc('X', fp);  /* Write single byte to force truncation */
			fflush(fp);
			fclose(fp);
		}
		fs_sync(cdda_incr_out_path);
		
		cdda_incr_active = 0;
		cdda_cancel_flag = 0;
		return -2;
	}


	/* Buffers */
	static unsigned char in_buf[CD_FRAMESIZE_RAW];
	static unsigned char out_buf[BINADPCM_SECTOR_SIZE > BINWAV_SECTOR_SIZE ? BINADPCM_SECTOR_SIZE : BINWAV_SECTOR_SIZE];
	static int16_t sample_buf[294];

	int sectors_this_step = 0;

	while (cdda_incr_sector < cdda_incr_total_sectors && sectors_this_step < CDDA_SECTORS_PER_STEP) {
		/* Read one sector from .bin */
		ssize_t rd = fs_read(cdda_incr_fd_in, in_buf, CD_FRAMESIZE_RAW);
		if (rd != CD_FRAMESIZE_RAW) {
			fs_close(cdda_incr_fd_in);
			fs_close(cdda_incr_fd_out);
			cdda_incr_fd_in = -1;
			cdda_incr_fd_out = -1;
			cdda_incr_active = 0;
			return -1;
		}

		int16_t *in_samples = (int16_t *)in_buf;
		ssize_t wr;

		if (cdda_incr_format == 1) {
			/* binwav: Downsample to 22kHz mono PCM */
			int16_t *out_samples = (int16_t *)out_buf;
			for (int j = 0; j < 294; j++) {
				int src_idx = j * 4;
				int l1 = in_samples[src_idx];
				int r1 = in_samples[src_idx + 1];
				int l2 = in_samples[src_idx + 2];
				int r2 = in_samples[src_idx + 3];
				int mono = (l1 + r1 + l2 + r2) / 4;
				if (mono > 32767) mono = 32767;
				if (mono < -32768) mono = -32768;
				out_samples[j] = (int16_t)mono;
			}
			wr = fs_write(cdda_incr_fd_out, out_buf, BINWAV_SECTOR_SIZE);
			if (wr != BINWAV_SECTOR_SIZE) {
				fs_close(cdda_incr_fd_in);
				fs_close(cdda_incr_fd_out);
				cdda_incr_fd_in = -1;
				cdda_incr_fd_out = -1;
				cdda_incr_active = 0;
				return -1;
			}
		} else {
			/* binadpcm: Downsample and encode ADPCM */
			for (int j = 0; j < 294; j++) {
				int src_idx = j * 4;
				int l1 = in_samples[src_idx];
				int r1 = in_samples[src_idx + 1];
				int l2 = in_samples[src_idx + 2];
				int r2 = in_samples[src_idx + 3];
				int mono = (l1 + r1 + l2 + r2) / 4;
				if (mono > 32767) mono = 32767;
				if (mono < -32768) mono = -32768;
				sample_buf[j] = (int16_t)mono;
			}

			/* ADPCM header */
			out_buf[0] = cdda_incr_predictor & 0xFF;
			out_buf[1] = (cdda_incr_predictor >> 8) & 0xFF;
			out_buf[2] = (unsigned char)cdda_incr_step_idx;
			out_buf[3] = 0;

			/* Encode 294 samples */
			for (int j = 0; j < 147; j++) {
				int nibble1 = ima_encode_sample(sample_buf[j * 2], &cdda_incr_predictor, &cdda_incr_step_idx);
				int nibble2 = ima_encode_sample(sample_buf[j * 2 + 1], &cdda_incr_predictor, &cdda_incr_step_idx);
				out_buf[4 + j] = (unsigned char)((nibble2 << 4) | nibble1);
			}
			out_buf[151] = 0;

			wr = fs_write(cdda_incr_fd_out, out_buf, BINADPCM_SECTOR_SIZE);
			if (wr != BINADPCM_SECTOR_SIZE) {
				fs_close(cdda_incr_fd_in);
				fs_close(cdda_incr_fd_out);
				cdda_incr_fd_in = -1;
				cdda_incr_fd_out = -1;
				cdda_incr_active = 0;
				return -1;
			}
		}

		cdda_incr_sector++;
		sectors_this_step++;
	}

	/* Call progress callback */
	if (cdda_progress_callback) {
		int pct = (cdda_incr_total_sectors > 0) ?
			(cdda_incr_sector * 100 / cdda_incr_total_sectors) : 0;
		cdda_progress_callback(cdda_incr_track_idx, cdda_conv_num_tracks,
		                       cdda_incr_sector, cdda_incr_total_sectors, pct);
	}

	/* Check if done with this track */
	if (cdda_incr_sector >= cdda_incr_total_sectors) {
		fs_close(cdda_incr_fd_in);
		fs_close(cdda_incr_fd_out);
		fs_sync(cdda_incr_out_path);
		cdda_incr_fd_in = -1;
		cdda_incr_fd_out = -1;

		/* Update track info */
		cdda_track_info_t *info = &cdda_conv_tracks[cdda_incr_track_idx];
		if (cdda_incr_format == 1) {
			info->has_binwav = 1;
		} else {
			info->has_binadpcm = 1;
		}

		cdda_incr_active = 0;
		return 0;  /* Done with this track */
	}

	return 1;  /* More work to do */
}

/* Get current conversion progress */
void cdda_get_convert_progress(int *sector, int *total_sectors, int *track_idx) {
	if (sector) *sector = cdda_incr_sector;
	if (total_sectors) *total_sectors = cdda_incr_total_sectors;
	if (track_idx) *track_idx = cdda_incr_track_idx;
}
