/*
 * PCSX4ALL libretro port header
 * QPSX_001 - SF2000 PSX port
 */

#ifndef __PSXPORT_H__
#define __PSXPORT_H__

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <ctype.h>
#include <sys/types.h>
#include <assert.h>

/* SF2000 lacks these */
#ifdef SF2000
static inline int fsync(int f) { (void)f; return 0; }
/* usleep - use busy wait on bare metal (microseconds) */
static inline int usleep(unsigned usec) {
    /* On SF2000, libretro handles frame pacing via retro_run() cadence.
     * This is a no-op stub - the frontend manages vsync timing. */
    (void)usec;
    return 0;
}
#endif

#define	CONFIG_VERSION	0

/* Timing */
unsigned get_ticks(void);
void wait_ticks(unsigned s);

/* Input */
void pad_update(void);
unsigned short pad_read(int num);

/* Video */
void video_flip(void);
void video_clear(void);
void port_printf(int x, int y, const char *text);

/* Framebuffer pointer - 320x240 RGB565 */
extern unsigned short *SCREEN;

/* Save states (stubs for now) */
int state_load(int slot);
int state_save(int slot);

/* Menu (stubs - no menu in libretro) */
int SelectGame(void);
int GameMenu(void);

/* Libretro audio callback */
#ifdef __cplusplus
extern "C" {
#endif
extern void retro_audio_cb(int16_t *buf, int samples);
#ifdef __cplusplus
}
#endif

#endif /* __PSXPORT_H__ */
