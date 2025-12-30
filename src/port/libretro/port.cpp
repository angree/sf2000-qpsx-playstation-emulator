/*
 * PCSX4ALL libretro port implementation
 * QPSX_075 - EXTENDED REGISTER ALLOCATOR + FILE LOGGING
 *
 * CRITICAL: This file does NOT call any SF2000 input functions!
 * All input is read from the cache set by update_input_cache() in libretro-core.cpp.
 *
 * v075: Debug logs disabled by default, use menu option to enable file logging.
 */

#include "port.h"
#include "libretro.h"
#include <string.h>
#include <stdio.h>

/*
 * Debug logging - uses the file logging system from libretro-core.cpp
 * When debug_log is enabled, writes to /mnt/sda1/log.txt
 */
extern "C" void port_debug_log(const char *fmt, ...);

/* Debug counter */
static int port_debug_counter = 0;
#define PORT_DEBUG_INTERVAL 60  /* Log every 60 frames */

#define SCREEN_WIDTH  320
#define SCREEN_HEIGHT 240

static uint16_t static_screen_buffer[SCREEN_WIDTH * SCREEN_HEIGHT];

unsigned short *SCREEN = static_screen_buffer;

extern retro_video_refresh_t video_cb;
extern retro_audio_sample_batch_t audio_batch_cb;

extern volatile int skip_video_output;

static unsigned tick_counter = 0;

unsigned get_ticks(void) { return tick_counter++; }
void wait_ticks(unsigned s) { (void)s; }

static uint16_t libretro_pad1 = 0xFFFF;
static uint16_t libretro_pad2 = 0xFFFF;

extern "C" void set_pad_state(int num, uint16_t state)
{
    if (num == 0) libretro_pad1 = state;
    else libretro_pad2 = state;
}

/* Import the cached input getter from libretro-core.cpp */
extern "C" uint16_t get_cached_pad(int num);

void pad_update(void)
{
    /*
     * QPSX_073 DEBUG: This is called by EmuUpdate() at VBlank.
     * We copy the cached input (from retro_run) to local state.
     * The game then reads it via pad_read().
     *
     * DO NOT call any SF2000 input functions here!
     */
    uint16_t old_pad1 = libretro_pad1;

    libretro_pad1 = get_cached_pad(0);
    libretro_pad2 = get_cached_pad(1);

    port_debug_counter++;

    /* DEBUG: Log when buttons are pressed during pad_update (only when debug_log enabled) */
    if (libretro_pad1 != 0xFFFF && (port_debug_counter % PORT_DEBUG_INTERVAL) == 0) {
        port_debug_log("[PAD-UPDATE] pad1=0x%04X old=0x%04X (VBlank copy)", libretro_pad1, old_pad1);
    }
}

unsigned short pad_read(int num)
{
    uint16_t val = (num == 0) ? libretro_pad1 : libretro_pad2;

    /* DEBUG: Log when game actually reads controller with buttons pressed (only when debug_log enabled) */
    if (val != 0xFFFF && (port_debug_counter % PORT_DEBUG_INTERVAL) == 0) {
        port_debug_log("[PAD-READ] num=%d val=0x%04X (game SIO read)", num, val);
    }

    return val;
}

void video_flip(void)
{
    if (skip_video_output) return;
    if (video_cb && SCREEN) video_cb(SCREEN, SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_WIDTH * 2);
}

void video_clear(void) { memset(SCREEN, 0, SCREEN_WIDTH * SCREEN_HEIGHT * 2); }
void sound_set(int frequency) { (void)frequency; }
void sound_close(void) { }
void sound_send(void) { }
