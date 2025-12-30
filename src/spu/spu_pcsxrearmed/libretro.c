/*
 * PCSX4ALL libretro audio output driver
 * QPSX_001 - SF2000 PSX port
 */

#include "out.h"
#include <stdint.h>

/* Audio callback provided by libretro-core.cpp */
extern void retro_audio_cb(int16_t *buf, int samples);

static int libretro_init(void)
{
    return 0;
}

static void libretro_finish(void)
{
}

static int libretro_busy(void)
{
    /* Never busy - libretro handles buffering */
    return 0;
}

static void libretro_feed(void *data, int bytes)
{
    /* data is int16_t stereo samples */
    /* bytes / 4 = number of sample pairs */
    int samples = bytes / 4;
    retro_audio_cb((int16_t *)data, samples);
}

void out_register_libretro(struct out_driver *drv)
{
    drv->name = "libretro";
    drv->init = libretro_init;
    drv->finish = libretro_finish;
    drv->busy = libretro_busy;
    drv->feed = libretro_feed;
}
