/*
 * PCSX4ALL libretro core for SF2000
 * QPSX - version defined by QPSX_VERSION macro below
 *
 * v088 changes:
 * - NEW: UNR Divide option (GTE perspective division optimization)
 *   * Uses Newton-Raphson approximation with 257-entry lookup table
 *   * May be faster than hardware DIV (~36 cycles) on cache-friendly code
 *   * Enable via menu: DYNAREC OPT -> UNR Divide
 * - NEW: NoFlags option (skip GTE overflow flag checking) - RISK!
 *   * Skips overflow flag computation in BOUNDS/LIM macros
 *   * Can break games that read GTE flags - use with caution!
 *   * Enable via menu: DYNAREC OPT -> !NoFlags!
 *
 * v087: RTPT loop unroll optimization (optional, OFF by default)
 * v086: CDDA batch reading (8 sectors at once)
 * v085: CDDA audio fix (synchronous feeding)
 * v084: Auto-menu invisible, 3 dynarec opts ON by default
 * v083: FPS fix - qpsx_apply_config() on auto-menu close
 * v082: Menu version fix, SPU desync workaround
 *
 * Based on v081, v080, v079, v078, v077, v076 (framework)
 */

#include "libretro.h"
#include "port.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <sys/time.h>

/* SF2000 xlog */
#ifdef SF2000
extern "C" {
    extern void xlog(const char *fmt, ...);
    extern void xlog_clear(void);
}
#endif

/*
 * QPSX_075: Debug Logging System
 *
 * Debug logs disabled by default (were causing slowdown)
 * When enabled via menu, writes to /mnt/sda1/log.txt
 */
static FILE *debug_log_file = NULL;
static const char *DEBUG_LOG_PATH = "/mnt/sda1/log.txt";

/* Global flag for debug logging (set from qpsx_config.debug_log) */
static int g_debug_log_enabled = 0;

static void debug_log_write(const char *prefix, const char *fmt, ...)
{
    if (!g_debug_log_enabled) return;

    /* Open log file if not already open */
    if (!debug_log_file) {
        debug_log_file = fopen(DEBUG_LOG_PATH, "a");
        if (!debug_log_file) return;  /* Can't open, silently fail */
    }

    /* Write prefix */
    fprintf(debug_log_file, "%s", prefix);

    /* Write formatted message */
    va_list args;
    va_start(args, fmt);
    vfprintf(debug_log_file, fmt, args);
    va_end(args);

    fprintf(debug_log_file, "\n");
    fflush(debug_log_file);
}

/* Called when debug_log option changes */
static void update_debug_log_state(int enabled)
{
    g_debug_log_enabled = enabled;
    if (!enabled && debug_log_file) {
        fclose(debug_log_file);
        debug_log_file = NULL;
    }
}

/* Always log critical messages (startup, errors) */
#ifdef SF2000
#define XLOG(fmt, ...) xlog("QPSX: " fmt "\n", ##__VA_ARGS__)
#else
#define XLOG(fmt, ...) printf("QPSX: " fmt "\n", ##__VA_ARGS__)
#endif

/* Debug logs - only when enabled, write to file */
#define DEBUG_LOG(fmt, ...) debug_log_write("QPSX: ", fmt, ##__VA_ARGS__)

/* Export debug log function for port.cpp */
extern "C" void port_debug_log(const char *fmt, ...)
{
    if (!g_debug_log_enabled) return;

    if (!debug_log_file) {
        debug_log_file = fopen(DEBUG_LOG_PATH, "a");
        if (!debug_log_file) return;
    }

    fprintf(debug_log_file, "PORT: ");

    va_list args;
    va_start(args, fmt);
    vfprintf(debug_log_file, fmt, args);
    va_end(args);

    fprintf(debug_log_file, "\n");
    fflush(debug_log_file);
}

/* PSX includes */
#include "psxcommon.h"
#include "r3000a.h"
#include "plugins.h"
#include "cdrom.h"
#include "cdriso.h"
#include "misc.h"

/* GPU includes */
#include "gpu/gpulib/gpu.h"
#include "gpu/gpu_unai/gpu.h"

/* SPU includes - needed to access spu state for audio resync */
#ifdef SPU_PCSXREARMED
#include "spu/spu_pcsxrearmed/spu_config.h"
#include "spu/spu_pcsxrearmed/externals.h"
#endif

#define SCREEN_WIDTH  320
#define SCREEN_HEIGHT 240
#define VIRTUAL_WIDTH 320

/* Libretro callbacks */
static retro_video_refresh_t real_video_cb;
retro_video_refresh_t video_cb;
retro_audio_sample_t audio_cb;
retro_audio_sample_batch_t audio_batch_cb;
retro_environment_t environ_cb;

/* Input callbacks */
retro_input_state_t input_state_cb;
retro_input_poll_t input_poll_cb;

/* ============== INPUT CACHING ============== */
/*
 * CRITICAL: SF2000 input hardware has minimum polling interval.
 * Calling input_state_cb too frequently returns 0 or garbage.
 * We read all buttons ONCE per retro_run() and cache the result.
 * pad_update() in port.cpp reads from this cache instead of calling
 * input_state_cb directly.
 *
 * PSX controller bit format (active low - 0=pressed):
 * Bit 0: SELECT, 1: L3, 2: R3, 3: START
 * Bit 4: UP, 5: RIGHT, 6: DOWN, 7: LEFT
 * Bit 8: L2, 9: R2, 10: L1, 11: R1
 * Bit 12: TRIANGLE, 13: CIRCLE, 14: CROSS, 15: SQUARE
 */
static uint16_t cached_pad1 = 0xFFFF;
static uint16_t cached_pad2 = 0xFFFF;

/* Debug logging for input path comparison - must be before functions that use it */
static int input_debug_counter = 0;
#define INPUT_DEBUG_INTERVAL 60  /* Log every 60 frames (1 second at 60fps) */

static void update_input_cache(void)
{
    uint16_t pad1 = 0xFFFF;  /* All buttons released (active low) */

    if (input_poll_cb) input_poll_cb();

    if (input_state_cb) {
        /* Map libretro buttons to PSX controller bits */
        if (input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_SELECT))   pad1 &= ~(1 << 0);
        if (input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_L3))       pad1 &= ~(1 << 1);
        if (input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_R3))       pad1 &= ~(1 << 2);
        if (input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_START))    pad1 &= ~(1 << 3);
        if (input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_UP))       pad1 &= ~(1 << 4);
        if (input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_RIGHT))    pad1 &= ~(1 << 5);
        if (input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_DOWN))     pad1 &= ~(1 << 6);
        if (input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_LEFT))     pad1 &= ~(1 << 7);
        if (input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_L2))       pad1 &= ~(1 << 8);
        if (input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_R2))       pad1 &= ~(1 << 9);
        if (input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_L))        pad1 &= ~(1 << 10);
        if (input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_R))        pad1 &= ~(1 << 11);
        if (input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_X))        pad1 &= ~(1 << 12);
        if (input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_A))        pad1 &= ~(1 << 13);
        if (input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_B))        pad1 &= ~(1 << 14);
        if (input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_Y))        pad1 &= ~(1 << 15);
    }

    /* DEBUG: Log when buttons are pressed (only when debug_log enabled) */
    if (pad1 != 0xFFFF && (input_debug_counter % INPUT_DEBUG_INTERVAL) == 0) {
        DEBUG_LOG("[INPUT-CACHE] pad1=0x%04X (buttons pressed)", pad1);
    }

    cached_pad1 = pad1;
    cached_pad2 = 0xFFFF;  /* Player 2 not used */
}

/* Called by port.cpp pad_update() - NEVER calls SF2000 input functions */
extern "C" uint16_t get_cached_pad(int num)
{
    uint16_t val = (num == 0) ? cached_pad1 : cached_pad2;

    /* DEBUG: Log when game reads input with buttons pressed (only when debug_log enabled) */
    if (val != 0xFFFF && (input_debug_counter % INPUT_DEBUG_INTERVAL) == 0) {
        DEBUG_LOG("[GET-CACHED] num=%d val=0x%04X (game reads)", num, val);
    }

    return val;
}

/* Paths */
static const char *retro_system_directory;
static const char *retro_save_directory;
static const char *retro_content_directory;

static char game_path[512];
static bool psx_initted = false;

volatile int skip_video_output = 0;

/* Framebuffer access */
extern uint16_t *SCREEN;

/* Audio output */
extern "C" void retro_audio_cb(int16_t *buf, int samples)
{
    if (audio_batch_cb) {
        audio_batch_cb(buf, samples);
    }
}

#define QPSX_VERSION "088"
#define QPSX_GLOBAL_CONFIG_PATH "/mnt/sda1/cores/config/pcsx4all.cfg"

#ifdef PSXREC
extern u32 cycle_multiplier;
#endif

/* ============== FPS COUNTER ============== */
static int fps_show = 0;
static int fps_current = 0;
static int fps_frame_count = 0;
static unsigned long fps_last_time = 0;

static unsigned long get_time_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}

/* ============== RGB565 COLOR MACROS ============== */
#define RGB565(r,g,b) (((r>>3)<<11)|((g>>2)<<5)|(b>>3))

/* Menu colors */
#define MENU_BG      RGB565(0, 0, 48)
#define MENU_FG      RGB565(255, 255, 255)
#define MENU_SEL     RGB565(255, 255, 0)
#define MENU_TITLE   RGB565(0, 255, 255)
#define MENU_AUTHOR  RGB565(160, 160, 160)
#define MENU_BORDER  RGB565(100, 100, 140)
#define MENU_SEP     RGB565(80, 80, 120)
#define MENU_DIM     RGB565(100, 100, 100)
#define MENU_GREEN   RGB565(0, 255, 0)
#define MENU_RED     RGB565(255, 80, 80)
#define MENU_INSTANT RGB565(100, 255, 100)
#define MENU_RESTART RGB565(255, 180, 80)
#define MENU_PRESET  RGB565(180, 140, 255)

/* FPS counter colors */
#define FPS_BG       RGB565(0, 0, 0)
#define FPS_GOOD     RGB565(0, 255, 0)
#define FPS_OK       RGB565(255, 255, 0)
#define FPS_BAD      RGB565(255, 0, 0)

/* Menu dimensions */
#define MENU_X      15
#define MENU_Y      10
#define MENU_W      290
#define MENU_H      220
#define MENU_LINE_H 10
#define MENU_VISIBLE 17

/* ============== MENU STATE ============== */
static int menu_active = 0;
static int menu_was_active = 0;
static int menu_item = 0;
static int menu_scroll = 0;
static int menu_first_frame = 0;
static int menu_entry_delay = 0;

/* 1.5-second START hold detection */
static int start_hold_frames = 0;
#define MENU_HOLD_FRAMES 90

/* Feedback message */
static char feedback_msg[64] = "";
static int feedback_timer = 0;

/* ============== CONFIGURATION STRUCTURE ============== */
static struct {
    int frameskip;        /* 0-5, use gpu.frameskip.set */
    int use_bios;
    char bios_file[64];
    int cycle_mult;       /* 256-2048, step 128 */

    /* SPU */
    int spu_update_freq;
    int spu_reverb;
    int spu_interpolation;
    int xa_update_freq;
    int spu_tempo;
    int spu_irq_always;

    /* Audio disable */
    int xa_disable;
    int cdda_disable;

    /* GPU */
    int gpu_dithering;
    int gpu_blending;
    int gpu_lighting;
    int gpu_fast_lighting;
    int gpu_pixel_skip;
    int gpu_interlace_force;
    int gpu_prog_ilace;
    int frame_duping;

    /* Dynarec hacks */
    int nosmccheck;
    int nogteflags;
    int gteregsunneeded;

    /* Dynarec optimizations (experimental) */
    int opt_lohi_cache;     /* LO/HI register caching */
    int opt_nclip_inline;   /* NCLIP GTE inline */
    int opt_const_prop;     /* Extended constant propagation */
    int opt_rtpt_unroll;    /* RTPT loop unroll optimization (v087) */

    /* GTE optimizations (v088) - EXPERIMENTAL! */
    int opt_gte_unrdiv;     /* UNR divide instead of hardware DIV */
    int opt_gte_noflags;    /* Skip overflow flag checking - RISK! Can break some games! */

    /* Display */
    int show_fps;

    /* Debug */
    int debug_log;        /* 0=off, 1=write to /mnt/sda1/log.txt */

} qpsx_config = {
    /* Defaults = v071 optimal */
    0,                  /* frameskip = 0 (disabled - causes issues) */
    1,                  /* use_bios = 1 (real BIOS) */
    "scph5501.bin",     /* bios_file */
    1536,               /* cycle_mult = 1536 (v073 fast default) */

    /* SPU */
    0, 0, 0, 0, 0, 0,

    /* Audio */
    0, 0,               /* xa_disable, cdda_disable = 0 */

    /* GPU */
    0,                  /* gpu_dithering = 0 */
    1,                  /* gpu_blending = 1 */
    1,                  /* gpu_lighting = 1 */
    0,                  /* gpu_fast_lighting = 0 (OFF - experimental, v081) */
    1,                  /* gpu_pixel_skip = 1 */
    0,                  /* gpu_interlace_force = 0 */
    0,                  /* gpu_prog_ilace = 0 */
    0,                  /* frame_duping = 0 */

    /* Dynarec */
    0, 0, 0,            /* nosmccheck, nogteflags, gteregsunneeded = 0 */

    /* Dynarec optimizations - v084: first 3 ON, v087: rtpt_unroll OFF (experimental) */
    1, 1, 1, 0,         /* opt_lohi_cache, opt_nclip_inline, opt_const_prop, opt_rtpt_unroll=OFF */

    /* GTE optimizations - v088: both OFF by default (experimental, can break games!) */
    0, 0,               /* opt_gte_unrdiv=OFF, opt_gte_noflags=OFF */

    /* Display */
    0,                  /* show_fps = 0 (off by default) */

    /* Debug */
    0                   /* debug_log = 0 (off by default) */
};

int qpsx_nosmccheck = 0;

/* Dynarec optimization flags - exported for use in recompiler and GTE */
int g_opt_lohi_cache = 0;     /* LO/HI register caching */
int g_opt_nclip_inline = 0;   /* NCLIP GTE operation inline */
int g_opt_const_prop = 0;     /* Extended constant propagation */
int g_opt_rtpt_unroll = 0;    /* RTPT loop unroll (v087) */

/* GTE optimization flags (v088) - EXPERIMENTAL */
int g_opt_gte_unrdiv = 0;     /* UNR (Newton-Raphson) divide instead of hardware DIV */
int g_opt_gte_noflags = 0;    /* Skip overflow flag checking - RISK! Can break some games! */

/* ============== MENU ITEMS ============== */
typedef struct {
    const char *name;
    int type;           /* 0=option, 1=separator, 2=action, 3=preset */
    int restart;        /* 1=needs restart, 0=instant apply */
} MenuItem;

#define MENU_ITEMS 45
enum {
    MI_SEP_HEADER = 0,
    MI_FRAMESKIP,
    MI_BIOS,
    MI_CYCLE,
    MI_SHOW_FPS,
    MI_DEBUG_LOG,
    MI_SEP_AUDIO,
    MI_XA_AUDIO,
    MI_CDDA_AUDIO,
    MI_SEP_GPU,
    MI_DITHERING,
    MI_BLENDING,
    MI_LIGHTING,
    MI_FAST_LIGHT,
    MI_PIXEL_SKIP,
    MI_INTERLACE,
    MI_PROG_ILACE,
    MI_FRAME_DUPE,
    MI_SEP_HACKS,
    MI_SMC_CHECK,
    MI_SEP_DYNAREC,
    MI_OPT_LOHI,
    MI_OPT_NCLIP,
    MI_OPT_CONST,
    MI_OPT_RTPT,
    MI_OPT_UNRDIV,      /* v088: UNR divide */
    MI_OPT_NOFLAGS,     /* v088: No GTE flags - RISK! */
    MI_SEP_PRESETS,
    MI_PRESET_COMPAT,
    MI_PRESET_BALANCED,
    MI_PRESET_FAST,
    MI_PRESET_FASTER,
    MI_PRESET_MAXSPEED,
    MI_RESET_DEFAULTS,
    MI_SEP_CONFIG,
    MI_SAVE_GAME,
    MI_SAVE_GLOBAL,
    MI_DEL_GAME,
    MI_DEL_GLOBAL,
    MI_SEP_EXIT,
    MI_EXIT,
    MI_SEP_FOOTER,
    MI_DUMMY,
    MI_DUMMY2
};

/*
 * QPSX_080: Fixed restart indicators
 * restart=0 means instant apply, restart=1 means needs restart
 *
 * Instant apply (restart=0):
 *   - Frameskip (applies via gpu.frameskip.set)
 *   - Show FPS (just display toggle)
 *   - Debug Log (file handle toggle)
 *
 * Requires restart (restart=1):
 *   - ALL other options (BIOS, Cycles, Audio, GPU, Hacks, Dynarec)
 */
static const MenuItem menu_items[MENU_ITEMS] = {
    {"--- BASIC ---",       1, 0},
    {"Frameskip",           0, 0},  /* Instant - gpu.frameskip.set */
    {"BIOS",                0, 1},  /* Restart - needs reinit */
    {"CPU Cycle",           0, 1},  /* Restart - dynarec timing */
    {"Show FPS",            0, 0},  /* Instant - display only */
    {"Debug Log",           0, 0},  /* Instant - file toggle */
    {"--- AUDIO ---",       1, 0},
    {"XA Audio",            0, 1},  /* Restart - SPU init */
    {"CD-DA",               0, 1},  /* Restart - CD init */
    {"--- GPU ---",         1, 0},
    {"Dithering",           0, 1},  /* Restart - GPU template */
    {"Blending",            0, 1},  /* Restart - GPU template */
    {"Lighting",            0, 1},  /* Restart - GPU template */
    {"Fast Light",          0, 1},  /* Restart - GPU template */
    {"Pixel Skip",          0, 1},  /* Restart - GPU template */
    {"Interlace",           0, 1},  /* Restart - GPU mode */
    {"Prog Ilace",          0, 1},  /* Restart - GPU mode */
    {"Frame Dupe",          0, 1},  /* Restart - GPU mode */
    {"--- HACKS ---",       1, 0},
    {"SMC Check",           0, 1},  /* Restart - dynarec flag */
    {"--- DYNAREC OPT ---", 1, 0},
    {"LO/HI Cache",         0, 1},  /* Restart - recompiler */
    {"NCLIP Inline",        0, 1},  /* Restart - recompiler */
    {"ConstProp+",          0, 1},  /* Restart - recompiler */
    {"RTPT Unroll",         0, 1},  /* Restart - GTE (v087) */
    {"UNR Divide",          0, 1},  /* Restart - GTE (v088) - fast div */
    {"!NoFlags!",           0, 1},  /* Restart - GTE (v088) - RISK! */
    {"--- PRESETS ---",     1, 0},
    {"MAX COMPAT",          3, 0},
    {"BALANCED",            3, 0},
    {"FAST",                3, 0},
    {"FASTER",              3, 0},
    {"MAX SPEED",           3, 0},
    {"RESET DEFAULTS",      3, 0},
    {"--- CONFIG ---",      1, 0},
    {"SAVE GAME CFG",       2, 0},
    {"SAVE GLOBAL CFG",     2, 0},
    {"DEL GAME CFG",        2, 0},
    {"DEL GLOBAL CFG",      2, 0},
    {"--------------",      1, 0},
    {"EXIT MENU",           2, 0},
    {"[*]=instant [R]=restart", 1, 0},
    {"",                    1, 0},
    {"",                    1, 0}
};

/* ============== DRAWING FUNCTIONS ============== */

static void DrawFBox(uint16_t *buffer, int x, int y, int w, int h, uint16_t color)
{
    for (int j = y; j < y + h && j < SCREEN_HEIGHT; j++) {
        for (int i = x; i < x + w && i < SCREEN_WIDTH; i++) {
            if (i >= 0 && j >= 0) {
                buffer[i + j * VIRTUAL_WIDTH] = color;
            }
        }
    }
}

static void DrawBox(uint16_t *buffer, int x, int y, int w, int h, uint16_t color)
{
    for (int i = x; i < x + w && i < SCREEN_WIDTH; i++) {
        if (i >= 0) {
            if (y >= 0 && y < SCREEN_HEIGHT) buffer[i + y * VIRTUAL_WIDTH] = color;
            if (y + h - 1 >= 0 && y + h - 1 < SCREEN_HEIGHT) buffer[i + (y + h - 1) * VIRTUAL_WIDTH] = color;
        }
    }
    for (int j = y; j < y + h && j < SCREEN_HEIGHT; j++) {
        if (j >= 0) {
            if (x >= 0 && x < SCREEN_WIDTH) buffer[x + j * VIRTUAL_WIDTH] = color;
            if (x + w - 1 >= 0 && x + w - 1 < SCREEN_WIDTH) buffer[(x + w - 1) + j * VIRTUAL_WIDTH] = color;
        }
    }
}

static const unsigned char font5x7[96][5] = {
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},
    {0x14,0x7F,0x14,0x7F,0x14},{0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},{0x00,0x1C,0x22,0x41,0x00},
    {0x00,0x41,0x22,0x1C,0x00},{0x08,0x2A,0x1C,0x2A,0x08},{0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},
    {0x20,0x10,0x08,0x04,0x02},{0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},{0x18,0x14,0x12,0x7F,0x10},
    {0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},
    {0x00,0x56,0x36,0x00,0x00},{0x00,0x08,0x14,0x22,0x41},{0x14,0x14,0x14,0x14,0x14},
    {0x41,0x22,0x14,0x08,0x00},{0x02,0x01,0x51,0x09,0x06},{0x32,0x49,0x79,0x41,0x3E},
    {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x01,0x01},
    {0x3E,0x41,0x41,0x51,0x32},{0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x04,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F},{0x7F,0x20,0x18,0x20,0x7F},{0x63,0x14,0x08,0x14,0x63},
    {0x03,0x04,0x78,0x04,0x03},{0x61,0x51,0x49,0x45,0x43},{0x00,0x00,0x7F,0x41,0x41},
    {0x02,0x04,0x08,0x10,0x20},{0x41,0x41,0x7F,0x00,0x00},{0x04,0x02,0x01,0x02,0x04},
    {0x40,0x40,0x40,0x40,0x40},{0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},
    {0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},{0x38,0x44,0x44,0x48,0x7F},
    {0x38,0x54,0x54,0x54,0x18},{0x08,0x7E,0x09,0x01,0x02},{0x08,0x14,0x54,0x54,0x3C},
    {0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},{0x20,0x40,0x44,0x3D,0x00},
    {0x00,0x7F,0x10,0x28,0x44},{0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},
    {0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},{0x7C,0x14,0x14,0x14,0x08},
    {0x08,0x14,0x14,0x18,0x7C},{0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x20,0x7C},{0x1C,0x20,0x40,0x20,0x1C},
    {0x3C,0x40,0x30,0x40,0x3C},{0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},
    {0x44,0x64,0x54,0x4C,0x44},{0x00,0x08,0x36,0x41,0x00},{0x00,0x00,0x7F,0x00,0x00},
    {0x00,0x41,0x36,0x08,0x00},{0x08,0x08,0x2A,0x1C,0x08},{0x08,0x1C,0x2A,0x08,0x08}
};

static void DrawText(uint16_t *buffer, int x, int y, uint16_t color, const char *text)
{
    int px = x;
    while (*text) {
        int c = *text - 32;
        if (c >= 0 && c < 96) {
            for (int col = 0; col < 5; col++) {
                unsigned char bits = font5x7[c][col];
                for (int row = 0; row < 7; row++) {
                    if (bits & (1 << row)) {
                        int dx = px + col;
                        int dy = y + row;
                        if (dx >= 0 && dx < SCREEN_WIDTH && dy >= 0 && dy < SCREEN_HEIGHT) {
                            buffer[dx + dy * VIRTUAL_WIDTH] = color;
                        }
                    }
                }
            }
        }
        px += 6;
        text++;
    }
}

static void DrawSeparator(uint16_t *buffer, int x, int y, int w, uint16_t color)
{
    for (int i = x; i < x + w && i < SCREEN_WIDTH; i++) {
        if (i >= 0 && y >= 0 && y < SCREEN_HEIGHT) {
            buffer[i + y * VIRTUAL_WIDTH] = color;
        }
    }
}

/* ============== FPS DISPLAY (just the number) ============== */
static void draw_fps_overlay(uint16_t *pixels)
{
    if (!fps_show || !pixels || menu_active) return;

    /* Choose color based on FPS */
    uint16_t col;
    if (fps_current >= 25) col = FPS_GOOD;
    else if (fps_current >= 15) col = FPS_OK;
    else col = FPS_BAD;

    /* Draw background box */
    DrawFBox(pixels, 2, 2, 24, 11, FPS_BG);

    /* Draw just the FPS number */
    char buf[8];
    snprintf(buf, sizeof(buf), "%2d", fps_current);
    DrawText(pixels, 4, 4, col, buf);
}

static void update_fps_counter(void)
{
    fps_frame_count++;

    unsigned long now = get_time_ms();
    if (fps_last_time == 0) {
        fps_last_time = now;
        return;
    }

    unsigned long elapsed = now - fps_last_time;
    if (elapsed >= 1000) {
        fps_current = (fps_frame_count * 1000) / elapsed;
        fps_frame_count = 0;
        fps_last_time = now;
    }
}

/* ============== VIDEO CALLBACK WRAPPER ============== */
static void wrapped_video_cb(const void *data, unsigned width, unsigned height, size_t pitch)
{
    if (fps_show && data && !menu_active) {
        draw_fps_overlay((uint16_t*)data);
    }
    if (real_video_cb) {
        real_video_cb(data, width, height, pitch);
    }
}

/* ============== SPU RESYNC FUNCTION ============== */
/*
 * COMPREHENSIVE SPU RESET on menu resume:
 * - Reset timing: spu.cycles_played = psxRegs.cycle
 * - Reset audio buffer pointer: spu.pS = spu.pSpuBuffer
 * - Reset XA buffer pointers
 * - Reset CDDA buffer pointers
 */
static void resync_spu_after_pause(void)
{
#ifdef SPU_PCSXREARMED
    /* Resync timing */
    spu.cycles_played = psxRegs.cycle;

    /* Reset main audio buffer pointer */
    if (spu.pSpuBuffer) {
        spu.pS = (short *)spu.pSpuBuffer;
    }

    /* Reset XA audio buffer pointers */
    if (spu.XAStart) {
        spu.XAPlay = spu.XAStart;
        spu.XAFeed = spu.XAStart;
    }

    /* Reset CDDA buffer pointers */
    if (spu.CDDAStart) {
        spu.CDDAPlay = spu.CDDAStart;
        spu.CDDAFeed = spu.CDDAStart;
    }

    XLOG("SPU resync: cycles=%u, buffers reset", spu.cycles_played);
#endif
}

/* ============== CONFIG FILE FUNCTIONS ============== */

static void get_game_config_path(char *path, int maxlen)
{
    const char *base = strrchr(game_path, '/');
    if (!base) base = strrchr(game_path, '\\');
    if (base) base++; else base = game_path;
    snprintf(path, maxlen, "/mnt/sda1/cores/config/%s.cfg", base);
}

static int write_config_file(const char *path)
{
    FILE *f = fopen(path, "w");
    if (!f) return 0;

    fprintf(f, "# QPSX v%s Config\n", QPSX_VERSION);
    fprintf(f, "frameskip = %d\n", qpsx_config.frameskip);
    fprintf(f, "use_bios = %d\n", qpsx_config.use_bios);
    fprintf(f, "bios_file = %s\n", qpsx_config.bios_file);
    fprintf(f, "cycle_multiplier = %d\n", qpsx_config.cycle_mult);
    fprintf(f, "show_fps = %d\n", qpsx_config.show_fps);
    fprintf(f, "xa_disable = %d\n", qpsx_config.xa_disable);
    fprintf(f, "cdda_disable = %d\n", qpsx_config.cdda_disable);
    fprintf(f, "spu_update_freq = %d\n", qpsx_config.spu_update_freq);
    fprintf(f, "spu_reverb = %d\n", qpsx_config.spu_reverb);
    fprintf(f, "spu_interpolation = %d\n", qpsx_config.spu_interpolation);
    fprintf(f, "xa_update_freq = %d\n", qpsx_config.xa_update_freq);
    fprintf(f, "spu_tempo = %d\n", qpsx_config.spu_tempo);
    fprintf(f, "spu_irq_always = %d\n", qpsx_config.spu_irq_always);
    fprintf(f, "frame_duping = %d\n", qpsx_config.frame_duping);
    fprintf(f, "gpu_dithering = %d\n", qpsx_config.gpu_dithering);
    fprintf(f, "gpu_blending = %d\n", qpsx_config.gpu_blending);
    fprintf(f, "gpu_lighting = %d\n", qpsx_config.gpu_lighting);
    fprintf(f, "gpu_fast_lighting = %d\n", qpsx_config.gpu_fast_lighting);
    fprintf(f, "gpu_pixel_skip = %d\n", qpsx_config.gpu_pixel_skip);
    fprintf(f, "gpu_interlace_force = %d\n", qpsx_config.gpu_interlace_force);
    fprintf(f, "gpu_prog_ilace = %d\n", qpsx_config.gpu_prog_ilace);
    fprintf(f, "nosmccheck = %d\n", qpsx_config.nosmccheck);
    fprintf(f, "nogteflags = %d\n", qpsx_config.nogteflags);
    fprintf(f, "gteregsunneeded = %d\n", qpsx_config.gteregsunneeded);
    fprintf(f, "opt_lohi_cache = %d\n", qpsx_config.opt_lohi_cache);
    fprintf(f, "opt_nclip_inline = %d\n", qpsx_config.opt_nclip_inline);
    fprintf(f, "opt_const_prop = %d\n", qpsx_config.opt_const_prop);
    fprintf(f, "opt_rtpt_unroll = %d\n", qpsx_config.opt_rtpt_unroll);
    fprintf(f, "opt_gte_unrdiv = %d\n", qpsx_config.opt_gte_unrdiv);
    fprintf(f, "opt_gte_noflags = %d\n", qpsx_config.opt_gte_noflags);
    fprintf(f, "debug_log = %d\n", qpsx_config.debug_log);

    fclose(f);
    return 1;
}

static int load_config_file(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) return 0;

    char line[256];
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;

        char key[64], value_str[128];
        if (sscanf(line, "%63[^=]=%127s", key, value_str) == 2) {
            char *k = key;
            while (*k == ' ' || *k == '\t') k++;
            char *end = k + strlen(k) - 1;
            while (end > k && (*end == ' ' || *end == '\t')) *end-- = '\0';

            int value = atoi(value_str);

            if (strcmp(k, "frameskip") == 0) qpsx_config.frameskip = value;
            else if (strcmp(k, "use_bios") == 0) qpsx_config.use_bios = value;
            else if (strcmp(k, "bios_file") == 0) strncpy(qpsx_config.bios_file, value_str, 63);
            else if (strcmp(k, "cycle_multiplier") == 0) qpsx_config.cycle_mult = value;
            else if (strcmp(k, "show_fps") == 0) qpsx_config.show_fps = value;
            else if (strcmp(k, "xa_disable") == 0) qpsx_config.xa_disable = value;
            else if (strcmp(k, "cdda_disable") == 0) qpsx_config.cdda_disable = value;
            else if (strcmp(k, "spu_update_freq") == 0) qpsx_config.spu_update_freq = value;
            else if (strcmp(k, "spu_reverb") == 0) qpsx_config.spu_reverb = value;
            else if (strcmp(k, "spu_interpolation") == 0) qpsx_config.spu_interpolation = value;
            else if (strcmp(k, "xa_update_freq") == 0) qpsx_config.xa_update_freq = value;
            else if (strcmp(k, "spu_tempo") == 0) qpsx_config.spu_tempo = value;
            else if (strcmp(k, "spu_irq_always") == 0) qpsx_config.spu_irq_always = value;
            else if (strcmp(k, "frame_duping") == 0) qpsx_config.frame_duping = value;
            else if (strcmp(k, "gpu_dithering") == 0) qpsx_config.gpu_dithering = value;
            else if (strcmp(k, "gpu_blending") == 0) qpsx_config.gpu_blending = value;
            else if (strcmp(k, "gpu_lighting") == 0) qpsx_config.gpu_lighting = value;
            else if (strcmp(k, "gpu_fast_lighting") == 0) qpsx_config.gpu_fast_lighting = value;
            else if (strcmp(k, "gpu_pixel_skip") == 0) qpsx_config.gpu_pixel_skip = value;
            else if (strcmp(k, "gpu_interlace_force") == 0) qpsx_config.gpu_interlace_force = value;
            else if (strcmp(k, "gpu_prog_ilace") == 0) qpsx_config.gpu_prog_ilace = value;
            else if (strcmp(k, "nosmccheck") == 0) qpsx_config.nosmccheck = value;
            else if (strcmp(k, "nogteflags") == 0) qpsx_config.nogteflags = value;
            else if (strcmp(k, "gteregsunneeded") == 0) qpsx_config.gteregsunneeded = value;
            else if (strcmp(k, "opt_lohi_cache") == 0) qpsx_config.opt_lohi_cache = value;
            else if (strcmp(k, "opt_nclip_inline") == 0) qpsx_config.opt_nclip_inline = value;
            else if (strcmp(k, "opt_const_prop") == 0) qpsx_config.opt_const_prop = value;
            else if (strcmp(k, "opt_rtpt_unroll") == 0) qpsx_config.opt_rtpt_unroll = value;
            else if (strcmp(k, "opt_gte_unrdiv") == 0) qpsx_config.opt_gte_unrdiv = value;
            else if (strcmp(k, "opt_gte_noflags") == 0) qpsx_config.opt_gte_noflags = value;
            else if (strcmp(k, "debug_log") == 0) qpsx_config.debug_log = value;
        }
    }
    fclose(f);
    return 1;
}

static int delete_config_file(const char *path)
{
    FILE *f = fopen(path, "w");
    if (!f) return 0;
    fclose(f);
    return 1;
}

static int config_exists(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) return 0;
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fclose(f);
    return (size > 10);
}

static void set_feedback(const char *msg)
{
    strncpy(feedback_msg, msg, sizeof(feedback_msg) - 1);
    feedback_timer = 90;
}

/* ============== PRESETS (cycles only, no frameskip) ============== */

static void apply_preset_max_compat(void)
{
    qpsx_config.cycle_mult = 512;
    qpsx_config.gpu_dithering = 1;
    qpsx_config.gpu_blending = 3;
    qpsx_config.gpu_fast_lighting = 0;
    qpsx_config.gpu_pixel_skip = 0;
    qpsx_config.nosmccheck = 0;
    set_feedback("Preset: MAX COMPAT (cycles 512)");
}

static void apply_preset_balanced(void)
{
    qpsx_config.cycle_mult = 768;
    qpsx_config.gpu_dithering = 0;
    qpsx_config.gpu_blending = 1;
    qpsx_config.gpu_fast_lighting = 1;
    qpsx_config.gpu_pixel_skip = 1;
    qpsx_config.nosmccheck = 0;
    set_feedback("Preset: BALANCED (cycles 768)");
}

static void apply_preset_fast(void)
{
    qpsx_config.cycle_mult = 1024;
    qpsx_config.gpu_dithering = 0;
    qpsx_config.gpu_blending = 0;
    qpsx_config.gpu_fast_lighting = 1;
    qpsx_config.gpu_pixel_skip = 1;
    qpsx_config.nosmccheck = 1;
    set_feedback("Preset: FAST (cycles 1024)");
}

static void apply_preset_faster(void)
{
    qpsx_config.cycle_mult = 1536;
    qpsx_config.gpu_dithering = 0;
    qpsx_config.gpu_blending = 0;
    qpsx_config.gpu_fast_lighting = 1;
    qpsx_config.gpu_pixel_skip = 1;
    qpsx_config.nosmccheck = 1;
    set_feedback("Preset: FASTER (cycles 1536)");
}

static void apply_preset_max_speed(void)
{
    qpsx_config.cycle_mult = 2048;
    qpsx_config.gpu_dithering = 0;
    qpsx_config.gpu_blending = 0;
    qpsx_config.gpu_lighting = 0;
    qpsx_config.gpu_fast_lighting = 0;
    qpsx_config.gpu_pixel_skip = 1;
    qpsx_config.nosmccheck = 1;
    set_feedback("Preset: MAX SPEED (cycles 2048)");
}

static void apply_reset_defaults(void)
{
    qpsx_config.frameskip = 0;
    qpsx_config.cycle_mult = 1536;  /* v073 default */
    qpsx_config.xa_disable = 0;
    qpsx_config.cdda_disable = 0;
    qpsx_config.gpu_dithering = 0;
    qpsx_config.gpu_blending = 1;
    qpsx_config.gpu_lighting = 1;
    qpsx_config.gpu_fast_lighting = 0;  /* v081: OFF by default */
    qpsx_config.gpu_pixel_skip = 1;
    qpsx_config.gpu_interlace_force = 0;
    qpsx_config.gpu_prog_ilace = 0;
    qpsx_config.frame_duping = 0;
    qpsx_config.nosmccheck = 0;
    qpsx_config.show_fps = 0;
    qpsx_config.debug_log = 0;
    set_feedback("Reset to DEFAULTS done");
}

/* ============== APPLY CONFIGURATION ============== */
static void qpsx_apply_config(void)
{
    /* Apply frameskip via native GPU mechanism */
    gpu.frameskip.set = qpsx_config.frameskip;

    fps_show = qpsx_config.show_fps;

    Config.SpuUpdateFreq = qpsx_config.spu_update_freq;
    Config.ForcedXAUpdates = qpsx_config.xa_update_freq;
    Config.SpuIrq = qpsx_config.spu_irq_always;

    Config.Xa = qpsx_config.xa_disable;
    Config.Cdda = qpsx_config.cdda_disable;

#ifdef PSXREC
    cycle_multiplier = qpsx_config.cycle_mult;
#endif

    gpu_unai_config_ext.dithering = qpsx_config.gpu_dithering;
    gpu_unai_config_ext.blending = qpsx_config.gpu_blending;
    gpu_unai_config_ext.lighting = qpsx_config.gpu_lighting;
    gpu_unai_config_ext.fast_lighting = qpsx_config.gpu_fast_lighting;
    gpu_unai_config_ext.pixel_skip = qpsx_config.gpu_pixel_skip;
    gpu_unai_config_ext.ilace_force = qpsx_config.gpu_interlace_force;
#ifndef USE_GPULIB
    gpu_unai_config_ext.prog_ilace = qpsx_config.gpu_prog_ilace;
#endif

#ifdef SPU_PCSXREARMED
    spu_config.iUseReverb = qpsx_config.spu_reverb;
    spu_config.iUseInterpolation = qpsx_config.spu_interpolation;
    spu_config.iTempo = qpsx_config.spu_tempo;
#endif

    qpsx_nosmccheck = qpsx_config.nosmccheck;

    /* Dynarec optimizations */
    g_opt_lohi_cache = qpsx_config.opt_lohi_cache;
    g_opt_nclip_inline = qpsx_config.opt_nclip_inline;
    g_opt_const_prop = qpsx_config.opt_const_prop;
    g_opt_rtpt_unroll = qpsx_config.opt_rtpt_unroll;

    /* GTE optimizations (v088) */
    g_opt_gte_unrdiv = qpsx_config.opt_gte_unrdiv;
    g_opt_gte_noflags = qpsx_config.opt_gte_noflags;

    /* Update debug log state */
    update_debug_log_state(qpsx_config.debug_log);
}

/* ============== MENU OVERLAY ============== */
static void draw_menu_overlay(uint16_t *pixels)
{
    DrawFBox(pixels, MENU_X, MENU_Y, MENU_W, MENU_H, MENU_BG);
    DrawBox(pixels, MENU_X, MENU_Y, MENU_W, MENU_H, MENU_BORDER);
    DrawBox(pixels, MENU_X + 1, MENU_Y + 1, MENU_W - 2, MENU_H - 2, MENU_BORDER);

    int y = MENU_Y + 5;
    char buf[48];

    /* QPSX_082: Use QPSX_VERSION instead of hardcoded version */
    snprintf(buf, sizeof(buf), "QPSX v%s", QPSX_VERSION);
    DrawText(pixels, MENU_X + 115, y, MENU_TITLE, buf);
    y += MENU_LINE_H;
    DrawText(pixels, MENU_X + 85, y, MENU_AUTHOR, "by Grzegorz Korycki");
    y += MENU_LINE_H + 2;
    DrawSeparator(pixels, MENU_X + 5, y, MENU_W - 10, MENU_SEP);
    y += 4;

    if (menu_item < menu_scroll) menu_scroll = menu_item;
    if (menu_item >= menu_scroll + MENU_VISIBLE) menu_scroll = menu_item - MENU_VISIBLE + 1;
    if (menu_scroll < 0) menu_scroll = 0;

    if (menu_scroll > 0) {
        DrawText(pixels, MENU_X + MENU_W - 30, y - 2, MENU_DIM, "...");
    }

    int end_item = menu_scroll + MENU_VISIBLE;
    if (end_item > MENU_ITEMS) end_item = MENU_ITEMS;

    for (int item = menu_scroll; item < end_item; item++) {
        const MenuItem *mi = &menu_items[item];
        uint16_t col = MENU_FG;
        uint16_t marker_col = MENU_INSTANT;
        const char *marker = "[*]";

        if (mi->type == 1) {
            DrawText(pixels, MENU_X + 8, y, MENU_SEP, mi->name);
            y += MENU_LINE_H;
            continue;
        }

        if (menu_item == item) col = MENU_SEL;
        if (mi->restart) { marker = "[R]"; marker_col = MENU_RESTART; }

        const char *sel = (menu_item == item) ? ">" : " ";

        switch (item) {
            case MI_FRAMESKIP:
                snprintf(buf, sizeof(buf), "%s%-12s: %d", sel, mi->name, qpsx_config.frameskip);
                break;
            case MI_BIOS:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.use_bios ? "Real" : "HLE");
                break;
            case MI_CYCLE:
                snprintf(buf, sizeof(buf), "%s%-12s: %d", sel, mi->name, qpsx_config.cycle_mult);
                break;
            case MI_SHOW_FPS:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.show_fps ? "ON" : "OFF");
                break;
            case MI_DEBUG_LOG:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.debug_log ? "ON" : "OFF");
                break;
            case MI_XA_AUDIO:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.xa_disable ? "OFF" : "ON");
                col = (!qpsx_config.xa_disable) ? ((menu_item == item) ? MENU_SEL : MENU_GREEN) : ((menu_item == item) ? MENU_SEL : MENU_RED);
                break;
            case MI_CDDA_AUDIO:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.cdda_disable ? "OFF" : "ON");
                col = (!qpsx_config.cdda_disable) ? ((menu_item == item) ? MENU_SEL : MENU_GREEN) : ((menu_item == item) ? MENU_SEL : MENU_RED);
                break;
            case MI_DITHERING:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.gpu_dithering ? "ON" : "OFF");
                break;
            case MI_BLENDING:
                snprintf(buf, sizeof(buf), "%s%-12s: %d", sel, mi->name, qpsx_config.gpu_blending);
                break;
            case MI_LIGHTING:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.gpu_lighting ? "ON" : "OFF");
                break;
            case MI_FAST_LIGHT:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.gpu_fast_lighting ? "ON" : "OFF");
                break;
            case MI_PIXEL_SKIP:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.gpu_pixel_skip ? "ON" : "OFF");
                break;
            case MI_INTERLACE:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.gpu_interlace_force ? "ON" : "OFF");
                break;
            case MI_PROG_ILACE:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.gpu_prog_ilace ? "ON" : "OFF");
                break;
            case MI_FRAME_DUPE:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.frame_duping ? "ON" : "OFF");
                break;
            case MI_SMC_CHECK:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.nosmccheck ? "OFF" : "ON");
                break;
            case MI_OPT_LOHI:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.opt_lohi_cache ? "ON" : "OFF");
                break;
            case MI_OPT_NCLIP:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.opt_nclip_inline ? "ON" : "OFF");
                break;
            case MI_OPT_CONST:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.opt_const_prop ? "ON" : "OFF");
                break;
            case MI_OPT_RTPT:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.opt_rtpt_unroll ? "ON" : "OFF");
                break;
            case MI_OPT_UNRDIV:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.opt_gte_unrdiv ? "ON" : "OFF");
                break;
            case MI_OPT_NOFLAGS:
                /* Show in red to indicate risk */
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.opt_gte_noflags ? "ON-RISK!" : "OFF");
                break;
            case MI_PRESET_COMPAT:
            case MI_PRESET_BALANCED:
            case MI_PRESET_FAST:
            case MI_PRESET_FASTER:
            case MI_PRESET_MAXSPEED:
            case MI_RESET_DEFAULTS:
                snprintf(buf, sizeof(buf), "%s[%s]", sel, mi->name);
                col = (menu_item == item) ? MENU_SEL : MENU_PRESET;
                marker = "";
                break;
            case MI_SAVE_GAME:
            case MI_SAVE_GLOBAL:
            case MI_DEL_GAME:
            case MI_DEL_GLOBAL:
            case MI_EXIT:
                snprintf(buf, sizeof(buf), "%s[%s]", sel, mi->name);
                if (menu_item == item) col = MENU_SEL;
                else if (item == MI_SAVE_GAME || item == MI_SAVE_GLOBAL) col = MENU_GREEN;
                else if (item == MI_DEL_GAME || item == MI_DEL_GLOBAL) col = MENU_RED;
                else col = MENU_TITLE;
                marker = "";
                break;
            default:
                buf[0] = 0;
                break;
        }

        DrawText(pixels, MENU_X + 8, y, col, buf);
        if (mi->type == 0 && marker[0]) {
            DrawText(pixels, MENU_X + MENU_W - 30, y, marker_col, marker);
        }
        y += MENU_LINE_H;
    }

    if (end_item < MENU_ITEMS) {
        DrawText(pixels, MENU_X + MENU_W - 30, y, MENU_DIM, "...");
    }

    if (feedback_timer > 0) {
        feedback_timer--;
        int msg_y = MENU_Y + MENU_H - 24;
        DrawFBox(pixels, MENU_X + 5, msg_y - 2, MENU_W - 10, 12, RGB565(0, 64, 0));
        DrawText(pixels, MENU_X + 10, msg_y, MENU_FG, feedback_msg);
    }

    int help_y = MENU_Y + MENU_H - 12;
    if (feedback_timer == 0) {
        DrawText(pixels, MENU_X + 8, help_y, MENU_DIM, "L/R:change A:select B/START:close");
    }
}

/* ============== MENU INPUT HANDLING ============== */
static void handle_menu_input(void)
{
    static int prev_up = 0, prev_down = 0, prev_left = 0, prev_right = 0;
    static int prev_a = 0, prev_start = 0, prev_b = 0;
    static int repeat_delay = 0;

    if (menu_first_frame) {
        prev_up = prev_down = prev_left = prev_right = prev_a = prev_start = prev_b = 1;
        menu_first_frame = 0;
        menu_entry_delay = 15;
    }

    if (menu_entry_delay > 0) {
        menu_entry_delay--;
        return;
    }

    int cur_up    = input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_UP);
    int cur_down  = input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_DOWN);
    int cur_left  = input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_LEFT);
    int cur_right = input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_RIGHT);
    int cur_a     = input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_A);
    int cur_b     = input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_B);
    int cur_start = input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_START);

    if ((cur_b && !prev_b) || (cur_start && !prev_start)) {
        menu_active = 0;
        menu_first_frame = 1;
        start_hold_frames = 0;
        qpsx_apply_config();
        prev_b = cur_b;
        prev_start = cur_start;
        return;
    }

    if (repeat_delay > 0) repeat_delay--;

    if (cur_up && !prev_up) {
        do { menu_item--; if (menu_item < 0) menu_item = MENU_ITEMS - 1; } while (menu_items[menu_item].type == 1);
        repeat_delay = 8;
    }

    if (cur_down && !prev_down) {
        do { menu_item++; if (menu_item >= MENU_ITEMS) menu_item = 0; } while (menu_items[menu_item].type == 1);
        repeat_delay = 8;
    }

    if (menu_items[menu_item].type == 0) {
        if (cur_left && !prev_left) {
            switch (menu_item) {
                case MI_FRAMESKIP: if (qpsx_config.frameskip > 0) qpsx_config.frameskip--; break;
                case MI_BIOS: qpsx_config.use_bios = !qpsx_config.use_bios; break;
                case MI_CYCLE: if (qpsx_config.cycle_mult > 256) qpsx_config.cycle_mult -= 128; break;
                case MI_SHOW_FPS: qpsx_config.show_fps = !qpsx_config.show_fps; break;
                case MI_DEBUG_LOG: qpsx_config.debug_log = !qpsx_config.debug_log; break;
                case MI_XA_AUDIO: qpsx_config.xa_disable = !qpsx_config.xa_disable; break;
                case MI_CDDA_AUDIO: qpsx_config.cdda_disable = !qpsx_config.cdda_disable; break;
                case MI_DITHERING: qpsx_config.gpu_dithering = !qpsx_config.gpu_dithering; break;
                case MI_BLENDING: if (qpsx_config.gpu_blending > 0) qpsx_config.gpu_blending--; break;
                case MI_LIGHTING: qpsx_config.gpu_lighting = !qpsx_config.gpu_lighting; break;
                case MI_FAST_LIGHT: qpsx_config.gpu_fast_lighting = !qpsx_config.gpu_fast_lighting; break;
                case MI_PIXEL_SKIP: qpsx_config.gpu_pixel_skip = !qpsx_config.gpu_pixel_skip; break;
                case MI_INTERLACE: qpsx_config.gpu_interlace_force = !qpsx_config.gpu_interlace_force; break;
                case MI_PROG_ILACE: qpsx_config.gpu_prog_ilace = !qpsx_config.gpu_prog_ilace; break;
                case MI_FRAME_DUPE: qpsx_config.frame_duping = !qpsx_config.frame_duping; break;
                case MI_SMC_CHECK: qpsx_config.nosmccheck = !qpsx_config.nosmccheck; break;
                case MI_OPT_LOHI: qpsx_config.opt_lohi_cache = !qpsx_config.opt_lohi_cache; break;
                case MI_OPT_NCLIP: qpsx_config.opt_nclip_inline = !qpsx_config.opt_nclip_inline; break;
                case MI_OPT_CONST: qpsx_config.opt_const_prop = !qpsx_config.opt_const_prop; break;
                case MI_OPT_RTPT: qpsx_config.opt_rtpt_unroll = !qpsx_config.opt_rtpt_unroll; break;
                case MI_OPT_UNRDIV: qpsx_config.opt_gte_unrdiv = !qpsx_config.opt_gte_unrdiv; break;
                case MI_OPT_NOFLAGS: qpsx_config.opt_gte_noflags = !qpsx_config.opt_gte_noflags; break;
            }
            qpsx_apply_config();
            repeat_delay = 8;
        }

        if (cur_right && !prev_right) {
            switch (menu_item) {
                case MI_FRAMESKIP: if (qpsx_config.frameskip < 5) qpsx_config.frameskip++; break;
                case MI_BIOS: qpsx_config.use_bios = !qpsx_config.use_bios; break;
                case MI_CYCLE: if (qpsx_config.cycle_mult < 2048) qpsx_config.cycle_mult += 128; break;
                case MI_SHOW_FPS: qpsx_config.show_fps = !qpsx_config.show_fps; break;
                case MI_DEBUG_LOG: qpsx_config.debug_log = !qpsx_config.debug_log; break;
                case MI_XA_AUDIO: qpsx_config.xa_disable = !qpsx_config.xa_disable; break;
                case MI_CDDA_AUDIO: qpsx_config.cdda_disable = !qpsx_config.cdda_disable; break;
                case MI_DITHERING: qpsx_config.gpu_dithering = !qpsx_config.gpu_dithering; break;
                case MI_BLENDING: if (qpsx_config.gpu_blending < 3) qpsx_config.gpu_blending++; break;
                case MI_LIGHTING: qpsx_config.gpu_lighting = !qpsx_config.gpu_lighting; break;
                case MI_FAST_LIGHT: qpsx_config.gpu_fast_lighting = !qpsx_config.gpu_fast_lighting; break;
                case MI_PIXEL_SKIP: qpsx_config.gpu_pixel_skip = !qpsx_config.gpu_pixel_skip; break;
                case MI_INTERLACE: qpsx_config.gpu_interlace_force = !qpsx_config.gpu_interlace_force; break;
                case MI_PROG_ILACE: qpsx_config.gpu_prog_ilace = !qpsx_config.gpu_prog_ilace; break;
                case MI_FRAME_DUPE: qpsx_config.frame_duping = !qpsx_config.frame_duping; break;
                case MI_SMC_CHECK: qpsx_config.nosmccheck = !qpsx_config.nosmccheck; break;
                case MI_OPT_LOHI: qpsx_config.opt_lohi_cache = !qpsx_config.opt_lohi_cache; break;
                case MI_OPT_NCLIP: qpsx_config.opt_nclip_inline = !qpsx_config.opt_nclip_inline; break;
                case MI_OPT_CONST: qpsx_config.opt_const_prop = !qpsx_config.opt_const_prop; break;
                case MI_OPT_RTPT: qpsx_config.opt_rtpt_unroll = !qpsx_config.opt_rtpt_unroll; break;
                case MI_OPT_UNRDIV: qpsx_config.opt_gte_unrdiv = !qpsx_config.opt_gte_unrdiv; break;
                case MI_OPT_NOFLAGS: qpsx_config.opt_gte_noflags = !qpsx_config.opt_gte_noflags; break;
            }
            qpsx_apply_config();
            repeat_delay = 8;
        }
    }

    if (cur_a && !prev_a && (menu_items[menu_item].type == 2 || menu_items[menu_item].type == 3)) {
        char path[256];
        switch (menu_item) {
            case MI_PRESET_COMPAT: apply_preset_max_compat(); qpsx_apply_config(); break;
            case MI_PRESET_BALANCED: apply_preset_balanced(); qpsx_apply_config(); break;
            case MI_PRESET_FAST: apply_preset_fast(); qpsx_apply_config(); break;
            case MI_PRESET_FASTER: apply_preset_faster(); qpsx_apply_config(); break;
            case MI_PRESET_MAXSPEED: apply_preset_max_speed(); qpsx_apply_config(); break;
            case MI_RESET_DEFAULTS: apply_reset_defaults(); qpsx_apply_config(); break;
            case MI_SAVE_GAME:
                get_game_config_path(path, sizeof(path));
                set_feedback(write_config_file(path) ? "Game config saved!" : "Save FAILED!");
                break;
            case MI_SAVE_GLOBAL:
                set_feedback(write_config_file(QPSX_GLOBAL_CONFIG_PATH) ? "Global config saved!" : "Save FAILED!");
                break;
            case MI_DEL_GAME:
                get_game_config_path(path, sizeof(path));
                if (config_exists(path)) { delete_config_file(path); set_feedback("Game config deleted!"); }
                else set_feedback("No game config found");
                break;
            case MI_DEL_GLOBAL:
                if (config_exists(QPSX_GLOBAL_CONFIG_PATH)) { delete_config_file(QPSX_GLOBAL_CONFIG_PATH); set_feedback("Global config deleted!"); }
                else set_feedback("No global config found");
                break;
            case MI_EXIT:
                menu_active = 0;
                menu_first_frame = 1;
                start_hold_frames = 0;
                qpsx_apply_config();
                break;
        }
        repeat_delay = 15;
    }

    prev_up = cur_up; prev_down = cur_down; prev_left = cur_left; prev_right = cur_right;
    prev_a = cur_a; prev_b = cur_b; prev_start = cur_start;
}

/* ============== 1.5-SECOND START HOLD DETECTION ============== */
static void check_menu_hotkey(void)
{
    int start = input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_START);

    if (start) {
        start_hold_frames++;
        if (start_hold_frames >= MENU_HOLD_FRAMES) {
            menu_active = 1;
            menu_first_frame = 1;
            start_hold_frames = 0;
            XLOG("Menu opened via 1.5-sec START hold");
        }
    } else {
        start_hold_frames = 0;
    }
}

/* ============== CONFIG LOADING ============== */
static void qpsx_load_config(void)
{
    char game_cfg[256];
    get_game_config_path(game_cfg, sizeof(game_cfg));

    if (load_config_file(game_cfg)) {
        XLOG("Loaded per-game config: %s", game_cfg);
        return;
    }

    if (load_config_file(QPSX_GLOBAL_CONFIG_PATH)) {
        XLOG("Loaded global config");
        return;
    }

    XLOG("No config found, using defaults");
    write_config_file(QPSX_GLOBAL_CONFIG_PATH);
}

/* ============== LIBRETRO API ============== */

void retro_set_environment(retro_environment_t cb)
{
    xlog("\n========================================\n");
    xlog("STARTING QPSX - VERSION %s\n", QPSX_VERSION);
    xlog("========================================\n");
    environ_cb = cb;
    struct retro_variable variables[] = { { NULL, NULL } };
    cb(RETRO_ENVIRONMENT_SET_VARIABLES, variables);
}

void retro_set_video_refresh(retro_video_refresh_t cb)
{
    real_video_cb = cb;
    video_cb = wrapped_video_cb;
}

void retro_set_audio_sample(retro_audio_sample_t cb) { audio_cb = cb; }
void retro_set_audio_sample_batch(retro_audio_sample_batch_t cb) { audio_batch_cb = cb; }
void retro_set_input_poll(retro_input_poll_t cb) { input_poll_cb = cb; }
void retro_set_input_state(retro_input_state_t cb) { input_state_cb = cb; }

void retro_init(void)
{
    XLOG("=== retro_init() START ===");

    const char *dir = NULL;
    if (environ_cb(RETRO_ENVIRONMENT_GET_SYSTEM_DIRECTORY, &dir) && dir)
        retro_system_directory = dir;
    else
        retro_system_directory = ".";

    if (environ_cb(RETRO_ENVIRONMENT_GET_SAVE_DIRECTORY, &dir) && dir)
        retro_save_directory = dir;
    else
        retro_save_directory = ".";

    if (environ_cb(RETRO_ENVIRONMENT_GET_CONTENT_DIRECTORY, &dir) && dir)
        retro_content_directory = dir;
    else
        retro_content_directory = ".";

    enum retro_pixel_format fmt = RETRO_PIXEL_FORMAT_RGB565;
    environ_cb(RETRO_ENVIRONMENT_SET_PIXEL_FORMAT, &fmt);

    fps_last_time = get_time_ms();
    fps_frame_count = 0;
    fps_current = 0;

    video_clear();
    XLOG("=== retro_init() DONE ===");
}

void retro_deinit(void)
{
    XLOG("=== retro_deinit() ===");
    if (psx_initted) {
        psxShutdown();
        ReleasePlugins();
        psx_initted = false;
    }
}

unsigned retro_api_version(void) { return RETRO_API_VERSION; }

void retro_get_system_info(struct retro_system_info *info)
{
    memset(info, 0, sizeof(*info));
    info->library_name = "pcsx4all";
    info->library_version = "QPSX_" QPSX_VERSION;
    info->valid_extensions = "bin|iso|img|cue|pbp";
    info->need_fullpath = true;
    info->block_extract = false;
}

void retro_get_system_av_info(struct retro_system_av_info *info)
{
    struct retro_game_geometry geom = { SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT, 4.0f/3.0f };
    struct retro_system_timing timing = { 60.0, 44100.0 };
    info->geometry = geom;
    info->timing = timing;
}

void retro_set_controller_port_device(unsigned port, unsigned device) { }
void retro_reset(void) { XLOG("=== retro_reset() ==="); psxReset(); }

static int run_frame_count = 0;
static int auto_menu_frames = 0;  /* QPSX_083: Auto menu open/close for SPU/GPU resync */

void retro_run(void)
{
    if (run_frame_count == 0) {
        XLOG("retro_run() frame 0");
        /*
         * QPSX_081 FIX: SPU desync at startup
         * spu.cycles_played starts at 0 but psxRegs.cycle may be large
         * after BIOS/HLE init. This caused SPU to be desynced, resulting
         * in ~20% slower performance until menu was opened.
         * Symptom: 40 FPS at start, 50 FPS after menu entry/exit.
         */
        resync_spu_after_pause();
    }
    run_frame_count++;
    input_debug_counter++;

    update_fps_counter();

    /* Cache input ONCE per retro_run - SF2000 hardware can't handle frequent polling */
    update_input_cache();

    /*
     * QPSX_084: INVISIBLE auto-menu for SPU/GPU desync fix
     *
     * Key insight from v083: qpsx_apply_config() call triggers the FPS fix
     * Now we make the menu INVISIBLE - user only sees brief pause (~60ms)
     *
     * At frame 250 (~5 sec at 50fps): Open menu invisibly for 3 frames
     * Frame 253: Auto-close with qpsx_apply_config() call
     */
    if (run_frame_count == 250 && !menu_active && auto_menu_frames == 0) {
        XLOG("QPSX_084: AUTO-OPENING INVISIBLE MENU");
        auto_menu_frames = 3;  /* Just 3 frames - menu is invisible anyway */
        menu_active = 1;
        menu_first_frame = 1;
    }

    /*
     * QPSX_078: Boot menu detection
     * If START is held during first 10 frames of boot, open menu immediately.
     * This allows users to recover from broken configs without SD card access.
     * Checking frames 5-10 gives input hardware time to initialize properly.
     */
    if (run_frame_count >= 5 && run_frame_count <= 10 && !menu_active) {
        int start = input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_START);
        if (start) {
            XLOG("START held at boot - opening menu (recovery mode)");
            menu_active = 1;
            menu_first_frame = 1;
            start_hold_frames = 0;
        }
    }

    if (!menu_active) {
        check_menu_hotkey();
    }

    /* Detect transition from menu to gameplay - resync SPU */
    if (menu_was_active && !menu_active) {
        XLOG("Menu closed - resyncing SPU");
        resync_spu_after_pause();
    }
    menu_was_active = menu_active;

    /* Menu mode - pause emulation */
    if (menu_active) {
        handle_menu_input();

        if (SCREEN) {
            /*
             * QPSX_084: Invisible auto-menu
             * When auto_menu_frames > 0, skip drawing overlay - user sees game, not menu
             * When auto_menu_frames == 0, normal manual menu - draw overlay
             */
            if (auto_menu_frames == 0) {
                draw_menu_overlay(SCREEN);  /* Manual menu - draw overlay */
            }
            /* Always output frame (with or without overlay) */
            if (real_video_cb) {
                real_video_cb(SCREEN, SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_WIDTH * 2);
            }
        }

        /*
         * QPSX_084: Auto-close menu after countdown (INVISIBLE)
         * Menu logic runs but user doesn't see overlay
         */
        if (auto_menu_frames > 0) {
            auto_menu_frames--;
            if (auto_menu_frames == 0) {
                XLOG("QPSX_084: AUTO-CLOSING INVISIBLE MENU - qpsx_apply_config() + resync");
                qpsx_apply_config();  /* Key fix from v083 */
                menu_active = 0;
            }
        }

        return;
    }

    /* Normal emulation - frameskip is handled by gpu.frameskip.set */
    psxCpu->Execute();
}

bool retro_load_game(const struct retro_game_info *info)
{
    XLOG("=== retro_load_game() ===");

    if (!info || !info->path) {
        XLOG("ERROR: No game info!");
        return false;
    }

    XLOG("Loading: %s", info->path);
    strncpy(game_path, info->path, sizeof(game_path) - 1);

    qpsx_load_config();
    qpsx_apply_config();

    memset(&Config, 0, sizeof(Config));

    Config.Xa = qpsx_config.xa_disable;
    Config.Cdda = qpsx_config.cdda_disable;
    Config.Mdec = 0;
    Config.PsxAuto = 1;

    Config.HLE = qpsx_config.use_bios ? 0 : 1;
    Config.SlowBoot = 0;
    Config.RCntFix = 0;
    Config.VSyncWA = 0;
    Config.FrameSkip = 0;
    Config.FrameLimit = 0;
    Config.Cpu = 0;

    snprintf(Config.BiosDir, sizeof(Config.BiosDir), "/mnt/sda1/bios");
    snprintf(Config.Bios, sizeof(Config.Bios), "%s", qpsx_config.bios_file);
    XLOG("BIOS: %s/%s", Config.BiosDir, Config.Bios);

    snprintf(Config.Mcd1, sizeof(Config.Mcd1), "/mnt/sda1/cores/config/pcsx4all_mcd1.mcd");
    snprintf(Config.Mcd2, sizeof(Config.Mcd2), "/mnt/sda1/cores/config/pcsx4all_mcd2.mcd");

    if (psxInit() == -1) {
        XLOG("ERROR: psxInit() failed!");
        return false;
    }

    if (LoadPlugins() == -1) {
        XLOG("ERROR: LoadPlugins() failed!");
        return false;
    }

    psx_initted = true;

    SetIsoFile(game_path);
    if (CDR_open() < 0) {
        XLOG("ERROR: CDR_open() failed!");
        return false;
    }

    psxReset();

    if (CheckCdrom() == -1) {
        XLOG("ERROR: CheckCdrom() failed!");
        return false;
    }

    if (Config.HLE) {
        if (LoadCdrom() == -1) {
            XLOG("ERROR: LoadCdrom() failed!");
            return false;
        }
    }

    XLOG("=== Game loaded! ===");
    return true;
}

void retro_unload_game(void)
{
    XLOG("=== retro_unload_game() ===");
}

unsigned retro_get_region(void) { return RETRO_REGION_NTSC; }
bool retro_load_game_special(unsigned type, const struct retro_game_info *info, size_t num) { return false; }
size_t retro_serialize_size(void) { return 0; }
bool retro_serialize(void *data, size_t size) { return false; }
bool retro_unserialize(const void *data, size_t size) { return false; }
void *retro_get_memory_data(unsigned id) { return NULL; }
size_t retro_get_memory_size(unsigned id) { return 0; }
void retro_cheat_reset(void) { }
void retro_cheat_set(unsigned index, bool enabled, const char *code) { }
