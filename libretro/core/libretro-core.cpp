/*
 * PCSX4ALL libretro core for SF2000
 * QPSX - version defined by QPSX_VERSION macro below
 *
 * v103 changes:
 * - ASM MemWrite now has 7 LEVELS (0-6) for precise debugging:
 *   * 0 = OFF    : Pure C path (baseline)
 *   * 1 = V98    : ASM(ptr_arith + store), C(addr, HW, LUT, codeinv) - WORKS!
 *   * 2 = +ADDR  : ASM(addr_calc + ptr_arith + store), C(HW, LUT, codeinv)
 *   * 3 = +LUT   : ASM(addr_calc + LUT + ptr_arith + store), C(HW, codeinv)
 *   * 4 = +HWBR  : ASM(addr_calc + LUT + HW_branch + ptr_arith + store), C(HW_call, codeinv)
 *   * 5 = +HWCALL: ASM(everything except codeinv), C(codeinv only)
 *   * 6 = FULL   : Full psxmem_asm.S (everything in ASM) - BROKEN!
 * - Test incrementally: if level N works but N+1 breaks, bug is in that ASM part!
 *
 * v102: Split ASM MemWrite into 3 levels (OFF/V98/FULL)
 * v101 changes:
 * - ALL OPTIMIZATIONS NOW OPTIONAL (menu-controlled, default OFF!)
 *   * DirectBlockLUT - menu option, needs restart
 *   * ASM MemWrite - already optional (v098)
 *   * Skip Code Inv - already optional (v098)
 * - Fixed v099/v100 where optimizations were hardcoded ON
 * - Now you can isolate which optimization breaks a game!
 *
 * v100: Direct Block LUT (was hardcoded ON - caused Ridge Racer issues)
 * v099: PURE ASM Memory functions (psxmem_asm.S)
 * v098: Memory optimization menu options (SkipCodeInv, ASM MemWrite toggle)
 * v097: Complete profiler with all 8 CPU categories
 * v096: Fixed profiler CPU breakdown display
 * v095: GTE ASM blocks (NCLIP, AVSZ3) - optional assembly replacements
 * v094: Detailed CPU profiler categories (Mem, HW, Exception, BIOS)
 * v093: Various stability improvements
 * v092: Performance Profiler with MIPS CP0 Count register
 * v091: GPU MIPS32 ASM (MOVN/MOVZ lighting, EXT/INS blending)
 * v090: Fast Blend (branchless C blending for MIPS32)
 * v089-v076: Various optimizations and framework
 */

#include "libretro.h"
#include "port.h"
#include "profiler.h"
#include "cdriso.h"  /* v258: CDDA conversion functions */
#include "plugin_lib/plugin_lib.h"  /* QPSX_280: For pl_init() */
#include "cdrom.h"   /* v283: For LidInterrupt() */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <time.h>       /* v283: For time() in CD swap */
#include <sys/time.h>

/* SF2000 xlog and timer */
#ifdef SF2000
extern "C" {
    extern void xlog(const char *fmt, ...);
    extern void xlog_clear(void);
    extern uint32_t os_get_tick_count(void);  /* v267: for ETA calculation */
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

/* v343: Only log when debug_log enabled (was always logging) */
#ifdef SF2000
#define XLOG(fmt, ...) do { if (g_debug_log_enabled) xlog("QPSX: " fmt "\n", ##__VA_ARGS__); } while(0)
#else
#define XLOG(fmt, ...) do { if (g_debug_log_enabled) printf("QPSX: " fmt "\n", ##__VA_ARGS__); } while(0)
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
#include "gte.h"       /* v294: For gte_update_dispatch() */
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
static int g_player2_enabled = 0;  /* v368: Set from config in qpsx_apply_config() */

/* Debug logging for input path comparison - must be before functions that use it */
static int input_debug_counter = 0;
#define INPUT_DEBUG_INTERVAL 60  /* Log every 60 frames (1 second at 60fps) */

/* v351: Forward declaration of remap_lut - defined later, used here */
extern uint8_t remap_lut[16];

static void update_input_cache(void)
{
    uint16_t pad1 = 0xFFFF;  /* All buttons released (active low) */
    uint16_t pad2 = 0xFFFF;  /* v251: Player 2 support */

    if (input_poll_cb) input_poll_cb();

    if (input_state_cb) {
        /* v351: Player 1 - use pre-compiled remap_lut[] for ZERO config access */
        #define CHECK_BTN_P1(id) \
            if (input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, id)) \
                pad1 &= ~(1 << remap_lut[id])

        CHECK_BTN_P1(RETRO_DEVICE_ID_JOYPAD_SELECT);
        CHECK_BTN_P1(RETRO_DEVICE_ID_JOYPAD_L3);
        CHECK_BTN_P1(RETRO_DEVICE_ID_JOYPAD_R3);
        CHECK_BTN_P1(RETRO_DEVICE_ID_JOYPAD_START);
        CHECK_BTN_P1(RETRO_DEVICE_ID_JOYPAD_UP);
        CHECK_BTN_P1(RETRO_DEVICE_ID_JOYPAD_RIGHT);
        CHECK_BTN_P1(RETRO_DEVICE_ID_JOYPAD_DOWN);
        CHECK_BTN_P1(RETRO_DEVICE_ID_JOYPAD_LEFT);
        CHECK_BTN_P1(RETRO_DEVICE_ID_JOYPAD_L2);
        CHECK_BTN_P1(RETRO_DEVICE_ID_JOYPAD_R2);
        CHECK_BTN_P1(RETRO_DEVICE_ID_JOYPAD_L);
        CHECK_BTN_P1(RETRO_DEVICE_ID_JOYPAD_R);
        CHECK_BTN_P1(RETRO_DEVICE_ID_JOYPAD_X);
        CHECK_BTN_P1(RETRO_DEVICE_ID_JOYPAD_A);
        CHECK_BTN_P1(RETRO_DEVICE_ID_JOYPAD_B);
        CHECK_BTN_P1(RETRO_DEVICE_ID_JOYPAD_Y);

        #undef CHECK_BTN_P1

        /* v368: Player 2 - only poll if enabled (saves 16 input_state_cb calls) */
        if (g_player2_enabled) {
            #define CHECK_BTN_P2(id) \
                if (input_state_cb(1, RETRO_DEVICE_JOYPAD, 0, id)) \
                    pad2 &= ~(1 << remap_lut[id])

            CHECK_BTN_P2(RETRO_DEVICE_ID_JOYPAD_SELECT);
            CHECK_BTN_P2(RETRO_DEVICE_ID_JOYPAD_L3);
            CHECK_BTN_P2(RETRO_DEVICE_ID_JOYPAD_R3);
            CHECK_BTN_P2(RETRO_DEVICE_ID_JOYPAD_START);
            CHECK_BTN_P2(RETRO_DEVICE_ID_JOYPAD_UP);
            CHECK_BTN_P2(RETRO_DEVICE_ID_JOYPAD_RIGHT);
            CHECK_BTN_P2(RETRO_DEVICE_ID_JOYPAD_DOWN);
            CHECK_BTN_P2(RETRO_DEVICE_ID_JOYPAD_LEFT);
            CHECK_BTN_P2(RETRO_DEVICE_ID_JOYPAD_L2);
            CHECK_BTN_P2(RETRO_DEVICE_ID_JOYPAD_R2);
            CHECK_BTN_P2(RETRO_DEVICE_ID_JOYPAD_L);
            CHECK_BTN_P2(RETRO_DEVICE_ID_JOYPAD_R);
            CHECK_BTN_P2(RETRO_DEVICE_ID_JOYPAD_X);
            CHECK_BTN_P2(RETRO_DEVICE_ID_JOYPAD_A);
            CHECK_BTN_P2(RETRO_DEVICE_ID_JOYPAD_B);
            CHECK_BTN_P2(RETRO_DEVICE_ID_JOYPAD_Y);

            #undef CHECK_BTN_P2
        }
    }

    /* DEBUG: Log when buttons are pressed (only when debug_log enabled) */
    if (pad1 != 0xFFFF && (input_debug_counter % INPUT_DEBUG_INTERVAL) == 0) {
        DEBUG_LOG("[INPUT-CACHE] pad1=0x%04X (buttons pressed)", pad1);
    }

    cached_pad1 = pad1;
    cached_pad2 = pad2;  /* v251: Player 2 active */
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

/* ============== v351: PRE-COMPILED REMAP LUT ============== */
/*
 * Pre-compiled button remap lookup table for ZERO hot-path overhead.
 *
 * PROBLEM (v348-v350):
 * - qpsx_config grew to ~210 bytes with remap_* fields
 * - Accessing qpsx_config.remap_* in update_input_cache() caused cache misses
 * - SF2000 L1 cache line = 32 bytes, structure spans ~7 cache lines
 * - Result: ~4% fps drop (39.5 -> 38 fps)
 *
 * SOLUTION (v351):
 * - Pre-compute 16-byte remap_lut[] once when config changes
 * - update_input_cache() uses ONLY this tiny array - always in L1 cache
 * - ZERO access to qpsx_config in hot path
 *
 * PSX pad bit positions (from PSX hardware spec, active-low):
 */
#define PSX_BIT_SELECT   0
#define PSX_BIT_L3       1
#define PSX_BIT_R3       2
#define PSX_BIT_START    3
#define PSX_BIT_UP       4
#define PSX_BIT_RIGHT    5
#define PSX_BIT_DOWN     6
#define PSX_BIT_LEFT     7
#define PSX_BIT_L2       8
#define PSX_BIT_R2       9
#define PSX_BIT_L1      10
#define PSX_BIT_R1      11
#define PSX_BIT_TRI     12
#define PSX_BIT_CIRCLE  13
#define PSX_BIT_CROSS   14
#define PSX_BIT_SQUARE  15

/* Lookup: remap config value (0-13) -> PSX bit position
 * Maps qpsx_config.remap_* values to actual PSX controller bits */
static const uint8_t psx_remap_bits[14] = {
    PSX_BIT_CROSS,    /* 0 = X */
    PSX_BIT_CIRCLE,   /* 1 = O */
    PSX_BIT_SQUARE,   /* 2 = Square */
    PSX_BIT_TRI,      /* 3 = Triangle */
    PSX_BIT_L1,       /* 4 = L1 */
    PSX_BIT_R1,       /* 5 = R1 */
    PSX_BIT_L2,       /* 6 = L2 */
    PSX_BIT_R2,       /* 7 = R2 */
    PSX_BIT_START,    /* 8 = Start */
    PSX_BIT_SELECT,   /* 9 = Select */
    PSX_BIT_UP,       /* 10 = Up */
    PSX_BIT_DOWN,     /* 11 = Down */
    PSX_BIT_LEFT,     /* 12 = Left */
    PSX_BIT_RIGHT     /* 13 = Right */
};

/* Pre-computed remap lookup table
 * Index: RETRO_DEVICE_ID_JOYPAD_* (0-15)
 * Value: PSX bit position to clear
 *
 * Size: 16 bytes = half cache line - ALWAYS in L1 cache!
 * Aligned to 32 bytes to guarantee single cache line access.
 */
#ifdef __GNUC__
uint8_t remap_lut[16] __attribute__((aligned(32)));
#else
uint8_t remap_lut[16];
#endif

/* Forward declaration - implemented after qpsx_config is defined */
static void remap_rebuild_lut(void);

/* Paths */
static const char *retro_system_directory;
static const char *retro_save_directory;
static const char *retro_content_directory;

static char game_path[512];
static bool psx_initted = false;

volatile int skip_video_output = 0;

/* Framebuffer access */
extern uint16_t *SCREEN;

/* v335: Target Speed global - 40-100% (100 = normal, 40 = slowest) */
int g_target_speed = 100;

/* v338: Audio resampler type - 0=Linear(fast), 1=Cubic+LUT(slower,better) */
static int g_resampler_type = 0;

/* v335: Fixed-point audio resampler for target speed */
#define FP_SHIFT 16
#define FP_ONE (1 << FP_SHIFT)
#define TARGET_SAMPLES_PER_FRAME_NTSC (44100 / 60)  /* 735 */
#define TARGET_SAMPLES_PER_FRAME_PAL  (44100 / 50)  /* 882 */


/* v338: Catmull-Rom cubic spline LUT - 256 entries, 15-bit fixed point */
/* Coefficients for: result = c0*p0 + c1*p1 + c2*p2 + c3*p3 */
static const struct { int16_t c0, c1, c2, c3; } cubic_lut[256] = {
    {     0,  32767,      0,      0}, {   -64,  32767,     65,      0},
    {  -126,  32763,    132,     -1}, {  -188,  32757,    201,     -2},
    {  -248,  32748,    272,     -4}, {  -308,  32737,    345,     -6},
    {  -366,  32724,    419,     -9}, {  -424,  32708,    496,    -12},
    {  -480,  32690,    574,    -16}, {  -536,  32669,    655,    -20},
    {  -591,  32646,    737,    -24}, {  -645,  32621,    821,    -29},
    {  -698,  32593,    907,    -34}, {  -750,  32563,    995,    -40},
    {  -801,  32531,   1084,    -46}, {  -851,  32497,   1175,    -53},
    {  -900,  32460,   1268,    -60}, {  -948,  32421,   1363,    -67},
    {  -996,  32380,   1459,    -75}, { -1042,  32337,   1557,    -84},
    { -1088,  32291,   1657,    -92}, { -1133,  32244,   1758,   -101},
    { -1176,  32194,   1861,   -111}, { -1219,  32142,   1965,   -120},
    { -1262,  32088,   2072,   -130}, { -1303,  32033,   2179,   -141},
    { -1343,  31974,   2289,   -152}, { -1383,  31914,   2399,   -163},
    { -1421,  31852,   2512,   -175}, { -1459,  31788,   2626,   -186},
    { -1496,  31722,   2741,   -199}, { -1533,  31654,   2858,   -211},
    { -1568,  31584,   2976,   -224}, { -1603,  31512,   3096,   -237},
    { -1636,  31438,   3217,   -251}, { -1669,  31362,   3339,   -264},
    { -1702,  31285,   3463,   -278}, { -1733,  31205,   3589,   -293},
    { -1764,  31124,   3715,   -307}, { -1793,  31041,   3843,   -322},
    { -1822,  30956,   3972,   -338}, { -1851,  30869,   4103,   -353},
    { -1878,  30780,   4235,   -369}, { -1905,  30690,   4368,   -385},
    { -1931,  30598,   4502,   -401}, { -1956,  30504,   4638,   -417},
    { -1981,  30408,   4775,   -434}, { -2005,  30311,   4913,   -451},
    { -2028,  30212,   5052,   -468}, { -2050,  30111,   5192,   -485},
    { -2072,  30009,   5334,   -503}, { -2093,  29905,   5476,   -521},
    { -2113,  29800,   5620,   -539}, { -2133,  29693,   5765,   -557},
    { -2152,  29584,   5911,   -575}, { -2170,  29474,   6058,   -594},
    { -2188,  29362,   6206,   -612}, { -2204,  29249,   6354,   -631},
    { -2221,  29135,   6504,   -650}, { -2236,  29018,   6655,   -670},
    { -2251,  28901,   6807,   -689}, { -2265,  28782,   6960,   -709},
    { -2279,  28661,   7114,   -728}, { -2292,  28539,   7268,   -748},
    { -2304,  28416,   7424,   -768}, { -2316,  28291,   7580,   -788},
    { -2327,  28165,   7738,   -808}, { -2337,  28038,   7896,   -829},
    { -2347,  27909,   8055,   -849}, { -2356,  27779,   8215,   -869},
    { -2365,  27648,   8375,   -890}, { -2373,  27515,   8536,   -911},
    { -2380,  27382,   8698,   -932}, { -2387,  27246,   8861,   -952},
    { -2394,  27110,   9025,   -973}, { -2399,  26973,   9189,   -994},
    { -2405,  26834,   9354,  -1015}, { -2409,  26694,   9520,  -1036},
    { -2413,  26553,   9686,  -1058}, { -2417,  26411,   9853,  -1079},
    { -2420,  26268,  10020,  -1100}, { -2422,  26124,  10188,  -1121},
    { -2424,  25978,  10357,  -1143}, { -2426,  25832,  10526,  -1164},
    { -2427,  25684,  10696,  -1185}, { -2427,  25536,  10866,  -1207},
    { -2427,  25386,  11037,  -1228}, { -2427,  25236,  11208,  -1249},
    { -2426,  25084,  11380,  -1270}, { -2424,  24932,  11552,  -1292},
    { -2422,  24779,  11724,  -1313}, { -2419,  24624,  11897,  -1334},
    { -2416,  24469,  12071,  -1356}, { -2413,  24313,  12244,  -1377},
    { -2409,  24156,  12419,  -1398}, { -2405,  23999,  12593,  -1419},
    { -2400,  23840,  12768,  -1440}, { -2395,  23681,  12943,  -1461},
    { -2389,  23520,  13119,  -1482}, { -2383,  23359,  13294,  -1503},
    { -2377,  23198,  13470,  -1523}, { -2370,  23035,  13647,  -1544},
    { -2362,  22872,  13823,  -1565}, { -2355,  22708,  14000,  -1585},
    { -2346,  22544,  14176,  -1606}, { -2338,  22378,  14354,  -1626},
    { -2329,  22212,  14531,  -1646}, { -2320,  22046,  14708,  -1666},
    { -2310,  21879,  14885,  -1686}, { -2300,  21711,  15063,  -1706},
    { -2290,  21542,  15241,  -1725}, { -2279,  21373,  15418,  -1745},
    { -2268,  21204,  15596,  -1764}, { -2257,  21034,  15774,  -1783},
    { -2245,  20863,  15952,  -1802}, { -2233,  20692,  16129,  -1821},
    { -2220,  20521,  16307,  -1840}, { -2208,  20349,  16485,  -1858},
    { -2195,  20177,  16662,  -1876}, { -2181,  20004,  16840,  -1895},
    { -2168,  19830,  17018,  -1912}, { -2154,  19657,  17195,  -1930},
    { -2139,  19483,  17372,  -1948}, { -2125,  19309,  17549,  -1965},
    { -2110,  19134,  17726,  -1982}, { -2095,  18959,  17903,  -1999},
    { -2079,  18783,  18080,  -2016}, { -2064,  18608,  18256,  -2032},
    { -2048,  18432,  18432,  -2048}, { -2032,  18256,  18608,  -2064},
    { -2016,  18080,  18783,  -2079}, { -1999,  17903,  18959,  -2095},
    { -1982,  17726,  19134,  -2110}, { -1965,  17549,  19309,  -2125},
    { -1948,  17372,  19483,  -2139}, { -1930,  17195,  19657,  -2154},
    { -1912,  17018,  19830,  -2168}, { -1895,  16840,  20004,  -2181},
    { -1876,  16662,  20177,  -2195}, { -1858,  16485,  20349,  -2208},
    { -1840,  16307,  20521,  -2220}, { -1821,  16129,  20692,  -2233},
    { -1802,  15952,  20863,  -2245}, { -1783,  15774,  21034,  -2257},
    { -1764,  15596,  21204,  -2268}, { -1745,  15418,  21373,  -2279},
    { -1725,  15241,  21542,  -2290}, { -1706,  15063,  21711,  -2300},
    { -1686,  14885,  21879,  -2310}, { -1666,  14708,  22046,  -2320},
    { -1646,  14531,  22212,  -2329}, { -1626,  14354,  22378,  -2338},
    { -1606,  14176,  22544,  -2346}, { -1585,  14000,  22708,  -2355},
    { -1565,  13823,  22872,  -2362}, { -1544,  13647,  23035,  -2370},
    { -1523,  13470,  23198,  -2377}, { -1503,  13294,  23359,  -2383},
    { -1482,  13119,  23520,  -2389}, { -1461,  12943,  23681,  -2395},
    { -1440,  12768,  23840,  -2400}, { -1419,  12593,  23999,  -2405},
    { -1398,  12419,  24156,  -2409}, { -1377,  12244,  24313,  -2413},
    { -1356,  12071,  24469,  -2416}, { -1334,  11897,  24624,  -2419},
    { -1313,  11724,  24779,  -2422}, { -1292,  11552,  24932,  -2424},
    { -1270,  11380,  25084,  -2426}, { -1249,  11208,  25236,  -2427},
    { -1228,  11037,  25386,  -2427}, { -1207,  10866,  25536,  -2427},
    { -1185,  10696,  25684,  -2427}, { -1164,  10526,  25832,  -2426},
    { -1143,  10357,  25978,  -2424}, { -1121,  10188,  26124,  -2422},
    { -1100,  10020,  26268,  -2420}, { -1079,   9853,  26411,  -2417},
    { -1058,   9686,  26553,  -2413}, { -1036,   9520,  26694,  -2409},
    { -1015,   9354,  26834,  -2405}, {  -994,   9189,  26973,  -2399},
    {  -973,   9025,  27110,  -2394}, {  -952,   8861,  27246,  -2387},
    {  -932,   8698,  27382,  -2380}, {  -911,   8536,  27515,  -2373},
    {  -890,   8375,  27648,  -2365}, {  -869,   8215,  27779,  -2356},
    {  -849,   8055,  27909,  -2347}, {  -829,   7896,  28038,  -2337},
    {  -808,   7738,  28165,  -2327}, {  -788,   7580,  28291,  -2316},
    {  -768,   7424,  28416,  -2304}, {  -748,   7268,  28539,  -2292},
    {  -728,   7114,  28661,  -2279}, {  -709,   6960,  28782,  -2265},
    {  -689,   6807,  28901,  -2251}, {  -670,   6655,  29018,  -2236},
    {  -650,   6504,  29135,  -2221}, {  -631,   6354,  29249,  -2204},
    {  -612,   6206,  29362,  -2188}, {  -594,   6058,  29474,  -2170},
    {  -575,   5911,  29584,  -2152}, {  -557,   5765,  29693,  -2133},
    {  -539,   5620,  29800,  -2113}, {  -521,   5476,  29905,  -2093},
    {  -503,   5334,  30009,  -2072}, {  -485,   5192,  30111,  -2050},
    {  -468,   5052,  30212,  -2028}, {  -451,   4913,  30311,  -2005},
    {  -434,   4775,  30408,  -1981}, {  -417,   4638,  30504,  -1956},
    {  -401,   4502,  30598,  -1931}, {  -385,   4368,  30690,  -1905},
    {  -369,   4235,  30780,  -1878}, {  -353,   4103,  30869,  -1851},
    {  -338,   3972,  30956,  -1822}, {  -322,   3843,  31041,  -1793},
    {  -307,   3715,  31124,  -1764}, {  -293,   3589,  31205,  -1733},
    {  -278,   3463,  31285,  -1702}, {  -264,   3339,  31362,  -1669},
    {  -251,   3217,  31438,  -1636}, {  -237,   3096,  31512,  -1603},
    {  -224,   2976,  31584,  -1568}, {  -211,   2858,  31654,  -1533},
    {  -199,   2741,  31722,  -1496}, {  -186,   2626,  31788,  -1459},
    {  -175,   2512,  31852,  -1421}, {  -163,   2399,  31914,  -1383},
    {  -152,   2289,  31974,  -1343}, {  -141,   2179,  32033,  -1303},
    {  -130,   2072,  32088,  -1262}, {  -120,   1965,  32142,  -1219},
    {  -111,   1861,  32194,  -1176}, {  -101,   1758,  32244,  -1133},
    {   -92,   1657,  32291,  -1088}, {   -84,   1557,  32337,  -1042},
    {   -75,   1459,  32380,   -996}, {   -67,   1363,  32421,   -948},
    {   -60,   1268,  32460,   -900}, {   -53,   1175,  32497,   -851},
    {   -46,   1084,  32531,   -801}, {   -40,    995,  32563,   -750},
    {   -34,    907,  32593,   -698}, {   -29,    821,  32621,   -645},
    {   -24,    737,  32646,   -591}, {   -20,    655,  32669,   -536},
    {   -16,    574,  32690,   -480}, {   -12,    496,  32708,   -424},
    {    -9,    419,  32724,   -366}, {    -6,    345,  32737,   -308},
    {    -4,    272,  32748,   -248}, {    -2,    201,  32757,   -188},
    {    -1,    132,  32763,   -126}, {     0,     65,  32767,    -64}
};

/* Forward declaration */
static void resample_linear_fixed(const int16_t* input, int in_samples, int16_t* output, int out_samples);

/* v338: Cubic resampler using LUT - better quality, slower than linear */
static void resample_cubic_lut(const int16_t* input, int in_samples,
                                int16_t* output, int out_samples)
{
    if (in_samples <= 3 || out_samples <= 1) {
        /* Fallback to linear for edge cases */
        resample_linear_fixed(input, in_samples, output, out_samples);
        return;
    }

    uint32_t ratio = ((uint32_t)(in_samples - 1) << FP_SHIFT) / (uint32_t)(out_samples - 1);
    uint32_t pos = 0;

    for (int i = 0; i < out_samples; i++) {
        int idx = pos >> FP_SHIFT;
        int lut_idx = (pos >> 8) & 0xFF;  /* Top 8 bits of fraction for LUT */

        /* Get 4 sample indices with boundary clamping */
        int i0 = (idx > 0) ? idx - 1 : 0;
        int i1 = idx;
        int i2 = (idx + 1 < in_samples) ? idx + 1 : in_samples - 1;
        int i3 = (idx + 2 < in_samples) ? idx + 2 : in_samples - 1;

        /* Left channel - cubic interpolation */
        int32_t l0 = input[i0 * 2];
        int32_t l1 = input[i1 * 2];
        int32_t l2 = input[i2 * 2];
        int32_t l3 = input[i3 * 2];
        int32_t left = (l0 * cubic_lut[lut_idx].c0 + l1 * cubic_lut[lut_idx].c1 +
                        l2 * cubic_lut[lut_idx].c2 + l3 * cubic_lut[lut_idx].c3) >> 15;

        /* Right channel */
        int32_t r0 = input[i0 * 2 + 1];
        int32_t r1 = input[i1 * 2 + 1];
        int32_t r2 = input[i2 * 2 + 1];
        int32_t r3 = input[i3 * 2 + 1];
        int32_t right = (r0 * cubic_lut[lut_idx].c0 + r1 * cubic_lut[lut_idx].c1 +
                         r2 * cubic_lut[lut_idx].c2 + r3 * cubic_lut[lut_idx].c3) >> 15;

        /* Clamp to int16_t range */
        if (left > 32767) left = 32767;
        if (left < -32768) left = -32768;
        if (right > 32767) right = 32767;
        if (right < -32768) right = -32768;

        output[i * 2] = (int16_t)left;
        output[i * 2 + 1] = (int16_t)right;

        pos += ratio;
    }
}
static int16_t g_resample_buffer[2048 * 3];  /* v354: Enlarged buffer for 40% speed (was 2048, overflow at <72%) */

/* Fixed-point linear resampler - optimized for MIPS32 (no FPU) */
static void resample_linear_fixed(const int16_t* input, int in_samples,
                                   int16_t* output, int out_samples)
{
    if (in_samples <= 1 || out_samples <= 1) {
        /* Edge case: just copy what we have */
        for (int i = 0; i < out_samples && i < in_samples; i++) {
            output[i*2] = input[i*2];
            output[i*2+1] = input[i*2+1];
        }
        return;
    }

    /* Fixed-point ratio: (in-1) / (out-1) * 65536 */
    uint32_t ratio = ((uint32_t)(in_samples - 1) << FP_SHIFT) / (uint32_t)(out_samples - 1);
    uint32_t pos = 0;

    for (int i = 0; i < out_samples; i++) {
        int idx = pos >> FP_SHIFT;
        uint32_t frac = pos & (FP_ONE - 1);
        uint32_t inv_frac = FP_ONE - frac;

        if (idx + 1 < in_samples) {
            /* Left channel - linear interpolation */
            output[i*2] = (int16_t)(
                ((int32_t)input[idx*2] * inv_frac +
                 (int32_t)input[(idx+1)*2] * frac) >> FP_SHIFT);
            /* Right channel */
            output[i*2+1] = (int16_t)(
                ((int32_t)input[idx*2+1] * inv_frac +
                 (int32_t)input[(idx+1)*2+1] * frac) >> FP_SHIFT);
        } else {
            /* Edge: use last sample */
            output[i*2] = input[(in_samples-1)*2];
            output[i*2+1] = input[(in_samples-1)*2+1];
        }

        pos += ratio;
    }
}

/* Audio output */
extern "C" void retro_audio_cb(int16_t *buf, int samples)
{
    if (!audio_batch_cb) return;

    /*
     * v350: AUDIO STRETCHING FOR INTENTIONAL SLOW MOTION
     * When g_target_speed < 100, use resampling (with pitch change).
     */
    if (g_target_speed < 100 && g_target_speed >= 40 && samples > 0) {
        int stretched = (samples * 100) / g_target_speed;
        if (stretched > 3072) stretched = 3072;
        if (g_resampler_type == 1)
            resample_cubic_lut(buf, samples, g_resample_buffer, stretched);
        else
            resample_linear_fixed(buf, samples, g_resample_buffer, stretched);
        audio_batch_cb(g_resample_buffer, stretched);
        return;
    }

    /* Normal output */
    audio_batch_cb(buf, samples);
}

#define QPSX_VERSION "374"
#define QPSX_GLOBAL_CONFIG_PATH "/mnt/sda1/cores/config/pcsx4all.cfg"
#define QPSX_NATIVE_CONFIG_PATH "/mnt/sda1/cores/config/psx_native.cfg"
#define QPSX_ASM_CONFIG_PATH "/mnt/sda1/cores/config/psx_asm.cfg"
#define QPSX_CRASH_MARKER_PATH "/mnt/sda1/cores/config/psx_crash.tmp"

/* ============== v353: BITFLAGS FOR BOOLEAN OPTIONS ============== */
/* Pack ON/OFF options into bytes instead of ints: 8 options = 1 byte vs 32 bytes
 * NOTE: Multi-value options (gpu_blending 0-3, gpu_interlace_force 0-2) stay as int
 * Only TRUE boolean (ON/OFF) options are packed.
 * SAVES: 14 booleans * 4 bytes = 56 bytes -> 3 bytes = 53 bytes saved!
 */

/* GPU_FLAGS (1 byte = 8 GPU boolean options) */
#define GPU_FLAG_DITHERING     (1 << 0)  /* bit 0: dithering */
#define GPU_FLAG_LIGHTING      (1 << 1)  /* bit 1: lighting */
#define GPU_FLAG_ASM_LIGHT     (1 << 2)  /* bit 2: asm lighting (merged fast+asm) */
#define GPU_FLAG_ASM_BLEND     (1 << 3)  /* bit 3: asm blending (merged fast+asm) */
#define GPU_FLAG_PIXEL_SKIP    (1 << 4)  /* bit 4: pixel skip */
#define GPU_FLAG_PROG_ILACE    (1 << 5)  /* bit 5: progressive interlace */
#define GPU_FLAG_FRAME_DUP     (1 << 6)  /* bit 6: frame duping */
#define GPU_FLAG_PREFETCH_TEX  (1 << 7)  /* bit 7: texture prefetch */

/* AUDIO_FLAGS (1 byte = 8 audio/system boolean options) */
#define AUDIO_FLAG_XA_DISABLE    (1 << 0)  /* bit 0: disable XA audio */
#define AUDIO_FLAG_CDDA_DISABLE  (1 << 1)  /* bit 1: disable CDDA audio */
#define AUDIO_FLAG_CDDA_BINSAV   (1 << 2)  /* bit 2: use .binadpcm/.binwav */
#define AUDIO_FLAG_USE_BIOS      (1 << 3)  /* bit 3: use real BIOS */
/* bits 4-7: reserved */

/* DEBUG_FLAGS (1 byte = 8 debug/display boolean options) */
#define DEBUG_FLAG_SHOW_FPS    (1 << 0)  /* bit 0: show FPS */
#define DEBUG_FLAG_PROFILER    (1 << 1)  /* bit 1: profiler overlay */
#define DEBUG_FLAG_LOG         (1 << 2)  /* bit 2: debug log to file */

/* v368: Input flags */
#define INPUT_FLAG_PLAYER2     (1 << 0)  /* bit 0: enable player 2 (default OFF) */
/* bits 3-7: reserved */

/* === BITFLAG HELPER MACROS === */
/* Read flag - returns 0 or non-zero (truthy) */
#define FLAG_GET(flags, bit)     ((flags) & (bit))

/* Read flag - returns exactly 0 or 1 */
#define FLAG_BOOL(flags, bit)    (((flags) & (bit)) ? 1 : 0)

/* Set flag ON */
#define FLAG_SET(flags, bit)     ((flags) |= (bit))

/* Set flag OFF */
#define FLAG_CLEAR(flags, bit)   ((flags) &= ~(bit))

/* Toggle flag */
#define FLAG_TOGGLE(flags, bit)  ((flags) ^= (bit))

/* Write flag to specific value (0 or 1) */
#define FLAG_WRITE(flags, bit, val) \
    do { if (val) FLAG_SET(flags, bit); else FLAG_CLEAR(flags, bit); } while(0)

/* v320: Safe mode - detect crash on startup and open config menu */
static int safe_mode_triggered = 0;
static int safe_mode_frames = 0;
#define SAFE_MODE_FRAMES 175  /* Mark as stable after 3.5 seconds (~175 frames at 50fps) */

static void safe_mode_write_marker(void) {
    FILE *f = fopen(QPSX_CRASH_MARKER_PATH, "w");
    if (f) {
        fprintf(f, "1\n");
        fclose(f);
    }
}

static void safe_mode_remove_marker(void) {
    remove(QPSX_CRASH_MARKER_PATH);
}

static int safe_mode_check_marker(void) {
    FILE *f = fopen(QPSX_CRASH_MARKER_PATH, "r");
    if (f) {
        fclose(f);
        return 1;  /* Crash marker exists - previous run crashed */
    }
    return 0;
}

/* ============== CD SWAP FILE BROWSER (v284) ============== */
/*
 * v284: Simplified - only show .cue files in game directory
 * No subdirectory navigation, just pick another disc from same folder
 */
#define FB_MAX_FILES 128        /* Only .cue files in one directory */
#define FB_MAX_PATH 256
#define FB_MAX_NAME 128
#define FB_VISIBLE_ITEMS 17     /* More items since no path bar */

static int cd_browser_active = 0;
static char fb_game_dir[FB_MAX_PATH];   /* Directory of loaded game */
static char fb_files[FB_MAX_FILES][FB_MAX_NAME];
static int fb_file_count = 0;
static int fb_selection = 0;
static int fb_scroll = 0;

/* FS functions from firmware */
extern "C" int fs_opendir(const char *path);
extern "C" int fs_readdir(int fd, void *buf);
extern "C" int fs_closedir(int fd);

/* CD-ROM functions for disc swap - declared in plugins.h and cdrom.h */
extern char CdromId[10];
extern char CdromLabel[33];
extern void (*cdrIsoMultidiskCallback)(void);

#ifdef PSXREC
extern u32 cycle_multiplier;
#endif

/* ============== NATIVE COMMAND MENU (v244) ============== */
static int native_cmd_menu_active = 0;       /* Menu is currently displayed */
static int native_cmd_menu_shown = 0;        /* Menu was already shown this session */
static int native_cmd_menu_scroll = 0;       /* Scroll offset */
static int native_cmd_menu_selected = 0;     /* Selected item index */
static int native_cmd_menu_enable = 1;       /* Show menu at startup (from config) */

/* Command list - embedded here for simplicity */
#define NCMD_MAX 128
typedef struct {
    const char* name;
    int opcode;      /* -1 = special entry (START, separator) */
    int implemented; /* 1 = implemented, 0 = dynarec only */
    int enabled;     /* User toggle */
} native_cmd_entry_t;

static native_cmd_entry_t native_cmd_list[NCMD_MAX] = {
    {">>> START EMULATOR <<<", -1, 1, 1},
    {"--- Save/Reset ---", -2, 0, 0},
    {"SAVE OPTIONS", -3, 1, 1},          /* opcode -3 = save action */
    {"RESET TO DEFAULTS", -4, 1, 1},     /* opcode -4 = reset action */
    {"--- ALU Immediate ---", -2, 0, 0},
    {"ADDI",   0x08, 1, 1},
    {"ADDIU",  0x09, 1, 1},
    {"SLTI",   0x0A, 1, 1},
    {"SLTIU",  0x0B, 1, 1},
    {"ANDI",   0x0C, 1, 1},
    {"ORI",    0x0D, 1, 1},
    {"XORI",   0x0E, 1, 1},
    {"LUI",    0x0F, 1, 1},
    {"--- ALU Register ---", -2, 0, 0},
    {"ADD",    0x220, 1, 1},
    {"ADDU",   0x221, 1, 1},
    {"SUB",    0x222, 1, 1},
    {"SUBU",   0x223, 1, 1},
    {"AND",    0x224, 1, 1},
    {"OR",     0x225, 1, 1},
    {"XOR",    0x226, 1, 1},
    {"NOR",    0x227, 1, 1},
    {"SLT",    0x22A, 1, 1},
    {"SLTU",   0x22B, 1, 1},
    {"--- Shift ---", -2, 0, 0},
    {"SLL",    0x200, 1, 1},
    {"SRL",    0x202, 1, 1},
    {"SRA",    0x203, 1, 1},
    {"SLLV",   0x204, 1, 1},
    {"SRLV",   0x206, 1, 1},
    {"SRAV",   0x207, 1, 1},
    {"--- Branch ---", -2, 0, 0},
    {"BEQ",    0x04, 1, 1},
    {"BNE",    0x05, 1, 1},
    {"BLEZ",   0x06, 1, 1},
    {"BGTZ",   0x07, 1, 1},
    {"BLTZ",   0x100, 1, 1},
    {"BGEZ",   0x101, 1, 1},
    {"--- Jump ---", -2, 0, 0},
    {"J",      0x02, 1, 1},
    {"JAL",    0x03, 1, 1},
    {"JR",     0x208, 1, 1},
    {"JALR",   0x209, 1, 1},
    {"--- Load ---", -2, 0, 0},
    {"LB",     0x20, 1, 1},
    {"LH",     0x21, 1, 1},
    {"LW",     0x23, 1, 1},
    {"LBU",    0x24, 1, 1},
    {"LHU",    0x25, 1, 1},
    {"LWL",    0x22, 0, 0},  /* Not implemented */
    {"LWR",    0x26, 0, 0},  /* Not implemented */
    {"--- Store ---", -2, 0, 0},
    {"SB",     0x28, 1, 1},
    {"SH",     0x29, 1, 1},
    {"SW",     0x2B, 1, 1},
    {"SWL",    0x2A, 0, 0},  /* Not implemented */
    {"SWR",    0x2E, 0, 0},  /* Not implemented */
    {"--- COP0 ---", -2, 0, 0},
    {"MFC0",   0x10, 1, 1},
    {"MTC0",   0x10, 1, 1},
    {"RFE",    0x10, 1, 1},
    {"--- Mul/Div (dynarec) ---", -2, 0, 0},
    {"MULT",   0x218, 0, 0},
    {"MULTU",  0x219, 0, 0},
    {"DIV",    0x21A, 0, 0},
    {"DIVU",   0x21B, 0, 0},
    {NULL, 0, 0, 0}  /* End marker */
};

static int native_cmd_count = 0;  /* Set during init */

/* v245: Quick check - 1 if ANY native command enabled, 0 if ALL disabled */
extern "C" int native_any_cmd_enabled = 1;

static void native_cmd_update_any_enabled(void) {
    native_any_cmd_enabled = 0;
    for (int i = 0; i < native_cmd_count; i++) {
        if (native_cmd_list[i].opcode >= 0 &&
            native_cmd_list[i].implemented &&
            native_cmd_list[i].enabled) {
            native_any_cmd_enabled = 1;
            return;
        }
    }
}

/* v244: Check if a PSX instruction is enabled for native compilation
 * Called from rec_native.cpp.h to implement menu toggles
 * Returns: 1 = enabled (compile native), 0 = disabled (use dynarec)
 */
extern "C" int native_cmd_check_enabled(u32 psx_opcode)
{
    u32 op = (psx_opcode >> 26) & 0x3F;
    u32 funct = psx_opcode & 0x3F;
    u32 rt = (psx_opcode >> 16) & 0x1F;

    /* Map PSX opcode to our menu entry opcode */
    int lookup_code = -999;  /* Not found */

    switch (op) {
        case 0x00: /* SPECIAL - use funct with 0x200 prefix to avoid I-type collision!
                    * v249 FIX: JR(funct=0x08) collided with ADDI(op=0x08)!
                    * JALR(funct=0x09) collided with ADDIU(op=0x09)!
                    * ADD(funct=0x20) collided with LB(op=0x20)! etc.
                    * Now: R-type funct -> 0x200 + funct */
            switch (funct) {
                case 0x00: case 0x02: case 0x03: /* SLL, SRL, SRA */
                case 0x04: case 0x06: case 0x07: /* SLLV, SRLV, SRAV */
                    lookup_code = 0x200 + funct; break;  /* Shift */
                case 0x08: lookup_code = 0x208; break; /* JR - was 0x08, collided with ADDI! */
                case 0x09: lookup_code = 0x209; break; /* JALR - was 0x09, collided with ADDIU! */
                case 0x18: case 0x19: case 0x1A: case 0x1B: /* MULT/DIV */
                    lookup_code = 0x200 + funct; break;
                case 0x20: case 0x21: case 0x22: case 0x23: /* ADD/SUB */
                case 0x24: case 0x25: case 0x26: case 0x27: /* AND/OR/XOR/NOR */
                case 0x2A: case 0x2B: /* SLT/SLTU */
                    lookup_code = 0x200 + funct; break;  /* ALU R-type */
            }
            break;
        case 0x01: /* REGIMM - BLTZ/BGEZ etc */
            if ((rt & 0x01) == 0) lookup_code = 0x100; /* BLTZ/BLTZAL */
            else lookup_code = 0x101; /* BGEZ/BGEZAL */
            break;
        case 0x02: lookup_code = 0x02; break; /* J */
        case 0x03: lookup_code = 0x03; break; /* JAL */
        case 0x04: lookup_code = 0x04; break; /* BEQ */
        case 0x05: lookup_code = 0x05; break; /* BNE */
        case 0x06: lookup_code = 0x06; break; /* BLEZ */
        case 0x07: lookup_code = 0x07; break; /* BGTZ */
        case 0x08: lookup_code = 0x08; break; /* ADDI */
        case 0x09: lookup_code = 0x09; break; /* ADDIU */
        case 0x0A: lookup_code = 0x0A; break; /* SLTI */
        case 0x0B: lookup_code = 0x0B; break; /* SLTIU */
        case 0x0C: lookup_code = 0x0C; break; /* ANDI */
        case 0x0D: lookup_code = 0x0D; break; /* ORI */
        case 0x0E: lookup_code = 0x0E; break; /* XORI */
        case 0x0F: lookup_code = 0x0F; break; /* LUI */
        case 0x10: lookup_code = 0x10; break; /* COP0 */
        case 0x20: lookup_code = 0x20; break; /* LB */
        case 0x21: lookup_code = 0x21; break; /* LH */
        case 0x22: lookup_code = 0x22; break; /* LWL */
        case 0x23: lookup_code = 0x23; break; /* LW */
        case 0x24: lookup_code = 0x24; break; /* LBU */
        case 0x25: lookup_code = 0x25; break; /* LHU */
        case 0x26: lookup_code = 0x26; break; /* LWR */
        case 0x28: lookup_code = 0x28; break; /* SB */
        case 0x29: lookup_code = 0x29; break; /* SH */
        case 0x2A: lookup_code = 0x2A; break; /* SWL */
        case 0x2B: lookup_code = 0x2B; break; /* SW */
        case 0x2E: lookup_code = 0x2E; break; /* SWR */
        /* v272: COP2/GTE instructions have trap handlers - allow them! */
        case 0x12: return 1; /* COP2 - emit_cop2_trap handles this */
        case 0x32: return 1; /* LWC2 - emit_lwc2 handles this */
        case 0x3A: return 1; /* SWC2 - emit_swc2 handles this */
    }

    /* Search for this opcode in menu and check enabled status */
    for (int i = 0; i < native_cmd_count; i++) {
        if (native_cmd_list[i].opcode == lookup_code) {
            /* Found - return enabled status */
            /* If not implemented, always return 0 (dynarec) */
            if (!native_cmd_list[i].implemented) return 0;
            return native_cmd_list[i].enabled;
        }
    }

    /* Not in menu = not implemented = use dynarec */
    return 0;
}

/* ============== ASM SELECT MENU (v285) ============== */
/*
 * Dynarec ASM Optimization Menu
 *
 * Purpose: Toggle individual opcode ASM replacements for debugging.
 * All items start GRAY (not implemented) - will be enabled as we port opcodes.
 *
 * Unlike Native Mode (block-level decision), ASM Select allows per-opcode
 * toggling, making it easy to isolate crashes during ASM porting.
 */
static int asm_cmd_menu_active = 0;       /* Menu is currently displayed */
static int asm_cmd_menu_scroll = 0;       /* Scroll offset */
static int asm_cmd_menu_selected = 0;     /* Selected item index */

/* ASM Select list - all opcodes for potential ASM optimization */
#define ASMCMD_MAX 128
typedef struct {
    const char* name;
    int opcode;      /* -1=START, -2=separator, -3=save, -4=reset */
    int implemented; /* 1 = ASM version exists, 0 = not yet ported */
    int enabled;     /* User toggle (only works if implemented) */
} asm_cmd_entry_t;

static asm_cmd_entry_t asm_cmd_list[ASMCMD_MAX] = {
    {">>> START EMULATOR <<<", -1, 1, 1},
    {"--- Save/Reset ---", -2, 0, 0},
    {"SAVE OPTIONS", -3, 1, 1},
    {"RESET TO DEFAULTS", -4, 1, 1},
    {"", -2, 0, 0},  /* Empty separator */
    {"=== PHASE 1: ALU ===", -2, 0, 0},
    {"--- ALU Immediate ---", -2, 0, 0},
    {"ADDIU",  0x09, 1, 0},   /* v286: ASM implemented! */
    {"ADDI",   0x08, 1, 0},   /* v288: Uses ADDIU (same function) */
    {"SLTI",   0x0A, 1, 0},   /* v288: ASM implemented! */
    {"SLTIU",  0x0B, 1, 0},   /* v288: ASM implemented! */
    {"ANDI",   0x0C, 1, 0},   /* v288: ASM implemented! */
    {"ORI",    0x0D, 1, 0},   /* v288: ASM implemented! */
    {"XORI",   0x0E, 1, 0},   /* v288: ASM implemented! */
    {"LUI",    0x0F, 1, 0},   /* v286: ASM implemented! */
    {"--- ALU Register ---", -2, 0, 0},
    {"ADDU",   0x221, 1, 0},  /* v288: ASM implemented! */
    {"ADD",    0x220, 1, 0},  /* v288: Uses ADDU (same function) */
    {"SUBU",   0x223, 1, 0},  /* v288: ASM implemented! */
    {"SUB",    0x222, 1, 0},  /* v288: Uses SUBU (same function) */
    {"AND",    0x224, 1, 0},  /* v288: ASM implemented! */
    {"OR",     0x225, 1, 0},  /* v288: ASM implemented! */
    {"XOR",    0x226, 1, 0},  /* v288: ASM implemented! */
    {"NOR",    0x227, 1, 0},  /* v288: ASM implemented! */
    {"SLT",    0x22A, 1, 0},  /* v288: ASM implemented! */
    {"SLTU",   0x22B, 1, 0},  /* v288: ASM implemented! */
    {"--- Shift ---", -2, 0, 0},
    {"SLL",    0x200, 1, 0},  /* v289: ASM implemented! */
    {"SRL",    0x202, 1, 0},  /* v289: ASM implemented! */
    {"SRA",    0x203, 1, 0},  /* v289: ASM implemented! */
    {"SLLV",   0x204, 1, 0},  /* v289: ASM implemented! */
    {"SRLV",   0x206, 1, 0},  /* v289: ASM implemented! */
    {"SRAV",   0x207, 1, 0},  /* v289: ASM implemented! */
    {"", -2, 0, 0},
    {"=== PHASE 2: MEMORY ===", -2, 0, 0},
    {"--- Load ---", -2, 0, 0},
    {"LW",     0x23, 1, 0},  /* v290: ASM implemented! */
    {"LH",     0x21, 1, 0},  /* v290: ASM implemented! */
    {"LHU",    0x25, 1, 0},  /* v290: ASM implemented! */
    {"LB",     0x20, 1, 0},  /* v290: ASM implemented! */
    {"LBU",    0x24, 1, 0},  /* v290: ASM implemented! */
    {"LWL",    0x22, 1, 0},  /* v290: ASM implemented! */
    {"LWR",    0x26, 1, 0},  /* v290: ASM implemented! */
    {"--- Store ---", -2, 0, 0},
    {"SW",     0x2B, 1, 0},  /* v290: ASM implemented! */
    {"SH",     0x29, 1, 0},  /* v290: ASM implemented! */
    {"SB",     0x28, 1, 0},  /* v290: ASM implemented! */
    {"SWL",    0x2A, 1, 0},  /* v290: ASM implemented! */
    {"SWR",    0x2E, 1, 0},  /* v290: ASM implemented! */
    {"", -2, 0, 0},
    {"=== PHASE 3: CONTROL ===", -2, 0, 0},
    {"--- Branch ---", -2, 0, 0},
    {"BEQ",    0x04, 1, 0},   /* v291: ASM implemented! */
    {"BNE",    0x05, 1, 0},   /* v291: ASM implemented! */
    {"BLEZ",   0x06, 1, 0},   /* v291: ASM implemented! */
    {"BGTZ",   0x07, 1, 0},   /* v291: ASM implemented! */
    {"BLTZ",   0x100, 1, 0},  /* v291: ASM implemented! */
    {"BGEZ",   0x101, 1, 0},  /* v291: ASM implemented! */
    {"BLTZAL", 0x110, 1, 0},  /* v291: ASM implemented! */
    {"BGEZAL", 0x111, 1, 0},  /* v291: ASM implemented! */
    {"--- Jump ---", -2, 0, 0},
    {"J",      0x02, 1, 0},   /* v291: ASM implemented! */
    {"JAL",    0x03, 1, 0},   /* v291: ASM implemented! */
    {"JR",     0x208, 1, 0},  /* v291: ASM implemented! */
    {"JALR",   0x209, 1, 0},  /* v291: ASM implemented! */
    {"", -2, 0, 0},
    {"=== PHASE 4: MDU ===", -2, 0, 0},
    {"--- Multiply/Divide ---", -2, 0, 0},
    {"MULT",   0x218, 1, 0},  /* v292: ASM implemented! */
    {"MULTU",  0x219, 1, 0},  /* v292: ASM implemented! */
    {"DIV",    0x21A, 1, 0},  /* v292: ASM implemented! */
    {"DIVU",   0x21B, 1, 0},  /* v292: ASM implemented! */
    {"MFHI",   0x210, 1, 0},  /* v292: ASM implemented! */
    {"MFLO",   0x212, 1, 0},  /* v292: ASM implemented! */
    {"MTHI",   0x211, 1, 0},  /* v292: ASM implemented! */
    {"MTLO",   0x213, 1, 0},  /* v292: ASM implemented! */
    {"", -2, 0, 0},
    {"=== PHASE 5: SYSTEM ===", -2, 0, 0},
    {"--- COP0 ---", -2, 0, 0},
    {"MFC0",   0x400, 1, 0},  /* v293: ASM implemented! */
    {"MTC0",   0x404, 1, 0},  /* v293: ASM implemented! */
    {"CFC0",   0x402, 1, 0},  /* v293: ASM implemented! */
    {"CTC0",   0x406, 1, 0},  /* v293: ASM implemented! */
    {"RFE",    0x410, 1, 0},  /* v293: ASM implemented! */
    {"--- GTE Transfer ---", -2, 0, 0},
    {"MFC2",   0x480, 1, 0},  /* v293: ASM implemented! */
    {"MTC2",   0x484, 1, 0},  /* v293: ASM implemented! */
    {"CFC2",   0x482, 1, 0},  /* v293: ASM implemented! */
    {"CTC2",   0x486, 1, 0},  /* v293: ASM implemented! */
    {"LWC2",   0x32, 1, 0},   /* v293: ASM implemented! */
    {"SWC2",   0x3A, 1, 0},   /* v293: ASM implemented! */
    {"--- GTE Runtime ---", -2, 0, 0},
    {"RTPT NoFlags", -10, 1, 0},  /* v294: Runtime ASM (function pointer) */
    {"MVMVA NoFlags", -11, 1, 0}, /* v294: Runtime ASM (function pointer) */
    {"NCLIP",  0x4C0, 0, 0},  /* Dynarec - already inline! */
    {"RTPS",   0x4C1, 0, 0},  /* Dynarec - not implemented */
    {"--- Special ---", -2, 0, 0},
    {"SYSCALL", 0x20C, 0, 0},
    {"BREAK",   0x20D, 0, 0},
    {"", -2, 0, 0},
    {"=== SUBSYSTEMS ===", -2, 0, 0},
    {"RegCache", -5, 0, 0},   /* Special: register caching system */
    {"AddrConv", -6, 0, 0},   /* Special: address conversion */
    {"CodeInv",  -7, 0, 0},   /* Special: code invalidation */
    {"LoHiCache", -8, 0, 0},  /* Special: LO/HI register cache */
    {NULL, 0, 0, 0}  /* End marker */
};

static int asm_cmd_count = 0;  /* Set during init */

/* Initialize ASM command count - called from retro_run() frame 0 */
static void asm_cmd_init(void) {
    asm_cmd_count = 0;
    for (int i = 0; i < ASMCMD_MAX && asm_cmd_list[i].name != NULL; i++) {
        asm_cmd_count++;
    }
    XLOG("asm_cmd: init, %d entries", asm_cmd_count);
}

/* ASM Select menu colors and functions are defined after RGB565 macro (below) */

/* ============== FPS COUNTER ============== */
static int fps_show = 0;
static int fps_current = 0;
static int fps_frame_count = 0;
static unsigned long fps_last_time = 0;

/* v295: GPU FPS counter - measures ACTUAL rendered GPU frames */
extern volatile int gpu_frame_count;  /* Defined in port.cpp */
static int gpu_fps_current = 0;
static int gpu_fps_snapshot = 0;

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
#define MENU_YELLOW  RGB565(255, 255, 0)    /* v367: CDDA PRETEND mode color */
#define MENU_WARN    RGB565(255, 170, 170)  /* v339: warning color #FAA */
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

/* ============== v258: CDDA TOOLS SUBMENU ============== */
static int cdda_tools_active = 0;        /* Submenu open */
static int cdda_tools_item = 0;          /* Selected track/action */
static int cdda_tools_scroll = 0;        /* Scroll offset */
static int cdda_tools_action = 0;        /* 0=convert, 1=delete */
static int cdda_tools_converting = 0;    /* Conversion in progress */
static int cdda_tools_progress_track = 0;
static int cdda_tools_progress_total = 0;
static int cdda_tools_progress_sector = 0;
static int cdda_tools_progress_sectors = 0;
static int cdda_tools_progress_pct = 0;

/* v267: ETA tracking */
static uint32_t cdda_eta_start_time = 0;   /* When conversion started (ms) */
static int cdda_eta_start_pct = 0;         /* Progress % when started (for resume) */
static int cdda_eta_minutes = 0;           /* Estimated remaining minutes */
static int cdda_eta_seconds = 0;           /* Estimated remaining seconds */

/* Progress callback for conversion */
static void cdda_progress_update(int track_idx, int total_tracks,
                                  int sector, int total_sectors, int total_pct) {
    cdda_tools_progress_track = track_idx + 1;
    cdda_tools_progress_total = total_tracks;
    cdda_tools_progress_sector = sector;
    cdda_tools_progress_sectors = total_sectors;
    cdda_tools_progress_pct = total_pct;
}

/* ============== CONFIGURATION STRUCTURE ============== */
/* v353: Bitflags for booleans - saves 53 bytes (14 bools * 4 = 56 -> 3 bytes) */
static struct {
    int frameskip;        /* 0-5, use gpu.frameskip.set */
    int target_speed;     /* v335: 40-100%, step 10%, 100=normal */
    int resampler_type;   /* v338: 0=Linear(fast), 1=Cubic(better) */
    char bios_file[64];
    int cycle_mult;       /* 256-2048, step 128 */
    int cycle_jump;       /* v370: Cycle adjustment step: 32/64/128/256 (default 128) */

    /* SPU */
    int spu_update_freq;
    int spu_reverb;
    int spu_interpolation;
    int xa_update_freq;
    int spu_tempo;
    int spu_irq_always;

    /* Audio - cdda_preload is multi-value (0-3) so stays as int */
    int cdda_preload;       /* v341: Preload small CDDA tracks: 0=off, 1=<0.5M, 2=<1M, 3=<1.5M */

    /* GPU - blending and interlace_force are multi-value so stay as int */
    int gpu_blending;       /* 0-3: blending level */
    int gpu_interlace_force;/* 0-2: pixel size 1x1/2x1/2x2 */

    /* Dynarec hacks - v367: nogteflags/gteregsunneeded REMOVED (hardcoded to 0) */
    int opt_gte_noflags;    /* Skip overflow flag checking - RISK! Can break some games! */

    /* v294: GTE function pointer dispatch - runtime ASM toggle */
    int gte_rtpt_asm;       /* RTPT ASM (0=C, 1=ASM no-flags) */
    int gte_mvmva_asm;      /* MVMVA ASM (0=C, 1=ASM no-flags) */

    /* v367: skip_code_inv/asm_memwrite REMOVED (hardcoded to 0) */

    /* v300: Block Chaining */
    int block_chain;        /* v309: Block chaining ON/OFF (runtime toggle) */

    /* v112: Speed hacks - multi-value */
    int cycle_batch;        /* Cycle batching level: 0=OFF, 1=Light(2x), 2=Med(4x), 3=Aggr(8x) */
    int block_cache;        /* Hot Block Cache: 0=OFF, 1=256, 2=1K, 3=4K, 4=16K, 5=64K, 6=256K, 7=1M */

    /* v115: Native Mode - multi-value */
    int native_mode;        /* 0=OFF, 1=STATS, 2=FAST, 3=NATIVE, 4=DIFF, 5=ASM, 6=ASM_SEL */

    /* v353: BITFLAGS - pack boolean options (saves 53 bytes!) */
    uint8_t gpu_flags;      /* GPU_FLAG_* : dithering, lighting, asm_light/blend, etc */
    uint8_t audio_flags;    /* AUDIO_FLAG_* : xa_disable, cdda_disable, use_bios, etc */
    uint8_t debug_flags;    /* DEBUG_FLAG_* : show_fps, profiler, debug_log */
    uint8_t input_flags;    /* v368: INPUT_FLAG_* : player2 enable */

    /* v352: Key remapping at END for cache alignment */
    int remap_a;          /* SF2000 A button -> PSX button */
    int remap_b;          /* SF2000 B button -> PSX button */
    int remap_x;          /* SF2000 X button -> PSX button */
    int remap_y;          /* SF2000 Y button -> PSX button */
    int remap_l;          /* SF2000 L button -> PSX button */
    int remap_r;          /* SF2000 R button -> PSX button */
    int remap_up;         /* SF2000 D-pad Up -> PSX button */
    int remap_down;       /* SF2000 D-pad Down -> PSX button */
    int remap_left;       /* SF2000 D-pad Left -> PSX button */
    int remap_right;      /* SF2000 D-pad Right -> PSX button */

} qpsx_config = {
    /* Defaults */
    0,                  /* frameskip = 0 */
    100,                /* target_speed = 100% */
    0,                  /* resampler_type = Linear */
    "scph5501.bin",     /* bios_file */
    1024,               /* cycle_mult */
    128,                /* v370: cycle_jump = 128 (default step) */

    /* SPU */
    0, 0, 0, 0, 0, 0,

    /* Audio - cdda_preload */
    0,                  /* cdda_preload=OFF */

    /* GPU multi-value */
    1,                  /* gpu_blending=1 */
    0,                  /* gpu_interlace_force=0 (1x1) */

    /* Dynarec - v367: nogteflags/gteregsunneeded hardcoded to 0 */
    1,                  /* opt_gte_noflags=ON */

    /* GTE ASM dispatch */
    0, 0,               /* gte_rtpt_asm, gte_mvmva_asm */

    /* v367: skip_code_inv/asm_memwrite hardcoded to 0 */

    /* Block chain */
    0,                  /* block_chain */

    /* Speed hacks */
    0, 0,               /* cycle_batch, block_cache */

    /* Native mode */
    0,                  /* native_mode */

    /* v353: BITFLAGS - packed boolean options */
    /* gpu_flags: lighting=ON, asm_light=ON, asm_blend=ON, pixel_skip=ON */
    (GPU_FLAG_LIGHTING | GPU_FLAG_ASM_LIGHT | GPU_FLAG_ASM_BLEND | GPU_FLAG_PIXEL_SKIP),
    /* audio_flags: cdda_binsav=ON, use_bios=OFF (HLE default) */
    (AUDIO_FLAG_CDDA_BINSAV),
    /* debug_flags: show_fps=ON */
    (DEBUG_FLAG_SHOW_FPS),
    /* v368: input_flags: player2=OFF (default) */
    0,

    /* v367: Remap defaults - PSX X on B button (Japanese style) */
    1, 0, 3, 2, 4, 5,   /* A=O, B=X, X=Tri, Y=Sq, L=L1, R=R1 */
    10, 11, 12, 13      /* Up, Down, Left, Right */
};

/* v351: Initialize fixed remap LUT - no dynamic remapping, just fixed defaults
 * This removes the dead remap_* config code while keeping fast LUT-based input
 */
static void remap_rebuild_lut(void)
{
    /* v352: Rebuild LUT from qpsx_config.remap_* settings
     * Converts config values (0-13) to PSX bit positions via psx_remap_bits[] */
    remap_lut[RETRO_DEVICE_ID_JOYPAD_A] = psx_remap_bits[qpsx_config.remap_a % 14];
    remap_lut[RETRO_DEVICE_ID_JOYPAD_B] = psx_remap_bits[qpsx_config.remap_b % 14];
    remap_lut[RETRO_DEVICE_ID_JOYPAD_X] = psx_remap_bits[qpsx_config.remap_x % 14];
    remap_lut[RETRO_DEVICE_ID_JOYPAD_Y] = psx_remap_bits[qpsx_config.remap_y % 14];
    remap_lut[RETRO_DEVICE_ID_JOYPAD_L] = psx_remap_bits[qpsx_config.remap_l % 14];
    remap_lut[RETRO_DEVICE_ID_JOYPAD_R] = psx_remap_bits[qpsx_config.remap_r % 14];

    /* D-pad remapping */
    remap_lut[RETRO_DEVICE_ID_JOYPAD_UP]    = psx_remap_bits[qpsx_config.remap_up % 14];
    remap_lut[RETRO_DEVICE_ID_JOYPAD_DOWN]  = psx_remap_bits[qpsx_config.remap_down % 14];
    remap_lut[RETRO_DEVICE_ID_JOYPAD_LEFT]  = psx_remap_bits[qpsx_config.remap_left % 14];
    remap_lut[RETRO_DEVICE_ID_JOYPAD_RIGHT] = psx_remap_bits[qpsx_config.remap_right % 14];

    /* Fixed mappings (not remappable) */
    remap_lut[RETRO_DEVICE_ID_JOYPAD_START]  = PSX_BIT_START;
    remap_lut[RETRO_DEVICE_ID_JOYPAD_SELECT] = PSX_BIT_SELECT;
    remap_lut[RETRO_DEVICE_ID_JOYPAD_L2]     = PSX_BIT_L2;
    remap_lut[RETRO_DEVICE_ID_JOYPAD_R2]     = PSX_BIT_R2;
    remap_lut[RETRO_DEVICE_ID_JOYPAD_L3]     = PSX_BIT_L3;
    remap_lut[RETRO_DEVICE_ID_JOYPAD_R3]     = PSX_BIT_R3;
}

/* v367: HARDCODED dynarec options - defined in their respective files:
 * qpsx_nosmccheck=1 in recompiler.cpp (SMC Check OFF)
 * g_opt_skip_code_inv=0, g_opt_asm_memwrite=0 in psxmem.cpp */

/* v352: CDDA speedup options - HARDCODED ON for speed */
int g_opt_cdda_fast_mix = 1;     /* Skip spu.spuMem writes - HARDCODED ON */
int g_opt_cdda_unity_vol = 1;    /* Skip volume multiply - HARDCODED ON */
int g_opt_cdda_asm_mix = 1;      /* MIPS32 ASM CDDA mixer - HARDCODED ON */
int g_opt_cdda_binsav = 1;       /* Use .binadpcm/.binwav - configurable */
int g_opt_cdda_preload = 0;      /* Preload tracks - configurable */

/* v367: CDDA Mode - OFF/PRETEND/ON system */
/* 0=OFF (no timing, no audio), 1=PRETEND (timing works, no audio), 2=ON (full audio) */
int qpsx_cdda_mode = 2;          /* Default: CDDA_ON (full audio) */

/* v367: Dummy variables for ASM Select - v066 dynarec doesn't use these */
int g_opt_gte_rtpt_asm = 0;
int g_opt_gte_mvmva_asm = 0;
int g_opt_native_mode = 0;  /* v367: Native Mode disabled - v066 dynarec */

/* v367: Dummy native mode functions */
extern "C" int native_mode_get_ratio(void) { return 0; }
extern "C" int native_mode_get_pure_block_ratio(void) { return 0; }
extern "C" void native_stats_reset_frame(void) { }

/* v367: Dummy native/dynarec usage counters - v066 dynarec doesn't track these */
u32 native_used = 0;
u32 dynarec_used = 0;

/* v367: Stub gte_update_dispatch - v066 gte.cpp doesn't have runtime dispatch */
void gte_update_dispatch(void) { }

/* v367: Dummy g_asm_* variables - kept for ASM Select UI compatibility */
int g_asm_addiu_enabled = 0, g_asm_lui_enabled = 0;
int g_asm_slti_enabled = 0, g_asm_sltiu_enabled = 0;
int g_asm_andi_enabled = 0, g_asm_ori_enabled = 0, g_asm_xori_enabled = 0;
int g_asm_addu_enabled = 0, g_asm_subu_enabled = 0;
int g_asm_and_enabled = 0, g_asm_or_enabled = 0, g_asm_xor_enabled = 0, g_asm_nor_enabled = 0;
int g_asm_slt_enabled = 0, g_asm_sltu_enabled = 0;
int g_asm_sll_enabled = 0, g_asm_srl_enabled = 0, g_asm_sra_enabled = 0;
int g_asm_sllv_enabled = 0, g_asm_srlv_enabled = 0, g_asm_srav_enabled = 0;
int g_asm_lw_enabled = 0, g_asm_lh_enabled = 0, g_asm_lhu_enabled = 0;
int g_asm_lb_enabled = 0, g_asm_lbu_enabled = 0, g_asm_lwl_enabled = 0, g_asm_lwr_enabled = 0;
int g_asm_sw_enabled = 0, g_asm_sh_enabled = 0, g_asm_sb_enabled = 0;
int g_asm_swl_enabled = 0, g_asm_swr_enabled = 0;
int g_asm_beq_enabled = 0, g_asm_bne_enabled = 0, g_asm_blez_enabled = 0, g_asm_bgtz_enabled = 0;
int g_asm_bltz_enabled = 0, g_asm_bgez_enabled = 0, g_asm_bltzal_enabled = 0, g_asm_bgezal_enabled = 0;
int g_asm_j_enabled = 0, g_asm_jal_enabled = 0, g_asm_jr_enabled = 0, g_asm_jalr_enabled = 0;
int g_asm_mult_enabled = 0, g_asm_multu_enabled = 0, g_asm_div_enabled = 0, g_asm_divu_enabled = 0;
int g_asm_mfhi_enabled = 0, g_asm_mflo_enabled = 0, g_asm_mthi_enabled = 0, g_asm_mtlo_enabled = 0;
int g_asm_mfc0_enabled = 0, g_asm_mtc0_enabled = 0, g_asm_cfc0_enabled = 0, g_asm_ctc0_enabled = 0;
int g_asm_rfe_enabled = 0;
int g_asm_mfc2_enabled = 0, g_asm_mtc2_enabled = 0, g_asm_cfc2_enabled = 0, g_asm_ctc2_enabled = 0;
int g_asm_lwc2_enabled = 0, g_asm_swc2_enabled = 0;
int g_asm_debug_stats = 0;
unsigned int g_asm_count = 0, g_asm_c_count = 0;

/* ============== MENU ITEMS ============== */
typedef struct {
    const char *name;
    int type;           /* 0=option, 1=separator, 2=action, 3=preset */
    int restart;        /* 1=needs restart, 0=instant apply */
} MenuItem;

/* v367: Menu with GPU/SMC/Dynarec HARDCODED like v066 */
#define MENU_ITEMS 38
enum {
    MI_SEP_HEADER = 0,
    MI_SWAP_CD,         /* Swap CD for multi-disc */
    MI_FRAMESKIP,
    MI_TARGET_SPEED,
    MI_RESAMPLER_TYPE,
    MI_BIOS,
    MI_CYCLE,
    MI_CYCLE_JUMP,      /* v370: Cycle adjustment step (32/64/128/256) */
    MI_SHOW_FPS,
    MI_PLAYER2,       /* v368: Player 2 enable (default OFF) */
    MI_DEBUG_LOG,
    MI_SEP_AUDIO,
    MI_XA_AUDIO,
    MI_CDDA_MODE,       /* v367: CDDA OFF/PRETEND/ON */
    MI_CDDA_PRELOAD,
    MI_CDDA_TOOLS,
    MI_SEP_GPU,
    /* v369: REMOVED MI_PIXEL_SIZE - line doubling removed for v066b speed */
    MI_SEP_REMAP,
    MI_RESTORE_PAD,
    MI_REMAP_A,
    MI_REMAP_B,
    MI_REMAP_X,
    MI_REMAP_Y,
    MI_REMAP_L,
    MI_REMAP_R,
    MI_REMAP_UP,
    MI_REMAP_DOWN,
    MI_REMAP_LEFT,
    MI_REMAP_RIGHT,
    MI_SEP_CONFIG,
    MI_SAVE_GAME,
    MI_SAVE_SLUS,
    MI_SAVE_GLOBAL,
    MI_DEL_GAME,
    MI_DEL_GLOBAL,
    MI_SEP_EXIT,
    MI_EXIT
};

/*
 * v367: Menu with clean v066 dynarec
 * GPU options (except Pixel Size), SMC Check, Dynarec options = HARDCODED
 */
static const MenuItem menu_items[MENU_ITEMS] = {
    {"--- QPSX v374 ---",   1, 0},
    {">> SWAP CD <<",       2, 0},
    {"Frameskip",           0, 0},
    {"Target Speed",        0, 1},
    {"Resampler",           0, 1},
    {"BIOS",                0, 1},
    {"Cycles",              0, 1},
    {"Cycle Jump",          0, 0},  /* v370: Step size for Cycles adjustment */
    {"Show FPS",            0, 0},
    {"Player 2",            0, 0},  /* v368: Enable player 2 input */
    {"Debug Log",           0, 0},
    {"--- AUDIO ---",       1, 0},
    {"XA Audio",            0, 1},
    {"CDDA Mode",           0, 0},  /* v367: OFF/PRETEND/ON */
    {"CDDA Preload",        0, 0},
    {"CDDA Tools",          2, 0},
    {"--- GPU ---",         1, 0},
    /* v369: Pixel Size REMOVED - adds overhead */
    {"--- REMAP PAD ---",   1, 0},
    {"Restore Defaults",    2, 0},
    {"Button A",            0, 0},
    {"Button B",            0, 0},
    {"Button X",            0, 0},
    {"Button Y",            0, 0},
    {"Button L",            0, 0},
    {"Button R",            0, 0},
    {"D-Pad Up",            0, 0},
    {"D-Pad Down",          0, 0},
    {"D-Pad Left",          0, 0},
    {"D-Pad Right",         0, 0},
    {"--- CONFIG ---",      1, 0},
    {"SAVE GAME CFG",       2, 0},
    {"SAVE SLUS CFG",       2, 0},
    {"SAVE GLOBAL CFG",     2, 0},
    {"DEL GAME CFG",        2, 0},
    {"DEL GLOBAL CFG",      2, 0},
    {"--------------",      1, 0},
    {"EXIT MENU",           2, 0}
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

/* ============== NATIVE COMMAND MENU (v244) ============== */
#define NCMD_COLOR_BG       RGB565(0, 0, 32)      /* Dark blue background */
#define NCMD_COLOR_TITLE    RGB565(255, 255, 255) /* White title */
#define NCMD_COLOR_ENABLED  RGB565(0, 255, 0)     /* Green - enabled */
#define NCMD_COLOR_DISABLED RGB565(255, 64, 64)   /* Red - disabled */
#define NCMD_COLOR_NOTIMPL  RGB565(128, 128, 128) /* Gray - not implemented */
#define NCMD_COLOR_SEPARATOR RGB565(255, 255, 0)  /* Yellow - separator */
#define NCMD_COLOR_SELECTED RGB565(64, 64, 255)   /* Blue - selected bg */
#define NCMD_COLOR_START    RGB565(0, 255, 255)   /* Cyan - START */
#define NCMD_COLOR_ACTION   RGB565(255, 128, 0)   /* Orange - actions */

#define NCMD_VISIBLE_ITEMS  20   /* v244: increased from 12 to 20 */
#define NCMD_LINE_HEIGHT    10
#define NCMD_MENU_X         10
#define NCMD_MENU_Y         18

/* PSX controller bit positions in cached_pad1 (different from RETRO_DEVICE_ID!) */
#define PSX_BTN_SELECT  (1 << 0)
#define PSX_BTN_START   (1 << 3)
#define PSX_BTN_UP      (1 << 4)
#define PSX_BTN_RIGHT   (1 << 5)
#define PSX_BTN_DOWN    (1 << 6)
#define PSX_BTN_LEFT    (1 << 7)
#define PSX_BTN_L       (1 << 10)
#define PSX_BTN_R       (1 << 11)
#define PSX_BTN_X       (1 << 12)
#define PSX_BTN_A       (1 << 13)
#define PSX_BTN_B       (1 << 14)
#define PSX_BTN_Y       (1 << 15)

/* v339/v342: Button remap names (index 0-13) */
static const char *remap_btn_names[14] = {
    "X", "O", "[]", "/\\", "L1", "R1", "L2", "R2",
    "Start", "Select", "Up", "Down", "Left", "Right"
};

static void native_cmd_menu_init(void) {
    /* Count items */
    native_cmd_count = 0;
    for (int i = 0; i < NCMD_MAX && native_cmd_list[i].name != NULL; i++) {
        native_cmd_count++;
    }
    native_cmd_update_any_enabled();
}

static void native_cmd_menu_load_config(void) {
    FILE *f = fopen(QPSX_NATIVE_CONFIG_PATH, "r");
    if (!f) return;

    char line[128];
    while (fgets(line, sizeof(line), f)) {
        /* Parse "NAME=0" or "NAME=1" */
        char name[32];
        int val;
        if (sscanf(line, "%31[^=]=%d", name, &val) == 2) {
            for (int i = 0; i < native_cmd_count; i++) {
                if (native_cmd_list[i].implemented &&
                    strcmp(native_cmd_list[i].name, name) == 0) {
                    native_cmd_list[i].enabled = val ? 1 : 0;
                    break;
                }
            }
        }
    }
    fclose(f);
    XLOG("native_cmd: loaded config");
    native_cmd_update_any_enabled();
}

static void native_cmd_menu_save_config(void) {
    FILE *f = fopen(QPSX_NATIVE_CONFIG_PATH, "w");
    if (!f) {
        XLOG("native_cmd: cannot save config");
        return;
    }

    fprintf(f, "# QPSX Native Command Config v%s\n", QPSX_VERSION);
    for (int i = 0; i < native_cmd_count; i++) {
        if (native_cmd_list[i].implemented && native_cmd_list[i].opcode >= 0) {
            fprintf(f, "%s=%d\n", native_cmd_list[i].name, native_cmd_list[i].enabled);
        }
    }
    fclose(f);
    XLOG("native_cmd: saved config");
}

/* ============== ASM SELECT MENU FUNCTIONS (v285) ============== */
/* Colors for ASM Select menu */
#define ASMCMD_COLOR_BG       RGB565(0, 32, 0)       /* Dark green background */
#define ASMCMD_COLOR_TITLE    RGB565(255, 255, 255)  /* White title */
#define ASMCMD_COLOR_ENABLED  RGB565(0, 255, 0)      /* Green - enabled */
#define ASMCMD_COLOR_DISABLED RGB565(255, 64, 64)    /* Red - disabled */
#define ASMCMD_COLOR_NOTIMPL  RGB565(128, 128, 128)  /* Gray - not implemented */
#define ASMCMD_COLOR_SEPARATOR RGB565(0, 255, 255)   /* Cyan - phase headers */
#define ASMCMD_COLOR_SELECTED RGB565(64, 128, 64)    /* Dark green - selected bg */
#define ASMCMD_COLOR_START    RGB565(255, 255, 0)    /* Yellow - START */
#define ASMCMD_COLOR_ACTION   RGB565(255, 128, 0)    /* Orange - actions */

#define ASMCMD_VISIBLE_ITEMS  20
#define ASMCMD_LINE_HEIGHT    10
#define ASMCMD_MENU_X         10
#define ASMCMD_MENU_Y         18

/* v288: Helper to sync global toggles from asm_cmd_list */
static void asm_cmd_sync_globals(void) {
    extern int g_asm_addiu_enabled, g_asm_lui_enabled;
    extern int g_asm_slti_enabled, g_asm_sltiu_enabled;
    extern int g_asm_andi_enabled, g_asm_ori_enabled, g_asm_xori_enabled;
    extern int g_asm_addu_enabled, g_asm_subu_enabled;
    extern int g_asm_and_enabled, g_asm_or_enabled, g_asm_xor_enabled, g_asm_nor_enabled;
    extern int g_asm_slt_enabled, g_asm_sltu_enabled;
    /* v289: Shift toggles */
    extern int g_asm_sll_enabled, g_asm_srl_enabled, g_asm_sra_enabled;
    extern int g_asm_sllv_enabled, g_asm_srlv_enabled, g_asm_srav_enabled;
    /* v290: Load/Store toggles */
    extern int g_asm_lw_enabled, g_asm_lh_enabled, g_asm_lhu_enabled;
    extern int g_asm_lb_enabled, g_asm_lbu_enabled, g_asm_lwl_enabled, g_asm_lwr_enabled;
    extern int g_asm_sw_enabled, g_asm_sh_enabled, g_asm_sb_enabled;
    extern int g_asm_swl_enabled, g_asm_swr_enabled;
    /* v291: Branch/Jump toggles */
    extern int g_asm_beq_enabled, g_asm_bne_enabled, g_asm_blez_enabled, g_asm_bgtz_enabled;
    extern int g_asm_bltz_enabled, g_asm_bgez_enabled, g_asm_bltzal_enabled, g_asm_bgezal_enabled;
    extern int g_asm_j_enabled, g_asm_jal_enabled, g_asm_jr_enabled, g_asm_jalr_enabled;
    /* v292: MDU toggles */
    extern int g_asm_mult_enabled, g_asm_multu_enabled, g_asm_div_enabled, g_asm_divu_enabled;
    extern int g_asm_mfhi_enabled, g_asm_mflo_enabled, g_asm_mthi_enabled, g_asm_mtlo_enabled;
    /* v293: CP0 toggles */
    extern int g_asm_mfc0_enabled, g_asm_mtc0_enabled, g_asm_cfc0_enabled, g_asm_ctc0_enabled;
    extern int g_asm_rfe_enabled;
    /* v293: GTE toggles */
    extern int g_asm_mfc2_enabled, g_asm_mtc2_enabled, g_asm_cfc2_enabled, g_asm_ctc2_enabled;
    extern int g_asm_lwc2_enabled, g_asm_swc2_enabled;

    for (int i = 0; i < asm_cmd_count; i++) {
        asm_cmd_entry_t *e = &asm_cmd_list[i];
        if (!e->implemented) continue;
        switch (e->opcode) {
            case 0x08: case 0x09: g_asm_addiu_enabled = e->enabled; break;
            case 0x0A: g_asm_slti_enabled = e->enabled; break;
            case 0x0B: g_asm_sltiu_enabled = e->enabled; break;
            case 0x0C: g_asm_andi_enabled = e->enabled; break;
            case 0x0D: g_asm_ori_enabled = e->enabled; break;
            case 0x0E: g_asm_xori_enabled = e->enabled; break;
            case 0x0F: g_asm_lui_enabled = e->enabled; break;
            case 0x220: case 0x221: g_asm_addu_enabled = e->enabled; break;
            case 0x222: case 0x223: g_asm_subu_enabled = e->enabled; break;
            case 0x224: g_asm_and_enabled = e->enabled; break;
            case 0x225: g_asm_or_enabled = e->enabled; break;
            case 0x226: g_asm_xor_enabled = e->enabled; break;
            case 0x227: g_asm_nor_enabled = e->enabled; break;
            case 0x22A: g_asm_slt_enabled = e->enabled; break;
            case 0x22B: g_asm_sltu_enabled = e->enabled; break;
            /* v289: Shift opcodes */
            case 0x200: g_asm_sll_enabled = e->enabled; break;
            case 0x202: g_asm_srl_enabled = e->enabled; break;
            case 0x203: g_asm_sra_enabled = e->enabled; break;
            case 0x204: g_asm_sllv_enabled = e->enabled; break;
            case 0x206: g_asm_srlv_enabled = e->enabled; break;
            case 0x207: g_asm_srav_enabled = e->enabled; break;
            /* v290: Load opcodes */
            case 0x23: g_asm_lw_enabled = e->enabled; break;
            case 0x21: g_asm_lh_enabled = e->enabled; break;
            case 0x25: g_asm_lhu_enabled = e->enabled; break;
            case 0x20: g_asm_lb_enabled = e->enabled; break;
            case 0x24: g_asm_lbu_enabled = e->enabled; break;
            case 0x22: g_asm_lwl_enabled = e->enabled; break;
            case 0x26: g_asm_lwr_enabled = e->enabled; break;
            /* v290: Store opcodes */
            case 0x2B: g_asm_sw_enabled = e->enabled; break;
            case 0x29: g_asm_sh_enabled = e->enabled; break;
            case 0x28: g_asm_sb_enabled = e->enabled; break;
            case 0x2A: g_asm_swl_enabled = e->enabled; break;
            case 0x2E: g_asm_swr_enabled = e->enabled; break;
            /* v291: Branch opcodes */
            case 0x04: g_asm_beq_enabled = e->enabled; break;
            case 0x05: g_asm_bne_enabled = e->enabled; break;
            case 0x06: g_asm_blez_enabled = e->enabled; break;
            case 0x07: g_asm_bgtz_enabled = e->enabled; break;
            /* v291: REGIMM branch opcodes */
            case 0x100: g_asm_bltz_enabled = e->enabled; break;
            case 0x101: g_asm_bgez_enabled = e->enabled; break;
            case 0x110: g_asm_bltzal_enabled = e->enabled; break;
            case 0x111: g_asm_bgezal_enabled = e->enabled; break;
            /* v291: Jump opcodes */
            case 0x02: g_asm_j_enabled = e->enabled; break;
            case 0x03: g_asm_jal_enabled = e->enabled; break;
            case 0x208: g_asm_jr_enabled = e->enabled; break;
            case 0x209: g_asm_jalr_enabled = e->enabled; break;
            /* v292: MDU opcodes */
            case 0x218: g_asm_mult_enabled = e->enabled; break;
            case 0x219: g_asm_multu_enabled = e->enabled; break;
            case 0x21A: g_asm_div_enabled = e->enabled; break;
            case 0x21B: g_asm_divu_enabled = e->enabled; break;
            case 0x210: g_asm_mfhi_enabled = e->enabled; break;
            case 0x212: g_asm_mflo_enabled = e->enabled; break;
            case 0x211: g_asm_mthi_enabled = e->enabled; break;
            case 0x213: g_asm_mtlo_enabled = e->enabled; break;
            /* v293: CP0 opcodes */
            case 0x400: g_asm_mfc0_enabled = e->enabled; break;
            case 0x404: g_asm_mtc0_enabled = e->enabled; break;
            case 0x402: g_asm_cfc0_enabled = e->enabled; break;
            case 0x406: g_asm_ctc0_enabled = e->enabled; break;
            case 0x410: g_asm_rfe_enabled = e->enabled; break;
            /* v293: GTE transfer opcodes */
            case 0x480: g_asm_mfc2_enabled = e->enabled; break;
            case 0x484: g_asm_mtc2_enabled = e->enabled; break;
            case 0x482: g_asm_cfc2_enabled = e->enabled; break;
            case 0x486: g_asm_ctc2_enabled = e->enabled; break;
            case 0x32: g_asm_lwc2_enabled = e->enabled; break;
            case 0x3A: g_asm_swc2_enabled = e->enabled; break;
            /* v294: GTE runtime (function pointer dispatch) */
            case -10: g_opt_gte_rtpt_asm = e->enabled; break;
            case -11: g_opt_gte_mvmva_asm = e->enabled; break;
        }
    }
    /* v294: Update GTE function pointers */
    gte_update_dispatch();
}

/* v288: Load ASM Select config from file */
static void asm_cmd_menu_load_config(void) {
    FILE *f = fopen(QPSX_ASM_CONFIG_PATH, "r");
    if (!f) return;

    char line[128];
    while (fgets(line, sizeof(line), f)) {
        char name[32];
        int val;
        if (sscanf(line, "%31[^=]=%d", name, &val) == 2) {
            for (int i = 0; i < asm_cmd_count; i++) {
                if (asm_cmd_list[i].implemented &&
                    strcmp(asm_cmd_list[i].name, name) == 0) {
                    asm_cmd_list[i].enabled = val ? 1 : 0;
                    break;
                }
            }
        }
    }
    fclose(f);
    asm_cmd_sync_globals();
    XLOG("asm_cmd: loaded config from %s", QPSX_ASM_CONFIG_PATH);
}

/* v288: Save ASM Select config to file */
static void asm_cmd_menu_save_config(void) {
    FILE *f = fopen(QPSX_ASM_CONFIG_PATH, "w");
    if (!f) {
        XLOG("asm_cmd: cannot save config to %s", QPSX_ASM_CONFIG_PATH);
        return;
    }

    fprintf(f, "# QPSX ASM Select Config v%s\n", QPSX_VERSION);
    for (int i = 0; i < asm_cmd_count; i++) {
        if (asm_cmd_list[i].implemented && asm_cmd_list[i].opcode >= 0) {
            fprintf(f, "%s=%d\n", asm_cmd_list[i].name, asm_cmd_list[i].enabled);
        }
    }
    fclose(f);
    XLOG("asm_cmd: saved config to %s", QPSX_ASM_CONFIG_PATH);
}

static void draw_asm_cmd_menu(uint16_t *buffer) {
    /* Clear to dark green background */
    for (int i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT; i++) {
        buffer[i] = ASMCMD_COLOR_BG;
    }

    /* Title */
    char title_buf[64];
    snprintf(title_buf, sizeof(title_buf), "ASM SELECT v%s - Dynarec Optimizer", QPSX_VERSION);
    DrawText(buffer, 30, 4, ASMCMD_COLOR_TITLE, title_buf);
    DrawSeparator(buffer, 10, 14, SCREEN_WIDTH - 20, ASMCMD_COLOR_TITLE);

    /* Draw menu items */
    int y = ASMCMD_MENU_Y;
    int start_idx = asm_cmd_menu_scroll;
    int end_idx = start_idx + ASMCMD_VISIBLE_ITEMS;
    if (end_idx > asm_cmd_count) end_idx = asm_cmd_count;

    for (int i = start_idx; i < end_idx; i++) {
        asm_cmd_entry_t *entry = &asm_cmd_list[i];
        char line[64];
        uint16_t text_color;

        /* Highlight selected item */
        if (i == asm_cmd_menu_selected) {
            DrawFBox(buffer, 0, y - 1, SCREEN_WIDTH, ASMCMD_LINE_HEIGHT, ASMCMD_COLOR_SELECTED);
        }

        /* Determine color and text based on entry type */
        if (entry->opcode == -1) {
            /* START button */
            text_color = ASMCMD_COLOR_START;
            snprintf(line, sizeof(line), "  %s", entry->name);
        } else if (entry->opcode == -2) {
            /* Separator/Phase header */
            if (entry->name[0] == '=') {
                text_color = ASMCMD_COLOR_SEPARATOR;
            } else {
                text_color = ASMCMD_COLOR_NOTIMPL;
            }
            snprintf(line, sizeof(line), "%s", entry->name);
        } else if (entry->opcode == -3 || entry->opcode == -4) {
            /* Action buttons */
            text_color = ASMCMD_COLOR_ACTION;
            snprintf(line, sizeof(line), "  %s", entry->name);
        } else if (entry->opcode < -4) {
            /* Subsystem toggle (special features) */
            if (!entry->implemented) {
                text_color = ASMCMD_COLOR_NOTIMPL;
                snprintf(line, sizeof(line), "[---] %s", entry->name);
            } else if (entry->enabled) {
                text_color = ASMCMD_COLOR_ENABLED;
                snprintf(line, sizeof(line), "[ON ] %s", entry->name);
            } else {
                text_color = ASMCMD_COLOR_DISABLED;
                snprintf(line, sizeof(line), "[OFF] %s", entry->name);
            }
        } else {
            /* Regular opcode */
            if (!entry->implemented) {
                text_color = ASMCMD_COLOR_NOTIMPL;
                snprintf(line, sizeof(line), "[---] %s", entry->name);
            } else if (entry->enabled) {
                text_color = ASMCMD_COLOR_ENABLED;
                snprintf(line, sizeof(line), "[ASM] %s", entry->name);
            } else {
                text_color = ASMCMD_COLOR_DISABLED;
                snprintf(line, sizeof(line), "[C++] %s", entry->name);
            }
        }

        DrawText(buffer, ASMCMD_MENU_X, y, text_color, line);
        y += ASMCMD_LINE_HEIGHT;
    }

    /* Scroll indicators */
    if (start_idx > 0) {
        DrawText(buffer, SCREEN_WIDTH - 30, ASMCMD_MENU_Y, ASMCMD_COLOR_TITLE, "^");
    }
    if (end_idx < asm_cmd_count) {
        DrawText(buffer, SCREEN_WIDTH - 30, y - ASMCMD_LINE_HEIGHT, ASMCMD_COLOR_TITLE, "v");
    }

    /* Instructions */
    DrawSeparator(buffer, 10, SCREEN_HEIGHT - 18, SCREEN_WIDTH - 20, ASMCMD_COLOR_TITLE);
    DrawText(buffer, 4, SCREEN_HEIGHT - 12, ASMCMD_COLOR_TITLE, "U/D:1 L/R:Page A:Toggle START:Run");
}

/* Update scroll position to keep selection visible */
static void asm_cmd_update_scroll(void) {
    if (asm_cmd_menu_selected < asm_cmd_menu_scroll) {
        asm_cmd_menu_scroll = asm_cmd_menu_selected;
    }
    if (asm_cmd_menu_selected >= asm_cmd_menu_scroll + ASMCMD_VISIBLE_ITEMS) {
        asm_cmd_menu_scroll = asm_cmd_menu_selected - ASMCMD_VISIBLE_ITEMS + 1;
    }
}

/* v287: Helper function to move selection up/down with separator skip */
static void asm_cmd_menu_move_selection(int direction) {
    int start = asm_cmd_menu_selected;
    do {
        asm_cmd_menu_selected += direction;
        /* Wrap around */
        if (asm_cmd_menu_selected < 0) asm_cmd_menu_selected = asm_cmd_count - 1;
        if (asm_cmd_menu_selected >= asm_cmd_count) asm_cmd_menu_selected = 0;
        /* Safety: don't infinite loop if all are separators */
        if (asm_cmd_menu_selected == start) break;
    } while (asm_cmd_list[asm_cmd_menu_selected].opcode == -2);
    asm_cmd_update_scroll();
}

/* Handle ASM Select menu input - v287: Same controls as native_cmd_menu */
static void handle_asm_cmd_menu_input(void) {
    static uint16_t prev_buttons = 0;
    uint16_t buttons = ~cached_pad1;  /* Invert because cached_pad1 uses active-low */
    uint16_t pressed = buttons & ~prev_buttons;
    prev_buttons = buttons;

    /* UP - move 1 position up with wrap */
    if (pressed & PSX_BTN_UP) {
        asm_cmd_menu_move_selection(-1);
    }

    /* DOWN - move 1 position down with wrap */
    if (pressed & PSX_BTN_DOWN) {
        asm_cmd_menu_move_selection(+1);
    }

    /* LEFT - page up */
    if (pressed & PSX_BTN_LEFT) {
        for (int i = 0; i < ASMCMD_VISIBLE_ITEMS; i++) {
            asm_cmd_menu_move_selection(-1);
        }
    }

    /* RIGHT - page down */
    if (pressed & PSX_BTN_RIGHT) {
        for (int i = 0; i < ASMCMD_VISIBLE_ITEMS; i++) {
            asm_cmd_menu_move_selection(+1);
        }
    }

    /* A button - toggle or activate */
    if (pressed & PSX_BTN_A) {
        asm_cmd_entry_t *entry = &asm_cmd_list[asm_cmd_menu_selected];
        if (entry->opcode == -1) {
            /* START EMULATOR */
            asm_cmd_menu_active = 0;
            XLOG("asm_cmd: starting emulator");
        } else if (entry->opcode == -3) {
            /* SAVE - v288: implemented! */
            asm_cmd_menu_save_config();
        } else if (entry->opcode == -4) {
            /* RESET - reset all to defaults */
            for (int i = 0; i < asm_cmd_count; i++) {
                asm_cmd_list[i].enabled = 0;
            }
            /* Reset global toggles - v286 */
            extern int g_asm_addiu_enabled;
            extern int g_asm_lui_enabled;
            g_asm_addiu_enabled = 0;
            g_asm_lui_enabled = 0;
            /* v288: Reset new ALU toggles */
            extern int g_asm_slti_enabled, g_asm_sltiu_enabled;
            extern int g_asm_andi_enabled, g_asm_ori_enabled, g_asm_xori_enabled;
            extern int g_asm_addu_enabled, g_asm_subu_enabled;
            extern int g_asm_and_enabled, g_asm_or_enabled, g_asm_xor_enabled, g_asm_nor_enabled;
            extern int g_asm_slt_enabled, g_asm_sltu_enabled;
            g_asm_slti_enabled = g_asm_sltiu_enabled = 0;
            g_asm_andi_enabled = g_asm_ori_enabled = g_asm_xori_enabled = 0;
            g_asm_addu_enabled = g_asm_subu_enabled = 0;
            g_asm_and_enabled = g_asm_or_enabled = g_asm_xor_enabled = g_asm_nor_enabled = 0;
            g_asm_slt_enabled = g_asm_sltu_enabled = 0;
            XLOG("asm_cmd: RESET to defaults");
        } else if (entry->opcode >= 0 || entry->opcode < -4) {
            /* Toggle opcode/subsystem if implemented */
            if (entry->implemented) {
                entry->enabled = !entry->enabled;
                XLOG("asm_cmd: %s now %s", entry->name, entry->enabled ? "ON" : "OFF");
                /* Update global toggle for dynarec */
                /* v286: ALU Immediate */
                if (entry->opcode == 0x09 || entry->opcode == 0x08) {  /* ADDIU/ADDI */
                    extern int g_asm_addiu_enabled;
                    g_asm_addiu_enabled = entry->enabled;
                } else if (entry->opcode == 0x0A) {  /* SLTI */
                    extern int g_asm_slti_enabled;
                    g_asm_slti_enabled = entry->enabled;
                } else if (entry->opcode == 0x0B) {  /* SLTIU */
                    extern int g_asm_sltiu_enabled;
                    g_asm_sltiu_enabled = entry->enabled;
                } else if (entry->opcode == 0x0C) {  /* ANDI */
                    extern int g_asm_andi_enabled;
                    g_asm_andi_enabled = entry->enabled;
                } else if (entry->opcode == 0x0D) {  /* ORI */
                    extern int g_asm_ori_enabled;
                    g_asm_ori_enabled = entry->enabled;
                } else if (entry->opcode == 0x0E) {  /* XORI */
                    extern int g_asm_xori_enabled;
                    g_asm_xori_enabled = entry->enabled;
                } else if (entry->opcode == 0x0F) {  /* LUI */
                    extern int g_asm_lui_enabled;
                    g_asm_lui_enabled = entry->enabled;
                }
                /* v288: ALU Register (opcode 0x2XX = R-type with funct) */
                else if (entry->opcode == 0x221 || entry->opcode == 0x220) {  /* ADDU/ADD */
                    extern int g_asm_addu_enabled;
                    g_asm_addu_enabled = entry->enabled;
                } else if (entry->opcode == 0x223 || entry->opcode == 0x222) {  /* SUBU/SUB */
                    extern int g_asm_subu_enabled;
                    g_asm_subu_enabled = entry->enabled;
                } else if (entry->opcode == 0x224) {  /* AND */
                    extern int g_asm_and_enabled;
                    g_asm_and_enabled = entry->enabled;
                } else if (entry->opcode == 0x225) {  /* OR */
                    extern int g_asm_or_enabled;
                    g_asm_or_enabled = entry->enabled;
                } else if (entry->opcode == 0x226) {  /* XOR */
                    extern int g_asm_xor_enabled;
                    g_asm_xor_enabled = entry->enabled;
                } else if (entry->opcode == 0x227) {  /* NOR */
                    extern int g_asm_nor_enabled;
                    g_asm_nor_enabled = entry->enabled;
                } else if (entry->opcode == 0x22A) {  /* SLT */
                    extern int g_asm_slt_enabled;
                    g_asm_slt_enabled = entry->enabled;
                } else if (entry->opcode == 0x22B) {  /* SLTU */
                    extern int g_asm_sltu_enabled;
                    g_asm_sltu_enabled = entry->enabled;
                }
            } else {
                XLOG("asm_cmd: %s not yet implemented", entry->name);
            }
        }
    }

    /* START button - start emulator */
    if (pressed & PSX_BTN_START) {
        asm_cmd_menu_active = 0;
        XLOG("asm_cmd: starting emulator (START pressed)");
    }

    /* B button - start without saving */
    if (pressed & PSX_BTN_B) {
        asm_cmd_menu_active = 0;
        XLOG("asm_cmd: starting emulator (B pressed)");
    }
}

static void draw_native_cmd_menu(uint16_t *buffer) {
    /* Clear to dark background */
    for (int i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT; i++) {
        buffer[i] = NCMD_COLOR_BG;
    }

    /* Title - use QPSX_VERSION */
    char title_buf[48];
    snprintf(title_buf, sizeof(title_buf), "NATIVE COMMAND CONFIG v%s", QPSX_VERSION);
    DrawText(buffer, 50, 4, NCMD_COLOR_TITLE, title_buf);
    DrawSeparator(buffer, 10, 14, SCREEN_WIDTH - 20, NCMD_COLOR_TITLE);

    /* Draw visible items */
    int y = NCMD_MENU_Y;
    int start_idx = native_cmd_menu_scroll;
    int end_idx = start_idx + NCMD_VISIBLE_ITEMS;
    if (end_idx > native_cmd_count) end_idx = native_cmd_count;

    for (int i = start_idx; i < end_idx; i++) {
        native_cmd_entry_t* cmd = &native_cmd_list[i];
        uint16_t text_color;
        char line[40];

        /* Selection highlight */
        if (i == native_cmd_menu_selected) {
            DrawFBox(buffer, 0, y - 1, SCREEN_WIDTH, NCMD_LINE_HEIGHT, NCMD_COLOR_SELECTED);
        }

        /* Determine color and text */
        if (cmd->opcode == -1) {
            /* START EMULATOR */
            text_color = NCMD_COLOR_START;
            snprintf(line, sizeof(line), "  %s", cmd->name);
        } else if (cmd->opcode == -2) {
            /* Separator */
            text_color = NCMD_COLOR_SEPARATOR;
            snprintf(line, sizeof(line), "%s", cmd->name);
        } else if (cmd->opcode == -3 || cmd->opcode == -4) {
            /* Action items (SAVE, RESET) */
            text_color = NCMD_COLOR_ACTION;
            snprintf(line, sizeof(line), "  [%s]", cmd->name);
        } else if (!cmd->implemented) {
            /* Not implemented - gray with [-] */
            text_color = NCMD_COLOR_NOTIMPL;
            snprintf(line, sizeof(line), "[-] %s (dynarec)", cmd->name);
        } else if (cmd->enabled) {
            /* Enabled - green with [X] */
            text_color = NCMD_COLOR_ENABLED;
            snprintf(line, sizeof(line), "[X] %s", cmd->name);
        } else {
            /* Disabled - red with [ ] */
            text_color = NCMD_COLOR_DISABLED;
            snprintf(line, sizeof(line), "[ ] %s", cmd->name);
        }

        DrawText(buffer, NCMD_MENU_X, y, text_color, line);
        y += NCMD_LINE_HEIGHT;
    }

    /* Scroll indicators */
    if (native_cmd_menu_scroll > 0) {
        DrawText(buffer, SCREEN_WIDTH - 30, NCMD_MENU_Y, NCMD_COLOR_TITLE, "^");
    }
    if (end_idx < native_cmd_count) {
        DrawText(buffer, SCREEN_WIDTH - 30, y - NCMD_LINE_HEIGHT, NCMD_COLOR_TITLE, "v");
    }

    /* Footer */
    DrawSeparator(buffer, 10, SCREEN_HEIGHT - 18, SCREEN_WIDTH - 20, NCMD_COLOR_TITLE);
    DrawText(buffer, 4, SCREEN_HEIGHT - 12, NCMD_COLOR_TITLE, "U/D:1 L/R:Page A:Toggle START:Run");
}

static void native_cmd_menu_move_selection(int delta) {
    /* Move selection, skipping separators, with wrap-around */
    int new_sel = native_cmd_menu_selected;
    int attempts = 0;

    do {
        new_sel += delta;
        /* Wrap around */
        if (new_sel < 0) new_sel = native_cmd_count - 1;
        if (new_sel >= native_cmd_count) new_sel = 0;
        attempts++;
        /* Stop if we've checked all items (prevent infinite loop) */
        if (attempts > native_cmd_count) break;
    } while (native_cmd_list[new_sel].opcode == -2);  /* Skip separators */

    native_cmd_menu_selected = new_sel;

    /* Adjust scroll to keep selection visible */
    if (native_cmd_menu_selected < native_cmd_menu_scroll) {
        native_cmd_menu_scroll = native_cmd_menu_selected;
    }
    if (native_cmd_menu_selected >= native_cmd_menu_scroll + NCMD_VISIBLE_ITEMS) {
        native_cmd_menu_scroll = native_cmd_menu_selected - NCMD_VISIBLE_ITEMS + 1;
    }
}

static void native_cmd_menu_reset_defaults(void) {
    /* Reset all implemented commands to enabled */
    for (int i = 0; i < native_cmd_count; i++) {
        if (native_cmd_list[i].implemented && native_cmd_list[i].opcode >= 0) {
            native_cmd_list[i].enabled = 1;
        }
    }
    XLOG("native_cmd: reset to defaults");
}

static void handle_native_cmd_menu_input(void) {
    static uint16_t prev_buttons = 0;
    uint16_t buttons = ~cached_pad1;  /* Invert because cached_pad1 uses active-low */
    uint16_t pressed = buttons & ~prev_buttons;
    prev_buttons = buttons;

    /* UP - move 1 position up with wrap */
    if (pressed & PSX_BTN_UP) {
        native_cmd_menu_move_selection(-1);
    }

    /* DOWN - move 1 position down with wrap */
    if (pressed & PSX_BTN_DOWN) {
        native_cmd_menu_move_selection(+1);
    }

    /* LEFT - page up (NCMD_VISIBLE_ITEMS positions) */
    if (pressed & PSX_BTN_LEFT) {
        for (int i = 0; i < NCMD_VISIBLE_ITEMS; i++) {
            native_cmd_menu_move_selection(-1);
        }
    }

    /* RIGHT - page down (NCMD_VISIBLE_ITEMS positions) */
    if (pressed & PSX_BTN_RIGHT) {
        for (int i = 0; i < NCMD_VISIBLE_ITEMS; i++) {
            native_cmd_menu_move_selection(+1);
        }
    }

    /* A button - toggle, start, save, or reset */
    if (pressed & PSX_BTN_A) {
        native_cmd_entry_t* cmd = &native_cmd_list[native_cmd_menu_selected];
        if (cmd->opcode == -1) {
            /* START EMULATOR */
            native_cmd_menu_active = 0;
            native_cmd_menu_save_config();
            XLOG("native_cmd: starting emulator");
        } else if (cmd->opcode == -3) {
            /* SAVE OPTIONS */
            native_cmd_menu_save_config();
            XLOG("native_cmd: options saved");
        } else if (cmd->opcode == -4) {
            /* RESET TO DEFAULTS */
            native_cmd_menu_reset_defaults();
        } else if (cmd->implemented && cmd->opcode >= 0) {
            /* Toggle command on/off */
            cmd->enabled = !cmd->enabled;
        }
    }

    /* START button - start emulator (saves automatically) */
    if (pressed & PSX_BTN_START) {
        native_cmd_menu_active = 0;
        native_cmd_menu_save_config();
        XLOG("native_cmd: starting emulator (START pressed)");
    }

    /* B button - start without saving */
    if (pressed & PSX_BTN_B) {
        native_cmd_menu_active = 0;
        XLOG("native_cmd: starting emulator (B pressed, no save)");
    }
}

/* ============== CD SWAP FILE BROWSER (v284) ============== */

/* Helper: check if string ends with .cue (case insensitive) */
static int str_ends_with_cue(const char *str) {
    int len = strlen(str);
    if (len < 4) return 0;
    const char *ext = str + len - 4;
    return (ext[0] == '.' &&
            (ext[1] == 'c' || ext[1] == 'C') &&
            (ext[2] == 'u' || ext[2] == 'U') &&
            (ext[3] == 'e' || ext[3] == 'E'));
}

/* Extract directory from game_path into fb_game_dir */
static void fb_init_game_dir(void) {
    /* game_path is like "/mnt/sda1/ROMS/qpsx/Game.cue" */
    strncpy(fb_game_dir, game_path, FB_MAX_PATH - 1);
    fb_game_dir[FB_MAX_PATH - 1] = '\0';

    /* Find last slash and truncate to get directory */
    char *last_slash = strrchr(fb_game_dir, '/');
    if (last_slash) {
        *last_slash = '\0';
    } else {
        /* Fallback if no slash found */
        strcpy(fb_game_dir, "/mnt/sda1/ROMS");
    }
    XLOG("CD swap dir: %s", fb_game_dir);
}

/* Scan game directory for .cue files only (v284: simplified) */
static void fb_scan_cue_files(void) {
    /* fs_readdir buffer structure (SF2000 firmware) */
    union {
        struct { uint8_t _1[0x10]; uint32_t type; };
        struct { uint8_t _2[0x22]; char d_name[0x225]; };
        uint8_t __[0x428];
    } buffer;

    fb_file_count = 0;
    fb_selection = 0;
    fb_scroll = 0;

    int dir_fd = fs_opendir(fb_game_dir);
    if (dir_fd < 0) {
        XLOG("fb_scan: cannot open %s", fb_game_dir);
        return;
    }

    /* Read directory - only .cue files */
    while (fb_file_count < FB_MAX_FILES) {
        memset(&buffer, 0, sizeof(buffer));
        if (fs_readdir(dir_fd, &buffer) < 0) break;

        /* Only .cue files */
        if (!str_ends_with_cue(buffer.d_name)) continue;

        /* Copy filename */
        strncpy(fb_files[fb_file_count], buffer.d_name, FB_MAX_NAME - 1);
        fb_files[fb_file_count][FB_MAX_NAME - 1] = '\0';
        fb_file_count++;
    }

    fs_closedir(dir_fd);
    XLOG("fb_scan: %s -> %d cue files", fb_game_dir, fb_file_count);
}

/* Perform the actual CD swap */
static int do_cd_swap(const char *iso_path) {
    XLOG("CD swap: %s", iso_path);

    /* Set new ISO file */
    SetIsoFile(iso_path);

    /* Clear disc ID/label */
    CdromId[0] = '\0';
    CdromLabel[0] = '\0';

    /* Unregister multi-CD callback */
    cdrIsoMultidiskCallback = NULL;

    /* Reload CD-ROM plugin */
    if (ReloadCdromPlugin() < 0) {
        XLOG("CD swap: ReloadCdromPlugin failed");
        return 0;
    }

    /* Open new CD */
    if (CDR_open() < 0) {
        XLOG("CD swap: CDR_open failed");
        return 0;
    }

    /* Simulate lid open (2 seconds in the future) */
    SetCdOpenCaseTime((s64)time(NULL) + 2);

    /* Trigger lid interrupt */
    LidInterrupt();

    XLOG("CD swap: success");
    return 1;
}

/* Handle file browser input (v284: simplified - no navigation) */
static void handle_cd_browser_input(void) {
    static uint16_t prev_buttons = 0;
    uint16_t buttons = ~cached_pad1;  /* Invert because active-low */
    uint16_t pressed = buttons & ~prev_buttons;
    prev_buttons = buttons;

    /* UP - move selection up */
    if (pressed & PSX_BTN_UP) {
        if (fb_selection > 0) fb_selection--;
        else fb_selection = fb_file_count - 1;  /* Wrap to bottom */
    }

    /* DOWN - move selection down */
    if (pressed & PSX_BTN_DOWN) {
        if (fb_selection < fb_file_count - 1) fb_selection++;
        else fb_selection = 0;  /* Wrap to top */
    }

    /* LEFT - page up */
    if (pressed & PSX_BTN_LEFT) {
        fb_selection -= FB_VISIBLE_ITEMS;
        if (fb_selection < 0) fb_selection = 0;
    }

    /* RIGHT - page down */
    if (pressed & PSX_BTN_RIGHT) {
        fb_selection += FB_VISIBLE_ITEMS;
        if (fb_selection >= fb_file_count) fb_selection = fb_file_count - 1;
    }

    /* Adjust scroll to keep selection visible */
    if (fb_selection < fb_scroll) fb_scroll = fb_selection;
    if (fb_selection >= fb_scroll + FB_VISIBLE_ITEMS) {
        fb_scroll = fb_selection - FB_VISIBLE_ITEMS + 1;
    }

    /* A button - select .cue file */
    if (pressed & PSX_BTN_A) {
        if (fb_file_count == 0) return;

        /* Build full path and do CD swap */
        char full_path[FB_MAX_PATH];
        snprintf(full_path, FB_MAX_PATH, "%s/%s", fb_game_dir, fb_files[fb_selection]);

        if (do_cd_swap(full_path)) {
            /* Success - close browser and menu */
            cd_browser_active = 0;
            menu_active = 0;
        }
    }

    /* B or START button - cancel and return to menu */
    if ((pressed & PSX_BTN_B) || (pressed & PSX_BTN_START)) {
        cd_browser_active = 0;
    }
}

/* Draw file browser overlay (v284: simplified - .cue only) */
static void draw_cd_browser(uint16_t *buffer) {
    int fb_x = 10;
    int fb_y = 10;
    int fb_w = 300;
    int fb_h = 220;

    uint16_t col_bg = RGB565(0, 0, 32);
    uint16_t col_border = RGB565(255, 255, 255);
    uint16_t col_title = RGB565(255, 255, 0);
    uint16_t col_file = RGB565(255, 255, 255);
    uint16_t col_sel = RGB565(64, 64, 255);
    uint16_t col_dim = RGB565(128, 128, 128);

    /* Background */
    DrawFBox(buffer, fb_x, fb_y, fb_w, fb_h, col_bg);
    /* Border */
    DrawBox(buffer, fb_x, fb_y, fb_w, fb_h, col_border);
    DrawBox(buffer, fb_x + 1, fb_y + 1, fb_w - 2, fb_h - 2, col_border);

    /* Title */
    DrawText(buffer, fb_x + 80, fb_y + 4, col_title, "SELECT DISC (.cue)");

    /* Separator */
    DrawFBox(buffer, fb_x + 4, fb_y + 15, fb_w - 8, 1, col_border);

    /* File list */
    int list_y = fb_y + 20;
    int item_height = 11;

    if (fb_file_count == 0) {
        DrawText(buffer, fb_x + 60, list_y + 60, col_dim, "(no .cue files in this folder)");
    } else {
        for (int i = 0; i < FB_VISIBLE_ITEMS && (fb_scroll + i) < fb_file_count; i++) {
            int idx = fb_scroll + i;
            int y = list_y + i * item_height;

            /* Selection highlight */
            if (idx == fb_selection) {
                DrawFBox(buffer, fb_x + 4, y - 1, fb_w - 8, item_height, col_sel);
            }

            /* Filename */
            char display[48];
            strncpy(display, fb_files[idx], 47);
            display[47] = '\0';
            DrawText(buffer, fb_x + 8, y, col_file, display);
        }
    }

    /* Scroll indicators */
    if (fb_scroll > 0) {
        DrawText(buffer, fb_x + fb_w - 20, fb_y + 20, col_title, "^");
    }
    if (fb_scroll + FB_VISIBLE_ITEMS < fb_file_count) {
        DrawText(buffer, fb_x + fb_w - 20, fb_y + fb_h - 30, col_title, "v");
    }

    /* Footer */
    DrawFBox(buffer, fb_x + 4, fb_y + fb_h - 16, fb_w - 8, 1, col_border);
    DrawText(buffer, fb_x + 30, fb_y + fb_h - 12, col_dim, "U/D:select  A:swap disc  B:cancel");
}

/* ============== SPEED % DISPLAY (v337: was FPS) ============== */
static void draw_fps_overlay(uint16_t *pixels)
{
    if (!fps_show || !pixels || menu_active || native_cmd_menu_active || asm_cmd_menu_active) return;

    int native_fps = Config.PsxType ? 50 : 60;  /* PsxType: 0=NTSC, 1=PAL */

    /* v345: Calculate EFFECTIVE PSX frames per second
     * Each retro_run() executes target_speed% of a full PSX frame
     * So effective_fps = fps_current * target_speed / 100 */
    int effective_fps = (fps_current * g_target_speed) / 100;

    /* Speed percentage = effective fps vs native fps */
    int speed_pct = (effective_fps * 100) / native_fps;
    if (speed_pct > 999) speed_pct = 999;

    /* Choose color based on speed percentage */
    uint16_t col;
    if (speed_pct >= 90) col = FPS_GOOD;
    else if (speed_pct >= 60) col = FPS_OK;
    else col = FPS_BAD;

    /* Draw background box - wider to fit "XXX% XX" format */
    DrawFBox(pixels, 2, 2, 48, 11, FPS_BG);

    /* Draw speed percentage AND effective FPS */
    char buf[16];
    snprintf(buf, sizeof(buf), "%3d%% %2d", speed_pct, effective_fps);
    DrawText(pixels, 4, 4, col, buf);
}

/* v295: GPU FPS overlay in TOP-RIGHT corner */
static void draw_gpu_fps_overlay(uint16_t *pixels)
{
    if (!fps_show || !pixels || menu_active || native_cmd_menu_active || asm_cmd_menu_active) return;

    /* v346 FIX: Calculate EFFECTIVE GPU frames per second
     * gpu_fps_current counts video_flip() calls (~50/sec for PAL)
     * But each call only does target_speed% of a full PSX frame
     * So effective GPU fps = gpu_fps_current * target_speed / 100 */
    int effective_gpu_fps = (gpu_fps_current * g_target_speed) / 100;

    /* Choose color based on effective GPU FPS relative to expected */
    uint16_t col;
    int native_fps = Config.PsxType ? 50 : 60;
    int expected_gpu_fps = (native_fps * g_target_speed) / 100;

    if (effective_gpu_fps >= (expected_gpu_fps * 90) / 100) col = FPS_GOOD;
    else if (effective_gpu_fps >= (expected_gpu_fps * 60) / 100) col = FPS_OK;
    else col = FPS_BAD;

    /* Draw in top-right corner (320-26=294) */
    DrawFBox(pixels, 294, 2, 24, 11, FPS_BG);

    /* Draw effective GPU FPS number */
    char buf[8];
    snprintf(buf, sizeof(buf), "%2d", effective_gpu_fps);
    DrawText(pixels, 296, 4, col, buf);
}

static void update_fps_counter(void)
{
    fps_frame_count++;

    unsigned long now = get_time_ms();
    if (fps_last_time == 0) {
        fps_last_time = now;
        gpu_fps_snapshot = gpu_frame_count;  /* v295: init snapshot */
        return;
    }

    unsigned long elapsed = now - fps_last_time;
    if (elapsed >= 1000) {
        fps_current = (fps_frame_count * 1000) / elapsed;
        fps_frame_count = 0;

        /* v295: Calculate GPU FPS from actual rendered frames */
        int gpu_frames = gpu_frame_count - gpu_fps_snapshot;
        gpu_fps_current = (gpu_frames * 1000) / elapsed;
        gpu_fps_snapshot = gpu_frame_count;

        fps_last_time = now;
    }
}

/* Forward declaration for profiler overlay (defined later) */
static void draw_profiler_overlay(uint16_t *pixels);

/* ============== VIDEO CALLBACK WRAPPER ============== */
static void wrapped_video_cb(const void *data, unsigned width, unsigned height, size_t pitch)
{
    /*
     * v108: Frame duping support
     * When data=NULL, frontend reuses previous frame (no copy needed).
     * We skip overlay drawing but still call video_cb for timing.
     */
    if (data) {
        /* v368: Draw overlays - native_cmd/asm_cmd menu checks REMOVED (always 0) */
        if (fps_show && !menu_active) {
            draw_fps_overlay((uint16_t*)data);
            draw_gpu_fps_overlay((uint16_t*)data);  /* v295: GPU FPS in right corner */
        }
        /* v368: profiler overlay REMOVED - profiler disabled at compile time */
    }

    /*
     * v108: Measure video output time
     * For normal frames: measures firmware copy time (~14ms)
     * For duped frames (data=NULL): should be nearly 0ms
     */
    PROFILE_START(PROF_VIDEO_OUTPUT);
    if (real_video_cb) {
        real_video_cb(data, width, height, pitch);
    }
    PROFILE_END(PROF_VIDEO_OUTPUT);

    /* End profiler frame timing AFTER video output */
    profiler_frame_end();
    /* v367b: native_mode REMOVED (hardcoded OFF) */

    /* Start timing for next frame */
    profiler_frame_start();
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

/* v250: Two config paths - filename (primary) and SLUS/SCES (secondary) */
static void get_game_config_path(char *path, int maxlen)
{
    /* Primary: filename-based (like v247/v248) */
    const char *base = strrchr(game_path, '/');
    if (!base) base = strrchr(game_path, '\\');
    if (base) base++; else base = game_path;
    snprintf(path, maxlen, "/mnt/sda1/cores/config/%s.cfg", base);
}

static void get_slus_config_path(char *path, int maxlen)
{
    /* Secondary: SLUS/SCES/SLES-based config */
    if (CdromId[0] != '\0') {
        snprintf(path, maxlen, "/mnt/sda1/cores/config/%s.cfg", CdromId);
    } else {
        path[0] = '\0';
    }
}

/* ============== v340: MULTI-SLUS AND NAME PATTERN MATCHING ============== */

/*
 * Check if cdrom_id matches any code in comma-separated list
 * Example: slus_code=SCUS-94900,SCES-00344,SCES-00967
 */
static int match_slus_codes(const char *cdrom_id, const char *codes_list)
{
    if (!cdrom_id || !cdrom_id[0] || !codes_list || !codes_list[0]) return 0;

    char buf[256];
    strncpy(buf, codes_list, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *token = strtok(buf, ",");
    while (token) {
        /* Trim whitespace */
        while (*token == ' ' || *token == '\t') token++;
        char *end = token + strlen(token) - 1;
        while (end > token && (*end == ' ' || *end == '\t' || *end == '\n' || *end == '\r')) *end-- = '\0';

        if (strcasecmp(cdrom_id, token) == 0) {
            return 1;  /* Match found */
        }
        token = strtok(NULL, ",");
    }
    return 0;
}

/*
 * Simple wildcard matching with * (matches any chars)
 * Case-insensitive
 */
static int wildcard_match(const char *pattern, const char *str)
{
    if (!pattern || !str) return 0;

    while (*pattern && *str) {
        if (*pattern == '*') {
            pattern++;
            if (!*pattern) return 1;  /* Trailing * matches all */
            /* Try matching rest of pattern at each position */
            while (*str) {
                if (wildcard_match(pattern, str)) return 1;
                str++;
            }
            return wildcard_match(pattern, str);
        }
        /* Case-insensitive compare */
        char p = (*pattern >= 'A' && *pattern <= 'Z') ? *pattern + 32 : *pattern;
        char s = (*str >= 'A' && *str <= 'Z') ? *str + 32 : *str;
        if (p != s) return 0;
        pattern++;
        str++;
    }
    /* Handle trailing wildcards */
    while (*pattern == '*') pattern++;
    return (!*pattern && !*str);
}

/*
 * Parse and evaluate name pattern with operators: !, &&, ||, *, ()
 * Example patterns:
 *   *crash*bandicoot*          - matches if name contains crash and bandicoot
 *   *crash*(!2&&!3)            - matches crash but not 2 or 3
 *   *tekken*||*ridge*racer*    - matches tekken OR ridge racer
 *   !*demo*                    - matches anything NOT containing demo
 *
 * Returns: 1 if matches, 0 if not
 */
static const char *pattern_parse_ptr;
static const char *pattern_game_name;

/* Forward declarations for recursive descent parser */
static int pattern_eval_or(void);
static int pattern_eval_and(void);
static int pattern_eval_not(void);
static int pattern_eval_primary(void);

static void pattern_skip_ws(void) {
    while (*pattern_parse_ptr == ' ' || *pattern_parse_ptr == '\t')
        pattern_parse_ptr++;
}

static int pattern_eval_primary(void) {
    pattern_skip_ws();

    if (*pattern_parse_ptr == '(') {
        pattern_parse_ptr++;  /* Skip ( */
        int result = pattern_eval_or();
        pattern_skip_ws();
        if (*pattern_parse_ptr == ')') pattern_parse_ptr++;  /* Skip ) */
        return result;
    }

    /* Extract wildcard pattern until operator or end */
    char wildcard[128];
    int i = 0;
    while (*pattern_parse_ptr &&
           *pattern_parse_ptr != '!' &&
           *pattern_parse_ptr != '&' &&
           *pattern_parse_ptr != '|' &&
           *pattern_parse_ptr != '(' &&
           *pattern_parse_ptr != ')' &&
           i < 126) {
        wildcard[i++] = *pattern_parse_ptr++;
    }
    wildcard[i] = '\0';

    /* Trim trailing whitespace */
    while (i > 0 && (wildcard[i-1] == ' ' || wildcard[i-1] == '\t')) {
        wildcard[--i] = '\0';
    }

    if (i == 0) return 1;  /* Empty pattern matches all */

    return wildcard_match(wildcard, pattern_game_name);
}

static int pattern_eval_not(void) {
    pattern_skip_ws();
    if (*pattern_parse_ptr == '!') {
        pattern_parse_ptr++;  /* Skip ! */
        return !pattern_eval_not();  /* Recursive for !! */
    }
    return pattern_eval_primary();
}

static int pattern_eval_and(void) {
    int result = pattern_eval_not();
    while (1) {
        pattern_skip_ws();
        if (pattern_parse_ptr[0] == '&' && pattern_parse_ptr[1] == '&') {
            pattern_parse_ptr += 2;  /* Skip && */
            int right = pattern_eval_not();
            result = result && right;
        } else {
            break;
        }
    }
    return result;
}

static int pattern_eval_or(void) {
    int result = pattern_eval_and();
    while (1) {
        pattern_skip_ws();
        if (pattern_parse_ptr[0] == '|' && pattern_parse_ptr[1] == '|') {
            pattern_parse_ptr += 2;  /* Skip || */
            int right = pattern_eval_and();
            result = result || right;
        } else {
            break;
        }
    }
    return result;
}

static int match_name_pattern(const char *game_name, const char *pattern)
{
    if (!pattern || !pattern[0]) return 1;  /* No pattern = match all */
    if (!game_name) return 0;

    pattern_parse_ptr = pattern;
    pattern_game_name = game_name;

    return pattern_eval_or();
}

/*
 * Get game name from game_path (filename without extension)
 */
static void get_game_name(char *name, int maxlen)
{
    const char *base = strrchr(game_path, '/');
    if (!base) base = strrchr(game_path, '\\');
    if (base) base++; else base = game_path;

    strncpy(name, base, maxlen - 1);
    name[maxlen - 1] = '\0';

    /* Remove extension */
    char *dot = strrchr(name, '.');
    if (dot) *dot = '\0';
}

/*
 * Scan config directory for matching SLUS config
 * Reads slus_code and name_pattern fields from each config
 * Returns 1 if found and loaded into result_path
 */
static int find_matching_slus_config(char *result_path, int maxlen)
{
    char game_name[256];
    get_game_name(game_name, sizeof(game_name));

    XLOG("v340: Scanning for SLUS config matching CdromId=%s, game=%s", CdromId, game_name);

    int dir = fs_opendir("/mnt/sda1/cores/config");
    if (dir < 0) {
        XLOG("v340: Cannot open config directory");
        return 0;
    }

    /* Buffer for readdir - needs to match firmware's dirent structure */
    struct {
        unsigned int d_ino;
        char d_name[256];
    } entry;

    while (fs_readdir(dir, &entry) > 0) {
        /* Skip non-.cfg files */
        int len = strlen(entry.d_name);
        if (len < 5 || strcmp(entry.d_name + len - 4, ".cfg") != 0) continue;

        /* Skip pcsx4all.cfg (global) and native/asm configs */
        if (strcmp(entry.d_name, "pcsx4all.cfg") == 0) continue;
        if (strcmp(entry.d_name, "psx_native.cfg") == 0) continue;
        if (strcmp(entry.d_name, "psx_asm.cfg") == 0) continue;

        /* Build full path and read config */
        char cfg_path[300];
        snprintf(cfg_path, sizeof(cfg_path), "/mnt/sda1/cores/config/%s", entry.d_name);

        FILE *f = fopen(cfg_path, "r");
        if (!f) continue;

        char slus_codes[256] = "";
        /* Support up to 8 name patterns */
        char name_patterns[8][256];
        int pattern_count = 0;
        char line[512];

        for (int i = 0; i < 8; i++) name_patterns[i][0] = '\0';

        while (fgets(line, sizeof(line), f)) {
            if (line[0] == '#' || line[0] == '\n') continue;

            char key[64], value[256];
            if (sscanf(line, "%63[^=]=%255[^\n\r]", key, value) == 2) {
                /* Trim key */
                char *k = key;
                while (*k == ' ' || *k == '\t') k++;
                char *end = k + strlen(k) - 1;
                while (end > k && (*end == ' ' || *end == '\t')) *end-- = '\0';

                if (strcmp(k, "slus_code") == 0) {
                    strncpy(slus_codes, value, sizeof(slus_codes) - 1);
                } else if (strncmp(k, "name_pattern", 12) == 0 && pattern_count < 8) {
                    /* Support name_pattern, name_pattern1, name_pattern2, etc */
                    strncpy(name_patterns[pattern_count], value, 255);
                    name_patterns[pattern_count][255] = '\0';
                    pattern_count++;
                }
            }
        }
        fclose(f);

        /* Check if SLUS code matches */
        if (slus_codes[0] && match_slus_codes(CdromId, slus_codes)) {
            XLOG("v340: SLUS match in %s (codes=%s)", entry.d_name, slus_codes);

            /* Check name patterns if present - match if ANY pattern matches */
            if (pattern_count > 0) {
                int any_match = 0;
                for (int i = 0; i < pattern_count; i++) {
                    if (match_name_pattern(game_name, name_patterns[i])) {
                        XLOG("v340: Name pattern %d matched: %s", i, name_patterns[i]);
                        any_match = 1;
                        break;
                    }
                }
                if (any_match) {
                    strncpy(result_path, cfg_path, maxlen - 1);
                    result_path[maxlen - 1] = '\0';
                    fs_closedir(dir);
                    return 1;
                } else {
                    XLOG("v340: No name pattern matched (tried %d patterns)", pattern_count);
                    /* Continue searching other configs */
                }
            } else {
                /* No name pattern - SLUS match is enough */
                strncpy(result_path, cfg_path, maxlen - 1);
                result_path[maxlen - 1] = '\0';
                fs_closedir(dir);
                return 1;
            }
        }
    }

    fs_closedir(dir);
    return 0;
}

static int write_config_file(const char *path)
{
    FILE *f = fopen(path, "w");
    if (!f) return 0;

    fprintf(f, "# QPSX v%s Config\n", QPSX_VERSION);
    fprintf(f, "frameskip = %d\n", qpsx_config.frameskip);
    fprintf(f, "target_speed = %d\n", qpsx_config.target_speed);
    fprintf(f, "resampler_type = %d\n", qpsx_config.resampler_type);
    /* Key remapping */
    fprintf(f, "remap_a = %d\n", qpsx_config.remap_a);
    fprintf(f, "remap_b = %d\n", qpsx_config.remap_b);
    fprintf(f, "remap_x = %d\n", qpsx_config.remap_x);
    fprintf(f, "remap_y = %d\n", qpsx_config.remap_y);
    fprintf(f, "remap_l = %d\n", qpsx_config.remap_l);
    fprintf(f, "remap_r = %d\n", qpsx_config.remap_r);
    fprintf(f, "remap_up = %d\n", qpsx_config.remap_up);
    fprintf(f, "remap_down = %d\n", qpsx_config.remap_down);
    fprintf(f, "remap_left = %d\n", qpsx_config.remap_left);
    fprintf(f, "remap_right = %d\n", qpsx_config.remap_right);
    /* v353: Save packed flags as individual fields for backward compatibility */
    fprintf(f, "use_bios = %d\n", FLAG_BOOL(qpsx_config.audio_flags, AUDIO_FLAG_USE_BIOS));
    fprintf(f, "bios_file = %s\n", qpsx_config.bios_file);
    fprintf(f, "cycle_multiplier = %d\n", qpsx_config.cycle_mult);
    fprintf(f, "cycle_jump = %d\n", qpsx_config.cycle_jump);  /* v370: Cycle step */
    fprintf(f, "show_fps = %d\n", FLAG_BOOL(qpsx_config.debug_flags, DEBUG_FLAG_SHOW_FPS));
    fprintf(f, "xa_disable = %d\n", FLAG_BOOL(qpsx_config.audio_flags, AUDIO_FLAG_XA_DISABLE));
    fprintf(f, "cdda_disable = %d\n", FLAG_BOOL(qpsx_config.audio_flags, AUDIO_FLAG_CDDA_DISABLE));
    fprintf(f, "spu_update_freq = %d\n", qpsx_config.spu_update_freq);
    fprintf(f, "spu_reverb = %d\n", qpsx_config.spu_reverb);
    fprintf(f, "spu_interpolation = %d\n", qpsx_config.spu_interpolation);
    fprintf(f, "xa_update_freq = %d\n", qpsx_config.xa_update_freq);
    fprintf(f, "spu_tempo = %d\n", qpsx_config.spu_tempo);
    fprintf(f, "spu_irq_always = %d\n", qpsx_config.spu_irq_always);
    fprintf(f, "frame_duping = %d\n", FLAG_BOOL(qpsx_config.gpu_flags, GPU_FLAG_FRAME_DUP));
    fprintf(f, "gpu_dithering = %d\n", FLAG_BOOL(qpsx_config.gpu_flags, GPU_FLAG_DITHERING));
    fprintf(f, "gpu_blending = %d\n", qpsx_config.gpu_blending);  /* Multi-value */
    fprintf(f, "gpu_lighting = %d\n", FLAG_BOOL(qpsx_config.gpu_flags, GPU_FLAG_LIGHTING));
    fprintf(f, "gpu_asm_light = %d\n", FLAG_BOOL(qpsx_config.gpu_flags, GPU_FLAG_ASM_LIGHT));
    fprintf(f, "gpu_asm_blend = %d\n", FLAG_BOOL(qpsx_config.gpu_flags, GPU_FLAG_ASM_BLEND));
    fprintf(f, "gpu_prefetch_tex = %d\n", FLAG_BOOL(qpsx_config.gpu_flags, GPU_FLAG_PREFETCH_TEX));
    fprintf(f, "gpu_pixel_skip = %d\n", FLAG_BOOL(qpsx_config.gpu_flags, GPU_FLAG_PIXEL_SKIP));
    fprintf(f, "gpu_interlace_force = %d\n", qpsx_config.gpu_interlace_force);  /* Multi-value */
    fprintf(f, "gpu_prog_ilace = %d\n", FLAG_BOOL(qpsx_config.gpu_flags, GPU_FLAG_PROG_ILACE));
    /* v367: nosmccheck/nogteflags/gteregsunneeded HARDCODED - not saved */
    fprintf(f, "opt_gte_noflags = %d\n", qpsx_config.opt_gte_noflags);
    fprintf(f, "gte_rtpt_asm = %d\n", qpsx_config.gte_rtpt_asm);
    fprintf(f, "gte_mvmva_asm = %d\n", qpsx_config.gte_mvmva_asm);
    /* v367: skip_code_inv/asm_memwrite HARDCODED - not saved */
    fprintf(f, "debug_log = %d\n", FLAG_BOOL(qpsx_config.debug_flags, DEBUG_FLAG_LOG));
    fprintf(f, "player2 = %d\n", FLAG_BOOL(qpsx_config.input_flags, INPUT_FLAG_PLAYER2));
    fprintf(f, "block_chain = %d\n", qpsx_config.block_chain);
    fprintf(f, "cycle_batch = %d\n", qpsx_config.cycle_batch);
    fprintf(f, "block_cache = %d\n", qpsx_config.block_cache);
    fprintf(f, "cdda_binsav = %d\n", FLAG_BOOL(qpsx_config.audio_flags, AUDIO_FLAG_CDDA_BINSAV));
    fprintf(f, "cdda_preload = %d\n", qpsx_config.cdda_preload);
    fprintf(f, "native_mode = %d\n", qpsx_config.native_mode);
    fprintf(f, "cdda_mode = %d\n", qpsx_cdda_mode);  /* v367: CDDA OFF/PRETEND/ON */

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

            /* Multi-value int fields */
            if (strcmp(k, "frameskip") == 0) qpsx_config.frameskip = value;
            else if (strcmp(k, "target_speed") == 0) qpsx_config.target_speed = value;
            else if (strcmp(k, "resampler_type") == 0) qpsx_config.resampler_type = value;
            /* Key remapping */
            else if (strcmp(k, "remap_a") == 0) qpsx_config.remap_a = value;
            else if (strcmp(k, "remap_b") == 0) qpsx_config.remap_b = value;
            else if (strcmp(k, "remap_x") == 0) qpsx_config.remap_x = value;
            else if (strcmp(k, "remap_y") == 0) qpsx_config.remap_y = value;
            else if (strcmp(k, "remap_l") == 0) qpsx_config.remap_l = value;
            else if (strcmp(k, "remap_r") == 0) qpsx_config.remap_r = value;
            else if (strcmp(k, "remap_up") == 0) qpsx_config.remap_up = value;
            else if (strcmp(k, "remap_down") == 0) qpsx_config.remap_down = value;
            else if (strcmp(k, "remap_left") == 0) qpsx_config.remap_left = value;
            else if (strcmp(k, "remap_right") == 0) qpsx_config.remap_right = value;
            /* v353: Boolean fields -> packed flags (backward compat) */
            else if (strcmp(k, "use_bios") == 0) FLAG_WRITE(qpsx_config.audio_flags, AUDIO_FLAG_USE_BIOS, value);
            else if (strcmp(k, "bios_file") == 0) strncpy(qpsx_config.bios_file, value_str, 63);
            else if (strcmp(k, "cycle_multiplier") == 0) qpsx_config.cycle_mult = value;
            else if (strcmp(k, "cycle_jump") == 0) qpsx_config.cycle_jump = value;  /* v370: Cycle step */
            else if (strcmp(k, "show_fps") == 0) FLAG_WRITE(qpsx_config.debug_flags, DEBUG_FLAG_SHOW_FPS, value);
            else if (strcmp(k, "xa_disable") == 0) FLAG_WRITE(qpsx_config.audio_flags, AUDIO_FLAG_XA_DISABLE, value);
            else if (strcmp(k, "cdda_disable") == 0) FLAG_WRITE(qpsx_config.audio_flags, AUDIO_FLAG_CDDA_DISABLE, value);
            else if (strcmp(k, "spu_update_freq") == 0) qpsx_config.spu_update_freq = value;
            else if (strcmp(k, "spu_reverb") == 0) qpsx_config.spu_reverb = value;
            else if (strcmp(k, "spu_interpolation") == 0) qpsx_config.spu_interpolation = value;
            else if (strcmp(k, "xa_update_freq") == 0) qpsx_config.xa_update_freq = value;
            else if (strcmp(k, "spu_tempo") == 0) qpsx_config.spu_tempo = value;
            else if (strcmp(k, "spu_irq_always") == 0) qpsx_config.spu_irq_always = value;
            else if (strcmp(k, "frame_duping") == 0) FLAG_WRITE(qpsx_config.gpu_flags, GPU_FLAG_FRAME_DUP, value);
            else if (strcmp(k, "gpu_dithering") == 0) FLAG_WRITE(qpsx_config.gpu_flags, GPU_FLAG_DITHERING, value);
            else if (strcmp(k, "gpu_blending") == 0) qpsx_config.gpu_blending = value;  /* Multi-value */
            else if (strcmp(k, "gpu_lighting") == 0) FLAG_WRITE(qpsx_config.gpu_flags, GPU_FLAG_LIGHTING, value);
            /* v352+: Merged fast+asm lighting/blending */
            else if (strcmp(k, "gpu_asm_light") == 0) FLAG_WRITE(qpsx_config.gpu_flags, GPU_FLAG_ASM_LIGHT, value);
            else if (strcmp(k, "gpu_asm_blend") == 0) FLAG_WRITE(qpsx_config.gpu_flags, GPU_FLAG_ASM_BLEND, value);
            /* Backwards compat - old keys set new merged field */
            else if (strcmp(k, "gpu_fast_lighting") == 0) FLAG_WRITE(qpsx_config.gpu_flags, GPU_FLAG_ASM_LIGHT, value);
            else if (strcmp(k, "gpu_fast_blending") == 0) FLAG_WRITE(qpsx_config.gpu_flags, GPU_FLAG_ASM_BLEND, value);
            else if (strcmp(k, "gpu_asm_lighting") == 0) FLAG_WRITE(qpsx_config.gpu_flags, GPU_FLAG_ASM_LIGHT, value);
            else if (strcmp(k, "gpu_asm_blending") == 0) FLAG_WRITE(qpsx_config.gpu_flags, GPU_FLAG_ASM_BLEND, value);
            else if (strcmp(k, "gpu_prefetch_tex") == 0) FLAG_WRITE(qpsx_config.gpu_flags, GPU_FLAG_PREFETCH_TEX, value);
            else if (strcmp(k, "gpu_pixel_skip") == 0) FLAG_WRITE(qpsx_config.gpu_flags, GPU_FLAG_PIXEL_SKIP, value);
            else if (strcmp(k, "gpu_interlace_force") == 0) qpsx_config.gpu_interlace_force = value;  /* Multi-value */
            else if (strcmp(k, "gpu_prog_ilace") == 0) FLAG_WRITE(qpsx_config.gpu_flags, GPU_FLAG_PROG_ILACE, value);
            /* v367: nosmccheck/nogteflags/gteregsunneeded HARDCODED - ignored from config */
            else if (strcmp(k, "opt_gte_noflags") == 0) qpsx_config.opt_gte_noflags = value;
            else if (strcmp(k, "gte_rtpt_asm") == 0) qpsx_config.gte_rtpt_asm = value;
            else if (strcmp(k, "gte_mvmva_asm") == 0) qpsx_config.gte_mvmva_asm = value;
            /* v367: skip_code_inv/asm_memwrite HARDCODED - ignored from config */
            else if (strcmp(k, "debug_log") == 0) FLAG_WRITE(qpsx_config.debug_flags, DEBUG_FLAG_LOG, value);
            else if (strcmp(k, "player2") == 0) FLAG_WRITE(qpsx_config.input_flags, INPUT_FLAG_PLAYER2, value);
            else if (strcmp(k, "block_chain") == 0) qpsx_config.block_chain = value;
            else if (strcmp(k, "cycle_batch") == 0) qpsx_config.cycle_batch = value;
            else if (strcmp(k, "block_cache") == 0) qpsx_config.block_cache = value;
            else if (strcmp(k, "cdda_binsav") == 0) FLAG_WRITE(qpsx_config.audio_flags, AUDIO_FLAG_CDDA_BINSAV, value);
            else if (strcmp(k, "cdda_preload") == 0) qpsx_config.cdda_preload = value;
            else if (strcmp(k, "native_mode") == 0) qpsx_config.native_mode = value;
            else if (strcmp(k, "cdda_mode") == 0) qpsx_cdda_mode = value;  /* v367: CDDA OFF/PRETEND/ON */
            /* Ignored old fields: opt_lohi_cache, opt_nclip_inline, opt_const_prop,
             * opt_rtpt_unroll, opt_gte_unrdiv, asm_block1, asm_block2, direct_block_lut,
             * cdda_fast_mix, cdda_unity_vol, cdda_asm_mix, native_remap_set */
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

/* ============== APPLY CONFIGURATION ============== */
static void qpsx_apply_config(void)
{
    /* Apply frameskip via native GPU mechanism */
    gpu.frameskip.set = qpsx_config.frameskip;

    /* v353: Use FLAG_BOOL for packed booleans */
    fps_show = FLAG_BOOL(qpsx_config.debug_flags, DEBUG_FLAG_SHOW_FPS);

    Config.SpuUpdateFreq = qpsx_config.spu_update_freq;
    Config.ForcedXAUpdates = qpsx_config.xa_update_freq;
    Config.SpuIrq = qpsx_config.spu_irq_always;

    /* v353: Audio flags */
    Config.Xa = FLAG_BOOL(qpsx_config.audio_flags, AUDIO_FLAG_XA_DISABLE);
    /* v367: Config.Cdda synced with qpsx_cdda_mode (OFF/PRETEND=disabled, ON=enabled) */
    Config.Cdda = (qpsx_cdda_mode != 2);  /* 0=OFF, 1=PRETEND -> Cdda=1(disabled); 2=ON -> Cdda=0(enabled) */

    /*
     * v254: DO NOT set cycle_multiplier here!
     * cycle_multiplier is set ONLY ONCE in retro_load_game() at startup.
     */

    /* v353: GPU flags (packed booleans) */
    gpu_unai_config_ext.dithering = FLAG_BOOL(qpsx_config.gpu_flags, GPU_FLAG_DITHERING);
    gpu_unai_config_ext.blending = qpsx_config.gpu_blending;  /* Multi-value, stays as int */
    gpu_unai_config_ext.lighting = FLAG_BOOL(qpsx_config.gpu_flags, GPU_FLAG_LIGHTING);
    /* v352: Merged fields - both fast and asm use same toggle */
    gpu_unai_config_ext.fast_lighting = FLAG_BOOL(qpsx_config.gpu_flags, GPU_FLAG_ASM_LIGHT);
    gpu_unai_config_ext.fast_blending = FLAG_BOOL(qpsx_config.gpu_flags, GPU_FLAG_ASM_BLEND);
    gpu_unai_config_ext.asm_lighting = FLAG_BOOL(qpsx_config.gpu_flags, GPU_FLAG_ASM_LIGHT);
    gpu_unai_config_ext.asm_blending = FLAG_BOOL(qpsx_config.gpu_flags, GPU_FLAG_ASM_BLEND);
    gpu_unai_config_ext.prefetch_tex = FLAG_BOOL(qpsx_config.gpu_flags, GPU_FLAG_PREFETCH_TEX);
    gpu_unai_config_ext.pixel_skip = FLAG_BOOL(qpsx_config.gpu_flags, GPU_FLAG_PIXEL_SKIP);
    /* v369: pixel_size REMOVED - adds ~240 IFs per frame overhead */
    gpu_unai_config_ext.pixel_size = 0;   /* Always 1x1 */
    gpu_unai_config_ext.ilace_force = 0;  /* GPU renders ALL lines */
#ifndef USE_GPULIB
    gpu_unai_config_ext.prog_ilace = FLAG_BOOL(qpsx_config.gpu_flags, GPU_FLAG_PROG_ILACE);
#endif

#ifdef SPU_PCSXREARMED
    spu_config.iUseReverb = qpsx_config.spu_reverb;
    spu_config.iUseInterpolation = qpsx_config.spu_interpolation;
    spu_config.iTempo = qpsx_config.spu_tempo;
#endif

    /* v367: qpsx_nosmccheck, g_opt_skip_code_inv, g_opt_asm_memwrite HARDCODED in their files */

    /* v335: Target Speed - update global and re-init timing */
    if (g_target_speed != qpsx_config.target_speed) {
        g_target_speed = qpsx_config.target_speed;
        extern void psxRcntReinitTiming(void);
        psxRcntReinitTiming();
    }

    /* v338: Resampler type - update global */
    g_resampler_type = qpsx_config.resampler_type;

    /* v353: CDDA options from flags */
    g_opt_cdda_binsav = FLAG_BOOL(qpsx_config.audio_flags, AUDIO_FLAG_CDDA_BINSAV);
    g_opt_cdda_preload = qpsx_config.cdda_preload;  /* Multi-value */

    /* v353: Debug flags */
    update_debug_log_state(FLAG_BOOL(qpsx_config.debug_flags, DEBUG_FLAG_LOG));
    /* v368: profiler_set_enabled REMOVED - profiler disabled at compile time */

    /* v368: Update player 2 global for fast access in update_input_cache() */
    g_player2_enabled = FLAG_BOOL(qpsx_config.input_flags, INPUT_FLAG_PLAYER2);

    /* v351: Rebuild pre-compiled remap LUT when config changes */
    remap_rebuild_lut();
}

/* ============== PROFILER OVERLAY (v108 - FIXED MATH) ============== */
/* v192: Wall-clock frame timing */
static unsigned long last_frame_usec = 0;
static float real_frame_ms_avg = 0;

static void draw_profiler_overlay(uint16_t *pixels)
{
    if (!g_profiler.enabled) return;

    /* v192: Measure REAL wall-clock time between frames */
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    unsigned long now_usec = tv_now.tv_sec * 1000000UL + tv_now.tv_usec;
    float real_frame_ms = 0;
    if (last_frame_usec > 0) {
        real_frame_ms = (now_usec - last_frame_usec) / 1000.0f;
        /* Smoothing: 90% old + 10% new */
        real_frame_ms_avg = real_frame_ms_avg * 0.9f + real_frame_ms * 0.1f;
    }
    last_frame_usec = now_usec;

    const ProfilerData *prof = profiler_get_data();
    char buf[64];
    const uint16_t color = 0xFFFF;  /* White */
    const uint16_t color_yellow = 0xFFE0;  /* Yellow for totals */
    const uint16_t color_red = 0xF800;  /* Red for hotspots */
    const uint16_t color_green = 0x07E0;  /* Green for OK */
    const uint16_t bg_color = 0x0000;  /* Black */

    /* v108: Full profiler with correct Frame sum */
    int height = g_profiler.detailed ? 150 : 95;
    DrawFBox(pixels, 2, 16, 175, height, bg_color);

    int y = 18;

    /* v192: REAL wall-clock frame time FIRST */
    int real_fps = (real_frame_ms_avg > 0.1f) ? (int)(1000.0f / real_frame_ms_avg) : 0;
    snprintf(buf, sizeof(buf), "REAL:%.1fms %dFPS", real_frame_ms_avg, real_fps);
    DrawText(pixels, 4, y, (real_fps >= 50) ? color_green : color_red, buf);
    y += 9;

    /* === Get all component times === */
    float cpu_total = prof->avg_ms[PROF_CPU_TOTAL];
    float gte_total = prof->avg_ms[PROF_GTE_TOTAL];
    float gpu_total = prof->avg_ms[PROF_GPU_TOTAL];
    float spu_total = prof->avg_ms[PROF_SPU_TOTAL];
    float cd_total = prof->avg_ms[PROF_CDROM_TOTAL];
    float video_ms = prof->avg_ms[PROF_VIDEO_OUTPUT];

    /*
     * v108: Frame = SUM of all components (not separate timer!)
     * Note: GTE is already included in CPU (called from CPU emulation)
     * So we don't add GTE separately to avoid double-counting
     */
    float frame_ms = cpu_total + gpu_total + spu_total + cd_total + video_ms;
    if (frame_ms < 0.1f) frame_ms = 0.1f;  /* Avoid div by zero */

    /* Calculate FPS and % of target speed (50 FPS = 20ms) */
    float fps = 1000.0f / frame_ms;
    float pct = (20.0f / frame_ms) * 100.0f;  /* 20ms = 50 FPS target */

    /* Profiled components sum */
    snprintf(buf, sizeof(buf), "Prof:%.1fms (%dFPS %d%%)",
             frame_ms, (int)fps, (int)pct);
    DrawText(pixels, 4, y, pct >= 100 ? color_green : color_yellow, buf);
    y += 9;

    /* === CPU BREAKDOWN === */
    float mem_r = prof->avg_ms[PROF_CPU_MEM_READ];
    float mem_w = prof->avg_ms[PROF_CPU_MEM_WRITE];
    float hw_r = prof->avg_ms[PROF_CPU_HW_READ];
    float hw_w = prof->avg_ms[PROF_CPU_HW_WRITE];
    float exc = prof->avg_ms[PROF_CPU_EXCEPTION];
    float bios = prof->avg_ms[PROF_CPU_BIOS];
    float comp = prof->avg_ms[PROF_CPU_COMPILE];
    float known = mem_r + mem_w + hw_r + hw_w + exc + bios + comp;
    float exec = cpu_total - known;
    if (exec < 0) exec = 0;

    snprintf(buf, sizeof(buf), "CPU:%.1fms (GTE:%.1f inside)", cpu_total, gte_total);
    DrawText(pixels, 4, y, color_yellow, buf);
    y += 8;

    if (g_profiler.detailed) {
        snprintf(buf, sizeof(buf), " Exec:%.1f Comp:%.2f", exec, comp);
        DrawText(pixels, 4, y, exec > 15 ? color_red : color, buf);
        y += 8;
        snprintf(buf, sizeof(buf), " MemR:%.1f MemW:%.1f", mem_r, mem_w);
        DrawText(pixels, 4, y, (mem_r + mem_w) > 5 ? color_red : color, buf);
        y += 8;
        snprintf(buf, sizeof(buf), " HW:%.2f Exc:%.2f Bio:%.2f", hw_r + hw_w, exc, bios);
        DrawText(pixels, 4, y, color, buf);
        y += 8;
    }

    /* === GPU === */
    snprintf(buf, sizeof(buf), "GPU:%.1fms", gpu_total);
    DrawText(pixels, 4, y, color_yellow, buf);
    y += 8;
    if (g_profiler.detailed) {
        snprintf(buf, sizeof(buf), " Poly:%.1f Spr:%.1f",
                 prof->avg_ms[PROF_GPU_POLY], prof->avg_ms[PROF_GPU_SPRITE]);
        DrawText(pixels, 4, y, color, buf);
        y += 8;
    }

    /* === SPU + CD + CDDA + Video === */
    float cdda_ms = prof->avg_ms[PROF_CDDA];
    snprintf(buf, sizeof(buf), "SPU:%.1f CD:%.1f CDDA:%.1f Vid:%.1f",
             spu_total, cd_total, cdda_ms, video_ms);
    DrawText(pixels, 4, y, (cdda_ms > 3 || video_ms > 5) ? color_red : color, buf);
    y += 8;
    /* v367b: Native Mode statistics REMOVED (hardcoded OFF) */
}

/* ============== v258: CDDA TOOLS SUBMENU ============== */
#define CDDA_MENU_X 20
#define CDDA_MENU_Y 15
#define CDDA_MENU_W 280
#define CDDA_MENU_H 210
#define CDDA_LINE_H 10
#define CDDA_VISIBLE 8

/* Menu states */
#define CDDA_STATE_LIST      0   /* Show track list + options */
#define CDDA_STATE_CONFIRM   1   /* Ask: WAV / ADPCM / Cancel */
#define CDDA_STATE_CONVERT   2   /* Converting with progress */
#define CDDA_STATE_DELETE    3   /* Delete mode - select what to delete */

static int cdda_state = CDDA_STATE_LIST;
static int cdda_confirm_sel = 0;  /* 0=WAV, 1=ADPCM, 2=Cancel */
static int cdda_delete_sel = 0;   /* 0=.binwav, 1=.binadpcm, 2=.bin, 3=Cancel */
static int cdda_convert_idx = 0;  /* Current track being converted */
static int cdda_convert_format = 0; /* 1=WAV, 2=ADPCM */
static int cdda_tracks_done = 0;
static int cdda_tracks_to_convert = 0;

/* Draw progress bar */
static void draw_progress_bar(uint16_t *pixels, int x, int y, int w, int h, int pct) {
    /* Border */
    DrawBox(pixels, x, y, w, h, MENU_BORDER);
    /* Background */
    DrawFBox(pixels, x + 1, y + 1, w - 2, h - 2, RGB565(4, 4, 4));
    /* Fill */
    int fill_w = ((w - 4) * pct) / 100;
    if (fill_w > 0) {
        DrawFBox(pixels, x + 2, y + 2, fill_w, h - 4, RGB565(0, 255, 0));
    }
}

static void draw_cdda_tools_submenu(uint16_t *pixels)
{
    char buf[128];
    int y;
    int num_tracks = cdda_get_num_tracks();

    /* Dark overlay background */
    DrawFBox(pixels, CDDA_MENU_X, CDDA_MENU_Y, CDDA_MENU_W, CDDA_MENU_H, MENU_BG);
    DrawBox(pixels, CDDA_MENU_X, CDDA_MENU_Y, CDDA_MENU_W, CDDA_MENU_H, MENU_BORDER);
    DrawBox(pixels, CDDA_MENU_X + 1, CDDA_MENU_Y + 1, CDDA_MENU_W - 2, CDDA_MENU_H - 2, MENU_BORDER);

    y = CDDA_MENU_Y + 5;

    /* Title */
    DrawText(pixels, CDDA_MENU_X + 85, y, MENU_TITLE, "CDDA TOOLS");
    y += CDDA_LINE_H + 2;

    /* Warnings - BRIGHT colors! */
    DrawText(pixels, CDDA_MENU_X + 8, y, RGB565(255, 255, 0), "! PC conversion is MUCH faster !");
    y += CDDA_LINE_H;
    DrawText(pixels, CDDA_MENU_X + 8, y, RGB565(255, 64, 64), "! DO NOT power off during convert !");
    y += CDDA_LINE_H;
    DrawSeparator(pixels, CDDA_MENU_X + 5, y, CDDA_MENU_W - 10, MENU_SEP);
    y += 5;

    /* ===== STATE: CONVERTING ===== */
    if (cdda_state == CDDA_STATE_CONVERT) {
        DrawText(pixels, CDDA_MENU_X + 8, y, RGB565(255, 255, 255), "CONVERTING - PLEASE WAIT");
        y += CDDA_LINE_H + 5;

        /* Track progress */
        snprintf(buf, sizeof(buf), "Track: %d / %d", cdda_tracks_done + 1, cdda_tracks_to_convert);
        DrawText(pixels, CDDA_MENU_X + 8, y, MENU_FG, buf);
        y += CDDA_LINE_H + 2;

        /* Current track progress bar */
        int track_pct = 0;
        if (cdda_tools_progress_sectors > 0) {
            track_pct = (cdda_tools_progress_sector * 100) / cdda_tools_progress_sectors;
        }
        snprintf(buf, sizeof(buf), "Current: %d%%", track_pct);
        DrawText(pixels, CDDA_MENU_X + 8, y, MENU_FG, buf);
        y += CDDA_LINE_H;
        draw_progress_bar(pixels, CDDA_MENU_X + 8, y, CDDA_MENU_W - 20, 12, track_pct);
        y += 18;

        /* Total progress */
        snprintf(buf, sizeof(buf), "Total: %d%%", cdda_tools_progress_pct);
        DrawText(pixels, CDDA_MENU_X + 8, y, MENU_FG, buf);
        y += CDDA_LINE_H;
        draw_progress_bar(pixels, CDDA_MENU_X + 8, y, CDDA_MENU_W - 20, 12, cdda_tools_progress_pct);
        y += 18;

        /* Sector info */
        snprintf(buf, sizeof(buf), "Sector: %d / %d",
                 cdda_tools_progress_sector, cdda_tools_progress_sectors);
        DrawText(pixels, CDDA_MENU_X + 8, y, MENU_DIM, buf);
        y += CDDA_LINE_H;

        /* v267: ETA display */
        if (cdda_tools_progress_pct > 2) {
            snprintf(buf, sizeof(buf), "ETA: %d:%02d", cdda_eta_minutes, cdda_eta_seconds);
        } else {
            snprintf(buf, sizeof(buf), "ETA: calculating...");
        }
        DrawText(pixels, CDDA_MENU_X + 8, y, RGB565(128, 255, 255), buf);
        y += CDDA_LINE_H + 5;

        /* Cancel instruction */
        DrawText(pixels, CDDA_MENU_X + 8, y, RGB565(255, 128, 128), "B: CANCEL (deletes incomplete file)");

        return;
    }

    /* ===== STATE: CONFIRM FORMAT ===== */
    if (cdda_state == CDDA_STATE_CONFIRM) {
        DrawText(pixels, CDDA_MENU_X + 8, y, RGB565(255, 255, 255), "CONVERT ALL TRACKS TO:");
        y += CDDA_LINE_H + 8;

        const char *opts[3] = {
            "22kHz WAV  (4x smaller)",
            "ADPCM      (16x smaller)",
            "Cancel"
        };
        for (int i = 0; i < 3; i++) {
            const char *sel = (i == cdda_confirm_sel) ? "> " : "  ";
            uint16_t col = (i == cdda_confirm_sel) ? MENU_SEL : MENU_FG;
            if (i == 2) col = (i == cdda_confirm_sel) ? MENU_SEL : MENU_DIM;
            snprintf(buf, sizeof(buf), "%s%s", sel, opts[i]);
            DrawText(pixels, CDDA_MENU_X + 30, y, col, buf);
            y += CDDA_LINE_H + 4;
        }

        y += 10;
        DrawText(pixels, CDDA_MENU_X + 8, y, MENU_DIM, "Up/Down: select   A: confirm");
        return;
    }

    /* ===== STATE: DELETE SELECT ===== */
    if (cdda_state == CDDA_STATE_DELETE) {
        DrawText(pixels, CDDA_MENU_X + 8, y, RGB565(255, 255, 255), "DELETE ALL (keeps at least 1 version):");
        y += CDDA_LINE_H + 8;

        const char *opts[4] = {
            "Delete all .binwav files",
            "Delete all .binadpcm files",
            "Delete all .bin (originals)",
            "Cancel"
        };
        for (int i = 0; i < 4; i++) {
            const char *sel = (i == cdda_delete_sel) ? "> " : "  ";
            uint16_t col = (i == cdda_delete_sel) ? MENU_SEL : MENU_FG;
            if (i == 3) col = (i == cdda_delete_sel) ? MENU_SEL : MENU_DIM;
            if (i < 3) col = (i == cdda_delete_sel) ? MENU_SEL : RGB565(255, 128, 128);
            snprintf(buf, sizeof(buf), "%s%s", sel, opts[i]);
            DrawText(pixels, CDDA_MENU_X + 20, y, col, buf);
            y += CDDA_LINE_H + 4;
        }

        y += 10;
        DrawText(pixels, CDDA_MENU_X + 8, y, MENU_DIM, "Up/Down: select   A: confirm");
        return;
    }

    /* ===== STATE: LIST ===== */
    /* No tracks found? */
    if (num_tracks == 0) {
        DrawText(pixels, CDDA_MENU_X + 8, y, MENU_DIM, "No CDDA audio tracks found.");
        y += CDDA_LINE_H * 2;
        DrawText(pixels, CDDA_MENU_X + 8, y, MENU_DIM, "This game has no CD audio.");
        DrawText(pixels, CDDA_MENU_X + 8, CDDA_MENU_Y + CDDA_MENU_H - 14, MENU_DIM, "B: back");
        return;
    }

    /* Track list header */
    DrawText(pixels, CDDA_MENU_X + 8, y, MENU_SEP, "Track   .bin   .wav   .adpcm");
    y += CDDA_LINE_H + 2;

    /* Calculate scroll */
    int start_item = cdda_tools_scroll;
    int end_item = start_item + CDDA_VISIBLE;
    if (end_item > num_tracks) end_item = num_tracks;

    /* Scroll up indicator */
    if (start_item > 0) {
        DrawText(pixels, CDDA_MENU_X + CDDA_MENU_W - 25, y - 2, MENU_DIM, "^^^");
    }

    /* Draw track list */
    for (int i = start_item; i < end_item; i++) {
        cdda_track_info_t *ti = cdda_get_track_info(i);
        if (!ti) continue;

        /* Show versions: [*] = in use, [X] = exists, [-] = missing */
        char bin_mark = ti->has_bin ? (ti->current_format == CDDA_FMT_BIN ? '*' : 'X') : '-';
        char wav_mark = ti->has_binwav ? (ti->current_format == CDDA_FMT_BINWAV ? '*' : 'X') : '-';
        char adp_mark = ti->has_binadpcm ? (ti->current_format == CDDA_FMT_BINADPCM ? '*' : 'X') : '-';

        snprintf(buf, sizeof(buf), "  T%02d    [%c]    [%c]    [%c]",
                 ti->track_num, bin_mark, wav_mark, adp_mark);
        DrawText(pixels, CDDA_MENU_X + 8, y, MENU_FG, buf);
        y += CDDA_LINE_H;
    }

    /* Scroll down indicator */
    if (end_item < num_tracks) {
        DrawText(pixels, CDDA_MENU_X + CDDA_MENU_W - 25, y, MENU_DIM, "vvv");
    }

    /* Legend */
    y = CDDA_MENU_Y + CDDA_MENU_H - 48;
    DrawText(pixels, CDDA_MENU_X + 8, y, MENU_DIM, "[*]=in use  [X]=exists  [-]=missing");
    y += CDDA_LINE_H + 2;
    DrawSeparator(pixels, CDDA_MENU_X + 5, y, CDDA_MENU_W - 10, MENU_SEP);
    y += 5;

    /* Actions */
    DrawText(pixels, CDDA_MENU_X + 8, y, RGB565(128, 255, 128), "A: CONVERT ALL   X: DELETE   B: back");
}

/*
 * v261: Incremental conversion - processes N sectors per frame
 * Returns: 1=more work, 0=all done, -2=cancelled
 */
static int cdda_do_convert_step(void) {
    int num_tracks = cdda_get_num_tracks();

    /* If a track conversion is in progress, continue it */
    if (cdda_is_converting()) {
        int result = cdda_convert_step();

        /* Update progress from incremental API */
        int sector, total_sectors, track_idx;
        cdda_get_convert_progress(&sector, &total_sectors, &track_idx);
        cdda_tools_progress_sector = sector;
        cdda_tools_progress_sectors = total_sectors;

        /* v266: Update total progress continuously, not just after track completes
         * Total = (completed_tracks * 100 + current_track_percent) / total_tracks
         */
        if (cdda_tracks_to_convert > 0) {
            int track_pct = (total_sectors > 0) ? (sector * 100) / total_sectors : 0;
            cdda_tools_progress_pct = ((cdda_tracks_done * 100) + track_pct) / cdda_tracks_to_convert;
        }

        /* v267: Calculate ETA based on progress rate */
        int progress_done = cdda_tools_progress_pct - cdda_eta_start_pct;
        if (progress_done > 2) {  /* Wait for at least 2% progress for accuracy */
            uint32_t elapsed_ms = os_get_tick_count() - cdda_eta_start_time;
            int remaining_pct = 100 - cdda_tools_progress_pct;
            /* eta_ms = (remaining * elapsed) / progress_done */
            uint32_t eta_ms = ((uint32_t)remaining_pct * elapsed_ms) / (uint32_t)progress_done;
            cdda_eta_seconds = (eta_ms / 1000) % 60;
            cdda_eta_minutes = (eta_ms / 1000) / 60;
            if (cdda_eta_minutes > 999) cdda_eta_minutes = 999;  /* Cap at 999 min */
        }

        if (result == 1) {
            /* More work on this track */
            return 1;
        } else if (result == 0) {
            /* Track finished! Move to next */
            cdda_tracks_done++;
            cdda_convert_idx++;
            /* Progress already updated above */
            /* Fall through to start next track or finish */
        } else if (result == -2) {
            /* Cancelled */
            return -2;
        } else {
            /* Error - skip this track */
            cdda_convert_idx++;
            /* Fall through to try next track */
        }
    }

    /* Find next track to convert */
    while (cdda_convert_idx < num_tracks) {
        cdda_track_info_t *ti = cdda_get_track_info(cdda_convert_idx);

        /* Skip if no .bin or already has target format */
        int skip = 0;
        if (!ti || !ti->has_bin) skip = 1;
        if (cdda_convert_format == 1 && ti && ti->has_binwav) skip = 1;
        if (cdda_convert_format == 2 && ti && ti->has_binadpcm) skip = 1;

        if (skip) {
            cdda_convert_idx++;
            continue;
        }

        /* Start converting this track */
        cdda_tools_progress_sectors = ti->bin_sectors;
        cdda_tools_progress_sector = 0;
        
        int start_result = cdda_start_convert_track(cdda_convert_idx, cdda_convert_format);
        if (start_result < 0) {
            /* Failed to start - skip this track */
            cdda_convert_idx++;
            continue;
        }
        
        return 1;  /* More work to do */
    }

    return 0;  /* All done */
}

static void handle_cdda_tools_input(void)
{
    static int prev_up = 0, prev_down = 0;
    static int prev_a = 0, prev_b = 0, prev_x = 0;

    int cur_up = input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_UP);
    int cur_down = input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_DOWN);
    int cur_a = input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_A);
    int cur_b = input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_B);
    int cur_x = input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_X);

    int num_tracks = cdda_get_num_tracks();

    /* ===== STATE: CONVERTING ===== */
    if (cdda_state == CDDA_STATE_CONVERT) {
        /* Check for B button = cancel */
        if (cur_b && !prev_b) {
            cdda_request_cancel();
        }
        
        /* Do one conversion step per frame for progress updates */
        int result = cdda_do_convert_step();
        if (result == 0) {
            /* All done! */
            cdda_state = CDDA_STATE_LIST;
            cdda_tools_converting = 0;
            cdda_scan_tracks();
            snprintf(feedback_msg, sizeof(feedback_msg), "Converted %d tracks!", cdda_tracks_done);
            feedback_timer = 180;
        } else if (result == -2) {
            /* Cancelled */
            cdda_state = CDDA_STATE_LIST;
            cdda_tools_converting = 0;
            cdda_scan_tracks();
            snprintf(feedback_msg, sizeof(feedback_msg), "Cancelled! %d tracks done.", cdda_tracks_done);
            feedback_timer = 180;
        }
        /* result == 1: more work to do, continue next frame */
        
        prev_up = cur_up; prev_down = cur_down;
        prev_a = cur_a; prev_b = cur_b; prev_x = cur_x;
        return;
    }

    /* ===== STATE: CONFIRM FORMAT ===== */
    if (cdda_state == CDDA_STATE_CONFIRM) {
        if (cur_up && !prev_up) {
            cdda_confirm_sel--;
            if (cdda_confirm_sel < 0) cdda_confirm_sel = 2;
        }
        if (cur_down && !prev_down) {
            cdda_confirm_sel++;
            if (cdda_confirm_sel > 2) cdda_confirm_sel = 0;
        }
        if (cur_b && !prev_b) {
            cdda_state = CDDA_STATE_LIST;
        }
        if (cur_a && !prev_a) {
            if (cdda_confirm_sel == 2) {
                /* Cancel */
                cdda_state = CDDA_STATE_LIST;
            } else {
                /* Start conversion */
                cdda_convert_format = cdda_confirm_sel + 1;  /* 1=WAV, 2=ADPCM */
                cdda_convert_idx = 0;
                cdda_tracks_done = 0;
                cdda_tools_progress_pct = 0;

                /* Count tracks to convert */
                cdda_tracks_to_convert = 0;
                for (int i = 0; i < num_tracks; i++) {
                    cdda_track_info_t *ti = cdda_get_track_info(i);
                    if (!ti || !ti->has_bin) continue;
                    if (cdda_convert_format == 1 && ti->has_binwav) continue;
                    if (cdda_convert_format == 2 && ti->has_binadpcm) continue;
                    cdda_tracks_to_convert++;
                }

                if (cdda_tracks_to_convert > 0) {
                    cdda_state = CDDA_STATE_CONVERT;
                    cdda_tools_converting = 1;
                    /* v267: Initialize ETA tracking */
                    cdda_eta_start_time = os_get_tick_count();
                    cdda_eta_start_pct = 0;  /* Starting fresh */
                    cdda_eta_minutes = 0;
                    cdda_eta_seconds = 0;
                } else {
                    snprintf(feedback_msg, sizeof(feedback_msg), "Nothing to convert!");
                    feedback_timer = 120;
                    cdda_state = CDDA_STATE_LIST;
                }
            }
        }
        prev_up = cur_up; prev_down = cur_down;
        prev_a = cur_a; prev_b = cur_b; prev_x = cur_x;
        return;
    }

    /* ===== STATE: DELETE SELECT ===== */
    if (cdda_state == CDDA_STATE_DELETE) {
        if (cur_up && !prev_up) {
            cdda_delete_sel--;
            if (cdda_delete_sel < 0) cdda_delete_sel = 3;
        }
        if (cur_down && !prev_down) {
            cdda_delete_sel++;
            if (cdda_delete_sel > 3) cdda_delete_sel = 0;
        }
        if (cur_b && !prev_b) {
            cdda_state = CDDA_STATE_LIST;
        }
        if (cur_a && !prev_a) {
            if (cdda_delete_sel == 3) {
                /* Cancel */
                cdda_state = CDDA_STATE_LIST;
            } else {
                /* Delete selected format from all tracks */
                /* Map menu selection to correct format constant:
                 * Menu: 0=binwav, 1=binadpcm, 2=bin
                 * Constants: CDDA_FMT_BIN=0, CDDA_FMT_BINWAV=1, CDDA_FMT_BINADPCM=2
                 */
                int format_map[3] = { CDDA_FMT_BINWAV, CDDA_FMT_BINADPCM, CDDA_FMT_BIN };
                int format = format_map[cdda_delete_sel];
                int deleted = 0;
                for (int i = 0; i < num_tracks; i++) {
                    if (cdda_delete_version(i, format) == 0) {
                        deleted++;
                    }
                }
                cdda_scan_tracks();
                snprintf(feedback_msg, sizeof(feedback_msg), "Deleted %d files", deleted);
                feedback_timer = 120;
                cdda_state = CDDA_STATE_LIST;
            }
        }
        prev_up = cur_up; prev_down = cur_down;
        prev_a = cur_a; prev_b = cur_b; prev_x = cur_x;
        return;
    }

    /* ===== STATE: LIST ===== */
    /* B = back to main menu */
    if (cur_b && !prev_b) {
        cdda_tools_active = 0;
        cdda_state = CDDA_STATE_LIST;
    }

    /* Up/Down = scroll track list */
    if (cur_up && !prev_up && num_tracks > CDDA_VISIBLE) {
        if (cdda_tools_scroll > 0) cdda_tools_scroll--;
    }
    if (cur_down && !prev_down && num_tracks > CDDA_VISIBLE) {
        if (cdda_tools_scroll < num_tracks - CDDA_VISIBLE) cdda_tools_scroll++;
    }

    /* A = Convert all (open confirm) */
    if (cur_a && !prev_a && num_tracks > 0) {
        cdda_confirm_sel = 0;
        cdda_state = CDDA_STATE_CONFIRM;
    }

    /* X = Delete (open delete menu) */
    if (cur_x && !prev_x && num_tracks > 0) {
        cdda_delete_sel = 0;
        cdda_state = CDDA_STATE_DELETE;
    }

    prev_up = cur_up; prev_down = cur_down;
    prev_a = cur_a; prev_b = cur_b; prev_x = cur_x;
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
    DrawText(pixels, MENU_X + 55, y, MENU_AUTHOR, "by Grzegorz Korycki @the_q_dev");
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

        /* v367: Full menu display - GPU/SMC/Dynarec HARDCODED like v066 */
        switch (item) {
            case MI_FRAMESKIP:
                snprintf(buf, sizeof(buf), "%s%-12s: %d", sel, mi->name, qpsx_config.frameskip);
                break;
            case MI_TARGET_SPEED:
                snprintf(buf, sizeof(buf), "%s%-12s: %d%%", sel, mi->name, qpsx_config.target_speed);
                break;
            case MI_RESAMPLER_TYPE:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.resampler_type ? "Cubic" : "Linear");
                if (qpsx_config.target_speed >= 100 && menu_item != item) col = MENU_DIM;
                break;
            case MI_BIOS:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, FLAG_GET(qpsx_config.audio_flags, AUDIO_FLAG_USE_BIOS) ? "Real" : "HLE");
                break;
            case MI_CYCLE:
                snprintf(buf, sizeof(buf), "%s%-12s: %d", sel, mi->name, qpsx_config.cycle_mult);
                break;
            case MI_CYCLE_JUMP:
                snprintf(buf, sizeof(buf), "%s%-12s: %d", sel, mi->name, qpsx_config.cycle_jump);
                break;
            case MI_SHOW_FPS:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, FLAG_GET(qpsx_config.debug_flags, DEBUG_FLAG_SHOW_FPS) ? "ON" : "OFF");
                break;
            case MI_PLAYER2:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, FLAG_GET(qpsx_config.input_flags, INPUT_FLAG_PLAYER2) ? "ON" : "OFF");
                break;
            case MI_DEBUG_LOG:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, FLAG_GET(qpsx_config.debug_flags, DEBUG_FLAG_LOG) ? "ON" : "OFF");
                break;
            case MI_XA_AUDIO:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, FLAG_GET(qpsx_config.audio_flags, AUDIO_FLAG_XA_DISABLE) ? "OFF" : "ON");
                col = (!FLAG_GET(qpsx_config.audio_flags, AUDIO_FLAG_XA_DISABLE)) ? ((menu_item == item) ? MENU_SEL : MENU_GREEN) : ((menu_item == item) ? MENU_SEL : MENU_RED);
                break;
            case MI_CDDA_MODE:
                {
                    const char *cdda_mode_str[] = {"OFF", "PRETEND", "ON"};
                    snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, cdda_mode_str[qpsx_cdda_mode % 3]);
                    if (qpsx_cdda_mode == 0) col = (menu_item == item) ? MENU_SEL : MENU_RED;
                    else if (qpsx_cdda_mode == 1) col = (menu_item == item) ? MENU_SEL : MENU_YELLOW;
                    else col = (menu_item == item) ? MENU_SEL : MENU_GREEN;
                }
                break;
            case MI_CDDA_PRELOAD:
                {
                    const char *preload_str[] = {"OFF", "<0.5M", "<1M", "<1.5M"};
                    snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, preload_str[qpsx_config.cdda_preload & 3]);
                }
                break;
            case MI_CDDA_TOOLS:
                snprintf(buf, sizeof(buf), "%s%-12s  [->]", sel, mi->name);
                break;
            /* v369: MI_PIXEL_SIZE removed - line doubling adds overhead */
            case MI_REMAP_A:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, remap_btn_names[qpsx_config.remap_a % 14]);
                break;
            case MI_REMAP_B:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, remap_btn_names[qpsx_config.remap_b % 14]);
                break;
            case MI_REMAP_X:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, remap_btn_names[qpsx_config.remap_x % 14]);
                break;
            case MI_REMAP_Y:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, remap_btn_names[qpsx_config.remap_y % 14]);
                break;
            case MI_REMAP_L:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, remap_btn_names[qpsx_config.remap_l % 14]);
                break;
            case MI_REMAP_R:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, remap_btn_names[qpsx_config.remap_r % 14]);
                break;
            case MI_REMAP_UP:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, remap_btn_names[qpsx_config.remap_up % 14]);
                break;
            case MI_REMAP_DOWN:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, remap_btn_names[qpsx_config.remap_down % 14]);
                break;
            case MI_REMAP_LEFT:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, remap_btn_names[qpsx_config.remap_left % 14]);
                break;
            case MI_REMAP_RIGHT:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, remap_btn_names[qpsx_config.remap_right % 14]);
                break;
            case MI_RESTORE_PAD:
                snprintf(buf, sizeof(buf), "%s[%s]", sel, mi->name);
                col = (menu_item == item) ? MENU_SEL : RGB565(0xAA, 0xFF, 0xAA);
                marker = "";
                break;
            case MI_SWAP_CD:
                snprintf(buf, sizeof(buf), "%s[%s]", sel, mi->name);
                col = (menu_item == item) ? MENU_SEL : RGB565(255, 128, 0);
                marker = "";
                break;
            case MI_SAVE_GAME:
            case MI_SAVE_SLUS:
            case MI_SAVE_GLOBAL:
            case MI_DEL_GAME:
            case MI_DEL_GLOBAL:
            case MI_EXIT:
                snprintf(buf, sizeof(buf), "%s[%s]", sel, mi->name);
                if (menu_item == item) col = MENU_SEL;
                else if (item == MI_SAVE_GAME || item == MI_SAVE_SLUS || item == MI_SAVE_GLOBAL) col = MENU_GREEN;
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
    static int prev_a = 0, prev_start = 0, prev_b = 0, prev_x = 0;  /* v343: added prev_x */
    static int repeat_delay = 0;

    if (menu_first_frame) {
        prev_up = prev_down = prev_left = prev_right = prev_a = prev_start = prev_b = prev_x = 1;
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
    int cur_x     = input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_X);  /* v343: X to close menu */
    int cur_start = input_state_cb(0, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_START);

    /* v343: Close menu with X or START (B now decreases option values) */
    if ((cur_x && !prev_x) || (cur_start && !prev_start)) {
        menu_active = 0;
        menu_first_frame = 1;
        start_hold_frames = 0;
        qpsx_apply_config();
        prev_x = cur_x;
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

    /* v355: Left/Right = page scroll (MENU_VISIBLE items) with wrap-around */
    if (cur_left && !prev_left) {
        for (int i = 0; i < MENU_VISIBLE; i++) {
            do { menu_item--; if (menu_item < 0) menu_item = MENU_ITEMS - 1; } while (menu_items[menu_item].type == 1);
        }
        repeat_delay = 8;
    }
    if (cur_right && !prev_right) {
        for (int i = 0; i < MENU_VISIBLE; i++) {
            do { menu_item++; if (menu_item >= MENU_ITEMS) menu_item = 0; } while (menu_items[menu_item].type == 1);
        }
        repeat_delay = 8;
    }

    /* v367: Full menu handlers - GPU/SMC/Dynarec HARDCODED like v066 */
    if (menu_items[menu_item].type == 0) {
        if (cur_b && !prev_b) {  /* B decreases */
            switch (menu_item) {
                case MI_FRAMESKIP: qpsx_config.frameskip = (qpsx_config.frameskip > 0) ? qpsx_config.frameskip - 1 : 5; break;
                case MI_TARGET_SPEED: qpsx_config.target_speed = (qpsx_config.target_speed > 40) ? qpsx_config.target_speed - 10 : 100; break;
                case MI_RESAMPLER_TYPE: qpsx_config.resampler_type = !qpsx_config.resampler_type; break;
                case MI_BIOS: FLAG_TOGGLE(qpsx_config.audio_flags, AUDIO_FLAG_USE_BIOS); break;
                case MI_CYCLE: {
                    /* v370: Use cycle_jump for step, align to multiples */
                    int step = qpsx_config.cycle_jump;
                    int val = qpsx_config.cycle_mult - step;
                    if (val < 256) val = 2048;
                    /* Align to multiple of step */
                    val = (val / step) * step;
                    if (val < 256) val = 256;
                    qpsx_config.cycle_mult = val;
                } break;
                case MI_CYCLE_JUMP: {
                    /* Cycle through 32/64/128/256 (backwards) */
                    if (qpsx_config.cycle_jump == 32) qpsx_config.cycle_jump = 256;
                    else if (qpsx_config.cycle_jump == 64) qpsx_config.cycle_jump = 32;
                    else if (qpsx_config.cycle_jump == 128) qpsx_config.cycle_jump = 64;
                    else qpsx_config.cycle_jump = 128;
                } break;
                case MI_SHOW_FPS: FLAG_TOGGLE(qpsx_config.debug_flags, DEBUG_FLAG_SHOW_FPS); break;
                case MI_PLAYER2: FLAG_TOGGLE(qpsx_config.input_flags, INPUT_FLAG_PLAYER2); break;
                case MI_DEBUG_LOG: FLAG_TOGGLE(qpsx_config.debug_flags, DEBUG_FLAG_LOG); break;
                case MI_XA_AUDIO: FLAG_TOGGLE(qpsx_config.audio_flags, AUDIO_FLAG_XA_DISABLE); break;
                case MI_CDDA_MODE: qpsx_cdda_mode = (qpsx_cdda_mode > 0) ? qpsx_cdda_mode - 1 : 2; break;
                case MI_CDDA_PRELOAD: qpsx_config.cdda_preload = (qpsx_config.cdda_preload > 0) ? qpsx_config.cdda_preload - 1 : 3; break;
                /* v369: MI_PIXEL_SIZE removed */
                case MI_REMAP_A: qpsx_config.remap_a = (qpsx_config.remap_a + 13) % 14; break;
                case MI_REMAP_B: qpsx_config.remap_b = (qpsx_config.remap_b + 13) % 14; break;
                case MI_REMAP_X: qpsx_config.remap_x = (qpsx_config.remap_x + 13) % 14; break;
                case MI_REMAP_Y: qpsx_config.remap_y = (qpsx_config.remap_y + 13) % 14; break;
                case MI_REMAP_L: qpsx_config.remap_l = (qpsx_config.remap_l + 13) % 14; break;
                case MI_REMAP_R: qpsx_config.remap_r = (qpsx_config.remap_r + 13) % 14; break;
                case MI_REMAP_UP: qpsx_config.remap_up = (qpsx_config.remap_up + 13) % 14; break;
                case MI_REMAP_DOWN: qpsx_config.remap_down = (qpsx_config.remap_down + 13) % 14; break;
                case MI_REMAP_LEFT: qpsx_config.remap_left = (qpsx_config.remap_left + 13) % 14; break;
                case MI_REMAP_RIGHT: qpsx_config.remap_right = (qpsx_config.remap_right + 13) % 14; break;
            }
            qpsx_apply_config();
            repeat_delay = 8;
        }

        if (cur_a && !prev_a) {  /* A increases */
            switch (menu_item) {
                case MI_FRAMESKIP: qpsx_config.frameskip = (qpsx_config.frameskip < 5) ? qpsx_config.frameskip + 1 : 0; break;
                case MI_TARGET_SPEED: qpsx_config.target_speed = (qpsx_config.target_speed < 100) ? qpsx_config.target_speed + 10 : 40; break;
                case MI_RESAMPLER_TYPE: qpsx_config.resampler_type = !qpsx_config.resampler_type; break;
                case MI_BIOS: FLAG_TOGGLE(qpsx_config.audio_flags, AUDIO_FLAG_USE_BIOS); break;
                case MI_CYCLE: {
                    /* v370: Use cycle_jump for step, align to multiples */
                    int step = qpsx_config.cycle_jump;
                    int val = qpsx_config.cycle_mult + step;
                    if (val > 2048) val = 256;
                    /* Align to multiple of step */
                    val = (val / step) * step;
                    if (val > 2048) val = 2048;
                    qpsx_config.cycle_mult = val;
                } break;
                case MI_CYCLE_JUMP: {
                    /* Cycle through 32/64/128/256 (forwards) */
                    if (qpsx_config.cycle_jump == 32) qpsx_config.cycle_jump = 64;
                    else if (qpsx_config.cycle_jump == 64) qpsx_config.cycle_jump = 128;
                    else if (qpsx_config.cycle_jump == 128) qpsx_config.cycle_jump = 256;
                    else qpsx_config.cycle_jump = 32;
                } break;
                case MI_SHOW_FPS: FLAG_TOGGLE(qpsx_config.debug_flags, DEBUG_FLAG_SHOW_FPS); break;
                case MI_PLAYER2: FLAG_TOGGLE(qpsx_config.input_flags, INPUT_FLAG_PLAYER2); break;
                case MI_DEBUG_LOG: FLAG_TOGGLE(qpsx_config.debug_flags, DEBUG_FLAG_LOG); break;
                case MI_XA_AUDIO: FLAG_TOGGLE(qpsx_config.audio_flags, AUDIO_FLAG_XA_DISABLE); break;
                case MI_CDDA_MODE: qpsx_cdda_mode = (qpsx_cdda_mode < 2) ? qpsx_cdda_mode + 1 : 0; break;
                case MI_CDDA_PRELOAD: qpsx_config.cdda_preload = (qpsx_config.cdda_preload < 3) ? qpsx_config.cdda_preload + 1 : 0; break;
                /* v369: MI_PIXEL_SIZE removed */
                case MI_REMAP_A: qpsx_config.remap_a = (qpsx_config.remap_a + 1) % 14; break;
                case MI_REMAP_B: qpsx_config.remap_b = (qpsx_config.remap_b + 1) % 14; break;
                case MI_REMAP_X: qpsx_config.remap_x = (qpsx_config.remap_x + 1) % 14; break;
                case MI_REMAP_Y: qpsx_config.remap_y = (qpsx_config.remap_y + 1) % 14; break;
                case MI_REMAP_L: qpsx_config.remap_l = (qpsx_config.remap_l + 1) % 14; break;
                case MI_REMAP_R: qpsx_config.remap_r = (qpsx_config.remap_r + 1) % 14; break;
                case MI_REMAP_UP: qpsx_config.remap_up = (qpsx_config.remap_up + 1) % 14; break;
                case MI_REMAP_DOWN: qpsx_config.remap_down = (qpsx_config.remap_down + 1) % 14; break;
                case MI_REMAP_LEFT: qpsx_config.remap_left = (qpsx_config.remap_left + 1) % 14; break;
                case MI_REMAP_RIGHT: qpsx_config.remap_right = (qpsx_config.remap_right + 1) % 14; break;
            }
            qpsx_apply_config();
            repeat_delay = 8;
        }
    }

    /* v370: CDDA Tools opens with A or B button */
    if (((cur_a && !prev_a) || (cur_b && !prev_b)) && menu_item == MI_CDDA_TOOLS) {
        cdda_scan_tracks();
        cdda_set_progress_callback(cdda_progress_update);
        cdda_tools_active = 1;
        cdda_tools_item = 0;
        cdda_tools_scroll = 0;
        cdda_tools_action = 0;
        prev_a = cur_a;
        prev_b = cur_b;
        return;
    }

    /* v370: Action handlers - both A and B select actions */
    if (((cur_a && !prev_a) || (cur_b && !prev_b)) && (menu_items[menu_item].type == 2 || menu_items[menu_item].type == 3)) {
        char path[256];
        switch (menu_item) {
            case MI_SWAP_CD:
                fb_init_game_dir();
                fb_scan_cue_files();
                cd_browser_active = 1;
                XLOG("Opening CD browser: %s", fb_game_dir);
                break;
            case MI_RESTORE_PAD:
                /* v370: Restore defaults - A=Circle, B=Cross (Japanese layout) */
                qpsx_config.remap_a = 1;  /* O (Circle) - confirm */
                qpsx_config.remap_b = 0;  /* X (Cross) - cancel */
                qpsx_config.remap_x = 3;  /* Triangle */
                qpsx_config.remap_y = 2;  /* Square */
                qpsx_config.remap_l = 4;  /* L1 */
                qpsx_config.remap_r = 5;  /* R1 */
                qpsx_config.remap_up = 10;    /* Up */
                qpsx_config.remap_down = 11;  /* Down */
                qpsx_config.remap_left = 12;  /* Left */
                qpsx_config.remap_right = 13; /* Right */
                set_feedback("Pad restored to defaults");
                break;
            case MI_SAVE_GAME:
                get_game_config_path(path, sizeof(path));
                set_feedback(write_config_file(path) ? "Game config saved!" : "Save FAILED!");
                break;
            case MI_SAVE_SLUS:
                get_slus_config_path(path, sizeof(path));
                if (path[0] != '\0') {
                    set_feedback(write_config_file(path) ? "SLUS config saved!" : "Save FAILED!");
                } else {
                    set_feedback("No SLUS/SCES ID!");
                }
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
    prev_a = cur_a; prev_b = cur_b; prev_x = cur_x; prev_start = cur_start;  /* v343: added prev_x */
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
/*
 * v340: Config loading priority:
 * 1. Exact ROM/CD filename config (e.g., "Crash Bandicoot.cue.cfg")
 * 2. SLUS config matching CdromId + optional name_pattern
 *    - Scans all configs for slus_code field with comma-separated codes
 *    - If name_pattern present, game name must match pattern
 * 3. Global config (pcsx4all.cfg)
 * 4. Defaults
 */
static void qpsx_load_config(void)
{
    char cfg_path[300];

    /* 1. Try exact filename-based config (primary) */
    get_game_config_path(cfg_path, sizeof(cfg_path));
    if (load_config_file(cfg_path)) {
        XLOG("v340: Loaded exact game config: %s", cfg_path);
        return;
    }

    /* 2. Try SLUS config matching with multi-code + pattern support */
    if (CdromId[0] != '\0') {
        /* 2a. First try exact SLUS filename (e.g., SCUS-94900.cfg) */
        get_slus_config_path(cfg_path, sizeof(cfg_path));
        if (cfg_path[0] != '\0' && load_config_file(cfg_path)) {
            XLOG("v340: Loaded exact SLUS config: %s", cfg_path);
            return;
        }

        /* 2b. Scan configs for matching slus_code + name_pattern */
        if (find_matching_slus_config(cfg_path, sizeof(cfg_path))) {
            if (load_config_file(cfg_path)) {
                XLOG("v340: Loaded matched SLUS config: %s", cfg_path);
                return;
            }
        }
    }

    /* 3. Try global config */
    if (load_config_file(QPSX_GLOBAL_CONFIG_PATH)) {
        XLOG("v340: Loaded global config");
        return;
    }

    XLOG("v340: No config found, using defaults");
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

    /* v092: Initialize profiler */
    profiler_init();

    video_clear();
    XLOG("=== retro_init() DONE ===");
}

void retro_deinit(void)
{
    XLOG("=== retro_deinit() ===");
    /* v343: Remove crash marker on normal exit */
    safe_mode_remove_marker();
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
    /* v141: Removed verbose retro_run logging */

    if (run_frame_count == 0) {
        XLOG("retro_run() frame 0");

        /* v244: Initialize native command menu */
        native_cmd_menu_init();
        native_cmd_menu_load_config();

        /* v285: Initialize ASM Select menu */
        asm_cmd_init();
        asm_cmd_menu_load_config();  /* v288: Load saved ASM toggles */

        /* v320: Safe mode - check for crash on previous run */
        /* v340: Fixed bug - marker was never removed after safe mode triggered */
        if (safe_mode_check_marker()) {
            XLOG("v340: SAFE MODE - previous run crashed! Opening main config menu.");
            safe_mode_triggered = 1;
            menu_active = 1;
            menu_first_frame = 1;
        }
        /* v340: Always write marker - if we crash before 3.5s stability, next run triggers safe mode */
        safe_mode_write_marker();
        XLOG("v340: Wrote crash marker (will remove after 3.5s stable)");

        /* v367b: Native command menus REMOVED - g_opt_native_mode always 0 */

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

    /* v340: Safe mode - remove crash marker after stable running */
    /* v343: Use >= instead of == to ensure removal even if skipped */
    safe_mode_frames++;
    if (safe_mode_frames >= SAFE_MODE_FRAMES && safe_mode_frames < SAFE_MODE_FRAMES + 5) {
        safe_mode_remove_marker();
        XLOG("v343: Removed crash marker - emulation stable after 3.5s");
    }

    update_fps_counter();

    /* Cache input ONCE per retro_run - SF2000 hardware can't handle frequent polling */
    update_input_cache();

    /* v244: Native command menu mode - pause emulation and show menu */
    /* v368: native_cmd_menu and asm_cmd_menu REMOVED (g_opt_native_mode always 0) */

    /* v368: cd_browser moved inside menu_active block below */

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
        /* v368: CD browser is sub-mode of menu - check first */
        if (cd_browser_active) {
            handle_cd_browser_input();
            if (SCREEN) {
                draw_cd_browser(SCREEN);
                video_cb(SCREEN, SCREEN_WIDTH, SCREEN_HEIGHT, VIRTUAL_WIDTH * sizeof(uint16_t));
            }
            return;  /* Don't run emulation while in browser */
        }

        /* v258: CDDA Tools submenu takes priority */
        if (cdda_tools_active) {
            handle_cdda_tools_input();
        } else {
            handle_menu_input();
        }

        if (SCREEN) {
            /*
             * QPSX_084: Invisible auto-menu
             * When auto_menu_frames > 0, skip drawing overlay - user sees game, not menu
             * When auto_menu_frames == 0, normal manual menu - draw overlay
             */
            if (auto_menu_frames == 0) {
                /* v258: Draw CDDA Tools submenu if active, else main menu */
                if (cdda_tools_active) {
                    draw_cdda_tools_submenu(SCREEN);
                } else {
                    draw_menu_overlay(SCREEN);  /* Manual menu - draw overlay */
                }
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

    /*
     * v107: Frame duping - skip every other video output when enabled
     * This reduces firmware video_cb overhead (~14ms per call)
     * User sees it as "frame duping" - reusing previous frame.
     */
    if (FLAG_GET(qpsx_config.gpu_flags, GPU_FLAG_FRAME_DUP)) {
        skip_video_output = (run_frame_count & 1);  /* Skip odd frames */
    } else {
        skip_video_output = 0;
    }

    /*
     * v349: TARGET_SPEED THROTTLING (OPCJA D)
     *
     * Problem: scaled_pal_cycles zmieniał TIMING VSync, ale emulator nadal
     * emulował te same instrukcje - nie zmniejszało to ilości PRACY!
     * Wynik: fps spadał proporcjonalnie do target_speed (80% = 80% fps).
     *
     * Fix: Zamiast zmieniać timing, POMIJAMY niektóre wywołania Execute().
     * Przy 80% target: emulujemy 4 z 5 klatek, piąta to frame dupe.
     * Przy 100%: brak throttlingu (pełna prędkość).
     *
     * Accumulator approach:
     * - Przy każdym retro_run() dodajemy g_target_speed do akumulatora
     * - Gdy accumulator >= 100: emuluj klatkę, odejmij 100
     * - Gdy < 100: skip (frame dupe poprzedniej klatki)
     */
    static int frame_throttle_acc = 100;  /* Start at 100 to emit first frame */

    if (g_target_speed >= 100) {
        /* Normal speed - no throttling, just execute */
        psxCpu->Execute();
    } else {
        frame_throttle_acc += g_target_speed;
        if (frame_throttle_acc >= 100) {
            frame_throttle_acc -= 100;
            /* Emulate this frame */
            psxCpu->Execute();
        } else {
            /* Skip emulation - frame dupe (output previous frame buffer) */
            if (SCREEN && real_video_cb) {
                real_video_cb(SCREEN, SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_WIDTH * 2);
            }
        }
    }
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

#ifdef PSXREC
    /* v254: Set cycle_multiplier ONLY HERE at startup, not in qpsx_apply_config()
     * This value is baked into dynarec blocks at compile time and cannot be
     * changed safely while emulation is running. */
    cycle_multiplier = qpsx_config.cycle_mult;
    XLOG("cycle_multiplier set to %u (startup only)", cycle_multiplier);
#endif

    memset(&Config, 0, sizeof(Config));

    Config.Xa = FLAG_BOOL(qpsx_config.audio_flags, AUDIO_FLAG_XA_DISABLE);
    /* v367: Config.Cdda synced with qpsx_cdda_mode */
    Config.Cdda = (qpsx_cdda_mode != 2);  /* OFF/PRETEND=disabled, ON=enabled */
    Config.Mdec = 0;
    Config.PsxAuto = 1;

    Config.HLE = FLAG_GET(qpsx_config.audio_flags, AUDIO_FLAG_USE_BIOS) ? 0 : 1;
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

    /* QPSX_277: FIX - Reinitialize timing after region detection! */
    psxRcntReinitTiming();
    /* QPSX_280: FIX - Initialize plugin_lib AFTER region detection!
     * Without this, pl_data.frame_interval stays 0, causing infinite loop
     * in pl_frame_limit() for NTSC games. PAL games "worked" because
     * the config mismatch triggered pl_frameskip_prepare(). */
    pl_init();


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
