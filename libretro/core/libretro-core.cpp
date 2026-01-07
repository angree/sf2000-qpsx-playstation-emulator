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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
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
    uint16_t pad2 = 0xFFFF;  /* v251: Player 2 support */

    if (input_poll_cb) input_poll_cb();

    if (input_state_cb) {
        /* Player 1 - Map libretro buttons to PSX controller bits */
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

        /* v251: Player 2 - same mapping on port 1 */
        if (input_state_cb(1, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_SELECT))   pad2 &= ~(1 << 0);
        if (input_state_cb(1, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_L3))       pad2 &= ~(1 << 1);
        if (input_state_cb(1, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_R3))       pad2 &= ~(1 << 2);
        if (input_state_cb(1, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_START))    pad2 &= ~(1 << 3);
        if (input_state_cb(1, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_UP))       pad2 &= ~(1 << 4);
        if (input_state_cb(1, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_RIGHT))    pad2 &= ~(1 << 5);
        if (input_state_cb(1, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_DOWN))     pad2 &= ~(1 << 6);
        if (input_state_cb(1, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_LEFT))     pad2 &= ~(1 << 7);
        if (input_state_cb(1, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_L2))       pad2 &= ~(1 << 8);
        if (input_state_cb(1, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_R2))       pad2 &= ~(1 << 9);
        if (input_state_cb(1, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_L))        pad2 &= ~(1 << 10);
        if (input_state_cb(1, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_R))        pad2 &= ~(1 << 11);
        if (input_state_cb(1, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_X))        pad2 &= ~(1 << 12);
        if (input_state_cb(1, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_A))        pad2 &= ~(1 << 13);
        if (input_state_cb(1, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_B))        pad2 &= ~(1 << 14);
        if (input_state_cb(1, RETRO_DEVICE_JOYPAD, 0, RETRO_DEVICE_ID_JOYPAD_Y))        pad2 &= ~(1 << 15);
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

#define QPSX_VERSION "270"
#define QPSX_GLOBAL_CONFIG_PATH "/mnt/sda1/cores/config/pcsx4all.cfg"
#define QPSX_NATIVE_CONFIG_PATH "/mnt/sda1/cores/config/psx_native.cfg"

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

    /* v110: CDDA speedups (runtime options) */
    int cdda_fast_mix;      /* Skip spu.spuMem writes for SPU Capture */
    int cdda_unity_vol;     /* Skip volume multiply when vol=100% */
    int cdda_asm_mix;       /* v111: MIPS32 ASM CDDA mixer */
    int cdda_binsav;        /* v257: Use .binadpcm/.binwav for CDDA if available */

    /* GPU */
    int gpu_dithering;
    int gpu_blending;
    int gpu_lighting;
    int gpu_fast_lighting;
    int gpu_fast_blending;  /* v090: Branchless blending for MIPS32 */
    int gpu_asm_lighting;   /* v091: MIPS32 ASM lighting (MOVN/MOVZ) */
    int gpu_asm_blending;   /* v091: MIPS32 ASM blending (EXT/INS) */
    int gpu_prefetch_tex;   /* v091: Texture cache prefetch (PREF) */
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

    /* v095: ASM blocks */
    int asm_block1;         /* ASM Block 1: NCLIP */
    int asm_block2;         /* ASM Block 2: AVSZ3 */

    /* v098: Memory optimizations */
    int skip_code_inv;      /* Skip code invalidation (RISK!) */
    int asm_memwrite;       /* ASM memory write path */

    /* v101: Direct Block LUT */
    int direct_block_lut;   /* Direct Block LUT dispatch loop */

    /* v112: Speed hacks */
    int cycle_batch;        /* Cycle batching level: 0=OFF, 1=Light(2x), 2=Med(4x), 3=Aggr(8x) */
    int block_cache;        /* Hot Block Cache: 0=OFF, 1=256, 2=1K, 3=4K, 4=16K, 5=64K, 6=256K, 7=1M entries */

    /* v115: Native Mode - run compatible MIPS I instructions directly
     * v236: 3=NATIVE (run native, fallback dynarec), 4=DIFF (compare native vs dynarec) */
    int native_mode;        /* 0=OFF, 1=STATS, 2=FAST, 3=NATIVE, 4=DIFF_TEST */

    /* Display */
    int show_fps;
    int profiler;         /* v092: Performance profiler overlay */

    /* Debug */
    int debug_log;        /* 0=off, 1=write to /mnt/sda1/log.txt */

} qpsx_config = {
    /* Defaults = v071 optimal */
    0,                  /* frameskip = 0 (disabled - causes issues) */
    1,                  /* use_bios = 1 (real BIOS) */
    "scph5501.bin",     /* bios_file */
    1024,               /* cycle_mult = 1024 (v250 default) */

    /* SPU */
    0, 0, 0, 0, 0, 0,

    /* Audio */
    0, 0,               /* xa_disable, cdda_disable = 0 */
    1, 1, 1,            /* v111: cdda_fast_mix=ON, cdda_unity_vol=ON, cdda_asm_mix=ON */
    1,                  /* v257: cdda_binsav=ON (use .binadpcm/.binwav if available) */

    /* GPU */
    0,                  /* gpu_dithering = 0 */
    1,                  /* gpu_blending = 1 */
    1,                  /* gpu_lighting = 1 */
    1,                  /* gpu_fast_lighting = 1 (ON - v089 default for speed) */
    1,                  /* gpu_fast_blending = 1 (ON - v090 default for MIPS32 speed) */
    0,                  /* gpu_asm_lighting = 0 (OFF - v091 experimental) */
    0,                  /* gpu_asm_blending = 0 (OFF - v091 experimental) */
    0,                  /* gpu_prefetch_tex = 0 (OFF - v091 experimental) */
    1,                  /* gpu_pixel_skip = 1 */
    0,                  /* gpu_interlace_force = 0 */
    0,                  /* gpu_prog_ilace = 0 */
    0,                  /* frame_duping = 0 */

    /* Dynarec */
    0, 0, 0,            /* nosmccheck, nogteflags, gteregsunneeded = 0 */

    /* Dynarec optimizations - v084: first 3 ON, v087: rtpt_unroll OFF (experimental) */
    1, 1, 1, 0,         /* opt_lohi_cache, opt_nclip_inline, opt_const_prop, opt_rtpt_unroll=OFF */

    /* GTE optimizations - v093: ENABLED for performance testing! */
    1, 1,               /* opt_gte_unrdiv=ON, opt_gte_noflags=ON */

    /* v095: ASM blocks - OFF by default, enable to test */
    0, 0,               /* asm_block1=OFF, asm_block2=OFF */

    /* v098: Memory optimizations - OFF by default (risky) */
    0, 0,               /* skip_code_inv=OFF, asm_memwrite=OFF */

    /* v101: Direct Block LUT - OFF by default */
    0,                  /* direct_block_lut=OFF */

    /* v112: Speed hacks - OFF by default (risky!) */
    0,                  /* cycle_batch=OFF (0=every block, 1=2x, 2=4x, 3=8x) */
    0,                  /* block_cache=OFF (0-7: OFF,256,1K,4K,16K,64K,256K,1M entries) */

    /* v241: Native Mode - OFF by default (full speed) */
    0,                  /* v241: native_mode=OFF (0=OFF, 1=STATS, 2=FAST, 3=NATIVE, 4=DIFF) */

    /* Display */
    0,                  /* show_fps = 0 (off by default) */
    0,                  /* profiler = 0 (off by default) */

    /* Debug */
    0                   /* debug_log = 0 (off by default) */
};

int qpsx_nosmccheck = 0;

/* Dynarec optimization flags - exported for use in recompiler and GTE */
int g_opt_lohi_cache = 0;     /* LO/HI register caching */
int g_opt_nclip_inline = 0;   /* NCLIP GTE operation inline */
int g_opt_const_prop = 0;     /* Extended constant propagation */
int g_opt_rtpt_unroll = 0;    /* RTPT loop unroll (v087) */

/* GTE optimization flags (v093) - ENABLED for testing */
int g_opt_gte_unrdiv = 1;     /* UNR (Newton-Raphson) divide instead of hardware DIV */
int g_opt_gte_noflags = 1;    /* Skip overflow flag checking - RISK! Can break some games! */

/* QPSX v095: Hand-written MIPS32 ASM blocks - OFF by default, enable to test */
int g_opt_asm_block1 = 0;     /* ASM Block 1: NCLIP (backface culling) */
int g_opt_asm_block2 = 0;     /* ASM Block 2: AVSZ3 (Z averaging) */

/* QPSX v098: Memory write optimizations */
int g_opt_skip_code_inv = 0;  /* Skip code invalidation after stores (RISK: breaks self-mod code!) */
int g_opt_asm_memwrite = 0;   /* Use ASM optimized memory write path */

/* QPSX v101: Direct Block LUT - single-level block lookup (default OFF) */
int g_opt_direct_block_lut = 0;  /* Direct Block LUT dispatch loop */

/* QPSX v110: CDDA speedup options (runtime) */
int g_opt_cdda_fast_mix = 1;     /* Skip spu.spuMem writes for SPU Capture */
int g_opt_cdda_unity_vol = 1;    /* Skip volume multiply when vol=100% */
int g_opt_cdda_asm_mix = 1;      /* v111: MIPS32 ASM CDDA mixer (default ON) */
int g_opt_cdda_binsav = 1;       /* v257: Use .binadpcm/.binwav for CDDA if available */

/* QPSX v112: Cycle batching - reduce dispatch loop overhead */
/* 0=OFF (check every block), 1=Light (2x), 2=Medium (4x), 3=Aggressive (8x) */
int g_opt_cycle_batch = 0;       /* Cycle batch level (default OFF for safety) */

/* QPSX v113: Hot Block Cache - L1/L2 friendly dispatch cache */
/* Caches recent PC->code mappings for faster dispatch. Uses SF2000's 128MB RAM. */
/* 0=OFF, 1=256, 2=1K, 3=4K, 4=16K, 5=64K, 6=256K, 7=1M entries (8 bytes each) */
int g_opt_block_cache = 0;       /* Hot Block Cache level (default OFF) */

/* QPSX v115: Native Mode - run MIPS I instructions directly on MIPS32 */
/* PSX (R3000A = MIPS I) instructions are BINARY COMPATIBLE with SF2000's MIPS32! */
/* Inspired by Sony POPS (PSP PS1 emulator) semi-native execution model */
/* 0=OFF, 1=STATS (show native ratio), 2=FAST (skip const prop), 3=SEMI (semi-native) */
int g_opt_native_mode = 0;       /* v241: Native Mode OFF (full speed default) */

/* v114/v115: Native mode stats from recompiler */
extern "C" int native_mode_get_ratio(void);
extern "C" int native_mode_get_pure_block_ratio(void);
extern "C" void native_stats_reset_frame(void);

/* ============== MENU ITEMS ============== */
typedef struct {
    const char *name;
    int type;           /* 0=option, 1=separator, 2=action, 3=preset */
    int restart;        /* 1=needs restart, 0=instant apply */
} MenuItem;

#define MENU_ITEMS 57
enum {
    MI_SEP_HEADER = 0,
    MI_FRAMESKIP,
    MI_BIOS,
    MI_CYCLE,
    MI_SHOW_FPS,
    MI_PROFILER,    /* v092: Performance profiler */
    MI_DEBUG_LOG,
    MI_SEP_AUDIO,
    MI_XA_AUDIO,
    MI_CDDA_AUDIO,
    MI_CDDA_FAST_MIX,   /* v110: CDDA fast mix (skip spu.spuMem) */
    MI_CDDA_UNITY_VOL,  /* v110: CDDA unity volume (skip multiply) */
    MI_CDDA_ASM_MIX,    /* v111: MIPS32 ASM CDDA mixer */
    MI_CDDA_BINSAV,     /* v255: Use .binsav (22kHz mono) for CDDA */
    MI_CDDA_TOOLS,      /* v258: CDDA conversion/delete tools */
    MI_SEP_GPU,
    MI_DITHERING,
    MI_BLENDING,
    MI_LIGHTING,
    MI_FAST_LIGHT,
    MI_FAST_BLEND,  /* v090: Branchless blending */
    MI_ASM_LIGHT,   /* v091: MIPS32 ASM lighting */
    MI_ASM_BLEND,   /* v091: MIPS32 ASM blending */
    MI_PREFETCH,    /* v091: Texture prefetch */
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
    MI_SEP_ASM,         /* v095: ASM blocks separator */
    MI_ASM_BLOCK1,      /* v095: ASM Block 1 (NCLIP) */
    MI_ASM_BLOCK2,      /* v095: ASM Block 2 (AVSZ3) */
    MI_SEP_MEMOPT,      /* v098: Memory optimization separator */
    MI_SKIP_CODE_INV,   /* v098: Skip code invalidation (RISK!) */
    MI_ASM_MEMWRITE,    /* v098: ASM memory write */
    MI_DIRECT_BLOCK_LUT,/* v101: Direct Block LUT dispatch */
    MI_SEP_SPEED,       /* v112: Speed hacks separator */
    MI_CYCLE_BATCH,     /* v112: Cycle batching (RISK!) */
    MI_BLOCK_CACHE,     /* v112: Block result cache */
    MI_NATIVE_MODE,     /* v114: Native MIPS execution mode */
    MI_SEP_CONFIG,
    MI_SAVE_GAME,
    MI_SAVE_SLUS,
    MI_SAVE_GLOBAL,
    MI_DEL_GAME,
    MI_DEL_GLOBAL,
    MI_SEP_EXIT,
    MI_EXIT,
    MI_SEP_FOOTER
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
    {"CPU Cycle",           0, 1},  /* v252: Restart required - cycles baked into dynarec blocks */
    {"Show FPS",            0, 0},  /* Instant - display only */
    {"Profiler",            0, 0},  /* Instant - v092 profiler overlay */
    {"Debug Log",           0, 0},  /* Instant - file toggle */
    {"--- AUDIO ---",       1, 0},
    {"XA Audio",            0, 1},  /* Restart - SPU init */
    {"CD-DA",               0, 1},  /* Restart - CD init */
    {" FastMix",            0, 0},  /* v110: CDDA fast mix (instant) */
    {" UnityVol",           0, 0},  /* v110: CDDA unity vol (instant) */
    {" ASM Mix",            0, 0},  /* v111: MIPS32 ASM mixer (instant) */
    {" Comp. CDDA",         0, 0},  /* v257: Use .binadpcm/.binwav if available (instant) */
    {" CDDA Tools",         0, 0},  /* v258: Conversion/delete (opens submenu) */
    {"--- GPU ---",         1, 0},
    {"Dithering",           0, 1},  /* Restart - GPU template */
    {"Blending",            0, 1},  /* Restart - GPU template */
    {"Lighting",            0, 1},  /* Restart - GPU template */
    {"Fast Light",          0, 1},  /* Restart - GPU template */
    {"Fast Blend",          0, 1},  /* Restart - GPU template (v090) */
    {"ASM Light",           0, 1},  /* Restart - GPU ASM (v091) */
    {"ASM Blend",           0, 1},  /* Restart - GPU ASM (v091) */
    {"Tex Prefetch",        0, 1},  /* Restart - GPU ASM (v091) */
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
    {"--- ASM BLOCKS ---",  1, 0},  /* v095: Hand-written ASM */
    {"ASM#1:NCLIP",         0, 0},  /* v095: ASM Block 1 (instant) */
    {"ASM#2:AVSZ3",         0, 0},  /* v095: ASM Block 2 (instant) */
    {"--- MEM OPT ---",     1, 0},  /* v098: Memory optimizations */
    {"!SkipCodeInv!",       0, 0},  /* v098: Skip code invalidation (RISK!) */
    {"ASM MemWrite",        0, 0},  /* v098: ASM memory write */
    {"DirectBlockLUT",      0, 1},  /* v101: Direct Block LUT (needs restart) */
    {"--- SPEED HACKS ---", 1, 0},  /* v112: Speed hacks */
    {"!CycleBatch!",        0, 0},  /* v112: Cycle batching (RISK! instant apply) */
    {"BlockCache",          0, 0},  /* v112: Block result cache (instant) */
    {"NativeMode",          0, 0},  /* v114: Native MIPS execution (instant) */
    {"--- CONFIG ---",      1, 0},
    {"SAVE GAME CFG",       2, 0},
    {"SAVE SLUS CFG",       2, 0},
    {"SAVE GLOBAL CFG",     2, 0},
    {"DEL GAME CFG",        2, 0},
    {"DEL GLOBAL CFG",      2, 0},
    {"--------------",      1, 0},
    {"EXIT MENU",           2, 0},
    {"[*]=instant [R]=restart", 1, 0}
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

static void draw_native_cmd_menu(uint16_t *buffer) {
    /* Clear to dark background */
    for (int i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT; i++) {
        buffer[i] = NCMD_COLOR_BG;
    }

    /* Title */
    DrawText(buffer, 50, 4, NCMD_COLOR_TITLE, "NATIVE COMMAND CONFIG v250");
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

/* ============== FPS DISPLAY (just the number) ============== */
static void draw_fps_overlay(uint16_t *pixels)
{
    if (!fps_show || !pixels || menu_active || native_cmd_menu_active) return;

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
        /* Normal frame - draw overlays (skip when any menu is active) */
        if (fps_show && !menu_active && !native_cmd_menu_active) {
            draw_fps_overlay((uint16_t*)data);
        }
        if (qpsx_config.profiler && !menu_active && !native_cmd_menu_active) {
            draw_profiler_overlay((uint16_t*)data);
        }
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

    /* v115: Reset native mode stats for next frame (fixes 0% bug from v114) */
    if (g_opt_native_mode >= 1) {
        native_stats_reset_frame();
    }

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
    fprintf(f, "gpu_fast_blending = %d\n", qpsx_config.gpu_fast_blending);
    fprintf(f, "gpu_asm_lighting = %d\n", qpsx_config.gpu_asm_lighting);
    fprintf(f, "gpu_asm_blending = %d\n", qpsx_config.gpu_asm_blending);
    fprintf(f, "gpu_prefetch_tex = %d\n", qpsx_config.gpu_prefetch_tex);
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
    fprintf(f, "asm_block1 = %d\n", qpsx_config.asm_block1);
    fprintf(f, "asm_block2 = %d\n", qpsx_config.asm_block2);
    fprintf(f, "skip_code_inv = %d\n", qpsx_config.skip_code_inv);
    fprintf(f, "asm_memwrite = %d\n", qpsx_config.asm_memwrite);
    fprintf(f, "debug_log = %d\n", qpsx_config.debug_log);
    fprintf(f, "profiler = %d\n", qpsx_config.profiler);
    fprintf(f, "direct_block_lut = %d\n", qpsx_config.direct_block_lut);
    fprintf(f, "cycle_batch = %d\n", qpsx_config.cycle_batch);
    fprintf(f, "block_cache = %d\n", qpsx_config.block_cache);
    fprintf(f, "cdda_fast_mix = %d\n", qpsx_config.cdda_fast_mix);
    fprintf(f, "cdda_unity_vol = %d\n", qpsx_config.cdda_unity_vol);
    fprintf(f, "cdda_asm_mix = %d\n", qpsx_config.cdda_asm_mix);
    fprintf(f, "cdda_binsav = %d\n", qpsx_config.cdda_binsav);  /* v257: Save compressed CDDA option */
    fprintf(f, "native_mode = %d\n", qpsx_config.native_mode);  /* v116: Save native mode */

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
            else if (strcmp(k, "gpu_fast_blending") == 0) qpsx_config.gpu_fast_blending = value;
            else if (strcmp(k, "gpu_asm_lighting") == 0) qpsx_config.gpu_asm_lighting = value;
            else if (strcmp(k, "gpu_asm_blending") == 0) qpsx_config.gpu_asm_blending = value;
            else if (strcmp(k, "gpu_prefetch_tex") == 0) qpsx_config.gpu_prefetch_tex = value;
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
            else if (strcmp(k, "asm_block1") == 0) qpsx_config.asm_block1 = value;
            else if (strcmp(k, "asm_block2") == 0) qpsx_config.asm_block2 = value;
            else if (strcmp(k, "skip_code_inv") == 0) qpsx_config.skip_code_inv = value;
            else if (strcmp(k, "asm_memwrite") == 0) qpsx_config.asm_memwrite = value;
            else if (strcmp(k, "debug_log") == 0) qpsx_config.debug_log = value;
            else if (strcmp(k, "profiler") == 0) qpsx_config.profiler = value;
            else if (strcmp(k, "direct_block_lut") == 0) qpsx_config.direct_block_lut = value;
            else if (strcmp(k, "cycle_batch") == 0) qpsx_config.cycle_batch = value;
            else if (strcmp(k, "block_cache") == 0) qpsx_config.block_cache = value;
            else if (strcmp(k, "cdda_fast_mix") == 0) qpsx_config.cdda_fast_mix = value;
            else if (strcmp(k, "cdda_unity_vol") == 0) qpsx_config.cdda_unity_vol = value;
            else if (strcmp(k, "cdda_asm_mix") == 0) qpsx_config.cdda_asm_mix = value;
            else if (strcmp(k, "cdda_binsav") == 0) qpsx_config.cdda_binsav = value;  /* v257 */
            else if (strcmp(k, "native_mode") == 0) qpsx_config.native_mode = value;  /* v116: Load native mode */
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

    fps_show = qpsx_config.show_fps;

    Config.SpuUpdateFreq = qpsx_config.spu_update_freq;
    Config.ForcedXAUpdates = qpsx_config.xa_update_freq;
    Config.SpuIrq = qpsx_config.spu_irq_always;

    Config.Xa = qpsx_config.xa_disable;
    Config.Cdda = qpsx_config.cdda_disable;

    /*
     * v254: DO NOT set cycle_multiplier here!
     *
     * qpsx_apply_config() is called every time user changes ANY setting in menu.
     * cycle_multiplier is used by dynarec at block compilation time - changing it
     * while emulation is running causes hangs/crashes because:
     * - Old blocks have old cycle values baked in
     * - New blocks get new values
     * - Inconsistency causes timing problems
     *
     * cycle_multiplier is set ONLY ONCE in retro_load_game() at startup.
     * Menu shows "(Restart)" to indicate restart is required.
     */

    gpu_unai_config_ext.dithering = qpsx_config.gpu_dithering;
    gpu_unai_config_ext.blending = qpsx_config.gpu_blending;
    gpu_unai_config_ext.lighting = qpsx_config.gpu_lighting;
    gpu_unai_config_ext.fast_lighting = qpsx_config.gpu_fast_lighting;
    gpu_unai_config_ext.fast_blending = qpsx_config.gpu_fast_blending;
    gpu_unai_config_ext.asm_lighting = qpsx_config.gpu_asm_lighting;
    gpu_unai_config_ext.asm_blending = qpsx_config.gpu_asm_blending;
    gpu_unai_config_ext.prefetch_tex = qpsx_config.gpu_prefetch_tex;
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

    /* v095: ASM blocks */
    g_opt_asm_block1 = qpsx_config.asm_block1;
    g_opt_asm_block2 = qpsx_config.asm_block2;

    /* v098: Memory optimizations */
    g_opt_skip_code_inv = qpsx_config.skip_code_inv;
    g_opt_asm_memwrite = qpsx_config.asm_memwrite;

    /* v101: Direct Block LUT (needs restart to take effect) */
    g_opt_direct_block_lut = qpsx_config.direct_block_lut;

    /* v112: Speed hacks (instant apply - RISKY!) */
    g_opt_cycle_batch = qpsx_config.cycle_batch;
    g_opt_block_cache = qpsx_config.block_cache;

    /* v114: Native Mode (instant apply) */
    g_opt_native_mode = qpsx_config.native_mode;

    /* v110/v111: CDDA speedup options (instant apply) */
    g_opt_cdda_fast_mix = qpsx_config.cdda_fast_mix;
    g_opt_cdda_unity_vol = qpsx_config.cdda_unity_vol;
    g_opt_cdda_asm_mix = qpsx_config.cdda_asm_mix;
    g_opt_cdda_binsav = qpsx_config.cdda_binsav;  /* v257 */

    /* Update debug log state */
    update_debug_log_state(qpsx_config.debug_log);

    /* v092: Profiler enable/disable */
    profiler_set_enabled(qpsx_config.profiler);
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

    /* v114/v115: Native Mode statistics - show native instruction ratio */
    if (g_opt_native_mode >= 1) {
        int ratio = native_mode_get_ratio();
        if (g_opt_native_mode == 3) {
            /* v217: Use native_used/dynarec_used (blocks compiled) for production stats */
            extern u32 native_used, dynarec_used;  /* rec_native.cpp.h + recompiler.cpp */

            /* Line 1: NAT vs DYN blocks compiled */
            u32 total = native_used + dynarec_used;
            int pct = (total > 0) ? (int)((native_used * 100) / total) : 0;
            snprintf(buf, sizeof(buf), "NAT:%u DYN:%u (%d%%)", native_used, dynarec_used, pct);
            DrawText(pixels, 4, y, pct >= 10 ? color_green : color_yellow, buf);
        } else {
            snprintf(buf, sizeof(buf), "Native MIPS: %d%% (MIPS-I compatible)", ratio);
            DrawText(pixels, 4, y, ratio >= 50 ? color_green : color_yellow, buf);
        }
    }
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
            case MI_PROFILER:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.profiler ? "ON" : "OFF");
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
            case MI_CDDA_FAST_MIX:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.cdda_fast_mix ? "ON" : "OFF");
                break;
            case MI_CDDA_UNITY_VOL:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.cdda_unity_vol ? "ON" : "OFF");
                break;
            case MI_CDDA_ASM_MIX:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.cdda_asm_mix ? "ON" : "OFF");
                break;
            case MI_CDDA_BINSAV:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.cdda_binsav ? "ON" : "OFF");
                break;
            case MI_CDDA_TOOLS:
                snprintf(buf, sizeof(buf), "%s%-12s  [->]", sel, mi->name);
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
            case MI_FAST_BLEND:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.gpu_fast_blending ? "ON" : "OFF");
                break;
            case MI_ASM_LIGHT:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.gpu_asm_lighting ? "ON" : "OFF");
                break;
            case MI_ASM_BLEND:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.gpu_asm_blending ? "ON" : "OFF");
                break;
            case MI_PREFETCH:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.gpu_prefetch_tex ? "ON" : "OFF");
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
            case MI_ASM_BLOCK1:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.asm_block1 ? "ON" : "OFF");
                break;
            case MI_ASM_BLOCK2:
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.asm_block2 ? "ON" : "OFF");
                break;
            case MI_SKIP_CODE_INV:
                /* Show in red to indicate risk */
                snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, qpsx_config.skip_code_inv ? "ON-RISK!" : "OFF");
                break;
            case MI_ASM_MEMWRITE:
                /* v103: 7 levels (0-6) for precise debugging */
                {
                    const char *lvl;
                    switch (qpsx_config.asm_memwrite) {
                        case 1:  lvl = "V98"; break;
                        case 2:  lvl = "+ADDR"; break;
                        case 3:  lvl = "+LUT"; break;
                        case 4:  lvl = "+HWBR"; break;
                        case 5:  lvl = "+HW!"; break;
                        case 6:  lvl = "FULL!"; break;
                        default: lvl = "OFF"; break;
                    }
                    snprintf(buf, sizeof(buf), "%s%-12s: %d:%s", sel, mi->name, qpsx_config.asm_memwrite, lvl);
                }
                break;
            case MI_DIRECT_BLOCK_LUT:
                snprintf(buf, sizeof(buf), "%s%-12s: %s[R]", sel, mi->name, qpsx_config.direct_block_lut ? "ON" : "OFF");
                break;
            case MI_CYCLE_BATCH:
                /* v112: Cycle batching - check events less frequently */
                {
                    const char *lvl;
                    switch (qpsx_config.cycle_batch) {
                        case 1:  lvl = "2x"; break;
                        case 2:  lvl = "4x!"; break;
                        case 3:  lvl = "8x!!"; break;
                        default: lvl = "OFF"; break;
                    }
                    snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, lvl);
                }
                break;
            case MI_BLOCK_CACHE:
                /* v113: Hot Block Cache - L1/L2 friendly dispatch cache */
                {
                    const char *lvl;
                    switch (qpsx_config.block_cache) {
                        case 1:  lvl = "256"; break;    /* 256 entries = 2KB (L1) */
                        case 2:  lvl = "1K"; break;     /* 1K entries = 8KB */
                        case 3:  lvl = "4K"; break;     /* 4K entries = 32KB (L2) */
                        case 4:  lvl = "16K"; break;    /* 16K entries = 128KB */
                        case 5:  lvl = "64K"; break;    /* 64K entries = 512KB */
                        case 6:  lvl = "256K"; break;   /* 256K entries = 2MB */
                        case 7:  lvl = "1M"; break;     /* 1M entries = 8MB */
                        default: lvl = "OFF"; break;
                    }
                    snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, lvl);
                }
                break;
            case MI_NATIVE_MODE:
                /* v241: Native Mode options with descriptions */
                {
                    const char *mode;
                    switch (qpsx_config.native_mode) {
                        case 0:  mode = "0:OFF"; break;      /* Pure dynarec, no native */
                        case 1:  mode = "1:STATS"; break;    /* Show native instruction statistics */
                        case 2:  mode = "2:FAST"; break;     /* Skip const prop for pure ALU */
                        case 3:  mode = "3:NATIVE!"; break;  /* Run native blocks (WIP-hangs) */
                        case 4:  mode = "4:DIFF"; break;     /* Dynarec + native compare (~30fps) */
                        default: mode = "0:OFF"; break;
                    }
                    snprintf(buf, sizeof(buf), "%s%-12s: %s", sel, mi->name, mode);
                }
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
                case MI_CDDA_FAST_MIX: qpsx_config.cdda_fast_mix = !qpsx_config.cdda_fast_mix; break;
                case MI_CDDA_UNITY_VOL: qpsx_config.cdda_unity_vol = !qpsx_config.cdda_unity_vol; break;
                case MI_CDDA_ASM_MIX: qpsx_config.cdda_asm_mix = !qpsx_config.cdda_asm_mix; break;
                case MI_CDDA_BINSAV: qpsx_config.cdda_binsav = !qpsx_config.cdda_binsav; break;
                case MI_CDDA_TOOLS:
                    /* Open CDDA Tools submenu */
                    cdda_scan_tracks();
                    cdda_set_progress_callback(cdda_progress_update);
                    cdda_tools_active = 1;
                    cdda_tools_item = 0;
                    cdda_tools_scroll = 0;
                    cdda_tools_action = 0;  /* Start on Convert */
                    break;
                case MI_DITHERING: qpsx_config.gpu_dithering = !qpsx_config.gpu_dithering; break;
                case MI_BLENDING: if (qpsx_config.gpu_blending > 0) qpsx_config.gpu_blending--; break;
                case MI_LIGHTING: qpsx_config.gpu_lighting = !qpsx_config.gpu_lighting; break;
                case MI_FAST_LIGHT: qpsx_config.gpu_fast_lighting = !qpsx_config.gpu_fast_lighting; break;
                case MI_FAST_BLEND: qpsx_config.gpu_fast_blending = !qpsx_config.gpu_fast_blending; break;
                case MI_ASM_LIGHT: qpsx_config.gpu_asm_lighting = !qpsx_config.gpu_asm_lighting; break;
                case MI_ASM_BLEND: qpsx_config.gpu_asm_blending = !qpsx_config.gpu_asm_blending; break;
                case MI_PREFETCH: qpsx_config.gpu_prefetch_tex = !qpsx_config.gpu_prefetch_tex; break;
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
                case MI_ASM_BLOCK1: qpsx_config.asm_block1 = !qpsx_config.asm_block1; break;
                case MI_ASM_BLOCK2: qpsx_config.asm_block2 = !qpsx_config.asm_block2; break;
                case MI_SKIP_CODE_INV: qpsx_config.skip_code_inv = !qpsx_config.skip_code_inv; break;
                case MI_ASM_MEMWRITE: /* v103: cycle 0←1←2←3←4←5←6 */ if (qpsx_config.asm_memwrite > 0) qpsx_config.asm_memwrite--; else qpsx_config.asm_memwrite = 6; break;
                case MI_DIRECT_BLOCK_LUT: qpsx_config.direct_block_lut = !qpsx_config.direct_block_lut; break;
                case MI_PROFILER: qpsx_config.profiler = !qpsx_config.profiler; break;
                case MI_CYCLE_BATCH: if (qpsx_config.cycle_batch > 0) qpsx_config.cycle_batch--; else qpsx_config.cycle_batch = 3; break;
                case MI_BLOCK_CACHE: if (qpsx_config.block_cache > 0) qpsx_config.block_cache--; else qpsx_config.block_cache = 7; break;
                case MI_NATIVE_MODE: if (qpsx_config.native_mode > 0) qpsx_config.native_mode--; else qpsx_config.native_mode = 4; break;  /* v236: 0-4 */
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
                case MI_CDDA_FAST_MIX: qpsx_config.cdda_fast_mix = !qpsx_config.cdda_fast_mix; break;
                case MI_CDDA_UNITY_VOL: qpsx_config.cdda_unity_vol = !qpsx_config.cdda_unity_vol; break;
                case MI_CDDA_ASM_MIX: qpsx_config.cdda_asm_mix = !qpsx_config.cdda_asm_mix; break;
                case MI_CDDA_BINSAV: qpsx_config.cdda_binsav = !qpsx_config.cdda_binsav; break;
                case MI_CDDA_TOOLS:
                    /* Open CDDA Tools submenu */
                    cdda_scan_tracks();
                    cdda_set_progress_callback(cdda_progress_update);
                    cdda_tools_active = 1;
                    cdda_tools_item = 0;
                    cdda_tools_scroll = 0;
                    cdda_tools_action = 0;  /* Start on Convert */
                    break;
                case MI_DITHERING: qpsx_config.gpu_dithering = !qpsx_config.gpu_dithering; break;
                case MI_BLENDING: if (qpsx_config.gpu_blending < 3) qpsx_config.gpu_blending++; break;
                case MI_LIGHTING: qpsx_config.gpu_lighting = !qpsx_config.gpu_lighting; break;
                case MI_FAST_LIGHT: qpsx_config.gpu_fast_lighting = !qpsx_config.gpu_fast_lighting; break;
                case MI_FAST_BLEND: qpsx_config.gpu_fast_blending = !qpsx_config.gpu_fast_blending; break;
                case MI_ASM_LIGHT: qpsx_config.gpu_asm_lighting = !qpsx_config.gpu_asm_lighting; break;
                case MI_ASM_BLEND: qpsx_config.gpu_asm_blending = !qpsx_config.gpu_asm_blending; break;
                case MI_PREFETCH: qpsx_config.gpu_prefetch_tex = !qpsx_config.gpu_prefetch_tex; break;
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
                case MI_ASM_BLOCK1: qpsx_config.asm_block1 = !qpsx_config.asm_block1; break;
                case MI_ASM_BLOCK2: qpsx_config.asm_block2 = !qpsx_config.asm_block2; break;
                case MI_SKIP_CODE_INV: qpsx_config.skip_code_inv = !qpsx_config.skip_code_inv; break;
                case MI_ASM_MEMWRITE: /* v103: cycle 0→1→2→3→4→5→6 */ qpsx_config.asm_memwrite = (qpsx_config.asm_memwrite + 1) % 7; break;
                case MI_DIRECT_BLOCK_LUT: qpsx_config.direct_block_lut = !qpsx_config.direct_block_lut; break;
                case MI_PROFILER: qpsx_config.profiler = !qpsx_config.profiler; break;
                case MI_CYCLE_BATCH: qpsx_config.cycle_batch = (qpsx_config.cycle_batch + 1) % 4; break;
                case MI_BLOCK_CACHE: qpsx_config.block_cache = (qpsx_config.block_cache + 1) % 8; break;
                case MI_NATIVE_MODE: qpsx_config.native_mode = (qpsx_config.native_mode + 1) % 5; break;  /* v236: 0-4 */
            }
            qpsx_apply_config();
            repeat_delay = 8;
        }
    }

    /* v258: CDDA Tools opens with A button */
    if (cur_a && !prev_a && menu_item == MI_CDDA_TOOLS) {
        cdda_scan_tracks();
        cdda_set_progress_callback(cdda_progress_update);
        cdda_tools_active = 1;
        cdda_tools_item = 0;
        cdda_tools_scroll = 0;
        cdda_tools_action = 0;
        prev_a = cur_a;
        return;
    }

    if (cur_a && !prev_a && (menu_items[menu_item].type == 2 || menu_items[menu_item].type == 3)) {
        char path[256];
        switch (menu_item) {
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
    char cfg_path[256];

    /* 1. Try filename-based config (primary, like v248) */
    get_game_config_path(cfg_path, sizeof(cfg_path));
    if (load_config_file(cfg_path)) {
        XLOG("Loaded game config: %s", cfg_path);
        return;
    }

    /* 2. Try SLUS/SCES/SLES config (secondary) */
    get_slus_config_path(cfg_path, sizeof(cfg_path));
    if (cfg_path[0] != '\0' && load_config_file(cfg_path)) {
        XLOG("Loaded SLUS config: %s", cfg_path);
        return;
    }

    /* 3. Try global config */
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

    /* v092: Initialize profiler */
    profiler_init();

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
    /* v141: Removed verbose retro_run logging */

    if (run_frame_count == 0) {
        XLOG("retro_run() frame 0");

        /* v244: Initialize native command menu */
        native_cmd_menu_init();
        native_cmd_menu_load_config();

        /* v244: Show native command menu at startup ONLY for mode 3 (NATIVE) */
        extern int g_opt_native_mode;
        if (g_opt_native_mode == 3 && native_cmd_menu_enable && !native_cmd_menu_shown) {
            native_cmd_menu_active = 1;
            native_cmd_menu_shown = 1;
            XLOG("native_cmd: showing menu at startup (native_mode=%d)", g_opt_native_mode);
        }

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

    /* v244: Native command menu mode - pause emulation and show menu */
    if (native_cmd_menu_active) {
        handle_native_cmd_menu_input();
        if (SCREEN) {
            draw_native_cmd_menu(SCREEN);
        }
        video_cb(SCREEN, SCREEN_WIDTH, SCREEN_HEIGHT, VIRTUAL_WIDTH * sizeof(uint16_t));
        return;  /* Don't run emulation while in menu */
    }

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
    if (qpsx_config.frame_duping) {
        skip_video_output = (run_frame_count & 1);  /* Skip odd frames */
    } else {
        skip_video_output = 0;
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

#ifdef PSXREC
    /* v254: Set cycle_multiplier ONLY HERE at startup, not in qpsx_apply_config()
     * This value is baked into dynarec blocks at compile time and cannot be
     * changed safely while emulation is running. */
    cycle_multiplier = qpsx_config.cycle_mult;
    XLOG("cycle_multiplier set to %u (startup only)", cycle_multiplier);
#endif

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
