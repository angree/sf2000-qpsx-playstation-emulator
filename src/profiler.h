/***************************************************************************
 *   QPSX v096 - Performance Profiler (DETAILED CPU breakdown)
 *
 *   Low-overhead profiling system using MIPS CP0 Count register.
 *   Provides breakdown of CPU/GTE/GPU/SPU timing per frame.
 *
 *   v096: Fixed display to show meaningful breakdown:
 *   - Comp: Block compilation time (usually ~0 when blocks cached)
 *   - Exec: Pure execution = CPU_TOTAL - Comp - Mem (THE BOTTLENECK)
 *   - Mem: Memory helper calls (rare - most access is inlined)
 *
 *   The "Exec" time is where optimization matters - it's the dispatch
 *   loop + compiled blocks running. Assembly optimization targets this.
 *
 *   Usage:
 *     PROFILE_START(PROF_GTE_RTPT);
 *     // ... code to profile ...
 *     PROFILE_END(PROF_GTE_RTPT);
 *
 *   At end of frame, call profiler_frame_end() to finalize.
 *   Call profiler_get_data() to retrieve timing info.
 ***************************************************************************/

#ifndef QPSX_PROFILER_H
#define QPSX_PROFILER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Profile categories - keep in sync with profiler_names[] in profiler.c
 */
enum ProfileCategory {
    /* CPU / Dynarec - DETAILED BREAKDOWN (v094) */
    PROF_CPU_TOTAL = 0,     /* Total CPU emulation time */
    PROF_CPU_COMPILE,       /* Block compilation (recRecompile) */
    PROF_CPU_MEM_READ,      /* Memory reads (psxMemRead8/16/32) */
    PROF_CPU_MEM_WRITE,     /* Memory writes (psxMemWrite8/16/32) */
    PROF_CPU_HW_READ,       /* Hardware I/O reads (psxHwRead) */
    PROF_CPU_HW_WRITE,      /* Hardware I/O writes (psxHwWrite) */
    PROF_CPU_EXCEPTION,     /* Exception handling (psxException) */
    PROF_CPU_BIOS,          /* HLE BIOS calls (psxBios_*) */
    PROF_CPU_ICACHE,        /* Instruction cache flush */

    /* GTE (Geometry Transform Engine) */
    PROF_GTE_TOTAL,         /* Total GTE time (sum of below) */
    PROF_GTE_RTPT,          /* RTPT - transform 3 vertices (HOT!) */
    PROF_GTE_RTPS,          /* RTPS - transform 1 vertex */
    PROF_GTE_MVMVA,         /* MVMVA - matrix-vector multiply */
    PROF_GTE_NCLIP,         /* NCLIP - normal clipping */
    PROF_GTE_NCDS,          /* NCDS - normal color depth single */
    PROF_GTE_OTHER,         /* Other GTE ops */

    /* GPU (Graphics Processing Unit) */
    PROF_GPU_TOTAL,         /* Total GPU time */
    PROF_GPU_POLY,          /* Polygon rendering */
    PROF_GPU_SPRITE,        /* Sprite blitting */
    PROF_GPU_LINE,          /* Line drawing */
    PROF_GPU_TILE,          /* Tile/rectangle fill */
    PROF_GPU_VRAM,          /* VRAM transfers */

    /* SPU (Sound Processing Unit) */
    PROF_SPU_TOTAL,         /* Total SPU time */
    PROF_SPU_MIX,           /* Voice mixing */
    PROF_SPU_ADPCM,         /* ADPCM decoding */
    PROF_SPU_REVERB,        /* Reverb processing */

    /* CD-ROM */
    PROF_CDROM_TOTAL,       /* Total CD-ROM time */
    PROF_CDDA,              /* CDDA audio decoding */

    /* Video output (v107) */
    PROF_VIDEO_OUTPUT,      /* Time spent in real_video_cb (firmware) */

    /* Frame */
    PROF_FRAME_TOTAL,       /* Total frame time */

    PROF_COUNT              /* Number of categories */
};

/*
 * Profiler data structure
 */
typedef struct {
    /* Raw cycle counts (accumulated per frame) */
    uint32_t cycles[PROF_COUNT];

    /* Converted to milliseconds (after frame_end) */
    float ms[PROF_COUNT];

    /* Percentages of frame time */
    float pct[PROF_COUNT];

    /* Frame counter */
    uint32_t frame_count;

    /* Averaging buffer (last N frames) */
    float avg_ms[PROF_COUNT];

    /* Is profiler enabled? */
    int enabled;

    /* Show detailed breakdown? */
    int detailed;
} ProfilerData;

/* Global profiler instance */
extern ProfilerData g_profiler;

/* CPU frequency for cycle->ms conversion (SF2000 = 918 MHz) */
#define PROFILER_CPU_MHZ 918
#define PROFILER_CYCLES_PER_MS (PROFILER_CPU_MHZ * 1000)

/*
 * Read MIPS CP0 Count register (cycle counter)
 * This is the lowest-overhead timing method available.
 * ~2-4 cycles overhead per read.
 */
#if defined(SF2000) || defined(__mips__)
static inline uint32_t profiler_read_cycles(void)
{
    uint32_t count;
    __asm__ volatile(
        ".set push\n"
        ".set noreorder\n"
        "mfc0 %0, $9\n"      /* CP0 Register 9 = Count */
        "nop\n"               /* Hazard barrier */
        ".set pop\n"
        : "=r"(count)
    );
    return count;
}
#else
/* Fallback for non-MIPS: use a simple incrementing counter for testing */
static inline uint32_t profiler_read_cycles(void)
{
    static uint32_t fake_counter = 0;
    return ++fake_counter;
}
#endif

/*
 * Profiler start/end timestamps (per-thread storage for nesting)
 */
extern uint32_t g_prof_start[PROF_COUNT];

/*
 * PROFILE_START - Begin timing a section
 * Low overhead: ~4 cycles on MIPS
 */
#define PROFILE_START(cat) \
    do { \
        if (g_profiler.enabled) { \
            g_prof_start[cat] = profiler_read_cycles(); \
        } \
    } while(0)

/*
 * PROFILE_END - End timing a section
 * Low overhead: ~8 cycles on MIPS
 */
#define PROFILE_END(cat) \
    do { \
        if (g_profiler.enabled) { \
            uint32_t _end = profiler_read_cycles(); \
            g_profiler.cycles[cat] += (_end - g_prof_start[cat]); \
        } \
    } while(0)

/*
 * PROFILE_ADD - Directly add cycles (for cases where start/end don't work)
 */
#define PROFILE_ADD(cat, cyc) \
    do { \
        if (g_profiler.enabled) { \
            g_profiler.cycles[cat] += (cyc); \
        } \
    } while(0)

/*
 * Initialize profiler
 */
void profiler_init(void);

/*
 * Reset profiler data for new frame
 */
void profiler_frame_start(void);

/*
 * Finalize frame - convert cycles to ms, calculate percentages
 */
void profiler_frame_end(void);

/*
 * Enable/disable profiler
 */
void profiler_set_enabled(int enabled);

/*
 * Get profiler data (read-only)
 */
const ProfilerData* profiler_get_data(void);

/*
 * Get category name for display
 */
const char* profiler_get_name(int category);

/*
 * Check if category is a "total" category (for display grouping)
 */
int profiler_is_total(int category);

/*
 * Get parent category for sub-categories (-1 if none)
 */
int profiler_get_parent(int category);

#ifdef __cplusplus
}
#endif

#endif /* QPSX_PROFILER_H */
