/***************************************************************************
 *   QPSX v094 - Performance Profiler Implementation (DETAILED CPU)
 ***************************************************************************/

#include "profiler.h"
#include <string.h>

/* Global profiler instance */
ProfilerData g_profiler;

/* Start timestamps for each category */
uint32_t g_prof_start[PROF_COUNT];

/* Category names for display - MUST match enum ProfileCategory order! */
static const char* profiler_names[PROF_COUNT] = {
    /* CPU - v094 detailed breakdown */
    "CPU Total",
    "  Compile",
    "  MemRead",
    "  MemWrite",
    "  HW Read",
    "  HW Write",
    "  Exception",
    "  BIOS",
    "  ICache",

    /* GTE */
    "GTE Total",
    "  RTPT",
    "  RTPS",
    "  MVMVA",
    "  NCLIP",
    "  NCDS",
    "  Other",

    /* GPU */
    "GPU Total",
    "  Poly",
    "  Sprite",
    "  Line",
    "  Tile",
    "  VRAM",

    /* SPU */
    "SPU Total",
    "  Mix",
    "  ADPCM",
    "  Reverb",

    /* CD-ROM */
    "CD Total",
    "  CDDA",

    /* Video output (v107) */
    "Video",

    /* Frame */
    "Frame"
};

/* Parent category mapping (-1 = no parent, is a total) */
static const int profiler_parents[PROF_COUNT] = {
    /* CPU - v094 detailed breakdown */
    -1,                 /* CPU Total - no parent */
    PROF_CPU_TOTAL,     /* Compile -> CPU */
    PROF_CPU_TOTAL,     /* MemRead -> CPU */
    PROF_CPU_TOTAL,     /* MemWrite -> CPU */
    PROF_CPU_TOTAL,     /* HW Read -> CPU */
    PROF_CPU_TOTAL,     /* HW Write -> CPU */
    PROF_CPU_TOTAL,     /* Exception -> CPU */
    PROF_CPU_TOTAL,     /* BIOS -> CPU */
    PROF_CPU_TOTAL,     /* ICache -> CPU */

    /* GTE */
    -1,                 /* GTE Total - no parent */
    PROF_GTE_TOTAL,     /* RTPT -> GTE */
    PROF_GTE_TOTAL,     /* RTPS -> GTE */
    PROF_GTE_TOTAL,     /* MVMVA -> GTE */
    PROF_GTE_TOTAL,     /* NCLIP -> GTE */
    PROF_GTE_TOTAL,     /* NCDS -> GTE */
    PROF_GTE_TOTAL,     /* Other -> GTE */

    /* GPU */
    -1,                 /* GPU Total - no parent */
    PROF_GPU_TOTAL,     /* Poly -> GPU */
    PROF_GPU_TOTAL,     /* Sprite -> GPU */
    PROF_GPU_TOTAL,     /* Line -> GPU */
    PROF_GPU_TOTAL,     /* Tile -> GPU */
    PROF_GPU_TOTAL,     /* VRAM -> GPU */

    /* SPU */
    -1,                 /* SPU Total - no parent */
    PROF_SPU_TOTAL,     /* Mix -> SPU */
    PROF_SPU_TOTAL,     /* ADPCM -> SPU */
    PROF_SPU_TOTAL,     /* Reverb -> SPU */

    /* CD-ROM */
    -1,                 /* CD Total - no parent */
    PROF_CDROM_TOTAL,   /* CDDA -> CD */

    /* Video output (v107) */
    -1,                 /* Video - no parent, measures firmware time */

    /* Frame */
    -1                  /* Frame - no parent */
};

/* Averaging weight (exponential moving average) */
#define AVG_WEIGHT 0.1f

void profiler_init(void)
{
    memset(&g_profiler, 0, sizeof(g_profiler));
    memset(g_prof_start, 0, sizeof(g_prof_start));
    g_profiler.enabled = 0;
    g_profiler.detailed = 1;
}

void profiler_frame_start(void)
{
    if (!g_profiler.enabled) return;

    /* Reset cycle counts for new frame */
    memset(g_profiler.cycles, 0, sizeof(g_profiler.cycles));

    /* Start frame timer */
    g_prof_start[PROF_FRAME_TOTAL] = profiler_read_cycles();
}

void profiler_frame_end(void)
{
    if (!g_profiler.enabled) return;

    /* End frame timer */
    uint32_t frame_end = profiler_read_cycles();
    g_profiler.cycles[PROF_FRAME_TOTAL] = frame_end - g_prof_start[PROF_FRAME_TOTAL];

    /* Calculate GTE total from sub-categories */
    g_profiler.cycles[PROF_GTE_TOTAL] =
        g_profiler.cycles[PROF_GTE_RTPT] +
        g_profiler.cycles[PROF_GTE_RTPS] +
        g_profiler.cycles[PROF_GTE_MVMVA] +
        g_profiler.cycles[PROF_GTE_NCLIP] +
        g_profiler.cycles[PROF_GTE_NCDS] +
        g_profiler.cycles[PROF_GTE_OTHER];

    /* Convert cycles to milliseconds */
    for (int i = 0; i < PROF_COUNT; i++) {
        g_profiler.ms[i] = (float)g_profiler.cycles[i] / (float)PROFILER_CYCLES_PER_MS;

        /* Exponential moving average for smoothing */
        if (g_profiler.frame_count == 0) {
            g_profiler.avg_ms[i] = g_profiler.ms[i];
        } else {
            g_profiler.avg_ms[i] = g_profiler.avg_ms[i] * (1.0f - AVG_WEIGHT) +
                                   g_profiler.ms[i] * AVG_WEIGHT;
        }
    }

    /* Calculate percentages of frame time */
    float frame_ms = g_profiler.avg_ms[PROF_FRAME_TOTAL];
    if (frame_ms > 0.001f) {
        for (int i = 0; i < PROF_COUNT; i++) {
            g_profiler.pct[i] = (g_profiler.avg_ms[i] / frame_ms) * 100.0f;
        }
    }

    g_profiler.frame_count++;
}

void profiler_set_enabled(int enabled)
{
    g_profiler.enabled = enabled;
    if (enabled && g_profiler.frame_count == 0) {
        profiler_init();
        g_profiler.enabled = 1;
    }
}

const ProfilerData* profiler_get_data(void)
{
    return &g_profiler;
}

const char* profiler_get_name(int category)
{
    if (category >= 0 && category < PROF_COUNT) {
        return profiler_names[category];
    }
    return "Unknown";
}

int profiler_is_total(int category)
{
    if (category >= 0 && category < PROF_COUNT) {
        return (profiler_parents[category] == -1);
    }
    return 0;
}

int profiler_get_parent(int category)
{
    if (category >= 0 && category < PROF_COUNT) {
        return profiler_parents[category];
    }
    return -1;
}
