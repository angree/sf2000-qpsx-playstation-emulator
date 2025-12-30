# QPSX - PlayStation Emulator for SF2000 / GB300

PlayStation 1 (PSX) emulator for Data Frog SF2000 and GB300 handheld devices.

Based on PCSX4ALL with MIPS-to-MIPS dynamic recompiler, ported to the SF2000/GB300 multicore framework.

**Current Status: Early Alpha** - Many games are playable but performance varies significantly.

## Downloads

| Platform | File |
|----------|------|
| SF2000 | `releases/sf2000/core_87000000` |
| GB300 | `releases/gb300/core_87000000` |

## Installation

### SF2000

1. Copy `core_87000000` to SD card: `cores/psx/core_87000000`
2. Copy PlayStation BIOS to: `bios/scph1001.bin` (US) or `scph5502.bin` (EU)
3. Copy games (CUE/BIN format) to: `ROMS/psx/`
4. Create ROM stubs: `psx;GameName.cue.gba` (empty files)

### GB300

Same as SF2000, but using the GB300 binary.

### Multi-BIN Games (CD Audio tracks)

For games with multiple BIN files (like Tekken with 28 tracks):

```
ROMS/psx/
├── GameName.cue                    <- CUE in root (shows in game list)
└── GameName/                       <- Subfolder with same name
    ├── GameName (Track 01).bin     <- Data track
    ├── GameName (Track 02).bin     <- Audio tracks
    └── ...
```

## Controls

**Single player only** (no multiplayer support yet)

| Button | Action |
|--------|--------|
| D-Pad | Movement |
| A | Cross (X) |
| B | Circle (O) |
| X | Square |
| Y | Triangle |
| L | L1 |
| R | R1 |
| SELECT | Select |
| START | Start / **Open Menu** (hold 1 sec) |

## In-Game Menu

Hold START for 1 second to open the configuration menu:

- **Frameskip**: 0-5 (0 = no skip, higher = more skip)
- **BIOS**: HLE (fast) or Real BIOS (more compatible)
- **CPU Cycle**: 256-2048 (higher = more accurate but slower)
- **Show FPS**: Display framerate counter
- And more options...

## Finding Optimal Settings

The emulator is still lacking speed for many games. Finding the best settings for each game requires experimentation:

1. **CPU Cycles**: Start with the default (1536). Lower values = faster but may cause audio glitches. **Find the lowest value where audio doesn't stutter.**

2. **BIOS Mode**:
   - **HLE BIOS** (default): Faster, works for most games
   - **Real BIOS**: More compatible, required for some games

3. **Frameskip**:
   - **0**: Best quality, slowest
   - **1**: Good balance for 3D games
   - **2**: Only useful for 2D games (3D games look choppy)

4. **CD-DA Audio**: Disable CD audio tracks for additional performance boost (menu: Audio -> CD-DA: OFF)

Each game may need different settings. Very demanding games like **Soul Blade** or **Tekken 3** cannot be configured satisfactorily on this hardware.

## Dynarec Optimizations

These options are in the menu under "DYNAREC OPT". **Default settings are recommended** - all are battle-tested.

| Option | Default | Description |
|--------|---------|-------------|
| **LO/HI Cache** | ON | Caches MIPS LO/HI registers in host registers. Speeds up multiplication/division sequences. Safe. |
| **NCLIP Inline** | ON | Inlines GTE NCLIP operation (backface culling). Reduces function call overhead. Safe. |
| **ConstProp+** | ON | Extended constant propagation. Optimizes memory address calculations. Safe. |
| **RTPT Unroll** | OFF | Loop unrolling for perspective transform. Experimental, marginal gain. |
| **UNR Divide** | OFF | Newton-Raphson division instead of hardware DIV. Experimental. |
| **!NoFlags!** | OFF | Skip GTE overflow flag checking. **RISKY - can break games that read GTE flags!** |

For best results, keep the first 3 options ON (default).

## Graphics Options

Graphics options provide **minimal performance gain** but can **significantly degrade visual quality**:

| Option | Recommendation |
|--------|----------------|
| **Dithering** | OFF - Reduces color banding but costs performance |
| **Blending** | ON (1-3) - Required for transparency effects |
| **Lighting** | ON - Needed for proper 3D shading |
| **Fast Light** | OFF - Experimental, may cause artifacts |
| **Pixel Skip** | ON - Slight speedup, minimal visual impact |

## Region Compatibility

**Currently only European (PAL) games are tested and supported.**

US (NTSC) games may work but are not officially tested.

## Known Limitations

- **Performance**: Many 3D games run below full speed
- **Very demanding games** (Soul Blade, Tekken 3, Ridge Racer Type 4) are not playable at acceptable framerates
- **No analog stick support** - digital controls only
- **Single player only** - no multiplayer
- **No memory card GUI** - save states work via menu
- **Audio sync issues** in some games at low cycle values

## Build Instructions

### Prerequisites

- MIPS GCC toolchain: `mips32-mti-elf-2019.09-03`
- SF2000 or GB300 multicore framework

### Building

```bash
# Set toolchain path
export PATH=/tmp/mips32-mti-elf/2019.09-03-2/bin:$PATH

# Build libretro core
make -f Makefile.libretro clean platform=sf2000
make -f Makefile.libretro platform=sf2000 -j4

# Link with multicore framework (SF2000)
cd sf2000_multicore_official
make CORE=cores/psx CONSOLE=psx core_87000000

# For GB300, use gb300_multicore instead
```

## Technical Details

- **CPU**: MIPS-to-MIPS dynamic recompiler (no interpreter fallback)
- **GPU**: Software renderer (GPU UNAI from PCSX4ALL)
- **SPU**: PCSX-ReARMed SPU with libretro audio backend
- **GTE**: Integer fixed-point (no FPU required)
- **CD**: CUE/BIN support with multi-track CD-DA

### Build Differences: SF2000 vs GB300

Both platforms use the **same libretro core** (`.a` library). Only the final linking step differs:

| Component | SF2000 | GB300 |
|-----------|--------|-------|
| Core `.a` file | Identical | Identical |
| Linker addresses | SF2000 firmware | GB300 firmware |
| Final binary | `core_87000000` | `core_87000000` |

## Credits

- **PCSX4ALL**: Original PSX emulator with MIPS dynarec by Senquack, Chui, Franxis
- **PCSX-ReARMed**: SPU and GTE optimizations by Notaz
- **GPU UNAI**: Software GPU renderer
- **SF2000/GB300 multicore framework**: Device-specific runtime

## License

MIT License - see LICENSE file

Based on GPL-licensed PCSX4ALL code. Original PCSX4ALL license in `gpl.txt`.
