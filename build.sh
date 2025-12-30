#!/bin/bash
# PCSX4ALL SF2000 build script
# QPSX_001 - Initial PSX port
#
# Based on UAE4ALL build.sh - follow EXACT same patterns!

set -e

# Toolchain path
export PATH=/tmp/mips32-mti-elf/2019.09-03-2/bin:$PATH

echo "=== PCSX4ALL SF2000 Build ==="
echo "Version: QPSX_001"
echo ""

# Clean previous build
echo "Cleaning..."
make -f Makefile.libretro clean platform=sf2000 2>/dev/null || true

# Build
echo "Building..."
make -f Makefile.libretro platform=sf2000 -j4

echo ""
echo "=== Build complete ==="
echo "Output: pcsx4all_libretro_sf2000.a"
echo ""
echo "To link final binary, copy to sf2000_multicore_official/cores/psx/"
echo "and run: make CORE=cores/psx CONSOLE=psx core_87000000"
