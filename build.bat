@echo off
setlocal enabledelayedexpansion

REM Get current directory name for dynamic core naming
for %%I in (.) do set FOLDER_NAME=%%~nxI

echo ========================================
echo Building %FOLDER_NAME% - PSX Emulator for SF2000
echo ========================================

set MULTICORE_DIR=C:\Temp\sf2000_multicore_official
set QPSX_DIR=%~dp0
set CORE_NAME=%FOLDER_NAME%

REM Convert Windows path to WSL path (e.g., C:\Temp_psx\QPSX_008 -> /mnt/c/Temp_psx/QPSX_008)
set DRIVE_LETTER=%QPSX_DIR:~0,1%
REM Convert to lowercase
for %%a in (a b c d e f g h i j k l m n o p q r s t u v w x y z) do call set DRIVE_LETTER=%%DRIVE_LETTER:%%a=%%a%%
set QPSX_PATH_TAIL=%QPSX_DIR:~2,-1%
set QPSX_PATH_TAIL=%QPSX_PATH_TAIL:\=/%
set WSL_QPSX_PATH=/mnt/%DRIVE_LETTER%%QPSX_PATH_TAIL%

echo WSL Path: %WSL_QPSX_PATH%
echo Core name: %CORE_NAME%

echo.
echo [1/5] Cleaning previous build...
wsl -e bash -c "cd '%WSL_QPSX_PATH%' && rm -f src/*.o src/**/*.o libretro/**/*.o 2>/dev/null; true"

echo.
echo [2/5] Compiling %CORE_NAME% library...
wsl -e bash -c "cd '%WSL_QPSX_PATH%' && export PATH=/tmp/mips32-mti-elf/2019.09-03-2/bin:$PATH && make -f Makefile.libretro clean platform=sf2000 && make -f Makefile.libretro platform=sf2000 -j4"

if %errorlevel% neq 0 (
    echo.
    echo *** COMPILATION FAILED ***
    pause
    exit /b 1
)

if not exist "%QPSX_DIR%pcsx4all_libretro_sf2000.a" (
    echo.
    echo *** ERROR: pcsx4all_libretro_sf2000.a not created! ***
    pause
    exit /b 1
)

echo.
echo [3/5] Setting up multicore framework...
wsl -e bash -c "mkdir -p /mnt/c/Temp/sf2000_multicore_official/cores/%CORE_NAME%"

REM Create the core Makefile
echo TARGET_NAME := pcsx4all> "%MULTICORE_DIR%\cores\%CORE_NAME%\Makefile"
echo.>> "%MULTICORE_DIR%\cores\%CORE_NAME%\Makefile"
echo ifeq ($(platform), sf2000)>> "%MULTICORE_DIR%\cores\%CORE_NAME%\Makefile"
echo 	TARGET := $(TARGET_NAME)_libretro_$(platform).a>> "%MULTICORE_DIR%\cores\%CORE_NAME%\Makefile"
echo 	STATIC_LINKING = 1>> "%MULTICORE_DIR%\cores\%CORE_NAME%\Makefile"
echo endif>> "%MULTICORE_DIR%\cores\%CORE_NAME%\Makefile"
echo.>> "%MULTICORE_DIR%\cores\%CORE_NAME%\Makefile"
echo all:>> "%MULTICORE_DIR%\cores\%CORE_NAME%\Makefile"
echo 	@echo "Using pre-built $(TARGET)">> "%MULTICORE_DIR%\cores\%CORE_NAME%\Makefile"
echo 	@test -f $(TARGET) ^|^| (echo "ERROR: $(TARGET) not found!" ^&^& exit 1)>> "%MULTICORE_DIR%\cores\%CORE_NAME%\Makefile"
echo.>> "%MULTICORE_DIR%\cores\%CORE_NAME%\Makefile"
echo clean:>> "%MULTICORE_DIR%\cores\%CORE_NAME%\Makefile"
echo 	@echo "Nothing to clean">> "%MULTICORE_DIR%\cores\%CORE_NAME%\Makefile"
echo.>> "%MULTICORE_DIR%\cores\%CORE_NAME%\Makefile"
echo .PHONY: all clean>> "%MULTICORE_DIR%\cores\%CORE_NAME%\Makefile"

REM Copy the .a file
copy /Y "%QPSX_DIR%pcsx4all_libretro_sf2000.a" "%MULTICORE_DIR%\cores\%CORE_NAME%\"

echo.
echo [4/5] Linking core_87000000...
wsl -e bash -c "cd /mnt/c/Temp/sf2000_multicore_official && rm -f core_87000000 core.elf libretro_core.a 2>/dev/null; export PATH=/tmp/mips32-mti-elf/2019.09-03-2/bin:$PATH && make CORE=cores/%CORE_NAME% CONSOLE=psx core_87000000"

if %errorlevel% neq 0 (
    echo.
    echo *** LINKING FAILED ***
    pause
    exit /b 1
)

if not exist "%MULTICORE_DIR%\core_87000000" (
    echo.
    echo *** ERROR: core_87000000 not created! ***
    pause
    exit /b 1
)

echo.
echo [5/5] Copying result...
copy /Y "%MULTICORE_DIR%\core_87000000" "%QPSX_DIR%core_87000000"

echo.
echo [6/6] Verifying build is different from previous version...
REM Extract version number and calculate previous version
set VERSION_NUM=%FOLDER_NAME:QPSX_=%
REM Strip leading zeros to avoid octal interpretation (012 -> 12)
set NUM_CLEAN=%VERSION_NUM%
:strip_zeros
if "!NUM_CLEAN:~0,1!"=="0" if not "!NUM_CLEAN!"=="0" (
    set "NUM_CLEAN=!NUM_CLEAN:~1!"
    goto :strip_zeros
)
set /a PREV_NUM=!NUM_CLEAN! - 1
set PREV_NUM_PAD=00!PREV_NUM!
set PREV_NUM_PAD=!PREV_NUM_PAD:~-3!
set PREV_FOLDER=QPSX_%PREV_NUM_PAD%
set PREV_CORE=%QPSX_DIR%..\%PREV_FOLDER%\core_87000000

if exist "%PREV_CORE%" (
    fc /b "%QPSX_DIR%core_87000000" "%PREV_CORE%" >nul 2>&1
    if !errorlevel! equ 0 (
        echo.
        echo ****************************************
        echo *** ALERT: BINARY IDENTICAL TO %PREV_FOLDER% ***
        echo *** CLAUDE FORGOT TO COMPILE AGAIN!!! ***
        echo ****************************************
        echo.
        echo Pliki sa IDENTYCZNE - prawdopodobnie zmiany nie zostaly skompilowane!
        echo.
        pause
        exit /b 1
    ) else (
        echo OK: Binary differs from %PREV_FOLDER%
    )
) else (
    echo Previous version %PREV_FOLDER% not found, skipping comparison
)

echo.
echo ========================================
echo BUILD SUCCESSFUL!
echo ========================================
echo.
echo Output: %QPSX_DIR%core_87000000
for %%A in ("%QPSX_DIR%core_87000000") do echo Size: %%~zA bytes
echo.
echo To deploy: copy core_87000000 to SD:\cores\psx\
echo Check log.txt on SD card for debug output!
echo.
pause
