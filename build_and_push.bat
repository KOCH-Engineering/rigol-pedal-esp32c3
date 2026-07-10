@echo off
REM Compiles the pedal firmware (catches build errors before pushing
REM anything), then commits and pushes source to GitHub, and optionally
REM publishes a GitHub Release with the compiled firmware.bin attached.
REM Run this from a Windows terminal (e.g. VS Code's integrated terminal)
REM in this project's folder. This does NOT flash the board - that's a
REM separate, deliberate step (see the end of this script, or just use
REM PlatformIO's own Upload button) since it requires a pedal physically
REM plugged in over USB at that exact moment.

REM Figure out how to invoke PlatformIO's CLI on this machine - "pio" may
REM not be on PATH even when the VS Code extension has it installed, since
REM the extension keeps its own bundled copy in a per-user venv.
set PIO=
where pio >nul 2>nul
if %errorlevel%==0 set PIO=pio
if not defined PIO (
    if exist "%USERPROFILE%\.platformio\penv\Scripts\pio.exe" (
        set "PIO=%USERPROFILE%\.platformio\penv\Scripts\pio.exe"
    )
)
if not defined PIO (
    echo Could not find PlatformIO's "pio" command.
    echo Install the PlatformIO CLI ^(or just the VS Code extension, which
    echo includes it^) from https://platformio.org/install, then re-run this.
    goto :error
)
echo Using: %PIO%

echo.
echo Compiling firmware...
"%PIO%" run
if errorlevel 1 goto :error

echo.
echo ============================================================
echo Build complete: .pio\build\esp32-c3-devkitm-1\firmware.bin
echo To flash it to a connected pedal, run:  pio run --target upload
echo (or use PlatformIO's own Upload button) - not done automatically
echo here, since that needs a board plugged in right now.
echo ============================================================

REM --- Commit and push changed source files to GitHub -----------------
REM Never fails the build if git/gh aren't set up yet - just warns.
where git >nul 2>nul
if not %errorlevel%==0 (
    echo [git] git not found on PATH - skipping commit/push.
    goto :done
)
if not exist ".git" (
    echo [git] No git repo here yet. One-time setup:
    echo   gh auth login
    echo   git init
    echo   git add -A
    echo   git commit -m "Initial commit"
    echo   gh repo create KOCH-Engineering/rigol-pedal-esp32c3 --public --source=. --remote=origin --push
    goto :done
)
echo.
echo Committing and pushing changes to GitHub...
git add -A
git diff --cached --quiet
if errorlevel 1 goto :do_commit
echo [git] no changes to commit.
goto :done

:do_commit
git commit -m "Automated build"
git push
if errorlevel 1 echo [git] push failed - check your GitHub auth via gh auth status and your remote.

:done
REM --- Optionally publish a GitHub Release with firmware.bin attached ---
REM Useful for keeping a version history of actual built binaries, same
REM idea as rigol-speaking-multimeter's build_exe.bat. Not required for
REM day-to-day flashing (you build straight from source via pio run
REM --target upload) - this is just an optional archive/backup step.
where gh >nul 2>nul
if not %errorlevel%==0 (
    echo.
    echo [gh] GitHub CLI not found - skipping release upload.
    echo   Install from https://cli.github.com/ to automate this.
    goto :end_release
)

for /f "delims=" %%V in ('powershell -NoProfile -Command "(Select-String -Path src\main.cpp -Pattern '#define FIRMWARE_VERSION \"([^\"]+)\"').Matches[0].Groups[1].Value"') do set FW_VER=%%V
if not defined FW_VER (
    echo [gh] Could not read FIRMWARE_VERSION from src\main.cpp - skipping release.
    goto :end_release
)
set TAG=v%FW_VER%

echo.
set /p DO_RELEASE="Publish GitHub release %TAG% with firmware.bin attached? [y/N] "
if /i not "%DO_RELEASE%"=="y" goto :end_release

gh release create %TAG% .pio\build\esp32-c3-devkitm-1\firmware.bin --title "%TAG%" --notes "Automated build." 2>nul
if errorlevel 1 (
    echo [gh] Release %TAG% may already exist - uploading/replacing its firmware.bin instead...
    gh release upload %TAG% .pio\build\esp32-c3-devkitm-1\firmware.bin --clobber
    if errorlevel 1 echo [gh] Could not create or update the release - check "gh auth status" and FIRMWARE_VERSION.
)

:end_release
pause
goto :eof

:error
echo.
echo Build failed - see the error above.
pause
