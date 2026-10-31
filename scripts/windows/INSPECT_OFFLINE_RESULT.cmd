@echo off
if "%~1"=="" (
    echo Drag result.mp4 onto this command file, or pass its full quoted path.
    pause
    exit /b 1
)
call "%~dp0RUN_PC.cmd" pc-tool inspect-video "%~1"
pause
