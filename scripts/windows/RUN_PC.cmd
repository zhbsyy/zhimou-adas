@echo off
setlocal
set "PY=D:\python3.12.4\python.exe"
if defined FPGA_ADAS_PYTHON set "PY=%FPGA_ADAS_PYTHON%"
if not exist "%PY%" (
    echo Python not found: %PY%
    echo Edit scripts/windows/RUN_PC.cmd or set FPGA_ADAS_PYTHON.
    pause
    exit /b 1
)
"%PY%" -B "%~dp0..\project.py" %*
set "RESULT=%ERRORLEVEL%"
if not "%RESULT%"=="0" pause
exit /b %RESULT%
