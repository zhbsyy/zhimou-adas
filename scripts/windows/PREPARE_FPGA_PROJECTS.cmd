@echo off
call "%~dp0RUN_PC.cmd" prepare hardware_decision
if errorlevel 1 exit /b 1
call "%~dp0RUN_PC.cmd" prepare hardware_four_camera
if errorlevel 1 exit /b 1
call "%~dp0RUN_PC.cmd" prepare hardware_rear_camera
echo Open the printed project directories in PDS. No bitstream was programmed.
pause
