@echo off
cd /d "%~dp0"
if exist "dist\SCARA_Cartesian_NC_v5_R14\SCARA_Cartesian_NC_v5_R14.exe" (
    start "" "dist\SCARA_Cartesian_NC_v5_R14\SCARA_Cartesian_NC_v5_R14.exe" %*
    exit /b
)
if exist "D:\zephyrproject\.venv\Scripts\python.exe" (
    "D:\zephyrproject\.venv\Scripts\python.exe" src\cartesian_nc_control.py %*
) else (
    py -3 src\cartesian_nc_control.py %*
)
if errorlevel 1 pause
