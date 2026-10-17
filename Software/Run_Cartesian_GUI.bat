@echo off
setlocal
cd /d "%~dp0"

if exist "src\cartesian_nc_control.py" (
  if exist ".venv-build\Scripts\pythonw.exe" (
    start "" "%CD%\.venv-build\Scripts\pythonw.exe" "src\cartesian_nc_control.py"
    exit /b 0
  )
  where pyw >nul 2>nul
  if %errorlevel%==0 (
    start "" pyw -3 "src\cartesian_nc_control.py"
    exit /b 0
  )
  where pythonw >nul 2>nul
  if %errorlevel%==0 (
    start "" pythonw "src\cartesian_nc_control.py"
    exit /b 0
  )
  python "src\cartesian_nc_control.py"
  exit /b %errorlevel%
)

if exist "dist\SCARA_Cartesian_NC_v5_R14\SCARA_Cartesian_NC_v5_R14.exe" (
  start "" "%CD%\dist\SCARA_Cartesian_NC_v5_R14\SCARA_Cartesian_NC_v5_R14.exe"
  exit /b 0
)

echo [ERROR] Could not find src\cartesian_nc_control.py or packaged executable.
exit /b 1
