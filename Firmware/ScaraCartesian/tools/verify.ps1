param(
    [string]$Gcc = 'C:/msys64/ucrt64/bin/gcc.exe',
    [string]$Python = 'D:/zephyrproject/.venv/Scripts/python.exe',
    [switch]$Gui
)
$ErrorActionPreference = 'Stop'
$project = Split-Path -Parent $PSScriptRoot
$workspace = Split-Path -Parent (Split-Path -Parent $project)
$hostBuild = Join-Path $project 'build_host'
New-Item -ItemType Directory -Force -Path $hostBuild | Out-Null
$common = @('-std=c11','-Wall','-Wextra','-Werror','-I',"$project/src")
$core = @("$project/src/cart_core.c","$project/src/jog_core.c","$project/src/home_worker.c")
& $Gcc @common "$project/tests/test_cart_core.c" @core -lm -o "$hostBuild/test_cart_core.exe"
if ($LASTEXITCODE) { throw 'Core compilation failed' }
& "$hostBuild/test_cart_core.exe"
if ($LASTEXITCODE) { throw 'Core tests failed' }
& $Gcc @common -I "$project/tests/fakes" "$project/tests/host_firmware.c" @core -o "$hostBuild/test_motor_io.exe"
if ($LASTEXITCODE) { throw 'ISR compilation failed' }
& "$hostBuild/test_motor_io.exe"
if ($LASTEXITCODE) { throw 'ISR tests failed' }
& $Python -m unittest discover -s "$workspace/Software/tests" -p test_cartesian_nc.py -v
if ($LASTEXITCODE) { throw 'Model/protocol tests failed' }
if ($Gui) {
    $env:PYTHONPATH = "$workspace/Software/.packaging"
    $env:PYTHONUTF8 = '1'
    & $Python "$workspace/Software/tests/gui_cartesian_nc_smoke.py"
    if ($LASTEXITCODE) { throw 'GUI tests failed' }
}

