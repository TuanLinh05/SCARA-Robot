param([string]$Gcc = '', [string]$Python = '', [switch]$Gui, [switch]$CoreOnly)
$ErrorActionPreference = 'Stop'
$project = Split-Path -Parent $PSScriptRoot
$workspace = Split-Path -Parent (Split-Path -Parent $project)
. (Join-Path $workspace 'tools/ProjectTools.ps1')
$gccTool = Resolve-ProjectTool -Requested $Gcc -Command gcc -Candidates @('C:/msys64/ucrt64/bin/gcc.exe')
$pythonTool = Resolve-ProjectTool -Requested $Python -Command python -Candidates @('D:/zephyrproject/.venv/Scripts/python.exe')
$hostBuild = Join-Path $project 'build_host'
New-Item -ItemType Directory -Force -Path $hostBuild | Out-Null
$common = @('-std=c11', '-Wall', '-Wextra', '-Werror', '-I', "$project/src")
$core = @('cart_core', 'cart_commands', 'jog_core', 'home_worker', 'command_text', 'command_stream', 'cart_telemetry') | ForEach-Object { "$project/src/$_.c" }

foreach ($test in @('test_cart_core', 'test_protocol')) {
    $executable = Join-Path $hostBuild "$test.exe"
    Invoke-ProjectCommand -Executable $gccTool -Arguments ($common + @("$project/tests/$test.c") + $core + @('-lm', '-o', $executable)) -Description "$test compilation"
    Invoke-ProjectCommand -Executable $executable -Arguments @() -Description $test
}
$motorTest = Join-Path $hostBuild 'test_motor_io.exe'
Invoke-ProjectCommand -Executable $gccTool -Arguments ($common + @('-I', "$project/tests/fakes", "$project/tests/host_firmware.c") + $core + @('-o', $motorTest)) -Description 'ISR harness compilation'
Invoke-ProjectCommand -Executable $motorTest -Arguments @() -Description 'ISR harness'

Invoke-WithProjectPython -Workspace $workspace -Action {
    Invoke-ProjectCommand -Executable $pythonTool -Arguments @('-m', 'unittest', 'discover', '-s', "$project/tests", '-p', 'test_*.py', '-v') -Description 'Firmware artifact tests'
    if (-not $CoreOnly) {
        Invoke-ProjectCommand -Executable $pythonTool -Arguments @('-m', 'unittest', 'discover', '-s', "$workspace/Software/tests", '-p', 'test_cartesian_nc.py', '-v') -Description 'Model/protocol tests'
        if ($Gui) {
            Invoke-ProjectCommand -Executable $pythonTool -Arguments @("$workspace/Software/tests/gui_cartesian_nc_smoke.py") -Description 'GUI smoke test'
        }
    }
}

