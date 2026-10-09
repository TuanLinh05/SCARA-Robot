param([string]$Python = '', [string]$Gcc = '', [switch]$Gui)
$ErrorActionPreference = 'Stop'
$workspace = Split-Path -Parent $PSScriptRoot
. (Join-Path $workspace 'tools/ProjectTools.ps1')
$pythonTool = Resolve-ProjectTool -Requested $Python -Command python -Candidates @('D:/zephyrproject/.venv/Scripts/python.exe')
$firmware = Join-Path $workspace 'Firmware/ScaraCartesian'

# Rebuild the fixture before testing so telemetry/GUI tests use current C sources.
& (Join-Path $firmware 'tools/verify.ps1') -Python $pythonTool -Gcc $Gcc -CoreOnly
Invoke-WithProjectPython -Workspace $workspace -Action {
    Invoke-ProjectCommand -Executable $pythonTool -Arguments @('-m', 'unittest', 'discover', '-s', "$PSScriptRoot/tests", '-p', 'test_*.py', '-v') -Description 'GUI unit tests'
    if ($Gui) {
        foreach ($script in (Get-ChildItem -LiteralPath "$PSScriptRoot/tests" -Filter 'gui_*_smoke.py' | Sort-Object Name)) {
            Invoke-ProjectCommand -Executable $pythonTool -Arguments @($script.FullName) -Description "GUI smoke test $($script.Name)"
        }
    }
}
