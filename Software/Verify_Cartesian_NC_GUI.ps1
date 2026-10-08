param([string]$Python='D:/zephyrproject/.venv/Scripts/python.exe',[switch]$Gui)
$ErrorActionPreference='Stop'
$env:PYTHONPATH="$(Join-Path $PSScriptRoot 'src');$(Join-Path $PSScriptRoot '.packaging')"
$env:PYTHONUTF8='1'
$env:PYTHONDONTWRITEBYTECODE='1'
$firmware=Join-Path (Split-Path -Parent $PSScriptRoot) 'Firmware/ScaraCartesian'
if (-not (Test-Path -LiteralPath "$firmware/build_host/test_motor_io.exe")) {
    & "$firmware/tools/verify.ps1" -Python $Python
    if ($LASTEXITCODE) { throw 'Firmware host tests failed' }
}
foreach ($suite in @('test_cartesian_nc.py','test_cartesian_workspace.py','test_cartesian_drawing.py','test_cartesian_path.py','test_cartesian_shapes.py','test_cartesian_audit.py')) {
    & $Python -m unittest discover -s "$PSScriptRoot/tests" -p $suite -v
    if ($LASTEXITCODE) { throw "Test suite failed: $suite" }
}
if ($Gui) {
    foreach ($script in @('gui_cartesian_nc_smoke.py','gui_cartesian_workspace_smoke.py','gui_cartesian_background_smoke.py','gui_cartesian_drawing_smoke.py','gui_cartesian_shapes_smoke.py')) {
        & $Python "$PSScriptRoot/tests/$script"
        if ($LASTEXITCODE) { throw "GUI test failed: $script" }
    }
}
