param([string]$Python = '')
$ErrorActionPreference = 'Stop'
$software = $PSScriptRoot
. (Join-Path (Split-Path -Parent $software) 'tools/ProjectTools.ps1')
$pythonTool = Resolve-ProjectTool -Requested $Python -Command python -Candidates @('D:/zephyrproject/.venv/Scripts/python.exe')
Invoke-WithProjectPython -Workspace (Split-Path -Parent $software) -Action {
    $hasBuilder = & $pythonTool -c 'import importlib.util; print(importlib.util.find_spec("PyInstaller") is not None)'
    if ($LASTEXITCODE -ne 0) { throw 'Python dependency check failed' }
    if ($hasBuilder -ne 'True') {
        Invoke-ProjectCommand -Executable $pythonTool -Arguments @('-m', 'pip', 'install', '--target', "$software/.packaging", '-r', "$software/requirements-build.txt") -Description 'Build dependency installation'
    }
    Push-Location -LiteralPath $software
    try {
        Invoke-ProjectCommand -Executable $pythonTool -Arguments @('-m', 'PyInstaller', '--noconfirm', 'packaging/SCARA_Cartesian_NC_v5_R14.spec') -Description 'GUI packaging'
    } finally { Pop-Location }
    $output = Join-Path $software 'dist/SCARA_Cartesian_NC_v5_R14'
    Copy-Item -LiteralPath "$software/README.md" -Destination "$output/README.md"
    Get-FileHash -LiteralPath "$output/SCARA_Cartesian_NC_v5_R14.exe"
}
