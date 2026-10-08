param([string]$Python='D:/zephyrproject/.venv/Scripts/python.exe')
$ErrorActionPreference='Stop'
$software=$PSScriptRoot
$env:PYTHONPATH="$(Join-Path $software 'src');$(Join-Path $software '.packaging')"
$env:PYTHONUTF8='1'
$env:PYTHONDONTWRITEBYTECODE='1'
$hasBuilder = & $Python -c 'import importlib.util; print(importlib.util.find_spec("PyInstaller") is not None)'
if ($hasBuilder -ne 'True') {
    & $Python -m pip install --target "$software/.packaging" -r "$software/requirements-build.txt"
    if ($LASTEXITCODE) { throw 'Build dependencies could not be installed' }
}
Push-Location -LiteralPath $software
try {
    & $Python -m PyInstaller --noconfirm packaging/SCARA_Cartesian_NC_v5_R14.spec
    if ($LASTEXITCODE) { throw 'GUI packaging failed' }
} finally { Pop-Location }
$output=Join-Path $software 'dist/SCARA_Cartesian_NC_v5_R14'
Copy-Item -LiteralPath "$software/README.md" -Destination "$output/README.md"
Get-FileHash -LiteralPath "$output/SCARA_Cartesian_NC_v5_R14.exe"
