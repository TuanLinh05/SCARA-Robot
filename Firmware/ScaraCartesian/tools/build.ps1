param(
    [string]$ZephyrRoot = 'D:/zephyrproject',
    [string]$SdkRoot = 'D:/zephyr-sdk/zephyr-sdk-0.16.8'
)
$ErrorActionPreference = 'Stop'
$project = Split-Path -Parent $PSScriptRoot
$workspace = Split-Path -Parent (Split-Path -Parent $project)
# Zephyr 3.7 splits paths containing spaces; the ARM tools also need short paths.
$aliasPath = Join-Path $ZephyrRoot 'scara_cartesian_workspace'
if (Test-Path -LiteralPath $aliasPath) {
    $existing = Get-Item -LiteralPath $aliasPath
    if ($existing.LinkType -ne 'Junction' -or [IO.Path]::GetFullPath([string]$existing.Target) -ne [IO.Path]::GetFullPath($workspace)) {
        throw "Existing alias does not point to this workspace: $aliasPath"
    }
} else {
    New-Item -ItemType Junction -Path $aliasPath -Target $workspace | Out-Null
}
$env:ZEPHYR_BASE = Join-Path $ZephyrRoot 'zephyr'
$env:ZEPHYR_SDK_INSTALL_DIR = $SdkRoot
$env:PYTHONDONTWRITEBYTECODE = '1'
$sourcePath = Join-Path $aliasPath 'Firmware/ScaraCartesian'
$buildPath = Join-Path $sourcePath 'build_mcu'
$pythonPath = Join-Path $ZephyrRoot '.venv/Scripts/python.exe'
& cmake -S $sourcePath -B $buildPath -G Ninja '-DBOARD=stm32_min_dev@blue' "-DPython3_EXECUTABLE=$pythonPath" '-DBUILD_VERSION=scara-cart-nc-home-v5-r9'
if ($LASTEXITCODE) { throw 'Zephyr configuration failed' }
& cmake --build $buildPath
if ($LASTEXITCODE) { throw 'Zephyr build failed' }
$hexPath = Join-Path $project 'Hex'
New-Item -ItemType Directory -Force -Path $hexPath | Out-Null
Copy-Item -LiteralPath "$buildPath/zephyr/zephyr.hex" -Destination "$hexPath/SCARA_Cartesian_NC_Home_v5_R9.hex"
Copy-Item -LiteralPath "$buildPath/zephyr/zephyr.bin" -Destination "$hexPath/SCARA_Cartesian_NC_Home_v5_R9.bin"
Get-FileHash -LiteralPath "$hexPath/SCARA_Cartesian_NC_Home_v5_R9.hex", "$hexPath/SCARA_Cartesian_NC_Home_v5_R9.bin"





