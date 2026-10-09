param(
    [string]$ZephyrRoot = 'D:/zephyrproject',
    [string]$SdkRoot = 'D:/zephyr-sdk/zephyr-sdk-0.16.8',
    [string]$Python = '',
    [string]$BuildDirectory = 'build_mcu',
    [switch]$SkipExport
)
$ErrorActionPreference = 'Stop'
$project = Split-Path -Parent $PSScriptRoot
$workspace = Split-Path -Parent (Split-Path -Parent $project)
. (Join-Path $workspace 'tools/ProjectTools.ps1')
$cmakeTool = Resolve-ProjectTool -Command cmake
$pythonPath = Resolve-ProjectTool -Requested $Python -Command '__zephyr_python__' -Candidates @((Join-Path $ZephyrRoot '.venv/Scripts/python.exe'))
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
$sourcePath = Join-Path $aliasPath 'Firmware/ScaraCartesian'
$buildPath = if ([IO.Path]::IsPathRooted($BuildDirectory)) { $BuildDirectory } else { Join-Path $sourcePath $BuildDirectory }
$savedZephyr = $env:ZEPHYR_BASE
$savedSdk = $env:ZEPHYR_SDK_INSTALL_DIR
try {
    $env:ZEPHYR_BASE = Join-Path $ZephyrRoot 'zephyr'
    $env:ZEPHYR_SDK_INSTALL_DIR = $SdkRoot
    Invoke-WithProjectPython -Workspace $workspace -Action {
        Invoke-ProjectCommand -Executable $cmakeTool -Arguments @('-S', $sourcePath, '-B', $buildPath, '-G', 'Ninja', '-DBOARD=stm32_min_dev@blue', "-DPython3_EXECUTABLE=$pythonPath", '-DBUILD_VERSION=scara-cart-nc-home-v5-r9') -Description 'Zephyr configuration'
        Invoke-ProjectCommand -Executable $cmakeTool -Arguments @('--build', $buildPath) -Description 'Zephyr build'
    }
} finally {
    $env:ZEPHYR_BASE = $savedZephyr
    $env:ZEPHYR_SDK_INSTALL_DIR = $savedSdk
}
if ($SkipExport) { return }
$hexPath = Join-Path $project 'Hex'
New-Item -ItemType Directory -Force -Path $hexPath | Out-Null
Copy-Item -LiteralPath "$buildPath/zephyr/zephyr.hex" -Destination "$hexPath/SCARA_Cartesian_NC_Home_v5_R9.hex"
Copy-Item -LiteralPath "$buildPath/zephyr/zephyr.bin" -Destination "$hexPath/SCARA_Cartesian_NC_Home_v5_R9.bin"
Get-FileHash -LiteralPath "$hexPath/SCARA_Cartesian_NC_Home_v5_R9.hex", "$hexPath/SCARA_Cartesian_NC_Home_v5_R9.bin"





