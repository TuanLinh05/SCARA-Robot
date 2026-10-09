param([string]$Python = '', [string]$Gcc = '', [switch]$Gui)
$ErrorActionPreference = 'Stop'
# Software verification compiles/tests firmware first, then the full Python suite.
& (Join-Path $PSScriptRoot 'Software/Verify_Cartesian_NC_GUI.ps1') -Python $Python -Gcc $Gcc -Gui:$Gui
