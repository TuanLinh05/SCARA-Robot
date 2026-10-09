# Shared native-tool invocation and Python environment for firmware/GUI scripts.
function Resolve-ProjectTool {
    param([string]$Requested, [string]$Command, [string[]]$Candidates = @())
    if ($Requested) {
        $resolved = Get-Command $Requested -ErrorAction SilentlyContinue
        if ($resolved) { return $resolved.Source }
        throw "Tool not found: $Requested"
    }
    $resolved = Get-Command $Command -ErrorAction SilentlyContinue
    if ($resolved) { return $resolved.Source }
    foreach ($candidate in $Candidates) {
        if (Test-Path -LiteralPath $candidate -PathType Leaf) { return $candidate }
    }
    throw "Install $Command or pass its executable path explicitly."
}

function Invoke-ProjectCommand {
    param([string]$Executable, [string[]]$Arguments, [string]$Description)
    & $Executable @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "$Description failed (exit code $LASTEXITCODE)."
    }
}

function Invoke-WithProjectPython {
    param([string]$Workspace, [scriptblock]$Action)
    $savedEnvironment = @{}
    foreach ($name in @('PYTHONPATH', 'PYTHONUTF8', 'PYTHONDONTWRITEBYTECODE', 'TEMP', 'TMP')) {
        $savedEnvironment[$name] = (Get-Item -LiteralPath "Env:$name" -ErrorAction SilentlyContinue).Value
    }
    try {
        $paths = @((Join-Path $Workspace 'Software/src'), (Join-Path $Workspace 'Software/.packaging'))
        if ($savedEnvironment['PYTHONPATH']) { $paths += $savedEnvironment['PYTHONPATH'] }
        $env:PYTHONPATH = $paths -join [IO.Path]::PathSeparator
        $env:PYTHONUTF8 = '1'
        $env:PYTHONDONTWRITEBYTECODE = '1'
        $temporaryDirectory = Join-Path $Workspace 'Software/build/temp'
        New-Item -ItemType Directory -Force -Path $temporaryDirectory | Out-Null
        $env:TEMP = $temporaryDirectory
        $env:TMP = $temporaryDirectory
        & $Action
    } finally {
        foreach ($name in $savedEnvironment.Keys) {
            if ($null -eq $savedEnvironment[$name]) {
                Remove-Item -LiteralPath "Env:$name" -ErrorAction SilentlyContinue
            } else {
                Set-Item -LiteralPath "Env:$name" -Value $savedEnvironment[$name]
            }
        }
    }
}
