# Keep this script ASCII-only so Windows PowerShell 5.1 does not misread
# BOM-less UTF-8 source text and turn localized messages into syntax errors.
param(
    [string]$Python = "",
    [ValidateRange(1, 65535)]
    [int]$Port = 8000,
    [switch]$SkipInstall
)

$ErrorActionPreference = "Stop"
$appRoot = $PSScriptRoot
$venvRoot = Join-Path $appRoot ".venv"
$venvPython = Join-Path $venvRoot "Scripts\python.exe"
$requirements = Join-Path $appRoot "backend\requirements.txt"

function Resolve-PythonExecutable {
    param([string]$Requested)

    if ($Requested) {
        if (Test-Path -LiteralPath $Requested) {
            return (Resolve-Path -LiteralPath $Requested).Path
        }
        $requestedCommand = Get-Command $Requested -ErrorAction SilentlyContinue
        if ($requestedCommand) {
            return $requestedCommand.Source
        }
        throw "Cannot find the requested Python executable: $Requested"
    }

    $pythonCommand = Get-Command python -ErrorAction SilentlyContinue
    if ($pythonCommand) {
        return $pythonCommand.Source
    }

    if ($env:LOCALAPPDATA) {
        $pythonRoot = Join-Path $env:LOCALAPPDATA "Programs\Python"
        if (Test-Path -LiteralPath $pythonRoot) {
            $candidate = Get-ChildItem -LiteralPath $pythonRoot -Directory |
                Sort-Object Name -Descending |
                ForEach-Object { Join-Path $_.FullName "python.exe" } |
                Where-Object { Test-Path -LiteralPath $_ } |
                Select-Object -First 1
            if ($candidate) {
                return $candidate
            }
        }
    }

    throw "Python 3 was not found. Use -Python with the full path to python.exe."
}

if (-not (Test-Path -LiteralPath $venvPython)) {
    $basePython = Resolve-PythonExecutable -Requested $Python
    Write-Host "Creating virtual environment: $venvRoot"
    & $basePython -m venv $venvRoot
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to create the virtual environment. Exit code: $LASTEXITCODE"
    }
}

if (-not $SkipInstall) {
    & $venvPython -c "import fastapi, numpy, uvicorn" 2>$null
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Installing FastAPI Web dependencies..."
        & $venvPython -m pip install -r $requirements
        if ($LASTEXITCODE -ne 0) {
            throw "Failed to install dependencies. Exit code: $LASTEXITCODE"
        }
    }
}

Write-Host "ModelDev Web: http://127.0.0.1:$Port/"
& $venvPython -m uvicorn backend.app:app `
    --app-dir $appRoot `
    --host 127.0.0.1 `
    --port $Port
exit $LASTEXITCODE
