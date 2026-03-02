# Shared ATLAS runtime helpers for Python resolution and preflight checks.
Set-StrictMode -Version Latest

function Test-AtlasRuntimePreflight {
    param(
        [Parameter(Mandatory = $true)]
        [string]$PythonExe,
        [Parameter(Mandatory = $true)]
        [string]$RepoRoot,
        [switch]$Quiet
    )

    $preflightScript = Join-Path $PSScriptRoot "atlas_runtime_preflight.py"
    if (-not (Test-Path $preflightScript)) {
        return $true
    }

    try {
        & $PythonExe $preflightScript --json *> $null
        if ($LASTEXITCODE -eq 0) {
            return $true
        }
        if (-not $Quiet) {
            Write-Warning "ATLAS runtime preflight failed for '$PythonExe' (exit=$LASTEXITCODE)."
        }
    } catch {
        if (-not $Quiet) {
            Write-Warning "ATLAS runtime preflight exception for '$PythonExe': $($_.Exception.Message)"
        }
    }
    return $false
}

function Resolve-AtlasPython {
    param(
        [Parameter(Mandatory = $true)]
        [string]$RepoRoot,
        [switch]$RequirePreflight
    )

    $candidates = New-Object System.Collections.Generic.List[string]

    if ($env:ATLAS_PYTHON -and (Test-Path $env:ATLAS_PYTHON)) {
        $candidates.Add($env:ATLAS_PYTHON)
    }
    if ($env:PYTHON -and (Test-Path $env:PYTHON)) {
        $candidates.Add($env:PYTHON)
    }

    $candidates.Add((Join-Path $RepoRoot ".venv\Scripts\python.exe"))
    $candidates.Add((Join-Path $RepoRoot "venv\Scripts\python.exe"))

    try {
        $cmd = Get-Command python -ErrorAction Stop
        if ($cmd -and $cmd.Source) {
            $candidates.Add($cmd.Source)
        }
    } catch {
        # No python command in PATH.
    }

    $seen = @{}
    foreach ($candidate in $candidates) {
        if (-not $candidate) {
            continue
        }
        if ($seen.ContainsKey($candidate)) {
            continue
        }
        $seen[$candidate] = $true
        if (-not (Test-Path $candidate)) {
            continue
        }
        if ($RequirePreflight -and -not (Test-AtlasRuntimePreflight -PythonExe $candidate -RepoRoot $RepoRoot -Quiet)) {
            continue
        }
        return $candidate
    }

    throw "No ATLAS Python interpreter found$([string]::Concat($(if($RequirePreflight){' that passes runtime preflight'}else{''})))."
}

