param(
    [string]$RepoRoot = "",
    [string]$PythonExe = "",
    [bool]$RunQualityGates = $false,
    [switch]$WithRestart
)

$ErrorActionPreference = "Stop"

if (-not $RepoRoot) {
    $RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
}

function Resolve-Python {
    param([string]$Root, [string]$ExplicitPath)
    if ($ExplicitPath -and (Test-Path $ExplicitPath)) {
        return (Resolve-Path $ExplicitPath).Path
    }

    $candidates = @(
        (Join-Path $Root ".venv\Scripts\python.exe"),
        (Join-Path $Root "venv\Scripts\python.exe")
    )
    foreach ($candidate in $candidates) {
        if (Test-Path $candidate) {
            return (Resolve-Path $candidate).Path
        }
    }

    $cmd = Get-Command python -ErrorAction SilentlyContinue
    if ($cmd -and $cmd.Source) {
        return $cmd.Source
    }

    throw "Python interpreter not found (.venv/venv/python)."
}

$AuditScript = (Resolve-Path (Join-Path $RepoRoot "scripts\atlas_agent_e2e_audit.py")).Path
$TaskLog = Join-Path $RepoRoot "logs\atlas_agent_e2e_task.log"
$PythonExe = Resolve-Python -Root $RepoRoot -ExplicitPath $PythonExe

New-Item -ItemType Directory -Path (Split-Path $TaskLog -Parent) -Force | Out-Null
Set-Location $RepoRoot

$args = @($AuditScript, "--run-quality-gates", $RunQualityGates.ToString().ToLowerInvariant())
if ($WithRestart.IsPresent) {
    $args += "--restart-push"
}

try {
    & $PythonExe @args 2>&1 | Out-File -FilePath $TaskLog -Append -Encoding utf8
    exit $LASTEXITCODE
} catch {
    $stamp = (Get-Date).ToString("o")
    "$stamp TASK_WRAPPER_ERROR $($_.Exception.Message)" | Out-File -FilePath $TaskLog -Append -Encoding utf8
    exit 1
}
