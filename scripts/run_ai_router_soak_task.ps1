param(
    [string]$RepoRoot = "",
    [string]$PythonExe = "",
    [string]$BaseUrl = "http://127.0.0.1:8002",
    [int]$Rounds = 3,
    [int]$Timeout = 120,
    [double]$SleepSeconds = 0.25,
    [switch]$NoWarmup
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

$ScriptPath = (Resolve-Path (Join-Path $RepoRoot "scripts\atlas_multi_ai_soak_test.py")).Path
$LogPath = Join-Path $RepoRoot "logs\ai_router_soak_nightly.log"
$PythonExe = Resolve-Python -Root $RepoRoot -ExplicitPath $PythonExe

New-Item -ItemType Directory -Path (Split-Path $LogPath -Parent) -Force | Out-Null
Set-Location $RepoRoot

$args = @(
    $ScriptPath,
    "--base-url", $BaseUrl,
    "--rounds", [string]([Math]::Max(1, $Rounds)),
    "--timeout", [string]([Math]::Max(10, $Timeout)),
    "--sleep-seconds", [string]([Math]::Max(0, $SleepSeconds))
)
if ($NoWarmup.IsPresent) {
    $args += "--no-warmup"
}

$stamp = (Get-Date).ToString("o")
"$stamp START task=ai_router_soak base_url=$BaseUrl rounds=$Rounds timeout=$Timeout" |
    Out-File -FilePath $LogPath -Append -Encoding utf8

try {
    $tmpOut = [System.IO.Path]::GetTempFileName()
    $tmpErr = [System.IO.Path]::GetTempFileName()
    $proc = Start-Process -FilePath $PythonExe `
        -ArgumentList $args `
        -WorkingDirectory $RepoRoot `
        -WindowStyle Hidden `
        -RedirectStandardOutput $tmpOut `
        -RedirectStandardError $tmpErr `
        -PassThru `
        -Wait

    if (Test-Path $tmpOut) {
        Get-Content $tmpOut -Raw -ErrorAction SilentlyContinue |
            Out-File -FilePath $LogPath -Append -Encoding utf8
        Remove-Item $tmpOut -Force -ErrorAction SilentlyContinue
    }
    if (Test-Path $tmpErr) {
        $errText = Get-Content $tmpErr -Raw -ErrorAction SilentlyContinue
        if (-not [string]::IsNullOrWhiteSpace($errText)) {
            $errText | Out-File -FilePath $LogPath -Append -Encoding utf8
        }
        Remove-Item $tmpErr -Force -ErrorAction SilentlyContinue
    }

    $endStamp = (Get-Date).ToString("o")
    "$endStamp END task=ai_router_soak exit_code=$($proc.ExitCode)" |
        Out-File -FilePath $LogPath -Append -Encoding utf8
    exit ([int]$proc.ExitCode)
} catch {
    $errStamp = (Get-Date).ToString("o")
    "$errStamp TASK_WRAPPER_ERROR $($_.Exception.Message)" |
        Out-File -FilePath $LogPath -Append -Encoding utf8
    exit 1
}
