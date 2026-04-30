[CmdletBinding()]
param(
    [string]$BaseUrl = "http://127.0.0.1:8795",
    [string]$HealthPath = "/health",
    [int]$Port = 8795,
    [string]$AppModule = "atlas_code_quant.api.main:app",
    [int]$StartupGraceSec = 420
)

$ErrorActionPreference = "Stop"

function Test-Healthy([string]$Url) {
    try {
        $resp = Invoke-WebRequest -Method Get -Uri $Url -TimeoutSec 6 -UseBasicParsing
        return ($resp.StatusCode -ge 200 -and $resp.StatusCode -lt 300)
    } catch {
        return $false
    }
}

function Stop-Existing([string]$Module, [int]$TargetPort) {
    $processes = Get-CimInstance Win32_Process -Filter "Name='python.exe'" -ErrorAction SilentlyContinue
    if (-not $processes) { return }
    foreach ($proc in $processes) {
        $cmd = $proc.CommandLine
        if (-not $cmd) { continue }
        if ($cmd -like "*uvicorn*" -and $cmd -like "*$Module*" -and $cmd -like "*--port $TargetPort*") {
            try { Stop-Process -Id $proc.ProcessId -Force -ErrorAction Stop } catch {}
        }
    }
}

function Get-Existing([string]$Module, [int]$TargetPort) {
    $items = @()
    $processes = Get-CimInstance Win32_Process -Filter "Name='python.exe'" -ErrorAction SilentlyContinue
    if (-not $processes) {
        return $items
    }
    foreach ($proc in $processes) {
        $cmd = $proc.CommandLine
        if (-not $cmd) { continue }
        if ($cmd -like "*uvicorn*" -and $cmd -like "*$Module*" -and $cmd -like "*--port $TargetPort*") {
            $items += $proc
        }
    }
    return $items
}

$healthUrl = "{0}{1}" -f $BaseUrl.TrimEnd("/"), $HealthPath
if (Test-Healthy -Url $healthUrl) {
    exit 0
}

$existing = Get-Existing -Module $AppModule -TargetPort $Port
if ($existing.Count -gt 0) {
    foreach ($proc in $existing) {
        try {
            $runtime = Get-Process -Id $proc.ProcessId -ErrorAction Stop
            $ageSec = (New-TimeSpan -Start $runtime.StartTime -End (Get-Date)).TotalSeconds
            if ($ageSec -lt [Math]::Max(60, $StartupGraceSec)) {
                # Evita bucles kill/restart durante warmup.
                exit 0
            }
        } catch {
            continue
        }
    }
}

$repoRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot "..\..\"))
$pythonExe = Join-Path $repoRoot ".venv_push\Scripts\python.exe"
if (-not (Test-Path $pythonExe)) {
    $pythonExe = "python"
}

Stop-Existing -Module $AppModule -TargetPort $Port

$logDir = Join-Path $repoRoot "logs"
New-Item -ItemType Directory -Path $logDir -Force | Out-Null
$outLog = Join-Path $logDir "atlas_8795_uvicorn.out.log"
$errLog = Join-Path $logDir "atlas_8795_uvicorn.err.log"

$args = @(
    "-m", "uvicorn", $AppModule,
    "--host", "0.0.0.0",
    "--port", "$Port",
    "--log-level", "info"
)

Start-Process -FilePath $pythonExe `
    -ArgumentList $args `
    -WorkingDirectory $repoRoot `
    -RedirectStandardOutput $outLog `
    -RedirectStandardError $errLog `
    -WindowStyle Hidden | Out-Null

exit 0
