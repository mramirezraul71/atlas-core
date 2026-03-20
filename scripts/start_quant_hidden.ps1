# Relanza Atlas Code-Quant (8792) en segundo plano sin mostrar consola.
param(
    [int]$HealthTimeoutSec = 45
)

$ErrorActionPreference = "Stop"
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$FreePortScript = Join-Path $PSScriptRoot "free_port.ps1"
$QuantDir = Join-Path $RepoRoot "atlas_code_quant"
$HealthUrl = "http://127.0.0.1:8792/health"
$Python = "C:\Python314\python.exe"
$LogDir = Join-Path $RepoRoot "logs"
$StdOutLog = Join-Path $LogDir "code_quant_hidden_stdout.log"
$StdErrLog = Join-Path $LogDir "code_quant_hidden_stderr.log"

if (-not (Test-Path $QuantDir)) {
    throw "Quant dir no encontrado: $QuantDir"
}

if (-not (Test-Path $Python)) {
    throw "Python no encontrado: $Python"
}

New-Item -ItemType Directory -Path $LogDir -Force | Out-Null

if (Test-Path $FreePortScript) {
    & $FreePortScript -Port 8792 -Kill | Out-Null
}

$args = @("-m", "uvicorn", "api.main:app", "--host", "0.0.0.0", "--port", "8792", "--log-level", "info")
$proc = Start-Process `
    -FilePath $Python `
    -ArgumentList $args `
    -WorkingDirectory $QuantDir `
    -WindowStyle Hidden `
    -RedirectStandardOutput $StdOutLog `
    -RedirectStandardError $StdErrLog `
    -PassThru
Start-Sleep -Milliseconds 500
if ($proc.HasExited) {
    throw "Code-Quant salio inmediatamente tras iniciar (exit=$($proc.ExitCode))."
}

$sw = [System.Diagnostics.Stopwatch]::StartNew()
while ($sw.Elapsed.TotalSeconds -lt $HealthTimeoutSec) {
    try {
        $resp = Invoke-WebRequest -Uri $HealthUrl -TimeoutSec 4 -UseBasicParsing -ErrorAction Stop
        if ($resp.StatusCode -eq 200) {
            Write-Host "Code-Quant OK on 8792 (PID: $($proc.Id))."
            exit 0
        }
    } catch {}
    Start-Sleep -Seconds 2
}

Write-Host "STDOUT log: $StdOutLog"
Write-Host "STDERR log: $StdErrLog"
throw "Code-Quant no respondio en 8792 dentro de ${HealthTimeoutSec}s."
