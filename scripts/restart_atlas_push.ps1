# Reinicia Atlas PUSH (adaptador HTTP + /ui) en :8791 sin interacción.
# Mata el proceso que escuche 8791, arranca uvicorn desacoplado y escribe logs.
#
# Si el arranque “completo” tarda o falla: descomenta en tu entorno
#   $env:HUMANOID_ENABLED = "0"; $env:SCHED_ENABLED = "0"
# (salta scheduler/humanoid pesado; /ui y API siguen).
$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$FreePort = Join-Path $PSScriptRoot "free_port.ps1"
$LogDir = Join-Path $Root "logs"
if (-not (Test-Path $LogDir)) { New-Item -ItemType Directory -Path $LogDir -Force | Out-Null }
$LogOut = Join-Path $LogDir "atlas_push_8791.stdout.log"
$LogErr = Join-Path $LogDir "atlas_push_8791.stderr.log"

if (Test-Path $FreePort) {
    & $FreePort -Port 8791 -Kill | Out-Null
} else {
    Write-Warning "free_port.ps1 no encontrado; si 8791 sigue ocupado, mata el proceso a mano."
}
Start-Sleep -Milliseconds 600

$env:PYTHONPATH = $Root
if (-not $env:ATLAS_ENABLE_RADAR_KALSHI) { $env:ATLAS_ENABLE_RADAR_KALSHI = "1" }

$pyCmd = Get-Command python -ErrorAction SilentlyContinue
$py = if ($pyCmd) { $pyCmd.Source } else { $null }
$usingLauncher = $false
if (-not $py) {
    $launcher = Get-Command py -ErrorAction SilentlyContinue
    if ($launcher) {
        $py = $launcher.Source
        $usingLauncher = $true
    }
}
if (-not $py) { throw "No se encontró python ni py launcher en PATH" }

# Desacoplar: logs propios, sin ventana (el proceso sigue aunque cierres el terminal)
$argList = @()
if ($usingLauncher) { $argList += @("-3.11") }
$argList += @(
    "-m", "uvicorn", "atlas_adapter.atlas_http_api:app",
    "--host", "127.0.0.1", "--port", "8791", "--log-level", "info"
)
Start-Process -FilePath $py -ArgumentList $argList -WorkingDirectory $Root `
    -WindowStyle Hidden `
    -RedirectStandardOutput $LogOut -RedirectStandardError $LogErr

$health = "http://127.0.0.1:8791/health"
$ok = $false
for ($i = 0; $i -lt 60; $i++) {
    Start-Sleep -Seconds 2
    try {
        $r = Invoke-WebRequest -Uri $health -UseBasicParsing -TimeoutSec 3
        if ($r.StatusCode -ge 200 -and $r.StatusCode -lt 500) { $ok = $true; break }
    } catch { }
}
if ($ok) {
    Write-Host "PUSH OK: $health  /ui: http://127.0.0.1:8791/ui" -ForegroundColor Green
} else {
    Write-Warning "PUSH no respondio en 2 min. Revisa: $LogErr y $LogOut"
    Write-Host "Sugerencia: HUMANOID_ENABLED=0 SCHED_ENABLED=0 (ver comentario en el script)." -ForegroundColor Yellow
}
