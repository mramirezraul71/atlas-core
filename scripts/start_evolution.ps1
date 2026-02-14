# ATLAS_EVOLUTION - Daemon de crecimiento perpetuo (Tríada cada 12h)
# Ejecutar desde ATLAS_PUSH: .\scripts\start_evolution.ps1
# Credenciales: C:\dev\credenciales.txt (PyPI, GitHub, Hugging Face)

$PUSH = if ($PSScriptRoot) { (Resolve-Path (Join-Path $PSScriptRoot "..")).Path } else { "C:\ATLAS_PUSH" }
$LogDir = Join-Path $PUSH "logs"
$ErrLog = Join-Path $LogDir "atlas_evolution.err.log"
$OutLog = Join-Path $LogDir "atlas_evolution.log"

if (-not (Test-Path $LogDir)) { New-Item -ItemType Directory -Path $LogDir -Force | Out-Null }

Write-Host "ATLAS_EVOLUTION Daemon - Iniciando en segundo plano" -ForegroundColor Cyan
Write-Host "  Base: $PUSH" -ForegroundColor Gray
Write-Host "  Logs: $OutLog | Errores: $ErrLog" -ForegroundColor Gray

Set-Location $PUSH
$proc = Start-Process -FilePath "python" -ArgumentList "atlas_evolution.py" -WorkingDirectory $PUSH -WindowStyle Hidden -PassThru -RedirectStandardOutput $OutLog -RedirectStandardError $ErrLog

Write-Host "  PID: $($proc.Id) - Daemon en ejecucion. Ver estado: GET http://127.0.0.1:8791/api/evolution/status" -ForegroundColor Green
