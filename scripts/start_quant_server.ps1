<#
.SYNOPSIS
    Arranca el servidor Atlas-Quant en puerto 8795.
    Disenado para ser llamado por Task Scheduler a las 9:00 AM.
    Si el servidor ya esta corriendo, no hace nada.
#>

$RepoRoot  = "C:\ATLAS_PUSH"
$QuantDir  = "$RepoRoot\atlas_code_quant"
$Python    = "$RepoRoot\venv\Scripts\python.exe"
$LogOut    = "$RepoRoot\logs\quant_server_stdout.log"
$LogErr    = "$RepoRoot\logs\quant_server_stderr.log"
$Port      = 8795

# Verificar si ya esta corriendo
try {
    $resp = Invoke-WebRequest -Uri "http://127.0.0.1:$Port/health" -TimeoutSec 3 -ErrorAction Stop
    if ($resp.StatusCode -eq 200) {
        Write-Host "[$(Get-Date -Format 'HH:mm:ss')] Servidor Quant ya activo en :$Port — skip" -ForegroundColor Green
        exit 0
    }
} catch { }

# Crear carpeta logs si no existe
if (-not (Test-Path "$RepoRoot\logs")) {
    New-Item -ItemType Directory -Path "$RepoRoot\logs" -Force | Out-Null
}

Write-Host "[$(Get-Date -Format 'HH:mm:ss')] Arrancando servidor Quant en :$Port ..." -ForegroundColor Cyan

# Arrancar uvicorn en segundo plano con logs
$proc = Start-Process -FilePath $Python `
    -ArgumentList "-m uvicorn api.main:app --host 0.0.0.0 --port $Port" `
    -WorkingDirectory $QuantDir `
    -RedirectStandardOutput $LogOut `
    -RedirectStandardError  $LogErr `
    -WindowStyle Hidden `
    -PassThru

Write-Host "[$(Get-Date -Format 'HH:mm:ss')] Proceso iniciado PID=$($proc.Id)" -ForegroundColor Cyan

# Esperar hasta que responda health (max 60s)
$deadline = (Get-Date).AddSeconds(60)
while ((Get-Date) -lt $deadline) {
    Start-Sleep -Seconds 2
    try {
        $r = Invoke-WebRequest -Uri "http://127.0.0.1:$Port/health" -TimeoutSec 3 -ErrorAction Stop
        if ($r.StatusCode -eq 200) {
            Write-Host "[$(Get-Date -Format 'HH:mm:ss')] Servidor OK en :$Port" -ForegroundColor Green
            exit 0
        }
    } catch { }
}

Write-Host "[$(Get-Date -Format 'HH:mm:ss')] WARN: servidor no respondio en 60s (puede seguir iniciando)" -ForegroundColor Yellow
