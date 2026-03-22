# ATLAS — Arranca Prometheus + Atlas-Quant metrics
# Ejecutar desde C:\ATLAS_PUSH (no requiere Admin)
#
# Puertos:
#   :9090  — metrics endpoint Python (prometheus_client)
#   :9091  — Prometheus server UI + query API
#   :3002  — Grafana

param()
$ErrorActionPreference = "Stop"
$Root    = "C:\ATLAS_PUSH"
$PromDir = "$Root\tools\prometheus"
$PromExe = "$PromDir\prometheus.exe"
$PromCfg = "$Root\docker\prometheus\prometheus.yml"
$PromVer = "2.51.2"
$VenvPy  = "$Root\venv\Scripts\python.exe"

Set-Location $Root

# ── [1] Descargar Prometheus si no existe ─────────────────────────
if (-not (Test-Path $PromExe)) {
    Write-Host "[1/4] Descargando Prometheus $PromVer..." -ForegroundColor Cyan
    $zip = "$env:TEMP\prometheus.zip"
    $url = "https://github.com/prometheus/prometheus/releases/download/v$PromVer/prometheus-$PromVer.windows-amd64.zip"
    Invoke-WebRequest -Uri $url -OutFile $zip -UseBasicParsing
    New-Item -ItemType Directory -Path $PromDir -Force | Out-Null
    Expand-Archive -Path $zip -DestinationPath "$env:TEMP\prom_extract" -Force
    $src = Get-ChildItem "$env:TEMP\prom_extract" -Filter "prometheus-*" | Select-Object -First 1
    Copy-Item "$($src.FullName)\prometheus.exe" $PromExe
    Remove-Item $zip -Force
    Write-Host "  OK: $PromExe" -ForegroundColor Gray
} else {
    Write-Host "[1/4] Prometheus ya existe." -ForegroundColor Gray
}

# ── [2] Servidor de metricas Python en :9090 ─────────────────────
Write-Host "[2/4] Iniciando servidor de metricas Atlas en :9090..." -ForegroundColor Cyan
$port9090 = netstat -ano | Select-String "0.0.0.0:9090 " | Select-String "LISTENING"
if ($port9090) {
    Write-Host "  :9090 ya escucha." -ForegroundColor Gray
} else {
    $pyCode = "from atlas_code_quant.production.grafana_dashboard import GrafanaDashboard; import time; gd=GrafanaDashboard(); gd.start_metrics_server(9090); [time.sleep(60) for _ in iter(int,1)]"
    Start-Process -FilePath $VenvPy `
        -ArgumentList "-c", $pyCode `
        -WorkingDirectory $Root `
        -WindowStyle Minimized
    Start-Sleep -Seconds 3
    Write-Host "  Metricas disponibles en http://localhost:9090/metrics" -ForegroundColor Gray
}

# ── [3] Prometheus en :9091 ───────────────────────────────────────
Write-Host "[3/4] Iniciando Prometheus en :9091..." -ForegroundColor Cyan
$port9091 = netstat -ano | Select-String "0.0.0.0:9091 " | Select-String "LISTENING"
if ($port9091) {
    Write-Host "  :9091 ya escucha." -ForegroundColor Gray
} else {
    Start-Process -FilePath $PromExe `
        -ArgumentList "--config.file=$PromCfg", "--web.listen-address=:9091", "--storage.tsdb.path=$PromDir\data", "--storage.tsdb.retention.time=7d" `
        -WorkingDirectory $PromDir `
        -WindowStyle Minimized
    Start-Sleep -Seconds 3
    Write-Host "  Prometheus en http://localhost:9091" -ForegroundColor Gray
}

# ── [4] Configurar datasource en Grafana via API ──────────────────
Write-Host "[4/4] Configurando datasource en Grafana..." -ForegroundColor Cyan
$creds   = [Convert]::ToBase64String([Text.Encoding]::ASCII.GetBytes("admin:atlas2026"))
$headers = @{ "Authorization" = "Basic $creds"; "Content-Type" = "application/json" }
$body    = '{"name":"atlas-prometheus","uid":"atlas-prom","type":"prometheus","url":"http://localhost:9091","access":"proxy","isDefault":true,"jsonData":{"timeInterval":"5s"}}'

try {
    Invoke-RestMethod -Uri "http://localhost:3002/api/datasources" -Method POST -Headers $headers -Body $body | Out-Null
    Write-Host "  Datasource creado OK." -ForegroundColor Green
} catch {
    try {
        $ds = Invoke-RestMethod -Uri "http://localhost:3002/api/datasources/name/atlas-prometheus" -Headers $headers
        Invoke-RestMethod -Uri "http://localhost:3002/api/datasources/$($ds.id)" -Method PUT -Headers $headers -Body $body | Out-Null
        Write-Host "  Datasource actualizado OK." -ForegroundColor Green
    } catch {
        Write-Host "  AVISO: configura el datasource manualmente en Grafana > Connections" -ForegroundColor Yellow
        Write-Host "  URL: http://localhost:9091" -ForegroundColor Yellow
    }
}

# ── Resumen ───────────────────────────────────────────────────────
Write-Host ""
Write-Host "============================================" -ForegroundColor Green
Write-Host "  ATLAS Monitoring Stack LISTO" -ForegroundColor Green
Write-Host "============================================" -ForegroundColor Green
Write-Host "  Metrics Python : http://localhost:9090/metrics"
Write-Host "  Prometheus UI  : http://localhost:9091"
Write-Host "  Grafana        : http://localhost:3002"
Write-Host "  Login          : admin / atlas2026"
Write-Host "============================================" -ForegroundColor Green
Start-Process "http://localhost:3002"
