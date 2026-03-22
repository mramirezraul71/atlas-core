# ATLAS — Arranca Prometheus + Atlas-Quant metrics
# Ejecutar desde C:\ATLAS_PUSH (no requiere Admin)
#
# Puertos:
#   :9090  — metrics endpoint Python (prometheus_client)
#   :9091  — Prometheus server UI + query API
#   :3002  — Grafana (ya corriendo como servicio)

$ErrorActionPreference = "Stop"
$Root      = "C:\ATLAS_PUSH"
$PromDir   = "$Root\tools\prometheus"
$PromExe   = "$PromDir\prometheus.exe"
$PromCfg   = "$Root\docker\prometheus\prometheus.yml"
$PromVer   = "2.51.2"
$VenvPy    = "$Root\venv\Scripts\python.exe"

Set-Location $Root

# ── [1] Descargar Prometheus si no existe ─────────────────────────
if (-not (Test-Path $PromExe)) {
    Write-Host "[1/3] Descargando Prometheus $PromVer para Windows..." -ForegroundColor Cyan
    $zip = "$env:TEMP\prometheus.zip"
    $url = "https://github.com/prometheus/prometheus/releases/download/v$PromVer/prometheus-$PromVer.windows-amd64.zip"
    Invoke-WebRequest -Uri $url -OutFile $zip -UseBasicParsing
    New-Item -ItemType Directory -Path $PromDir -Force | Out-Null
    Expand-Archive -Path $zip -DestinationPath "$env:TEMP\prom_extract" -Force
    $extracted = Get-ChildItem "$env:TEMP\prom_extract" -Filter "prometheus-*" | Select-Object -First 1
    Copy-Item "$($extracted.FullName)\prometheus.exe" $PromExe
    Copy-Item "$($extracted.FullName)\promtool.exe"   "$PromDir\promtool.exe"
    Remove-Item $zip -Force
    Write-Host "  Prometheus instalado en $PromDir" -ForegroundColor Gray
} else {
    Write-Host "[1/3] Prometheus ya existe — omitiendo descarga." -ForegroundColor Gray
}

# ── [2] Arrancar atlas_quant_core en modo API (expone :9090/metrics)
Write-Host "[2/3] Iniciando Atlas-Quant API (metrics en :9090)..." -ForegroundColor Cyan
$quantRunning = netstat -ano | Select-String ":9090 " | Select-String "LISTENING"
if ($quantRunning) {
    Write-Host "  :9090 ya escucha — omitiendo." -ForegroundColor Gray
} else {
    $env:PYTHONPATH = "$Root\atlas_code_quant"
    $env:ATLAS_MODE = "paper"
    $env:QUANT_API_KEY = "atlas-quant-local"
    Start-Process -FilePath $VenvPy `
        -ArgumentList "-m", "uvicorn", "api.main:app", "--host", "0.0.0.0", "--port", "8792", "--log-level", "info" `
        -WorkingDirectory "$Root\atlas_code_quant" `
        -WindowStyle Minimized
    Write-Host "  Esperando arranque..." -ForegroundColor Gray
    Start-Sleep -Seconds 4

    # Iniciar servidor de métricas Prometheus desde Python
    Start-Process -FilePath $VenvPy -ArgumentList @(
        "-c",
        "from atlas_code_quant.production.grafana_dashboard import GrafanaDashboard; gd=GrafanaDashboard(); gd.start_metrics_server(9090); import time; [time.sleep(3600) for _ in iter(int,1)]"
    ) -WorkingDirectory $Root -WindowStyle Minimized
    Write-Host "  Metrics server iniciado en :9090" -ForegroundColor Gray
    Start-Sleep -Seconds 2
}

# ── [3] Arrancar Prometheus apuntando a :9090, exponiendo :9091 ───
Write-Host "[3/3] Iniciando Prometheus (UI en :9091)..." -ForegroundColor Cyan
$promRunning = netstat -ano | Select-String ":9091 " | Select-String "LISTENING"
if ($promRunning) {
    Write-Host "  :9091 ya escucha — Prometheus ya corre." -ForegroundColor Gray
} else {
    Start-Process -FilePath $PromExe -ArgumentList @(
        "--config.file=$PromCfg",
        "--web.listen-address=:9091",
        "--storage.tsdb.retention.time=7d",
        "--storage.tsdb.path=$PromDir\data"
    ) -WorkingDirectory $PromDir -WindowStyle Minimized
    Start-Sleep -Seconds 3
    Write-Host "  Prometheus corriendo en http://localhost:9091" -ForegroundColor Gray
}

# ── [4] Configurar datasource en Grafana via API ──────────────────
Write-Host "[+] Configurando datasource Prometheus en Grafana..." -ForegroundColor Cyan
Start-Sleep -Seconds 2

$grafanaAuth = [Convert]::ToBase64String([Text.Encoding]::ASCII.GetBytes("admin:atlas2026"))
$dsBody = @{
    name      = "atlas-prometheus"
    uid       = "atlas-prom"
    type      = "prometheus"
    url       = "http://localhost:9091"
    access    = "proxy"
    isDefault = $true
    jsonData  = @{ timeInterval = "5s" }
} | ConvertTo-Json

try {
    # Intentar crear; si ya existe, actualizar
    $resp = Invoke-RestMethod -Uri "http://localhost:3002/api/datasources" `
        -Method POST `
        -Headers @{ Authorization = "Basic $grafanaAuth"; "Content-Type" = "application/json" } `
        -Body $dsBody -ErrorAction SilentlyContinue
    Write-Host "  Datasource creado OK" -ForegroundColor Green
} catch {
    # Si ya existe (409), buscar ID y hacer PUT
    try {
        $existing = Invoke-RestMethod -Uri "http://localhost:3002/api/datasources/name/atlas-prometheus" `
            -Headers @{ Authorization = "Basic $grafanaAuth" } -ErrorAction SilentlyContinue
        if ($existing.id) {
            Invoke-RestMethod -Uri "http://localhost:3002/api/datasources/$($existing.id)" `
                -Method PUT `
                -Headers @{ Authorization = "Basic $grafanaAuth"; "Content-Type" = "application/json" } `
                -Body $dsBody | Out-Null
            Write-Host "  Datasource actualizado OK (id=$($existing.id))" -ForegroundColor Green
        }
    } catch {
        Write-Host "  AVISO: no se pudo configurar datasource via API — hazlo manual en Grafana" -ForegroundColor Yellow
    }
}

# ── Resumen ───────────────────────────────────────────────────────
Write-Host ""
Write-Host "======================================" -ForegroundColor Green
Write-Host " ATLAS Monitoring Stack" -ForegroundColor Green
Write-Host "======================================" -ForegroundColor Green
Write-Host " Metrics Python  : http://localhost:9090/metrics"
Write-Host " Prometheus UI   : http://localhost:9091"
Write-Host " Grafana         : http://localhost:3002  (admin/atlas2026)"
Write-Host "======================================" -ForegroundColor Green
Write-Host ""
Write-Host "Abriendo Grafana..." -ForegroundColor Cyan
Start-Sleep -Seconds 2
Start-Process "http://localhost:3002"
