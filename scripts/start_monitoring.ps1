# ATLAS — Arranca Prometheus + Atlas-Quant metrics
# Ejecutar desde C:\ATLAS_PUSH (no requiere Admin)
#
# Puertos:
#   :9090  — metrics endpoint Python (prometheus_client)
#   :9091  — Prometheus server UI + query API
#   :3003  — Grafana local workspace

param()
$ErrorActionPreference = "Stop"
$Root    = "C:\ATLAS_PUSH"
$PromDir = "$Root\tools\prometheus"
$PromExe = "$PromDir\prometheus.exe"
$PromCfg = "$Root\docker\prometheus\prometheus.yml"
$PromLocalCfg = "$PromDir\prometheus.local.yml"
$PromVer = "2.51.2"
$VenvPy  = "$Root\venv\Scripts\python.exe"
$GrafanaPort = 3003
$GrafanaUrl = "http://localhost:$GrafanaPort"
$GrafanaCfg = "$Root\grafana\local\custom.ini"

Set-Location $Root

function Get-GrafanaHome {
    $candidates = @(
        "C:\Program Files\GrafanaLabs\grafana",
        "C:\Program Files (x86)\GrafanaLabs\grafana",
        "C:\GrafanaLabs\grafana"
    )
    foreach ($candidate in $candidates) {
        if (Test-Path (Join-Path $candidate "bin\grafana-server.exe")) {
            return $candidate
        }
    }
    $found = Get-ChildItem "C:\Program Files" -Filter "grafana-server.exe" -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($found) {
        return (Split-Path $found.DirectoryName)
    }
    return $null
}

function Wait-GrafanaReady([string]$Url, [int]$Attempts = 20) {
    for ($i = 0; $i -lt $Attempts; $i++) {
        try {
            Invoke-WebRequest -Uri "$Url/api/health" -UseBasicParsing -TimeoutSec 3 | Out-Null
            return $true
        } catch {
            Start-Sleep -Seconds 2
        }
    }
    return $false
}

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
    $pyCode = "from atlas_code_quant.production.grafana_dashboard import GrafanaDashboard; import time; gd=GrafanaDashboard(); gd.start_metrics_server(9090);`nwhile True:`n    gd.sync_from_canonical(account_scope='paper')`n    time.sleep(2)"
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
    @"
global:
  scrape_interval: 2s
  evaluation_interval: 2s

scrape_configs:
  - job_name: atlas-quant
    static_configs:
      - targets:
          - localhost:9090
    metrics_path: /metrics
    scrape_timeout: 4s
"@ | Set-Content -Path $PromLocalCfg -Encoding UTF8

    Start-Process -FilePath $PromExe `
        -ArgumentList "--config.file=$PromLocalCfg", "--web.listen-address=:9091", "--storage.tsdb.path=$PromDir\data", "--storage.tsdb.retention.time=7d" `
        -WorkingDirectory $PromDir `
        -WindowStyle Minimized
    Start-Sleep -Seconds 3
    Write-Host "  Prometheus en http://localhost:9091" -ForegroundColor Gray
}

# ── [4] Grafana local en :3003 ─────────────────────────────────────
Write-Host "[4/5] Iniciando Grafana local en :$GrafanaPort..." -ForegroundColor Cyan
$portGrafana = netstat -ano | Select-String "0.0.0.0:$GrafanaPort " | Select-String "LISTENING"
if ($portGrafana) {
    Write-Host "  :$GrafanaPort ya escucha." -ForegroundColor Gray
} else {
    if (-not (Test-Path $GrafanaCfg)) {
        throw "No se encontro la configuracion local de Grafana: $GrafanaCfg"
    }
    $grafanaHome = Get-GrafanaHome
    if (-not $grafanaHome) {
        throw "No se encontro grafana-server.exe. Instala Grafana antes de ejecutar este script."
    }
    New-Item -ItemType Directory -Path "$Root\grafana\local\data" -Force | Out-Null
    New-Item -ItemType Directory -Path "$Root\grafana\local\logs" -Force | Out-Null
    New-Item -ItemType Directory -Path "$Root\grafana\local\plugins" -Force | Out-Null
    Start-Process -FilePath (Join-Path $grafanaHome "bin\grafana-server.exe") `
        -ArgumentList @("--config", $GrafanaCfg, "--homepath", $grafanaHome) `
        -WorkingDirectory $grafanaHome `
        -WindowStyle Minimized
    if (-not (Wait-GrafanaReady -Url $GrafanaUrl)) {
        throw "Grafana local no respondio en $GrafanaUrl"
    }
    Write-Host "  Grafana local disponible en $GrafanaUrl" -ForegroundColor Gray
}

# ── [5] Configurar datasource + dashboards via API ────────────────
Write-Host "[5/5] Configurando datasource e importando dashboards..." -ForegroundColor Cyan
$creds   = [Convert]::ToBase64String([Text.Encoding]::ASCII.GetBytes("admin:atlas2026"))
$headers = @{ "Authorization" = "Basic $creds"; "Content-Type" = "application/json" }
$body    = '{"name":"atlas-prometheus","uid":"atlas-prom","type":"prometheus","url":"http://localhost:9091","access":"proxy","isDefault":true,"editable":true,"jsonData":{"httpMethod":"GET","timeInterval":"2s"}}'

try {
    Invoke-RestMethod -Uri "$GrafanaUrl/api/datasources" -Method POST -Headers $headers -Body $body | Out-Null
    Write-Host "  Datasource creado OK." -ForegroundColor Green
} catch {
    try {
        $ds = Invoke-RestMethod -Uri "$GrafanaUrl/api/datasources/name/atlas-prometheus" -Headers $headers
        Invoke-RestMethod -Uri "$GrafanaUrl/api/datasources/$($ds.id)" -Method PUT -Headers $headers -Body $body | Out-Null
        Write-Host "  Datasource actualizado OK." -ForegroundColor Green
    } catch {
        Write-Host "  AVISO: configura el datasource manualmente en Grafana > Connections" -ForegroundColor Yellow
        Write-Host "  URL: http://localhost:9091" -ForegroundColor Yellow
    }
}

function Import-GrafanaDashboard([string]$jsonPath) {
    if (-not (Test-Path $jsonPath)) {
        Write-Host "  AVISO: dashboard no encontrado: $jsonPath" -ForegroundColor Yellow
        return
    }
    $pyCode = @"
import base64
import json
import pathlib
import sys
import urllib.error
import urllib.request

json_path = pathlib.Path(sys.argv[1])
grafana_url = sys.argv[2].rstrip("/")
auth_header = sys.argv[3]

dashboard = json.loads(json_path.read_text(encoding="utf-8"))
dashboard["id"] = None
payload = json.dumps({
    "dashboard": dashboard,
    "folderId": 0,
    "overwrite": True,
}).encode("utf-8")

req = urllib.request.Request(
    f"{grafana_url}/api/dashboards/db",
    data=payload,
    method="POST",
    headers={
        "Authorization": auth_header,
        "Content-Type": "application/json",
    },
)

try:
    with urllib.request.urlopen(req, timeout=20) as resp:
        print(resp.read().decode("utf-8", errors="ignore"))
except urllib.error.HTTPError as exc:
    body = exc.read().decode("utf-8", errors="ignore")
    raise SystemExit(f"HTTP {exc.code}: {body}")
"@
    $authHeader = "Basic $creds"
    $result = $pyCode | & $VenvPy - $jsonPath $GrafanaUrl $authHeader 2>&1
    if ($LASTEXITCODE -ne 0) {
        throw ($result | Out-String).Trim()
    }
}

try {
    Import-GrafanaDashboard "$Root\grafana\dashboards\atlas.json"
    Import-GrafanaDashboard "$Root\grafana\dashboards\atlas_pro_2026.json"
    Write-Host "  Dashboards importados/actualizados OK." -ForegroundColor Green
} catch {
    Write-Host "  AVISO: no se pudieron importar dashboards actuales." -ForegroundColor Yellow
    Write-Host "  Error: $($_.Exception.Message)" -ForegroundColor Yellow
}

# ── Resumen ───────────────────────────────────────────────────────
Write-Host ""
Write-Host "============================================" -ForegroundColor Green
Write-Host "  ATLAS Monitoring Stack LISTO" -ForegroundColor Green
Write-Host "============================================" -ForegroundColor Green
Write-Host "  Metrics Python : http://localhost:9090/metrics"
Write-Host "  Prometheus UI  : http://localhost:9091"
Write-Host "  Grafana        : $GrafanaUrl"
Write-Host "  Login/API      : admin / atlas2026 (acceso visual anonimo activo)"
Write-Host "============================================" -ForegroundColor Green
Start-Process $GrafanaUrl
