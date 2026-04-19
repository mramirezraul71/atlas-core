# ATLAS - Arranca Prometheus + Grafana para Atlas Quant
# Ejecutar desde C:\ATLAS_PUSH (no requiere Admin)
#
# Topologia operativa:
#   :8795  - Atlas Quant API con /metrics embebido
#   :9090  - Prometheus UI + query API
#   :3002  - Grafana workspace activo

param(
    [int]$QuantPort = 8795,
    [int]$PrometheusPort = 9090,
    [int]$GrafanaPort = 3002,
    [switch]$SkipOpenBrowser
)

$ErrorActionPreference = "Stop"
$Root = "C:\ATLAS_PUSH"
$PromDir = "$Root\tools\prometheus"
$PromExe = "$PromDir\prometheus.exe"
$PromCfg = "$PromDir\prometheus.atlas-local.yml"
$PromVer = "2.51.2"
$GrafanaUrl = "http://localhost:$GrafanaPort"
$GrafanaCfg = "$Root\grafana\local\custom.ini"
$HealthUrl = "http://127.0.0.1:$QuantPort/health"
$MetricsUrl = "http://127.0.0.1:$QuantPort/metrics"

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

function Wait-HttpOk([string]$Url, [int]$Attempts = 20, [int]$DelaySec = 2) {
    for ($i = 0; $i -lt $Attempts; $i++) {
        try {
            Invoke-WebRequest -Uri $Url -UseBasicParsing -TimeoutSec 3 | Out-Null
            return $true
        } catch {
            Start-Sleep -Seconds $DelaySec
        }
    }
    return $false
}

function Test-Listening([int]$Port) {
    return [bool](Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue)
}

function Ensure-GrafanaConfig([string]$Path, [int]$Port) {
    if (-not (Test-Path $Path)) {
        New-Item -ItemType Directory -Path (Split-Path $Path) -Force | Out-Null
        @"
[server]
http_port = $Port
root_url = http://localhost:$Port

[security]
admin_user = admin
admin_password = atlas2026

[users]
allow_sign_up = false

[dashboards]
default_home_dashboard_path = C:\ATLAS_PUSH\grafana\dashboards\atlas_pro_2026.json
"@ | Set-Content -Path $Path -Encoding UTF8
        return
    }

    $content = Get-Content $Path -Raw
    $content = [regex]::Replace($content, '(?m)^http_port\s*=.*$', "http_port = $Port")
    $content = [regex]::Replace($content, '(?m)^root_url\s*=.*$', "root_url = http://localhost:$Port")
    Set-Content -Path $Path -Value $content -Encoding UTF8
}

Write-Host "[1/5] Verificando Atlas Quant en :$QuantPort ..." -ForegroundColor Cyan
if (-not (Wait-HttpOk -Url $HealthUrl -Attempts 8 -DelaySec 2)) {
    throw "Atlas Quant no responde en $HealthUrl"
}
if (-not (Wait-HttpOk -Url $MetricsUrl -Attempts 8 -DelaySec 2)) {
    throw "Atlas Quant no expone /metrics en $MetricsUrl"
}
Write-Host "  Quant OK: $HealthUrl y $MetricsUrl" -ForegroundColor Gray

if (-not (Test-Path $PromExe)) {
    Write-Host "[2/5] Descargando Prometheus $PromVer..." -ForegroundColor Cyan
    $zip = "$env:TEMP\prometheus.zip"
    $url = "https://github.com/prometheus/prometheus/releases/download/v$PromVer/prometheus-$PromVer.windows-amd64.zip"
    Invoke-WebRequest -Uri $url -OutFile $zip -UseBasicParsing
    New-Item -ItemType Directory -Path $PromDir -Force | Out-Null
    Expand-Archive -Path $zip -DestinationPath "$env:TEMP\prom_extract" -Force
    $src = Get-ChildItem "$env:TEMP\prom_extract" -Filter "prometheus-*" | Select-Object -First 1
    Copy-Item "$($src.FullName)\prometheus.exe" $PromExe -Force
    Remove-Item $zip -Force
} else {
    Write-Host "[2/5] Prometheus ya existe." -ForegroundColor Gray
}

@"
global:
  scrape_interval: 5s
  evaluation_interval: 5s

scrape_configs:
  - job_name: atlas-quant
    scrape_interval: 5s
    scrape_timeout: 4s
    static_configs:
      - targets:
          - localhost:$QuantPort
    metrics_path: /metrics
"@ | Set-Content -Path $PromCfg -Encoding UTF8

Write-Host "[3/5] Verificando Prometheus en :$PrometheusPort ..." -ForegroundColor Cyan
if (-not (Test-Listening -Port $PrometheusPort)) {
    Start-Process -FilePath $PromExe `
        -ArgumentList "--config.file=$PromCfg", "--web.listen-address=:$PrometheusPort", "--storage.tsdb.path=$PromDir\data", "--storage.tsdb.retention.time=7d" `
        -WorkingDirectory $PromDir `
        -WindowStyle Minimized | Out-Null
}
if (-not (Wait-HttpOk -Url "http://127.0.0.1:$PrometheusPort/-/ready" -Attempts 15 -DelaySec 1)) {
    throw "Prometheus no respondio en http://127.0.0.1:$PrometheusPort/-/ready"
}
Write-Host "  Prometheus OK en http://localhost:$PrometheusPort" -ForegroundColor Gray

Write-Host "[4/5] Verificando Grafana en :$GrafanaPort ..." -ForegroundColor Cyan
if (-not (Test-Listening -Port $GrafanaPort)) {
    $grafanaHome = Get-GrafanaHome
    if (-not $grafanaHome) {
        throw "No se encontro grafana-server.exe. Instala Grafana antes de ejecutar este script."
    }
    New-Item -ItemType Directory -Path "$Root\grafana\local\data" -Force | Out-Null
    New-Item -ItemType Directory -Path "$Root\grafana\local\logs" -Force | Out-Null
    New-Item -ItemType Directory -Path "$Root\grafana\local\plugins" -Force | Out-Null
    Ensure-GrafanaConfig -Path $GrafanaCfg -Port $GrafanaPort
    Start-Process -FilePath (Join-Path $grafanaHome "bin\grafana-server.exe") `
        -ArgumentList @("--config", $GrafanaCfg, "--homepath", $grafanaHome) `
        -WorkingDirectory $grafanaHome `
        -WindowStyle Minimized | Out-Null
}
if (-not (Wait-HttpOk -Url "$GrafanaUrl/api/health" -Attempts 20 -DelaySec 2)) {
    throw "Grafana no respondio en $GrafanaUrl"
}
Write-Host "  Grafana OK en $GrafanaUrl" -ForegroundColor Gray

Write-Host "[5/5] Configurando datasources e importando dashboards..." -ForegroundColor Cyan
$creds = [Convert]::ToBase64String([Text.Encoding]::ASCII.GetBytes("admin:atlas2026"))
$headers = @{ "Authorization" = "Basic $creds"; "Content-Type" = "application/json" }

function Save-GrafanaDatasource([string]$Name, [string]$Uid, [int]$Id) {
    $payload = @{
        id        = $Id
        uid       = $Uid
        orgId     = 1
        name      = $Name
        type      = "prometheus"
        access    = "proxy"
        url       = "http://localhost:$PrometheusPort"
        isDefault = ($Uid -eq "atlas-prom")
        editable  = $true
        basicAuth = $false
        jsonData  = @{
            httpMethod   = "GET"
            timeInterval = "2s"
        }
    } | ConvertTo-Json -Depth 6 -Compress

    try {
        Invoke-RestMethod -Uri "$GrafanaUrl/api/datasources/$Id" -Method PUT -Headers $headers -Body $payload | Out-Null
    } catch {
        try {
            Invoke-RestMethod -Uri "$GrafanaUrl/api/datasources" -Method POST -Headers $headers -Body $payload | Out-Null
        } catch {
            Write-Host "  AVISO: no se pudo guardar datasource $Name ($Uid)" -ForegroundColor Yellow
        }
    }
}

Save-GrafanaDatasource -Name "atlas-prometheus" -Uid "atlas-prom" -Id 1
Save-GrafanaDatasource -Name "atlas-prometheus-local" -Uid "atlas-prom-local" -Id 4

function Import-GrafanaDashboard([string]$jsonPath) {
    if (-not (Test-Path $jsonPath)) {
        return
    }
    $pyCode = @"
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

with urllib.request.urlopen(req, timeout=20) as resp:
    print(resp.read().decode("utf-8", errors="ignore"))
"@
    $result = $pyCode | & "$Root\venv\Scripts\python.exe" - $jsonPath $GrafanaUrl ("Basic $creds") 2>&1
    if ($LASTEXITCODE -ne 0) {
        throw ($result | Out-String).Trim()
    }
}

Import-GrafanaDashboard "$Root\grafana\dashboards\atlas.json"
Import-GrafanaDashboard "$Root\grafana\dashboards\atlas_pro_2026.json"
Import-GrafanaDashboard "$Root\grafana\dashboards\atlas-options-health.json"
Import-GrafanaDashboard "$Root\grafana\dashboards\atlas-options-signals-intent.json"
Import-GrafanaDashboard "$Root\grafana\dashboards\atlas-options-paper-performance.json"

Write-Host ""
Write-Host "============================================" -ForegroundColor Green
Write-Host "  ATLAS Monitoring Stack LISTO" -ForegroundColor Green
Write-Host "============================================" -ForegroundColor Green
Write-Host "  Quant API      : http://localhost:$QuantPort"
Write-Host "  Quant metrics  : http://localhost:$QuantPort/metrics"
Write-Host "  Prometheus UI  : http://localhost:$PrometheusPort"
Write-Host "  Grafana        : $GrafanaUrl"
Write-Host "  Login/API      : admin / atlas2026"
Write-Host "============================================" -ForegroundColor Green

if (-not $SkipOpenBrowser) {
    Start-Process $GrafanaUrl
}
