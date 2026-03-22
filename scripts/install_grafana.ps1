# ATLAS — Instala Grafana en Windows con puerto 3002
# Ejecutar como Administrador

$ErrorActionPreference = "Stop"
$GrafanaVersion = "10.4.2"
$GrafanaPort = 3002
$Installer = "$env:TEMP\grafana-$GrafanaVersion.windows-amd64.msi"

# ── [1] Descargar ─────────────────────────────────────────────────
Write-Host "[1/4] Descargando Grafana $GrafanaVersion..." -ForegroundColor Cyan
$url = "https://dl.grafana.com/oss/release/grafana-$GrafanaVersion.windows-amd64.msi"
Invoke-WebRequest -Uri $url -OutFile $Installer -UseBasicParsing

# ── [2] Instalar ──────────────────────────────────────────────────
Write-Host "[2/4] Instalando (silencioso)..." -ForegroundColor Cyan
$msi = Start-Process msiexec.exe -ArgumentList "/i `"$Installer`" /qn /l*v `"$env:TEMP\grafana_install.log`"" -Wait -PassThru
if ($msi.ExitCode -ne 0) {
    Write-Host "ERROR MSI exit code: $($msi.ExitCode). Log: $env:TEMP\grafana_install.log" -ForegroundColor Red
    exit 1
}

# ── Detectar directorio real de instalacion ───────────────────────
$candidates = @(
    "C:\Program Files\GrafanaLabs\grafana",
    "C:\Program Files (x86)\GrafanaLabs\grafana",
    "C:\GrafanaLabs\grafana"
)
$InstallDir = $null
foreach ($c in $candidates) {
    if (Test-Path "$c\bin\grafana-server.exe") { $InstallDir = $c; break }
}
if (-not $InstallDir) {
    # Buscar en todo Program Files
    $found = Get-ChildItem "C:\Program Files" -Filter "grafana-server.exe" -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($found) { $InstallDir = $found.DirectoryName | Split-Path }
}
if (-not $InstallDir) {
    Write-Host "ERROR: no se encontro grafana-server.exe tras la instalacion." -ForegroundColor Red
    Write-Host "Revisa: $env:TEMP\grafana_install.log"
    exit 1
}
Write-Host "  Instalado en: $InstallDir" -ForegroundColor Gray

# ── [3] Configurar puerto 3002 ────────────────────────────────────
Write-Host "[3/4] Configurando puerto $GrafanaPort..." -ForegroundColor Cyan

$confDir = "$InstallDir\conf"
New-Item -ItemType Directory -Path $confDir -Force | Out-Null

$customIni = "$confDir\custom.ini"
@"
[server]
http_port = $GrafanaPort

[security]
admin_user = admin
admin_password = atlas2026

[users]
allow_sign_up = false

[dashboards]
default_home_dashboard_path = C:\ATLAS_PUSH\grafana\dashboards\atlas_pro_2026.json

[paths]
provisioning = $confDir\provisioning
"@ | Set-Content $customIni -Encoding UTF8

# Copiar provisioning
$provDest = "$confDir\provisioning"
New-Item -ItemType Directory -Path "$provDest\datasources" -Force | Out-Null
New-Item -ItemType Directory -Path "$provDest\dashboards"  -Force | Out-Null
Copy-Item "C:\ATLAS_PUSH\grafana\provisioning\datasources\atlas.yaml" "$provDest\datasources\atlas.yaml" -Force
Copy-Item "C:\ATLAS_PUSH\grafana\provisioning\dashboards\atlas.yaml"  "$provDest\dashboards\atlas.yaml"  -Force

Write-Host "  custom.ini escrito en: $customIni" -ForegroundColor Gray

# ── [4] Iniciar servicio ──────────────────────────────────────────
Write-Host "[4/4] Iniciando servicio Grafana..." -ForegroundColor Cyan
Start-Service -Name Grafana -ErrorAction SilentlyContinue
Start-Sleep -Seconds 6

$svc = Get-Service -Name Grafana -ErrorAction SilentlyContinue
if ($svc -and $svc.Status -eq "Running") {
    Write-Host ""
    Write-Host "Grafana corriendo en http://localhost:$GrafanaPort" -ForegroundColor Green
    Write-Host "Usuario: admin  |  Password: atlas2026" -ForegroundColor Green
    Start-Process "http://localhost:$GrafanaPort"
} else {
    Write-Host "ERROR: servicio Grafana no arranco. Estado: $($svc.Status)" -ForegroundColor Red
    Write-Host "Revisa el log de Grafana en: $InstallDir\data\log\grafana.log"
}
