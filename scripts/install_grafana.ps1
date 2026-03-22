# ATLAS — Instala Grafana en Windows con puerto 3002
# Ejecutar como Administrador

$ErrorActionPreference = "Stop"
$GrafanaVersion = "10.4.2"
$GrafanaPort = 3002
$InstallDir = "C:\Program Files\GrafanaLabs\grafana"
$Installer = "$env:TEMP\grafana-$GrafanaVersion.windows-amd64.msi"

Write-Host "[1/4] Descargando Grafana $GrafanaVersion..." -ForegroundColor Cyan
$url = "https://dl.grafana.com/oss/release/grafana-$GrafanaVersion.windows-amd64.msi"
Invoke-WebRequest -Uri $url -OutFile $Installer -UseBasicParsing

Write-Host "[2/4] Instalando (silencioso)..." -ForegroundColor Cyan
Start-Process msiexec.exe -ArgumentList "/i `"$Installer`" /qn" -Wait

Write-Host "[3/4] Configurando puerto $GrafanaPort..." -ForegroundColor Cyan
$customIni = "$InstallDir\conf\custom.ini"
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
"@ | Set-Content $customIni -Encoding UTF8

# Copiar provisioning
$provDest = "$InstallDir\conf\provisioning"
Copy-Item "C:\ATLAS_PUSH\grafana\provisioning\datasources" "$provDest\datasources" -Recurse -Force
Copy-Item "C:\ATLAS_PUSH\grafana\provisioning\dashboards"  "$provDest\dashboards"  -Recurse -Force

# Ajustar URL del datasource para instalacion local (no Docker)
$dsFile = "$provDest\datasources\atlas.yaml"
(Get-Content $dsFile) -replace "http://prometheus:9090", "http://localhost:9090" |
    Set-Content $dsFile -Encoding UTF8

Write-Host "[4/4] Iniciando servicio Grafana..." -ForegroundColor Cyan
Start-Service -Name Grafana
Start-Sleep -Seconds 5

$status = (Get-Service -Name Grafana).Status
Write-Host ""
if ($status -eq "Running") {
    Write-Host "Grafana corriendo en http://localhost:$GrafanaPort" -ForegroundColor Green
    Write-Host "Usuario: admin  |  Password: atlas2026" -ForegroundColor Green
    Start-Process "http://localhost:$GrafanaPort"
} else {
    Write-Host "ERROR: servicio en estado $status" -ForegroundColor Red
}
