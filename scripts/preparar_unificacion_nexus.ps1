# Prepara la estructura de carpetas para unificar ATLAS NEXUS en este repo (atlas-core).
# No copia código; solo crea directorios y muestra el siguiente paso.
# Uso: .\scripts\preparar_unificacion_nexus.ps1

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $PSCommandPath)

$nexusAtlas = Join-Path $root "nexus\atlas_nexus"
$nexusRobot = Join-Path $root "nexus\atlas_nexus_robot\backend"

New-Item -ItemType Directory -Force -Path $nexusAtlas | Out-Null
New-Item -ItemType Directory -Force -Path $nexusRobot | Out-Null

Write-Host "Estructura preparada en:" -ForegroundColor Green
Write-Host "  $nexusAtlas"
Write-Host "  $nexusRobot"
Write-Host ""
Write-Host "Siguiente paso: copiar el codigo de ATLAS NEXUS segun nexus\INTEGRAR_AQUI.md" -ForegroundColor Cyan
Write-Host "Luego en config\atlas.env configurar:" -ForegroundColor Cyan
Write-Host "  NEXUS_ATLAS_PATH=$nexusAtlas"
Write-Host "  NEXUS_ROBOT_PATH=$nexusRobot"
