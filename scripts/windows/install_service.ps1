# Install ATLAS as Windows Service (requires admin). Uses Python launcher.
param(
  [Parameter(Mandatory=$false)][string]$RepoPath = (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent),
  [string]$ServiceName = "ATLAS_PUSH",
  [int]$Port = 8791
)

$ErrorActionPreference = "Stop"
$python = (Join-Path $RepoPath ".venv\Scripts\python.exe")
$launcher = Join-Path $RepoPath "tools\service_launcher.py"
if (-not (Test-Path $python)) { Write-Error "Venv no encontrado en $RepoPath. Ejecuta 01_setup_venv.ps1."; exit 1 }
if (-not (Test-Path $launcher)) { Write-Error "Launcher no encontrado: $launcher"; exit 1 }

$binPath = "`"$python`" `"$launcher`""
$displayName = "ATLAS Cognitive OS"
Write-Host "Instalando servicio: $ServiceName" -ForegroundColor Cyan
Write-Host "  BinPath: $binPath" -ForegroundColor Gray
$existing = Get-Service -Name $ServiceName -ErrorAction SilentlyContinue
if ($existing) {
  Write-Host "Servicio ya existe. Desinstalar primero: .\uninstall_service.ps1 -ServiceName $ServiceName" -ForegroundColor Yellow
  exit 2
}
New-Service -Name $ServiceName -BinaryPathName $binPath -DisplayName $displayName -StartupType Automatic
Write-Host "Servicio instalado. Iniciar: .\start_service.ps1 -ServiceName $ServiceName" -ForegroundColor Green
