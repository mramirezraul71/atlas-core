# Remove ATLAS Windows Service (requires admin).
param(
  [string]$ServiceName = "ATLAS_PUSH"
)

$ErrorActionPreference = "Stop"
$s = Get-Service -Name $ServiceName -ErrorAction SilentlyContinue
if (-not $s) { Write-Host "Servicio $ServiceName no existe." -ForegroundColor Yellow; exit 0 }
if ($s.Status -eq "Running") { Stop-Service -Name $ServiceName -Force }
sc.exe delete $ServiceName
Write-Host "Servicio $ServiceName eliminado." -ForegroundColor Green
