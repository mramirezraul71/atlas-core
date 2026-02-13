# Atlas Product Pack - Uninstaller (removes service, optionally keeps data)
param([switch]$KeepData, [string]$TargetDir = "")
$ErrorActionPreference = "Stop"
$root = if ($TargetDir) { $TargetDir } else { "C:\AtlasPush" }
if (-not (Test-Path $root)) { $root = "C:\Program Files\AtlasPush" }
Write-Host "Uninstalling from: $root" -ForegroundColor Cyan
$svc = Get-Service -Name "ATLAS_PUSH" -ErrorAction SilentlyContinue
if ($svc) {
    Stop-Service -Name "ATLAS_PUSH" -Force -ErrorAction SilentlyContinue
    sc.exe delete ATLAS_PUSH 2>$null
    Write-Host "Service removed." -ForegroundColor Green
}
if (-not $KeepData) {
    Remove-Item -Path $root -Recurse -Force -ErrorAction SilentlyContinue
    Write-Host "Removed $root" -ForegroundColor Green
} else {
    Write-Host "Data kept at $root" -ForegroundColor Yellow
}
Write-Host "Uninstall complete." -ForegroundColor Green
