# Atlas Product Pack - Restore from backup ZIP (config + logs + snapshots)
param([string]$ZipPath, [string]$TargetDir = "C:\AtlasPush")
$ErrorActionPreference = "Stop"
if (-not $ZipPath -or -not (Test-Path $ZipPath)) { Write-Host "Usage: .\atlas_restore.ps1 -ZipPath <path>" -ForegroundColor Red; exit 1 }
$root = (Resolve-Path $TargetDir -ErrorAction Stop).Path
$tempDir = Join-Path $env:TEMP "atlas_restore_extract"
if (Test-Path $tempDir) { Remove-Item $tempDir -Recurse -Force }
Expand-Archive -Path $ZipPath -DestinationPath $tempDir -Force
$configSrc = Join-Path $tempDir "config"
if (Test-Path $configSrc) { Copy-Item "$configSrc\*" (Join-Path $root "config") -Recurse -Force }
$logsSrc = Join-Path $tempDir "logs"
if (Test-Path $logsSrc) { Copy-Item "$logsSrc\*" (Join-Path $root "logs") -Recurse -Force }
$snapSrc = Join-Path $tempDir "snapshots"
if (Test-Path $snapSrc) { Copy-Item "$snapSrc\*" (Join-Path $root "snapshots") -Recurse -Force }
Remove-Item $tempDir -Recurse -Force -ErrorAction SilentlyContinue
Write-Host "Restored to $root" -ForegroundColor Green
