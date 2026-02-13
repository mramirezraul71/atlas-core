# Atlas Product Pack - Backup (config + logs + snapshots to ZIP)
param([string]$TargetDir = "C:\AtlasPush", [string]$OutZip = "")
$ErrorActionPreference = "Stop"
$root = (Resolve-Path $TargetDir -ErrorAction Stop).Path
$ts = Get-Date -Format "yyyyMMdd_HHmmss"
$OutZip = if ($OutZip) { $OutZip } else { Join-Path $root "..\atlas_backup_$ts.zip" }
$tempDir = Join-Path $env:TEMP "atlas_backup_$ts"
New-Item -ItemType Directory -Force -Path $tempDir | Out-Null
Copy-Item (Join-Path $root "config\*") $tempDir -Recurse -Force -ErrorAction SilentlyContinue
$logs = Join-Path $root "logs"
if (Test-Path $logs) { Copy-Item $logs (Join-Path $tempDir "logs") -Recurse -Force -ErrorAction SilentlyContinue }
$snap = Join-Path $root "snapshots"
if (Test-Path $snap) { Copy-Item $snap (Join-Path $tempDir "snapshots") -Recurse -Force -ErrorAction SilentlyContinue }
Compress-Archive -Path "$tempDir\*" -DestinationPath $OutZip -Force
Remove-Item $tempDir -Recurse -Force -ErrorAction SilentlyContinue
Write-Host "Backup: $OutZip" -ForegroundColor Green
