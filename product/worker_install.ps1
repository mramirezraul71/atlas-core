# Atlas Product Pack - Worker (light) install: minimal node that connects to HQ
param([string]$TargetDir = "C:\AtlasPush", [string]$HqUrl = "")
$ErrorActionPreference = "Stop"
$root = if ($TargetDir) { (Resolve-Path $TargetDir -ErrorAction SilentlyContinue).Path } else { "C:\AtlasPush" }
if (-not $root) { $root = "C:\AtlasPush"; New-Item -ItemType Directory -Force -Path $root | Out-Null }
Write-Host "Worker install: $root" -ForegroundColor Cyan
# Reuse main installer but set WORKER_ONLY=true in env
$envFile = Join-Path $root "config\atlas.env"
$configDir = Join-Path $root "config"
New-Item -ItemType Directory -Force -Path $configDir | Out-Null
if (-not (Test-Path $envFile)) {
    $example = Join-Path $root "config\atlas.env.example"
    if (Test-Path $example) { Copy-Item $example $envFile }
}
Add-Content $envFile "`nWORKER_ONLY=true"
Add-Content $envFile "SCHED_ENABLED=true"
Add-Content $envFile "CI_ENABLED=false"
Add-Content $envFile "GOVERNED_AUTONOMY_ENABLED=false"
Add-Content $envFile "METALEARN_ENABLED=false"
if ($HqUrl) { Add-Content $envFile "CLUSTER_HQ_URL=$HqUrl" }
Write-Host "Worker config written. Run worker_start.ps1 to start." -ForegroundColor Green
