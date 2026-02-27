# Idempotent: ensure cloudflared path exists (no login). Create bin dir if missing.
param([string]$RepoPath = $PSScriptRoot + "\..\..", [string]$CloudflaredPath = "")
$binDir = Join-Path $RepoPath "bin"
if (-not (Test-Path $binDir)) { New-Item -ItemType Directory -Path $binDir -Force | Out-Null }
if ($CloudflaredPath) {
    if (Test-Path $CloudflaredPath) { Write-Host "cloudflared already at $CloudflaredPath"; exit 0 }
}
$target = Join-Path $binDir "cloudflared.exe"
if (Test-Path $target) { Write-Host "cloudflared.exe already present at $target"; exit 0 }
Write-Host "Place cloudflared.exe in $binDir or set CLOUDFLARE_CLOUDFLARED_PATH. Download from https://developers.cloudflare.com/cloudflare-one/connections/connect-apps/install-and-setup/installation/"
exit 0
