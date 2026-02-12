# Idempotent: check Tailscale installed (no login).
$path = $env:TAILSCALE_EXE_PATH
if (-not $path) { $path = "C:\Program Files\Tailscale\tailscale.exe" }
if (Test-Path $path) {
    Write-Host "Tailscale found at $path"
    & $path status 2>&1
    exit 0
}
try {
    $p = Get-Command tailscale -ErrorAction SilentlyContinue
    if ($p) { Write-Host "Tailscale in PATH: $($p.Source)"; exit 0 }
} catch {}
Write-Host "Install Tailscale from https://tailscale.com/download and set TAILSCALE_EXE_PATH if needed."
exit 0
