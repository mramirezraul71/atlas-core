# Call POST /gateway/bootstrap (idempotent). Requires -RepoPath, optional -AtlasPort.
param([Parameter(Mandatory=$true)][string]$RepoPath, [int]$AtlasPort = 8791)
$base = "http://127.0.0.1:$AtlasPort"
try {
    $r = Invoke-RestMethod -Method POST -Uri "$base/gateway/bootstrap" -ContentType "application/json" -Body "{}" -TimeoutSec 60
    $r | ConvertTo-Json -Depth 5
    if ($r.ok) { exit 0 } else { exit 1 }
} catch {
    Write-Error $_.Exception.Message
    exit 1
}
