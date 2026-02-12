# Idempotent: check gateway status and candidates. Requires -RepoPath and optional -AtlasPort.
param([Parameter(Mandatory=$true)][string]$RepoPath, [int]$AtlasPort = 8791)
$base = "http://127.0.0.1:$AtlasPort"
try {
    $status = Invoke-RestMethod -Method GET -Uri "$base/gateway/status" -TimeoutSec 5
    $status | ConvertTo-Json -Depth 5
    $check = Invoke-RestMethod -Method POST -Uri "$base/gateway/check" -ContentType "application/json" -Body "{}" -TimeoutSec 10
    $check | ConvertTo-Json -Depth 5
} catch {
    Write-Error $_.Exception.Message
    exit 1
}
