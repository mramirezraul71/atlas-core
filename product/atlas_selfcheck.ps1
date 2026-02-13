# Atlas Product Pack - Selfcheck (call GET /support/selfcheck or run local checks)
param([string]$BaseUrl = "http://127.0.0.1:8791")
$ErrorActionPreference = "Stop"
Write-Host "Selfcheck: $BaseUrl" -ForegroundColor Cyan
try {
    $r = Invoke-RestMethod -Uri "$BaseUrl/support/selfcheck" -TimeoutSec 15
    $ok = $r.ok
    $problems = $r.data.problems
    $suggestions = $r.data.suggestions
    foreach ($p in $problems) {
        $sev = $p.severity
        $color = if ($sev -eq "critical") { "Red" } elseif ($sev -eq "warning") { "Yellow" } else { "Gray" }
        Write-Host "  [$sev] $($p.message)" -ForegroundColor $color
        if ($p.suggestion) { Write-Host "    -> $($p.suggestion)" -ForegroundColor Gray }
    }
    foreach ($s in $suggestions) { Write-Host "  + $s" -ForegroundColor Green }
    if ($ok) { Write-Host "Selfcheck OK." -ForegroundColor Green; exit 0 }
    else { Write-Host "Selfcheck found issues." -ForegroundColor Red; exit 1 }
} catch {
    Write-Host "Selfcheck FAIL: $($_.Exception.Message)" -ForegroundColor Red
    exit 1
}
