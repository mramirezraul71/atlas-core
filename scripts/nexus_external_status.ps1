param(
    [string]$Url = $(if ($env:NEXUS_BASE_URL) { ($env:NEXUS_BASE_URL.TrimEnd('/')) + '/health' } else { 'http://127.0.0.1:8000/health' }),
    [int]$TimeoutSec = 3
)

$ErrorActionPreference = 'Stop'

try {
    $resp = Invoke-WebRequest -Uri $Url -Method GET -TimeoutSec $TimeoutSec -UseBasicParsing
    if ($resp.StatusCode -eq 200) {
        Write-Output "NEXUS_EXTERNAL_OK url=$Url status=200"
        if ($resp.Content) { Write-Output $resp.Content }
        exit 0
    }
    Write-Output "NEXUS_EXTERNAL_WARN url=$Url status=$($resp.StatusCode)"
    if ($resp.Content) { Write-Output $resp.Content }
    exit 0
} catch {
    Write-Output "NEXUS_EXTERNAL_WARN url=$Url error=$($_.Exception.Message)"
    exit 0
}
