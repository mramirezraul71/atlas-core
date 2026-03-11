param(
    [int]$IntervalSeconds = 900
)

$ErrorActionPreference = "Continue"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$runner = (Resolve-Path (Join-Path $PSScriptRoot "run_cloudflare_ns_monitor.ps1")).Path
$logPath = Join-Path $repoRoot "logs\cloudflare_ns_monitor_loop.log"

if ($IntervalSeconds -lt 60) {
    $IntervalSeconds = 60
}

while ($true) {
    try {
        $stamp = (Get-Date).ToString("yyyy-MM-dd HH:mm:ss")
        Add-Content -Path $logPath -Encoding UTF8 -Value "[$stamp] monitor tick"
        & powershell -NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File $runner | ForEach-Object {
            Add-Content -Path $logPath -Encoding UTF8 -Value "[$stamp] $_"
        }
    } catch {
        $errStamp = (Get-Date).ToString("yyyy-MM-dd HH:mm:ss")
        Add-Content -Path $logPath -Encoding UTF8 -Value "[$errStamp] ERROR $($_.Exception.Message)"
    }
    Start-Sleep -Seconds $IntervalSeconds
}
