param(
    [int]$IntervalSeconds = 60
)

$ErrorActionPreference = "Continue"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$runner = (Resolve-Path (Join-Path $PSScriptRoot "panaderia_tunnel_autoheal.ps1")).Path
$logPath = Join-Path $repoRoot "logs\panaderia_tunnel_autoheal_loop.log"

if ($IntervalSeconds -lt 30) {
    $IntervalSeconds = 30
}

while ($true) {
    try {
        $stamp = (Get-Date).ToString("yyyy-MM-dd HH:mm:ss")
        Add-Content -Path $logPath -Encoding UTF8 -Value "[$stamp] autoheal tick"
        & powershell -NoProfile -ExecutionPolicy Bypass -File $runner | ForEach-Object {
            Add-Content -Path $logPath -Encoding UTF8 -Value "[$stamp] $_"
        }
    } catch {
        $errStamp = (Get-Date).ToString("yyyy-MM-dd HH:mm:ss")
        Add-Content -Path $logPath -Encoding UTF8 -Value "[$errStamp] ERROR $($_.Exception.Message)"
    }
    Start-Sleep -Seconds $IntervalSeconds
}
