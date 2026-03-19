param(
    [int]$IntervalSeconds = 60
)

$ErrorActionPreference = "Continue"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$runner = (Resolve-Path (Join-Path $PSScriptRoot "panaderia_tunnel_autoheal.ps1")).Path
$logPath = Join-Path $repoRoot "logs\panaderia_tunnel_autoheal_loop.log"
$lockPath = Join-Path $repoRoot "logs\panaderia_tunnel_autoheal_loop.lock"
$script:LockHandle = $null

if ($IntervalSeconds -lt 30) {
    $IntervalSeconds = 30
}

function Acquire-Lock {
    try {
        $script:LockHandle = [System.IO.File]::Open(
            $lockPath,
            [System.IO.FileMode]::OpenOrCreate,
            [System.IO.FileAccess]::ReadWrite,
            [System.IO.FileShare]::None
        )
        return $true
    } catch {
        return $false
    }
}

function Release-Lock {
    try {
        if ($script:LockHandle) {
            $script:LockHandle.Close()
            $script:LockHandle.Dispose()
            $script:LockHandle = $null
        }
    } catch {}
}

if (-not (Acquire-Lock)) {
    $stamp = (Get-Date).ToString("yyyy-MM-dd HH:mm:ss")
    Add-Content -Path $logPath -Encoding UTF8 -Value "[$stamp] duplicate_loop_detected pid=$PID"
    exit 0
}

try {
    while ($true) {
        try {
            $stamp = (Get-Date).ToString("yyyy-MM-dd HH:mm:ss")
            Add-Content -Path $logPath -Encoding UTF8 -Value "[$stamp] autoheal tick pid=$PID"
            & $runner 2>&1 | ForEach-Object {
                $line = "$_"
                if (-not [string]::IsNullOrWhiteSpace($line)) {
                    Add-Content -Path $logPath -Encoding UTF8 -Value "[$stamp] $line"
                }
            }
        } catch {
            $errStamp = (Get-Date).ToString("yyyy-MM-dd HH:mm:ss")
            Add-Content -Path $logPath -Encoding UTF8 -Value "[$errStamp] ERROR $($_.Exception.Message)"
        }
        Start-Sleep -Seconds $IntervalSeconds
    }
} finally {
    Release-Lock
}
