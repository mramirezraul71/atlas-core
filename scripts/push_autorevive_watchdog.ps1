param(
  [string]$HealthUrl = "http://127.0.0.1:8791/health",
  [int]$IntervalSec = 15,
  [int]$FailThreshold = 3,
  [int]$RestartCooldownSec = 180,
  [int]$HealthTimeoutSec = 60
)

$ErrorActionPreference = "Continue"
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$RestartScript = Join-Path $PSScriptRoot "restart_push_from_api.ps1"
$LogDir = Join-Path $RepoRoot "logs"
$LogFile = Join-Path $LogDir "push_autorevive_watchdog.log"
$StateFile = Join-Path $LogDir "push_autorevive_state.json"
$LockFile = Join-Path $LogDir "push_autorevive_watchdog.lock"
$script:WatchdogLock = $null
$script:WatchdogMutex = $null
$script:WatchdogMutexOwner = $false

New-Item -ItemType Directory -Path $LogDir -Force | Out-Null

function Acquire-WatchdogLock {
  try {
    $script:WatchdogLock = [System.IO.File]::Open(
      $LockFile,
      [System.IO.FileMode]::OpenOrCreate,
      [System.IO.FileAccess]::ReadWrite,
      [System.IO.FileShare]::None
    )
    return $true
  } catch {
    return $false
  }
}

function Acquire-WatchdogMutex {
  try {
    $name = "Global\\ATLAS_PUSH_AUTOREVIVE_WATCHDOG"
    $script:WatchdogMutex = New-Object System.Threading.Mutex($false, $name)
    $acquired = $script:WatchdogMutex.WaitOne(0, $false)
    if (-not $acquired) {
      $script:WatchdogMutexOwner = $false
      return $false
    }
    $script:WatchdogMutexOwner = $true
    return $true
  } catch {
    return $false
  }
}

function Release-WatchdogLock {
  try {
    if ($script:WatchdogLock) {
      $script:WatchdogLock.Close()
      $script:WatchdogLock.Dispose()
      $script:WatchdogLock = $null
    }
  } catch {}
}

function Release-WatchdogMutex {
  try {
    if ($script:WatchdogMutex -and $script:WatchdogMutexOwner) {
      $script:WatchdogMutex.ReleaseMutex() | Out-Null
    }
  } catch {}
  try {
    if ($script:WatchdogMutex) {
      $script:WatchdogMutex.Dispose()
      $script:WatchdogMutex = $null
    }
    $script:WatchdogMutexOwner = $false
  } catch {}
}

function Write-Log([string]$msg) {
  $ts = (Get-Date).ToString("o")
  "$ts PUSH_AUTOREVIVE $msg" | Out-File -FilePath $LogFile -Append -Encoding utf8
}

function Save-State([hashtable]$state) {
  ($state | ConvertTo-Json -Depth 6) | Set-Content -Path $StateFile -Encoding UTF8
}

function Load-State() {
  if (-not (Test-Path $StateFile)) {
    return @{
      started_at = (Get-Date).ToString("o")
      checks = 0
      fail_streak = 0
      last_ok_at = $null
      last_error = $null
      last_restart_at = $null
      restarts = 0
    }
  }
  try {
    return (Get-Content $StateFile -Raw | ConvertFrom-Json -AsHashtable)
  } catch {
    return @{
      started_at = (Get-Date).ToString("o")
      checks = 0
      fail_streak = 0
      last_ok_at = $null
      last_error = "state_parse_error"
      last_restart_at = $null
      restarts = 0
    }
  }
}

if (-not (Test-Path $RestartScript)) {
  Write-Log "FATAL restart script missing: $RestartScript"
  exit 1
}

$mutexAcquired = Acquire-WatchdogMutex
if (-not $mutexAcquired) {
  Write-Log "watchdog_mutex_busy -> continuing_with_file_lock_guard"
}

if (-not (Acquire-WatchdogLock)) {
  Write-Log "watchdog_duplicate_instance_detected -> exiting"
  if ($mutexAcquired) { Release-WatchdogMutex }
  exit 0
}

$state = Load-State
Write-Log "watchdog_started interval=${IntervalSec}s threshold=$FailThreshold cooldown=${RestartCooldownSec}s"
Save-State $state

while ($true) {
  $state.checks = [int]$state.checks + 1
  $ok = $false
  $errMsg = ""
  try {
    $resp = Invoke-WebRequest -Uri $HealthUrl -UseBasicParsing -TimeoutSec $HealthTimeoutSec -ErrorAction Stop
    if ($resp.StatusCode -eq 200) { $ok = $true }
    else { $errMsg = "status_$($resp.StatusCode)" }
  } catch {
    $errMsg = $_.Exception.Message
  }

  if ($ok) {
    $state.fail_streak = 0
    $state.last_ok_at = (Get-Date).ToString("o")
    $state.last_error = $null
    Save-State $state
    Start-Sleep -Seconds $IntervalSec
    continue
  }

  $state.fail_streak = [int]$state.fail_streak + 1
  $state.last_error = $errMsg
  Save-State $state
  Write-Log "health_fail streak=$($state.fail_streak) error=$errMsg"

  if ([int]$state.fail_streak -ge $FailThreshold) {
    $canRestart = $true
    if ($state.last_restart_at) {
      try {
        $last = [DateTime]::Parse($state.last_restart_at)
        $elapsed = ((Get-Date) - $last).TotalSeconds
        if ($elapsed -lt $RestartCooldownSec) {
          $canRestart = $false
          Write-Log "restart_skipped cooldown_remaining=$([int]($RestartCooldownSec-$elapsed))s"
        }
      } catch {}
    }

    if ($canRestart) {
      Write-Log "restart_triggered"
      try {
        & powershell -NoProfile -ExecutionPolicy Bypass -File $RestartScript 2>&1 | Out-File -FilePath $LogFile -Append -Encoding utf8
        $rc = $LASTEXITCODE
        if ($rc -ne 0) {
          throw "restart_script_exit_code_$rc"
        }
        $state.restarts = [int]$state.restarts + 1
        $state.last_restart_at = (Get-Date).ToString("o")
        $state.fail_streak = 0
        $state.last_error = $null
        Save-State $state
        Write-Log "restart_done"
      } catch {
        $state.last_error = "restart_error: $($_.Exception.Message)"
        Save-State $state
        Write-Log "restart_error $($_.Exception.Message)"
      }
    }
  }

  Start-Sleep -Seconds $IntervalSec
}

Release-WatchdogLock
Release-WatchdogMutex
