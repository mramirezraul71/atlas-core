param(
  [Parameter(Mandatory = $true)][string]$RepoRoot,
  [Parameter(Mandatory = $true)][string]$JobFile,
  [Parameter(Mandatory = $true)][string]$ItemId,
  [Parameter(Mandatory = $true)][string]$InstallMethod,
  [Parameter(Mandatory = $true)][string]$InstallTarget
)

$ErrorActionPreference = "Stop"
$PyExe = Join-Path $RepoRoot ".venv\\Scripts\\python.exe"
if (-not (Test-Path $PyExe)) { $PyExe = "python" }

function _WriteJob([hashtable]$obj) {
  $dir = Split-Path -Parent $JobFile
  if (-not (Test-Path $dir)) {
    New-Item -ItemType Directory -Path $dir -Force | Out-Null
  }
  ($obj | ConvertTo-Json -Depth 14) | Set-Content -Path $JobFile -Encoding UTF8
}

function Invoke-CapturedProcess {
  param(
    [Parameter(Mandatory = $true)][string]$Exe,
    [string[]]$Args = @(),
    [int]$TimeoutMs = 180000
  )
  $tmpOut = [System.IO.Path]::GetTempFileName()
  $tmpErr = [System.IO.Path]::GetTempFileName()
  $exitCode = -1
  $timedOut = $false
  $output = ""
  try {
    $p = Start-Process -FilePath $Exe -ArgumentList $Args -NoNewWindow -PassThru -RedirectStandardOutput $tmpOut -RedirectStandardError $tmpErr
    $finished = $p.WaitForExit($TimeoutMs)
    if (-not $finished) {
      $timedOut = $true
      try { $p.Kill() } catch {}
      $exitCode = 124
    } else {
      $exitCode = [int]$p.ExitCode
    }
    if (Test-Path $tmpOut) {
      $txt = Get-Content -Path $tmpOut -Raw -ErrorAction SilentlyContinue
      if ($txt) { $output = ($output + "`n" + $txt).Trim() }
    }
    if (Test-Path $tmpErr) {
      $txt = Get-Content -Path $tmpErr -Raw -ErrorAction SilentlyContinue
      if ($txt) { $output = ($output + "`n" + $txt).Trim() }
    }
  } finally {
    Remove-Item -Path $tmpOut -Force -ErrorAction SilentlyContinue
    Remove-Item -Path $tmpErr -Force -ErrorAction SilentlyContinue
  }
  return @{
    exit_code = $exitCode
    timed_out = $timedOut
    output = $output
  }
}

function Invoke-WingetApply {
  param([string]$PackageId)
  $attempts = @(
    @("upgrade","--id",$PackageId,"--source","winget","--disable-interactivity","--silent","--accept-package-agreements","--accept-source-agreements"),
    @("upgrade","-e","--id",$PackageId,"--source","winget","--disable-interactivity","--silent","--accept-package-agreements","--accept-source-agreements"),
    @("install","-e","--id",$PackageId,"--source","winget","--disable-interactivity","--silent","--accept-package-agreements","--accept-source-agreements")
  )
  foreach ($args in $attempts) {
    $r = Invoke-CapturedProcess -Exe "winget" -Args $args -TimeoutMs 240000
    $raw = [string]($r.output | Out-String)
    $low = $raw.ToLowerInvariant()
    $tail = ($raw -split "`r?`n" | Where-Object { $_.Trim() } | Select-Object -Last 1)
    $noUpdate = $low.Contains("no newer package versions are available") -or
      $low.Contains("already installed") -or
      $low.Contains("no hay versiones") -or
      $low.Contains("no applicable update found")
    if (([int]$r.exit_code -eq 0 -and -not [bool]$r.timed_out) -or $noUpdate) {
      return @{
        ok = $true
        command = ("winget " + ($args -join " "))
        tail = $tail
      }
    }
  }
  return @{
    ok = $false
    command = ("winget apply " + $PackageId)
    tail = "winget_failed_all_attempts"
  }
}

_WriteJob @{
  ok = $true
  item_id = $ItemId
  status = "running"
  started_at = (Get-Date).ToString("o")
  install_method = $InstallMethod
  install_target = $InstallTarget
  total = 5
  done = 0
  failed = 0
  steps = @()
}

try {
  $steps = @()
  $method = $InstallMethod.ToLowerInvariant()

  $steps += @{ step = "preflight"; ok = $true; method = $method; target = $InstallTarget }
  _WriteJob @{
    ok = $true; item_id = $ItemId; status = "running"; started_at = (Get-Date).ToString("o")
    install_method = $InstallMethod; install_target = $InstallTarget
    total = 5; done = 1; failed = 0; steps = $steps
  }

  $null = Invoke-WebRequest -Uri "https://www.google.com" -UseBasicParsing -TimeoutSec 8 -ErrorAction Stop
  $steps += @{ step = "network_probe"; ok = $true }
  _WriteJob @{
    ok = $true; item_id = $ItemId; status = "running"; started_at = (Get-Date).ToString("o")
    install_method = $InstallMethod; install_target = $InstallTarget
    total = 5; done = 2; failed = 0; steps = $steps
  }

  $applyOk = $false
  $applyCmd = ""
  $applyTail = ""
  if ($method -eq "winget") {
    $w = Invoke-WingetApply -PackageId $InstallTarget
    $applyOk = [bool]$w.ok
    $applyCmd = [string]$w.command
    $applyTail = [string]$w.tail
  } elseif ($method -eq "pip") {
    $r = Invoke-CapturedProcess -Exe $PyExe -Args @("-m","pip","install","-U",$InstallTarget) -TimeoutMs 240000
    $raw = [string]($r.output | Out-String)
    $applyOk = ([int]$r.exit_code -eq 0 -and -not [bool]$r.timed_out)
    $applyCmd = "$PyExe -m pip install -U $InstallTarget"
    $applyTail = ($raw -split "`r?`n" | Where-Object { $_.Trim() } | Select-Object -Last 1)
  } elseif ($method -eq "npm") {
    $r = Invoke-CapturedProcess -Exe "npm" -Args @("install","-g",$InstallTarget) -TimeoutMs 240000
    $raw = [string]($r.output | Out-String)
    $applyOk = ([int]$r.exit_code -eq 0 -and -not [bool]$r.timed_out)
    $applyCmd = "npm install -g $InstallTarget"
    $applyTail = ($raw -split "`r?`n" | Where-Object { $_.Trim() } | Select-Object -Last 1)
  } else {
    throw "unsupported_install_method:$InstallMethod"
  }
  $steps += @{ step = "apply"; ok = $applyOk; command = $applyCmd; tail = $applyTail }
  if (-not $applyOk) { throw "apply_failed:$applyTail" }
  _WriteJob @{
    ok = $true; item_id = $ItemId; status = "running"; started_at = (Get-Date).ToString("o")
    install_method = $InstallMethod; install_target = $InstallTarget
    total = 5; done = 3; failed = 0; steps = $steps
  }

  $verifyOk = $false
  $verifyTail = ""
  if ($method -eq "winget") {
    $r = Invoke-CapturedProcess -Exe "winget" -Args @("list","--id",$InstallTarget,"--source","winget","--disable-interactivity") -TimeoutMs 120000
    $raw = [string]($r.output | Out-String)
    $low = $raw.ToLowerInvariant()
    $verifyOk = (([int]$r.exit_code -eq 0 -and -not [bool]$r.timed_out) -and $low.Contains($InstallTarget.ToLowerInvariant()))
    $verifyTail = ($raw -split "`r?`n" | Where-Object { $_.Trim() } | Select-Object -Last 1)
  } elseif ($method -eq "pip") {
    $r = Invoke-CapturedProcess -Exe $PyExe -Args @("-m","pip","show",$InstallTarget) -TimeoutMs 120000
    $raw = [string]($r.output | Out-String)
    $verifyOk = (([int]$r.exit_code -eq 0 -and -not [bool]$r.timed_out) -and $raw.ToLowerInvariant().Contains("name:"))
    $verifyTail = ($raw -split "`r?`n" | Where-Object { $_.Trim() } | Select-Object -Last 1)
  } elseif ($method -eq "npm") {
    $r = Invoke-CapturedProcess -Exe "npm" -Args @("list","-g",$InstallTarget,"--depth=0") -TimeoutMs 120000
    $raw = [string]($r.output | Out-String)
    $verifyOk = (([int]$r.exit_code -eq 0 -and -not [bool]$r.timed_out) -and $raw.ToLowerInvariant().Contains($InstallTarget.ToLowerInvariant()))
    $verifyTail = ($raw -split "`r?`n" | Where-Object { $_.Trim() } | Select-Object -Last 1)
  }
  $steps += @{ step = "verify"; ok = $verifyOk; tail = $verifyTail }
  if (-not $verifyOk) { throw "verify_failed:$verifyTail" }
  _WriteJob @{
    ok = $true; item_id = $ItemId; status = "running"; started_at = (Get-Date).ToString("o")
    install_method = $InstallMethod; install_target = $InstallTarget
    total = 5; done = 4; failed = 0; steps = $steps
  }

  $supOk = $false
  $supError = ""
  try {
    $payload = @{
      objective = "Integrar software $ItemId en stack ATLAS para mejorar capacidades operativas."
      context = @{
        source = "software_apply_item"
        item_id = $ItemId
        method = $InstallMethod
        target = $InstallTarget
      }
    } | ConvertTo-Json -Depth 8
    $null = Invoke-RestMethod -Uri "http://127.0.0.1:8791/supervisor/advise" -Method Post -ContentType "application/json" -Body $payload -TimeoutSec 45
    $supOk = $true
  } catch {
    $supOk = $false
    $supError = $_.Exception.Message
  }
  $steps += @{ step = "supervisor_adapt"; ok = $supOk; error = $(if($supOk){$null}else{$supError}) }

  _WriteJob @{
    ok = $true
    item_id = $ItemId
    status = "done"
    started_at = (Get-Date).ToString("o")
    finished_at = (Get-Date).ToString("o")
    install_method = $InstallMethod
    install_target = $InstallTarget
    total = 5
    done = 5
    failed = 0
    steps = $steps
  }
}
catch {
  $msg = $_.Exception.Message
  if (-not $msg -or -not $msg.Trim()) { $msg = "software_apply_failed" }
  _WriteJob @{
    ok = $false
    item_id = $ItemId
    status = "failed"
    finished_at = (Get-Date).ToString("o")
    install_method = $InstallMethod
    install_target = $InstallTarget
    total = 5
    done = 4
    failed = 1
    error = $msg
  }
}
