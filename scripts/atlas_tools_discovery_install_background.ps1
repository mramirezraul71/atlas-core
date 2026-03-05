param(
  [Parameter(Mandatory = $true)][string]$RepoRoot,
  [Parameter(Mandatory = $true)][string]$JobFile,
  [Parameter(Mandatory = $true)][string]$ToolId,
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
  ($obj | ConvertTo-Json -Depth 12) | Set-Content -Path $JobFile -Encoding UTF8
}

function Invoke-CapturedProcess {
  param(
    [Parameter(Mandatory = $true)][string]$Exe,
    [string[]]$Args = @(),
    [int]$TimeoutMs = 120000
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
      $outText = Get-Content -Path $tmpOut -Raw -ErrorAction SilentlyContinue
      if ($outText) { $output = ($output + "`n" + $outText).Trim() }
    }
    if (Test-Path $tmpErr) {
      $errText = Get-Content -Path $tmpErr -Raw -ErrorAction SilentlyContinue
      if ($errText) { $output = ($output + "`n" + $errText).Trim() }
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

_WriteJob @{
  ok = $true
  tool = $ToolId
  status = "running"
  started_at = (Get-Date).ToString("o")
  install_method = $InstallMethod
  install_target = $InstallTarget
  total = 6
  done = 0
  failed = 0
  steps = @()
}

try {
  $steps = @()

  # Step 1: preflight
  $steps += @{ step = "preflight"; ok = $true }
  _WriteJob @{
    ok = $true; tool = $ToolId; status = "running"; started_at = (Get-Date).ToString("o")
    install_method = $InstallMethod; install_target = $InstallTarget
    total = 6; done = 1; failed = 0; steps = $steps
  }

  # Step 2: network probe
  $probeOk = $false
  try {
    $null = Invoke-WebRequest -Uri "https://www.google.com" -UseBasicParsing -TimeoutSec 8 -ErrorAction Stop
    $probeOk = $true
  } catch {}
  $steps += @{ step = "network_probe"; ok = $probeOk; error = $(if($probeOk){$null}else{"network_unavailable"}) }
  if (-not $probeOk) { throw "network_unavailable" }
  _WriteJob @{
    ok = $true; tool = $ToolId; status = "running"; started_at = (Get-Date).ToString("o")
    install_method = $InstallMethod; install_target = $InstallTarget
    total = 6; done = 2; failed = 0; steps = $steps
  }

  # Step 3: install
  $installCmd = @()
  if ($InstallMethod -eq "winget") {
    $installCmd = @("winget","install","--id",$InstallTarget,"-e","--disable-interactivity","--silent","--accept-package-agreements","--accept-source-agreements")
  } elseif ($InstallMethod -eq "pip") {
    $installCmd = @($PyExe,"-m","pip","install","-U",$InstallTarget)
  } elseif ($InstallMethod -eq "npm") {
    $installCmd = @("npm","install","-g",$InstallTarget)
  } else {
    throw "unsupported_install_method:$InstallMethod"
  }

  $exec = Invoke-CapturedProcess -Exe $installCmd[0] -Args $installCmd[1..($installCmd.Length-1)] -TimeoutMs 180000
  $raw = [string]($exec.output | Out-String)
  $installOk = ([int]$exec.exit_code -eq 0)
  if ($exec.timed_out) {
    $installOk = $false
    $raw = (($raw + "`ninstall_timeout_180s") | Out-String)
  }
  if ((-not $installOk) -and ($InstallMethod -eq "winget")) {
    $rawNorm = ($raw | Out-String).ToLowerInvariant()
    $noUpdate = $rawNorm.Contains("no hay versiones") -or
      $rawNorm.Contains("no newer package versions are available") -or
      $rawNorm.Contains("already installed") -or
      $rawNorm.Contains("no applicable update found")
    if ($noUpdate) { $installOk = $true }
  }
  $line = ($raw -split "`r?`n" | Where-Object { $_.Trim() } | Select-Object -Last 1)
  $steps += @{ step = "install"; ok = $installOk; command = ($installCmd -join " "); tail = $line }
  if (-not $installOk) { throw ("install_failed:" + ($line | Out-String).Trim()) }
  _WriteJob @{
    ok = $true; tool = $ToolId; status = "running"; started_at = (Get-Date).ToString("o")
    install_method = $InstallMethod; install_target = $InstallTarget
    total = 6; done = 3; failed = 0; steps = $steps
  }

  # Step 4: dependency sync (best effort)
  $depOk = $true
  $depTail = ""
  try {
    $dep = Invoke-CapturedProcess -Exe $PyExe -Args @("-m","pip","check") -TimeoutMs 60000
    $depOut = [string]($dep.output | Out-String)
    $depTail = ($depOut -split "`r?`n" | Where-Object { $_.Trim() } | Select-Object -Last 1)
    if ([bool]$dep.timed_out) {
      $depOk = $false
      $depTail = "dependency_sync_timeout_60s"
    }
  } catch {
    $depOk = $false
    $depTail = $_.Exception.Message
  }
  $steps += @{ step = "dependency_sync"; ok = $depOk; tail = $depTail }
  _WriteJob @{
    ok = $true; tool = $ToolId; status = "running"; started_at = (Get-Date).ToString("o")
    install_method = $InstallMethod; install_target = $InstallTarget
    total = 6; done = 4; failed = 0; steps = $steps
  }

  # Step 5: supervisor adaptation plan
  $supOk = $false
  $supData = $null
  try {
    $supBody = @{
      objective = "Integrar la herramienta $ToolId en ATLAS para operación multitarea y multifuncional."
      context = @{
        source = "tools_discovery_install"
        tool_id = $ToolId
        install_method = $InstallMethod
        install_target = $InstallTarget
      }
    } | ConvertTo-Json -Depth 8
    $supResp = Invoke-RestMethod -Uri "http://localhost:8791/supervisor/advise" -Method Post -ContentType "application/json" -Body $supBody -TimeoutSec 60
    $supData = $supResp
    $supOk = $true
  } catch {
    $supOk = $false
    $supData = @{ ok = $false; error = $_.Exception.Message }
  }
  $steps += @{ step = "supervisor_adaptation"; ok = $supOk }
  _WriteJob @{
    ok = $true; tool = $ToolId; status = "running"; started_at = (Get-Date).ToString("o")
    install_method = $InstallMethod; install_target = $InstallTarget
    total = 6; done = 5; failed = 0; steps = $steps
  }

  # Step 6: verify + report (method-aware validation, no false negatives by PATH)
  $verifyCmd = @()
  $verifyOk = $false
  $verifyTail = ""
  if ($InstallMethod -eq "winget") {
    try {
      $v = Invoke-CapturedProcess -Exe "winget" -Args @("list","--id",$InstallTarget,"--source","winget","--disable-interactivity") -TimeoutMs 90000
      $vraw = [string]($v.output | Out-String)
      $verifyTail = ($vraw -split "`r?`n" | Where-Object { $_.Trim() } | Select-Object -Last 1)
      $low = ($vraw | Out-String).ToLowerInvariant()
      $targetNorm = ([string]$InstallTarget).ToLowerInvariant()
      $verifyOk = (([int]$v.exit_code -eq 0) -and (-not [bool]$v.timed_out)) -and (($low -match [regex]::Escape($targetNorm)) -or ($low.Contains(([string]$ToolId).ToLowerInvariant())))
      if ([bool]$v.timed_out) {
        $verifyTail = "verify_timeout_90s"
      }
    } catch {
      $verifyOk = $false
      $verifyTail = $_.Exception.Message
    }
    $verifyCmd = @("winget","list","--id",$InstallTarget,"--source","winget","--disable-interactivity")
  } elseif ($InstallMethod -eq "pip") {
    try {
      $v = Invoke-CapturedProcess -Exe $PyExe -Args @("-m","pip","show",$InstallTarget) -TimeoutMs 60000
      $vraw = [string]($v.output | Out-String)
      $verifyTail = ($vraw -split "`r?`n" | Where-Object { $_.Trim() } | Select-Object -Last 1)
      $low = ($vraw | Out-String).ToLowerInvariant()
      $verifyOk = (([int]$v.exit_code -eq 0) -and (-not [bool]$v.timed_out)) -and $low.Contains("name:")
      if ([bool]$v.timed_out) {
        $verifyTail = "verify_timeout_60s"
      }
    } catch {
      $verifyOk = $false
      $verifyTail = $_.Exception.Message
    }
    $verifyCmd = @($PyExe,"-m","pip","show",$InstallTarget)
  } elseif ($InstallMethod -eq "npm") {
    try {
      $v = Invoke-CapturedProcess -Exe "npm" -Args @("list","-g",$InstallTarget,"--depth=0") -TimeoutMs 90000
      $vraw = [string]($v.output | Out-String)
      $verifyTail = ($vraw -split "`r?`n" | Where-Object { $_.Trim() } | Select-Object -Last 1)
      $low = ($vraw | Out-String).ToLowerInvariant()
      $verifyOk = (([int]$v.exit_code -eq 0) -and (-not [bool]$v.timed_out)) -and $low.Contains(([string]$InstallTarget).ToLowerInvariant())
      if ([bool]$v.timed_out) {
        $verifyTail = "verify_timeout_90s"
      }
    } catch {
      $verifyOk = $false
      $verifyTail = $_.Exception.Message
    }
    $verifyCmd = @("npm","list","-g",$InstallTarget,"--depth=0")
  } else {
    $verifyOk = $true
    $verifyTail = "no_verify_method"
  }
  $steps += @{ step = "verify"; ok = $verifyOk; command = ($verifyCmd -join " "); tail = $verifyTail }
  if (-not $verifyOk) { throw ("verify_failed:" + (($verifyTail | Out-String).Trim())) }
  $reportsDir = Join-Path $RepoRoot "reports"
  New-Item -ItemType Directory -Path $reportsDir -Force | Out-Null
  $ts = Get-Date -Format "yyyyMMdd-HHmmss"
  $reportPath = Join-Path $reportsDir ("tools_discovery_install_{0}_{1}.md" -f $ToolId, $ts)
  $supAnalysis = ""
  try {
    if ($supData -and $supData.data -and $supData.data.analysis) { $supAnalysis = [string]$supData.data.analysis }
  } catch {}
  $reportLines = @()
  $reportLines += "# ATLAS Expansion Report"
  $reportLines += ""
  $reportLines += "- Tool: $ToolId"
  $reportLines += "- Install method: $InstallMethod"
  $reportLines += "- Install target: $InstallTarget"
  $reportLines += "- Finished: $(Get-Date -Format o)"
  $reportLines += ""
  $reportLines += "## Execution Steps"
  foreach ($st in $steps) {
    $flag = if ($st.ok) { "ok" } else { "fail" }
    $reportLines += "- $($st.step): $flag"
  }
  $reportLines += ""
  $reportLines += "## Supervisor Adaptation"
  $reportLines += "$supAnalysis"
  $reportLines += ""
  $reportLines += "## Raw Supervisor Payload"
  $reportLines += '```json'
  $reportLines += ($supData | ConvertTo-Json -Depth 12)
  $reportLines += '```'
  $report = ($reportLines -join "`r`n")
  Set-Content -Path $reportPath -Value $report -Encoding UTF8

  _WriteJob @{
    ok = $true
    tool = $ToolId
    status = "done"
    started_at = (Get-Date).ToString("o")
    finished_at = (Get-Date).ToString("o")
    install_method = $InstallMethod
    install_target = $InstallTarget
    total = 6
    done = 6
    failed = 0
    steps = $steps
    supervisor = $supData
    report_path = $reportPath
  }
}
catch {
  $msg = $_.Exception.Message
  if (-not $msg -or -not $msg.Trim()) {
    $msg = "unknown_discovery_install_error"
  }
  $reportsDir = Join-Path $RepoRoot "reports"
  New-Item -ItemType Directory -Path $reportsDir -Force | Out-Null
  $ts = Get-Date -Format "yyyyMMdd-HHmmss"
  $reportPath = Join-Path $reportsDir ("tools_discovery_install_{0}_{1}_FAILED.md" -f $ToolId, $ts)
  $reportLines = @()
  $reportLines += "# ATLAS Expansion Report (FAILED)"
  $reportLines += ""
  $reportLines += "- Tool: $ToolId"
  $reportLines += "- Install method: $InstallMethod"
  $reportLines += "- Install target: $InstallTarget"
  $reportLines += "- Finished: $(Get-Date -Format o)"
  $reportLines += "- Error: $msg"
  $reportLines += ""
  $reportLines += "## Notes"
  $reportLines += "- El pipeline no completó adaptación automática."
  $report = ($reportLines -join "`r`n")
  Set-Content -Path $reportPath -Value $report -Encoding UTF8
  _WriteJob @{
    ok = $false
    tool = $ToolId
    status = "failed"
    finished_at = (Get-Date).ToString("o")
    install_method = $InstallMethod
    install_target = $InstallTarget
    total = 6
    done = 5
    failed = 1
    error = $msg
    report_path = $reportPath
  }
}
