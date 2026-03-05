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
    $installCmd = @("winget","install","--id",$InstallTarget,"-e","--silent","--accept-package-agreements","--accept-source-agreements")
  } elseif ($InstallMethod -eq "pip") {
    $installCmd = @($PyExe,"-m","pip","install","-U",$InstallTarget)
  } elseif ($InstallMethod -eq "npm") {
    $installCmd = @("npm","install","-g",$InstallTarget)
  } else {
    throw "unsupported_install_method:$InstallMethod"
  }

  $raw = & $installCmd[0] $installCmd[1..($installCmd.Length-1)] 2>&1 | Out-String
  $installOk = $LASTEXITCODE -eq 0
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
    $depOut = & $PyExe -m pip check 2>&1 | Out-String
    $depTail = ($depOut -split "`r?`n" | Where-Object { $_.Trim() } | Select-Object -Last 1)
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
      $vraw = & winget list --id $InstallTarget --source winget 2>&1 | Out-String
      $verifyTail = ($vraw -split "`r?`n" | Where-Object { $_.Trim() } | Select-Object -Last 1)
      $low = ($vraw | Out-String).ToLowerInvariant()
      $targetNorm = ([string]$InstallTarget).ToLowerInvariant()
      $verifyOk = ($LASTEXITCODE -eq 0) -and (($low -match [regex]::Escape($targetNorm)) -or ($low.Contains(([string]$ToolId).ToLowerInvariant())))
    } catch {
      $verifyOk = $false
      $verifyTail = $_.Exception.Message
    }
    $verifyCmd = @("winget","list","--id",$InstallTarget,"--source","winget")
  } elseif ($InstallMethod -eq "pip") {
    try {
      $vraw = & $PyExe -m pip show $InstallTarget 2>&1 | Out-String
      $verifyTail = ($vraw -split "`r?`n" | Where-Object { $_.Trim() } | Select-Object -Last 1)
      $low = ($vraw | Out-String).ToLowerInvariant()
      $verifyOk = ($LASTEXITCODE -eq 0) -and $low.Contains("name:")
    } catch {
      $verifyOk = $false
      $verifyTail = $_.Exception.Message
    }
    $verifyCmd = @($PyExe,"-m","pip","show",$InstallTarget)
  } elseif ($InstallMethod -eq "npm") {
    try {
      $vraw = & npm list -g $InstallTarget --depth=0 2>&1 | Out-String
      $verifyTail = ($vraw -split "`r?`n" | Where-Object { $_.Trim() } | Select-Object -Last 1)
      $low = ($vraw | Out-String).ToLowerInvariant()
      $verifyOk = ($LASTEXITCODE -eq 0) -and $low.Contains(([string]$InstallTarget).ToLowerInvariant())
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
