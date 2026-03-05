param(
  [Parameter(Mandatory = $true)][string]$RepoRoot,
  [Parameter(Mandatory = $true)][string]$JobFile,
  [Parameter(Mandatory = $true)][string]$ToolsCsv
)

$ErrorActionPreference = "Stop"

function _WriteJob([hashtable]$obj) {
  $dir = Split-Path -Parent $JobFile
  if (-not (Test-Path $dir)) {
    New-Item -ItemType Directory -Path $dir -Force | Out-Null
  }
  ($obj | ConvertTo-Json -Depth 12) | Set-Content -Path $JobFile -Encoding UTF8
}

function Invoke-UpdateToolWithTimeout {
  param(
    [Parameter(Mandatory = $true)][string]$ScriptPath,
    [Parameter(Mandatory = $true)][string]$ToolId,
    [Parameter(Mandatory = $true)][string]$RepoRoot,
    [int]$TimeoutMs = 240000
  )
  $tmpOut = [System.IO.Path]::GetTempFileName()
  $tmpErr = [System.IO.Path]::GetTempFileName()
  $raw = ""
  $timedOut = $false
  $exitCode = -1
  try {
    $argList = @("-NoProfile","-ExecutionPolicy","Bypass","-File",$ScriptPath,"-Tool",$ToolId,"-RepoRoot",$RepoRoot)
    $p = Start-Process -FilePath "powershell" -ArgumentList $argList -NoNewWindow -PassThru -RedirectStandardOutput $tmpOut -RedirectStandardError $tmpErr
    $finished = $p.WaitForExit($TimeoutMs)
    if (-not $finished) {
      $timedOut = $true
      try { $p.Kill() } catch {}
      $exitCode = 124
    } else {
      $exitCode = [int]$p.ExitCode
    }
    if (Test-Path $tmpOut) {
      $o = Get-Content -Path $tmpOut -Raw -ErrorAction SilentlyContinue
      if ($o) { $raw = ($raw + "`n" + $o).Trim() }
    }
    if (Test-Path $tmpErr) {
      $e = Get-Content -Path $tmpErr -Raw -ErrorAction SilentlyContinue
      if ($e) { $raw = ($raw + "`n" + $e).Trim() }
    }
  } finally {
    Remove-Item -Path $tmpOut -Force -ErrorAction SilentlyContinue
    Remove-Item -Path $tmpErr -Force -ErrorAction SilentlyContinue
  }

  if ($timedOut) {
    return @{
      ok = $false
      tool = $ToolId
      error = "update_tool_timeout_240s"
    }
  }

  $line = ($raw -split "`r?`n" | Where-Object { $_.Trim() } | Select-Object -Last 1)
  $parsed = $null
  try { $parsed = $line | ConvertFrom-Json } catch {}
  if ($null -eq $parsed) {
    $lineSafe = if ($null -eq $line) { "" } else { $line }
    $lineFlat = ($lineSafe -replace "`r|`n"," ")
    if ($exitCode -ne 0 -and -not [string]::IsNullOrWhiteSpace($lineFlat)) {
      return @{
        ok = $false
        tool = $ToolId
        error = ("update_tool_failed_rc_{0}: {1}" -f $exitCode, $lineFlat.Substring(0, [Math]::Min(240, $lineFlat.Length)))
      }
    }
    return @{
      ok = $false
      tool = $ToolId
      error = ("invalid_update_output: " + $lineFlat.Substring(0, [Math]::Min(240, $lineFlat.Length)))
    }
  }
  return $parsed
}

function Invoke-CapturedProcess {
  param(
    [Parameter(Mandatory = $true)][string]$Exe,
    [string[]]$Args = @(),
    [int]$TimeoutMs = 180000
  )
  $tmpOut = [System.IO.Path]::GetTempFileName()
  $tmpErr = [System.IO.Path]::GetTempFileName()
  $output = ""
  $timedOut = $false
  $exitCode = -1
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
      $o = Get-Content -Path $tmpOut -Raw -ErrorAction SilentlyContinue
      if ($o) { $output = ($output + "`n" + $o).Trim() }
    }
    if (Test-Path $tmpErr) {
      $e = Get-Content -Path $tmpErr -Raw -ErrorAction SilentlyContinue
      if ($e) { $output = ($output + "`n" + $e).Trim() }
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

function Invoke-WingetDirectFallback {
  param(
    [string]$ToolId,
    [string[]]$PackageIds
  )
  $errors = @()
  foreach ($pkg in @($PackageIds | Where-Object { $_ -and $_.Trim() })) {
    $attempts = @(
      @("upgrade","--id",$pkg,"--source","winget","--disable-interactivity","--silent","--accept-package-agreements","--accept-source-agreements"),
      @("upgrade","-e","--id",$pkg,"--source","winget","--disable-interactivity","--silent","--accept-package-agreements","--accept-source-agreements"),
      @("install","-e","--id",$pkg,"--source","winget","--disable-interactivity","--silent","--accept-package-agreements","--accept-source-agreements")
    )
    foreach ($args in $attempts) {
      $r = Invoke-CapturedProcess -Exe "winget" -Args $args -TimeoutMs 180000
      $raw = [string]($r.output | Out-String)
      $tail = ($raw -split "`r?`n" | Where-Object { $_.Trim() } | Select-Object -Last 1)
      $low = $raw.ToLowerInvariant()
      $noUpdate = $low.Contains("no newer package versions are available") -or
        $low.Contains("no hay versiones") -or
        $low.Contains("already installed") -or
        $low.Contains("no applicable update found")
      if (([int]$r.exit_code -eq 0 -and -not [bool]$r.timed_out) -or $noUpdate) {
        return @{
          ok = $true
          strategy = "direct_terminal"
          tool = $ToolId
          package_id = $pkg
          command = ("winget " + ($args -join " "))
          tail = $tail
        }
      }
      $errors += @(
        ("{0}:{1}:rc={2}:timeout={3}:tail={4}" -f $ToolId, $pkg, [int]$r.exit_code, [bool]$r.timed_out, $tail)
      )
    }
  }
  return @{
    ok = $false
    strategy = "direct_terminal"
    tool = $ToolId
    error = ("direct_fallback_failed: " + (($errors | Select-Object -Last 3) -join " | "))
  }
}

function Invoke-DirectTerminalFallback {
  param([string]$ToolId)
  switch ($ToolId.ToLowerInvariant()) {
    "cloudflared" { return Invoke-WingetDirectFallback -ToolId $ToolId -PackageIds @("Cloudflare.cloudflared","cloudflare.cloudflared") }
    "node" { return Invoke-WingetDirectFallback -ToolId $ToolId -PackageIds @("OpenJS.NodeJS","OpenJS.NodeJS.LTS") }
    "ollama" { return Invoke-WingetDirectFallback -ToolId $ToolId -PackageIds @("Ollama.Ollama") }
    default {
      return @{
        ok = $false
        strategy = "direct_terminal"
        tool = $ToolId
        error = "no_direct_fallback_for_tool"
      }
    }
  }
}

function Set-ResultFallback {
  param(
    [Parameter(Mandatory = $true)]$Parsed,
    [Parameter(Mandatory = $true)]$Fallback,
    [bool]$Succeeded = $false
  )
  $steps = @()
  try { if ($Parsed.steps) { $steps = @($Parsed.steps) } } catch {}
  $steps += @{
    step = "supervisor_direct_terminal_fallback"
    ok = $Succeeded
    detail = ($(if ($Fallback.command) { $Fallback.command } else { $Fallback.error }))
  }

  if ($Parsed -is [hashtable]) {
    $Parsed["supervisor_fallback"] = $Fallback
    $Parsed["steps"] = $steps
    if ($Succeeded) {
      $Parsed["ok"] = $true
      $Parsed["error"] = $null
    } else {
      if (-not $Parsed["error"]) { $Parsed["error"] = $Fallback.error }
    }
    return $Parsed
  }

  try { $Parsed | Add-Member -NotePropertyName supervisor_fallback -NotePropertyValue $Fallback -Force } catch {}
  try { $Parsed | Add-Member -NotePropertyName steps -NotePropertyValue $steps -Force } catch {}
  if ($Succeeded) {
    try { $Parsed | Add-Member -NotePropertyName ok -NotePropertyValue $true -Force } catch {}
    try { $Parsed | Add-Member -NotePropertyName error -NotePropertyValue $null -Force } catch {}
  } else {
    try {
      if (-not $Parsed.error) { $Parsed | Add-Member -NotePropertyName error -NotePropertyValue $Fallback.error -Force }
    } catch {}
  }
  return $Parsed
}

$tools = @()
if ($ToolsCsv) {
  $tools = $ToolsCsv.Split(",") | ForEach-Object { $_.Trim() } | Where-Object { $_ }
}

$jobStartedAt = (Get-Date).ToString("o")

_WriteJob @{
  ok = $true
  tool = "all"
  status = "running"
  started_at = $jobStartedAt
  tools = $tools
  total = $tools.Count
  done = 0
  failed = 0
  results = @()
}
Write-Host ("[tools-update-all] start total={0} tools={1}" -f $tools.Count, ($tools -join ","))

try {
  $script = Join-Path $RepoRoot "scripts\atlas_tool_update.ps1"
  if (-not (Test-Path $script)) {
    throw "update script missing: $script"
  }

  $results = @()
  $done = 0
  $failed = 0

  foreach ($t in $tools) {
    Write-Host ("[tools-update-all] begin tool={0} done={1} failed={2}" -f $t, $done, $failed)
    _WriteJob @{
      ok = ($failed -eq 0)
      tool = "all"
      status = "running"
      started_at = $jobStartedAt
      current_tool = $t
      tools = $tools
      total = $tools.Count
      done = $done
      failed = $failed
      results = $results
    }

    $parsed = Invoke-UpdateToolWithTimeout -ScriptPath $script -ToolId $t -RepoRoot $RepoRoot -TimeoutMs 240000
    if (-not [bool]$parsed.ok) {
      Write-Host ("[tools-update-all] fallback begin tool={0}" -f $t)
      $fb = Invoke-DirectTerminalFallback -ToolId $t
      if ([bool]$fb.ok) {
        Write-Host ("[tools-update-all] fallback success tool={0}" -f $t)
        $parsed = Set-ResultFallback -Parsed $parsed -Fallback $fb -Succeeded $true
      } else {
        Write-Host ("[tools-update-all] fallback failed tool={0} error={1}" -f $t, $fb.error)
        $parsed = Set-ResultFallback -Parsed $parsed -Fallback $fb -Succeeded $false
      }
    }

    $done += 1
    if (-not [bool]$parsed.ok) { $failed += 1 }
    $results += @($parsed)
    Write-Host ("[tools-update-all] end tool={0} ok={1} done={2} failed={3}" -f $t, [bool]$parsed.ok, $done, $failed)

    _WriteJob @{
      ok = ($failed -eq 0)
      tool = "all"
      status = "running"
      started_at = $jobStartedAt
      current_tool = $t
      tools = $tools
      total = $tools.Count
      done = $done
      failed = $failed
      results = $results
    }
  }

  $allOk = ($failed -eq 0)
  _WriteJob @{
    ok = $allOk
    tool = "all"
    status = if ($allOk) { "done" } else { "failed" }
    finished_at = (Get-Date).ToString("o")
    tools = $tools
    total = $tools.Count
    done = $done
    failed = $failed
    results = $results
    error = if ($allOk) { $null } else { "one_or_more_updates_failed" }
  }
  Write-Host ("[tools-update-all] finished status={0} done={1} failed={2}" -f $(if ($allOk) { "done" } else { "failed" }), $done, $failed)
}
catch {
  Write-Host ("[tools-update-all] fatal error={0}" -f $_.Exception.Message)
  _WriteJob @{
    ok = $false
    tool = "all"
    status = "failed"
    finished_at = (Get-Date).ToString("o")
    tools = $tools
    total = $tools.Count
    error = $_.Exception.Message
  }
}
