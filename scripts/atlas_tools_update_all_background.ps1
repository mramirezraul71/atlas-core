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

$tools = @()
if ($ToolsCsv) {
  $tools = $ToolsCsv.Split(",") | ForEach-Object { $_.Trim() } | Where-Object { $_ }
}

_WriteJob @{
  ok = $true
  tool = "all"
  status = "running"
  started_at = (Get-Date).ToString("o")
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
      started_at = (Get-Date).ToString("o")
      current_tool = $t
      tools = $tools
      total = $tools.Count
      done = $done
      failed = $failed
      results = $results
    }

    $raw = & powershell -NoProfile -ExecutionPolicy Bypass -File $script -Tool $t -RepoRoot $RepoRoot 2>&1 | Out-String
    $line = ($raw -split "`r?`n" | Where-Object { $_.Trim() } | Select-Object -Last 1)
    $parsed = $null
    try { $parsed = $line | ConvertFrom-Json } catch {}
    if ($null -eq $parsed) {
      $lineSafe = if ($null -eq $line) { "" } else { $line }
      $lineFlat = ($lineSafe -replace "`r|`n"," ")
      $parsed = @{
        ok = $false
        tool = $t
        error = ("invalid_update_output: " + $lineFlat.Substring(0, [Math]::Min(240, $lineFlat.Length)))
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
      started_at = (Get-Date).ToString("o")
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
