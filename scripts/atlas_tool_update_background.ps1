param(
  [Parameter(Mandatory = $true)][string]$Tool,
  [Parameter(Mandatory = $true)][string]$RepoRoot,
  [Parameter(Mandatory = $true)][string]$JobFile
)

$ErrorActionPreference = "Stop"

function _WriteJob([hashtable]$obj) {
  $dir = Split-Path -Parent $JobFile
  if (-not (Test-Path $dir)) {
    New-Item -ItemType Directory -Path $dir -Force | Out-Null
  }
  ($obj | ConvertTo-Json -Depth 10) | Set-Content -Path $JobFile -Encoding UTF8
}

_WriteJob @{
  ok = $true
  tool = $Tool
  status = "running"
  started_at = (Get-Date).ToString("o")
}

try {
  $script = Join-Path $RepoRoot "scripts\atlas_tool_update.ps1"
  if (-not (Test-Path $script)) {
    throw "update script missing: $script"
  }

  $raw = & powershell -NoProfile -ExecutionPolicy Bypass -File $script -Tool $Tool -RepoRoot $RepoRoot 2>&1 | Out-String
  $line = ($raw -split "`r?`n" | Where-Object { $_.Trim() } | Select-Object -Last 1)
  $parsed = $null
  try {
    $parsed = $line | ConvertFrom-Json
  }
  catch {}
  if ($null -eq $parsed) {
    $lineSafe = if ($null -eq $line) { "" } else { $line }
    $lineFlat = ($lineSafe -replace "`r|`n"," ")
    throw ("invalid_update_output: " + $lineFlat.Substring(0, [Math]::Min(240, $lineFlat.Length)))
  }

  $isOk = [bool]$parsed.ok
  $status = if ($isOk) { "done" } else { "failed" }
  _WriteJob @{
    ok = $isOk
    tool = $Tool
    status = $status
    finished_at = (Get-Date).ToString("o")
    result = $parsed
    error = if ($isOk) { $null } else { ($(if ($parsed.error) { $parsed.error } else { "tools_update_failed" })) }
  }
}
catch {
  _WriteJob @{
    ok = $false
    tool = $Tool
    status = "failed"
    finished_at = (Get-Date).ToString("o")
    error = $_.Exception.Message
  }
}
