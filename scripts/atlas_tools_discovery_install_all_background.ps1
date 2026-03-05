param(
  [Parameter(Mandatory = $true)][string]$RepoRoot,
  [Parameter(Mandatory = $true)][string]$JobFile,
  [Parameter(Mandatory = $true)][string]$ManifestFile
)

$ErrorActionPreference = "Stop"

function _WriteJob([hashtable]$obj) {
  $dir = Split-Path -Parent $JobFile
  if (-not (Test-Path $dir)) { New-Item -ItemType Directory -Path $dir -Force | Out-Null }
  ($obj | ConvertTo-Json -Depth 14) | Set-Content -Path $JobFile -Encoding UTF8
}

$installScript = Join-Path $RepoRoot "scripts\atlas_tools_discovery_install_background.ps1"
if (-not (Test-Path $installScript)) { throw "missing install script: $installScript" }
if (-not (Test-Path $ManifestFile)) { throw "missing manifest: $ManifestFile" }

$manifest = Get-Content $ManifestFile -Raw | ConvertFrom-Json
$items = @()
if ($manifest -and $manifest.tools) { $items = @($manifest.tools) }

$toolIds = @($items | ForEach-Object { [string]$_.id } | Where-Object { $_ })
$total = $toolIds.Count
$done = 0
$failed = 0
$results = @()

_WriteJob @{
  ok = $true
  source = "discovery-bulk"
  tool = "discovery_all"
  status = "running"
  started_at = (Get-Date).ToString("o")
  tools = $toolIds
  total = $total
  done = 0
  failed = 0
  results = @()
}
Write-Host ("[discovery-install-all] start total={0} tools={1}" -f $toolIds.Count, ($toolIds -join ","))

foreach ($it in $items) {
  $id = [string]$it.id
  $method = [string]$it.install_method
  $target = [string]$it.install_target
  if (-not $id -or -not $method -or -not $target) { continue }
  Write-Host ("[discovery-install-all] begin tool={0} method={1} target={2}" -f $id, $method, $target)

  _WriteJob @{
    ok = $true; source = "discovery-bulk"; tool = "discovery_all"; status = "running"
    started_at = (Get-Date).ToString("o")
    current_tool = $id
    tools = $toolIds; total = $total; done = $done; failed = $failed; results = $results
  }

  $childJob = Join-Path $RepoRoot ("logs\tools_update_jobs\discovery_child_{0}.json" -f [Guid]::NewGuid().ToString("N"))
  & powershell -NoProfile -ExecutionPolicy Bypass -File $installScript -RepoRoot $RepoRoot -JobFile $childJob -ToolId $id -InstallMethod $method -InstallTarget $target
  $child = $null
  try { $child = Get-Content $childJob -Raw | ConvertFrom-Json } catch {}
  if (-not $child) {
    $child = @{ ok = $false; tool = $id; status = "failed"; error = "child_job_parse_failed" }
  }
  $done += 1
  if (-not [bool]$child.ok) { $failed += 1 }
  $results += @($child)
  Write-Host ("[discovery-install-all] end tool={0} ok={1} done={2} failed={3}" -f $id, [bool]$child.ok, $done, $failed)

  _WriteJob @{
    ok = $true; source = "discovery-bulk"; tool = "discovery_all"; status = "running"
    started_at = (Get-Date).ToString("o")
    current_tool = $id
    tools = $toolIds; total = $total; done = $done; failed = $failed; results = $results
  }
}

$allOk = ($failed -eq 0)
_WriteJob @{
  ok = $allOk
  source = "discovery-bulk"
  tool = "discovery_all"
  status = if ($allOk) { "done" } else { "failed" }
  finished_at = (Get-Date).ToString("o")
  tools = $toolIds
  total = $total
  done = $done
  failed = $failed
  results = $results
  error = if ($allOk) { $null } else { "one_or_more_discovery_installs_failed" }
}
Write-Host ("[discovery-install-all] finished status={0} done={1} failed={2}" -f $(if ($allOk) { "done" } else { "failed" }), $done, $failed)
