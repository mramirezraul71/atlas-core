param(
  [Parameter(Mandatory = $true)][string]$RepoRoot,
  [Parameter(Mandatory = $true)][string]$JobFile,
  [Parameter(Mandatory = $true)][string]$ManifestFile
)

$ErrorActionPreference = "Stop"

function _WriteJob([hashtable]$obj) {
  $dir = Split-Path -Parent $JobFile
  if (-not (Test-Path $dir)) { New-Item -ItemType Directory -Path $dir -Force | Out-Null }
  $json = ($obj | ConvertTo-Json -Depth 16)
  $attempts = 10
  for ($i = 1; $i -le $attempts; $i++) {
    try {
      $json | Set-Content -Path $JobFile -Encoding UTF8 -ErrorAction Stop
      return
    } catch {
      if ($i -ge $attempts) {
        Write-Warning ("[software-apply-all] write-status-failed attempts={0} file={1} err={2}" -f $attempts, $JobFile, $_.Exception.Message)
        return
      }
      Start-Sleep -Milliseconds (80 + (40 * $i))
    }
  }
}

$itemScript = Join-Path $RepoRoot "scripts\atlas_software_apply_item_background.ps1"
if (-not (Test-Path $itemScript)) { throw "missing item script: $itemScript" }
if (-not (Test-Path $ManifestFile)) { throw "missing manifest: $ManifestFile" }

$manifest = Get-Content $ManifestFile -Raw | ConvertFrom-Json
$items = @()
if ($manifest -and $manifest.items) { $items = @($manifest.items) }

$itemIds = @($items | ForEach-Object { [string]$_.id } | Where-Object { $_ })
$total = $itemIds.Count
$done = 0
$failed = 0
$results = @()
$startedAt = (Get-Date).ToString("o")

_WriteJob @{
  ok = $true
  source = "software_apply_all"
  scope = "software"
  status = "running"
  started_at = $startedAt
  items = $itemIds
  total = $total
  done = 0
  failed = 0
  results = @()
}
Write-Host ("[software-apply-all] start total={0} items={1}" -f $total, ($itemIds -join ","))

foreach ($it in $items) {
  $id = [string]$it.id
  $method = [string]$it.install_method
  $target = [string]$it.install_target
  if (-not $id -or -not $method -or -not $target) { continue }

  _WriteJob @{
    ok = ($failed -eq 0)
    source = "software_apply_all"
    scope = "software"
    status = "running"
    started_at = $startedAt
    current_item = $id
    items = $itemIds
    total = $total
    done = $done
    failed = $failed
    results = $results
  }
  Write-Host ("[software-apply-all] begin item={0} method={1}" -f $id, $method)

  $childJob = Join-Path $RepoRoot ("logs\software_update_jobs\software_child_{0}.json" -f [Guid]::NewGuid().ToString("N"))
  & powershell -NoProfile -ExecutionPolicy Bypass -File $itemScript -RepoRoot $RepoRoot -JobFile $childJob -ItemId $id -InstallMethod $method -InstallTarget $target
  $child = $null
  try { $child = Get-Content $childJob -Raw | ConvertFrom-Json } catch {}
  if (-not $child) {
    $child = @{ ok = $false; item_id = $id; status = "failed"; error = "child_job_parse_failed" }
  }

  $done += 1
  if (-not [bool]$child.ok) { $failed += 1 }
  $results += @($child)
  Write-Host ("[software-apply-all] end item={0} ok={1} done={2} failed={3}" -f $id, [bool]$child.ok, $done, $failed)

  _WriteJob @{
    ok = ($failed -eq 0)
    source = "software_apply_all"
    scope = "software"
    status = "running"
    started_at = $startedAt
    current_item = $id
    items = $itemIds
    total = $total
    done = $done
    failed = $failed
    results = $results
  }
}

$allOk = ($failed -eq 0)
_WriteJob @{
  ok = $allOk
  source = "software_apply_all"
  scope = "software"
  status = if ($allOk) { "done" } else { "done_with_errors" }
  started_at = $startedAt
  finished_at = (Get-Date).ToString("o")
  items = $itemIds
  total = $total
  done = $done
  failed = $failed
  results = $results
  error = if ($allOk) { $null } else { "one_or_more_software_applies_failed" }
}
Write-Host ("[software-apply-all] finished status={0} done={1} failed={2}" -f $(if ($allOk) { "done" } else { "done_with_errors" }), $done, $failed)
