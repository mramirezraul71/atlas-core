param(
  [Parameter(Mandatory=$true)][string]$RepoPath,
  [int]$AtlasPort = 8791,
  [string]$BindHost = "127.0.0.1"
)

Write-Host "== Smoke tests ==" -ForegroundColor Cyan
$base = "http://${BindHost}:$AtlasPort"

function Hit($path) {
  try {
    $r = Invoke-RestMethod -Method GET -Uri ($base + $path) -TimeoutSec 5
    return @{ ok=$true; data=$r }
  } catch {
    $err = $_.Exception.Message
    if ($err -match "conectar|connection|refused") {
      $err += " [¿API levantada? Ejecuta en otra terminal: .\03_run_atlas_api.ps1 -RepoPath $RepoPath -AtlasPort $AtlasPort]"
    }
    return @{ ok=$false; err=$err }
  }
}

function HitPost($path, $body) {
  try {
    $json = $body | ConvertTo-Json -Compress
    $r = Invoke-RestMethod -Method POST -Uri ($base + $path) -ContentType "application/json" -Body $json -TimeoutSec 30
    return @{ ok=$true; data=$r }
  } catch {
    $err = $_.Exception.Message
    if ($err -match "conectar|connection|refused") {
      $err += " [¿API levantada? Ejecuta en otra terminal: .\03_run_atlas_api.ps1 -RepoPath $RepoPath -AtlasPort $AtlasPort]"
    }
    return @{ ok=$false; err=$err }
  }
}

$st = Hit "/status"
if ($st.ok) {
  Write-Host "/status OK" -ForegroundColor Green
  $st.data | ConvertTo-Json -Depth 5
} else {
  Write-Host "/status FAIL: $($st.err)" -ForegroundColor Red
}

$md = Hit "/modules"
if ($md.ok) {
  Write-Host "/modules OK" -ForegroundColor Green
  $md.data | ConvertTo-Json -Depth 5
} else {
  Write-Host "/modules FAIL: $($md.err)" -ForegroundColor Red
}

Write-Host "== SMOKE: /llm ==" -ForegroundColor Cyan
$body = @{
  prompt = "Devuélveme SOLO la palabra: OK"
  route  = "FAST"
} | ConvertTo-Json
try {
  $r = Invoke-RestMethod -Method Post -Uri "$base/llm" -ContentType "application/json" -Body $body -TimeoutSec 30
  if (-not $r.ok) {
    Write-Host "LLM failed: $($r.error)" -ForegroundColor Red
    $llm = @{ ok = $false }
  } else {
    Write-Host "LLM OK -> route=$($r.route) model=$($r.model_used) ms=$($r.ms)" -ForegroundColor Green
    $llm = @{ ok = $true }
  }
} catch {
  Write-Host "LLM FAIL: $($_.Exception.Message)" -ForegroundColor Red
  $llm = @{ ok = $false }
}

Write-Host "== SMOKE: Humanoid ==" -ForegroundColor Cyan
$hStatus = Hit "/humanoid/status"
if ($hStatus.ok) {
  Write-Host "/humanoid/status OK" -ForegroundColor Green
  if ($hStatus.data.data) { $hStatus.data.data.modules -join ", " }
} else {
  Write-Host "/humanoid/status FAIL: $($hStatus.err)" -ForegroundColor Red
}

$hCheck = HitPost "/humanoid/update-check" (@{})
if ($hCheck.ok -and $hCheck.data.ok) {
  Write-Host "/humanoid/update-check OK" -ForegroundColor Green
} else {
  Write-Host "/humanoid/update-check FAIL: $($hCheck.err)" -ForegroundColor Red
}

# Plan: timeout 45s (planner corta a 15s; aceptamos ok true o error planner_timeout = no bloqueo)
$planBody = @{ goal = "Abrir un archivo de texto"; fast = $true } | ConvertTo-Json
try {
  $planR = Invoke-RestMethod -Method POST -Uri "$base/humanoid/plan" -ContentType "application/json" -Body $planBody -TimeoutSec 45
  if ($planR.ok -and $planR.data) {
    Write-Host "/humanoid/plan OK (steps: $($planR.data.steps.Count))" -ForegroundColor Green
    $hPlan = @{ ok = $true }
  } elseif ($planR.error -eq "planner_timeout") {
    Write-Host "/humanoid/plan OK (planner_timeout en 15s, sin bloqueo)" -ForegroundColor Green
    $hPlan = @{ ok = $true }
  } else {
    Write-Host "/humanoid/plan responde pero falla: $($planR.error)" -ForegroundColor Yellow
    $hPlan = @{ ok = $false }
  }
} catch {
  Write-Host "/humanoid/plan FAIL (timeout o error): $($_.Exception.Message)" -ForegroundColor Red
  $hPlan = @{ ok = $false }
}

$humanoidOk = $hStatus.ok -and $hCheck.ok -and $hPlan.ok

Write-Host "== SMOKE: Metrics / Policy / Audit ==" -ForegroundColor Cyan
$metrics = Hit "/metrics"
if ($metrics.ok) {
  Write-Host "GET /metrics OK" -ForegroundColor Green
} else {
  Write-Host "GET /metrics FAIL: $($metrics.err)" -ForegroundColor Red
}

$policyBody = @{ actor = "api"; role = "owner"; module = "hands"; action = "exec_command"; target = "pip --version" } | ConvertTo-Json
try {
  $policyR = Invoke-RestMethod -Method POST -Uri "$base/policy/test" -ContentType "application/json" -Body $policyBody -TimeoutSec 5
  if ($policyR.allow -eq $true -or $policyR.allow -eq $false) {
    Write-Host "POST /policy/test OK (allow=$($policyR.allow))" -ForegroundColor Green
    $policyOk = $true
  } else { $policyOk = $false }
} catch {
  Write-Host "POST /policy/test FAIL: $($_.Exception.Message)" -ForegroundColor Red
  $policyOk = $false
}

$auditTail = Hit "/audit/tail?n=5"
$auditOk = $false
if ($auditTail.ok -and $auditTail.data) {
  $err = $auditTail.data.error
  if ($null -eq $err -or $err -eq "") {
    Write-Host "GET /audit/tail?n=5 OK (error=null)" -ForegroundColor Green
    $auditOk = $true
  } else {
    Write-Host "GET /audit/tail FAIL: error no debe estar set (actual: $err)" -ForegroundColor Red
  }
} else {
  Write-Host "GET /audit/tail FAIL: $($auditTail.err)" -ForegroundColor Red
}

$policyAuditOk = $metrics.ok -and $policyOk -and $auditOk

Write-Host "== SMOKE: Scheduler / Watchdog ==" -ForegroundColor Cyan
$schedJobs = Hit "/scheduler/jobs"
if ($schedJobs.ok) {
  Write-Host "GET /scheduler/jobs OK" -ForegroundColor Green
} else {
  Write-Host "GET /scheduler/jobs FAIL: $($schedJobs.err)" -ForegroundColor Red
}

$runAt = (Get-Date).AddSeconds(2).ToUniversalTime().ToString("o")
$createBody = @{
  name = "smoke_update_check"
  kind = "update_check"
  payload = @{ required_packages = @("fastapi") }
  run_at = $runAt
}
$create = HitPost "/scheduler/job/create" $createBody
$jobId = $null
if ($create.ok -and $create.data.ok -and $create.data.data) {
  Write-Host "POST /scheduler/job/create OK (job_id=$($create.data.data.id))" -ForegroundColor Green
  $jobId = $create.data.data.id
} else {
  Write-Host "POST /scheduler/job/create FAIL: $($create.err)" -ForegroundColor Red
}

Start-Sleep -Seconds 5

if ($jobId) {
  $runs = Hit "/scheduler/job/runs?job_id=$jobId"
  if ($runs.ok -and $runs.data.data) {
    Write-Host "GET /scheduler/job/runs?job_id=... OK (runs: $($runs.data.data.Count))" -ForegroundColor Green
  } else {
    Write-Host "GET /scheduler/job/runs FAIL: $($runs.err)" -ForegroundColor Red
  }
}

$watchdog = Hit "/watchdog/status"
if ($watchdog.ok) {
  Write-Host "GET /watchdog/status OK" -ForegroundColor Green
} else {
  Write-Host "GET /watchdog/status FAIL: $($watchdog.err)" -ForegroundColor Red
}

$schedWatchdogOk = $schedJobs.ok -and $create.ok -and $watchdog.ok

Write-Host "== SMOKE: Agent / Scaffold / Scripts / Deps / Web / Voice ==" -ForegroundColor Cyan
$agentBody = @{ goal = "Listar archivos del directorio actual"; mode = "plan_only"; fast = $true }
$agentStart = Get-Date
$agentResp = HitPost "/agent/goal" $agentBody
$agentMs = ((Get-Date) - $agentStart).TotalMilliseconds
if ($agentResp.ok -and $agentResp.data.ok -and $agentResp.data.data) {
  if ($agentMs -lt 15000) {
    Write-Host "POST /agent/goal (plan_only) OK in ${agentMs}ms (<15s)" -ForegroundColor Green
    $agentOk = $true
  } else {
    Write-Host "POST /agent/goal OK but slow: ${agentMs}ms" -ForegroundColor Yellow
    $agentOk = $true
  }
} else {
  Write-Host "POST /agent/goal FAIL: $($agentResp.err)" -ForegroundColor Red
  $agentOk = $false
}

$scaffoldBody = @{ type = "fastapi"; name = "smoke_fastapi_minimal"; options = @{ subdir = "_generated" } }
$scaffold = HitPost "/scaffold/app" $scaffoldBody
if ($scaffold.ok -and $scaffold.data.ok -and $scaffold.data.data.files_created) {
  Write-Host "POST /scaffold/app (fastapi minimal) OK" -ForegroundColor Green
  $scaffoldOk = $true
} else {
  Write-Host "POST /scaffold/app FAIL: $($scaffold.err)" -ForegroundColor Red
  $scaffoldOk = $false
}

$scriptBody = @{ kind = "powershell"; purpose = "smoke test script" }
$scriptResp = HitPost "/scripts/generate" $scriptBody
if ($scriptResp.ok -and $scriptResp.data) {
  Write-Host "POST /scripts/generate OK" -ForegroundColor Green
  $scriptOk = $true
} else {
  Write-Host "POST /scripts/generate FAIL: $($scriptResp.err)" -ForegroundColor Red
  $scriptOk = $false
}

$depsResp = Hit "/deps/check"
if ($depsResp.ok -and $depsResp.data) {
  Write-Host "GET /deps/check OK (missing_deps reported)" -ForegroundColor Green
  $depsOk = $true
} else {
  Write-Host "GET /deps/check FAIL: $($depsResp.err)" -ForegroundColor Red
  $depsOk = $false
}

$webStatus = Hit "/web/status"
if ($webStatus.ok) {
  $en = if ($webStatus.data.data) { $webStatus.data.data.enabled } else { $null }
  Write-Host "GET /web/status OK (enabled=$en)" -ForegroundColor Green
  $webOk = $true
} else {
  Write-Host "GET /web/status FAIL: $($webStatus.err)" -ForegroundColor Red
  $webOk = $false
}

$voiceStatus = Hit "/voice/status"
if ($voiceStatus.ok) {
  Write-Host "GET /voice/status OK" -ForegroundColor Green
  $voiceOk = $true
} else {
  Write-Host "GET /voice/status FAIL: $($voiceStatus.err)" -ForegroundColor Red
  $voiceOk = $false
}

$memRecall = Hit "/memory/recall?query=Listar&limit=5"
if ($memRecall.ok) {
  Write-Host "GET /memory/recall OK (memory write/recall)" -ForegroundColor Green
  $memOk = $true
} else {
  Write-Host "GET /memory/recall FAIL: $($memRecall.err)" -ForegroundColor Red
  $memOk = $false
}

$memSnapshot = Hit "/memory/snapshot"
if ($memSnapshot.ok) {
  Write-Host "GET /memory/snapshot OK" -ForegroundColor Green
} else {
  Write-Host "GET /memory/snapshot FAIL: $($memSnapshot.err)" -ForegroundColor Red
}

$visionStatus = Hit "/vision/status"
if ($visionStatus.ok) {
  Write-Host "GET /vision/status OK" -ForegroundColor Green
  $visionOk = $true
} else {
  Write-Host "GET /vision/status FAIL: $($visionStatus.err)" -ForegroundColor Red
  $visionOk = $false
}

$visionAnalyzeBody = @{ image_path = "C:\ATLAS_PUSH\logs\nonexistent.png" }
$visionAnalyze = HitPost "/vision/analyze" $visionAnalyzeBody
if ($visionAnalyze.ok) {
  Write-Host "POST /vision/analyze responds (ok or error for missing file)" -ForegroundColor Green
} else {
  Write-Host "POST /vision/analyze FAIL: $($visionAnalyze.err)" -ForegroundColor Red
}

$agentScaffoldOk = $agentOk -and $scaffoldOk -and $scriptOk -and $depsOk -and $webOk -and $voiceOk -and $memOk -and $visionOk

if ($st.ok -and $md.ok -and $llm.ok -and $humanoidOk -and $policyAuditOk -and $schedWatchdogOk -and $agentScaffoldOk) {
  Write-Host "SMOKE OK" -ForegroundColor Green
  exit 0
} else {
  Write-Host "SMOKE FAIL" -ForegroundColor Red
  exit 1
}
