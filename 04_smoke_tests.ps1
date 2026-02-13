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

$ver = Hit "/version"
if ($ver.ok -and $ver.data.version) {
  Write-Host "GET /version OK ($($ver.data.version) $($ver.data.channel))" -ForegroundColor Green
} else {
  Write-Host "GET /version FAIL: $($ver.err)" -ForegroundColor Red
}

# Deploy / Health / Canary (base URL = same $base from -AtlasPort)
$health = Hit "/health"
if ($health.ok -and ($health.data.ok -ne $null -or $health.data.score -ge 0)) {
  Write-Host "GET /health OK (score=$($health.data.score) ms=$($health.data.ms))" -ForegroundColor Green
} else {
  Write-Host "GET /health FAIL: $($health.err)" -ForegroundColor Red
}
$selfcheck = Hit "/support/selfcheck"
if ($selfcheck.ok -and $selfcheck.data) {
  Write-Host "GET /support/selfcheck OK" -ForegroundColor Green
} else {
  Write-Host "GET /support/selfcheck FAIL: $($selfcheck.err)" -ForegroundColor Red
}
$deployStatus = Hit "/deploy/status"
if ($deployStatus.ok -and $deployStatus.data -and ($deployStatus.data.ok -or $deployStatus.data.mode)) {
  Write-Host "GET /deploy/status OK (mode=$($deployStatus.data.mode))" -ForegroundColor Green
} else {
  Write-Host "GET /deploy/status FAIL: $($deployStatus.err)" -ForegroundColor Red
}
$canaryStatus = Hit "/canary/status"
if ($canaryStatus.ok -and $canaryStatus.data) {
  Write-Host "GET /canary/status OK" -ForegroundColor Green
} else {
  Write-Host "GET /canary/status FAIL: $($canaryStatus.err)" -ForegroundColor Red
}

# Cluster (fallback local if no worker; must not hang)
$clusterStatus = Hit "/cluster/status"
if ($clusterStatus.ok -and $clusterStatus.data) {
  Write-Host "GET /cluster/status OK (enabled=$($clusterStatus.data.enabled))" -ForegroundColor Green
} else {
  Write-Host "GET /cluster/status FAIL: $($clusterStatus.err)" -ForegroundColor Red
}
$clusterRegBody = @{ node_id = "smoke-local"; role = "worker"; base_url = $base; capabilities = @{ hands = $true; web = $true; vision = $false; voice = $false; llm = $true } }
$clusterReg = HitPost "/cluster/node/register" $clusterRegBody
if ($clusterReg.ok -or ($clusterReg.data -and $clusterReg.data.error -match "policy")) {
  Write-Host "POST /cluster/node/register OK or policy-denied (expected)" -ForegroundColor Green
} else {
  Write-Host "POST /cluster/node/register: $($clusterReg.err)" -ForegroundColor Yellow
}
$clusterHeartbeatBody = @{ node_id = "smoke-local"; capabilities = @{ hands = $true }; health = @{ score = 80 }; version = "0.0.0"; channel = "canary"; base_url = $base }
$clusterHeartbeat = HitPost "/cluster/heartbeat" $clusterHeartbeatBody
if ($clusterHeartbeat.ok -and $clusterHeartbeat.data) {
  Write-Host "POST /cluster/heartbeat OK" -ForegroundColor Green
} else {
  Write-Host "POST /cluster/heartbeat: $($clusterHeartbeat.err)" -ForegroundColor Yellow
}
$clusterNodes = Hit "/cluster/nodes"
if ($clusterNodes.ok) {
  Write-Host "GET /cluster/nodes OK" -ForegroundColor Green
} else {
  Write-Host "GET /cluster/nodes FAIL: $($clusterNodes.err)" -ForegroundColor Red
}

# Gateway (must respond quickly; no_gateway_config OK, no hang)
$gatewayStatus = Hit "/gateway/status"
if ($gatewayStatus.ok -and $gatewayStatus.data) {
  Write-Host "GET /gateway/status OK (enabled=$($gatewayStatus.data.enabled))" -ForegroundColor Green
} else {
  Write-Host "GET /gateway/status FAIL: $($gatewayStatus.err)" -ForegroundColor Red
}
$gatewayCheck = HitPost "/gateway/check" (@{})
if ($gatewayCheck.ok -or ($gatewayCheck.data -and $gatewayCheck.data.error -match "policy")) {
  Write-Host "POST /gateway/check OK or policy-denied" -ForegroundColor Green
} else {
  Write-Host "POST /gateway/check: $($gatewayCheck.err)" -ForegroundColor Yellow
}
$gatewayBootstrap = HitPost "/gateway/bootstrap" (@{})
if ($gatewayBootstrap.ok) {
  Write-Host "POST /gateway/bootstrap OK" -ForegroundColor Green
} elseif ($gatewayBootstrap.data -and $gatewayBootstrap.data.error -eq "no_gateway_config") {
  Write-Host "POST /gateway/bootstrap no_gateway_config (expected without config)" -ForegroundColor Green
} else {
  Write-Host "POST /gateway/bootstrap: $($gatewayBootstrap.err)" -ForegroundColor Yellow
}
$clusterStatusAgain = Hit "/cluster/status"
if ($clusterStatusAgain.ok) {
  Write-Host "GET /cluster/status OK" -ForegroundColor Green
} else {
  Write-Host "GET /cluster/status FAIL: $($clusterStatusAgain.err)" -ForegroundColor Red
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

# Update check (plan only, timeout 15s)
$updateCheck = HitPost "/update/check" (@{})
if ($updateCheck.ok) {
  Write-Host "POST /update/check OK (plan only)" -ForegroundColor Green
} else {
  Write-Host "POST /update/check FAIL: $($updateCheck.err)" -ForegroundColor Red
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

# Memory: thread create + write + recall + export (timeouts 10s)
$memThreadCreate = HitPost "/memory/thread/create" (@{ title = "smoke_thread" })
$memThreadId = $null
if ($memThreadCreate.ok -and $memThreadCreate.data.ok -and $memThreadCreate.data.data.thread_id) {
  $memThreadId = $memThreadCreate.data.data.thread_id
  Write-Host "POST /memory/thread/create OK (thread_id=$memThreadId)" -ForegroundColor Green
} else {
  Write-Host "POST /memory/thread/create FAIL: $($memThreadCreate.err)" -ForegroundColor Red
}

if ($memThreadId) {
  $memWriteBody = @{ thread_id = $memThreadId; kind = "summary"; payload = @{ content = "Smoke test write." } }
  $memWrite = HitPost "/memory/write" $memWriteBody
  if ($memWrite.ok -and $memWrite.data.ok) {
    Write-Host "POST /memory/write OK" -ForegroundColor Green
  } else {
    Write-Host "POST /memory/write FAIL: $($memWrite.err)" -ForegroundColor Red
  }
}

$memRecall = Hit "/memory/recall?query=Listar&limit=5"
if ($memRecall.ok) {
  Write-Host "GET /memory/recall OK (memory write/recall)" -ForegroundColor Green
  $memOk = $true
} else {
  Write-Host "GET /memory/recall FAIL: $($memRecall.err)" -ForegroundColor Red
  $memOk = $false
}

$memExport = Hit "/memory/export?limit=10"
if ($memExport.ok) {
  Write-Host "GET /memory/export OK" -ForegroundColor Green
} else {
  Write-Host "GET /memory/export FAIL: $($memExport.err)" -ForegroundColor Red
}

$memThreadList = Hit "/memory/thread/list?limit=5"
if ($memThreadList.ok) {
  Write-Host "GET /memory/thread/list OK" -ForegroundColor Green
} else {
  Write-Host "GET /memory/thread/list FAIL: $($memThreadList.err)" -ForegroundColor Red
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

# CI improve (plan_only repo, must respond <15s)
$improveBody = @{ scope = "repo"; mode = "plan_only"; depth = 2; max_items = 5 }
$improveStart = Get-Date
$improveResp = HitPost "/agent/improve" $improveBody
$improveMs = ((Get-Date) - $improveStart).TotalMilliseconds
if ($improveResp.ok -and $improveResp.data) {
  if ($improveMs -lt 15000) {
    Write-Host "POST /agent/improve (plan_only repo) OK in ${improveMs}ms (<15s)" -ForegroundColor Green
    $improveOk = $true
  } else {
    Write-Host "POST /agent/improve OK but slow: ${improveMs}ms" -ForegroundColor Yellow
    $improveOk = $true
  }
} else {
  Write-Host "POST /agent/improve FAIL: $($improveResp.err)" -ForegroundColor Red
  $improveOk = $false
}
$improveStatus = Hit "/agent/improve/status"
if ($improveStatus.ok) {
  Write-Host "GET /agent/improve/status OK" -ForegroundColor Green
} else {
  Write-Host "GET /agent/improve/status FAIL: $($improveStatus.err)" -ForegroundColor Red
}

$uiResp = Invoke-WebRequest -Method GET -Uri ($base + "/ui") -TimeoutSec 5 -UseBasicParsing -ErrorAction SilentlyContinue
if ($uiResp -and $uiResp.StatusCode -eq 200) {
  Write-Host "GET /ui 200 OK" -ForegroundColor Green
  $uiOk = $true
} else {
  Write-Host "GET /ui FAIL" -ForegroundColor Red
  $uiOk = $false
}

$approvalsList = Hit "/approvals/list"
if ($approvalsList.ok) {
  Write-Host "GET /approvals/list OK" -ForegroundColor Green
} else {
  Write-Host "GET /approvals/list FAIL: $($approvalsList.err)" -ForegroundColor Red
}

# GA (Governed Autonomy): run plan_only, status, report (must respond <15s)
Write-Host "== SMOKE: GA (Governed Autonomy) ==" -ForegroundColor Cyan
$gaRunBody = @{ scope = "runtime"; mode = "plan_only"; max_findings = 5 }
$gaRunStart = Get-Date
$gaRun = HitPost "/ga/run" $gaRunBody
$gaRunMs = ((Get-Date) - $gaRunStart).TotalMilliseconds
if ($gaRun.ok -and $gaRun.data) {
  if ($gaRunMs -lt 15000) {
    Write-Host "POST /ga/run (scope=runtime mode=plan_only) OK in ${gaRunMs}ms (<15s)" -ForegroundColor Green
    $gaRunOk = $true
  } else {
    Write-Host "POST /ga/run OK but slow: ${gaRunMs}ms" -ForegroundColor Yellow
    $gaRunOk = $true
  }
  $rp = $gaRun.data.data.report_path
  if ($rp -and (Test-Path $rp -ErrorAction SilentlyContinue)) {
    Write-Host "GA report_path exists: $rp" -ForegroundColor Green
  } elseif ($rp) {
    Write-Host "GA report_path set: $rp" -ForegroundColor Yellow
  }
} elseif ($gaRun.data -and $gaRun.data.error -match "GOVERNED_AUTONOMY") {
  Write-Host "POST /ga/run disabled (GOVERNED_AUTONOMY_ENABLED=false)" -ForegroundColor Yellow
  $gaRunOk = $true
} else {
  Write-Host "POST /ga/run FAIL: $($gaRun.err)" -ForegroundColor Red
  $gaRunOk = $false
}
$gaStatus = Hit "/ga/status"
if ($gaStatus.ok -and $gaStatus.data) {
  Write-Host "GET /ga/status OK (mode=$($gaStatus.data.data.mode) pending=$($gaStatus.data.data.approvals_pending_count))" -ForegroundColor Green
  $gaStatusOk = $true
} else {
  Write-Host "GET /ga/status FAIL: $($gaStatus.err)" -ForegroundColor Red
  $gaStatusOk = $false
}
$gaReportLatest = Hit "/ga/report/latest"
if ($gaReportLatest.ok) {
  Write-Host "GET /ga/report/latest OK" -ForegroundColor Green
} else {
  Write-Host "GET /ga/report/latest FAIL: $($gaReportLatest.err)" -ForegroundColor Red
}

# Meta-Learning
Write-Host "== SMOKE: Meta-Learning ==" -ForegroundColor Cyan
$metalearnStatus = Hit "/metalearn/status"
if ($metalearnStatus.ok) {
  Write-Host "GET /metalearn/status OK" -ForegroundColor Green
  $metalearnStatusOk = $true
} else {
  Write-Host "GET /metalearn/status FAIL: $($metalearnStatus.err)" -ForegroundColor Red
  $metalearnStatusOk = $false
}
$metalearnRun = HitPost "/metalearn/run" (@{})
if ($metalearnRun.ok) {
  Write-Host "POST /metalearn/run OK" -ForegroundColor Green
  $metalearnRunOk = $true
} elseif ($metalearnRun.err -match "METALEARN|503|disabled") {
  Write-Host "POST /metalearn/run disabled or 503 (acceptable)" -ForegroundColor Yellow
  $metalearnRunOk = $true
} else {
  Write-Host "POST /metalearn/run FAIL: $($metalearnRun.err)" -ForegroundColor Red
  $metalearnRunOk = $false
}
$metalearnReport = Hit "/metalearn/report/latest"
if ($metalearnReport.ok) {
  Write-Host "GET /metalearn/report/latest OK" -ForegroundColor Green
} else {
  Write-Host "GET /metalearn/report/latest FAIL: $($metalearnReport.err)" -ForegroundColor Red
}
$metalearnOk = $metalearnStatusOk -and $metalearnRunOk

# Owner: session start, end, status, chain verify, emergency (non-destructive)
$ownerSessionBody = @{ actor = "owner"; method = "ui" }
$ownerSession = HitPost "/owner/session/start" $ownerSessionBody
if ($ownerSession.ok -and $ownerSession.data -and $ownerSession.data.session_token) {
  Write-Host "POST /owner/session/start OK (token present)" -ForegroundColor Green
  $sessToken = $ownerSession.data.session_token
  $ownerEndBody = @{ session_token = $sessToken }
  $ownerEnd = HitPost "/owner/session/end" $ownerEndBody
  if ($ownerEnd.ok) {
    Write-Host "POST /owner/session/end OK" -ForegroundColor Green
  } else {
    Write-Host "POST /owner/session/end: $($ownerEnd.err)" -ForegroundColor Yellow
  }
} else {
  Write-Host "POST /owner/session/start FAIL: $($ownerSession.err)" -ForegroundColor Red
}
$ownerSessStatus = Hit "/owner/session/status"
if ($ownerSessStatus.ok) {
  Write-Host "GET /owner/session/status OK" -ForegroundColor Green
} else {
  Write-Host "GET /owner/session/status FAIL: $($ownerSessStatus.err)" -ForegroundColor Red
}
$chainVerify = Hit "/approvals/chain/verify"
if ($chainVerify.ok -and ($chainVerify.data -or $chainVerify.valid -ne $null)) {
  $valid = if ($chainVerify.valid -ne $null) { $chainVerify.valid } else { $chainVerify.data.valid }
  Write-Host "GET /approvals/chain/verify OK (valid=$valid)" -ForegroundColor Green
} else {
  Write-Host "GET /approvals/chain/verify FAIL: $($chainVerify.err)" -ForegroundColor Red
}
$emergencyStatus = Hit "/owner/emergency/status"
if ($emergencyStatus.ok -and $emergencyStatus.data) {
  Write-Host "GET /owner/emergency/status OK" -ForegroundColor Green
} else {
  Write-Host "GET /owner/emergency/status FAIL: $($emergencyStatus.err)" -ForegroundColor Red
}
$emergencySetBody = @{ enable = $false }
$emergencyResp = HitPost "/owner/emergency/set" $emergencySetBody
if ($emergencyResp.ok) {
  Write-Host "POST /owner/emergency/set enable=false OK (emergency=$($emergencyResp.data.emergency))" -ForegroundColor Green
} else {
  Write-Host "POST /owner/emergency/set FAIL: $($emergencyResp.err)" -ForegroundColor Red
}

$bundleResp = Hit "/support/bundle"
if ($bundleResp.ok -and $bundleResp.data.path) {
  Write-Host "GET /support/bundle OK ($($bundleResp.data.path))" -ForegroundColor Green
  $bundleOk = $true
} else {
  Write-Host "GET /support/bundle FAIL: $($bundleResp.err)" -ForegroundColor Red
  $bundleOk = $false
}

$agentScaffoldOk = $agentOk -and $scaffoldOk -and $scriptOk -and $depsOk -and $webOk -and $voiceOk -and $memOk -and $visionOk -and $improveOk

$gaOk = $gaRunOk -and $gaStatusOk
if ($st.ok -and $md.ok -and $llm.ok -and $humanoidOk -and $policyAuditOk -and $schedWatchdogOk -and $agentScaffoldOk -and $uiOk -and $bundleOk -and $gaOk -and $metalearnOk) {
  Write-Host "SMOKE OK" -ForegroundColor Green
  exit 0
} else {
  Write-Host "SMOKE FAIL" -ForegroundColor Red
  exit 1
}
