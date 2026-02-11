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

$hPlan = HitPost "/humanoid/plan" (@{ goal = "Abrir un archivo de texto" })
if ($hPlan.ok -and $hPlan.data.ok) {
  Write-Host "/humanoid/plan OK" -ForegroundColor Green
} else {
  Write-Host "/humanoid/plan FAIL or no steps: $($hPlan.err)" -ForegroundColor Red
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

if ($st.ok -and $md.ok -and $llm.ok -and $humanoidOk -and $policyAuditOk) {
  Write-Host "SMOKE OK" -ForegroundColor Green
  exit 0
} else {
  Write-Host "SMOKE FAIL" -ForegroundColor Red
  exit 1
}
