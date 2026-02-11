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

if ($st.ok -and $md.ok -and $llm.ok -and $humanoidOk) {
  Write-Host "SMOKE OK ✅" -ForegroundColor Green
  exit 0
} else {
  Write-Host "SMOKE FAIL ❌" -ForegroundColor Red
  exit 1
}
