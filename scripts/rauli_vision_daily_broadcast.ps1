param(
  [string]$CommsBase = "http://127.0.0.1:8791",
  [string]$VisionApiBase = "https://vision.rauliatlasapp.com",
  [string]$Source = "vision",
  [int]$MaxChannels = 60,
  [switch]$DryRun,
  [switch]$Force
)

$ErrorActionPreference = "Stop"

function Invoke-JsonGet {
  param([string]$Url)
  Invoke-RestMethod -Uri $Url -TimeoutSec 20
}

function Invoke-JsonPost {
  param(
    [string]$Url,
    [hashtable]$Body
  )
  $json = $Body | ConvertTo-Json -Depth 8
  Invoke-RestMethod -Uri $Url -Method POST -ContentType "application/json" -Body $json -TimeoutSec 25
}

function Group-ChannelsByCategory {
  param([object[]]$Channels)
  $groups = @{}
  foreach ($ch in ($Channels | Where-Object { $_ -ne $null })) {
    $cat = [string]$ch.category
    if ([string]::IsNullOrWhiteSpace($cat)) { $cat = "General" }
    if (-not $groups.ContainsKey($cat)) {
      $groups[$cat] = New-Object System.Collections.Generic.List[string]
    }
    $groups[$cat].Add([string]$ch.title)
  }
  return $groups
}

function Build-BroadcastMessage {
  param(
    [string]$RecipientName,
    [object[]]$Channels,
    [string]$VisionLink
  )
  $groups = Group-ChannelsByCategory -Channels $Channels
  $lines = New-Object System.Collections.Generic.List[string]
  $null = $lines.Add("Hola $RecipientName, soy ATLAS.")
  $null = $lines.Add("RAULI VISION ya tiene TV en vivo activa y optimizada para Cuba.")
  $null = $lines.Add("")
  $null = $lines.Add("Canales por categoria:")
  foreach ($cat in ($groups.Keys | Sort-Object)) {
    $titles = ($groups[$cat] | Select-Object -Unique) -join ", "
    $null = $lines.Add("- ${cat}: $titles")
  }
  $null = $lines.Add("")
  $null = $lines.Add("Abrir app: $VisionLink")
  $null = $lines.Add("Si necesitas ayuda, escribe: ATLAS + tu pregunta.")
  return ($lines -join "`n")
}

$root = "C:\ATLAS_PUSH"
$logsDir = Join-Path $root "logs"
$stateFile = Join-Path $logsDir "rauli_vision_broadcast_state.json"
$auditFile = Join-Path $logsDir "rauli_vision_broadcast_audit.jsonl"
New-Item -ItemType Directory -Path $logsDir -Force | Out-Null

$today = (Get-Date).ToString("yyyy-MM-dd")
if ((Test-Path $stateFile) -and -not $Force) {
  try {
    $state = Get-Content -Raw $stateFile | ConvertFrom-Json
    if ($state.last_success_date -eq $today) {
      Write-Host "[BROADCAST] Ya se envio hoy ($today). Usa -Force para reenviar." -ForegroundColor Yellow
      exit 0
    }
  } catch {
    # Ignore corrupted state and continue.
  }
}

$commsBase = $CommsBase.TrimEnd("/")
$visionBase = $VisionApiBase.TrimEnd("/")

Write-Host "[BROADCAST] Validando estado de WhatsApp..." -ForegroundColor Cyan
$wa = Invoke-JsonGet -Url "$commsBase/api/comms/whatsapp/status"
if (-not $wa.ready) {
  throw "WhatsApp no esta listo en $commsBase."
}

Write-Host "[BROADCAST] Cargando usuarios linked de source=$Source..." -ForegroundColor Cyan
$usersResp = Invoke-JsonGet -Url "$commsBase/api/comms/atlas/users/whatsapp?source=$Source&state=linked&limit=300"
$users = @($usersResp.items)
if ($users.Count -eq 0) {
  Write-Host "[BROADCAST] No hay usuarios linked para source=$Source." -ForegroundColor Yellow
  exit 0
}

Write-Host "[BROADCAST] Cargando canales desde $visionBase..." -ForegroundColor Cyan
$searchResp = Invoke-JsonGet -Url "$visionBase/api/video/search?max=$MaxChannels"
$channels = @($searchResp.results)
if ($channels.Count -eq 0) {
  throw "No se encontraron canales para broadcast."
}

$results = @()
foreach ($u in $users) {
  $recipient = [string]$u.external_user_id
  $name = [string]$u.full_name
  if ([string]::IsNullOrWhiteSpace($name)) { $name = [string]$u.username }
  if ([string]::IsNullOrWhiteSpace($name)) { $name = $recipient }

  $message = Build-BroadcastMessage -RecipientName $name -Channels $channels -VisionLink "https://vision.rauliatlasapp.com"

  if ($DryRun) {
    $results += [pscustomobject]@{
      user = $recipient
      ok = $true
      dry_run = $true
      error = ""
    }
    continue
  }

  try {
    $send = Invoke-JsonPost -Url "$commsBase/api/comms/atlas/users/whatsapp/send" -Body @{
      source = $Source
      external_user_id = $recipient
      message = $message
    }
    $results += [pscustomobject]@{
      user = $recipient
      ok = [bool]$send.ok
      dry_run = $false
      error = ""
    }
  } catch {
    $results += [pscustomobject]@{
      user = $recipient
      ok = $false
      dry_run = $false
      error = $_.Exception.Message
    }
  }
}

$success = @($results | Where-Object { $_.ok }).Count
$total = @($results).Count
$summary = [pscustomobject]@{
  ts_utc = (Get-Date).ToUniversalTime().ToString("o")
  source = $Source
  users_total = $total
  users_sent_ok = $success
  dry_run = [bool]$DryRun
  channels_total = $channels.Count
  categories = @((Group-ChannelsByCategory -Channels $channels).Keys | Sort-Object)
}

$summary | ConvertTo-Json -Depth 8 | Add-Content -Path $auditFile -Encoding UTF8
$results | Format-Table -AutoSize
Write-Host "[BROADCAST] Enviados OK: $success/$total" -ForegroundColor Green

if (-not $DryRun -and $success -gt 0) {
  $statePayload = @{
    last_success_date = $today
    last_success_utc = (Get-Date).ToUniversalTime().ToString("o")
    users_sent_ok = $success
    users_total = $total
  } | ConvertTo-Json -Depth 6
  Set-Content -Path $stateFile -Value $statePayload -Encoding UTF8
}

if ($success -lt $total -and -not $DryRun) {
  exit 2
}

exit 0
