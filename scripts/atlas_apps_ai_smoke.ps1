param(
    [string]$PushBase = "http://127.0.0.1:8791",
    [string]$PanaderiaBase = "http://127.0.0.1:3001",
    [string]$VisionBase = "http://127.0.0.1:3000",
    [string]$PanUser = "admin",
    [string]$PanPassword = "admin123"
)

$ErrorActionPreference = "Stop"

function Invoke-JsonPost {
    param(
        [string]$Url,
        [hashtable]$Body,
        [hashtable]$Headers = @{},
        [int]$TimeoutSec = 60
    )
    $json = $Body | ConvertTo-Json -Depth 8
    Invoke-RestMethod -Uri $Url -Method Post -ContentType "application/json" -Body $json -Headers $Headers -TimeoutSec $TimeoutSec
}

function Safe-Get {
    param([string]$Url, [int]$TimeoutSec = 20)
    try {
        $r = Invoke-RestMethod -Uri $Url -TimeoutSec $TimeoutSec
        [pscustomobject]@{ ok = $true; data = $r; error = "" }
    } catch {
        [pscustomobject]@{ ok = $false; data = $null; error = $_.Exception.Message }
    }
}

function Safe-Post {
    param(
        [string]$Url,
        [hashtable]$Body,
        [hashtable]$Headers = @{},
        [int]$TimeoutSec = 60
    )
    try {
        $r = Invoke-JsonPost -Url $Url -Body $Body -Headers $Headers -TimeoutSec $TimeoutSec
        [pscustomobject]@{ ok = $true; data = $r; error = "" }
    } catch {
        [pscustomobject]@{ ok = $false; data = $null; error = $_.Exception.Message }
    }
}

$out = [ordered]@{}

$out.push_health = Safe-Get "$PushBase/health"
$out.push_comms = Safe-Get "$PushBase/api/comms/atlas/status" 25
$out.push_message = Safe-Post "$PushBase/api/comms/atlas/message" @{
    user_id = "smoke"
    channel = "panaderia-app"
    message = "Responde solo OK"
    context = @{ source = "smoke" }
} @{} 90

$login = Safe-Post "$PanaderiaBase/api/auth/login" @{
    username = $PanUser
    password = $PanPassword
} @{} 20
$out.panaderia_login = $login

$panHeaders = @{}
if ($login.ok -and $login.data.token) {
    $panHeaders.Authorization = "Bearer $($login.data.token)"
}

$out.panaderia_health = Safe-Get "$PanaderiaBase/api/health"
$out.panaderia_ai = Safe-Post "$PanaderiaBase/api/ai/atlas/chat" @{
    message = "Quiero reportar una venta"
    history = @()
    context = @{
        profile = "BETY"
        role = "contadora"
        source = "smoke"
    }
} $panHeaders 60

$out.vision_health = Safe-Get "$VisionBase/api/health"
$out.vision_chat = Safe-Post "$VisionBase/api/chat" @{
    message = "Resume TikTok en una linea"
    url = "https://tiktok.com"
} @{} 60

$summary = [ordered]@{
    push_ok = [bool]$out.push_health.ok
    push_provider = if ($out.push_message.ok) { $out.push_message.data.provider } else { "" }
    panaderia_ok = [bool]$out.panaderia_health.ok
    panaderia_ai_provider = if ($out.panaderia_ai.ok) { $out.panaderia_ai.data.provider } else { "" }
    panaderia_ai_offline = if ($out.panaderia_ai.ok) { [bool]$out.panaderia_ai.data.offline } else { $null }
    vision_ok = [bool]$out.vision_health.ok
    vision_reply_preview = if ($out.vision_chat.ok) { [string]$out.vision_chat.data.reply } else { "" }
}

[pscustomobject]@{
    ok = [bool]($out.push_health.ok -and $out.panaderia_health.ok -and $out.vision_health.ok)
    summary = $summary
    details = $out
} | ConvertTo-Json -Depth 8
