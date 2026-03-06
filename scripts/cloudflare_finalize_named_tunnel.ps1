param(
    [string]$TunnelName = "atlas-rauli-main",
    [string]$AccountId = "",
    [string]$ZoneName = "",
    [string]$PanaderiaHost = "panaderia",
    [string]$PanaderiaApiHost = "panaderia-api",
    [string]$DashboardHost = "atlas-dashboard",
    [string]$CredFile = "C:\dev\credenciales.txt",
    [switch]$SkipCredWrite
)

$ErrorActionPreference = "Stop"

function Write-Info([string]$msg) { Write-Host "[INFO] $msg" -ForegroundColor Cyan }
function Write-Ok([string]$msg) { Write-Host "[OK]   $msg" -ForegroundColor Green }
function Write-WarnMsg([string]$msg) { Write-Host "[WARN] $msg" -ForegroundColor Yellow }

function Get-Token {
    if ($env:CLOUDFLARE_API_TOKEN -and $env:CLOUDFLARE_API_TOKEN.Trim()) {
        return $env:CLOUDFLARE_API_TOKEN.Trim()
    }
    if (Test-Path $CredFile) {
        $line = Get-Content $CredFile | Where-Object { $_ -match '^CLOUDFLARE_API_TOKEN=' } | Select-Object -First 1
        if ($line) {
            return ($line -replace '^CLOUDFLARE_API_TOKEN=', '').Trim()
        }
    }
    throw "No encontré CLOUDFLARE_API_TOKEN en env ni en $CredFile"
}

function Get-CredValue {
    param(
        [string]$Key
    )
    if (-not (Test-Path $CredFile)) { return $null }
    $line = Get-Content $CredFile | Where-Object { $_ -match "^$([regex]::Escape($Key))=" } | Select-Object -First 1
    if (-not $line) { return $null }
    return ($line -replace "^$([regex]::Escape($Key))=", '').Trim()
}

function Invoke-CfApi {
    param(
        [string]$Method,
        [string]$Path,
        [object]$Body = $null
    )
    $uri = "https://api.cloudflare.com/client/v4$Path"
    $headers = @{
        Authorization = "Bearer $script:CfToken"
        "Content-Type" = "application/json"
    }
    try {
        if ($null -ne $Body) {
            $payload = ($Body | ConvertTo-Json -Depth 12 -Compress)
            $resp = Invoke-RestMethod -Method $Method -Uri $uri -Headers $headers -Body $payload
        } else {
            $resp = Invoke-RestMethod -Method $Method -Uri $uri -Headers $headers
        }
    } catch {
        $statusCode = $null
        try { $statusCode = [int]$_.Exception.Response.StatusCode } catch {}
        $msg = $_.Exception.Message
        throw "Cloudflare API $Method $Path falló (status=$statusCode): $msg"
    }

    if (-not $resp.success) {
        $errs = ""
        try { $errs = ($resp.errors | ConvertTo-Json -Compress) } catch { $errs = "unknown_error" }
        throw "Cloudflare API $Method $Path success=false errors=$errs"
    }
    return $resp
}

function Ensure-DnsCname {
    param(
        [string]$ZoneId,
        [string]$Name,
        [string]$Target
    )
    $qName = [System.Uri]::EscapeDataString($Name)
    $existing = Invoke-CfApi -Method "GET" -Path "/zones/$ZoneId/dns_records?type=CNAME&name=$qName"
    $record = $null
    if ($existing.result -and $existing.result.Count -gt 0) {
        $record = $existing.result[0]
    }

    $body = @{
        type = "CNAME"
        name = $Name
        content = $Target
        proxied = $true
        ttl = 1
    }

    if ($record) {
        $needsUpdate = ($record.content -ne $Target) -or (-not [bool]$record.proxied)
        if ($needsUpdate) {
            $null = Invoke-CfApi -Method "PUT" -Path "/zones/$ZoneId/dns_records/$($record.id)" -Body $body
            Write-Ok "DNS actualizado: $Name -> $Target"
        } else {
            Write-Ok "DNS ya correcto: $Name -> $Target"
        }
    } else {
        $null = Invoke-CfApi -Method "POST" -Path "/zones/$ZoneId/dns_records" -Body $body
        Write-Ok "DNS creado: $Name -> $Target"
    }
}

function Upsert-CredValue {
    param(
        [string]$Key,
        [string]$Value
    )
    if (-not (Test-Path $CredFile)) {
        throw "No existe archivo de credenciales: $CredFile"
    }
    $lines = Get-Content -Path $CredFile
    $prefix = "$Key="
    $updated = $false
    for ($i = 0; $i -lt $lines.Count; $i++) {
        if ($lines[$i].StartsWith($prefix)) {
            $lines[$i] = "$Key=$Value"
            $updated = $true
            break
        }
    }
    if (-not $updated) {
        $lines += "$Key=$Value"
    }
    Set-Content -Path $CredFile -Value $lines -Encoding UTF8
}

$script:CfToken = Get-Token
Write-Info "Validando token Cloudflare..."
$verify = Invoke-CfApi -Method "GET" -Path "/user/tokens/verify"
Write-Ok "Token activo: $($verify.result.status)"

$accountIdCandidate = $AccountId
if (-not $accountIdCandidate) {
    if ($env:CLOUDFLARE_ACCOUNT_ID -and $env:CLOUDFLARE_ACCOUNT_ID.Trim()) {
        $accountIdCandidate = $env:CLOUDFLARE_ACCOUNT_ID.Trim()
    }
}
if (-not $accountIdCandidate) {
    $accountIdCandidate = Get-CredValue -Key "CLOUDFLARE_ACCOUNT_ID"
}

if ($accountIdCandidate) {
    $accountId = [string]$accountIdCandidate
    Write-Info "Cuenta seleccionada por AccountId explícito: $accountId"
} else {
    $accountsResp = Invoke-CfApi -Method "GET" -Path "/accounts"
    if (-not $accountsResp.result -or $accountsResp.result.Count -eq 0) {
        throw "El token no expone ninguna cuenta Cloudflare. Pasa -AccountId o define CLOUDFLARE_ACCOUNT_ID en env/credenciales."
    }
    $account = $accountsResp.result[0]
    $accountId = [string]$account.id
    Write-Info "Cuenta seleccionada: $($account.name) ($accountId)"
}

Write-Info "Comprobando permisos de Tunnel API..."
try {
    $null = Invoke-CfApi -Method "GET" -Path "/accounts/$accountId/cfd_tunnel?per_page=1"
} catch {
    throw @"
No se puede acceder a Cloudflare Tunnel API con este token.
Necesitas un token con scopes:
- Account: Cloudflare Tunnel Edit
- Zone: DNS Edit
- Zone: Zone Read
Detalle: $($_.Exception.Message)
"@
}

$zonesResp = Invoke-CfApi -Method "GET" -Path "/zones?per_page=50"
$zones = @($zonesResp.result)
if ($ZoneName) {
    $zones = @($zones | Where-Object { $_.name -eq $ZoneName })
}
if (-not $zones -or $zones.Count -eq 0) {
    throw "No hay zonas DNS disponibles para el token. Define una zona en Cloudflare o usa -ZoneName con una válida."
}
$zone = $zones[0]
$zoneId = [string]$zone.id
$zoneNameFinal = [string]$zone.name
Write-Info "Zona seleccionada: $zoneNameFinal ($zoneId)"

$listTunnel = Invoke-CfApi -Method "GET" -Path "/accounts/$accountId/cfd_tunnel?name=$([System.Uri]::EscapeDataString($TunnelName))"
$tunnel = $null
if ($listTunnel.result -and $listTunnel.result.Count -gt 0) {
    $tunnel = $listTunnel.result[0]
    Write-Ok "Tunnel existente encontrado: $($tunnel.id)"
} else {
    $rng = New-Object byte[] 32
    try {
        [System.Security.Cryptography.RandomNumberGenerator]::Fill($rng)
    } catch {
        $crypto = [System.Security.Cryptography.RNGCryptoServiceProvider]::new()
        $crypto.GetBytes($rng)
        $crypto.Dispose()
    }
    $secret = [Convert]::ToBase64String($rng)
    $createBody = @{
        name = $TunnelName
        tunnel_secret = $secret
        config_src = "cloudflare"
    }
    $created = Invoke-CfApi -Method "POST" -Path "/accounts/$accountId/cfd_tunnel" -Body $createBody
    $tunnel = $created.result
    Write-Ok "Tunnel creado: $($tunnel.id)"
}

$tunnelId = [string]$tunnel.id
$target = "$tunnelId.cfargotunnel.com"

$fqdnPan = "$PanaderiaHost.$zoneNameFinal"
$fqdnPanApi = "$PanaderiaApiHost.$zoneNameFinal"
$fqdnDash = "$DashboardHost.$zoneNameFinal"

Ensure-DnsCname -ZoneId $zoneId -Name $fqdnPan -Target $target
Ensure-DnsCname -ZoneId $zoneId -Name $fqdnPanApi -Target $target
Ensure-DnsCname -ZoneId $zoneId -Name $fqdnDash -Target $target

$tokenResp = Invoke-CfApi -Method "GET" -Path "/accounts/$accountId/cfd_tunnel/$tunnelId/token"
$tunnelRunToken = [string]$tokenResp.result
if (-not $tunnelRunToken) {
    throw "No pude obtener token de ejecución del túnel."
}

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$cfgDir = Join-Path $repoRoot "config\cloudflared"
New-Item -ItemType Directory -Path $cfgDir -Force | Out-Null
$cfgPath = Join-Path $cfgDir "cloudflare_atlas_config.yaml"
$credPath = Join-Path $cfgDir "$tunnelId.json"
$yaml = @"
tunnel: "$tunnelId"
credentials-file: "$credPath"
protocol: quic
loglevel: info
transport-loglevel: info
logfile: "$repoRoot\logs\cloudflared_access.log"
metrics: "127.0.0.1:50333"

ingress:
  - hostname: "$fqdnPan"
    service: "http://127.0.0.1:5173"
  - hostname: "$fqdnPanApi"
    service: "http://127.0.0.1:3001"
  - hostname: "$fqdnDash"
    service: "http://127.0.0.1:8791"
  - service: "http_status:404"
"@
Set-Content -Path $cfgPath -Value $yaml -Encoding Ascii
Write-Ok "Config actualizada: $cfgPath"

if (-not $SkipCredWrite) {
    Upsert-CredValue -Key "ATLAS_PANADERIA_APP_URL" -Value ("https://$fqdnPan")
    Upsert-CredValue -Key "ATLAS_PANADERIA_PUBLIC_URL" -Value ("https://$fqdnPan")
    Upsert-CredValue -Key "ATLAS_PANADERIA_PUBLIC_API_URL" -Value ("https://$fqdnPanApi")
    Upsert-CredValue -Key "ATLAS_DASHBOARD_PUBLIC_URL" -Value ("https://$fqdnDash")
    Write-Ok "Credenciales públicas actualizadas en $CredFile"
}

Write-Host ""
Write-Ok "Cloudflare finalizado."
Write-Host "Tunnel ID: $tunnelId"
Write-Host "Panaderia: https://$fqdnPan"
Write-Host "Panaderia API: https://$fqdnPanApi"
Write-Host "Dashboard: https://$fqdnDash"
Write-Host ""
Write-Host "Para correr túnel nombrado:"
Write-Host "cloudflared tunnel run --token <TOKEN_OCULTO>"
Write-Host "TOKEN len: $($tunnelRunToken.Length) (oculto por seguridad)"
