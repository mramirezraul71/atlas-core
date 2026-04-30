param(
    [string]$Domain = "midominio.com",
    [string]$ZoneName = "midominio.com",
    [string]$CredFile = "C:\dev\credenciales.txt",
    [string[]]$ExpectedNameservers = @("elsa.ns.cloudflare.com", "margo.ns.cloudflare.com"),
    [string[]]$ExpectedHosts = @("panaderia.midominio.com", "panaderia-api.midominio.com", "atlas-dashboard.midominio.com"),
    [string]$LogFile = "C:\ATLAS_PUSH\logs\cloudflare_ns_monitor.jsonl"
)

$ErrorActionPreference = "Stop"

function Get-CloudflareToken {
    if ($env:CLOUDFLARE_API_TOKEN -and $env:CLOUDFLARE_API_TOKEN.Trim()) {
        return $env:CLOUDFLARE_API_TOKEN.Trim()
    }
    if (-not (Test-Path $CredFile)) {
        return $null
    }
    $line = Get-Content -Path $CredFile | Where-Object { $_ -match '^CLOUDFLARE_API_TOKEN=' } | Select-Object -First 1
    if (-not $line) {
        return $null
    }
    return ($line -replace '^CLOUDFLARE_API_TOKEN=', '').Trim()
}

function Invoke-CfApi {
    param(
        [string]$Token,
        [string]$Path
    )
    $headers = @{
        Authorization = "Bearer $Token"
        "Content-Type" = "application/json"
    }
    $uri = "https://api.cloudflare.com/client/v4$Path"
    return Invoke-RestMethod -Method Get -Uri $uri -Headers $headers
}

function Normalize-NS {
    param([string[]]$Values)
    if (-not $Values) { return @() }
    return @($Values | ForEach-Object { $_.TrimEnd(".").ToLowerInvariant() } | Sort-Object -Unique)
}

function Get-PublicNameservers {
    param([string]$QueryDomain)
    try {
        $result = Resolve-DnsName -Type NS $QueryDomain -ErrorAction Stop |
            Select-Object -ExpandProperty NameHost
        return Normalize-NS -Values $result
    } catch {
        return @()
    }
}

function Get-ExpectedHostRecords {
    param(
        [string]$Token,
        [string]$ZoneId,
        [string[]]$Hosts
    )
    $resp = Invoke-CfApi -Token $Token -Path "/zones/$ZoneId/dns_records?per_page=200"
    $records = @($resp.result | Where-Object { $_.name -in $Hosts } | Select-Object name, type, content, proxied, ttl)
    return $records
}

$nowUtc = (Get-Date).ToUniversalTime().ToString("o")
$expectedNs = Normalize-NS -Values $ExpectedNameservers
$publicNs = Get-PublicNameservers -QueryDomain $Domain
$token = Get-CloudflareToken

$zoneStatus = "unknown"
$zoneId = $null
$cfAssignedNs = @()
$cfOriginalNs = @()
$expectedRecords = @()
$apiError = $null

if ($token) {
    try {
        $zonesResp = Invoke-CfApi -Token $token -Path "/zones?name=$([System.Uri]::EscapeDataString($ZoneName))"
        if ($zonesResp.result -and $zonesResp.result.Count -gt 0) {
            $zone = $zonesResp.result[0]
            $zoneStatus = [string]$zone.status
            $zoneId = [string]$zone.id
            $cfAssignedNs = Normalize-NS -Values @($zone.name_servers)
            $cfOriginalNs = Normalize-NS -Values @($zone.original_name_servers)
            $expectedRecords = Get-ExpectedHostRecords -Token $token -ZoneId $zoneId -Hosts $ExpectedHosts
        } else {
            $zoneStatus = "not_found"
        }
    } catch {
        $apiError = $_.Exception.Message
        $zoneStatus = "api_error"
    }
} else {
    $apiError = "CLOUDFLARE_API_TOKEN no encontrado en entorno ni credenciales."
    $zoneStatus = "no_token"
}

$publicNsReady = (@($publicNs) -join ",") -eq (@($expectedNs) -join ",")
$zoneActive = $zoneStatus -eq "active"
$ready = $publicNsReady -and $zoneActive

$payload = [ordered]@{
    timestamp_utc = $nowUtc
    domain = $Domain
    zone_name = $ZoneName
    zone_id = $zoneId
    zone_status = $zoneStatus
    expected_ns = $expectedNs
    public_ns = $publicNs
    cloudflare_assigned_ns = $cfAssignedNs
    cloudflare_original_ns = $cfOriginalNs
    public_ns_ready = $publicNsReady
    zone_active = $zoneActive
    ready = $ready
    expected_hosts_found = @($expectedRecords).Count
    expected_hosts = @($expectedRecords)
    api_error = $apiError
}

$logDir = Split-Path -Parent $LogFile
New-Item -ItemType Directory -Path $logDir -Force | Out-Null
($payload | ConvertTo-Json -Depth 8 -Compress) | Add-Content -Path $LogFile -Encoding UTF8

$flagPath = Join-Path $logDir "cloudflare_zone_active.flag"
if ($ready) {
    Set-Content -Path $flagPath -Encoding UTF8 -Value "READY $nowUtc"
} elseif (Test-Path $flagPath) {
    Remove-Item -Path $flagPath -Force -ErrorAction SilentlyContinue
}

Write-Output ("CLOUDFLARE_MONITOR timestamp={0} zone_status={1} public_ns_ready={2} ready={3}" -f $nowUtc, $zoneStatus, $publicNsReady, $ready)
if ($apiError) {
    Write-Output ("CLOUDFLARE_MONITOR api_error={0}" -f $apiError)
}
