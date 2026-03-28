param(
    [string]$GrafanaUrl = "http://127.0.0.1:3003",
    [string]$User = "admin",
    [string]$Password = "atlas2026",
    [switch]$SkipProvisioningRebuild,
    [string]$ReportPath = "C:\ATLAS_PUSH\reports\atlas_grafana_provisioning_check_latest.json"
)

$ErrorActionPreference = "Stop"
$Root = "C:\ATLAS_PUSH"
$VenvPy = "$Root\venv\Scripts\python.exe"

Set-Location $Root

function New-BasicAuthHeader([string]$Username, [string]$Secret) {
    $token = [Convert]::ToBase64String([Text.Encoding]::ASCII.GetBytes("${Username}:${Secret}"))
    return @{
        Authorization = "Basic $token"
        "Content-Type" = "application/json"
    }
}

function Invoke-GrafanaJson {
    param(
        [string]$Path,
        [string]$Method = "GET",
        [hashtable]$Headers,
        [object]$Body = $null
    )

    $uri = "$($GrafanaUrl.TrimEnd('/'))$Path"
    if ($null -eq $Body) {
        return Invoke-RestMethod -Uri $uri -Method $Method -Headers $Headers -UseBasicParsing
    }

    $payload = if ($Body -is [string]) { $Body } else { $Body | ConvertTo-Json -Depth 12 -Compress }
    return Invoke-RestMethod -Uri $uri -Method $Method -Headers $Headers -Body $payload -UseBasicParsing
}

function ConvertTo-Array($Value) {
    if ($null -eq $Value) {
        return @()
    }
    if ($Value -is [System.Array]) {
        return $Value
    }
    if ($Value.PSObject.Properties.Name -contains "value") {
        return @(($Value.value))
    }
    return @($Value)
}

function New-StepResult {
    param(
        [string]$Name,
        [scriptblock]$Action
    )

    try {
        $value = & $Action
        return [ordered]@{
            ok = $true
            step = $Name
            value = $value
        }
    } catch {
        $details = $_.ErrorDetails.Message
        if (-not $details) {
            $details = $_.Exception.Message
        }
        return [ordered]@{
            ok = $false
            step = $Name
            error = $details
        }
    }
}

$headers = New-BasicAuthHeader -Username $User -Secret $Password
$telegramEnvReady = -not [string]::IsNullOrWhiteSpace($env:TELEGRAM_BOT_TOKEN) -and -not [string]::IsNullOrWhiteSpace($env:TELEGRAM_ADMIN_CHAT_ID)

$provisioningRebuild = [ordered]@{
    ok = $true
    skipped = [bool]$SkipProvisioningRebuild
}

if (-not $SkipProvisioningRebuild) {
    try {
        $previousBotToken = $env:TELEGRAM_BOT_TOKEN
        $previousChatId = $env:TELEGRAM_ADMIN_CHAT_ID
        try {
            if (-not $telegramEnvReady) {
                $env:TELEGRAM_BOT_TOKEN = ""
                $env:TELEGRAM_ADMIN_CHAT_ID = ""
            }

            $output = & $VenvPy -c "import sys; sys.path.insert(0, r'C:\ATLAS_PUSH'); sys.path.insert(0, r'C:\ATLAS_PUSH\atlas_code_quant'); from atlas_code_quant.production.grafana_dashboard import GrafanaDashboard; GrafanaDashboard().save_provisioning(); print('provisioning_ok')" 2>&1
        } finally {
            if ($null -eq $previousBotToken) {
                Remove-Item Env:TELEGRAM_BOT_TOKEN -ErrorAction SilentlyContinue
            } else {
                $env:TELEGRAM_BOT_TOKEN = $previousBotToken
            }
            if ($null -eq $previousChatId) {
                Remove-Item Env:TELEGRAM_ADMIN_CHAT_ID -ErrorAction SilentlyContinue
            } else {
                $env:TELEGRAM_ADMIN_CHAT_ID = $previousChatId
            }
        }

        if ($LASTEXITCODE -ne 0) {
            throw (($output | Out-String).Trim())
        }
        $provisioningRebuild.output = (($output | Out-String).Trim())
    } catch {
        $provisioningRebuild.ok = $false
        $provisioningRebuild.error = $_.Exception.Message
    }
}

if (-not $telegramEnvReady) {
    $contactPointsFallback = "apiVersion: 1`ncontactPoints: []`n"
    $policiesFallback = @"
apiVersion: 1
policies:
  - orgId: 1
    receiver: grafana-default-email
    group_by: ['alertname', 'component', 'severity']
    group_wait: 30s
    group_interval: 5m
    repeat_interval: 4h
"@
    foreach ($base in @("grafana\provisioning\alerting", "grafana\local\provisioning\alerting")) {
        $targetDir = Join-Path $Root $base
        New-Item -ItemType Directory -Path $targetDir -Force | Out-Null
        Set-Content -Path (Join-Path $targetDir "atlas_contact_points.yaml") -Value $contactPointsFallback -Encoding UTF8
        Set-Content -Path (Join-Path $targetDir "atlas_notification_policies.yaml") -Value $policiesFallback -Encoding UTF8
    }
}

$health = New-StepResult -Name "health" -Action {
    Invoke-GrafanaJson -Path "/api/health" -Headers $headers
}

$reload = [ordered]@{
    datasources = (New-StepResult -Name "datasources_reload" -Action {
        Invoke-GrafanaJson -Path "/api/admin/provisioning/datasources/reload" -Method "POST" -Headers $headers
    })
    dashboards = (New-StepResult -Name "dashboards_reload" -Action {
        Invoke-GrafanaJson -Path "/api/admin/provisioning/dashboards/reload" -Method "POST" -Headers $headers
    })
    alerting = (New-StepResult -Name "alerting_reload" -Action {
        Invoke-GrafanaJson -Path "/api/admin/provisioning/alerting/reload" -Method "POST" -Headers $headers
    })
}

$datasourceCheck = New-StepResult -Name "datasource_check" -Action {
    Invoke-GrafanaJson -Path "/api/datasources/name/atlas-prometheus" -Headers $headers
}
$dashboardSearch = New-StepResult -Name "dashboard_search" -Action {
    Invoke-GrafanaJson -Path "/api/search?query=ATLAS" -Headers $headers
}
$contactPointsCheck = New-StepResult -Name "contact_points" -Action {
    Invoke-GrafanaJson -Path "/api/v1/provisioning/contact-points" -Headers $headers
}
$policiesCheck = New-StepResult -Name "policies" -Action {
    Invoke-GrafanaJson -Path "/api/v1/provisioning/policies" -Headers $headers
}

$dashboardItems = if ($dashboardSearch.ok) { ConvertTo-Array $dashboardSearch.value } else { @() }
$contactPointItems = if ($contactPointsCheck.ok) { ConvertTo-Array $contactPointsCheck.value } else { @() }
$policyRoutes = @()
if ($policiesCheck.ok -and $policiesCheck.value.routes) {
    $policyRoutes = ConvertTo-Array $policiesCheck.value.routes
}

$dashboardUids = @($dashboardItems | ForEach-Object { $_.uid } | Where-Object { $_ })
$contactPointNames = @($contactPointItems | ForEach-Object { $_.name } | Where-Object { $_ })
$routeReceivers = @($policyRoutes | ForEach-Object { $_.receiver } | Where-Object { $_ })
$atlasTelegramPresent = $contactPointNames -contains "atlas-telegram"
$atlasPolicyRoutePresent = $routeReceivers -contains "atlas-telegram"
$datasourceOk = $datasourceCheck.ok -and ($datasourceCheck.value.uid -eq "atlas-prom")
$dashboardsOk = ($dashboardUids -contains "atlas-quant-pro-2026") -and ($dashboardUids -contains "atlas-quant-main")
$alertingReady = if ($telegramEnvReady) { $atlasTelegramPresent -and $atlasPolicyRoutePresent } else { $policiesCheck.ok }

$overallStatus = if (
    $health.ok -and
    $datasourceOk -and
    $dashboardsOk -and
    $alertingReady -and
    $provisioningRebuild.ok
) {
    "ready"
} elseif ($health.ok) {
    "degraded"
} else {
    "failed"
}

$report = [ordered]@{
    checked_at = (Get-Date).ToString("o")
    grafana_url = $GrafanaUrl
    grafana_version = if ($health.ok) { $health.value.version } else { $null }
    provisioning_rebuild = $provisioningRebuild
    telegram_env_ready = $telegramEnvReady
    overall_status = $overallStatus
    reload = $reload
    verification = [ordered]@{
        datasource_ok = $datasourceOk
        dashboards_ok = $dashboardsOk
        alerting_ready = $alertingReady
        dashboard_uids = $dashboardUids
        contact_point_names = $contactPointNames
        policy_root_receiver = if ($policiesCheck.ok) { $policiesCheck.value.receiver } else { $null }
        policy_route_receivers = $routeReceivers
        atlas_telegram_present = $atlasTelegramPresent
        atlas_policy_route_present = $atlasPolicyRoutePresent
    }
    raw = [ordered]@{
        health = $health
        datasource = $datasourceCheck
        dashboards = $dashboardSearch
        contact_points = $contactPointsCheck
        policies = $policiesCheck
    }
}

$reportDir = Split-Path -Parent $ReportPath
if ($reportDir) {
    New-Item -ItemType Directory -Path $reportDir -Force | Out-Null
}
$report | ConvertTo-Json -Depth 12 | Set-Content -Path $ReportPath -Encoding UTF8

Write-Host ""
Write-Host "============================================" -ForegroundColor Cyan
Write-Host "  ATLAS Grafana Provisioning Check" -ForegroundColor Cyan
Write-Host "============================================" -ForegroundColor Cyan
Write-Host "  Grafana         : $GrafanaUrl"
Write-Host "  Version         : $($report.grafana_version)"
Write-Host "  Overall status  : $overallStatus"
Write-Host "  Telegram env    : $telegramEnvReady"
Write-Host "  Datasource OK   : $datasourceOk"
Write-Host "  Dashboards OK   : $dashboardsOk"
Write-Host "  Alerting ready  : $alertingReady"
Write-Host "  Contact points  : $([string]::Join(', ', $contactPointNames))"
Write-Host "  Policy receivers: $([string]::Join(', ', $routeReceivers))"
Write-Host "  Report          : $ReportPath"
Write-Host "============================================" -ForegroundColor Cyan

if ($overallStatus -eq "failed") {
    exit 1
}
