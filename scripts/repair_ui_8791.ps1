param(
    [string]$Url = "http://127.0.0.1:8791/ui",
    [switch]$NoKillChrome,
    [switch]$NoOpen
)

$ErrorActionPreference = "Stop"

function Write-Step([string]$msg) {
    Write-Host "[repair-ui] $msg" -ForegroundColor Cyan
}

function Test-Endpoint([string]$endpoint, [int]$timeoutSec = 8) {
    try {
        $r = Invoke-WebRequest -Uri $endpoint -UseBasicParsing -TimeoutSec $timeoutSec
        return @{
            ok = $true
            status = [int]$r.StatusCode
            contentType = [string]$r.Headers["Content-Type"]
            length = [int]($r.Content | Out-String).Length
            error = ""
        }
    } catch {
        return @{
            ok = $false
            status = 0
            contentType = ""
            length = 0
            error = $_.Exception.Message
        }
    }
}

function Test-EndpointWithRetry([string]$endpoint, [int]$timeoutSec = 8, [int]$attempts = 3) {
    for ($i = 1; $i -le $attempts; $i++) {
        $r = Test-Endpoint -endpoint $endpoint -timeoutSec $timeoutSec
        if ($r.ok) { return $r }
        if ($i -lt $attempts) { Start-Sleep -Milliseconds 700 }
    }
    return $r
}

function Clear-ChromeCache([bool]$killChrome) {
    $chromeUserData = Join-Path $env:LOCALAPPDATA "Google\Chrome\User Data"
    if (-not (Test-Path $chromeUserData)) {
        throw "No se encontró Chrome User Data en: $chromeUserData"
    }

    if ($killChrome) {
        Write-Step "Cerrando procesos de Chrome..."
        Get-Process chrome -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
        Start-Sleep -Milliseconds 400
    }

    $targets = @(
        (Join-Path $chromeUserData "Default\Cache"),
        (Join-Path $chromeUserData "Default\Code Cache"),
        (Join-Path $chromeUserData "Default\GPUCache"),
        (Join-Path $chromeUserData "Default\Service Worker\CacheStorage"),
        (Join-Path $chromeUserData "Default\Network\Cache")
    )

    foreach ($p in $targets) {
        if (Test-Path $p) {
            Remove-Item -Path $p -Recurse -Force -ErrorAction SilentlyContinue
            Write-Host "  CLEARED: $p" -ForegroundColor Green
        } else {
            Write-Host "  SKIP:    $p" -ForegroundColor DarkGray
        }
    }
}

Write-Step "Verificando servicio 8791..."
$urlCandidates = @($Url)
if ($Url -notmatch "127\.0\.0\.1") { $urlCandidates += "http://127.0.0.1:8791/ui" }
if ($Url -notmatch "localhost") { $urlCandidates += "http://localhost:8791/ui" }

$uiCheck = $null
$workingUrl = $null
foreach ($baseUrl in $urlCandidates) {
    $sep = "?"
    if ($baseUrl -match "\?") { $sep = "&" }
    $probe = ("{0}{1}precheck={2}" -f $baseUrl, $sep, [int](Get-Random))
    $uiCheck = Test-EndpointWithRetry -endpoint $probe -timeoutSec 8 -attempts 3
    if ($uiCheck.ok) {
        $workingUrl = $baseUrl
        break
    }
}

if (-not $uiCheck.ok) {
    Write-Host "ERROR /ui: $($uiCheck.error)" -ForegroundColor Red
    exit 2
}
if ($workingUrl) { $Url = $workingUrl }
Write-Host "  OK /ui -> status=$($uiCheck.status), type=$($uiCheck.contentType), url=$Url" -ForegroundColor Green

Write-Step "Limpiando cache de Chrome..."
Clear-ChromeCache -killChrome:(-not $NoKillChrome)

Write-Step "Validando módulos JS críticos..."
$assets = @(
    "http://127.0.0.1:8791/v4/static/lib/state.js",
    "http://127.0.0.1:8791/v4/static/components/landing.js",
    "http://127.0.0.1:8791/v4/static/app.js"
)
foreach ($a in $assets) {
    $chk = Test-Endpoint $a
    if ($chk.ok -and $chk.status -eq 200) {
        Write-Host "  OK $a" -ForegroundColor Green
    } else {
        Write-Host "  FAIL $a -> $($chk.error)" -ForegroundColor Yellow
    }
}

if (-not $NoOpen) {
    $openSep = "?"
    if ($Url -match "\?") { $openSep = "&" }
    $openUrl = ("{0}{1}cache_reset={2}" -f $Url, $openSep, [int](Get-Random))
    Write-Step "Abriendo Chrome limpio en: $openUrl"
    Start-Process "chrome.exe" $openUrl
}

Write-Step "Completado."
Write-Host "Sugerencia: si persiste, abre Chrome sin extensiones:" -ForegroundColor Yellow
Write-Host "  chrome.exe --user-data-dir=C:\temp\atlas-clean-profile --disable-extensions $Url" -ForegroundColor Yellow
