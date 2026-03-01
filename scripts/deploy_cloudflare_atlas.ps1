param(
    [Parameter(Mandatory = $true)]
    [string]$TunnelId,
    [Parameter(Mandatory = $true)]
    [string]$CloudflareZone,
    [string]$PanaderiaHost = "panaderia",
    [string]$PanaderiaApiHost = "panaderia-api",
    [string]$VisionHost = "vision",
    [string]$AtlasHost = "atlas-dashboard",
    [switch]$SkipInstall
)

$ErrorActionPreference = "Stop"

function Write-Info([string]$m) { Write-Host "[INFO] $m" -ForegroundColor Cyan }
function Write-WarnMsg([string]$m) { Write-Host "[WARN] $m" -ForegroundColor Yellow }

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$cfgDir = Join-Path $repoRoot "config\\cloudflared"
$cfDir = Join-Path $repoRoot "cloudflare"
$logsDir = Join-Path $repoRoot "logs"
$binDir = Join-Path $repoRoot "tools\\bin"
$ipDir = Join-Path $repoRoot "config\\cloudflare"
New-Item -ItemType Directory -Path $cfgDir -Force | Out-Null
New-Item -ItemType Directory -Path $cfDir -Force | Out-Null
New-Item -ItemType Directory -Path $logsDir -Force | Out-Null
New-Item -ItemType Directory -Path $binDir -Force | Out-Null
New-Item -ItemType Directory -Path $ipDir -Force | Out-Null

$cloudflaredExe = Join-Path $binDir "cloudflared.exe"

function Install-CloudflaredLocal {
    if (Test-Path $cloudflaredExe) {
        Write-Info "cloudflared ya existe en $cloudflaredExe"
        return
    }
    $url = "https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-windows-amd64.exe"
    Write-Info "Descargando cloudflared desde $url"
    Invoke-WebRequest -Uri $url -OutFile $cloudflaredExe -UseBasicParsing
    Write-Info "cloudflared instalado en $cloudflaredExe"
}

function Update-CloudflareIPRanges {
    $v4Url = "https://www.cloudflare.com/ips-v4"
    $v6Url = "https://www.cloudflare.com/ips-v6"
    $v4Path = Join-Path $ipDir "ips-v4.txt"
    $v6Path = Join-Path $ipDir "ips-v6.txt"
    Invoke-WebRequest -Uri $v4Url -OutFile $v4Path -UseBasicParsing
    Invoke-WebRequest -Uri $v6Url -OutFile $v6Path -UseBasicParsing
    Write-Info "Rangos Cloudflare actualizados en $ipDir"
}

function Write-TunnelConfig {
    $cfgPath = Join-Path $cfgDir "cloudflare_atlas_config.yaml"
    $credPath = Join-Path $cfgDir "$TunnelId.json"
    $yaml = @"
tunnel: "$TunnelId"
credentials-file: "$credPath"
protocol: quic
loglevel: info
transport-loglevel: info
logfile: "$repoRoot\\logs\\cloudflared_access.log"
metrics: "127.0.0.1:50333"

ingress:
  - hostname: "$PanaderiaHost.$CloudflareZone"
    service: "http://127.0.0.1:5173"
  - hostname: "$PanaderiaApiHost.$CloudflareZone"
    service: "http://127.0.0.1:3001"
  - hostname: "$VisionHost.$CloudflareZone"
    service: "http://127.0.0.1:3000"
  - hostname: "$AtlasHost.$CloudflareZone"
    service: "http://127.0.0.1:8791"
  - service: "http_status:404"
"@
    Set-Content -Path $cfgPath -Value $yaml -Encoding Ascii
    Write-Info "Config YAML escrita en $cfgPath"
    return $cfgPath
}

function Write-WorkerTemplate {
    $workerDir = Join-Path $cfDir "worker"
    New-Item -ItemType Directory -Path $workerDir -Force | Out-Null
    $workerJs = Join-Path $workerDir "atlas_cuba_proxy_worker.js"
    $wrangler = Join-Path $workerDir "wrangler.toml"

    $js = @'
export default {
  async fetch(request, env) {
    const url = new URL(request.url);
    const target = url.searchParams.get("url");
    if (!target) {
      return new Response("Missing ?url=", { status: 400 });
    }
    const parsed = new URL(target);
    const host = parsed.hostname.toLowerCase();
    const allowed = (env.ALLOWED_HOSTS || "").split(",").map(s => s.trim().toLowerCase()).filter(Boolean);
    if (allowed.length && !allowed.includes(host)) {
      return new Response("Host not allowed", { status: 403 });
    }
    const upstreamReq = new Request(target, {
      method: request.method,
      headers: request.headers,
      body: request.body,
      redirect: "follow",
    });
    const res = await fetch(upstreamReq, { cf: { cacheEverything: false } });
    const out = new Response(res.body, res);
    out.headers.set("x-atlas-worker", "atlas-cuba-proxy");
    return out;
  }
};
'@
    $toml = @'
name = "atlas-cuba-proxy"
main = "atlas_cuba_proxy_worker.js"
compatibility_date = "2026-03-01"

[vars]
# Separar por comas, ej: "github.com,registry.npmjs.org,files.pythonhosted.org"
ALLOWED_HOSTS = ""
'@
    Set-Content -Path $workerJs -Value $js -Encoding Ascii
    Set-Content -Path $wrangler -Value $toml -Encoding Ascii
    Write-Info "Plantilla Worker creada en $workerDir"
}

if (-not $SkipInstall) {
    Install-CloudflaredLocal
} else {
    Write-WarnMsg "SkipInstall activo: no se instala cloudflared."
}

Update-CloudflareIPRanges
$cfgPath = Write-TunnelConfig
Write-WorkerTemplate

if (Test-Path $cloudflaredExe) {
    try {
        & $cloudflaredExe tunnel ingress validate --config $cfgPath | Out-Null
        Write-Info "Validacion ingress: OK"
    } catch {
        Write-WarnMsg "No se pudo validar ingress con cloudflared local: $($_.Exception.Message)"
    }
} else {
    Write-WarnMsg "cloudflared no instalado localmente. Valida luego con: cloudflared tunnel ingress validate --config `"$cfgPath`""
}

# No iniciar tunel aqui, solo preparar artefactos.
Write-Output "ATLAS_CLOUDFLARE_PREP_OK config=$cfgPath"
