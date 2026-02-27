# Mata proceso en el puerto del servicio, limpia cache y arranca de nuevo.
# Uso: .\restart_service_clean.ps1 -Service nexus [-ClearCache]
#      .\restart_service_clean.ps1 -Service push  -ClearCache
#      .\restart_service_clean.ps1 -Service all   -ClearCache
param(
    [Parameter(Mandatory=$true)]
    [ValidateSet("nexus","push","robot","all")]
    [string]$Service,
    [switch]$ClearCache
)

$ErrorActionPreference = "Stop"
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$FreePortScript = Join-Path $PSScriptRoot "free_port.ps1"
$StartNexusScript = Join-Path $PSScriptRoot "start_nexus.ps1"

function Free-Port {
    param([int]$Port)
    if (Test-Path $FreePortScript) {
        & $FreePortScript -Port $Port -Kill | Out-Null
    }
}

function Clear-ProjectCache {
    Write-Host "Limpiando cache (__pycache__, temp_models_cache, logs/*.log antiguos)..." -ForegroundColor Cyan
    $dirs = @(
        (Join-Path $RepoRoot "nexus\atlas_nexus"),
        (Join-Path $RepoRoot "modules"),
        (Join-Path $RepoRoot "atlas_adapter"),
        (Join-Path $RepoRoot "evolution_daemon.py" | Split-Path)
    )
    foreach ($d in $dirs) {
        if (Test-Path $d) {
            Get-ChildItem -Path $d -Recurse -Directory -Filter "__pycache__" -ErrorAction SilentlyContinue | Remove-Item -Recurse -Force -ErrorAction SilentlyContinue
        }
    }
    $tempCache = Join-Path $RepoRoot "temp_models_cache"
    if (Test-Path $tempCache) {
        Remove-Item -Path (Join-Path $tempCache "*") -Recurse -Force -ErrorAction SilentlyContinue
        Write-Host "  temp_models_cache vaciado." -ForegroundColor Gray
    }
    Write-Host "Cache limpiado." -ForegroundColor Green
}

function Start-NexusService {
    Free-Port -Port 8000
    if (Test-Path $StartNexusScript) {
        Start-Process powershell -ArgumentList "-NoProfile","-ExecutionPolicy","Bypass","-File",$StartNexusScript -WorkingDirectory $RepoRoot -WindowStyle Hidden
        Write-Host "NEXUS (8000) arrancado." -ForegroundColor Green
    } else {
        $nexusDir = $env:NEXUS_ATLAS_PATH
        if (-not $nexusDir) { $nexusDir = Join-Path $RepoRoot "nexus\atlas_nexus" }
        if (Test-Path (Join-Path $nexusDir "nexus.py")) {
            Start-Process python -ArgumentList "nexus.py","--mode","api" -WorkingDirectory $nexusDir -WindowStyle Hidden
            Write-Host "NEXUS (8000) arrancado (python)." -ForegroundColor Green
        }
    }
}

function Start-RobotService {
    Free-Port -Port 8002
    $robotPath = $env:NEXUS_ROBOT_PATH
    if (-not $robotPath) { $robotPath = Join-Path $RepoRoot "nexus\atlas_nexus_robot\backend" }
    if (Test-Path (Join-Path $robotPath "main.py")) {
        Start-Process python -ArgumentList "main.py" -WorkingDirectory $robotPath -WindowStyle Hidden
        Write-Host "Robot (8002) arrancado." -ForegroundColor Green
    }
}

function Start-PushService {
    Free-Port -Port 8791
    Set-Location $RepoRoot
    Write-Host "Iniciando PUSH (8791) en primer plano. Cierra con Ctrl+C." -ForegroundColor Yellow
    python -m uvicorn atlas_adapter.atlas_http_api:app --host 0.0.0.0 --port 8791
}

# ----- Main -----
if ($ClearCache) { Clear-ProjectCache }

switch ($Service) {
    "nexus" {
        Start-NexusService
    }
    "robot" {
        Start-RobotService
    }
    "push" {
        Start-PushService
    }
    "all" {
        Free-Port -Port 8000
        Free-Port -Port 8002
        Free-Port -Port 8791
        Start-NexusService
        Start-Sleep -Seconds 2
        Start-RobotService
        Write-Host "NEXUS y Robot arrancados. PUSH (dashboard) debe estar ya en ejecucion en 8791." -ForegroundColor Cyan
    }
}
