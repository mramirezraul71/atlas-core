[CmdletBinding()]
param(
    [string]$BaseUrl = "http://localhost:8795",
    [string]$HealthPath = "/health",
    [string]$FallbackHealthPath = "/api/health",
    [string]$FunctionalPath = "/ui",
    [string]$BindHost = "0.0.0.0",
    [int]$Port = 8795,
    [string]$AppModule = "atlas_code_quant.api.main:app",
    [string]$PythonExe = "python",
    [int]$HealthTimeoutSec = 5,
    [int]$PostStartRetries = 30,
    [int]$PostStartDelaySeconds = 2,
    [int]$MaxLogSizeMb = 25,
    [int]$MaxLogArchives = 8,
    [switch]$Silent,
    [switch]$CheckOnly
)

$ErrorActionPreference = "Stop"

function Write-Info([string]$Message) {
    if ($Silent) {
        return
    }
    Write-Host "[atlas-8795-watchdog] $Message"
}

function Resolve-PythonExe {
    param(
        [string]$RepoRoot,
        [string]$RequestedPythonExe
    )
    $pushVenvPython = Join-Path $RepoRoot ".venv_push\Scripts\python.exe"
    if (Test-Path $pushVenvPython) {
        return $pushVenvPython
    }
    $venvPython = Join-Path $RepoRoot ".venv\Scripts\python.exe"
    if (Test-Path $venvPython) {
        return $venvPython
    }
    return $RequestedPythonExe
}

function Ensure-PythonImport {
    param(
        [string]$PythonPath,
        [string]$ModuleName,
        [string]$PipPackage
    )
    $probe = "import importlib.util; raise SystemExit(0 if importlib.util.find_spec('$ModuleName') else 1)"
    & $PythonPath -c $probe | Out-Null
    if ($LASTEXITCODE -eq 0) {
        return
    }
    Write-Info "Dependencia faltante ($ModuleName). Instalando paquete $PipPackage..."
    & $PythonPath -m pip install $PipPackage --disable-pip-version-check | Out-Null
    if ($LASTEXITCODE -ne 0) {
        throw "No se pudo instalar dependencia $PipPackage con $PythonPath"
    }
}

function Ensure-TimezoneData {
    param([string]$PythonPath)
    $tzProbe = "from zoneinfo import ZoneInfo; ZoneInfo('America/New_York'); print('ok')"
    & $PythonPath -c $tzProbe | Out-Null
    if ($LASTEXITCODE -eq 0) {
        return
    }
    Write-Info "Falta tzdata para ZoneInfo. Instalando tzdata en runtime..."
    & $PythonPath -m pip install tzdata --disable-pip-version-check | Out-Null
    if ($LASTEXITCODE -ne 0) {
        throw "No se pudo instalar tzdata con $PythonPath"
    }
}

function Ensure-StartupDependencies {
    param([string]$PythonPath)
    Ensure-PythonImport -PythonPath $PythonPath -ModuleName "pandas" -PipPackage "pandas"
    Ensure-PythonImport -PythonPath $PythonPath -ModuleName "duckdb" -PipPackage "duckdb"
    Ensure-PythonImport -PythonPath $PythonPath -ModuleName "xgboost" -PipPackage "xgboost"
    Ensure-TimezoneData -PythonPath $PythonPath
}

function Rotate-LogIfNeeded {
    param(
        [string]$Path,
        [int]$MaxSizeMb,
        [int]$MaxArchives
    )
    if (-not (Test-Path $Path)) {
        return
    }
    $file = Get-Item -Path $Path -ErrorAction SilentlyContinue
    if (-not $file) {
        return
    }
    $maxBytes = [int64]$MaxSizeMb * 1MB
    if ($file.Length -lt $maxBytes) {
        return
    }
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $archivePath = "$Path.$stamp"
    Move-Item -Path $Path -Destination $archivePath -Force
    Write-Info "Log rotado: $archivePath"

    $archives = Get-ChildItem -Path "$Path.*" -File -ErrorAction SilentlyContinue |
        Sort-Object LastWriteTime -Descending
    if (-not $archives) {
        return
    }
    $stale = $archives | Select-Object -Skip $MaxArchives
    foreach ($oldFile in $stale) {
        Remove-Item -Path $oldFile.FullName -Force -ErrorAction SilentlyContinue
    }
}

function Test-Health([string]$Url, [int]$TimeoutSec) {
    try {
        $resp = Invoke-WebRequest -Method Get -Uri $Url -TimeoutSec $TimeoutSec -UseBasicParsing
        return ($resp.StatusCode -ge 200 -and $resp.StatusCode -lt 300)
    } catch {
        return $false
    }
}

function Test-PortListening([int]$TargetPort) {
    try {
        $client = New-Object System.Net.Sockets.TcpClient
        $async = $client.BeginConnect("127.0.0.1", $TargetPort, $null, $null)
        $ok = $async.AsyncWaitHandle.WaitOne(1200, $false)
        if (-not $ok) {
            $client.Close()
            return $false
        }
        $client.EndConnect($async) | Out-Null
        $client.Close()
        return $true
    } catch {
        return $false
    }
}

function Test-ServiceHealthy {
    param(
        [int]$TargetPort,
        [string]$PrimaryUrl,
        [string]$SecondaryUrl,
        [string]$FunctionalUrl,
        [int]$TimeoutSec
    )
    if (-not (Test-PortListening -TargetPort $TargetPort)) {
        return $false
    }

    $healthOk = Test-Health -Url $PrimaryUrl -TimeoutSec $TimeoutSec
    if (-not $healthOk -and -not [string]::IsNullOrWhiteSpace($SecondaryUrl)) {
        $healthOk = Test-Health -Url $SecondaryUrl -TimeoutSec $TimeoutSec
    }
    if (-not $healthOk) {
        return $false
    }

    if ([string]::IsNullOrWhiteSpace($FunctionalUrl)) {
        return $true
    }
    return (Test-Health -Url $FunctionalUrl -TimeoutSec $TimeoutSec)
}

function Get-ListenerPid([int]$TargetPort) {
    try {
        $listener = Get-NetTCPConnection -LocalPort $TargetPort -State Listen -ErrorAction Stop | Select-Object -First 1
        if ($listener) {
            return [int]$listener.OwningProcess
        }
    } catch {
        return $null
    }
    return $null
}

function Stop-StalePortProcess([int]$TargetPort) {
    $listenerPid = Get-ListenerPid -TargetPort $TargetPort
    if (-not $listenerPid) {
        return
    }
    Write-Info "Se detecto proceso escuchando en $TargetPort (PID=$listenerPid). Terminando para reinicio limpio."
    try {
        Stop-Process -Id $listenerPid -Force -ErrorAction Stop
    } catch {
        Write-Info "No se pudo cerrar PID=$listenerPid automaticamente: $($_.Exception.Message)"
    }
}

function Stop-MatchingUvicorn([string]$Module, [int]$TargetPort) {
    $processes = Get-CimInstance Win32_Process -Filter "Name='python.exe'" -ErrorAction SilentlyContinue
    if (-not $processes) {
        return
    }
    foreach ($proc in $processes) {
        $cmd = $proc.CommandLine
        if (-not $cmd) { continue }
        if ($cmd -like "*uvicorn*" -and $cmd -like "*$Module*" -and $cmd -like "*--port $TargetPort*") {
            Write-Info "Cerrando uvicorn previo PID=$($proc.ProcessId)."
            try {
                Stop-Process -Id $proc.ProcessId -Force -ErrorAction Stop
            } catch {
                Write-Info "No se pudo cerrar PID=$($proc.ProcessId): $($_.Exception.Message)"
            }
        }
    }
}

$healthUrl = "{0}{1}" -f $BaseUrl.TrimEnd("/"), $HealthPath
$fallbackUrl = "{0}{1}" -f $BaseUrl.TrimEnd("/"), $FallbackHealthPath
$functionalUrl = "{0}{1}" -f $BaseUrl.TrimEnd("/"), $FunctionalPath
$repoRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot "..\.."))
$resolvedPythonExe = Resolve-PythonExe -RepoRoot $repoRoot -RequestedPythonExe $PythonExe

if (Test-ServiceHealthy -TargetPort $Port -PrimaryUrl $healthUrl -SecondaryUrl $fallbackUrl -FunctionalUrl $functionalUrl -TimeoutSec $HealthTimeoutSec) {
    Write-Info "Servicio 8795 saludable (puerto + health + funcional: $functionalUrl)."
    exit 0
}

if ($CheckOnly) {
    Write-Info "CheckOnly activo: servicio no saludable, sin reinicio."
    exit 1
}

Write-Info "Servicio 8795 no saludable. Ejecutando recuperacion automatica."
Stop-MatchingUvicorn -Module $AppModule -TargetPort $Port
Stop-StalePortProcess -TargetPort $Port
Start-Sleep -Seconds 1

$logDir = Join-Path $PSScriptRoot "..\..\logs"
$logDir = [System.IO.Path]::GetFullPath($logDir)
New-Item -ItemType Directory -Path $logDir -Force | Out-Null

$outLog = Join-Path $logDir "atlas_8795_uvicorn.out.log"
$errLog = Join-Path $logDir "atlas_8795_uvicorn.err.log"
Rotate-LogIfNeeded -Path $outLog -MaxSizeMb $MaxLogSizeMb -MaxArchives $MaxLogArchives
Rotate-LogIfNeeded -Path $errLog -MaxSizeMb $MaxLogSizeMb -MaxArchives $MaxLogArchives

$args = @(
    "-m", "uvicorn", $AppModule,
    "--host", $BindHost,
    "--port", "$Port"
)

$cmdForTrace = "$resolvedPythonExe " + ($args -join " ")
Write-Info "Lanzando: $cmdForTrace"
Ensure-StartupDependencies -PythonPath $resolvedPythonExe

$process = Start-Process -FilePath $resolvedPythonExe `
    -ArgumentList $args `
    -WorkingDirectory $repoRoot `
    -RedirectStandardOutput $outLog `
    -RedirectStandardError $errLog `
    -WindowStyle Hidden `
    -PassThru

Write-Info "Nuevo uvicorn PID=$($process.Id). Esperando health check."

for ($attempt = 1; $attempt -le $PostStartRetries; $attempt++) {
    Start-Sleep -Seconds $PostStartDelaySeconds
    if (Test-ServiceHealthy -TargetPort $Port -PrimaryUrl $healthUrl -SecondaryUrl $fallbackUrl -FunctionalUrl $functionalUrl -TimeoutSec $HealthTimeoutSec) {
        Write-Info "Recuperacion OK. Servicio activo en intento $attempt."
        exit 0
    }
    Write-Info "Sin health check aun (intento $attempt/$PostStartRetries)."
}

Write-Info "Fallo de recuperacion. Revisa logs:"
Write-Info "STDOUT: $outLog"
Write-Info "STDERR: $errLog"
exit 2

# Uso manual:
#   powershell -ExecutionPolicy Bypass -File .\scripts\windows\atlas_quant_8795_watchdog.ps1
#   powershell -ExecutionPolicy Bypass -File .\scripts\windows\atlas_quant_8795_watchdog.ps1 -CheckOnly
#   powershell -ExecutionPolicy Bypass -File .\scripts\windows\atlas_quant_8795_watchdog.ps1 -MaxLogSizeMb 50 -MaxLogArchives 10
#
# Integracion Task Scheduler (Windows):
#   1) Crear tarea -> Trigger: Repeat task every 1 minute, indefinitely.
#   2) Action:
#      Program/script: powershell.exe
#      Arguments: -ExecutionPolicy Bypass -File "C:\ATLAS_PUSH\scripts\windows\atlas_quant_8795_watchdog.ps1"
#   3) Marcar "Run whether user is logged on or not" y "Run with highest privileges" si aplica.
