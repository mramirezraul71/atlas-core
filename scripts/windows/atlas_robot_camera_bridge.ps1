[CmdletBinding()]
param(
    [string]$BackendBaseUrl = "http://localhost:8002",
    [int]$MaxCameraIndex = 3,
    [int]$BackendRetries = 10,
    [int]$RetryDelaySeconds = 3,
    [int]$ForceDeviceIndex = -1
)

$ErrorActionPreference = "Stop"

function Write-Info([string]$Message) {
    Write-Host "[atlas-camera-bridge] $Message"
}

function Invoke-OpenCvProbe {
    param(
        [int]$MaxIndex,
        [int]$ForcedIndex
    )

    $pythonCode = @"
import json
import sys
import cv2

max_index = int(sys.argv[1])
forced_index = int(sys.argv[2])
indices = [forced_index] if forced_index >= 0 else list(range(max_index + 1))
found_index = None
error_messages = []

for idx in indices:
    cap = None
    try:
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        if not cap.isOpened():
            error_messages.append(f"{idx}:not_opened")
            continue
        ok, frame = cap.read()
        if ok and frame is not None:
            found_index = idx
            break
        error_messages.append(f"{idx}:frame_failed")
    except Exception as exc:
        error_messages.append(f"{idx}:{exc}")
    finally:
        if cap is not None:
            cap.release()

payload = {
    "camera_available": found_index is not None,
    "device_index": found_index,
    "errors": error_messages,
}
print(json.dumps(payload))
"@

    $tmpPy = Join-Path $env:TEMP "atlas_camera_probe.py"
    Set-Content -Path $tmpPy -Value $pythonCode -Encoding UTF8

    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.FileName = "python"
    $psi.Arguments = "`"$tmpPy`" $MaxIndex $ForcedIndex"
    $psi.RedirectStandardOutput = $true
    $psi.RedirectStandardError = $true
    $psi.UseShellExecute = $false
    $psi.CreateNoWindow = $true
    $proc = [System.Diagnostics.Process]::Start($psi)

    $waitMs = 15000
    if (-not $proc.WaitForExit($waitMs)) {
        try { $proc.Kill() } catch {}
        Remove-Item $tmpPy -ErrorAction SilentlyContinue
        throw "OpenCV probe timeout after $waitMs ms"
    }

    $stdoutText = $proc.StandardOutput.ReadToEnd()
    $stderrText = $proc.StandardError.ReadToEnd()
    Remove-Item $tmpPy -ErrorAction SilentlyContinue

    if ($stderrText) {
        Write-Info ("OpenCV stderr: " + $stderrText.Trim())
    }

    if (-not $stdoutText) {
        throw "OpenCV probe returned empty output"
    }

    $rawOutput = $stdoutText -split "`r?`n"
    $jsonLine = $rawOutput | Where-Object { $_ -match "^\s*\{.+\}\s*$" } | Select-Object -Last 1
    if (-not $jsonLine) {
        throw ("OpenCV probe did not return JSON payload. Raw: " + ($rawOutput -join " | "))
    }
    return ($jsonLine | ConvertFrom-Json)
}

function Wait-Backend {
    param(
        [string]$HealthUrl,
        [int]$Retries,
        [int]$DelaySeconds
    )
    for ($attempt = 1; $attempt -le $Retries; $attempt++) {
        try {
            $resp = Invoke-RestMethod -Method Get -Uri $HealthUrl -TimeoutSec 5
            if ($resp.status) {
                Write-Info "Backend activo en intento $attempt."
                return $true
            }
        } catch {
            Write-Info "Backend no disponible (intento $attempt/$Retries)."
        }
        Start-Sleep -Seconds $DelaySeconds
    }
    return $false
}

$healthUrl = "$BackendBaseUrl/api/health"
$retestUrl = "$BackendBaseUrl/modules/camera/retest"
$retestUrlApi = "$BackendBaseUrl/api/modules/camera/retest"

if (-not (Wait-Backend -HealthUrl $healthUrl -Retries $BackendRetries -DelaySeconds $RetryDelaySeconds)) {
    throw "Backend 8002 no disponible en $healthUrl"
}

$probe = Invoke-OpenCvProbe -MaxIndex $MaxCameraIndex -ForcedIndex $ForceDeviceIndex
Write-Info ("Probe OpenCV -> available={0}, device_index={1}" -f $probe.camera_available, $probe.device_index)
if ($probe.errors -and $probe.errors.Count -gt 0) {
    Write-Info ("Detalles probe: " + ($probe.errors -join ", "))
}

if (-not $probe.camera_available) {
    Write-Info "No hay camara disponible por OpenCV. No se llama retest endpoint."
    exit 2
}

$body = @{ device_index = [int]$probe.device_index } | ConvertTo-Json
$response = $null
try {
    $response = Invoke-RestMethod -Method Post -Uri $retestUrl -ContentType "application/json" -Body $body -TimeoutSec 15
} catch {
    Write-Info "Ruta /modules/camera/retest no disponible, intentando alias /api/modules/camera/retest."
    $response = Invoke-RestMethod -Method Post -Uri $retestUrlApi -ContentType "application/json" -Body $body -TimeoutSec 15
}

Write-Info ("Respuesta robot: " + ($response | ConvertTo-Json -Depth 5 -Compress))
if ($response.camera_available -eq $true) {
    Write-Info "Resultado final: camera ON"
    exit 0
}

Write-Info "Resultado final: camera OFF"
exit 1

# Uso manual:
#   powershell -ExecutionPolicy Bypass -File .\scripts\windows\atlas_robot_camera_bridge.ps1
#   powershell -ExecutionPolicy Bypass -File .\scripts\windows\atlas_robot_camera_bridge.ps1 -MaxCameraIndex 5 -BackendRetries 20
#
# Integracion Task Scheduler (Windows):
#   1) Crear tarea -> Trigger: "At log on" o "At startup".
#   2) Action:
#      Program/script: powershell.exe
#      Arguments: -ExecutionPolicy Bypass -File "C:\ATLAS_PUSH\scripts\windows\atlas_robot_camera_bridge.ps1"
#   3) Si la camara demora en estar READY, aumenta -BackendRetries y -RetryDelaySeconds.
#   4) Para fijar un indice concreto, usa -ForceDeviceIndex 1 (por ejemplo).
