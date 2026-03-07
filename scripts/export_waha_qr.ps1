param(
    [string]$ApiBase = "http://127.0.0.1:8791",
    [string]$OutFile = "C:\ATLAS_PUSH\logs\waha_qr.png"
)

$ErrorActionPreference = "Stop"

$url = "$ApiBase/api/comms/whatsapp/qr"
$resp = Invoke-RestMethod -Method GET -Uri $url -TimeoutSec 20

if (-not $resp.ok) {
    throw "No se pudo obtener QR: $($resp.error)"
}

if ($resp.png_base64) {
    $bytes = [Convert]::FromBase64String([string]$resp.png_base64)
    [System.IO.File]::WriteAllBytes($OutFile, $bytes)
    Write-Host "[OK] QR guardado en $OutFile"
    exit 0
}

if ($resp.data_url -and $resp.data_url -match '^data:image\/png;base64,(.+)$') {
    $bytes = [Convert]::FromBase64String($Matches[1])
    [System.IO.File]::WriteAllBytes($OutFile, $bytes)
    Write-Host "[OK] QR guardado en $OutFile"
    exit 0
}

throw "Respuesta QR no contiene png_base64/data_url."
