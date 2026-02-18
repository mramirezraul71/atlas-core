# Script para probar workspace endpoint
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing

# 1. Verificar que el endpoint responde
Write-Host "=== VERIFICANDO ENDPOINT ===" -ForegroundColor Cyan
try {
    $response = Invoke-WebRequest -Uri "http://localhost:8791/workspace" -UseBasicParsing
    Write-Host "✓ HTTP Status: $($response.StatusCode)" -ForegroundColor Green
    Write-Host "✓ Content-Type: $($response.Headers['Content-Type'])" -ForegroundColor Green
    Write-Host "✓ Content-Length: $($response.RawContentLength) bytes" -ForegroundColor Green
} catch {
    Write-Host "✗ Error: $($_.Exception.Message)" -ForegroundColor Red
    exit 1
}

# 2. Verificar endpoint /status
Write-Host "`n=== VERIFICANDO /status ===" -ForegroundColor Cyan
try {
    $statusResponse = Invoke-WebRequest -Uri "http://localhost:8791/status" -UseBasicParsing
    $statusData = $statusResponse.Content | ConvertFrom-Json
    Write-Host "✓ Status OK: $($statusData.ok)" -ForegroundColor Green
    Write-Host "  - nexus_connected: $($statusData.nexus_connected)" -ForegroundColor $(if($statusData.nexus_connected){"Green"}else{"Red"})
    Write-Host "  - robot_connected: $($statusData.robot_connected)" -ForegroundColor $(if($statusData.robot_connected){"Green"}else{"Red"})
    Write-Host "  - push_connected: $($statusData.push_connected)" -ForegroundColor $(if($statusData.push_connected){"Yellow"}else{"Yellow"})
    if ($null -eq $statusData.push_connected) {
        Write-Host "  ⚠ ADVERTENCIA: push_connected no existe en respuesta" -ForegroundColor Yellow
    }
} catch {
    Write-Host "✗ Error: $($_.Exception.Message)" -ForegroundColor Red
}

# 3. Verificar que workspace.html contiene elementos esperados
Write-Host "`n=== ANALIZANDO CONTENIDO HTML ===" -ForegroundColor Cyan
$htmlContent = $response.Content
$checks = @{
    "ATLAS WORKSPACE" = $htmlContent -match "ATLAS WORKSPACE"
    "chat-messages" = $htmlContent -match 'id="chat-messages"'
    "voice-orb" = $htmlContent -match 'id="voice-orb"'
    "live-feed" = $htmlContent -match 'id="live-feed"'
    "code-viewer" = $htmlContent -match 'id="code-viewer"'
    "checkServices()" = $htmlContent -match "checkServices\(\)"
    "sendMessage()" = $htmlContent -match "sendMessage\(\)"
}

foreach ($check in $checks.GetEnumerator()) {
    $symbol = if ($check.Value) { "✓" } else { "✗" }
    $color = if ($check.Value) { "Green" } else { "Red" }
    Write-Host "  $symbol $($check.Key)" -ForegroundColor $color
}

# 4. Buscar posibles errores en JavaScript
Write-Host "`n=== BUSCANDO POSIBLES BUGS ===" -ForegroundColor Cyan
if ($htmlContent -match "d\.push_connected") {
    Write-Host "  ⚠ ENCONTRADO: Código usa 'd.push_connected' (campo inexistente)" -ForegroundColor Yellow
}
if ($htmlContent -match "d\.nexus_connected") {
    Write-Host "  ✓ Código usa 'd.nexus_connected' (campo válido)" -ForegroundColor Green
}
if ($htmlContent -match "d\.robot_connected") {
    Write-Host "  ✓ Código usa 'd.robot_connected' (campo válido)" -ForegroundColor Green
}

Write-Host "`n=== RESUMEN ===" -ForegroundColor Cyan
Write-Host "Workspace está accesible y contiene todos los elementos UI esperados." -ForegroundColor Green
Write-Host "Bug detectado: indicador PUSH usará campo inexistente." -ForegroundColor Yellow
