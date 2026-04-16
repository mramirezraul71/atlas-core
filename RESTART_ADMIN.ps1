# ATLAS Restart - Run as Administrator
Write-Host "=== ATLAS RESTART ===" -ForegroundColor Cyan

# Kill port 8795
$port = 8795
$connections = netstat -ano | Select-String ":$port" | Select-String "LISTENING"
foreach ($conn in $connections) {
    $pid_str = ($conn -split '\s+')[-1]
    Write-Host "Killing PID $pid_str"
    Stop-Process -Id $pid_str -Force -ErrorAction SilentlyContinue
}
Start-Sleep 4

# Set env vars
$env:ATLAS_SANDBOX_RESTRICTED_SYMBOLS = "OS"
$env:QUANT_EXIT_GOVERNANCE_ENABLED = "false"
$env:TRADIER_PAPER_TOKEN = "UqYAFhBY0sPWSP4OmuAHChHB0lAN"

# Launch
Set-Location C:\ATLAS_PUSH
$proc = Start-Process -FilePath "C:\ATLAS_PUSH\venv\Scripts\python.exe" `
    -ArgumentList "C:\ATLAS_PUSH\immediate_start.py" `
    -WorkingDirectory "C:\ATLAS_PUSH" `
    -WindowStyle Normal -PassThru
Write-Host "ATLAS launched PID=$($proc.Id)" -ForegroundColor Green
Write-Host "Check: http://localhost:8795/health" -ForegroundColor Yellow
