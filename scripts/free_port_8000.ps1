# Libera el puerto 8000 (NEXUS). Encuentra el PID que lo usa y lo termina.
# Uso: .\free_port_8000.ps1 [-Kill]   (sin -Kill solo muestra el proceso)
param([switch]$Kill)

$port = 8000

function Get-PidOnPort {
    # Primero intentar netstat (mas fiable en Windows cuando OwningProcess=0)
    $raw = netstat -ano
    $lines = $raw | Select-String "LISTENING" | Where-Object { $_.Line -match ":$port\s" }
    foreach ($line in $lines) {
        $parts = ($line.Line.Trim() -split '\s+') | Where-Object { $_ -ne '' }
        if ($parts.Count -ge 5) {
            $pidStr = $parts[-1]
            if ([int]::TryParse($pidStr, [ref]$null) -and [int]$pidStr -gt 0) {
                return [int]$pidStr
            }
        }
    }
    # Get-NetTCPConnection como respaldo
    $conn = Get-NetTCPConnection -LocalPort $port -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($conn -and $conn.OwningProcess -gt 0) { return $conn.OwningProcess }
    return $null
}

$procId = Get-PidOnPort
if (-not $procId) {
    Write-Host "Puerto $port libre." -ForegroundColor Green
    exit 0
}
$proc = Get-Process -Id $procId -ErrorAction SilentlyContinue
$name = if ($proc) { $proc.ProcessName } else { "PID $procId" }
Write-Host "Puerto $port en uso por: $name (PID $procId)" -ForegroundColor Yellow
if ($Kill) {
    Stop-Process -Id $procId -Force -ErrorAction SilentlyContinue
    Start-Sleep -Milliseconds 500
    $still = Get-PidOnPort
    if (-not $still) {
        Write-Host "Proceso $procId terminado. Puerto $port libre." -ForegroundColor Green
    } else {
        taskkill /PID $procId /F 2>$null
        Write-Host "Proceso $procId terminado (taskkill)." -ForegroundColor Green
    }
} else {
    Write-Host "Para liberar ejecuta: .\free_port_8000.ps1 -Kill" -ForegroundColor Cyan
}
