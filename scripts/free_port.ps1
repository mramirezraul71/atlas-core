# Libera un puerto: encuentra el PID que lo usa y opcionalmente lo termina.
# Uso: .\free_port.ps1 -Port 8000 [-Kill]
param(
    [Parameter(Mandatory=$true)]
    [int]$Port,
    [switch]$Kill
)

function Get-PidOnPort {
    param([int]$P)
    $conn = Get-NetTCPConnection -LocalPort $P -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($conn -and $conn.OwningProcess -gt 0) { return $conn.OwningProcess }
    # Fallback: netstat (menos fiable; parseo robusto)
    $raw = netstat -ano
    $lines = $raw | Select-String "LISTENING" | Where-Object { $_.Line -match ":$P\s" }
    foreach ($line in $lines) {
        $parts = ($line.Line.Trim() -split '\s+') | Where-Object { $_ -ne '' }
        if ($parts.Count -ge 5) {
            $pidStr = $parts[-1]
            $tmp = 0
            if ([int]::TryParse($pidStr, [ref]$tmp) -and $tmp -gt 0) {
                return $tmp
            }
        }
    }
    return $null
}

$procId = Get-PidOnPort -P $Port
if (-not $procId) {
    Write-Host "Puerto $Port libre." -ForegroundColor Green
    exit 0
}
$proc = Get-Process -Id $procId -ErrorAction SilentlyContinue
$name = if ($proc) { $proc.ProcessName } else { "PID $procId" }
Write-Host "Puerto $Port en uso por: $name (PID $procId)" -ForegroundColor Yellow
if ($Kill) {
    Stop-Process -Id $procId -Force -ErrorAction SilentlyContinue
    Start-Sleep -Milliseconds 500
    if (Get-PidOnPort -P $Port) {
        # taskkill puede fallar si el PID ya murió; no abortar el script por eso.
        try {
            cmd /c "taskkill /PID $procId /F >nul 2>nul"
        } catch {}
    }
    Write-Host "Proceso $procId terminado. Puerto $Port libre." -ForegroundColor Green
} else {
    Write-Host "Para liberar: .\free_port.ps1 -Port $Port -Kill" -ForegroundColor Cyan
}
