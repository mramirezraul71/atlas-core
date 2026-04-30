# Libera un puerto: encuentra el PID que lo usa y opcionalmente termina
# el árbol de procesos asociado para evitar que un supervisor huérfano
# relance de inmediato el listener antiguo.
# Uso: .\free_port.ps1 -Port 8000 [-Kill]
param(
    [Parameter(Mandatory = $true)]
    [int]$Port,
    [switch]$Kill
)

function Get-PidOnPort {
    param([int]$P)

    $conn = Get-NetTCPConnection -LocalPort $P -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($conn -and $conn.OwningProcess -gt 0) {
        return $conn.OwningProcess
    }

    # Fallback: netstat (menos fiable; parseo robusto)
    $raw = netstat -ano
    $lines = $raw | Select-String "LISTENING" | Where-Object { $_.Line -match ":$P\s" }
    foreach ($line in $lines) {
        $parts = ($line.Line.Trim() -split '\s+') | Where-Object { $_ -ne "" }
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

function Get-AtlasAncestorPid {
    param([int]$ProcessId)

    $current = $ProcessId
    $selected = $ProcessId
    $eligible = @("python.exe", "nohup.exe", "uvicorn.exe")

    while ($current -gt 0) {
        $proc = Get-CimInstance Win32_Process -Filter "ProcessId = $current" -ErrorAction SilentlyContinue
        if (-not $proc) {
            break
        }

        $name = if ($proc.Name) { $proc.Name.ToLower() } else { "" }
        if ($eligible -contains $name) {
            $selected = $proc.ProcessId
            $current = [int]$proc.ParentProcessId
            continue
        }

        break
    }

    return $selected
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
    $killId = Get-AtlasAncestorPid -ProcessId $procId
    $killTarget = if ($killId -ne $procId) {
        "$name (PID $procId) via ancestro $killId"
    } else {
        "$name (PID $procId)"
    }

    try {
        cmd /c "taskkill /PID $killId /T /F >nul 2>nul"
    } catch {}

    Start-Sleep -Milliseconds 800

    if (Get-PidOnPort -P $Port) {
        Stop-Process -Id $procId -Force -ErrorAction SilentlyContinue
        Start-Sleep -Milliseconds 400
    }

    if (Get-PidOnPort -P $Port) {
        Write-Host "No se pudo liberar completamente el puerto $Port." -ForegroundColor Red
        exit 1
    }

    Write-Host "Proceso/arbol $killTarget terminado. Puerto $Port libre." -ForegroundColor Green
} else {
    Write-Host "Para liberar: .\free_port.ps1 -Port $Port -Kill" -ForegroundColor Cyan
}
