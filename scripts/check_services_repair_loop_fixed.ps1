# Check Services Repair Loop
# Analiza bitácora para detectar ciclos repetitivos de services_repair

$bitacoraPath = "c:\ATLAS_PUSH\logs\ans_evolution_bitacora.json"

if (-not (Test-Path $bitacoraPath)) {
    Write-Host "❌ Bitácora no encontrada: $bitacoraPath"
    exit 1
}

try {
    $data = Get-Content $bitacoraPath | ConvertFrom-Json
    $entries = $data | Where-Object { $_.message -like "*services_repair*" } | Select-Object -Last 20

    Write-Host "🔍 ANÁLISIS DE CICLO SERVICES_REPAIR"
    Write-Host "=" * 50
    Write-Host "Últimas 20 entradas de services_repair:"
    Write-Host ""

    foreach ($entry in $entries) {
        $timestamp = $entry.timestamp
        $message = $entry.message

        # Formatear timestamp
        $dt = [DateTime]::Parse($timestamp)
        $timeStr = $dt.ToString("HH:mm:ss")

        Write-Host "[$timeStr] $message"
    }

    Write-Host ""
    Write-Host "📊 ANÁLISIS DE PATRONES:"

    # Contar frecuencia
    $lastHour = $data | Where-Object {
        $dt = [DateTime]::Parse($_.timestamp)
        $dt -gt (Get-Date).AddHours(-1) -and $_.message -like "*services_repair*"
    }

    $last30min = $data | Where-Object {
        $dt = [DateTime]::Parse($_.timestamp)
        $dt -gt (Get-Date).AddMinutes(-30) -and $_.message -like "*services_repair*"
    }

    $last10min = $data | Where-Object {
        $dt = [DateTime]::Parse($_.timestamp)
        $dt -gt (Get-Date).AddMinutes(-10) -and $_.message -like "*services_repair*"
    }

    Write-Host "Ejecuciones en última hora: $($lastHour.Count)"
    Write-Host "Ejecuciones en últimos 30 min: $($last30min.Count)"
    Write-Host "Ejecuciones en últimos 10 min: $($last10min.Count)"

    # Detectar patrón repetitivo
    if ($last10min.Count -ge 3) {
        Write-Host ""
        Write-Host "🚨 ALERTA: CICLO REPETITIVO DETECTADO"
        Write-Host "El POT services_repair se está ejecutando demasiado frecuentemente"
        Write-Host ""

        # Analizar intervalos
        $times = $last10min | ForEach-Object { [DateTime]::Parse($_.timestamp) } | Sort-Object
        $intervals = @()

        for ($i = 1; $i -lt $times.Count; $i++) {
            $interval = ($times[$i] - $times[$i-1]).TotalSeconds
            $intervals += $interval
        }

        if ($intervals.Count -gt 0) {
            $avgInterval = $intervals | Measure-Object -Average
            Write-Host "Intervalo promedio entre ejecuciones: $($avgInterval.Average.ToString('F1')) segundos"

            if ($avgInterval.Average -lt 60) {
                Write-Host "🔥 CRÍTICO: Intervalo menor a 1 minuto - POSIBLE LOOP INFINITO"
            } elseif ($avgInterval.Average -lt 180) {
                Write-Host "⚠️ ADVERTENCIA: Intervalo menor a 3 minutos - posible problema"
            }
        }

        Write-Host ""
        Write-Host "🔧 ACCIONES RECOMENDADAS:"
        Write-Host "1. Revisar logs del POT services_repair"
        Write-Host "2. Verificar si los servicios realmente se están reparando"
        Write-Host "3. Identificar la causa raíz del fallo continuo"
        Write-Host "4. Considerar deshabilitar el trigger temporalmente"

    } else {
        Write-Host ""
        Write-Host "✅ No se detecta ciclo repetitivo crítico"
    }

    # Mostrar estado actual de servicios
    Write-Host ""
    Write-Host "🏥 ESTADO ACTUAL DE SERVICIOS:"

    $healthChecks = @(
        @{Name="Robot Backend"; Port=8002; URL="http://127.0.0.1:8002/api/health"},
        @{Name="NEXUS"; Port=8000; URL="http://127.0.0.1:8000/health"},
        @{Name="Push Dashboard"; Port=8791; URL="http://127.0.0.1:8791/health"}
    )

    foreach ($service in $healthChecks) {
        try {
            $response = Invoke-WebRequest -Uri $service.URL -TimeoutSec 3 -UseBasicParsing -ErrorAction Stop
            $status = "✅ UP"
            $code = $response.StatusCode
        } catch {
            $status = "❌ DOWN"
            $code = "ERROR"
        }

        Write-Host "  $($service.Name) (puerto $($service.Port)): $status ($code)"
    }

} catch {
    Write-Host "❌ Error analizando bitácora: $($_.Exception.Message)"
}
