param(
    [string]$TaskName = "ATLAS_Panaderia_Tunnel_AutoHeal",
    [int]$IntervalMinutes = 1,
    [switch]$RunNow
)

$ErrorActionPreference = "Stop"
$PSNativeCommandUseErrorActionPreference = $false

if ($IntervalMinutes -lt 1) {
    throw "IntervalMinutes debe ser >= 1."
}

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$runnerScript = (Resolve-Path (Join-Path $PSScriptRoot "panaderia_tunnel_autoheal.ps1")).Path
$loopScript = (Resolve-Path (Join-Path $PSScriptRoot "panaderia_tunnel_autoheal_loop.ps1")).Path
$intervalSeconds = [Math]::Max(30, $IntervalMinutes * 60)
$taskCmd = "powershell -NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File `"$runnerScript`""
$runKeyName = "ATLAS_Panaderia_AutoHeal_Loop"
$runKeyPath = "HKCU:\Software\Microsoft\Windows\CurrentVersion\Run"
$runValue = "powershell -NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File `"$loopScript`" -IntervalSeconds $intervalSeconds"

function Upsert-RunKey {
    param(
        [string]$Name,
        [string]$Value
    )
    if (-not (Test-Path $runKeyPath)) {
        New-Item -Path $runKeyPath | Out-Null
    }
    New-ItemProperty -Path $runKeyPath -Name $Name -PropertyType String -Value $Value -Force | Out-Null
}

function Test-AutohealLoopRunning {
    $procs = Get-CimInstance Win32_Process | Where-Object {
        $_.Name -match "powershell" -and $_.CommandLine -match "panaderia_tunnel_autoheal_loop\.ps1"
    }
    return [bool]($procs -and $procs.Count -gt 0)
}

function Ensure-LoopRunning {
    if (Test-AutohealLoopRunning) {
        Write-Host "[OK] Loop autoheal ya está en ejecución."
        return
    }
    Start-Process -FilePath "powershell.exe" -ArgumentList @(
        "-NoProfile",
        "-ExecutionPolicy", "Bypass",
        "-WindowStyle", "Hidden",
        "-File", $loopScript,
        "-IntervalSeconds", "$intervalSeconds"
    ) -WindowStyle Hidden | Out-Null
    Start-Sleep -Seconds 1
    if (Test-AutohealLoopRunning) {
        Write-Host "[OK] Loop autoheal iniciado en background."
    } else {
        Write-Warning "No se pudo confirmar loop autoheal en ejecución."
    }
}

$baseArgs = @(
    "/Create",
    "/TN", $TaskName,
    "/SC", "MINUTE",
    "/MO", "$IntervalMinutes",
    "/TR", $taskCmd,
    "/F"
)

Write-Host ("Configurando tarea programada: {0} cada {1} minuto(s)" -f $TaskName, $IntervalMinutes)
$created = $false
foreach ($runLevel in @("HIGHEST", "LIMITED")) {
    try {
        $args = @($baseArgs + @("/RL", $runLevel))
        $output = & schtasks $args 2>&1
        $output | Out-Host
        if ($LASTEXITCODE -eq 0) {
            Write-Host ("Tarea creada con RL={0}" -f $runLevel)
            $created = $true
            break
        }
        Write-Warning ("No se pudo crear con RL={0}. Intentando siguiente nivel..." -f $runLevel)
    } catch {
        Write-Warning ("No se pudo crear con RL={0}: {1}" -f $runLevel, $_.Exception.Message)
    }
}

if (-not $created) {
    Write-Warning "No fue posible crear tarea programada (sin permisos). Activando fallback por HKCU Run."
    Upsert-RunKey -Name $runKeyName -Value $runValue
    Write-Host ("[OK] Autoarranque configurado en HKCU Run: {0}" -f $runKeyName)
    if ($RunNow.IsPresent) {
        Ensure-LoopRunning
    }
    Write-Host "Fallback aplicado. Resumen RunKey:"
    Get-ItemProperty -Path $runKeyPath -Name $runKeyName | Format-List | Out-Host
    exit 0
}

if ($RunNow.IsPresent) {
    Write-Host ("Ejecutando tarea ahora: {0}" -f $TaskName)
    schtasks /Run /TN $TaskName | Out-Host
    Ensure-LoopRunning
}

Write-Host "Tarea configurada. Resumen:"
schtasks /Query /TN $TaskName /V /FO LIST | Out-Host
