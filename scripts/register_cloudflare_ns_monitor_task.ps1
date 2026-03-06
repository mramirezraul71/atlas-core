param(
    [string]$TaskName = "ATLAS_Cloudflare_NS_Monitor",
    [int]$IntervalMinutes = 15,
    [switch]$RunNow
)

$ErrorActionPreference = "Stop"

if ($IntervalMinutes -lt 5) {
    throw "IntervalMinutes debe ser >= 5."
}

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$runnerScript = (Resolve-Path (Join-Path $PSScriptRoot "run_cloudflare_ns_monitor.ps1")).Path

$taskCmd = "powershell -NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File `"$runnerScript`""

$baseArgs = @(
    "/Create",
    "/TN", $TaskName,
    "/SC", "MINUTE",
    "/MO", "$IntervalMinutes",
    "/TR", $taskCmd,
    "/F"
)

Write-Host ("Configurando tarea programada: {0} cada {1} minutos" -f $TaskName, $IntervalMinutes)
$created = $false
foreach ($runLevel in @("HIGHEST", "LIMITED")) {
    $args = @($baseArgs + @("/RL", $runLevel))
    $output = & schtasks $args 2>&1
    $output | Out-Host
    if ($LASTEXITCODE -eq 0) {
        Write-Host ("Tarea creada con RL={0}" -f $runLevel)
        $created = $true
        break
    }
    Write-Warning ("No se pudo crear con RL={0}. Intentando siguiente nivel..." -f $runLevel)
}

if (-not $created) {
    throw "No fue posible crear la tarea programada con los niveles de ejecución disponibles."
}

if ($RunNow.IsPresent) {
    Write-Host ("Ejecutando tarea ahora: {0}" -f $TaskName)
    schtasks /Run /TN $TaskName | Out-Host
}

Write-Host "Tarea configurada. Resumen:"
schtasks /Query /TN $TaskName /V /FO LIST | Out-Host
