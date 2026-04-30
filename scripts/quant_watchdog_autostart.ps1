<#
.SYNOPSIS
    Gestiona autostart del watchdog Quant en HKCU\Run.

.DESCRIPTION
    Permite habilitar, deshabilitar o consultar el estado del arranque automático
    del script:
    C:\ATLAS_PUSH\scripts\quant_watchdog_simple.ps1

.EXAMPLE
    .\scripts\quant_watchdog_autostart.ps1 -Action status
    .\scripts\quant_watchdog_autostart.ps1 -Action enable
    .\scripts\quant_watchdog_autostart.ps1 -Action disable
#>

param(
    [ValidateSet("enable", "disable", "status")]
    [string]$Action = "status"
)

$ErrorActionPreference = "Stop"

$RunKeyPath = "HKCU:\Software\Microsoft\Windows\CurrentVersion\Run"
$ValueName = "ATLASQuantWatchdog"
$ScriptPath = "C:\ATLAS_PUSH\scripts\quant_watchdog_simple.ps1"
$CommandValue = "powershell.exe -NoProfile -ExecutionPolicy Bypass -File `"$ScriptPath`""

function Write-Info([string]$msg) {
    Write-Host "[ATLAS] $msg" -ForegroundColor Cyan
}

function Write-Ok([string]$msg) {
    Write-Host "[ATLAS] $msg" -ForegroundColor Green
}

function Write-WarnMsg([string]$msg) {
    Write-Host "[ATLAS] $msg" -ForegroundColor Yellow
}

function Get-AutostartValue {
    try {
        $item = Get-ItemProperty -Path $RunKeyPath -Name $ValueName -ErrorAction Stop
        return [string]$item.$ValueName
    } catch {
        return $null
    }
}

switch ($Action) {
    "enable" {
        if (-not (Test-Path $ScriptPath)) {
            throw "No existe el script watchdog: $ScriptPath"
        }
        New-ItemProperty -Path $RunKeyPath -Name $ValueName -PropertyType String -Value $CommandValue -Force | Out-Null
        Write-Ok "Autostart habilitado: $ValueName"
        Write-Info "Comando: $CommandValue"
    }

    "disable" {
        $current = Get-AutostartValue
        if ($null -eq $current) {
            Write-WarnMsg "Autostart ya estaba deshabilitado."
        } else {
            Remove-ItemProperty -Path $RunKeyPath -Name $ValueName -ErrorAction Stop
            Write-Ok "Autostart deshabilitado: $ValueName"
        }
    }

    "status" {
        $current = Get-AutostartValue
        if ($null -eq $current) {
            Write-WarnMsg "Estado: DESHABILITADO (sin entrada en HKCU\\Run)"
        } else {
            Write-Ok "Estado: HABILITADO"
            Write-Info "Valor actual: $current"
        }
    }
}

