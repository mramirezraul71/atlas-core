param(
    [string]$Task = "",
    [ValidateSet("auto", "default", "plan")]
    [string]$Mode = "auto",
    [string]$Profile = "auto",
    [switch]$NoMemory,
    [switch]$DryRun,
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$CodexArgs
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Get-AutoProfile {
    param([string]$Text)
    if ($null -eq $Text) { $Text = "" }
    $t = $Text.Trim().ToLowerInvariant()
    if ([string]::IsNullOrWhiteSpace($t)) { return "atlas_realtime" }

    $isComplexKeyword = $t -match "(arquitect|architecture|diseno|design|refactor|migrat|incident|postmortem|root cause|debug|produccion|security|threat model|performance|optimiz)"
    $isMediumKeyword = $t -match "(feature|endpoint|api|test|fix|bug|review|integrat|document|script)"
    $len = $t.Length

    if ($len -ge 320 -or $isComplexKeyword) { return "atlas_supervisor" }
    if ($len -ge 120 -or $isMediumKeyword) { return "atlas_safe" }
    return "atlas_realtime"
}

function Get-AutoMode {
    param([string]$Text)
    if ($null -eq $Text) { $Text = "" }
    $t = $Text.Trim().ToLowerInvariant()
    if ([string]::IsNullOrWhiteSpace($t)) { return "Default" }

    $planHint = $t -match "(plan|roadmap|estrategia|strategy|arquitect|architecture|diseno|analysis|analisis|evaluar|comparar)"
    if ($planHint) { return "Plan" }
    return "Default"
}

function Ensure-MemoryFiles {
    param([string]$MemoryDir)

    if (-not (Test-Path $MemoryDir)) {
        New-Item -ItemType Directory -Path $MemoryDir -Force | Out-Null
    }

    $handoff = Join-Path $MemoryDir "atlas_handoff.md"
    $log = Join-Path $MemoryDir "atlas_memory_log.md"

    if (-not (Test-Path $handoff)) {
@"
# ATLAS handoff

## Current focus
- (update this before closing session)

## Last decisions
- (key technical decisions)

## Next steps
- (top 3 actions for next session)
"@ | Set-Content -Path $handoff -Encoding UTF8
    }

    if (-not (Test-Path $log)) {
        "# ATLAS memory log`n" | Set-Content -Path $log -Encoding UTF8
    }

    return @{ Handoff = $handoff; Log = $log }
}

$codex = Get-Command codex -ErrorAction SilentlyContinue
if (-not $codex) {
    throw "No se encontro el comando 'codex' en PATH."
}

$selectedProfile = if ($Profile -and $Profile -ne "auto") { $Profile } else { Get-AutoProfile -Text $Task }
$selectedMode = if ($Mode -ne "auto") { (Get-Culture).TextInfo.ToTitleCase($Mode) } else { Get-AutoMode -Text $Task }

$launchArgs = @("--profile", $selectedProfile)
if ($CodexArgs) { $launchArgs += $CodexArgs }

$prompt = $Task
if (-not $NoMemory) {
    $memory = Ensure-MemoryFiles -MemoryDir "C:\ATLAS_PUSH\.codex\memory"
    $handoffText = Get-Content -Path $memory.Handoff -Raw
    if ($handoffText.Length -gt 6000) {
        $handoffText = $handoffText.Substring(0, 6000)
    }

    if ([string]::IsNullOrWhiteSpace($prompt)) {
        $prompt = "Continuamos ATLAS con memoria persistente. Usa el handoff como contexto base y propone siguientes pasos concretos."
    }

    $prompt = @"
Contexto persistente (handoff ATLAS):
$handoffText

Tarea de esta sesion:
$prompt
"@
}

Write-Host "Perfil seleccionado: $selectedProfile"
Write-Host "Modo sugerido: $selectedMode"
Write-Host "Comando: codex $($launchArgs -join ' ')"
Write-Host "Dentro de Codex puedes ajustar con: /mode"
if (-not $NoMemory) {
    Write-Host "Memoria: C:\ATLAS_PUSH\.codex\memory\atlas_handoff.md"
}

if ($DryRun) { exit 0 }

& $codex.Source @launchArgs $prompt
