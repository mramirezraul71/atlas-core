# Fase 14 - Tests con Python 3.11 explicito
# Ejecutar desde: C:\ATLAS_PUSH\workspace_prime
# Uso: .\run_phase14_py311.ps1

$py = "C:\Users\r6957\AppData\Local\Programs\Python\Python311\python.exe"
if (-not (Test-Path $py)) {
    Write-Host "No encontrado: $py - Ajusta la variable py en este script."
    exit 1
}

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $root

$scripts = @(
    "memory_manager.py",
    "web_tools.py",
    "vision_eyes.py",
    "desktop_hands.py",
    "smart_browser.py",
    "visual_agent.py",
    "nl_parser.py",
    "plan_executor.py",
    "browser_hands.py"
)

$failed = @()
foreach ($s in $scripts) {
    Write-Host "--- $s ---"
    & $py $s 2>&1
    if ($LASTEXITCODE -ne 0) { $failed += $s }
}

if ($failed.Count -eq 0) {
    Write-Host "`n[OK] Fase 14: todos los tests pasaron."
} else {
    Write-Host "`n[FALLO] Scripts con error: $($failed -join ', ')"
    exit 1
}
