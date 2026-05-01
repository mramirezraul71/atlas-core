# Requisitos: Lean CLI (https://github.com/QuantConnect/lean-cli) y Docker si aplica.
# No instala paquetes pesados ni datos de mercado en el repo.

$ErrorActionPreference = "Stop"
$ProjectRoot = if ($env:ATLAS_LEAN_PROJECT_ROOT) { $env:ATLAS_LEAN_PROJECT_ROOT } else { Join-Path $PSScriptRoot ".." "lean_user_project" }
$Algo = if ($env:ATLAS_LEAN_ALGORITHM) { $env:ATLAS_LEAN_ALGORITHM } else { "AtlasMinimal" }

Write-Host "ATLAS_LEAN_PROJECT_ROOT = $ProjectRoot"
Write-Host "ATLAS_LEAN_ALGORITHM     = $Algo"
Write-Host ""
Write-Host "Pasos manuales (QuantConnect):"
Write-Host "  1. lean create-project `"$Algo`" --language csharp"
Write-Host "  2. Copiar logica minima al Main.cs / algoritmo generado"
Write-Host "  3. lean backtest `"$Algo`""
Write-Host "  4. Comprobar statistics.json bajo $ProjectRoot\backtests\"
Write-Host ""
Write-Host "Variables ATLAS_*: ver atlas_code_quant/lean/README.md y config/atlas.env.example"
