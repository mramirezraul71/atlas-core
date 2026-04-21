# tests/smoke/ps1_parse_check.ps1
#
# Smoke de sintaxis para el script de arranque simplificado en A2.
# No ejecuta nada: solo verifica que PowerShell puede PARSEAR el
# fichero sin errores. Pensado para correr en Windows (PowerShell 5+)
# o en PowerShell 7 multi-plataforma.
#
# Uso:
#   pwsh -NoProfile -File tests/smoke/ps1_parse_check.ps1
#
# Salida:
#   exit 0  -> parseo OK
#   exit 1  -> errores de sintaxis (se imprimen)

param(
  [string]$Target = "03_run_atlas_api.ps1"
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$path = Join-Path $repoRoot $Target

if (!(Test-Path $path)) {
  Write-Error "No encontrado: $path"
  exit 1
}

Write-Host "[ps1_parse_check] parseando $path"

# Parser estático: no ejecuta el script.
$tokens = $null
$errors = $null
[System.Management.Automation.Language.Parser]::ParseFile($path, [ref]$tokens, [ref]$errors) | Out-Null

if ($errors -and $errors.Count -gt 0) {
  Write-Host "[ps1_parse_check] ERRORES DE PARSEO:" -ForegroundColor Red
  foreach ($e in $errors) {
    Write-Host ("  L{0}:C{1}  {2}" -f `
      $e.Extent.StartLineNumber, `
      $e.Extent.StartColumnNumber, `
      $e.Message)
  }
  exit 1
}

# Comprobaciones adicionales mínimas (no son un test funcional):
#   - Existe la asignación $LiveApp exactamente una vez.
#   - Sigue existiendo la rama -AppImport.
$content = Get-Content -Raw -Path $path

$liveAppMatches = [regex]::Matches($content, '(?m)^\s*\$LiveApp\s*=\s*"atlas_adapter\.atlas_http_api:app"')
if ($liveAppMatches.Count -ne 1) {
  Write-Host "[ps1_parse_check] FALLO: se esperaba 1 asignación de \$LiveApp, encontradas $($liveAppMatches.Count)" -ForegroundColor Red
  exit 1
}

if ($content -notmatch '\$AppImport') {
  Write-Host "[ps1_parse_check] FALLO: la rama de override manual -AppImport no está presente" -ForegroundColor Red
  exit 1
}

Write-Host "[ps1_parse_check] OK: parseo limpio, \$LiveApp único, rama -AppImport preservada." -ForegroundColor Green
exit 0
