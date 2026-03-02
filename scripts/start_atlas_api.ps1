# Arrancar API Atlas (dashboard en http://127.0.0.1:8791/ui)
$ErrorActionPreference = "Stop"
$Root = $PSScriptRoot + "\.."
Set-Location $Root
$env:PYTHONPATH = $Root

function Resolve-AtlasPython {
    param([string]$RepoRoot)
    $candidates = @(
        (Join-Path $RepoRoot ".venv\Scripts\python.exe"),
        (Join-Path $RepoRoot "venv\Scripts\python.exe")
    )
    foreach ($p in $candidates) {
        if (Test-Path $p) { return $p }
    }
    $fallback = Get-Command python -ErrorAction SilentlyContinue
    if ($fallback) { return $fallback.Source }
    throw "Python no encontrado (ni en .venv ni en PATH)."
}

$Python = Resolve-AtlasPython -RepoRoot $Root
& $Python -m uvicorn atlas_adapter.atlas_http_api:app --host 127.0.0.1 --port 8791
