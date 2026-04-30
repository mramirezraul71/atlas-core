# Atlas Product Pack - Export portable ZIP (code + requirements lock, exclude heavy logs)
param([string]$SourceDir = "", [string]$OutZip = "")
$ErrorActionPreference = "Stop"
$repo = if ($SourceDir) { (Resolve-Path $SourceDir).Path } else { (Join-Path $PSScriptRoot "..") }
$repo = $repo.TrimEnd("\")
$ts = Get-Date -Format "yyyyMMdd_HHmmss"
$OutZip = if ($OutZip) { $OutZip } else { Join-Path $repo "atlas_portable_$ts.zip" }
$exclude = @("*.pyc", "__pycache__", ".git", "node_modules", ".venv", "venv", "logs\*", "*.sqlite-shm", "*.sqlite-wal")
Write-Host "Exporting: $repo -> $OutZip" -ForegroundColor Cyan
$reqLock = Join-Path $repo "requirements.lock"
if (-not (Test-Path $reqLock)) {
    $req = Join-Path $repo "requirements.txt"
    if (Test-Path $req) {
        & pip freeze 2>$null | Where-Object { $_ -match "fastapi|uvicorn|httpx|pydantic|dotenv" } | Set-Content $reqLock -ErrorAction SilentlyContinue
    }
}
$tempDir = Join-Path $env:TEMP "atlas_export_$ts"
New-Item -ItemType Directory -Force -Path $tempDir | Out-Null
robocopy $repo $tempDir /E /XD .git __pycache__ node_modules .venv venv logs /XF *.pyc *.sqlite-shm *.sqlite-wal /NFL /NDL /NJH /NJS /nc /ns /np 2>$null
if (Test-Path $reqLock) { Copy-Item $reqLock $tempDir -Force }
Compress-Archive -Path "$tempDir\*" -DestinationPath $OutZip -Force
Remove-Item $tempDir -Recurse -Force -ErrorAction SilentlyContinue
Write-Host "Portable package: $OutZip" -ForegroundColor Green
