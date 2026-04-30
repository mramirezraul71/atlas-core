param(
  [string]$Workspace = "C:\ATLAS_PUSH",
  [string]$PythonExe = "python",
  [string]$VenvPath = ".venv_openclaw_atlas",
  [switch]$InstallOptionalOllamaClient
)

$ErrorActionPreference = "Stop"
Set-Location $Workspace

function Invoke-Checked {
  param(
    [string]$Exe,
    [string[]]$CmdArgs
  )
  & $Exe @CmdArgs
  if ($LASTEXITCODE -ne 0) {
    throw "Command failed ($LASTEXITCODE): $Exe $($CmdArgs -join ' ')"
  }
}

function Resolve-VenvPython {
  param([string]$Path)
  return (Join-Path $Path "Scripts\python.exe")
}

if (!(Test-Path $VenvPath)) {
  Write-Host "Creando entorno aislado: $VenvPath" -ForegroundColor Yellow
  Invoke-Checked -Exe $PythonExe -CmdArgs @("-m", "venv", $VenvPath)
} else {
  Write-Host "Entorno ya existe: $VenvPath" -ForegroundColor Cyan
}

$VenvPython = Resolve-VenvPython -Path $VenvPath
if (!(Test-Path $VenvPython)) {
  throw "No se encontro python dentro del entorno: $VenvPython"
}

Write-Host "Actualizando pip/setuptools/wheel..." -ForegroundColor Yellow
Invoke-Checked -Exe $VenvPython -CmdArgs @("-m", "pip", "install", "--upgrade", "pip", "setuptools", "wheel")

if ($InstallOptionalOllamaClient.IsPresent) {
  Write-Host "Instalando cliente opcional de Ollama..." -ForegroundColor Yellow
  Invoke-Checked -Exe $VenvPython -CmdArgs @("-m", "pip", "install", "ollama")
}

Write-Host "Validando import del skill..." -ForegroundColor Yellow
Invoke-Checked -Exe $VenvPython -CmdArgs @(
  "-c",
  "from modules.humanoid.openclaw import register; s=register(config={'execution':{'backend':'script','dry_run':True}}); r=s.run('ATLAS, mueve el brazo derecho 10 grados'); assert r['ok'] is True; print('skill_ok')"
)

Write-Host ""
Write-Host "Entorno listo: $Workspace\$VenvPath" -ForegroundColor Green
Write-Host "Activar: .\$VenvPath\Scripts\Activate.ps1"
Write-Host "Registrar plugin: powershell -ExecutionPolicy Bypass -File scripts\register_openclaw_atlas_plugin.ps1"
Write-Host "Probar skill: .\$VenvPath\Scripts\python.exe -m modules.humanoid.openclaw.skill ""ATLAS, mueve el brazo derecho 10 grados"""
