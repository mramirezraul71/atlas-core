param(
  [string]$Workspace = "C:\ATLAS_PUSH",
  [string]$PythonExe = "python",
  [string]$TargetPython = "3.11",
  [switch]$BackupAndRecreateMismatched
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

function Get-VenvVersion {
  param([string]$VenvPath)
  $py = Join-Path $VenvPath "Scripts\python.exe"
  if (!(Test-Path $py)) {
    return ""
  }
  $v = & $py -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')"
  return ($v | Out-String).Trim()
}

function Install-ServiceVenv {
  param(
    [string]$Name,
    [string]$VenvPath,
    [string]$RequirementsPath,
    [string]$ConstraintsPath
  )

  Write-Host "== $Name ==" -ForegroundColor Cyan
  if (Test-Path $VenvPath) {
    $current = Get-VenvVersion -VenvPath $VenvPath
    if ($current -and $current -ne $TargetPython) {
      if ($BackupAndRecreateMismatched) {
        $ts = Get-Date -Format "yyyyMMdd-HHmmss"
        $backup = "$VenvPath.py$current.bak-$ts"
        Write-Host "Venv con Python $current detectado en $VenvPath; moviendo a backup: $backup" -ForegroundColor Yellow
        Move-Item -Path $VenvPath -Destination $backup -Force
      } else {
        throw "Venv $VenvPath usa Python $current (esperado $TargetPython). Reintenta con -BackupAndRecreateMismatched."
      }
    }
  }

  if (!(Test-Path $VenvPath)) {
    Write-Host "Creando venv: $VenvPath" -ForegroundColor Yellow
    Invoke-Checked -Exe $PythonExe -CmdArgs @("-m", "venv", $VenvPath)
  }

  $py = Join-Path $VenvPath "Scripts\python.exe"
  if (!(Test-Path $py)) {
    throw "No se encontró Python en venv: $py"
  }

  Write-Host "Actualizando pip/setuptools/wheel..." -ForegroundColor Yellow
  Invoke-Checked -Exe $py -CmdArgs @("-m", "pip", "install", "--upgrade", "pip", "setuptools", "wheel")

  Write-Host "Instalando dependencias con constraints..." -ForegroundColor Yellow
  Invoke-Checked -Exe $py -CmdArgs @("-m", "pip", "install", "-r", $RequirementsPath, "-c", $ConstraintsPath)

  Write-Host "Validando pip check..." -ForegroundColor Yellow
  Invoke-Checked -Exe $py -CmdArgs @("-m", "pip", "check")
}

Install-ServiceVenv `
  -Name "PUSH" `
  -VenvPath ".venv_push" `
  -RequirementsPath $(
    if (Test-Path "requirements.push.txt") {
      "requirements.push.txt"
    } else {
      "requirements.txt"
    }
  ) `
  -ConstraintsPath "constraints\push-base.txt"

Install-ServiceVenv `
  -Name "NEXUS" `
  -VenvPath ".venv_nexus" `
  -RequirementsPath "nexus\atlas_nexus\requirements.txt" `
  -ConstraintsPath "constraints\nexus-base.txt"

Install-ServiceVenv `
  -Name "ROBOT" `
  -VenvPath ".venv_robot" `
  -RequirementsPath "nexus\atlas_nexus_robot\backend\requirements.txt" `
  -ConstraintsPath "constraints\robot-base.txt"

Write-Host ""
Write-Host "Entornos listos:" -ForegroundColor Green
Write-Host "  PUSH  -> $Workspace\.venv_push"
Write-Host "  NEXUS -> $Workspace\.venv_nexus"
Write-Host "  ROBOT -> $Workspace\.venv_robot"
