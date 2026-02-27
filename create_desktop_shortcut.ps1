# ==============================================
# ATLAS - Crear Acceso Directo en Escritorio
# ==============================================

$ErrorActionPreference = "SilentlyContinue"

# Rutas
$DesktopPath = [Environment]::GetFolderPath('Desktop')
$AtlasRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$LauncherPath = Join-Path $AtlasRoot "atlas_launcher.py"
$IconPath = Join-Path $AtlasRoot "atlas_adapter\static\favicon.ico"

# Buscar Python
$PythonExe = (Get-Command python -ErrorAction SilentlyContinue).Source
if (-not $PythonExe) {
    $PythonExe = (Get-Command python3 -ErrorAction SilentlyContinue).Source
}
if (-not $PythonExe) {
    # Buscar en paths conocidos
    $KnownPaths = @(
        "$env:LOCALAPPDATA\Programs\Python\Python312\python.exe",
        "$env:LOCALAPPDATA\Programs\Python\Python311\python.exe",
        "$env:LOCALAPPDATA\Programs\Python\Python310\python.exe",
        "C:\Python312\python.exe",
        "C:\Python311\python.exe",
        "C:\Python310\python.exe"
    )
    foreach ($p in $KnownPaths) {
        if (Test-Path $p) {
            $PythonExe = $p
            break
        }
    }
}

if (-not $PythonExe) {
    Write-Host "[ERROR] Python no encontrado" -ForegroundColor Red
    exit 1
}

Write-Host "================================================" -ForegroundColor Cyan
Write-Host "     ATLAS - Creando Acceso Directo" -ForegroundColor Cyan
Write-Host "================================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "[INFO] Python: $PythonExe" -ForegroundColor Green
Write-Host "[INFO] Launcher: $LauncherPath" -ForegroundColor Green
Write-Host "[INFO] Desktop: $DesktopPath" -ForegroundColor Green
Write-Host ""

# Crear shortcut
$ShortcutPath = Join-Path $DesktopPath "ATLAS.lnk"

# Eliminar shortcut anterior si existe
if (Test-Path $ShortcutPath) {
    Remove-Item $ShortcutPath -Force
    Write-Host "[INFO] Acceso directo anterior eliminado" -ForegroundColor Yellow
}

# Crear COM object
$WScriptShell = New-Object -ComObject WScript.Shell
$Shortcut = $WScriptShell.CreateShortcut($ShortcutPath)

# Configurar shortcut
$Shortcut.TargetPath = $PythonExe
$Shortcut.Arguments = "`"$LauncherPath`" --ui"
$Shortcut.WorkingDirectory = $AtlasRoot
$Shortcut.Description = "ATLAS Cognitive Brain Architecture - Sistema Completo"
$Shortcut.WindowStyle = 1  # Normal window

# Icono (si existe)
if (Test-Path $IconPath) {
    $Shortcut.IconLocation = $IconPath
} else {
    # Usar icono de Python por defecto
    $Shortcut.IconLocation = "$PythonExe,0"
}

# Guardar
$Shortcut.Save()

Write-Host ""
Write-Host "[OK] Acceso directo creado: ATLAS.lnk" -ForegroundColor Green
Write-Host ""
Write-Host "================================================" -ForegroundColor Cyan
Write-Host "  El acceso directo iniciara:" -ForegroundColor White
Write-Host "  - Servidor HTTP (Dashboard + API)" -ForegroundColor White
Write-Host "  - Sistema Cognitivo completo" -ForegroundColor White
Write-Host "  - Voice Assistant" -ForegroundColor White
Write-Host "  - Sistema de Autonomia" -ForegroundColor White
Write-Host "  - Abrira el navegador automaticamente" -ForegroundColor White
Write-Host "================================================" -ForegroundColor Cyan
Write-Host ""
