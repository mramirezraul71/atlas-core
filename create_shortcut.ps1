# Script para crear acceso directo de ATLAS Cognitive System
$WshShell = New-Object -ComObject WScript.Shell
$Shortcut = $WshShell.CreateShortcut("$env:USERPROFILE\Desktop\ATLAS Cognitive System.lnk")
$Shortcut.TargetPath = "python.exe"
$Shortcut.Arguments = "`"C:\ATLAS_PUSH\start_atlas.py`" --foreground"
$Shortcut.WorkingDirectory = "C:\ATLAS_PUSH"
$Shortcut.Description = "Iniciar ATLAS Cognitive Brain Architecture - Sistema Completo"
$Shortcut.IconLocation = "C:\Windows\System32\shell32.dll,21"
$Shortcut.Save()
Write-Host "Acceso directo creado en el escritorio: ATLAS Cognitive System.lnk"
