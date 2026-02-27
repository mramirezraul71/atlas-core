# Script para crear acceso directo unificado de ATLAS
$WshShell = New-Object -ComObject WScript.Shell
$Shortcut = $WshShell.CreateShortcut("$env:USERPROFILE\Desktop\ATLAS.lnk")
$Shortcut.TargetPath = "python.exe"
$Shortcut.Arguments = "`"C:\ATLAS_PUSH\atlas_launcher.py`" --ui"
$Shortcut.WorkingDirectory = "C:\ATLAS_PUSH"
$Shortcut.Description = "ATLAS - Cognitive Brain Architecture - Sistema Completo"
$Shortcut.IconLocation = "C:\Windows\System32\shell32.dll,21"
$Shortcut.Save()
Write-Host "Acceso directo unificado creado: ATLAS.lnk"
