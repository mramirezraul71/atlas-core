
import subprocess, sys

# Buscar procesos python con atlas_telegram_bot.py (el v2, sin _nlp)
result = subprocess.run(
    ['powershell', '-Command',
     'Get-WmiObject Win32_Process | Where-Object {\].CommandLine -like "*atlas_telegram_bot.py*" -and \].CommandLine -notlike "*nlp*"} | Select-Object ProcessId, CommandLine'],
    capture_output=True, text=True
)
print('Procesos bot v2:', result.stdout[:500])

# Matar por commandline
result2 = subprocess.run(
    ['powershell', '-Command',
     'Get-WmiObject Win32_Process | Where-Object {\].CommandLine -like "*atlas_telegram_bot.py*" -and \].CommandLine -notlike "*nlp*"} | ForEach-Object { Stop-Process -Id \].ProcessId -Force; Write-Host "Killed" \].ProcessId }'],
    capture_output=True, text=True
)
print('Kill result:', result2.stdout[:300])
print('Done')
