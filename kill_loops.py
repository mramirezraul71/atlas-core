
import subprocess

# Kill autonomous_loop.py instances (PIDs 22976 and 39980)
# Kill trade_monitor.py instances (PIDs 44112 and 46680)
targets = ['22976', '39980', '44112', '46680']
for pid in targets:
    r = subprocess.run(f'taskkill /F /PID {pid}', shell=True, 
                       capture_output=True, text=True, errors='ignore', timeout=8)
    print(f'Kill {pid}: rc={r.returncode} {r.stdout.strip()} {r.stderr[:60].strip()}')

# Also kill by script name to catch any others
for name in ['autonomous_loop.py', 'trade_monitor.py', 'os_blocker']:
    r = subprocess.run(
        f'powershell -NoProfile -Command "Get-WmiObject Win32_Process | Where-Object {{$_.CommandLine -like \'*{name}*\'}} | ForEach-Object {{ Stop-Process -Id $_.ProcessId -Force; Write-Output (\'Killed \' + $_.ProcessId) }}"',
        shell=True, capture_output=True, text=True, errors='ignore', timeout=15
    )
    print(f'Kill by name {name}: {r.stdout.strip()} {r.stderr[:60].strip()}')

# Verify
import time
time.sleep(2)
r2 = subprocess.run(
    'powershell -NoProfile -Command "Get-WmiObject Win32_Process | Where-Object {$_.CommandLine -like \'*autonomous_loop*\' -or $_.CommandLine -like \'*trade_monitor*\'} | Select-Object ProcessId,CommandLine | Format-List"',
    shell=True, capture_output=True, text=True, errors='ignore', timeout=15
)
print('Remaining:', r2.stdout.strip() if r2.stdout.strip() else 'NONE — all killed')
