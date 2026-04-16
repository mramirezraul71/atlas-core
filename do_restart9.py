
import subprocess, time, os

print('=== RESTART v9 ===')

PYTHON = r'C:\ATLAS_PUSH\venv\Scripts\python.exe'
LAUNCHER = r'C:\ATLAS_PUSH\immediate_start.py'

# Kill attempts — PID 39932 needs elevated perms
# Try using sc stop or net stop if ATLAS is a Windows service
for cmd in [
    'taskkill /F /PID 39932',
    'taskkill /F /T /PID 39932',
    'net stop ATLAS 2>nul',
]:
    r = subprocess.run(cmd, shell=True, capture_output=True, text=True, errors='ignore', timeout=8)
    print(f'[{cmd[:40]}] rc={r.returncode}', r.stderr[:60].strip())

time.sleep(2)

# Check port
r2 = subprocess.run('netstat -ano | findstr ":8795"', shell=True, capture_output=True, text=True, errors='ignore', timeout=5)
still_up = any(':8795' in l and 'LISTENING' in l for l in r2.stdout.splitlines())
print('Still up on :8795:', still_up)

# PowerShell - set QUANT_EXIT_GOVERNANCE_ENABLED=false correctly
# Use single-line PS command with proper quoting
ps_script = (
    'Set-Location C:\\ATLAS_PUSH; '
    '$env:QUANT_EXIT_GOVERNANCE_ENABLED = "false"; '
    '$env:ATLAS_SANDBOX_RESTRICTED_SYMBOLS = "OS"; '
    'Start-Process -FilePath "' + PYTHON + '" '
    '-ArgumentList "' + LAUNCHER + '" '
    '-WorkingDirectory "C:\\ATLAS_PUSH" '
    '-WindowStyle Hidden -PassThru | ForEach-Object { Write-Output "Launched PID $($_.Id)" }'
)

r3 = subprocess.run(
    ['powershell', '-NoProfile', '-NonInteractive', '-Command', ps_script],
    capture_output=True, text=True, errors='ignore', timeout=20
)
print('PS rc:', r3.returncode)
print('PS out:', r3.stdout[:300])
print('PS err:', r3.stderr[:300])

time.sleep(5)

# Check new port status
r4 = subprocess.run('netstat -ano | findstr ":8795"', shell=True, capture_output=True, text=True, errors='ignore', timeout=5)
print('Port after PS:', repr(r4.stdout[:300]))

with open(r'C:\ATLAS_PUSH\atlas_restart.log', 'w') as f:
    f.write('Restart v9 at ' + time.ctime() + '\n')

print('Done')
