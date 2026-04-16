
import subprocess, time, os, sys

print('=== RESTART v8 ===')

# Read creds
cred_path = r'C:\dev\credenciales.txt'
env = os.environ.copy()
try:
    with open(cred_path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if '=' in line and not line.startswith('#') and '=' in line:
                k, v = line.split('=', 1)
                env[k.strip()] = v.strip()
    print('Creds OK, EXIT_GOVERNANCE=' + env.get('QUANT_EXIT_GOVERNANCE_ENABLED', '?'))
except Exception as e:
    print('Cred error:', e)

PYTHON = r'C:\ATLAS_PUSH\venv\Scripts\python.exe'
LAUNCHER = r'C:\ATLAS_PUSH\immediate_start.py'

# Kill existing - try every method
pid = '39932'
methods = [
    f'taskkill /F /PID {pid}',
    f'taskkill /F /T /PID {pid}',
    'taskkill /F /IM python.exe /FI "PID eq ' + pid + '"',
]
for cmd in methods:
    r = subprocess.run(cmd, shell=True, capture_output=True, text=True, errors='ignore', timeout=8)
    print(f'Kill attempt "{cmd[:40]}": rc={r.returncode} out={r.stdout[:80].strip()} err={r.stderr[:80].strip()}')

time.sleep(3)

# Verify
r2 = subprocess.run('netstat -ano | findstr ":8795"', shell=True, capture_output=True, text=True, errors='ignore', timeout=5)
print('Port after kill:', repr(r2.stdout[:200]))
still_up = any(':8795' in l and 'LISTENING' in l for l in r2.stdout.splitlines())
print('Still listening:', still_up)

# Build env block for PowerShell
env_ps_lines = []
for k in ['QUANT_EXIT_GOVERNANCE_ENABLED', 'TRADIER_PAPER_TOKEN', 'TRADIER_LIVE_TOKEN',
          'ATLAS_SANDBOX_RESTRICTED_SYMBOLS']:
    v = env.get(k, '')
    if v:
        env_ps_lines.append(f'$env:{k}="{v}"')
env_ps_block = '; '.join(env_ps_lines)

# Method 1: PowerShell Start-Process (bypasses WinError 87)
ps_cmd = (
    f'PowerShell -WindowStyle Hidden -Command "'
    f'{env_ps_block}; '
    f'$env:ATLAS_SANDBOX_RESTRICTED_SYMBOLS=\\"OS\\"; '
    f'Start-Process -FilePath \\"{PYTHON}\\" -ArgumentList \\"{LAUNCHER}\\" '
    f'-WorkingDirectory \\"C:\\\\ATLAS_PUSH\\" -WindowStyle Hidden'
    f'"'
)
print('Trying PowerShell Start-Process...')
r3 = subprocess.run(ps_cmd, shell=True, capture_output=True, text=True, errors='ignore', timeout=15)
print(f'PS rc={r3.returncode} out={r3.stdout[:200]} err={r3.stderr[:200]}')

time.sleep(3)

# Check if up
r4 = subprocess.run('netstat -ano | findstr ":8795"', shell=True, capture_output=True, text=True, errors='ignore', timeout=5)
print('Port after PS launch:', repr(r4.stdout[:300]))

with open(r'C:\ATLAS_PUSH\atlas_restart.log', 'w') as f:
    f.write('Restart v8 at ' + time.ctime() + '\n')
    f.write('Exit governance: ' + env.get('QUANT_EXIT_GOVERNANCE_ENABLED', '?') + '\n')

print('Done')
