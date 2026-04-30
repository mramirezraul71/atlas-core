
import subprocess, time, os

# Read credentials
cred_path = r'C:\dev\credenciales.txt'
env = os.environ.copy()
try:
    with open(cred_path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if '=' in line and not line.startswith('#') and not line.startswith('['):
                k, v = line.split('=', 1)
                env[k.strip()] = v.strip()
    print('Creds loaded. EXIT_GOVERNANCE=' + env.get('QUANT_EXIT_GOVERNANCE_ENABLED', '?'))
except Exception as e:
    print('Cred error:', e)

# Set the OS blocker env var
env['ATLAS_SANDBOX_RESTRICTED_SYMBOLS'] = 'OS'
env['QUANT_EXIT_GOVERNANCE_ENABLED'] = 'false'

PYTHON = r'C:\ATLAS_PUSH\venv\Scripts\python.exe'
CWD = r'C:\ATLAS_PUSH'

# Launch uvicorn for the quant API on port 8795
# Command: python -m uvicorn atlas_code_quant.api.main:app --host 0.0.0.0 --port 8795
ps_cmd = (
    '$env:ATLAS_SANDBOX_RESTRICTED_SYMBOLS = "OS"; '
    '$env:QUANT_EXIT_GOVERNANCE_ENABLED = "false"; '
    '$env:TRADIER_PAPER_TOKEN = "UqYAFhBY0sPWSP4OmuAHChHB0lAN"; '
    '$env:TRADIER_LIVE_TOKEN = "Fkcw9ysQaVGME3xt4tMWfPZNUGxv"; '
    '$proc = Start-Process '
    r'-FilePath "C:\ATLAS_PUSH\venv\Scripts\python.exe" '
    r'-ArgumentList "-m uvicorn atlas_code_quant.api.main:app --host 0.0.0.0 --port 8795 --log-level info" '
    r'-WorkingDirectory "C:\ATLAS_PUSH" '
    '-WindowStyle Hidden -PassThru; '
    'Write-Output "Launched PID=$($proc.Id)"'
)

r = subprocess.run(
    ['powershell', '-NoProfile', '-NonInteractive', '-Command', ps_cmd],
    capture_output=True, text=True, errors='ignore', timeout=20
)
print('PS rc:', r.returncode)
print('PS out:', r.stdout.strip())
print('PS err:', r.stderr[:300].strip())

# Wait a moment and check port
time.sleep(8)
r2 = subprocess.run('netstat -ano | findstr ":8795"', shell=True,
                    capture_output=True, text=True, errors='ignore', timeout=5)
print('Port 8795 after launch:', repr(r2.stdout[:300]))
