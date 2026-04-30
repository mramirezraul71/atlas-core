
import subprocess, time, os

# Read creds
env = os.environ.copy()
try:
    with open(r'C:\dev\credenciales.txt', 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if '=' in line and not line.startswith('#') and not line.startswith('['):
                k, v = line.split('=', 1)
                env[k.strip()] = v.strip()
except: pass

env['ATLAS_SANDBOX_RESTRICTED_SYMBOLS'] = 'OS'
env['QUANT_EXIT_GOVERNANCE_ENABLED'] = 'false'

PYTHON = r'C:\ATLAS_PUSH\venv\Scripts\python.exe'
# The original command from quant_main_stdout.log used this exact path
# uvicorn atlas_code_quant.api.main:app --host 0.0.0.0 --port 8795
# CWD = C:\ATLAS_PUSH

# Try with env set via PowerShell properly
ps_lines = [
    '$env:ATLAS_SANDBOX_RESTRICTED_SYMBOLS = "OS"',
    '$env:QUANT_EXIT_GOVERNANCE_ENABLED = "false"',
    '$env:TRADIER_PAPER_TOKEN = "UqYAFhBY0sPWSP4OmuAHChHB0lAN"',
    '$env:TRADIER_LIVE_TOKEN = "Fkcw9ysQaVGME3xt4tMWfPZNUGxv"',
    '$env:ATLAS_API_KEY = "atlas-quant-local"',
    'Set-Location "C:\\ATLAS_PUSH"',
    r'$proc = Start-Process -FilePath "C:\ATLAS_PUSH\venv\Scripts\python.exe" ' +
    r'-ArgumentList @("-m", "uvicorn", "atlas_code_quant.api.main:app", "--host", "0.0.0.0", "--port", "8795", "--log-level", "info") ' +
    r'-WorkingDirectory "C:\ATLAS_PUSH" ' +
    '-WindowStyle Hidden -PassThru',
    'Start-Sleep 2',
    'Write-Output "PID=$($proc.Id) ExitCode=$($proc.ExitCode)"',
]
ps_cmd = '; '.join(ps_lines)

r = subprocess.run(
    ['powershell', '-NoProfile', '-NonInteractive', '-Command', ps_cmd],
    capture_output=True, text=True, errors='ignore', timeout=20
)
print('PS rc:', r.returncode)
print('PS out:', r.stdout.strip()[:300])
print('PS err:', r.stderr[:300].strip())

time.sleep(5)

# Check port
r2 = subprocess.run('netstat -ano | findstr ":8795"', shell=True,
                    capture_output=True, text=True, errors='ignore', timeout=5)
print('Port 8795:', repr(r2.stdout[:300]))
