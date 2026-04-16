
import subprocess, time

ps = (
    '$env:ATLAS_SANDBOX_RESTRICTED_SYMBOLS = "OS"; '
    '$env:QUANT_EXIT_GOVERNANCE_ENABLED = "false"; '
    '$env:TRADIER_PAPER_TOKEN = "UqYAFhBY0sPWSP4OmuAHChHB0lAN"; '
    '$p = Start-Process '
    '-FilePath "C:\\ATLAS_PUSH\\venv\\Scripts\\python.exe" '
    '-ArgumentList @("-m","uvicorn","atlas_code_quant.api.main:app","--host","0.0.0.0","--port","8795","--log-level","info") '
    '-WorkingDirectory "C:\\ATLAS_PUSH" '
    '-RedirectStandardOutput "C:\\ATLAS_PUSH\\api_stdout.log" '
    '-RedirectStandardError "C:\\ATLAS_PUSH\\api_stderr.log" '
    '-NoNewWindow -PassThru; '
    'Start-Sleep 2; '
    'Write-Output "PID=$($p.Id) HasExited=$($p.HasExited)"'
)
r = subprocess.run(['powershell', '-NoProfile', '-NonInteractive', '-Command', ps],
                   capture_output=True, text=True, errors='ignore', timeout=20)
print('Launch:', r.stdout.strip(), r.stderr[:100].strip())

time.sleep(20)

# Check port and health
r2 = subprocess.run('netstat -ano | findstr ":8795"', shell=True,
                    capture_output=True, text=True, errors='ignore', timeout=5)
print('Port 8795:', repr(r2.stdout[:300]))

import requests as req
try:
    h = req.get('http://127.0.0.1:8795/health', timeout=6)
    print('Health:', h.json())
except Exception as e:
    print('Health error:', str(e)[:100])

# Check stderr for errors
try:
    with open(r'C:\ATLAS_PUSH\api_stderr.log', 'r', encoding='utf-8', errors='ignore') as f:
        err = f.read()
    if err.strip():
        print('Stderr:', err[-1500:])
    else:
        print('Stderr: clean')
except: pass
