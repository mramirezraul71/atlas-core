
import subprocess, os, time

CWD = r'C:\ATLAS_PUSH'
LOG = r'C:\ATLAS_PUSH\api_launch.log'

# Launch using PowerShell which CAN set env vars and start processes
# Use -Command with separate -EnvVariable approach
ps = (
    'Set-Location "C:\\ATLAS_PUSH"; '
    '$env:ATLAS_SANDBOX_RESTRICTED_SYMBOLS = "OS"; '
    '$env:QUANT_EXIT_GOVERNANCE_ENABLED = "false"; '
    '$p = Start-Process '
    '-FilePath "C:\\ATLAS_PUSH\\venv\\Scripts\\python.exe" '
    '-ArgumentList @("-m","uvicorn","atlas_code_quant.api.main:app","--host","0.0.0.0","--port","8795","--log-level","info") '
    '-WorkingDirectory "C:\\ATLAS_PUSH" '
    '-RedirectStandardOutput "C:\\ATLAS_PUSH\\api_stdout.log" '
    '-RedirectStandardError "C:\\ATLAS_PUSH\\api_stderr.log" '
    '-NoNewWindow -PassThru; '
    'Start-Sleep 3; '
    'Write-Output "PID=$($p.Id) HasExited=$($p.HasExited)"'
)

r = subprocess.run(
    ['powershell', '-NoProfile', '-NonInteractive', '-Command', ps],
    capture_output=True, text=True, errors='ignore', timeout=20
)
print('PS out:', r.stdout.strip()[:300])
print('PS err:', r.stderr[:200].strip())

time.sleep(12)

# Read error log
for log_file in [r'C:\ATLAS_PUSH\api_stderr.log', r'C:\ATLAS_PUSH\api_stdout.log']:
    try:
        with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
        if content.strip():
            print(f'\n{log_file}:')
            print(content[:2000])
    except Exception as e:
        print(f'{log_file} error: {e}')

# Port check
r2 = subprocess.run('netstat -ano | findstr ":8795"', shell=True,
                    capture_output=True, text=True, errors='ignore', timeout=5)
print('\nPort 8795:', repr(r2.stdout[:300]))
