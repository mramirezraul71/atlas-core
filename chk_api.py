
import subprocess, requests

# Port check
r = subprocess.run('netstat -ano | findstr ":8795"', shell=True,
                   capture_output=True, text=True, errors='ignore', timeout=5)
print('Port 8795:', repr(r.stdout[:300]))

# PID 24684 alive?
r2 = subprocess.run('tasklist /FI "PID eq 24684"', shell=True,
                    capture_output=True, text=True, errors='ignore', timeout=5)
print('PID 24684:', r2.stdout.strip()[-200:])

# Health try
try:
    h = requests.get('http://127.0.0.1:8795/health', timeout=6)
    print('Health:', h.json())
except Exception as e:
    print('Health error:', str(e)[:100])

# Check env vars in the launched process
import os
print('Our ATLAS_SANDBOX_RESTRICTED_SYMBOLS:', os.getenv('ATLAS_SANDBOX_RESTRICTED_SYMBOLS', 'NOT SET'))
