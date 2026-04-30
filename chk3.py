
import subprocess, requests

r = subprocess.run('netstat -ano | findstr ":8795"', shell=True,
                   capture_output=True, text=True, errors='ignore', timeout=5)
print('Port 8795:', repr(r.stdout[:300]))

# Check PID 18392
r2 = subprocess.run('tasklist /FI "PID eq 18392"', shell=True,
                    capture_output=True, text=True, errors='ignore', timeout=5)
print('PID 18392:', r2.stdout[-200:].strip())

try:
    h = requests.get('http://127.0.0.1:8795/health', timeout=8)
    print('Health:', h.json())
except Exception as e:
    print('Health error:', str(e)[:150])
