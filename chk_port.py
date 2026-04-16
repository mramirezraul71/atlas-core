
import subprocess, requests, json

r = subprocess.run('netstat -ano | findstr ":8795"', shell=True, 
                   capture_output=True, text=True, errors='ignore', timeout=5)
print('Port 8795:', repr(r.stdout[:300]))

# Try health
try:
    h = requests.get('http://127.0.0.1:8795/health', timeout=5)
    print('Health:', h.json())
except Exception as e:
    print('Health error:', e)
