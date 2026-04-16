
import subprocess, requests, json, time

for attempt in range(6):
    r = subprocess.run('netstat -ano | findstr ":8795"', shell=True,
                       capture_output=True, text=True, errors='ignore', timeout=5)
    listening = any(':8795' in l and 'LISTENING' in l for l in r.stdout.splitlines())
    print(f'Attempt {attempt+1}: Port 8795 listening={listening}')
    print('  netstat:', repr(r.stdout[:200]))
    
    if listening:
        try:
            h = requests.get('http://127.0.0.1:8795/health', timeout=8)
            print('  Health:', h.json())
        except Exception as e:
            print('  Health error:', e)
        break
    
    if attempt < 5:
        time.sleep(10)

# Also check if PID 24684 is still running
r2 = subprocess.run('tasklist | findstr "24684"', shell=True,
                    capture_output=True, text=True, errors='ignore', timeout=5)
print('PID 24684 status:', r2.stdout.strip() or 'NOT FOUND')

# Check all python processes
r3 = subprocess.run('tasklist | findstr python', shell=True,
                    capture_output=True, text=True, errors='ignore', timeout=5)
print('Python processes:', r3.stdout[:400])
