
import subprocess, time, os

print('=== ATLAS RESTART v7 ===')

cred_path = r'C:\dev\credenciales.txt'
env = os.environ.copy()
try:
    with open(cred_path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if '=' in line and not line.startswith('#'):
                k, v = line.split('=', 1)
                env[k.strip()] = v.strip()
    print('Creds OK, EXIT_GOVERNANCE=' + env.get('QUANT_EXIT_GOVERNANCE_ENABLED', '?'))
except Exception as e:
    print('Cred error:', e)

# Kill parent PID 29136 (which owns 39932 on :8795)
for pid in ['39932', '29136']:
    try:
        kr = subprocess.run('taskkill /F /T /PID ' + pid, shell=True, 
                            capture_output=True, text=True, errors='ignore', timeout=10)
        print(f'Kill {pid}: {kr.stdout.strip()} {kr.stderr[:100].strip()}')
    except Exception as e:
        print(f'Kill {pid} error: {e}')

time.sleep(4)

# Verify port free
r = subprocess.run('netstat -ano | findstr ":8795"', shell=True, 
                   capture_output=True, text=True, errors='ignore', timeout=5)
listening = any(':8795' in l and 'LISTENING' in l for l in r.stdout.splitlines())
print('Port 8795 still listening:', listening)
print('Netstat:', repr(r.stdout[:300]))

# Launch via bat file using shell=True start
env_copy = env.copy()
try:
    cmd = 'start /B cmd /c "C:\\ATLAS_PUSH\\start_atlas.bat"'
    r2 = subprocess.run(cmd, shell=True, env=env_copy, 
                        capture_output=True, text=True, errors='ignore', timeout=5)
    print('Start /B rc:', r2.returncode, r2.stdout[:200], r2.stderr[:200])
except subprocess.TimeoutExpired:
    print('Timeout on start /B (process started in background)')
except Exception as e:
    print('Start /B error:', e)

with open(r'C:\ATLAS_PUSH\atlas_restart.log', 'w') as f:
    f.write('Restart attempt at ' + time.ctime() + '\n')

print('Done')
