
import subprocess, time, os

# Check port 8795 now
r = subprocess.run('netstat -ano | findstr ":8795"', shell=True, 
                   capture_output=True, text=True, errors='ignore', timeout=5)
print('Port 8795:', repr(r.stdout[:400]))

# Check port 8791 (UI) and 8799 (MCP)
r2 = subprocess.run('netstat -ano | findstr ":879"', shell=True, 
                    capture_output=True, text=True, errors='ignore', timeout=5)
print('ATLAS ports (879x):', repr(r2.stdout[:500]))

# Check running python processes
r3 = subprocess.run('tasklist | findstr python', shell=True, 
                    capture_output=True, text=True, errors='ignore', timeout=5)
print('Python processes:', r3.stdout[:400])

# Read restart log if it exists
try:
    with open(r'C:\ATLAS_PUSH\atlas_restart.log', 'r') as f:
        print('Restart log:', f.read()[:300])
except:
    print('No restart log found')

# Check if there are any recent log files
r4 = subprocess.run('dir /B /O-D C:\\ATLAS_PUSH\\atlas_code_quant\\logs\\', shell=True,
                    capture_output=True, text=True, errors='ignore', timeout=5)
print('Log files:', r4.stdout[:300])
