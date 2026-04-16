
import subprocess

# Use findstr to locate the exit_pass and entry_pass sections
r = subprocess.run(
    'findstr /N "exit_pass entry_pass urgent_exit sorted_cands SANDBOX" C:\\ATLAS_PUSH\\atlas_code_quant\\api\\main.py',
    shell=True, capture_output=True, text=True, errors='ignore'
)
print('Found lines:')
print(r.stdout[:3000])
