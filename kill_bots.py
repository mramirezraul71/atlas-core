import subprocess

# Usar tasklist para encontrar procesos python con bot
r = subprocess.run(
    ['tasklist', '/FI', 'IMAGENAME eq python.exe', '/V', '/FO', 'CSV'],
    capture_output=True, text=True, shell=True
)
print('tasklist output:')
print(r.stdout[:1000])

# Matar todos los python (incluye bots) - luego relanzamos uno limpio
r2 = subprocess.run(['taskkill', '/F', '/IM', 'python.exe'], capture_output=True, text=True)
print('Kill result:', r2.stdout.strip(), r2.stderr.strip())
