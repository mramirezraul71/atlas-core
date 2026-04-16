import subprocess
import re

# Usar wmic process list para ver todos
result = subprocess.run(
    ['wmic', 'process', 'get', 'ProcessId,CommandLine', '/FORMAT:CSV'],
    capture_output=True, text=True, timeout=30
)

# Filtrar líneas con python
for line in result.stdout.split('\n'):
    if 'python' in line.lower() and ('atlas' in line.lower() or 'uvicorn' in line.lower() or 'service' in line.lower()):
        print(line[:300])
