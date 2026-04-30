import os
import re

search_dir = r"C:\ATLAS_PUSH\atlas_code_quant"
pattern = re.compile(r'auto_cycle|_run_cycle|exit_pass|entry_pass|position_effect.*close|submit.*exit|exit.*submit', re.IGNORECASE)

results = []
for root, dirs, files in os.walk(search_dir):
    dirs[:] = [d for d in dirs if d not in ['__pycache__', 'venv', '.git', 'node_modules', 'tests']]
    for f in files:
        if f.endswith('.py'):
            full = os.path.join(root, f)
            try:
                with open(full, 'r', encoding='utf-8', errors='ignore') as fh:
                    for i, line in enumerate(fh, 1):
                        if pattern.search(line) and len(line.strip()) > 10:
                            results.append(f"{os.path.basename(full)}:{i}: {line.rstrip()[:100]}")
            except:
                pass

for r in results[:40]:
    print(r)
