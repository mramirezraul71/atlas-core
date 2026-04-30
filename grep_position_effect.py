import os
import re

search_dir = r"C:\ATLAS_PUSH\atlas_code_quant"
pattern = re.compile(r'position_effect|open_or_close|close.*open|existing_position|has_position', re.IGNORECASE)

results = []
for root, dirs, files in os.walk(search_dir):
    # Skip venv and __pycache__
    dirs[:] = [d for d in dirs if d not in ['__pycache__', 'venv', '.git', 'node_modules']]
    for f in files:
        if f.endswith('.py'):
            full = os.path.join(root, f)
            try:
                with open(full, 'r', encoding='utf-8', errors='ignore') as fh:
                    for i, line in enumerate(fh, 1):
                        if pattern.search(line) and 'position_effect' in line.lower():
                            results.append(f"{full}:{i}: {line.rstrip()}")
            except:
                pass

for r in results[:30]:
    print(r)
print(f"\nTotal matches: {len(results)}")
