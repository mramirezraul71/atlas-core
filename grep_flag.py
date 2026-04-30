import os

search_dir = r"C:\ATLAS_PUSH\atlas_code_quant"
results = []
for root, dirs, files in os.walk(search_dir):
    dirs[:] = [d for d in dirs if d not in ['__pycache__', '.git', 'node_modules', 'tests']]
    for f in files:
        if f.endswith('.py'):
            full = os.path.join(root, f)
            try:
                with open(full, 'r', encoding='utf-8', errors='ignore') as fh:
                    content = fh.read()
                    if 'exit_governance_disabled' in content or 'exit_governance_enabled' in content.lower():
                        for i, line in enumerate(content.split('\n'), 1):
                            if 'exit_governance_disabled' in line or 'exit_governance_enabled' in line.lower() or 'disabled.flag' in line:
                                results.append(f"{os.path.basename(full)}:{i}: {line.rstrip()[:100]}")
            except:
                pass

for r in results[:30]:
    print(r)
