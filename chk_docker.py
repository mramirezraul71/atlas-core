
with open(r'C:\ATLAS_PUSH\immediate_start.py', 'r', encoding='utf-8', errors='ignore') as f:
    lines = f.readlines()
for i, l in enumerate(lines, 1):
    if any(k in l.lower() for k in ['docker', 'browser', 'chrome', 'camera', 'grafana', 'opencv']):
        print(f'L{i}: {l.rstrip()}')
