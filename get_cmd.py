import subprocess

for pid in [39932, 39592, 26368, 43052]:
    result = subprocess.run(
        ['wmic', 'process', 'where', f'ProcessId={pid}', 'get', 'CommandLine', '/VALUE'],
        capture_output=True, text=True, timeout=10
    )
    output = result.stdout.strip()
    if output:
        print(f"PID {pid}:")
        print(f"  {output[:300]}")
    else:
        print(f"PID {pid}: no encontrado o sin cmdline")
    print()
