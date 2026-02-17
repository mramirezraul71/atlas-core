"""Final system verification."""
import requests

print()
print('='*60)
print('        ATLAS - VERIFICACION FINAL DEL SISTEMA')
print('='*60)

# Servicios
services = [
    ('NEXUS (8000)', 'http://127.0.0.1:8000/health'),
    ('Robot (8002)', 'http://127.0.0.1:8002/api/health'),
    ('Dashboard (8791)', 'http://127.0.0.1:8791/health'),
    ('Ollama LLM (11434)', 'http://127.0.0.1:11434/api/tags'),
]

print()
print('[SERVICIOS]')
all_ok = True
for name, url in services:
    try:
        r = requests.get(url, timeout=5)
        status = 'OK' if r.status_code == 200 else 'FAIL'
        if r.status_code != 200:
            all_ok = False
    except:
        status = 'OFFLINE'
        all_ok = False
    print(f'  {name:25} {status}')

# Health
print()
print('[HEALTH SCORE]')
try:
    r = requests.get('http://127.0.0.1:8791/health', timeout=5)
    d = r.json()
    score = d.get('score', 0)
    incidents = d.get('checks', {}).get('ans_open_incidents', 0)
    print(f'  Score: {score}/100')
    print(f'  Incidentes abiertos: {incidents}')
except Exception as e:
    print(f'  Error: {e}')

# Bitacora
print()
print('[BITACORA]')
try:
    r = requests.get('http://127.0.0.1:8791/ans/bitacora?limit=20', timeout=5)
    d = r.json()
    if d.get('ok'):
        entries = d.get('entries', [])
        errors = [e for e in entries if not e.get('ok', True)]
        print(f'  Ultimas 20 entradas: {len(entries)}')
        print(f'  Errores recientes: {len(errors)}')
except Exception as e:
    print(f'  Error: {e}')

print()
print('='*60)
if all_ok and score == 100 and incidents == 0:
    print('           SISTEMA OPERATIVO - CERO ERRORES')
else:
    print('           ATENCION REQUERIDA')
print('='*60)
print()
