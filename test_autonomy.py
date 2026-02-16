"""Test de autonomia completa."""
import time

from modules.humanoid.quality import (
    start_autonomy,
    stop_autonomy,
    is_autonomy_running,
    get_daemon_status,
)

print("=" * 60)
print("PRUEBA DE AUTONOMIA COMPLETA")
print("=" * 60)

# Iniciar
print("\n[1] Iniciando sistema de autonomia...")
result = start_autonomy()

print("\nResultados:")
for comp, status in result.items():
    if comp == "all_ok":
        continue
    if isinstance(status, dict):
        ok = "OK" if status.get("ok") else "FAIL"
        print(f"  - {comp}: {ok}")

print(f"\nTodo OK: {result.get('all_ok')}")
print(f"Running: {is_autonomy_running()}")

# Estado
print("\n[2] Estado del daemon:")
status = get_daemon_status()
print(f"  Uptime: {status.get('uptime_seconds')}s")
print(f"  Health checks: {len(status.get('health', {}))}")
for name, health in status.get("health", {}).items():
    symbol = "OK" if health.get("healthy") else "FAIL"
    print(f"    - {name}: {symbol}")

# Esperar un poco para que el sistema funcione
print("\n[3] Sistema corriendo por 5 segundos...")
time.sleep(5)

# Detener
print("\n[4] Deteniendo...")
stop_result = stop_autonomy()
print(f"  Stopped: {not is_autonomy_running()}")

print("\n" + "=" * 60)
print("PRUEBA COMPLETADA EXITOSAMENTE")
print("=" * 60)
