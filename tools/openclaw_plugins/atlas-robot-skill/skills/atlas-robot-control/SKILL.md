# ATLAS Robot Control

Usa `atlas_robot_control` cuando el usuario quiera mover ATLAS o ejecutar acciones sobre actuadores.

Reglas:
- Pasa la orden completa en `textCommand`.
- Mantente conservador; si la orden es ambigua, usa una accion pequena o pide precision.
- El skill Python valida limites fisicos y puede activar parada de emergencia si detecta violacion.
- Usa `context` solo para informacion operativa util, por ejemplo `{"source":"openclaw"}`.

Ejemplos:
- `{"textCommand":"ATLAS, mueve el brazo derecho 10 grados"}`
- `{"textCommand":"ATLAS, abre la pinza derecha"}`
- `{"textCommand":"ATLAS, parada de emergencia"}`
