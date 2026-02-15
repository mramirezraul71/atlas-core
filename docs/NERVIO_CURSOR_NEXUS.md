# Nervio: Cursor → Cerebro → Nexus (ojos externos + manos)

El **nervio** es la capa que conecta el cerebro (módulo Cursor / plan con IA) con los efectores: **ojos externos** (Nexus Robot 8002) y **manos** (mouse/teclado en la máquina local).

## Flujo

```
[Usuario] Objetivo en panel Cursor
       → POST /cursor/run (goal, mode, resources, …)
       → cursor_run() genera plan (IA router, timeout 30 s)
       → Si mode=auto: _cursor_auto_execute(steps)
          → Por cada paso:
             - "ver pantalla" / "capture" / "screenshot" → nerve.eyes_capture(use_nexus=True)
             - "click" / "clic" → nerve.hands_locate(query) + nerve.hands_execute("click", {x,y})
             - "crear módulo" / "instalar" → selfprog (como antes)
       → Evidencia y resumen vuelven al dashboard
```

## Módulos

- **`modules/humanoid/nerve/`**
  - **eyes.py**: `eyes_capture(use_nexus_if_available=True, source="screen")`  
    Si Nexus está conectado (NEXUS_ENABLED, Robot 8002 responde), obtiene un frame desde `GET /api/vision/snapshot?source=screen`. Si no, usa captura local (hands_eyes).
  - **hands.py**: `hands_execute(action, payload)`, `hands_locate(query)`  
    Delegan en hands_eyes (mouse/teclado local). El nervio llega al mouse de esta máquina.

## Nexus Robot (8002)

- **GET /api/vision/snapshot?source=screen|camera**  
  Un frame JPEG para PUSH (ojos externos). Añadido para el nervio.
- **GET /api/vision/external/eyes**  
  Metadatos: `snapshot_url`, streams, capacidades.
- **POST /api/command**  
  Comandos de actuadores (bridge). Mouse/click siguen en PUSH vía hands_eyes.

## Variables de entorno

- `NEXUS_ENABLED`: activar uso de ojos externos cuando esté disponible.
- `NEXUS_ROBOT_API_URL`: base del Robot (por defecto http://127.0.0.1:8002).
- `NERVE_NEXUS_TIMEOUT`: timeout (s) para snapshot (por defecto 5).

## Resumen

- **Ojos**: local (pantalla) o Nexus (snapshot del Robot) según disponibilidad.
- **Manos**: siempre local (pyautogui); el nervio llega hasta el mouse de la máquina donde corre PUSH.
