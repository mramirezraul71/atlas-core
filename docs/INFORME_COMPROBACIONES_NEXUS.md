# Informe final: comprobaciones NEXUS / Robot / PUSH

**Fecha:** 2025-02-14  
**Repo:** ATLAS_PUSH (atlas-core), worktree `crs`

---

## 1. Resumen ejecutivo

| Componente        | Puerto | Estado comprobado | Nota |
|-------------------|--------|-------------------|------|
| **PUSH (cerebro)** | 8791   | OK (200)          | Responde a /health. |
| **NEXUS (cuerpo)** | 8000   | Parcial           | Escucha y responde a /status 200; /health devuelve 404 en la instancia actual. |
| **Robot (cámaras)**| 8002   | NO                | No hay proceso escuchando; conexión rechazada. |

**Causa principal de "Desconectado" en el Dashboard:** El heartbeat PUSH usa solo `/health` en 8000. Si NEXUS no expone `/health` (o devuelve 404), el panel marca NEXUS como Desconectado aunque `/status` funcione.

**Acción realizada:** El heartbeat ahora prueba primero `/health` y, si falla (p. ej. 404), prueba `/status`. Así el Dashboard puede mostrar Conectado cuando NEXUS responde en `/status`. El script `check_nexus_ports.py` también prueba `/status` como respaldo.

---

## 2. Comprobaciones realizadas

### 2.1 Script de diagnóstico de puertos

```text
python scripts/check_nexus_ports.py
```

- **NEXUS (8000):** DESCONECTADO según script (GET /health → HTTP 404).  
- **Robot (8002):** DESCONECTADO (conexión rechazada, ningún proceso escuchando).

### 2.2 Puertos en escucha (netstat)

- **8000:** LISTENING (hay un servicio; no es el mismo código que expone /health en este repo o la ruta difiere).
- **8002:** No aparece → backend Robot no está arrancado.
- **8791:** LISTENING → PUSH en marcha.

### 2.3 Peticiones HTTP manuales

- `http://127.0.0.1:8791/health` → **200** (PUSH operativo).
- `http://127.0.0.1:8000/status` → **200** (NEXUS operativo en /status).
- `http://127.0.0.1:8000/health` → **404** (la instancia en 8000 no sirve /health).
- `http://127.0.0.1:8002/...` → conexión rechazada (Robot no arrancado).

### 2.4 Configuración

- **config/atlas.env:** NO EXISTE. Solo existe `config/atlas.env.example`.
- **Variables NEXUS en example:** NEXUS_ENABLED=true, NEXUS_BASE_URL=http://127.0.0.1:8000, NEXUS_ATLAS_PATH, NEXUS_ROBOT_PATH (rutas tipo C:\ATLAS_PUSH\nexus\...).

Sin `atlas.env`, PUSH puede estar usando valores por defecto de entorno; si NEXUS_ENABLED o NEXUS_BASE_URL no están definidos, el cliente NEXUS puede no intentar conexión o usar URL incorrecta.

### 2.5 Estructura del repo

- `nexus/atlas_nexus/nexus.py` → existe.
- `nexus/atlas_nexus_robot/backend/main.py` → existe.
- `nexus/atlas_nexus/api/rest_api.py` → define `/health` y `/status`; la instancia que corre en 8000 no responde en `/health` (posible versión antigua o otro despliegue).

---

## 3. Cambios aplicados en este informe

1. **modules/nexus_heartbeat.py**  
   - `ping_nexus()` ahora intenta primero `GET {base}/health` y, si falla (p. ej. 404), `GET {base}/status`.  
   - Si alguno devuelve 200, se considera NEXUS conectado. Con esto el Dashboard puede mostrar Conectado cuando NEXUS solo expone `/status`.

2. **scripts/check_nexus_ports.py**  
   - Si `GET .../health` devuelve 404, se prueba `GET .../status` para NEXUS (8000).  
   - El informe de diagnóstico ya no marca NEXUS como fallido solo por falta de `/health`.

---

## 4. Recomendaciones

| Prioridad | Acción |
|-----------|--------|
| 1 | Crear `config/atlas.env` a partir de `config/atlas.env.example` y definir NEXUS_ENABLED=true, NEXUS_BASE_URL=http://127.0.0.1:8000, y rutas NEXUS_ATLAS_PATH / NEXUS_ROBOT_PATH si se usa el monorepo. |
| 2 | Arrancar el backend Robot (8002) si se quieren cámaras: `cd nexus/atlas_nexus_robot/backend && python main.py` (o `uvicorn main:app --port 8002`). Así el panel de cámaras dejará de mostrar "Arranca el backend para ver el stream". |
| 3 | Asegurar que la instancia de NEXUS en 8000 sea la del repo actual (con `/health` en rest_api.py). Si 8000 lo sirve otro proceso, reiniciar NEXUS desde `nexus/atlas_nexus` con `python nexus.py --mode api`. |
| 4 | Tras cambios, ejecutar de nuevo `python scripts/check_nexus_ports.py` y recargar el Dashboard (8791/ui). Usar "Reconectar NEXUS" si aplica. |

---

## 5. Conclusión

- **PUSH (8791):** Operativo.  
- **NEXUS (8000):** Servicio en escucha; responde en `/status` pero no en `/health` en el entorno comprobado. Con el fallback a `/status` en heartbeat y en el script de diagnóstico, el Dashboard y las comprobaciones pueden considerar NEXUS conectado.  
- **Robot (8002):** No arrancado; hay que iniciar el backend para cámaras y stream.  
- **Config:** Crear `atlas.env` desde el example para fijar NEXUS_* y rutas del monorepo.

Documento de diagnóstico detallado: `docs/PROMPT_NEXUS_CLAUDE_MAPEO.md` (sección 7: Revisar por qué no se conecta).

---

## 6. Solución aplicada (adelante)

- **config/atlas.env** creado desde `atlas.env.example`; `NEXUS_ATLAS_PATH` y `NEXUS_ROBOT_PATH` ajustados a la ruta del worktree/repo actual para que `start_nexus_services.py` y el heartbeat encuentren NEXUS y Robot.
- **Heartbeat:** ya usa fallback a `/status` si `/health` falla (NEXUS marcado como conectado cuando responde en 8000).
- **Backend Robot (8002):** arrancado; en `nexus/atlas_nexus_robot/backend/main.py` se corrigió el `UnicodeEncodeError` en Windows (prints con emojis) y se puso `reload=False` para arranque en segundo plano.
- **Diagnóstico final:** `python scripts/check_nexus_ports.py` → NEXUS (8000): CONECTADO, Robot (8002): CONECTADO. El dashboard debería mostrar ambos como Conectado; si PUSH se inició antes de crear `atlas.env`, reiniciar PUSH para que cargue `NEXUS_ENABLED` y las rutas.
