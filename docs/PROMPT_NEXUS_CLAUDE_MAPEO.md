# Mapeo: Prompt Claude Sonnet 4.5 (NEXUS) a repo ATLAS_PUSH

**Origen:** Prompt usado en Nexus para diagnosticar "robot no funciona" y restaurar conexion PUSH-NEXUS.  
**Repo actual:** atlas-core (ATLAS_PUSH), monorepo con `nexus/` dentro.

---

## 1. Resumen del prompt Claude

- **Objetivo:** Restaurar conexion PUSH (8791) y NEXUS (8000), resiliencia, Triada de Crecimiento con logging industrial, validacion fisica del handshake.
- **Fases:** Clonar repo, Diagnosticar (handshake, bitacora, gobernanza), Implementar (ConnectionManager, BitacoraANS, Triada, PhysicalValidation, main.py), Instalar y probar.

---

## 2. Estructura que asumia el prompt vs estructura real

| Prompt asumia | En ATLAS_PUSH (real) |
|---------------|----------------------|
| atlas-core/nexus y push con main.py cada uno | PUSH = atlas_adapter/atlas_http_api.py (FastAPI, 8791). NEXUS = nexus/atlas_nexus/nexus.py (8000). |
| shared/connection_manager.py (nuevo) | No existia. Conexion via modules/nexus_client.py (urllib, sin retry) y modules/nexus_heartbeat.py. |
| shared/bitacora_ans.py (nuevo) | Existe modules/humanoid/ans/evolution_bitacora.py: bitacora en memoria (deque 300), POST /ans/evolution-log. Sin log a archivo. |
| Triada en atlas_evolution.py | Existe evolution_daemon.py y logica ANS; no hay Triada SCAN-SANDBOX-VALIDATE-ASSIMILATE como en el prompt. |
| shared/physical_validation.py | Existe "Prueba de Nervios" en nexus_actions.run_nerve_test(), invocada al startup de PUSH. |
| Dashboard en localhost:5173 | Dashboard PUSH = http://127.0.0.1:8791/ui. Robot = 5174. |

---

## 3. Archivos criticos en ATLAS_PUSH

| Funcion | Archivo(s) |
|---------|------------|
| Conexion PUSH a NEXUS (estado panel) | modules/nexus_client.py: get_nexus_status(), NEXUS_BASE_URL, timeout 5s, sin retry. |
| Heartbeat y auto-reactivacion NEXUS | modules/nexus_heartbeat.py: ping_nexus(), start_heartbeat(), restart_nexus(), callback a /ans/evolution-log. |
| Bitacora evolution | modules/humanoid/ans/evolution_bitacora.py: append_evolution_log(), get_evolution_entries(). Solo memoria. |
| Endpoint evolution-log | atlas_adapter/atlas_http_api.py: POST /ans/evolution-log. |
| Inicio heartbeat + callback | atlas_adapter/atlas_http_api.py lifespan: register_status_callback, start_heartbeat(), nexus_actions.run_nerve_test(). |
| Config puertos | config/atlas.env.example: ACTIVE_PORT=8791, NEXUS_BASE_URL, NEXUS_ATLAS_PATH. |
| Dashboard unificado | atlas_adapter/static/dashboard.html: consume /nexus/status y estado heartbeat. |

---

## 4. Problemas que el prompt identificaba vs estado en repo

| Problema (prompt) | Estado en ATLAS_PUSH |
|-------------------|----------------------|
| Falta retry con exponential backoff | Parcial: heartbeat cada 15s; tras 2 fallos llama restart_nexus(). No backoff 5-10-30-60s. nexus_client un solo intento. |
| Timeouts | Timeout fijo 5s (NEXUS_TIMEOUT). |
| No healthcheck periodico | Cubierto: heartbeat GET /health cada 15s. |
| Conexion bloqueante sin async | Cierto: urllib sincrono. |
| Bitacora silenciada | Bitacora en memoria; no log a archivo. Dashboard recibe callback en cada cambio. |
| CONECTADO no se muestra | Depende de NEXUS en 8000 y de get_nexus_status() y callback. Si NEXUS no arranca o URL mal = Desconectado. |

---

## 5. Soluciones del prompt aplicables aqui

1. **ConnectionManager con retry exponencial:** Anadir retries con delays [5,10,30,60] en nexus_client o en heartbeat antes de restart_nexus().
2. **Bitacora a archivo:** En evolution_bitacora opcionalmente escribir en logs/bitacora_ans_YYYYMMDD.log.
3. **Triada SCAN-SANDBOX-VALIDATE-ASSIMILATE:** Opcional; integrar con evolution_daemon y append_evolution_log por fase.
4. **Validacion fisica:** Refinar nexus_actions.run_nerve_test() para registrar en bitacora handshake fisico OK/KO.
5. **main.py:** No aplicable; entrypoint PUSH es uvicorn atlas_adapter.atlas_http_api:app. Startup ya tiene heartbeat y callback.

---

## 6. Pruebas recomendadas

1. PUSH: `python -m uvicorn atlas_adapter.atlas_http_api:app --host 127.0.0.1 --port 8791`
2. NEXUS: `python nexus/atlas_nexus/nexus.py --mode api` o scripts/start_nexus_services.py
3. GET http://127.0.0.1:8000/health (NEXUS) y http://127.0.0.1:8791/health (PUSH)
4. Dashboard http://127.0.0.1:8791/ui debe mostrar Robot Conectado si NEXUS esta arriba
5. Bitacora: mensajes "[CONEXION] Buscando NEXUS en puerto 8000... OK/Desconectado."

---

## 7. Revisar por qué no se conecta (diagnóstico)

Cuando el **Dashboard** muestra **"NEXUS (8000): Desconectado | Robot (8002): Desconectado"** y la **Bitácora ANS** repite **"Error [CONEXIÓN] Buscando NEXUS en puerto 8000"**, pero aparecen entradas **OK [NEXUS] Ejecutando autodiagnóstico** y **OK [MOUSE] Movimiento verificado**, significa:

- **PUSH está en marcha** (bitácora, Prueba de Nervios y callback funcionan).
- **La conexión HTTP de PUSH hacia NEXUS (8000) y/o Robot (8002) falla**: el panel y el heartbeat no reciben respuesta.

### 7.1 Qué revisar en orden

| Paso | Comprobación | Acción si falla |
|------|----------------|------------------|
| 1 | ¿NEXUS escucha en 8000? | Desde la raíz del repo: `python scripts/check_nexus_ports.py` (comprueba 8000 y 8002). O abrir en navegador `http://127.0.0.1:8000/health`. Si no responde, arrancar NEXUS. |
| 2 | ¿Robot backend escucha en 8002? | Abrir `http://127.0.0.1:8002/api/health` (o `/api/camera/service/status`). Si no responde, arrancar el backend del robot. |
| 3 | Variables de entorno | En `config/atlas.env`: `NEXUS_ENABLED=true`, `NEXUS_BASE_URL=http://127.0.0.1:8000`. Si usas otro host/puerto, ajustar. |
| 4 | Rutas del monorepo | Si NEXUS está dentro del repo: `NEXUS_ATLAS_PATH=C:\ATLAS_PUSH\nexus\atlas_nexus`, `NEXUS_ROBOT_PATH=C:\ATLAS_PUSH\nexus\atlas_nexus_robot\backend`. Si `start_nexus_services.py` usa rutas por defecto `C:\ATLAS_NEXUS\...`, no encontrará los módulos; definir estas variables o ejecutar desde la raíz del repo. |
| 5 | Orden de arranque | 1) NEXUS (8000), 2) Robot backend (8002) si quieres cámaras, 3) PUSH (8791). O usar `scripts/start_nexus_services.py` y luego PUSH. |
| 6 | Firewall / antivirus | Que no bloquee localhost 8000/8002. Probar con navegador en la misma máquina. |
| 7 | Puerto ocupado | Si otro proceso usa 8000 u 8002, NEXUS/Robot no arrancan. Liberar puerto o cambiar puerto en la config de NEXUS/Robot. |

### 7.2 Cómo arrancar NEXUS y Robot (monorepo)

Desde la raíz ATLAS_PUSH:

```powershell
# Opción A: script que arranca ambos (usa NEXUS_ATLAS_PATH y NEXUS_ROBOT_PATH)
$env:NEXUS_ATLAS_PATH = "C:\ATLAS_PUSH\nexus\atlas_nexus"
$env:NEXUS_ROBOT_PATH = "C:\ATLAS_PUSH\nexus\atlas_nexus_robot\backend"
python scripts/start_nexus_services.py

# Opción B: manual, dos terminales
# Terminal 1 - NEXUS (8000)
cd nexus\atlas_nexus
python nexus.py --mode api

# Terminal 2 - Robot backend (8002)
cd nexus\atlas_nexus_robot\backend
python main.py
# o: python -m uvicorn main:app --host 0.0.0.0 --port 8002
```

Luego arrancar PUSH (8791) y abrir el dashboard en `http://127.0.0.1:8791/ui`. El botón **"Reconectar NEXUS"** vuelve a intentar el handshake.

### 7.3 Interpretación rápida de Bitácora y Dashboard

- **Solo errores "Buscando NEXUS en puerto 8000"** → NEXUS no está en marcha o no escucha en 8000; seguir 7.1 y 7.2.
- **"Robot (8002) no disponible. Arranca el backend para ver el stream"** → Backend del robot (cámaras) no arrancado; arrancar servicio en 8002.
- **OK [NEXUS] autodiagnóstico** y **OK [MOUSE]** pero Dashboard Desconectado → Las rutinas locales de PUSH funcionan; el fallo es la petición HTTP a 8000/8002 (servicio no escucha, URL incorrecta o bloqueo de red).

---

## 8. Conclusion

El prompt encaja con este repo. Prioridades: 1) Retry con backoff en nexus_client/heartbeat. 2) Bitacora a archivo opcional. 3) Triada y PhysicalValidation como ampliaciones opcionales. Para no conectar: usar sección 7 (revisar por qué no se conecta) y el script de diagnóstico de puertos.
