# ATLAS: Arquitectura Cognitiva

## Autoconocimiento (Self-Model)

El sistema **se conoce por dentro**. No es una caja negra.

- **Módulo:** `modules/humanoid/self_model/`
- **API:** `GET /ans/self-model` o `GET /humanoid/self-model`
- **Manifiesto:** Anatomía del sistema (cerebro, nervios, órganos, checks, heals, dependencias)
- **Descubrimiento:** Checks y heals se leen del registry real; no hay duplicación

### Estructura del manifiesto

```
anatomy/
  brain/        → Cerebro: LLM, coherencia, lógica
  nervous_system/ → ANS: checks, heals, dependencias
  kernel/       → Módulos registrados
  organs/       → memory, audit, scheduler, llm, api, deps, disk, logs
dependency_graph/ → check_to_heal (qué heal aplica a qué check)
```

---

## Señales en vivo (Live Stream)

**Estilo Cursor:** Ver en tiempo real lo que ejecuta el sistema.

- **API SSE:** `GET /ans/live` (Server-Sent Events)
- **API polling:** `GET /ans/live/recent?limit=100`
- **Dashboard:** Panel "Consola en vivo" con botón "Iniciar stream"

### Eventos emitidos

| phase       | Descripción                    |
|-------------|--------------------------------|
| cycle_start | Inicio del ciclo ANS           |
| check_start | Comienza un check              |
| check_end   | Resultado del check (OK/FAIL)  |
| incident    | Incidente creado               |
| heal_attempt| Intento de heal                |
| heal_end    | Resultado del heal             |
| skip        | Heal omitido (governed, límite, etc.) |
| cycle_end   | Fin del ciclo con resumen      |

---

## Cerebro funcional (Diagnoser)

El cerebro **analiza** incidentes usando el self-model:

- **Módulo:** `modules/humanoid/brain/ans_diagnoser.py`
- **Función:** `diagnose_with_self_model(incidents, actions, manifest)`
- **Salida:** Causas raíz, heals bloqueados/disponibles, recomendaciones
- **Integración:** Cada reporte ANS incluye el diagnóstico del cerebro

---

## Sistema nervioso integrado

Flujo: **checks → incidente → decisión (governance/policy/limits) → heal → verificación**

- **Trazabilidad:** Bitácora + live stream + reporte con diagnóstico
- **Bypass governance:** `SYSTEM_MODE=aggressive` o `GOVERNANCE_MODE=growth` permite heals sin aprobación
- **Heals seguros:** Solo los de `SAFE_HEALS` se ejecutan automáticamente

---

## Endpoints clave

| Ruta              | Descripción                          |
|-------------------|--------------------------------------|
| `/ans/self-model` | Autoconocimiento (manifest)          |
| `/ans/live`       | SSE stream en vivo                   |
| `/ans/live/recent`| Últimos eventos (polling)            |
| `/humanoid/self-model` | Mismo manifest (alternativo)   |
| `/ans/run-now`    | Ejecutar ciclo ANS manualmente       |
| `/ans/bitacora`   | Bitácora: problema → acción → resultado |

---

## Integración MakePlay / Make.com

ATLAS envía eventos en tiempo real a un webhook externo (Make.com, Zapier, etc.) para feedback loop continuo.

### Flujo

1. **Eventos ANS en vivo** → Cada evento del live stream (check, heal, incident) se POST al webhook
2. **Scanner permanente** → Job que corre cada 60s (configurable), envía snapshot: health_score, incidents_open, last_actions, self_model
3. **Make.com** → Crea escenario con "Custom webhook", recibe JSON y dispara flujos (alertas, logs, integraciones)

### Config (`atlas.env`)

```env
MAKEPLAY_ENABLED=true
MAKEPLAY_WEBHOOK_URL=https://hook.eu2.make.com/xxxxx
MAKEPLAY_SCAN_INTERVAL_SECONDS=60
```

### Payload webhook eventos ANS

```json
{"source":"atlas","stream":"ans_live","event":{"ts":"...","phase":"check_end","check_id":"llm_health","ok":true}}
```

### Payload webhook scanner

```json
{"source":"atlas","stream":"scanner","snapshot":{"health_score":85,"incidents_open":2,"last_actions":[...],"ts":"..."}}
```

### API manual

- `POST /humanoid/makeplay/scan` → Ejecuta scanner y envía al webhook

---

## Ejecución

1. **Reiniciar API** para cargar cambios
2. **Dashboard:** Abrir "Consola en vivo" → "Iniciar stream"
3. **Run Now** o esperar ciclo automático (cada 30s)
4. Ver en tiempo real: checks, incidentes, heals, omisiones
