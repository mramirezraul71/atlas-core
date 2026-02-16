# ATLAS Central Workshop (Taller Central de Reparaciones)

## Descripción

El **Workshop Central** es el sistema de autoreparación y mantenimiento preventivo de ATLAS. Funciona como un "taller mecánico" interno que:

1. **Recibe incidentes** del Sistema Nervioso Autónomo (ANS)
2. **Los mueve por bandejas** de trabajo: `inbox → working → resolved | failed`
3. **Aplica runbooks** según el tipo de problema detectado
4. **Verifica** la salud del sistema después de cada reparación
5. **Genera reportes** con evidencia de cada acción

## Arquitectura

```
┌─────────────────────────────────────────────────────────────────────┐
│                         ATLAS Dashboard                             │
│                    (atlas_adapter, port 8791)                       │
└──────────────────────────────┬──────────────────────────────────────┘
                               │
         ┌─────────────────────┼─────────────────────┐
         │                     │                     │
         ▼                     ▼                     ▼
┌────────────────┐   ┌────────────────┐   ┌────────────────┐
│      ANS       │   │   Scheduler    │   │   Approvals    │
│  (Detección)   │   │   (Ejecución)  │   │  (Autorización)│
│                │   │                │   │                │
│ • checks       │   │ • workshop_    │   │ • Aprobación   │
│ • incidents    │──▶│   cycle job    │◀──│   para heavy   │
│ • heals        │   │ • Intervalo    │   │ • Telegram     │
└────────────────┘   │   configurable │   └────────────────┘
                     └───────┬────────┘
                             │
                             ▼
              ┌──────────────────────────┐
              │    Workshop Central      │
              │  (atlas_central_workshop │
              │         .py)             │
              │                          │
              │  • Runbooks por módulo   │
              │  • Verificación health   │
              │  • Reportes JSON/MD      │
              └──────────────────────────┘
```

## Directorios de Trabajo

```
logs/workshop/
├── inbox/          # Tickets nuevos (incidentes ingresados)
├── working/        # Tickets en proceso de reparación
├── resolved/       # Tickets resueltos exitosamente
├── failed/         # Tickets que fallaron reparación
├── maintenance/    # Logs de ciclos de mantenimiento
├── reports/        # Reportes JSON y Markdown
└── approval_state.json  # Estado de aprobaciones consumidas
```

## Modos de Operación

| Modo | Descripción | Requiere Aprobación |
|------|-------------|---------------------|
| `incidents` | Solo procesa incidentes del ANS | No (seguro) |
| `maintenance` | Solo ejecuta tareas de mantenimiento | Sí (heavy) |
| `full` | Incidentes + Mantenimiento | Sí (heavy) |

## Runbooks Disponibles

| Runbook ID | Trigger (check_id/message) | Acciones |
|------------|---------------------------|----------|
| `camera_repair` | camera | Ejecuta `atlas_camera_autorepair.py` |
| `services_repair` | nexus, robot, gateway | Reinicia servicios Robot y Push |
| `dependency_repair` | deps, llm | Ejecuta ANS + repo_hygiene |
| `disk_maintenance` | disk, storage | Limpia cache con `-ClearCache` |
| `api_repair` | api, health | Reinicia servicio Push |
| `generic_repair` | (fallback) | Ejecuta ANS run-now |

## API Endpoints

### GET `/ans/workshop/status`
Estado general del Workshop: conteos por bandeja, último reporte.

```json
{
  "ok": true,
  "inbox_count": 2,
  "working_count": 0,
  "resolved_count": 15,
  "failed_count": 1,
  "reports_count": 8,
  "last_report": { "mode": "full", "overall_ok": true, ... }
}
```

### GET `/ans/workshop/tickets?tray=inbox&limit=50`
Lista tickets en una bandeja específica.

### POST `/ans/workshop/run-now`
Ejecuta el Workshop manualmente.

```json
{
  "mode": "incidents",
  "limit": 50,
  "require_approval_heavy": true
}
```

### GET `/ans/workshop/reports?limit=20`
Lista los últimos reportes generados.

## Scheduler Integration

El Workshop se ejecuta automáticamente mediante el job `workshop_cycle` en el scheduler.

### Variables de Entorno

| Variable | Default | Descripción |
|----------|---------|-------------|
| `WORKSHOP_ENABLED` | `true` | Habilita/deshabilita el job |
| `WORKSHOP_INTERVAL_SECONDS` | `1800` | Intervalo entre ciclos (30 min) |
| `WORKSHOP_DEFAULT_MODE` | `incidents` | Modo por defecto |
| `WORKSHOP_REQUIRE_APPROVAL_HEAVY` | `true` | Si heavy requiere aprobación |
| `WORKSHOP_APPROVAL_COOLDOWN_SECONDS` | `900` | Cooldown entre solicitudes |
| `WORKSHOP_INCIDENT_LIMIT` | `50` | Máx incidentes por ciclo |

### Flujo de Aprobación (Heavy)

1. El scheduler intenta ejecutar `workshop_cycle` con `mode=full` o `mode=maintenance`
2. Si `require_approval_heavy=true`:
   - Busca aprobación existente con `domain=workshop, operation=maintenance`
   - Si hay aprobación **approved** no consumida → la consume y ejecuta
   - Si hay aprobación **pending** → espera (no ejecuta, no crea nueva)
   - Si no hay ninguna → crea nueva solicitud pending (con cooldown)
3. Las aprobaciones llegan a Telegram con botones inline Aprobar/Rechazar
4. Una vez aprobada, el siguiente ciclo la consume y ejecuta

## Uso Manual (CLI)

```powershell
# Solo incidentes (seguro)
python scripts/atlas_central_workshop.py --mode incidents

# Mantenimiento completo (requiere aprobación si está en scheduler)
python scripts/atlas_central_workshop.py --mode full --require-approval-heavy

# Con límite de incidentes
python scripts/atlas_central_workshop.py --mode incidents --limit 30

# URLs personalizadas
python scripts/atlas_central_workshop.py --push-base http://localhost:8791 --robot-base http://localhost:8002
```

## Reportes

Cada ciclo genera dos archivos:

1. **JSON** (`workshop_report_YYYYMMDD_HHMMSS.json`): Datos estructurados completos
2. **Markdown** (`workshop_report_YYYYMMDD_HHMMSS.md`): Resumen legible

### Estructura del Reporte JSON

```json
{
  "started_at": "2026-02-15T10:30:00+00:00",
  "ended_at": "2026-02-15T10:32:15+00:00",
  "mode": "full",
  "overall_ok": true,
  "incident_cycle": {
    "ingested": 3,
    "processed_count": 3,
    "processed": [
      {
        "ticket": "20260215_103012_inc123_camera.json",
        "runbook": "camera_repair",
        "ok_final": true,
        "steps": [...]
      }
    ]
  },
  "maintenance": {
    "ok": true,
    "tasks": [...]
  },
  "approval": {
    "granted": true,
    "reason": "approved",
    "approval_id": "apr_abc123"
  }
}
```

## Seguridad

- **Operaciones Heavy** (maintenance/full) requieren aprobación del owner
- Las aprobaciones se envían a Telegram con botones inline
- Cooldown de 15 minutos entre solicitudes de aprobación
- Las aprobaciones tienen TTL y expiran automáticamente
- Cada aprobación solo puede consumirse una vez

## Troubleshooting

### El Workshop no ejecuta
1. Verificar `WORKSHOP_ENABLED=true`
2. Verificar que el scheduler esté activo
3. Revisar logs en `logs/workshop/`

### Esperando aprobación infinitamente
1. Verificar Telegram (debe haber llegado la solicitud)
2. Aprobar/Rechazar desde el dashboard o Telegram
3. Si expiró, rechazar y dejar que cree una nueva

### Tickets se quedan en `working`
1. El proceso pudo haber crasheado
2. Mover manualmente a `inbox` para reprocesar
3. Revisar `history` dentro del ticket JSON

## Evolución Futura

- [ ] Runbooks con LLM para diagnóstico avanzado
- [ ] Integración con sistema de embeddings para selección de runbook
- [ ] Dashboard visual con drag-and-drop entre bandejas
- [ ] Webhooks para notificaciones externas
- [ ] Métricas y alertas de tiempo de resolución
