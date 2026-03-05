# ATLAS Supervisor Report — 20260304-095410

**Evento:** fs_event
**Workspace:** `C:\ATLAS_PUSH`
**Hora:** `2026-03-04T09:54:10`

## Archivos tocados (último evento)
- `C:\ATLAS_PUSH\rauli_core_app\analysis_options.yaml`
- `C:\ATLAS_PUSH\scripts\atlas_agent_audit_to_md.py`
- `C:\ATLAS_PUSH\scripts\atlas_runtime_preflight.py`
- `C:\ATLAS_PUSH\scripts\register_atlas_agent_e2e_task.ps1`
- `C:\ATLAS_PUSH\scripts\start_atlas_api.ps1`
- `C:\ATLAS_PUSH\scripts\start_evolution.ps1`
- `C:\ATLAS_PUSH\scripts\start_nexus.ps1`
- `C:\ATLAS_PUSH\tools\atlas_agent\README.md`
- `C:\ATLAS_PUSH\tools\atlas_agent\config.py`
- `C:\ATLAS_PUSH\tools\atlas_agent\policy.py`
- `C:\ATLAS_PUSH\tools\atlas_supervisor\auto_fix.py`

## Git
- Branch: `deploy/strategy/update-20260225-20260225`
- Cambios pendientes: **30**

```
M .gitignore
 M _external/RAULI-VISION
 M _external/rauli-panaderia
 M _generated/smoke_fastapi_minimal/main.py
 M atlas_adapter/atlas_http_api.py
 M config/atlas.env
 M config/atlas.env.example
 M modules/humanoid/quality/autonomy_daemon.py
 M requirements.txt
 M scripts/restart_push_from_api.ps1
?? .codex/
?? .venv_nexus/
?? .venv_push/
?? .venv_robot/
?? ATLAS_SUPERVISOR.md
?? constraints/
?? docs/ATLAS_HTTP_API_MODULAR_PLAN.md
?? docs/SERVICE_VENVS.md
?? reports/
?? requirements.push.txt
?? scripts/atlas_agent_audit_to_md.py
?? scripts/atlas_agent_e2e_audit.py
?? scripts/atlas_agent_init.ps1
?? scripts/atlas_route_inventory.py
?? scripts/register_atlas_agent_e2e_task.ps1
?? scripts/windows/setup_service_venvs.ps1
?? tests/tools/
?? tools/atlas_agent/
?? tools/atlas_clawd_bridge/
?? tools/atlas_supervisor/
```

## Hallazgos rápidos
- Sin hallazgos críticos en chequeo rápido.

## Smoke tests
- Ejecutado: `True`
- Trigger: `fs_event`
- Resultado: **SKIP**

```
[SKIP] Evento 'fs_event' no incluido en SMOKE_ON_EVENTS.
```

## Recomendaciones inmediatas (Supervisor)
- Si hay errores repetidos, priorizar: imports/rutas -> scripts PS1 -> dependencias -> runtime/puertos.
- Mantener cambios pequeños y auditables (1 tema por commit).
