# ATLAS Supervisor Report — 20260304-094624

**Evento:** startup
**Workspace:** `C:\ATLAS_PUSH`
**Hora:** `2026-03-04T09:46:24`

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
- Trigger: `startup`
- Resultado: **NO/FAIL**

```
== Smoke tests ==
/status FAIL: La operaciÃ³n sobrepasÃ³ el tiempo de espera.
GET /version OK (0.0.0 canary)
GET /health OK (score= ms=)
GET /support/selfcheck OK
GET /deploy/status OK (mode=)
GET /canary/status OK
GET /cluster/status OK (enabled=)
POST /cluster/node/register OK or policy-denied (expected)
POST /cluster/heartbeat OK
GET /cluster/nodes OK
GET /gateway/status OK (enabled=)
POST /gateway/check OK or policy-denied
POST /gateway/bootstrap OK
GET /cluster/status OK
/modules OK
{
    "ok":  true,
    "modules":  [
                    {
                        "name":  "vision",
                        "enabled":  false
                    },
                    {
                        "name":  "voice",
                        "enabled":  false
                    },
                    {
                        "name":  "agent_router",
                        "enabled":  false
                    },
                    {
                        "name":  "telegram",
                        "enabled":  true
                    }
                ]
}
GET /mode/capabilities OK (mode=)
GET /ai/status OK (ollama_available=True)
== SMOKE: /llm ==
LLM OK -> route=FAST model=llama3.2:3b ms=2649
== SMOKE: Humanoid ==
/humanoid/status OK
brain, hands, eyes, ears, autonomy, comms, update
/humanoid/update-check OK
/humanoid/plan OK (steps: 3)
== SMOKE: Metrics / Policy / Audit ==
GET /metrics OK
POST /policy/test OK (allow=True)
GET /audit/tail?n=5 OK (error=null)
== SMOKE: Scheduler / Watchdog ==
GET /scheduler/jobs OK
POST /update/check OK (plan only)
POST /scheduler/job/create OK (job_id=9211b841-b036-4f60-ae67-470f25ac87e1)
GET /scheduler/job/runs FAIL: La operaciÃ³n sobrepasÃ³ el tiempo de espera.
GET /watchdog/status OK
== SMOKE: Agent / Scaffold / Scripts / Deps / Web / Voice ==
POST /agent/goal (plan_only) OK in 1220.9809ms (<15s)
POST /scaffold/app (fastapi minimal) OK
POST /scripts/generate OK
GET /deps/check OK (missing_deps reported)
GET /web/status OK (enabled=False)
GET /voice/status OK
POST /memory/thread/create OK (thread_id=05b44ed0-caa4-48cf-acdc-45aa3d062273)
POST /memory/write OK
GET /memory/recall OK (memory write/recall)
GET /memory/export OK
GET /memory/thread/list OK
GET /memory/snapshot OK
GET /vision/status OK
POST /vision/analyze responds (ok or error for missing file)
GET /screen/status OK (enabled=)
POST /cursor/run OK but slow: 18572.8124ms
GET /cursor/status OK
POST /bench/run OK (level=quick)
POST /agent/improve (plan_only repo) OK in 63.7339ms (<15s)
GET /agent/improve/status OK
GET /ui 200 OK
GET /approvals/list OK
GET /ans/status OK (enabled= mode=)
POST /ans/run-now OK or policy-denied
GET /ans/report/latest OK
== SMOKE: GA (Governed Autonomy) ==
POST /ga/run (scope=runtime mode=plan_only) OK in 1077.3852ms (<15s)
GA report_path exists: C:\ATLAS_PUSH\snapshots\ga\GA_REPORT_20260304_144746.md
GET /ga/status OK (mode=plan_only pending=5)
GET /ga/report/latest OK
== SMOKE: Meta-Learning ==
GET /metalearn/status OK
POST /metalearn/run OK
GET /metalearn/report/latest OK
POST /owner/session/start OK (token present)
POST /owner/session/end OK
GET /owner/session/status OK
GET /approvals/chain/verify OK (valid=False)
GET /owner/emergency/status OK
POST /owner/emergency/set enable=false OK (emergency=False)
== SMOKE: Governance ==
GET /governance/status OK (mode=growth emergency=False)
GET /governance/rules OK
POST /governance/mode OK (session activa o no requerida)
GET /health y /status OK tras governance checks
GET /support/bundle FAIL: 
SMOKE FAIL
```

## Recomendaciones inmediatas (Supervisor)
- Si hay errores repetidos, priorizar: imports/rutas -> scripts PS1 -> dependencias -> runtime/puertos.
- Mantener cambios pequeños y auditables (1 tema por commit).
