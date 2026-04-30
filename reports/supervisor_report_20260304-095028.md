# ATLAS Supervisor Report — 20260304-095028

**Evento:** fs_event
**Workspace:** `C:\ATLAS_PUSH`
**Hora:** `2026-03-04T09:50:28`

## Archivos tocados (último evento)
- `C:\ATLAS_PUSH\01_setup_venv.ps1`
- `C:\ATLAS_PUSH\02_install_deps.ps1`
- `C:\ATLAS_PUSH\03_run_atlas_api.ps1`
- `C:\ATLAS_PUSH\05_kill_port.ps1`
- `C:\ATLAS_PUSH\06_git_sanity.ps1`
- `C:\ATLAS_PUSH\07_install_all.ps1`
- `C:\ATLAS_PUSH\agents\coder_agent.py`
- `C:\ATLAS_PUSH\agents\inspector_agent.py`
- `C:\ATLAS_PUSH\config\repo_monitor.yaml`
- `C:\ATLAS_PUSH\modules\humanoid\cortex\__init__.py`
- `C:\ATLAS_PUSH\modules\humanoid\cortex\frontal\__init__.py`
- `C:\ATLAS_PUSH\modules\humanoid\cortex\frontal\decision_maker.py`
- `C:\ATLAS_PUSH\modules\humanoid\cortex\frontal\goal_context.py`
- `C:\ATLAS_PUSH\modules\humanoid\cortex\frontal\inhibitory_control.py`
- `C:\ATLAS_PUSH\modules\humanoid\cortex\frontal\parallel_executor.py`
- `C:\ATLAS_PUSH\modules\humanoid\cortex\frontal\resource_arbiter.py`
- `C:\ATLAS_PUSH\modules\humanoid\cortex\frontal\task_planner.py`
- `C:\ATLAS_PUSH\modules\humanoid\cortex\occipital\__init__.py`
- `C:\ATLAS_PUSH\modules\humanoid\cortex\occipital\depth_estimation.py`
- `C:\ATLAS_PUSH\modules\humanoid\cortex\occipital\object_recognition.py`
- `C:\ATLAS_PUSH\modules\humanoid\cortex\occipital\vision_pipeline.py`
- `C:\ATLAS_PUSH\modules\humanoid\cortex\parietal\__init__.py`
- `C:\ATLAS_PUSH\modules\humanoid\cortex\parietal\body_schema.py`
- `C:\ATLAS_PUSH\modules\humanoid\cortex\parietal\sensory_fusion.py`
- `C:\ATLAS_PUSH\modules\humanoid\cortex\parietal\spatial_map.py`
- `C:\ATLAS_PUSH\modules\humanoid\cortex\temporal\__init__.py`
- `C:\ATLAS_PUSH\modules\humanoid\cortex\temporal\audio_processor.py`
- `C:\ATLAS_PUSH\modules\humanoid\cortex\temporal\episodic_recall.py`
- `C:\ATLAS_PUSH\modules\humanoid\cortex\temporal\language_understanding.py`
- `C:\ATLAS_PUSH\modules\humanoid\cortex\unified_memory.py`
- `C:\ATLAS_PUSH\modules\humanoid\cursor\__init__.py`
- `C:\ATLAS_PUSH\modules\humanoid\cursor\executor.py`
- `C:\ATLAS_PUSH\modules\humanoid\cursor\run.py`
- `C:\ATLAS_PUSH\modules\humanoid\cursor\status.py`
- `C:\ATLAS_PUSH\modules\humanoid\deploy\__init__.py`
- `C:\ATLAS_PUSH\modules\humanoid\deploy\bluegreen.py`
- `C:\ATLAS_PUSH\modules\humanoid\deploy\canary.py`
- `C:\ATLAS_PUSH\modules\humanoid\deploy\healthcheck.py`
- `C:\ATLAS_PUSH\modules\humanoid\deploy\models.py`
- `C:\ATLAS_PUSH\modules\humanoid\deploy\ports.py`
- `C:\ATLAS_PUSH\modules\humanoid\deploy\process_manager.py`
- `C:\ATLAS_PUSH\modules\humanoid\deploy\status.py`
- `C:\ATLAS_PUSH\modules\humanoid\deploy\switcher.py`
- `C:\ATLAS_PUSH\modules\humanoid\deps_checker.py`
- `C:\ATLAS_PUSH\modules\humanoid\directives\__init__.py`
- `C:\ATLAS_PUSH\modules\humanoid\directives\api.py`
- `C:\ATLAS_PUSH\modules\humanoid\directives\global.md`
- `C:\ATLAS_PUSH\modules\humanoid\directives\manager.py`
- `C:\ATLAS_PUSH\modules\humanoid\directives\metadata.json`
- `C:\ATLAS_PUSH\modules\humanoid\dispatch\__init__.py`
- `C:\ATLAS_PUSH\modules\humanoid\dispatch\dispatcher.py`
- `C:\ATLAS_PUSH\modules\humanoid\dispatch\policies.py`
- `C:\ATLAS_PUSH\modules\humanoid\ears\__init__.py`
- `C:\ATLAS_PUSH\modules\humanoid\ears\stt_service.py`
- `C:\ATLAS_PUSH\modules\humanoid\ears\tts_service.py`
- `C:\ATLAS_PUSH\modules\humanoid\evolution_memory\__init__.py`
- `C:\ATLAS_PUSH\modules\humanoid\evolution_memory\db.py`
- `C:\ATLAS_PUSH\modules\humanoid\eyes\__init__.py`
- `C:\ATLAS_PUSH\modules\humanoid\eyes\vision_service.py`
- `C:\ATLAS_PUSH\modules\humanoid\face\__init__.py`
- `C:\ATLAS_PUSH\modules\humanoid\face\detector.py`
- `C:\ATLAS_PUSH\modules\humanoid\ga\__init__.py`
- `C:\ATLAS_PUSH\modules\humanoid\ga\api.py`
- `C:\ATLAS_PUSH\modules\humanoid\ga\approvals_bridge.py`
- `C:\ATLAS_PUSH\modules\humanoid\ga\cycle.py`
- `C:\ATLAS_PUSH\modules\humanoid\ga\detector.py`
- `C:\ATLAS_PUSH\modules\humanoid\ga\executor.py`
- `C:\ATLAS_PUSH\modules\humanoid\ga\models.py`
- `C:\ATLAS_PUSH\modules\humanoid\ga\planner.py`
- `C:\ATLAS_PUSH\modules\humanoid\ga\reporter.py`
- `C:\ATLAS_PUSH\modules\humanoid\ga\scheduler_jobs.py`
- `C:\ATLAS_PUSH\modules\humanoid\ga\scorer.py`
- `C:\ATLAS_PUSH\modules\humanoid\ga\signals.py`
- `C:\ATLAS_PUSH\modules\humanoid\gateway\__init__.py`
- `C:\ATLAS_PUSH\modules\humanoid\gateway\audit.py`
- `C:\ATLAS_PUSH\modules\humanoid\gateway\bootstrap.py`
- `C:\ATLAS_PUSH\modules\humanoid\gateway\cloudflare.py`
- `C:\ATLAS_PUSH\modules\humanoid\gateway\detector.py`
- `C:\ATLAS_PUSH\modules\humanoid\gateway\health.py`
- `C:\ATLAS_PUSH\modules\humanoid\gateway\lan.py`

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
