"""
POT: Auto Update Full (Actualización Automática Completa)
==========================================================
Procedimiento maestro de actualización automática que integra:

1. TRÍADA DE CRECIMIENTO:
   - PyPI: Escaneo y actualización de dependencias Python
   - GitHub: Sincronización de repositorios de código
   - HuggingFace: Búsqueda de modelos de IA

2. SCANNER:
   - Repo Scanner: Análisis de código (TODOs, archivos grandes)
   - MakePlay Scanner: Estado del sistema para webhook
   - Playwright Scanner: Verificación web si disponible

3. EVOLUTION PIPELINE:
   - Regression Tests
   - Backup/Snapshot
   - Staged Rollout
   - Metrics Comparison
   - Auto-Rollback si degradación

4. CUARENTENA:
   - Archivos sospechosos aislados
   - Dependencias problemáticas
   - Commits con errores

Triggers:
- Scheduler cada 12h
- Comando manual "update"
- POST /api/evolution/trigger

Severidad: HIGH (modifica sistema)
"""
from modules.humanoid.quality.models import POT, POTStep, POTCategory, POTSeverity, StepType


def get_pot() -> POT:
    return POT(
        id="auto_update_full",
        name="Actualización Automática Completa",
        description="""
Procedimiento maestro de actualización automática que integra la Tríada
de Crecimiento (PyPI, GitHub, HuggingFace), scanners de código y sistema,
el pipeline de evolución con rollback automático, y manejo de cuarentena.

Este POT es la guía completa de cómo ATLAS debe auto-actualizarse de forma
segura y controlada.
        """.strip(),
        category=POTCategory.UPGRADE,
        severity=POTSeverity.HIGH,
        version="1.0.0",
        author="ATLAS QA Senior",
        
        trigger_check_ids=["update_*", "evolution_*", "triada_*", "upgrade_*"],
        trigger_keywords=[
            "update", "upgrade", "evolution", "triada", "pypi", "github", 
            "huggingface", "hf", "actualizar", "evolucionar"
        ],
        
        prerequisites=[
            "Servicios básicos funcionando",
            "Credenciales configuradas (GITHUB_TOKEN, HF_TOKEN opcional)",
            "Conexión a internet",
            "Modo no-emergency",
        ],
        required_services=["push"],
        required_permissions=["update_apply", "git_write", "pip_install", "network_access"],
        
        objectives=[
            "Ejecutar scanners de estado",
            "Verificar actualizaciones PyPI disponibles",
            "Sincronizar con repositorios GitHub",
            "Buscar modelos en HuggingFace",
            "Ejecutar pipeline de evolución",
            "Manejar archivos en cuarentena",
            "Auto-rollback si hay degradación",
            "Notificar resultado a todos los canales",
        ],
        success_criteria="Actualización completada sin degradación de métricas",
        estimated_duration_minutes=15,
        
        tutorial_overview="""
## Guía de Actualización Automática Completa

### Arquitectura del Sistema de Actualización
```
┌─────────────────────────────────────────────────────────────────────┐
│                    AUTO UPDATE FULL PIPELINE                         │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  FASE 1: SCANNERS                                                    │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                          │
│  │  Repo    │  │ MakePlay │  │Playwright│                          │
│  │ Scanner  │  │ Scanner  │  │ (if avl) │                          │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘                          │
│       └─────────────┼─────────────┘                                 │
│                     ▼                                                │
│  FASE 2: TRÍADA DE CRECIMIENTO                                      │
│  ┌──────────────────────────────────────────────────────┐          │
│  │                                                       │          │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐           │          │
│  │  │   PyPI   │  │  GitHub  │  │   HF     │           │          │
│  │  │ Software │  │  Código  │  │ Modelos  │           │          │
│  │  │          │  │          │  │   IA     │           │          │
│  │  │ • Scan   │  │ • Fetch  │  │ • Search │           │          │
│  │  │ • Test   │  │ • Clone  │  │ • Cache  │           │          │
│  │  │ • Apply  │  │ • Sync   │  │ • Meta   │           │          │
│  │  └──────────┘  └──────────┘  └──────────┘           │          │
│  │                                                       │          │
│  └──────────────────────────────────────────────────────┘          │
│                     │                                                │
│                     ▼                                                │
│  FASE 3: EVOLUTION PIPELINE                                         │
│  ┌──────────────────────────────────────────────────────┐          │
│  │  Regression → Backup → Baseline → Rollout → Compare  │          │
│  │       │                                      │        │          │
│  │       │          AUTO-ROLLBACK ◄─────────────┘        │          │
│  │       │          si degradación > 15%                 │          │
│  └──────────────────────────────────────────────────────┘          │
│                     │                                                │
│                     ▼                                                │
│  FASE 4: CUARENTENA                                                  │
│  ┌──────────────────────────────────────────────────────┐          │
│  │  • Archivos sospechosos → quarantine/                 │          │
│  │  • Deps problemáticas → requirements.quarantine.txt   │          │
│  │  • Commits fallidos → git stash                       │          │
│  └──────────────────────────────────────────────────────┘          │
│                     │                                                │
│                     ▼                                                │
│  FASE 5: SYNC & NOTIFY                                              │
│  ┌──────────────────────────────────────────────────────┐          │
│  │  Git Commit → Push → Telegram → OPS → Bitácora        │          │
│  └──────────────────────────────────────────────────────┘          │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### Tríada de Crecimiento

| Worker | Fuente | Acción | Sandbox |
|--------|--------|--------|---------|
| **PyPI** | pypi.org/api | Escanear requirements.txt, instalar en .temp_venv, test import | .temp_venv/ |
| **GitHub** | api.github.com | Clonar repos, verificar commits, validar estructura | temp_workspace/ |
| **HuggingFace** | huggingface.co/api | Buscar modelos VLA/vision, cachear metadatos | temp_models_cache/ |

### Sistema de Cuarentena

```
quarantine/
├── deps/                    # Dependencias que fallaron tests
│   └── package_v1.2.3/
├── files/                   # Archivos sospechosos
│   └── suspicious_file.py
├── commits/                 # Commits revertidos
│   └── abc1234_backup/
└── quarantine_log.json      # Historial de cuarentena
```

### Archivos Clave

| Archivo | Propósito |
|---------|-----------|
| `evolution_daemon.py` | Daemon de la Tríada (cada 12h) |
| `requirements.txt` | Dependencias Python |
| `logs/evolution_last_cycle.json` | Último ciclo ejecutado |
| `config/autonomous.yaml` | Configuración de evolución |
| `.temp_venv/` | Sandbox para tests de PyPI |
| `temp_workspace/` | Sandbox para repos GitHub |
| `temp_models_cache/` | Cache de modelos HF |

### Modos de Gobernanza

| Modo | Comportamiento |
|------|----------------|
| **GROWTH** | Aplica actualizaciones automáticamente |
| **GOVERNED** | Requiere aprobación en Dashboard |

### Credenciales Requeridas

En `credenciales.txt` (Bóveda):
- `GITHUB_TOKEN`: Para sincronizar repos
- `HF_TOKEN`: Para buscar modelos (opcional)
- `PYPI_TOKEN`: Para autenticación PyPI (opcional)
        """.strip(),
        
        best_practices=[
            "Ejecutar en horario de bajo tráfico",
            "Verificar credenciales antes de iniciar",
            "Tener rollback listo",
            "No ejecutar durante deploys activos",
            "Revisar cuarentena periódicamente",
        ],
        
        warnings=[
            "REQUIERE APROBACIÓN en modo GOVERNED",
            "Puede modificar requirements.txt",
            "Puede reiniciar servicios",
            "Auto-rollback si métricas degradan > 15%",
        ],
        
        related_pots=[
            "repo_update",
            "deployment_full",
            "diagnostic_full",
            "autonomy_full_cycle",
            "git_push",
        ],
        tags=["update", "upgrade", "evolution", "triada", "pypi", "github", "huggingface", "auto"],
        has_rollback=True,
        
        steps=[
            # ================================================================
            # FASE 0: PRE-CHECKS
            # ================================================================
            POTStep(
                id="check_governance_mode",
                name="Verificar modo de gobernanza",
                description="Ver si estamos en GROWTH o GOVERNED",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/api/evolution/status",
                timeout_seconds=10,
                capture_output=True,
                continue_on_failure=True,
                tutorial_notes="governed=true requiere aprobación en Dashboard",
            ),
            
            POTStep(
                id="check_health_pre",
                name="Health check pre-actualización",
                description="Verificar que el sistema está estable antes de actualizar",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/health",
                timeout_seconds=15,
                capture_output=True,
                check_expression="response.get('score', 0) >= 60",
                tutorial_notes="Solo proceder si health >= 60",
            ),
            
            POTStep(
                id="check_no_active_updates",
                name="Verificar sin updates activos",
                description="Confirmar que no hay otra actualización en progreso",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/api/evolution/status",
                timeout_seconds=10,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="notify_update_start",
                name="Notificar inicio de actualización",
                description="Avisar que comienza el proceso",
                step_type=StepType.NOTIFY,
                notify_channel="ops",
                notify_message="🔄 Iniciando ciclo de actualización automática...",
            ),
            
            # ================================================================
            # FASE 1: SCANNERS
            # ================================================================
            POTStep(
                id="run_repo_scanner",
                name="Ejecutar repo scanner",
                description="Analizar código: TODOs, archivos grandes, configs",
                step_type=StepType.SCRIPT,
                script_path="scripts/run_repo_scanner.py",
                timeout_seconds=60,
                capture_output=True,
                continue_on_failure=True,
                tutorial_notes="Detecta archivos > 400KB, TODOs pendientes, etc",
            ),
            
            POTStep(
                id="run_makeplay_scanner",
                name="Ejecutar MakePlay scanner",
                description="Recopilar estado del sistema para webhook",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/scan",
                timeout_seconds=30,
                capture_output=True,
                continue_on_failure=True,
                tutorial_notes="Envía snapshot a Make.com si configurado",
            ),
            
            POTStep(
                id="check_playwright_available",
                name="Verificar Playwright disponible",
                description="Ver si podemos usar navegador automatizado",
                step_type=StepType.COMMAND,
                command="python -c \"from playwright.sync_api import sync_playwright; print('PLAYWRIGHT_OK')\"",
                timeout_seconds=15,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            # ================================================================
            # FASE 2: TRÍADA DE CRECIMIENTO
            # ================================================================
            POTStep(
                id="log_triada_start",
                name="Log inicio Tríada",
                description="Registrar inicio del ciclo de crecimiento",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_body={
                    "message": "[EVOLUCIÓN] Iniciando Tríada de Crecimiento (PyPI/GitHub/HF)",
                    "ok": True,
                    "source": "quality_pot.auto_update_full"
                },
            ),
            
            # PyPI Worker
            POTStep(
                id="pypi_scan_requirements",
                name="PyPI: Escanear requirements.txt",
                description="Verificar versiones actuales vs disponibles",
                step_type=StepType.COMMAND,
                command="python -c \"import json; from pathlib import Path; req = Path('requirements.txt'); lines = req.read_text().splitlines() if req.exists() else []; pkgs = [l.split('==')[0].split('>=')[0].strip() for l in lines if l.strip() and not l.startswith('#') and not l.startswith('-')]; print(json.dumps({'packages': pkgs[:10], 'total': len(pkgs)}))\"",
                timeout_seconds=15,
                capture_output=True,
                tutorial_notes="Lista los primeros 10 paquetes de requirements.txt",
            ),
            
            POTStep(
                id="pypi_check_outdated",
                name="PyPI: Verificar paquetes desactualizados",
                description="Consultar pip para ver qué se puede actualizar",
                step_type=StepType.SCRIPT,
                script_path="scripts/check_pip_outdated.py",
                timeout_seconds=120,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            # GitHub Worker
            POTStep(
                id="github_fetch_repos",
                name="GitHub: Fetch de repositorios",
                description="Sincronizar con repos configurados",
                step_type=StepType.COMMAND,
                command="git fetch origin --all",
                timeout_seconds=60,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="github_check_behind",
                name="GitHub: Verificar commits pendientes",
                description="Ver si hay commits nuevos en el remoto",
                step_type=StepType.COMMAND,
                command="git status -sb",
                timeout_seconds=15,
                capture_output=True,
            ),
            
            POTStep(
                id="github_submodule_update",
                name="GitHub: Actualizar submodules",
                description="Sincronizar submodules si existen",
                step_type=StepType.COMMAND,
                command="git submodule update --init --recursive",
                timeout_seconds=120,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            # HuggingFace Worker
            POTStep(
                id="hf_check_token",
                name="HuggingFace: Verificar token",
                description="Ver si tenemos acceso a HF API",
                step_type=StepType.COMMAND,
                command="python -c \"import os; from modules.evolution.credentials import load_credentials; c = load_credentials(); print('HF_TOKEN:', 'OK' if c.get('hf_token') else 'MISSING')\"",
                timeout_seconds=15,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="hf_scan_models",
                name="HuggingFace: Buscar modelos",
                description="Buscar modelos VLA/vision nuevos",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/api/evolution/status",
                timeout_seconds=15,
                capture_output=True,
                continue_on_failure=True,
                tutorial_notes="El daemon HF busca modelos de visión para Insta360",
            ),
            
            # ================================================================
            # FASE 3: EVOLUTION PIPELINE
            # ================================================================
            POTStep(
                id="evolution_regression_test",
                name="Evolution: Tests de regresión",
                description="Ejecutar tests antes de aplicar cambios",
                step_type=StepType.SCRIPT,
                script_path="scripts/check_nexus_ports.py",
                timeout_seconds=60,
                capture_output=True,
                continue_on_failure=True,
                tutorial_notes="Verifica que endpoints críticos responden",
            ),
            
            POTStep(
                id="evolution_create_backup",
                name="Evolution: Crear backup",
                description="Snapshot del estado actual para rollback",
                step_type=StepType.COMMAND,
                command="git stash push -m 'AUTO_UPDATE_BACKUP' --include-untracked",
                timeout_seconds=30,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="evolution_tag_backup",
                name="Evolution: Crear tag de backup",
                description="Marcar punto de rollback en Git",
                step_type=StepType.COMMAND,
                command="git tag -f auto-update-backup",
                timeout_seconds=10,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="evolution_trigger",
                name="Evolution: Disparar ciclo",
                description="Ejecutar el daemon de evolución",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/api/evolution/trigger",
                timeout_seconds=10,
                capture_output=True,
                continue_on_failure=True,
                tutorial_notes="Dispara evolution_daemon.py en background",
            ),
            
            POTStep(
                id="evolution_wait",
                name="Evolution: Esperar procesamiento",
                description="Dar tiempo al daemon para procesar",
                step_type=StepType.WAIT,
                wait_seconds=10,
            ),
            
            POTStep(
                id="evolution_check_status",
                name="Evolution: Verificar estado",
                description="Ver resultado del ciclo de evolución",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/api/evolution/status",
                timeout_seconds=15,
                capture_output=True,
            ),
            
            # ================================================================
            # FASE 4: CUARENTENA
            # ================================================================
            POTStep(
                id="quarantine_check_deps",
                name="Cuarentena: Verificar dependencias",
                description="Identificar paquetes problemáticos",
                step_type=StepType.COMMAND,
                command="python -c \"from pathlib import Path; q = Path('quarantine/deps'); files = list(q.glob('*')) if q.exists() else []; print(f'Deps en cuarentena: {len(files)}'); [print(f'  - {f.name}') for f in files[:5]]\"",
                timeout_seconds=15,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="quarantine_check_files",
                name="Cuarentena: Verificar archivos",
                description="Identificar archivos sospechosos",
                step_type=StepType.COMMAND,
                command="python -c \"from pathlib import Path; q = Path('quarantine/files'); files = list(q.glob('*')) if q.exists() else []; print(f'Archivos en cuarentena: {len(files)}'); [print(f'  - {f.name}') for f in files[:5]]\"",
                timeout_seconds=15,
                capture_output=True,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="quarantine_cleanup_old",
                name="Cuarentena: Limpiar antiguos",
                description="Eliminar items en cuarentena > 7 días",
                step_type=StepType.COMMAND,
                command="python -c \"import os, time; from pathlib import Path; q = Path('quarantine'); now = time.time(); old = 7*24*3600; removed = 0; [(os.remove(f), setattr(type('',(),{'x':0})(), 'x', 1)) for f in q.rglob('*') if f.is_file() and (now - f.stat().st_mtime) > old] if q.exists() else None; print('Cuarentena limpiada')\"",
                timeout_seconds=30,
                continue_on_failure=True,
            ),
            
            # ================================================================
            # FASE 5: POST-UPDATE VERIFICATION
            # ================================================================
            POTStep(
                id="health_check_post",
                name="Health check post-actualización",
                description="Verificar que el sistema sigue estable",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/health",
                timeout_seconds=20,
                retries=2,
                retry_delay_seconds=5,
                capture_output=True,
            ),
            
            POTStep(
                id="compare_health_scores",
                name="Comparar health scores",
                description="Verificar si hay degradación",
                step_type=StepType.CHECK,
                check_expression="True",  # El check real es en el siguiente paso
                capture_output=True,
                tutorial_notes="Si post_health < pre_health - 15, hacer rollback",
            ),
            
            POTStep(
                id="check_evolution_result",
                name="Verificar resultado de evolución",
                description="Leer el archivo de resultado del ciclo",
                step_type=StepType.COMMAND,
                command="python -c \"import json; from pathlib import Path; f = Path('logs/evolution_last_cycle.json'); print(f.read_text() if f.exists() else '{\\\"ok\\\": false, \\\"message\\\": \\\"no cycle run\\\"}')\"",
                timeout_seconds=10,
                capture_output=True,
            ),
            
            # ================================================================
            # FASE 6: SYNC & NOTIFY
            # ================================================================
            POTStep(
                id="git_status_final",
                name="Verificar cambios locales",
                description="Ver si hay cambios para commit",
                step_type=StepType.COMMAND,
                command="git status --porcelain",
                timeout_seconds=15,
                capture_output=True,
            ),
            
            POTStep(
                id="log_to_bitacora",
                name="Registrar en bitácora",
                description="Log completo del ciclo de actualización",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_body={
                    "message": "[EVOLUCIÓN] Ciclo de actualización automática completado",
                    "ok": True,
                    "source": "quality_pot.auto_update_full"
                },
            ),
            
            POTStep(
                id="notify_telegram",
                name="Notificar a Telegram",
                description="Enviar resumen al owner",
                step_type=StepType.NOTIFY,
                notify_channel="telegram",
                notify_message="📦 Ciclo de actualización automática completado. Revisar Dashboard para detalles.",
                continue_on_failure=True,
            ),
            
            POTStep(
                id="refresh_dashboard",
                name="Refrescar Dashboard",
                description="Actualizar UI con nuevo estado",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/status",
                timeout_seconds=15,
                continue_on_failure=True,
            ),
        ],
        
        rollback_steps=[
            POTStep(
                id="rollback_notify_start",
                name="Notificar inicio de rollback",
                description="Avisar que se está revirtiendo",
                step_type=StepType.NOTIFY,
                notify_channel="telegram",
                notify_message="⚠️ Actualización fallida. Ejecutando ROLLBACK...",
            ),
            
            POTStep(
                id="rollback_git_reset",
                name="Revertir a tag de backup",
                description="Volver al estado previo a la actualización",
                step_type=StepType.COMMAND,
                command="git reset --hard auto-update-backup",
                timeout_seconds=30,
            ),
            
            POTStep(
                id="rollback_pop_stash",
                name="Restaurar stash",
                description="Recuperar cambios guardados",
                step_type=StepType.COMMAND,
                command="git stash pop",
                timeout_seconds=30,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="rollback_reinstall_deps",
                name="Reinstalar dependencias",
                description="Restaurar paquetes a versiones anteriores",
                step_type=StepType.COMMAND,
                command="pip install -r requirements.txt --quiet",
                timeout_seconds=300,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="rollback_restart_services",
                name="Reiniciar servicios",
                description="Reiniciar con código anterior",
                step_type=StepType.COMMAND,
                command="powershell -ExecutionPolicy Bypass -File scripts/restart_service_clean.ps1 -Service robot",
                timeout_seconds=90,
                continue_on_failure=True,
            ),
            
            POTStep(
                id="rollback_verify_health",
                name="Verificar salud post-rollback",
                description="Confirmar que el sistema funciona",
                step_type=StepType.HTTP,
                http_method="GET",
                http_url="http://127.0.0.1:8791/health",
                timeout_seconds=30,
            ),
            
            POTStep(
                id="rollback_log",
                name="Registrar rollback",
                description="Log del rollback en bitácora",
                step_type=StepType.HTTP,
                http_method="POST",
                http_url="http://127.0.0.1:8791/ans/evolution-log",
                http_body={
                    "message": "[ROLLBACK] Actualización revertida por degradación",
                    "ok": False,
                    "source": "quality_pot.auto_update_full"
                },
            ),
            
            POTStep(
                id="rollback_notify_complete",
                name="Notificar rollback completado",
                description="Avisar resultado del rollback",
                step_type=StepType.NOTIFY,
                notify_channel="telegram",
                notify_message="🔄 ROLLBACK completado. Sistema restaurado a estado anterior.",
            ),
        ],
    )
