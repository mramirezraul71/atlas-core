"""FastAPI lifespan hooks extracted from the main PUSH API module."""
from __future__ import annotations

import asyncio
import logging
import os
import sys
import time
from contextlib import asynccontextmanager
from importlib import import_module

from atlas_adapter.bootstrap.env import load_vault_env
from atlas_adapter.bootstrap.settings import BASE_DIR


def _schedule_autonomy_daemon_start() -> None:
    """Reuse the current API helper until autonomy bootstrap is extracted too."""
    try:
        api_module = import_module("atlas_adapter.atlas_http_api")
        start_fn = getattr(api_module, "autonomy_daemon_start", None)
        if callable(start_fn):
            asyncio.create_task(asyncio.to_thread(start_fn))
    except Exception:
        pass


def _schedule_autonomy_manager_start() -> None:
    """Bootstrap del Autonomy Manager sin bloquear el arranque de PUSH."""
    try:
        from modules.humanoid.autonomy_manager.daemon import get_autonomy_manager_daemon

        get_autonomy_manager_daemon().start()
    except Exception:
        pass


def _schedule_background_startup(
    name: str, fn, *, logger_name: str | None = None, success_message: str | None = None
) -> None:
    log = logging.getLogger(logger_name or __name__)

    async def _runner():
        try:
            result = await asyncio.to_thread(fn)
            if success_message:
                if isinstance(result, dict) and not result.get("ok", True):
                    log.warning("%s: %s", name, result)
                else:
                    log.info("%s", success_message)
        except Exception as exc:
            log.error("%s failed: %s", name, exc)

    asyncio.create_task(_runner())


@asynccontextmanager
async def app_lifespan(app):
    try:
        from modules.humanoid.deploy.healthcheck import _set_app_start_time

        _set_app_start_time()
    except Exception:
        pass
    load_vault_env()
    try:
        from modules.humanoid.owner.emergency import set_emergency_from_env

        set_emergency_from_env()
    except Exception:
        pass
    humanoid = os.getenv("HUMANOID_ENABLED", "true").strip().lower() in (
        "1",
        "true",
        "yes",
        "y",
        "on",
    )
    sched = os.getenv("SCHED_ENABLED", "true").strip().lower() in (
        "1",
        "true",
        "yes",
        "y",
        "on",
    )
    worker_only = os.getenv("WORKER_ONLY", "false").strip().lower() in (
        "1",
        "true",
        "yes",
    )
    safe_startup = os.getenv("ATLAS_SAFE_STARTUP", "false").strip().lower() in (
        "1",
        "true",
        "yes",
        "y",
        "on",
    )
    minimal_startup = os.getenv("ATLAS_MINIMAL_STARTUP", "false").strip().lower() in (
        "1",
        "true",
        "yes",
        "y",
        "on",
    )
    if minimal_startup:
        logging.getLogger(__name__).warning(
            "ATLAS minimal startup enabled; background subsystems skipped."
        )
        yield
        return
    if humanoid and sched:
        from concurrent.futures import ThreadPoolExecutor

        from modules.humanoid.scheduler import start_scheduler
        from modules.humanoid.watchdog import start_watchdog

        _executor = ThreadPoolExecutor(max_workers=2)
        start_scheduler(executor=_executor)
        start_watchdog()
        try:
            if os.getenv("VISION_UBIQ_ENABLED", "true").strip().lower() in (
                "1",
                "true",
                "yes",
                "y",
                "on",
            ):
                from modules.humanoid.vision.ubiq import ensure_db, start_motion_watchdog

                ensure_db()
                start_motion_watchdog()
        except Exception:
            pass
        if not worker_only:
            try:
                from modules.humanoid.ci import ensure_ci_jobs

                ensure_ci_jobs()
            except Exception:
                pass
            try:
                from modules.humanoid.ga.scheduler_jobs import ensure_ga_jobs

                ensure_ga_jobs()
            except Exception:
                pass
            try:
                from modules.humanoid.metalearn.scheduler_jobs import ensure_metalearn_jobs

                ensure_metalearn_jobs()
            except Exception:
                pass
            try:
                from modules.humanoid.ans.scheduler_jobs import ensure_ans_jobs

                ensure_ans_jobs()
                if (
                    (not safe_startup)
                    and os.getenv("ANS_ENABLED", "true").strip().lower()
                    in ("1", "true", "yes")
                    and os.getenv("ANS_RUN_AT_STARTUP", "true").strip().lower()
                    in ("1", "true", "yes")
                ):

                    def _run_triada_at_startup():
                        try:
                            from modules.humanoid.ans.engine import run_ans_cycle

                            run_ans_cycle(
                                mode=os.getenv("ANS_MODE", "auto"),
                                timeout_sec=60,
                            )
                        except Exception:
                            pass

                    import threading

                    thread = threading.Thread(
                        target=_run_triada_at_startup, daemon=True
                    )
                    thread.start()
            except Exception:
                pass
            try:
                from modules.humanoid.nervous.scheduler_jobs import ensure_nervous_jobs

                ensure_nervous_jobs()
            except Exception:
                pass
            try:
                from modules.humanoid.vision.world_state_jobs import ensure_world_state_jobs

                ensure_world_state_jobs()
            except Exception:
                pass
            try:
                if not safe_startup:
                    from modules.humanoid.comms.bootstrap import bootstrap_comms

                    def _bootstrap_comms_bg():
                        comms_result = bootstrap_comms(skip_tests=True)
                        if not comms_result.get("ok"):
                            comms_logger = logging.getLogger("atlas.comms.startup")
                            for warning in comms_result.get("warnings", []):
                                comms_logger.warning("Comms bootstrap: %s", warning)
                        return comms_result

                    _schedule_background_startup(
                        "comms_bootstrap",
                        _bootstrap_comms_bg,
                        logger_name="atlas.comms.startup",
                        success_message="Comms bootstrap lanzado en background",
                    )
            except Exception as comms_err:
                logging.getLogger("atlas.comms.startup").error(
                    "Comms bootstrap failed: %s", comms_err
                )
            try:
                if (not safe_startup) and os.getenv(
                    "QUALITY_AUTONOMY_ENABLED", "true"
                ).strip().lower() in ("1", "true", "yes", "y", "on"):
                    from modules.humanoid.quality import start_autonomous_system

                    def _start_quality_bg():
                        quality_result = start_autonomous_system()
                        try:
                            if os.getenv(
                                "AUTONOMY_DAEMON_AUTOSTART", "true"
                            ).strip().lower() in ("1", "true", "yes", "y", "on"):
                                from modules.humanoid.quality.autonomy_daemon import (
                                    is_autonomy_running,
                                    start_autonomy,
                                )

                                if not is_autonomy_running():
                                    start_autonomy()
                        except Exception:
                            pass
                        try:
                            from modules.humanoid.ans.evolution_bitacora import (
                                append_evolution_log,
                            )

                            append_evolution_log(
                                message="[QUALITY] AutonomÃ­a POT iniciada (dispatcher+triggers).",
                                ok=bool(quality_result.get("all_ok", False)),
                                source="quality",
                            )
                        except Exception:
                            pass
                        return quality_result

                    _schedule_background_startup(
                        "quality_autonomy",
                        _start_quality_bg,
                        success_message="Quality autonomy lanzada en background",
                    )
            except Exception:
                pass
            try:
                from modules.humanoid.scheduler.repo_monitor_jobs import ensure_repo_monitor_jobs

                ensure_repo_monitor_jobs()
            except Exception:
                pass
            try:
                from modules.humanoid.comms.makeplay_scheduler import ensure_makeplay_jobs

                ensure_makeplay_jobs()
            except Exception:
                pass
            try:
                from modules.humanoid.scheduler.repo_hygiene_jobs import ensure_repo_hygiene_jobs

                ensure_repo_hygiene_jobs()
            except Exception:
                pass
            try:
                from modules.humanoid.approvals.scheduler_jobs import ensure_approvals_jobs

                ensure_approvals_jobs()
            except Exception:
                pass
            try:
                from modules.humanoid.scheduler.workshop_jobs import ensure_workshop_jobs

                ensure_workshop_jobs()
            except Exception:
                pass
            try:
                from modules.humanoid.repo.git_hooks import ensure_post_commit_hook

                ensure_post_commit_hook()
            except Exception:
                pass
        try:
            from modules.nexus_heartbeat import register_status_callback, start_heartbeat

            dashboard_base = (
                os.getenv("ATLAS_DASHBOARD_URL") or "http://127.0.0.1:8791"
            ).rstrip("/")

            def _on_nexus_change(connected: bool, message: str) -> None:
                if connected:
                    text = "[CONEXIÃ“N] Buscando NEXUS en puerto 8000... OK."
                else:
                    extra = (message or "").strip()
                    extra = (": " + extra[:120]) if extra else ""
                    text = (
                        "[CONEXIÃ“N] Buscando NEXUS en puerto 8000... Desconectado"
                        + extra
                    )
                try:
                    import json
                    import urllib.request

                    req = urllib.request.Request(
                        dashboard_base + "/ans/evolution-log",
                        data=json.dumps({"message": text, "ok": connected}).encode(
                            "utf-8"
                        ),
                        headers={"Content-Type": "application/json"},
                        method="POST",
                    )
                    urllib.request.urlopen(req, timeout=1)
                except Exception as e:
                    logging.getLogger("atlas.startup").warning(
                        "ANS no disponible en startup: %s. Continuando.", e
                    )

            register_status_callback(_on_nexus_change)
            start_heartbeat()

            def _run_nerve_test_background():
                try:
                    if str(BASE_DIR) not in sys.path:
                        sys.path.insert(0, str(BASE_DIR))
                    import nexus_actions

                    nexus_actions.run_nerve_test()
                except Exception:
                    pass

            import threading

            nerve_thread = threading.Thread(
                target=_run_nerve_test_background,
                daemon=True,
            )
            nerve_thread.start()
        except Exception:
            pass
    try:
        if str(BASE_DIR) not in sys.path:
            sys.path.insert(0, str(BASE_DIR))

        from autonomous.health_monitor.health_aggregator import HealthAggregator
        from autonomous.learning.learning_orchestrator import LearningOrchestrator
        from autonomous.telemetry.alert_manager import AlertManager
        from autonomous.telemetry.metrics_aggregator import MetricsAggregator

        log = logging.getLogger(__name__)
        log.info("Iniciando background tasks de ATLAS AUTONOMOUS...")
        health_agg = HealthAggregator()
        health_agg.start_monitoring()
        log.info("âœ“ Health Monitoring activo")
        alert_mgr = AlertManager()
        asyncio.create_task(alert_mgr.start_evaluation_loop(60))
        log.info("âœ“ Alert Manager activo")
        learning = LearningOrchestrator()

        async def _learning_loop():
            while True:
                await asyncio.sleep(3600)
                try:
                    learning.run_learning_cycle()
                    log.info("âœ“ Learning cycle ejecutado")
                except Exception as exc:
                    log.error("Error en learning cycle: %s", exc)

        asyncio.create_task(_learning_loop())
        log.info("âœ“ Learning Engine activo")

        metrics_agg = MetricsAggregator()

        def _load_telemetry_interval() -> float:
            interval = 60.0
            try:
                cfg_path = BASE_DIR / "config" / "autonomous.yaml"
                if cfg_path.exists():
                    import yaml

                    cfg = yaml.safe_load(
                        cfg_path.read_text(encoding="utf-8", errors="replace")
                    ) or {}
                    interval = float(
                        (((cfg.get("telemetry") or {}).get("metrics") or {}).get(
                            "aggregation_interval"
                        ))
                        or interval
                    )
            except Exception:
                pass
            return max(5.0, interval)

        telemetry_interval_sec = _load_telemetry_interval()

        def _probe_service(url: str) -> tuple[float, float]:
            import urllib.request

            t0_probe = time.perf_counter()
            try:
                req = urllib.request.Request(
                    url, method="GET", headers={"Accept": "application/json"}
                )
                with urllib.request.urlopen(req, timeout=2) as resp:
                    if 200 <= int(resp.status) < 400:
                        return (1.0, (time.perf_counter() - t0_probe) * 1000.0)
            except Exception:
                pass
            return (0.0, 0.0)

        def _collect_telemetry_once() -> None:
            try:
                import psutil

                disk_root = (os.getenv("SystemDrive") or "C:") + "\\"
                metrics_agg.collect_metric(
                    "system",
                    "cpu_percent",
                    float(psutil.cpu_percent(interval=0)),
                )
                metrics_agg.collect_metric(
                    "system",
                    "ram_percent",
                    float(psutil.virtual_memory().percent),
                )
                metrics_agg.collect_metric(
                    "system",
                    "disk_usage_percent",
                    float(psutil.disk_usage(disk_root).percent),
                )
            except Exception:
                pass
            try:
                from modules.observability.metrics import MetricsCollector

                summary = MetricsCollector.get_metrics_summary() or {}
                metrics_agg.collect_metric(
                    "observability",
                    "total_requests",
                    float(summary.get("total_requests", 0) or 0),
                )
                metrics_agg.collect_metric(
                    "observability",
                    "active_requests",
                    float(summary.get("active_requests", 0) or 0),
                )
                metrics_agg.collect_metric(
                    "observability",
                    "memory_mb",
                    float(summary.get("memory_mb", 0) or 0),
                )
            except Exception:
                pass
            try:
                services = {
                    "push": "http://127.0.0.1:8791/health",
                    "nexus": "http://127.0.0.1:8000/health",
                    "robot": "http://127.0.0.1:8002/api/health",
                }
                for service_name, service_url in services.items():
                    if service_name == "push":
                        metrics_agg.collect_metric("services", "push_online", 1.0)
                        continue
                    online, latency_ms = _probe_service(service_url)
                    metrics_agg.collect_metric(
                        "services",
                        f"{service_name}_online",
                        online,
                    )
                    if latency_ms > 0:
                        metrics_agg.collect_metric(
                            "services",
                            f"{service_name}_latency_ms",
                            float(latency_ms),
                        )
            except Exception:
                pass

        async def _telemetry_loop():
            while True:
                try:
                    await asyncio.to_thread(_collect_telemetry_once)
                except Exception:
                    pass
                await asyncio.sleep(telemetry_interval_sec)

        asyncio.create_task(_telemetry_loop())
        log.info("âœ“ Telemetry Metrics activo (intervalo %ss)", telemetry_interval_sec)
    except Exception as exc:
        logging.getLogger(__name__).debug(
            "ATLAS AUTONOMOUS background tasks no iniciados: %s", exc
        )
    try:
        from modules.observability.metrics import update_system_metrics

        async def _metrics_loop():
            while True:
                await asyncio.sleep(10)
                try:
                    update_system_metrics()
                except Exception:
                    pass

        asyncio.create_task(_metrics_loop())
    except Exception:
        pass
    try:
        if os.getenv("AUTONOMY_DAEMON_AUTOSTART", "true").strip().lower() in (
            "1",
            "true",
            "yes",
            "y",
            "on",
        ):
            from modules.humanoid.quality.autonomy_daemon import is_autonomy_running

            if not is_autonomy_running():
                _schedule_autonomy_daemon_start()
            if not is_autonomy_running():
                await asyncio.sleep(0.5)
                _schedule_autonomy_daemon_start()
    except Exception:
        pass
    try:
        if os.getenv("ATLAS_AUTONOMY_MANAGER_AUTOSTART", "true").strip().lower() in (
            "1",
            "true",
            "yes",
            "y",
            "on",
        ):
            await asyncio.to_thread(_schedule_autonomy_manager_start)
    except Exception:
        pass
    try:
        if not safe_startup:
            from atlas_adapter.supervisor_daemon import start_supervisor_daemon

            await start_supervisor_daemon()
    except Exception:
        pass
    # â”€â”€ Self-Healing Loop â€” autocorrecciÃ³n de todos los mÃ³dulos ATLAS â”€â”€â”€â”€â”€â”€â”€â”€â”€
    try:
        if os.getenv("ATLAS_SELF_HEALING_ENABLED", "true").strip().lower() in (
            "1", "true", "yes", "y", "on"
        ):
            import importlib.util as _ilu
            import threading as _threading

            _heal_script = BASE_DIR / "scripts" / "atlas_self_healing_loop.py"
            if _heal_script.exists():
                import sys as _sys

                _spec = _ilu.spec_from_file_location("atlas_self_healing_runtime", str(_heal_script))
                if _spec and _spec.loader:
                    _heal_mod = _ilu.module_from_spec(_spec)
                    _sys.modules[_spec.name] = _heal_mod
                    _spec.loader.exec_module(_heal_mod)
                    _heal_fn = getattr(_heal_mod, "run_healing_cycle", None)

                    def _self_heal_daemon():
                        import time as _time
                        _interval = int(os.getenv("ATLAS_HEAL_INTERVAL_SEC", "60"))
                        _heal_log = logging.getLogger("atlas.heal.daemon")
                        _heal_log.info("Self-Healing daemon iniciado (intervalo=%ss)", _interval)
                        while True:
                            try:
                                if callable(_heal_fn):
                                    _heal_fn()
                            except Exception as _e:
                                _heal_log.error("Error en ciclo self-healing: %s", _e)
                            _time.sleep(_interval)

                    async def _delayed_self_heal_start():
                        _startup_grace = max(
                            5,
                            int(
                                os.getenv(
                                    "ATLAS_SELF_HEALING_STARTUP_GRACE_SEC", "45"
                                )
                                or "45"
                            ),
                        )
                        await asyncio.sleep(_startup_grace)
                        _heal_thread = _threading.Thread(
                            target=_self_heal_daemon,
                            name="atlas-self-healing",
                            daemon=True,
                        )
                        _heal_thread.start()

                    asyncio.create_task(_delayed_self_heal_start())
                    logging.getLogger(__name__).info(
                        "âœ“ Self-Healing Loop programado con gracia de arranque"
                    )
    except Exception as _heal_err:
        logging.getLogger(__name__).warning("Self-Healing Loop no pudo iniciar: %s", _heal_err)
    # ATLAS DOCTOR eliminado
    yield
    try:
        from atlas_adapter.supervisor_daemon import stop_supervisor_daemon

        await stop_supervisor_daemon()
    except Exception:
        pass

