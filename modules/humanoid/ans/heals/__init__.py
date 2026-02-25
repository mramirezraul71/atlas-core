"""ANS heals: register all heals."""
from __future__ import annotations

from modules.humanoid.ans.registry import register_heal

from . import (clear_stale_locks, fallback_models, install_optional_deps,
               install_tesseract, mark_node_offline, regenerate_support_bundle,
               restart_nexus_services, restart_scheduler,
               retry_gateway_bootstrap, rotate_logs, tune_router)


def _register_all() -> None:
    register_heal("clear_stale_locks", clear_stale_locks.run)
    register_heal("restart_scheduler", restart_scheduler.run)
    register_heal("fallback_models", fallback_models.run)
    register_heal("tune_router", tune_router.run)
    register_heal("rotate_logs", rotate_logs.run)
    register_heal("retry_gateway_bootstrap", retry_gateway_bootstrap.run)
    register_heal("mark_node_offline", mark_node_offline.run)
    register_heal("regenerate_support_bundle", regenerate_support_bundle.run)
    register_heal("install_optional_deps", install_optional_deps.run)
    register_heal("install_tesseract", install_tesseract.run)
    register_heal("restart_nexus_services", restart_nexus_services.run)


_register_all()
