"""ANS heals: register all heals."""
from __future__ import annotations

from . import clear_stale_locks, restart_scheduler, fallback_models, tune_router
from . import rotate_logs, retry_gateway_bootstrap, mark_node_offline, regenerate_support_bundle
from . import install_optional_deps, install_tesseract
from modules.humanoid.ans.registry import register_heal


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


_register_all()
