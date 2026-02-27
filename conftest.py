from __future__ import annotations

import os

import pytest


def _truthy_env(name: str) -> bool:
    return os.getenv(name, "").strip().lower() in ("1", "true", "yes", "on")


def pytest_collection_modifyitems(config: pytest.Config, items: list[pytest.Item]) -> None:
    """Por defecto, no ejecutar E2E en `pytest` local.

    Activación explícita:
    - set RUN_E2E=1
    """
    if _truthy_env("RUN_E2E"):
        return

    skip_e2e = pytest.mark.skip(reason="E2E deshabilitado por defecto (set RUN_E2E=1).")
    for item in items:
        path = str(getattr(item, "fspath", "")).replace("\\", "/")
        if "/tests/e2e/" in path:
            item.add_marker(skip_e2e)

