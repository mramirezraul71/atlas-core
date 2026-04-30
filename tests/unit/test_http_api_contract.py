"""Contract tests for ``atlas_adapter.atlas_http_api``.

Objetivo: fijar por tests el contrato público del adaptador HTTP
después de la limpieza del Paso C, de forma hermética (sin uvicorn y
sin rutas Windows). Cubre los 9 invariantes listados en
``docs/atlas_push/PLAN_STEP_C.md`` §6.1:

    1. Import time no explota fuera de Windows.
    2. Inventario exacto de rutas.
    3. Ausencia de duplicados (method, path).
    4. Shape literal de /status.
    5. Shape literal de /tools.
    6. Shape literal de /modules.
    7. Shape de /execute (tool soportada / no soportada).
    8. Shape de /intent (happy path + empty; matiz B preservado).
    9. Higiene de imports (sin ``importlib`` ni literales ``C:\\ATLAS``).

Los tests reutilizan el patrón de aislamiento de
``tests/unit/test_intent_router.py``: sobreescriben ``ATLAS_*`` vía
``monkeypatch.setenv`` apuntando a ``tmp_path`` y recargan los módulos
afectados, de modo que ``command_router`` escribe en el filesystem
temporal y no en ``C:\\ATLAS``.
"""
from __future__ import annotations

import importlib
import inspect
import sys
from collections import Counter
from pathlib import Path

import pytest
from fastapi.testclient import TestClient


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def http_app(tmp_path, monkeypatch):
    """Devuelve la app FastAPI del adaptador con filesystem aislado.

    Fuerza ``ATLAS_*`` a ``tmp_path`` y recarga
    ``modules.command_router`` y ``atlas_adapter.atlas_http_api`` para
    que capturen las rutas de entorno en frío. Garantiza que los tests
    no dependan de ``C:\\ATLAS`` ni de ningún estado previo de Vault.
    """
    atlas_root = tmp_path / "ATLAS"
    vault_dir = atlas_root / "ATLAS_VAULT"
    notes_dir = vault_dir / "NOTES"
    logs_dir = atlas_root / "logs"
    snaps_dir = atlas_root / "snapshots"

    monkeypatch.setenv("ATLAS_ROOT", str(atlas_root))
    monkeypatch.setenv("ATLAS_VAULT_DIR", str(vault_dir))
    monkeypatch.setenv("ATLAS_NOTES_DIR", str(notes_dir))
    monkeypatch.setenv("ATLAS_LOGS_DIR", str(logs_dir))
    monkeypatch.setenv("ATLAS_SNAPS_DIR", str(snaps_dir))

    # Import en frío: tirar cualquier versión previa ya importada para
    # que command_router re-evalúe las env vars.
    for mod_name in (
        "atlas_adapter.atlas_http_api",
        "atlas_push.intents.intent_router",
        "atlas_push.intents",
        "atlas_push",
        "modules.command_router",
    ):
        sys.modules.pop(mod_name, None)

    import atlas_adapter.atlas_http_api as http_api  # noqa: WPS433

    return http_api


@pytest.fixture
def client(http_app):
    return TestClient(http_app.app)


# ---------------------------------------------------------------------------
# Invariante 1 — Import time no explota fuera de Windows
# ---------------------------------------------------------------------------

def test_import_works_without_windows_paths(http_app):
    """El módulo se importa en frío en un sandbox sin ``C:\\ATLAS``."""
    assert http_app.app.title == "ATLAS Adapter"
    assert http_app.app.version == "1.0.0"
    # Sanidad: ``handle`` quedó ligado al symbol real de command_router
    # (importado como paquete, no cargado vía importlib.util).
    from modules.command_router import handle as cr_handle
    assert http_app.handle is cr_handle


# ---------------------------------------------------------------------------
# Invariante 2 — Inventario exacto de rutas públicas
# ---------------------------------------------------------------------------

def _public_routes(app):
    """Pares (path, frozenset(methods)) para rutas que no son de docs."""
    out = []
    for r in app.routes:
        path = getattr(r, "path", "")
        methods = frozenset(getattr(r, "methods", set()) or set())
        if path.startswith("/openapi") or path in (
            "/docs",
            "/redoc",
            "/docs/oauth2-redirect",
        ):
            continue
        if not methods:
            continue
        out.append((path, methods))
    return out


def test_exact_public_route_set(http_app):
    routes = _public_routes(http_app.app)
    actual = {(p, m) for p, m in routes}
    expected = {
        ("/status", frozenset({"GET"})),
        ("/tools", frozenset({"GET"})),
        ("/modules", frozenset({"GET"})),
        ("/execute", frozenset({"POST"})),
        ("/intent", frozenset({"POST"})),
    }
    assert actual == expected, f"Rutas inesperadas: {actual ^ expected}"


# ---------------------------------------------------------------------------
# Invariante 3 — Sin duplicados (método, path)
# ---------------------------------------------------------------------------

def test_no_duplicate_handlers(http_app):
    counts = Counter(_public_routes(http_app.app))
    duplicates = {k: v for k, v in counts.items() if v > 1}
    assert duplicates == {}, (
        f"Rutas duplicadas detectadas (regresión de D-002): {duplicates}"
    )


# ---------------------------------------------------------------------------
# Invariante 4 — Shape literal de /status
# ---------------------------------------------------------------------------

def test_status_shape(client):
    r = client.get("/status")
    assert r.status_code == 200
    body = r.json()
    assert set(body.keys()) == {"ok", "atlas"}
    assert body["ok"] is True
    assert isinstance(body["atlas"], str)
    # ``command_router.status()`` devuelve una cadena con prefijo
    # ``ATLAS: OK``; fijamos el prefijo como ancla de contrato.
    assert body["atlas"].startswith("ATLAS: OK")


# ---------------------------------------------------------------------------
# Invariante 5 — Shape literal de /tools
# ---------------------------------------------------------------------------

def test_tools_shape_literal(client):
    r = client.get("/tools")
    assert r.status_code == 200
    body = r.json()
    assert set(body.keys()) == {"ok", "tools"}
    assert body["ok"] is True
    assert body["tools"] == [
        "atlas.status",
        "atlas.doctor",
        "atlas.modules",
        "atlas.snapshot",
        "atlas.note.create",
        "atlas.note.append",
        "atlas.note.view",
        "atlas.inbox",
    ]


# ---------------------------------------------------------------------------
# Invariante 6 — Shape literal de /modules
# ---------------------------------------------------------------------------

def test_modules_shape_literal(client):
    r = client.get("/modules")
    assert r.status_code == 200
    body = r.json()
    assert set(body.keys()) == {"ok", "modules"}
    assert body["ok"] is True
    assert body["modules"] == [
        {"name": "vision", "enabled": False},
        {"name": "voice", "enabled": False},
        {"name": "agent_router", "enabled": False},
        {"name": "telegram", "enabled": True},
    ]


# ---------------------------------------------------------------------------
# Invariante 7 — Shape de /execute (soportada y no soportada)
# ---------------------------------------------------------------------------

def test_execute_supported_tool_shape(client):
    r = client.post("/execute", json={"tool": "atlas.status", "args": {}})
    assert r.status_code == 200
    body = r.json()
    assert set(body.keys()) == {"ok", "tool", "output"}
    assert body["ok"] is True
    assert body["tool"] == "atlas.status"
    assert isinstance(body["output"], str)
    # Misma ancla de contrato que /status.
    assert body["output"].startswith("ATLAS: OK")


def test_execute_unsupported_tool_shape(client):
    r = client.post("/execute", json={"tool": "bogus.not.a.tool", "args": {}})
    assert r.status_code == 200
    body = r.json()
    assert set(body.keys()) == {"ok", "error"}
    assert body["ok"] is False
    assert "Tool no soportada" in body["error"]
    assert "bogus.not.a.tool" in body["error"]


# ---------------------------------------------------------------------------
# Invariante 8 — Shape de /intent (happy path + empty, matiz B preservado)
# ---------------------------------------------------------------------------

def test_intent_happy_path_shape(client):
    r = client.post("/intent", json={"text": "/status"})
    assert r.status_code == 200
    body = r.json()
    assert set(body.keys()) == {"ok", "input", "output", "ms"}
    assert body["ok"] is True
    assert isinstance(body["output"], str)
    # Misma ancla que /status: /intent "/status" delega en el mismo
    # command_router.handle, así que debe arrancar igual.
    assert body["output"].startswith("ATLAS: OK")
    assert isinstance(body["ms"], int)
    assert body["ms"] >= 0
    # ``input`` refleja el payload (con defaults de IntentIn).
    assert body["input"]["text"] == "/status"
    assert body["input"]["user"] == "raul"
    assert body["input"]["meta"] is None


def test_intent_empty_preserves_b_nuance(client):
    """Matiz B preservado: ``text == ""`` → ``ATLAS: vacío.``.

    ``command_router.handle`` usa ``if not text`` (falsy strict), por
    eso la cadena vacía entra en la rama ``empty`` y devuelve la
    marca literal. Este test fija ese contrato a futuro.
    """
    r = client.post("/intent", json={"text": ""})
    assert r.status_code == 200
    body = r.json()
    assert set(body.keys()) == {"ok", "input", "output", "ms"}
    # Paridad estricta (opción i de PLAN_STEP_B.md): ok permanece True
    # salvo excepción; IntentResult.ok = False NO se propaga a /intent.
    assert body["ok"] is True
    assert body["output"] == "ATLAS: vacío."


# ---------------------------------------------------------------------------
# Invariante 9 — Higiene de imports (sin importlib ni C:\ATLAS)
# ---------------------------------------------------------------------------

def test_source_has_no_importlib_or_windows_paths(http_app):
    """Fija a futuro que el módulo no vuelve a caer en D-003.

    Chequea el fuente del módulo contra tres regresiones concretas:
    uso de ``importlib``, literales ``C:\\ATLAS`` y ``C:\\ATLAS_PUSH``.
    """
    source_path = Path(inspect.getsourcefile(http_app))
    src = source_path.read_text(encoding="utf-8")

    assert "importlib" not in src, (
        "Regresión de D-003: importlib vuelve a aparecer en "
        "atlas_http_api.py"
    )
    assert "C:\\ATLAS" not in src, (
        "Regresión de D-003: ruta Windows absoluta C:\\ATLAS en "
        "atlas_http_api.py"
    )
    assert "C:\\ATLAS_PUSH" not in src, (
        "Regresión de D-003: ruta Windows absoluta C:\\ATLAS_PUSH en "
        "atlas_http_api.py"
    )
