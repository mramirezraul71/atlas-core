"""Unit tests para `atlas_push.intents.IntentRouter`.

Invariantes fijados (ver docs/atlas_push/PLAN_STEP_B.md §8):

- Los 12 ``kind``s + ``"empty"`` se clasifican correctamente.
- ``IntentResult`` es inmutable (``frozen=True``).
- ``output`` de ``IntentRouter.handle`` es byte‑idéntico al de
  ``modules.command_router.handle`` para el mismo input.
- ``raw_input`` se preserva literal.
- ``meta == {}`` en todos los casos en B.
- Aislamiento de filesystem via ``tmp_path`` + ``monkeypatch.setenv``.
"""

from __future__ import annotations

import importlib
from dataclasses import FrozenInstanceError

import pytest


# ---------------------------------------------------------------------------
# Fixtures de aislamiento
# ---------------------------------------------------------------------------


@pytest.fixture
def isolated_atlas(tmp_path, monkeypatch):
    """Recarga ``modules.command_router`` con rutas ATLAS apuntando a tmp_path.

    Evita tocar rutas reales del sistema (``C:\\ATLAS``, ``/home/...``).
    Devuelve el módulo ``command_router`` recién reimportado, ya
    configurado contra ``tmp_path``.

    Uso::

        def test_something(isolated_atlas):
            cr = isolated_atlas
            cr.handle("/status")
    """
    root = tmp_path / "ATLAS"
    vault = root / "VAULT"
    notes = vault / "NOTES"
    logs = root / "logs"
    snaps = root / "snapshots"

    monkeypatch.setenv("ATLAS_ROOT", str(root))
    monkeypatch.setenv("ATLAS_VAULT_DIR", str(vault))
    monkeypatch.setenv("ATLAS_NOTES_DIR", str(notes))
    monkeypatch.setenv("ATLAS_LOGS_DIR", str(logs))
    monkeypatch.setenv("ATLAS_SNAPS_DIR", str(snaps))
    monkeypatch.setenv("ATLAS_LOG_FILE", str(logs / "atlas.log"))

    # Recargar command_router para que lea las env vars.
    import modules.command_router as cr  # noqa: WPS433 — import dentro de fixture intencional.
    cr = importlib.reload(cr)
    return cr


@pytest.fixture
def router(isolated_atlas):
    """Devuelve un ``IntentRouter`` inyectado con el ``handle`` del
    ``command_router`` recargado (aislado en ``tmp_path``)."""
    from atlas_push.intents import IntentRouter  # noqa: WPS433
    return IntentRouter(handle_fn=isolated_atlas.handle)


# ---------------------------------------------------------------------------
# Tabla de casos: 12 kinds + empty
# ---------------------------------------------------------------------------


CASES = [
    # (descripcion, input, kind_esperado)
    ("status slash", "/status", "status"),
    ("doctor slash", "/doctor", "doctor"),
    ("modules slash", "/modules", "modules"),
    ("snapshot slash sin label", "/snapshot", "snapshot"),
    ("snapshot slash con label", "/snapshot mi-etiqueta", "snapshot"),
    ("note create slash", "/note create Foo", "note.create"),
    ("note append slash", "/note append Foo | hola mundo", "note.append"),
    ("note view slash", "/note view Foo", "note.view"),
    (
        "natural note create — llamada",
        "Atlas, crea una nota llamada Bar",
        "natural.note.create",
    ),
    (
        "natural note create — con el título",
        "Atlas, crea una nota con el título Baz",
        "natural.note.create",
    ),
    (
        "natural note append",
        "Atlas, agrega a la nota Baz que hola mundo",
        "natural.note.append",
    ),
    (
        "natural snapshot",
        "Atlas, crea un snapshot llamado hito-1",
        "natural.snapshot",
    ),
    (
        "natural modules — tienes",
        "Atlas, dime qué módulos tienes activos",
        "natural.modules",
    ),
    (
        "inbox fallback",
        "esto es texto libre que no encaja en nada",
        "inbox.fallback",
    ),
    ("empty string", "", "empty"),
    # Nota: ``command_router.handle`` usa ``if not text``, que es
    # truthy para un string de sólo espacios. Por tanto whitespace
    # NO se clasifica como ``empty``: cae al fallback de inbox.
    ("whitespace only", "   \n\t ", "inbox.fallback"),
]


# ---------------------------------------------------------------------------
# Clasificación y paridad de output
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("desc,text,expected_kind", CASES, ids=[c[0] for c in CASES])
def test_kind_classification(router, desc, text, expected_kind):
    result = router.handle(text)
    assert result.kind == expected_kind, (
        f"[{desc}] kind esperado {expected_kind!r}, obtuvo {result.kind!r}"
    )


@pytest.mark.parametrize("desc,text,expected_kind", CASES, ids=[c[0] for c in CASES])
def test_raw_input_preserved(router, desc, text, expected_kind):
    result = router.handle(text)
    assert result.raw_input == text


@pytest.mark.parametrize("desc,text,expected_kind", CASES, ids=[c[0] for c in CASES])
def test_meta_is_empty_dict_in_b(router, desc, text, expected_kind):
    result = router.handle(text)
    assert result.meta == {}
    assert isinstance(result.meta, dict)


def test_output_parity_against_command_router(router, isolated_atlas):
    """Para cada caso no‑empty, ``IntentRouter.handle(text).output`` es
    byte‑idéntico a ``command_router.handle(text)``.

    Se ejecuta para cada caso en fila, delegando al mismo
    ``handle`` (módulo ya aislado en ``tmp_path``). Esto fija que
    ``IntentRouter`` no transforma el string de salida.
    """
    for desc, text, expected_kind in CASES:
        if expected_kind == "empty":
            # ``IntentRouter`` cortocircuita y no llama a ``handle``.
            # Verificamos que el literal coincide con lo que
            # ``command_router.handle("")`` devuelve hoy.
            assert router.handle(text).output == "ATLAS: vacío."
            assert isolated_atlas.handle(text) == "ATLAS: vacío."
            continue

        direct = isolated_atlas.handle(text)
        via_router = router.handle(text).output
        assert via_router == direct, (
            f"[{desc}] paridad rota:\n"
            f"  direct  = {direct!r}\n"
            f"  router  = {via_router!r}"
        )


# ---------------------------------------------------------------------------
# ok semantics
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("desc,text,expected_kind", CASES, ids=[c[0] for c in CASES])
def test_ok_field_semantics(router, desc, text, expected_kind):
    result = router.handle(text)
    if expected_kind == "empty":
        assert result.ok is False
    else:
        assert result.ok is True


# ---------------------------------------------------------------------------
# Inmutabilidad
# ---------------------------------------------------------------------------


def test_intent_result_is_frozen(router):
    result = router.handle("/status")
    with pytest.raises(FrozenInstanceError):
        result.kind = "doctor"  # type: ignore[misc]
    with pytest.raises(FrozenInstanceError):
        result.output = "mutated"  # type: ignore[misc]
    with pytest.raises(FrozenInstanceError):
        result.ok = False  # type: ignore[misc]


# ---------------------------------------------------------------------------
# Comportamiento defensivo ante excepciones del handle subyacente
# ---------------------------------------------------------------------------


def test_router_wraps_exceptions_instead_of_raising():
    """Si el ``handle`` subyacente lanza, el router devuelve
    ``IntentResult`` con ``ok=False`` y ``output=str(exc)``, sin
    propagar la excepción al caller."""
    from atlas_push.intents import IntentRouter

    def _boom(_text: str) -> str:
        raise RuntimeError("kaboom")

    router = IntentRouter(handle_fn=_boom)
    result = router.handle("/status")
    assert result.ok is False
    assert result.kind == "status"
    assert result.output == "kaboom"
    assert result.raw_input == "/status"


# ---------------------------------------------------------------------------
# None como entrada
# ---------------------------------------------------------------------------


def test_none_input_is_treated_as_empty(router):
    # Cualquier llamada defensiva con None debe caer en "empty" y no
    # reventar.
    result = router.handle(None)  # type: ignore[arg-type]
    assert result.kind == "empty"
    assert result.ok is False
    assert result.output == "ATLAS: vacío."
    assert result.raw_input == ""
