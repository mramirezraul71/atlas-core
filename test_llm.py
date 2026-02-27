from __future__ import annotations

import os

import pytest


def _llm_tests_enabled() -> bool:
    """Evita llamadas de red durante `pytest` por defecto.

    Activación explícita:
    - set RUN_LLM_TESTS=1
    - proveer API key (OPENAI_API_KEY o ANTHROPIC_API_KEY)
    """
    if os.getenv("RUN_LLM_TESTS", "").strip() not in ("1", "true", "yes", "on"):
        return False
    return bool(os.getenv("OPENAI_API_KEY") or os.getenv("ANTHROPIC_API_KEY"))


pytestmark = pytest.mark.skipif(
    not _llm_tests_enabled(),
    reason="LLM smoke test deshabilitado por defecto (set RUN_LLM_TESTS=1 y API key válida).",
)


def test_llm_think_smoke():
    from modules.atlas_llm import atlas

    out = atlas.think("Di hola, confirma que estás vivo y listo")
    assert isinstance(out, str)
    assert out.strip()
