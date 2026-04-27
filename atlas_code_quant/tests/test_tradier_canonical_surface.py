"""F4 — Tests estructurales de la implementación canónica Tradier.

Verifican que el stack ``atlas_code_quant.execution.tradier_execution``
(+ ``tradier_controls``, ``tradier_pdt_ledger``) sigue siendo el stack
canónico expuesto: clases / funciones públicas presentes, sin entrar en
comportamiento. También se asegura que el stack PHASE1 sigue presente
físicamente como legacy y que las flags de F4 están publicadas.

Estos tests usan **inspección AST** porque ambos stacks tienen
dependencias preexistentes no instaladas en este sandbox
(``backtesting`` para el canónico, ``atlas_options_brain`` para PHASE1).
Eso es ortogonal a F4 y no se resuelve aquí.

Ver:
    - docs/ATLAS_CODE_QUANT_F4_TRADIER_CANONICALIZATION.md
    - atlas_code_quant/execution/README_TRADIER.md
    - atlas_code_quant/config/legacy_flags.py
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]

CANONICAL_TRADIER_EXECUTION = (
    REPO_ROOT / "atlas_code_quant" / "execution" / "tradier_execution.py"
)
CANONICAL_TRADIER_CONTROLS = (
    REPO_ROOT / "atlas_code_quant" / "execution" / "tradier_controls.py"
)
CANONICAL_TRADIER_PDT_LEDGER = (
    REPO_ROOT / "atlas_code_quant" / "execution" / "tradier_pdt_ledger.py"
)
CANONICAL_README = (
    REPO_ROOT / "atlas_code_quant" / "execution" / "README_TRADIER.md"
)

PHASE1_TRADIER_EXECUTOR = (
    REPO_ROOT
    / "atlas_options_brain_fase1"
    / "atlas_options_brain"
    / "broker"
    / "tradier_executor.py"
)
PHASE1_TRADIER_LIVE = (
    REPO_ROOT
    / "atlas_options_brain_fase1"
    / "atlas_options_brain"
    / "broker"
    / "tradier_live.py"
)
PHASE1_BROKER_INIT = (
    REPO_ROOT
    / "atlas_options_brain_fase1"
    / "atlas_options_brain"
    / "broker"
    / "__init__.py"
)
PHASE1_TRADIER_PROVIDER = (
    REPO_ROOT
    / "atlas_options_brain_fase1"
    / "atlas_options_brain"
    / "providers"
    / "tradier_provider.py"
)


def _public_names(path: Path) -> set[str]:
    """Devuelve nombres públicos definidos a nivel módulo (no underscored)."""
    src = path.read_text(encoding="utf-8")
    tree = ast.parse(src, filename=str(path))
    names: set[str] = set()
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            if not node.name.startswith("_"):
                names.add(node.name)
        elif isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and not target.id.startswith("_"):
                    names.add(target.id)
    return names


# ---------------------------------------------------------------------------
# Stack canónico: presencia física y superficie pública
# ---------------------------------------------------------------------------


def test_canonical_tradier_files_exist() -> None:
    """Los tres módulos del stack canónico siguen presentes en disco."""
    assert CANONICAL_TRADIER_EXECUTION.exists()
    assert CANONICAL_TRADIER_CONTROLS.exists()
    assert CANONICAL_TRADIER_PDT_LEDGER.exists()


def test_canonical_readme_present_and_marks_canonical() -> None:
    """``execution/README_TRADIER.md`` declara el stack como canónico."""
    assert CANONICAL_README.exists(), "Falta execution/README_TRADIER.md"
    text = CANONICAL_README.read_text(encoding="utf-8")
    assert "canónica" in text.lower() or "canonical" in text.lower()
    assert "atlas_code_quant/execution" in text
    assert "atlas_options_brain_fase1" in text


def test_tradier_execution_docstring_marks_canonical_f4() -> None:
    """El docstring del módulo canónico contiene la marca ``.. canonical:: F4``."""
    src = CANONICAL_TRADIER_EXECUTION.read_text(encoding="utf-8")
    tree = ast.parse(src)
    doc = ast.get_docstring(tree) or ""
    assert ".. canonical:: F4" in doc, (
        "tradier_execution.py debe declarar .. canonical:: F4 en el docstring"
    )
    assert "CANÓNICA" in doc or "canónica" in doc.lower()
    assert "atlas_code_quant.execution" in doc


@pytest.mark.parametrize(
    "expected_name",
    [
        # Símbolos públicos esperados en el módulo canónico (sin probar comportamiento).
        "TradierOrderBlocked",
        "should_route_to_tradier",
        "is_opening_order",
        "build_tradier_order_payload",
        "route_order_to_tradier",
    ],
)
def test_canonical_module_exposes_expected_symbol(expected_name: str) -> None:
    """``tradier_execution`` debe seguir exponiendo su superficie pública pre-F4."""
    names = _public_names(CANONICAL_TRADIER_EXECUTION)
    assert expected_name in names, (
        f"Falta símbolo público {expected_name!r} en tradier_execution.py"
    )


def test_canonical_controls_exposes_account_session() -> None:
    """``tradier_controls`` debe seguir exponiendo el resolutor de cuenta y PDT."""
    names = _public_names(CANONICAL_TRADIER_CONTROLS)
    # Comprobaciones flexibles: aceptamos cualquier helper de session/pdt expuesto.
    assert "TradierAccountSession" in names
    expected_any = {"resolve_account_session", "check_pdt_status"}
    assert expected_any & names, (
        "tradier_controls.py debe exponer resolve_account_session y/o check_pdt_status"
    )


def test_canonical_pdt_ledger_exposes_record_intent() -> None:
    """``tradier_pdt_ledger`` debe seguir exponiendo el ledger de intents."""
    names = _public_names(CANONICAL_TRADIER_PDT_LEDGER)
    assert "record_live_order_intent" in names
    # count_intraday_day_trades es el otro símbolo importado por controls.
    assert "count_intraday_day_trades" in names


# ---------------------------------------------------------------------------
# Stack PHASE1: presencia física y marcado legacy
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "phase1_path",
    [PHASE1_TRADIER_EXECUTOR, PHASE1_TRADIER_LIVE, PHASE1_BROKER_INIT, PHASE1_TRADIER_PROVIDER],
)
def test_phase1_files_still_exist(phase1_path: Path) -> None:
    """F4 no borra nada del stack PHASE1, sólo lo etiqueta."""
    assert phase1_path.exists(), f"PHASE1 desaparecido inesperadamente: {phase1_path}"


@pytest.mark.parametrize(
    "phase1_path",
    [PHASE1_TRADIER_EXECUTOR, PHASE1_TRADIER_LIVE, PHASE1_BROKER_INIT, PHASE1_TRADIER_PROVIDER],
)
def test_phase1_files_marked_as_legacy(phase1_path: Path) -> None:
    """Cada archivo PHASE1 debe llevar el marcador ``.. legacy:: F4 PHASE1``."""
    src = phase1_path.read_text(encoding="utf-8")
    tree = ast.parse(src, filename=str(phase1_path))
    doc = ast.get_docstring(tree) or ""
    assert ".. legacy:: F4 PHASE1" in doc, (
        f"{phase1_path.name} debe declarar .. legacy:: F4 PHASE1 en el docstring"
    )
    assert "atlas_code_quant.execution.tradier_execution" in doc, (
        f"{phase1_path.name} debe apuntar al stack canónico en el docstring"
    )


def test_phase1_executor_signatures_unchanged() -> None:
    """Sanity: los símbolos públicos del ejecutor PHASE1 no cambian en F4."""
    names = _public_names(PHASE1_TRADIER_EXECUTOR)
    assert "TradierOrderExecutor" in names
    # TradierMultilegType es Literal alias a nivel módulo (Assign).
    assert "TradierMultilegType" in names


def test_phase1_live_signatures_unchanged() -> None:
    names = _public_names(PHASE1_TRADIER_LIVE)
    for sym in ("LiveOrder", "LiveOrderLeg", "TradierLiveExecutionSink", "TradierOrderBuilder"):
        assert sym in names, f"PHASE1 tradier_live: falta símbolo público {sym}"


# ---------------------------------------------------------------------------
# Flags F4 en legacy_flags.py
# ---------------------------------------------------------------------------


def test_legacy_flags_publish_canonical_and_phase1() -> None:
    """``legacy_flags.py`` debe publicar las flags documentales F4."""
    from atlas_code_quant.config import legacy_flags

    assert hasattr(legacy_flags, "ATLAS_TRADIER_CANONICAL_STACK")
    assert hasattr(legacy_flags, "ATLAS_TRADIER_PHASE1_LEGACY_STACK")
    assert legacy_flags.ATLAS_TRADIER_CANONICAL_STACK == "atlas_code_quant"
    assert legacy_flags.ATLAS_TRADIER_PHASE1_LEGACY_STACK == "atlas_options_brain_fase1"
    assert "ATLAS_TRADIER_CANONICAL_STACK" in legacy_flags.__all__
    assert "ATLAS_TRADIER_PHASE1_LEGACY_STACK" in legacy_flags.__all__
