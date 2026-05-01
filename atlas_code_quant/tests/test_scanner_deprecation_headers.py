"""F3 — Tests de deprecación HTTP de endpoints scanner legacy.

Validan que los 10 endpoints scanner (5 v1 + 5 v2) en
``atlas_code_quant/api/main.py`` están correctamente marcados como
deprecated y aplican el helper ``_apply_scanner_deprecation_headers``
antes de cualquier lógica de negocio.

Estos tests usan **inspección de AST** sobre el módulo, no
``TestClient``, porque la app FastAPI tiene dependencias preexistentes
(``sqlalchemy``, ``backtesting``) que no están instaladas en todos los
sandboxes — y eso es ortogonal a F3. La inspección de AST nos da una
garantía estructural fuerte y reproducible de que:

    1. El helper y constantes F3 existen y son consistentes.
    2. Cada handler legacy:
       * recibe ``response: Response`` como parámetro,
       * lleva ``deprecated=True`` en su decorador,
       * llama a ``_apply_scanner_deprecation_headers(...)`` con la
         ruta y ``api_version`` correctas,
       * NO modifica su status code, body, ni schema (el AST conserva
         el cuerpo intacto: comprobamos que ``return StdResponse(...)``
         sigue siendo el camino normal v1, y que las v2 siguen siendo
         alias delegados a su par v1).

Ver:
    - docs/ATLAS_CODE_QUANT_F3_SCANNER_DEPRECATION.md
    - atlas_code_quant/config/legacy_flags.py (ATLAS_LEGACY_SCANNER_ENABLED)
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parents[2]
API_MAIN = REPO_ROOT / "atlas_code_quant" / "api" / "main.py"


V1_ENDPOINTS = {
    "scanner_status": "/scanner/status",
    "scanner_report": "/scanner/report",
    "scanner_config": "/scanner/config",
    "scanner_control": "/scanner/control",
    "scanner_universe_search": "/scanner/universe/search",
}

V2_ENDPOINTS = {
    "scanner_status_v2": "/api/v2/quant/scanner/status",
    "scanner_report_v2": "/api/v2/quant/scanner/report",
    "scanner_config_v2": "/api/v2/quant/scanner/config",
    "scanner_control_v2": "/api/v2/quant/scanner/control",
    "scanner_universe_search_v2": "/api/v2/quant/scanner/universe/search",
}

ALL_ENDPOINTS = {**V1_ENDPOINTS, **V2_ENDPOINTS}


@pytest.fixture(scope="module")
def main_source() -> str:
    assert API_MAIN.exists(), f"api/main.py no encontrado en {API_MAIN}"
    return API_MAIN.read_text(encoding="utf-8")


@pytest.fixture(scope="module")
def main_tree(main_source: str) -> ast.Module:
    return ast.parse(main_source, filename=str(API_MAIN))


@pytest.fixture(scope="module")
def handler_nodes(main_tree: ast.Module) -> dict[str, ast.AsyncFunctionDef]:
    """Mapea nombre de handler -> nodo AST de su ``async def``."""
    found: dict[str, ast.AsyncFunctionDef] = {}
    for node in ast.walk(main_tree):
        if isinstance(node, ast.AsyncFunctionDef) and node.name in ALL_ENDPOINTS:
            found[node.name] = node
    return found


# ---------------------------------------------------------------------------
# Constantes y helper F3 a nivel módulo
# ---------------------------------------------------------------------------


def test_module_defines_deprecation_constants(main_source: str) -> None:
    """Las constantes y el logger F3 deben existir en ``api/main.py``."""
    assert '_SCANNER_DEPRECATION_SUNSET: str = "2026-12-31T23:59:59Z"' in main_source
    assert (
        '_SCANNER_DEPRECATION_DOC_LINK: str = \'</docs/scanner-legacy>; rel="deprecation"\''
        in main_source
    )
    assert (
        '_scanner_deprecation_logger = logging.getLogger("quant.api.deprecation.scanner")'
        in main_source
    )


def test_helper_apply_scanner_deprecation_headers_defined(main_tree: ast.Module) -> None:
    """``_apply_scanner_deprecation_headers`` debe estar definido a nivel módulo."""
    helper: ast.FunctionDef | None = None
    for node in main_tree.body:
        if (
            isinstance(node, ast.FunctionDef)
            and node.name == "_apply_scanner_deprecation_headers"
        ):
            helper = node
            break
    assert helper is not None, "Helper F3 no definido en api/main.py"

    # Firma esperada: response, *, endpoint, api_version
    args = helper.args
    assert [a.arg for a in args.args] == ["response"]
    assert [a.arg for a in args.kwonlyargs] == ["endpoint", "api_version"]


def test_helper_sets_three_required_headers(main_source: str) -> None:
    """El helper debe escribir los tres headers HTTP de deprecación."""
    assert 'response.headers["Deprecation"] = "true"' in main_source
    assert 'response.headers["Sunset"] = _SCANNER_DEPRECATION_SUNSET' in main_source
    assert 'response.headers["Link"] = _SCANNER_DEPRECATION_DOC_LINK' in main_source


def test_helper_emits_structured_warning(main_source: str) -> None:
    """El helper debe emitir un WARNING estructurado por hit a endpoint legacy."""
    assert "_scanner_deprecation_logger.warning(" in main_source
    assert '"deprecated scanner endpoint hit"' in main_source
    assert '"event": "scanner_endpoint_deprecated"' in main_source
    assert '"replacement": "/api/radar/* (institutional Radar)"' in main_source


# ---------------------------------------------------------------------------
# Handlers individuales — comprobaciones estructurales
# ---------------------------------------------------------------------------


def _decorator_has_deprecated_true(node: ast.AsyncFunctionDef) -> bool:
    for dec in node.decorator_list:
        if not isinstance(dec, ast.Call):
            continue
        for kw in dec.keywords:
            if kw.arg == "deprecated" and isinstance(kw.value, ast.Constant):
                if kw.value.value is True:
                    return True
    return False


def _decorator_path(node: ast.AsyncFunctionDef) -> str | None:
    for dec in node.decorator_list:
        if not isinstance(dec, ast.Call):
            continue
        if not dec.args:
            continue
        first = dec.args[0]
        if isinstance(first, ast.Constant) and isinstance(first.value, str):
            return first.value
    return None


def _has_response_param(node: ast.AsyncFunctionDef) -> bool:
    for a in node.args.args:
        if a.arg == "response":
            return True
    return False


def _calls_apply_helper(node: ast.AsyncFunctionDef) -> ast.Call | None:
    for sub in ast.walk(node):
        if (
            isinstance(sub, ast.Call)
            and isinstance(sub.func, ast.Name)
            and sub.func.id == "_apply_scanner_deprecation_headers"
        ):
            return sub
    return None


@pytest.mark.parametrize("handler_name,expected_path", list(ALL_ENDPOINTS.items()))
def test_handler_exists(
    handler_nodes: dict[str, ast.AsyncFunctionDef],
    handler_name: str,
    expected_path: str,
) -> None:
    assert handler_name in handler_nodes, (
        f"Handler {handler_name} no encontrado en api/main.py"
    )
    path = _decorator_path(handler_nodes[handler_name])
    assert path == expected_path, (
        f"{handler_name}: ruta esperada {expected_path}, encontrada {path}"
    )


@pytest.mark.parametrize("handler_name", list(ALL_ENDPOINTS.keys()))
def test_handler_decorator_has_deprecated_true(
    handler_nodes: dict[str, ast.AsyncFunctionDef], handler_name: str
) -> None:
    """Cada handler legacy debe llevar ``deprecated=True`` (F3)."""
    node = handler_nodes[handler_name]
    assert _decorator_has_deprecated_true(node), (
        f"{handler_name}: falta ``deprecated=True`` en el decorador FastAPI"
    )


@pytest.mark.parametrize("handler_name", list(ALL_ENDPOINTS.keys()))
def test_handler_accepts_response_object(
    handler_nodes: dict[str, ast.AsyncFunctionDef], handler_name: str
) -> None:
    """Cada handler legacy debe recibir ``response: Response`` para poder
    estampar headers sin ``Response(...)`` adicional."""
    node = handler_nodes[handler_name]
    assert _has_response_param(node), (
        f"{handler_name}: falta el parámetro ``response`` en la firma"
    )


@pytest.mark.parametrize("handler_name,expected_path", list(ALL_ENDPOINTS.items()))
def test_handler_calls_apply_helper_with_correct_args(
    handler_nodes: dict[str, ast.AsyncFunctionDef],
    handler_name: str,
    expected_path: str,
) -> None:
    """Cada handler legacy debe llamar al helper F3 con su ruta y api_version."""
    node = handler_nodes[handler_name]
    call = _calls_apply_helper(node)
    assert call is not None, (
        f"{handler_name}: no llama a _apply_scanner_deprecation_headers"
    )

    # endpoint kwarg
    endpoint_value: str | None = None
    api_version_value: str | None = None
    for kw in call.keywords:
        if kw.arg == "endpoint" and isinstance(kw.value, ast.Constant):
            endpoint_value = kw.value.value
        elif kw.arg == "api_version" and isinstance(kw.value, ast.Constant):
            api_version_value = kw.value.value

    assert endpoint_value == expected_path, (
        f"{handler_name}: helper invocado con endpoint={endpoint_value!r}, "
        f"esperado {expected_path!r}"
    )
    expected_version = "v2" if handler_name in V2_ENDPOINTS else "v1"
    assert api_version_value == expected_version, (
        f"{handler_name}: helper invocado con api_version={api_version_value!r}, "
        f"esperado {expected_version!r}"
    )


@pytest.mark.parametrize("handler_name", list(V1_ENDPOINTS.keys()))
def test_v1_handler_calls_helper_before_auth(
    handler_nodes: dict[str, ast.AsyncFunctionDef], handler_name: str
) -> None:
    """En v1, el helper debe ejecutarse ANTES de ``_auth(...)`` para que los
    headers de deprecación estén presentes incluso si la auth falla."""
    node = handler_nodes[handler_name]

    helper_lineno: int | None = None
    auth_lineno: int | None = None
    for sub in ast.walk(node):
        if isinstance(sub, ast.Call) and isinstance(sub.func, ast.Name):
            if sub.func.id == "_apply_scanner_deprecation_headers" and helper_lineno is None:
                helper_lineno = sub.lineno
            elif sub.func.id == "_auth" and auth_lineno is None:
                auth_lineno = sub.lineno

    assert helper_lineno is not None, f"{handler_name}: no llama al helper"
    assert auth_lineno is not None, f"{handler_name}: no llama a _auth"
    assert helper_lineno < auth_lineno, (
        f"{handler_name}: helper en línea {helper_lineno} debe ejecutarse "
        f"antes que _auth en línea {auth_lineno}"
    )


@pytest.mark.parametrize("v2_name", list(V2_ENDPOINTS.keys()))
def test_v2_handler_delegates_to_v1(
    handler_nodes: dict[str, ast.AsyncFunctionDef], v2_name: str
) -> None:
    """Cada handler v2 debe seguir delegando en su par v1, garantizando que
    F3 NO cambia la semántica ni el cuerpo de la respuesta."""
    node = handler_nodes[v2_name]
    expected_v1 = v2_name.removesuffix("_v2")

    delegates = False
    for sub in ast.walk(node):
        if (
            isinstance(sub, ast.Call)
            and isinstance(sub.func, ast.Name)
            and sub.func.id == expected_v1
        ):
            delegates = True
            break
    assert delegates, (
        f"{v2_name}: debe seguir delegando en {expected_v1} (no cambiar semántica en F3)"
    )


@pytest.mark.parametrize("handler_name", list(ALL_ENDPOINTS.keys()))
def test_handler_response_body_unchanged(
    handler_nodes: dict[str, ast.AsyncFunctionDef], handler_name: str
) -> None:
    """El handler debe seguir devolviendo (directa o indirectamente) un
    ``StdResponse``, garantizando que F3 no altera el contrato del body."""
    node = handler_nodes[handler_name]
    returns_std = False
    for sub in ast.walk(node):
        if not isinstance(sub, ast.Return) or sub.value is None:
            continue
        # Caso 1: return StdResponse(...)
        if isinstance(sub.value, ast.Call) and isinstance(sub.value.func, ast.Name):
            if sub.value.func.id == "StdResponse":
                returns_std = True
                break
        # Caso 2 (v2): return await scanner_xxx(...) — delega en handler v1
        if isinstance(sub.value, ast.Await) and isinstance(sub.value.value, ast.Call):
            inner = sub.value.value.func
            if isinstance(inner, ast.Name) and inner.id in V1_ENDPOINTS:
                returns_std = True
                break
    assert returns_std, (
        f"{handler_name}: el cuerpo de respuesta no devuelve StdResponse "
        f"ni delega en un handler v1 (F3 no debe cambiar el body)"
    )


# ---------------------------------------------------------------------------
# Flag legacy
# ---------------------------------------------------------------------------


def test_legacy_flag_atlas_legacy_scanner_enabled_documented() -> None:
    """``ATLAS_LEGACY_SCANNER_ENABLED`` debe estar documentada y por defecto True."""
    from atlas_code_quant.config import legacy_flags

    assert hasattr(legacy_flags, "ATLAS_LEGACY_SCANNER_ENABLED"), (
        "F3 documenta ATLAS_LEGACY_SCANNER_ENABLED en config/legacy_flags.py"
    )
    assert legacy_flags.ATLAS_LEGACY_SCANNER_ENABLED is True
    assert "ATLAS_LEGACY_SCANNER_ENABLED" in legacy_flags.__all__
