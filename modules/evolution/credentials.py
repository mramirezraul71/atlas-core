"""Integración de credenciales para producción: lee C:\\dev\\credenciales.txt y extrae tokens de PyPI, GitHub y Hugging Face. Parser CLAVE=VALOR."""
from __future__ import annotations

import logging
import re
from pathlib import Path
from typing import Dict, Optional

logger = logging.getLogger("atlas.evolution.credentials")

CREDENTIALS_PATH = Path(r"C:\dev\credenciales.txt")
KEY_ALIASES = {
    "github_token": ["GITHUB_TOKEN", "GITHUB_ACCESS_TOKEN", "GH_TOKEN"],
    "hf_token": ["HF_TOKEN", "HUGGINGFACE_TOKEN", "HUGGING_FACE_TOKEN"],
    "pypi_token": ["PYPI_TOKEN", "PIP_PASSWORD", "TWINE_PASSWORD"],
}
PYPI_USERNAME = "__token__"
PYPI_PASSWORD_PREFIX = "pypi-"


def _parse_line(line: str) -> Optional[tuple]:
    """Extrae (key, value) de una línea. Acepta CLAVE=VALOR, CLAVE = VALOR, CLAVE: VALOR."""
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    for sep in ("=", ":"):
        if sep in line:
            i = line.index(sep)
            key = line[:i].strip()
            value = line[i + 1 :].strip()
            if key and value:
                value = value.strip('"').strip("'")
                return (key.upper().replace("-", "_"), value)
    return None


def load_credentials(path: Optional[Path] = None) -> Dict[str, str]:
    """
    Lee el archivo de credenciales y devuelve un dict normalizado.
    Claves: github_token, hf_token (minúsculas). Si falta una clave, no se incluye.
    """
    p = path or CREDENTIALS_PATH
    raw: Dict[str, str] = {}
    if not p.exists():
        logger.warning("Credenciales no encontradas: %s", p)
        return _normalize(raw)
    try:
        with open(p, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                parsed = _parse_line(line)
                if parsed:
                    k, v = parsed
                    raw[k] = v
    except Exception as e:
        logger.exception("Error leyendo credenciales: %s", e)
    return _normalize(raw)


def _normalize(raw: Dict[str, str]) -> Dict[str, str]:
    """Mapea alias a claves canónicas (github_token, hf_token, pypi_token)."""
    out: Dict[str, str] = {}
    for canonical, aliases in KEY_ALIASES.items():
        for alias in aliases:
            if alias in raw and (raw[alias] or "").strip():
                v = raw[alias].strip()
                if canonical == "pypi_token":
                    v = ensure_pypi_password(v)
                out[canonical] = v
                break
    return out


def ensure_pypi_password(value: str) -> str:
    """Asegura que la contraseña PyPI incluya el prefijo pypi- (requerido por la API)."""
    if not value:
        return value
    value = value.strip().strip('"').strip("'")
    if value.lower().startswith(PYPI_PASSWORD_PREFIX.lower()):
        return value
    return PYPI_PASSWORD_PREFIX + value


def get_pypi_auth(creds: Dict[str, str]) -> tuple:
    """Devuelve (username, password) para PyPI: __token__ y valor con prefijo pypi-."""
    token = creds.get("pypi_token", "").strip()
    if not token:
        return ("", "")
    return (PYPI_USERNAME, ensure_pypi_password(token))
