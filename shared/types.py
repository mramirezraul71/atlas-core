"""Tipos compartidos entre NEXUS y PUSH para el contrato de integración."""

from typing import Optional, Any


def nexus_status_shape() -> dict:
    """Forma esperada del status de NEXUS para el panel unificado."""
    return {
        "ok": bool,
        "connected": bool,
        "base_url": Optional[str],
        "directives": Optional[dict],
        "summary": Optional[dict],
        "error": Optional[str],
    }
