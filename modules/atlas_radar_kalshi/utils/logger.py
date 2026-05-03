"""
logger.py — Logger del Radar Kalshi.

Compatible con el dashboard de atlas-core:

- Emite en formato ``[ts] [LEVEL] [radar.kalshi.<comp>] mensaje``.
- Persiste en ``<log_dir>/atlas.log`` (compartido con ``core/logger.py``)
  y en ``<log_dir>/radar_kalshi.log`` (canal dedicado).
- Expone también un ``StreamHandler`` para stdout cuando se ejecuta
  ``main.py`` de forma standalone.
"""
from __future__ import annotations

import logging
import sys
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Optional


class _SafeRotatingFileHandler(RotatingFileHandler):
    """En Windows, ``os.rename`` del rollover falla si otro proceso mantiene el archivo."""

    def doRollover(self) -> None:  # type: ignore[override]
        try:
            super().doRollover()
        except (PermissionError, OSError):
            pass


_FORMAT = "[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s"
_DATEFMT = "%Y-%m-%d %H:%M:%S"

_configured: dict[str, bool] = {}


def get_logger(component: str, log_dir: Optional[Path] = None,
               level: str = "INFO") -> logging.Logger:
    """
    Devuelve un logger ``radar.kalshi.<component>`` configurado una sola
    vez por instancia de proceso.
    """
    name = f"radar.kalshi.{component}"
    logger = logging.getLogger(name)

    if _configured.get(name):
        return logger

    logger.setLevel(getattr(logging, level.upper(), logging.INFO))
    logger.propagate = False

    fmt = logging.Formatter(_FORMAT, datefmt=_DATEFMT)

    # 1) Stdout (útil en main.py)
    sh = logging.StreamHandler(sys.stdout)
    sh.setFormatter(fmt)
    logger.addHandler(sh)

    # 2) Archivos rotativos (compartidos con el dashboard de atlas-core)
    if log_dir is not None:
        try:
            log_dir.mkdir(parents=True, exist_ok=True)
            shared = _SafeRotatingFileHandler(
                log_dir / "atlas.log", maxBytes=5_000_000, backupCount=5,
                encoding="utf-8",
            )
            shared.setFormatter(fmt)
            logger.addHandler(shared)

            dedicated = _SafeRotatingFileHandler(
                log_dir / "radar_kalshi.log", maxBytes=10_000_000,
                backupCount=10, encoding="utf-8",
            )
            dedicated.setFormatter(fmt)
            logger.addHandler(dedicated)
        except Exception as exc:  # nunca rompemos por logs
            logger.warning("No se pudo crear file-handler: %s", exc)

    _configured[name] = True
    return logger
