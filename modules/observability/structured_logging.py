"""
Logging estructurado JSON para observabilidad.
"""
import json
import logging
import sys
from datetime import datetime
from typing import Any, Dict


class StructuredFormatter(logging.Formatter):
    """Formatea registros como JSON."""

    def format(self, record: logging.LogRecord) -> str:
        log_data: Dict[str, Any] = {
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
            "module": getattr(record, "module", ""),
            "function": getattr(record, "funcName", ""),
            "line": getattr(record, "lineno", 0),
        }
        if record.exc_info:
            log_data["exception"] = self.formatException(record.exc_info)
        return json.dumps(log_data, default=str)


class StructuredLogger:
    """Logger que escribe líneas JSON a stdout."""

    def __init__(self, name: str, level: int = logging.INFO):
        self.logger = logging.getLogger(name)
        self.logger.setLevel(level)
        self.logger.handlers.clear()
        handler = logging.StreamHandler(sys.stdout)
        handler.setFormatter(StructuredFormatter())
        self.logger.addHandler(handler)

    def _log(self, level: str, message: str, **extra: Any) -> None:
        log_data = {
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "level": level,
            "message": message,
            **extra,
        }
        msg = json.dumps(log_data, default=str)
        if level == "INFO":
            self.logger.info(msg)
        elif level == "WARNING":
            self.logger.warning(msg)
        elif level == "ERROR":
            self.logger.error(msg)
        elif level == "DEBUG":
            self.logger.debug(msg)

    def info(self, message: str, **extra: Any) -> None:
        self._log("INFO", message, **extra)

    def warning(self, message: str, **extra: Any) -> None:
        self._log("WARNING", message, **extra)

    def error(self, message: str, **extra: Any) -> None:
        self._log("ERROR", message, **extra)

    def debug(self, message: str, **extra: Any) -> None:
        self._log("DEBUG", message, **extra)
