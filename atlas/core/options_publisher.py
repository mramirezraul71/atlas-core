"""
Publicación periódica del estado de opciones para dashboard / Grafana (JSON en disco).

Sin scheduler interno: el backend de Atlas Code Quant llama ``publish_once()`` cuando corresponda.

Ejemplo de integración en el loop del backend::

    reporter = OptionsReporter(options_service)
    publisher = OptionsStatePublisher(
        reporter,
        output_path=r"C:\\ATLAS_PUSH\\atlas_code_quant\\data\\options\\latest.json",
        history_dir=r"C:\\ATLAS_PUSH\\atlas_code_quant\\data\\options\\history",
        keep_last=20,
    )
    state = publisher.publish_once()  # dict + escritura atómica a latest + histórico opcional
"""
from __future__ import annotations

import json
import os
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .options_reporter import OptionsReporter

_HISTORY_GLOB = "options-state-*.json"


def _utc_timestamp_filename() -> str:
    """Marca temporal UTC sin ``:`` (válido en Windows)."""
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")


def _write_json_atomic(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path = path.resolve()
    fd, tmp_name = tempfile.mkstemp(
        suffix=".json.tmp",
        prefix=".options-state-",
        dir=str(path.parent),
    )
    tmp_path = Path(tmp_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        os.replace(str(tmp_path), str(path))
    except BaseException:
        try:
            tmp_path.unlink(missing_ok=True)
        except OSError:
            pass
        raise


def _history_candidates(history_dir: Path) -> list[Path]:
    return sorted(history_dir.glob(_HISTORY_GLOB))


def _prune_history(history_dir: Path, keep_last: int) -> None:
    if keep_last <= 0:
        return
    files = _history_candidates(history_dir)
    if len(files) <= keep_last:
        return
    # Más reciente primero (mtime); conservar keep_last.
    files.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    for stale in files[keep_last:]:
        try:
            stale.unlink()
        except OSError:
            pass


class OptionsStatePublisher:
    """
    Escribe el snapshot del reporter en ``output_path`` y, opcionalmente, copias rotativas
    bajo ``history_dir``.
    """

    def __init__(
        self,
        reporter: OptionsReporter,
        *,
        output_path: str,
        history_dir: str | None = None,
        keep_last: int = 0,
    ) -> None:
        self._reporter = reporter
        self._output_path = output_path
        self._history_dir = history_dir
        self._keep_last = int(keep_last)

    def publish_once(self) -> dict[str, Any]:
        data = self._reporter.snapshot_state()
        main = Path(self._output_path)
        _write_json_atomic(main, data)

        if self._history_dir:
            hist = Path(self._history_dir)
            hist.mkdir(parents=True, exist_ok=True)
            stamp = _utc_timestamp_filename()
            hist_file = hist / f"options-state-{stamp}.json"
            _write_json_atomic(hist_file, data)
            _prune_history(hist, self._keep_last)

        return data
