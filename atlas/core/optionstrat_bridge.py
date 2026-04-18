"""
Bridge disco para la vista OptionStrat del dashboard (Atlas Code Quant).

Escribe dos artefactos JSON configurables:

- **Live:** posiciones abiertas + clave opcional ``metrics`` (distribuciones, PnL diario,
  buckets horarios para heatmap). Los campos ``updated_at``, ``open_count`` y ``positions``
  se mantienen; desactivar métricas con ``include_metrics=False``.
- **Histórico:** posiciones cerradas, cada ``position_id`` una sola vez (append idempotente).

Sin scheduler: el loop del backend invoca ``sync_from_options()`` junto con otras tareas
(p. ej. ``OptionsStatePublisher.publish_once()``) cuando corresponda.

Ejemplo de cableado coordinado::

    reporter = OptionsReporter(options_service)
    publisher = OptionsStatePublisher(reporter, output_path=".../options_summary.json", ...)
    bridge = OptionStratBridge(
        options_service,
        live_output_path=".../optionstrat_live.json",
        history_output_path=".../optionstrat_history.json",
    )
    publisher.publish_once()
    bridge.sync_from_options()
"""
from __future__ import annotations

import json
import os
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .options_client import AtlasOptionsService
from .options_metrics import OptionsMetrics
from .options_reporter import _estimate_notional


def _atomic_write_json(path: Path, data: Any) -> None:
    """Serializa ``data`` (dict o list) a JSON con escritura atómica (temp + replace)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path = path.resolve()
    fd, tmp_name = tempfile.mkstemp(
        suffix=".json.tmp",
        prefix=".optionstrat-",
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


def _load_history_list(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    try:
        with open(path, encoding="utf-8") as f:
            raw = json.load(f)
    except (json.JSONDecodeError, OSError):
        return []
    if not isinstance(raw, list):
        return []
    out: list[dict[str, Any]] = []
    for item in raw:
        if isinstance(item, dict) and "position_id" in item:
            out.append(item)
    return out


class OptionStratBridge:
    """
    Sincroniza el libro de opciones del servicio hacia JSON live + histórico OptionStrat.
    """

    def __init__(
        self,
        options_service: AtlasOptionsService,
        *,
        live_output_path: str,
        history_output_path: str,
        include_metrics: bool = True,
    ) -> None:
        self._svc = options_service
        self._live_output_path = live_output_path
        self._history_output_path = history_output_path
        self._include_metrics = include_metrics

    def sync_from_options(self) -> dict[str, Any]:
        self._svc.mark_all()
        get_quote = self._svc.client.provider.get_quote

        open_list = self._svc.list_open_positions()
        closed_list = self._svc.list_closed_positions()

        live_enriched: list[dict[str, Any]] = []
        for pos in open_list:
            d = self._svc.get_position(pos.position_id)
            st = str(d.get("strategy_type", "") or "")
            notion = round(_estimate_notional(pos, st, get_quote), 2)
            row = dict(d)
            row["notional_estimate"] = notion
            live_enriched.append(row)

        closed_enriched: list[dict[str, Any]] = []
        for pos in closed_list:
            d = self._svc.get_position(pos.position_id)
            st = str(d.get("strategy_type", "") or "")
            notion = round(_estimate_notional(pos, st, get_quote), 2)
            row = dict(d)
            row["notional_estimate"] = notion
            closed_enriched.append(row)

        live_feed = self._build_live_payload(live_enriched)
        live_doc: dict[str, Any] = {
            "updated_at": datetime.now(timezone.utc).isoformat(),
            "open_count": len(live_feed),
            "positions": live_feed,
        }
        if self._include_metrics:
            live_doc["metrics"] = OptionsMetrics(self._svc).compute_aggregates(
                skip_mark_all=True,
            )
        _atomic_write_json(Path(self._live_output_path), live_doc)

        self._append_history_records(closed_enriched)

        return {
            "open_count": len(live_enriched),
            "closed_count": len(closed_enriched),
            "live_output": self._live_output_path,
            "history_output": self._history_output_path,
        }

    def _build_live_payload(self, positions_open: list[dict[str, Any]]) -> list[dict[str, Any]]:
        """Formatea filas enriquecidas (``get_position`` + ``notional_estimate``) para OptionStrat."""
        out: list[dict[str, Any]] = []
        for d in positions_open:
            snap = d.get("last_snapshot")
            snap_d = snap if isinstance(snap, dict) else {}
            u = snap_d.get("unrealized_pnl")
            row: dict[str, Any] = {
                "id": d["position_id"],
                "position_id": d["position_id"],
                "symbol": d.get("symbol", ""),
                "strategy_type": d.get("strategy_type", ""),
                "opened_at": d.get("opened_at"),
                "entry_net_premium": d["entry_net_premium"],
                "notional_estimate": float(d.get("notional_estimate", 0.0)),
                "unrealized_pnl": float(u) if u is not None else None,
            }
            if "timestamp" in snap_d:
                row["last_mark_at"] = snap_d["timestamp"]
            if "match_info" in snap_d:
                row["match_info"] = snap_d["match_info"]
            out.append(row)
        return out

    def _history_record(self, d: dict[str, Any]) -> dict[str, Any]:
        return {
            "position_id": d["position_id"],
            "symbol": d.get("symbol", ""),
            "strategy_type": d.get("strategy_type", ""),
            "opened_at": d.get("opened_at"),
            "closed_at": d.get("closed_at"),
            "entry_net_premium": d["entry_net_premium"],
            "realized_pnl": d.get("realized_pnl"),
            "notional_estimate": float(d.get("notional_estimate", 0.0)),
        }

    def _append_history_records(self, positions_closed: list[dict[str, Any]]) -> None:
        path = Path(self._history_output_path)
        history = _load_history_list(path)
        existing = {str(r["position_id"]) for r in history}
        added_any = False
        for d in positions_closed:
            pid = str(d["position_id"])
            if pid in existing:
                continue
            history.append(self._history_record(d))
            existing.add(pid)
            added_any = True
        if added_any:
            _atomic_write_json(path, history)
