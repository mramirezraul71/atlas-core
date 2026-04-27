"""
dashboard/router.py — Router FastAPI del Radar Kalshi.

Se monta en ``atlas_adapter.atlas_http_api:app`` (puerto 8791) y expone:

- ``GET  /ui/radar``               → SPA (HTML estático).
- ``GET  /api/radar/status``       → estado, balance, sesión.
- ``GET  /api/radar/markets``      → universo + última probabilidad.
- ``GET  /api/radar/decisions``    → últimas N :class:`BrainDecision`.
- ``GET  /api/radar/orders``       → órdenes enviadas (audit log).
- ``GET  /api/radar/metrics``      → KPIs (PnL acumulado, hit-rate, etc.).
- ``WS   /api/radar/stream``       → canal en vivo (eventos del scanner
                                     + decisiones brain).

Patrones aplicados (referencia: dashboards profesionales tipo
`Deephaven <https://deephaven.io/blog/2025/11/13/real-time-trading-dashboard/>`_
y guías de FastAPI + WebSockets):

- KPI tiles arriba (Balance, P&L día, Edge medio, Órdenes activas).
- Tabla en vivo de mercados ordenable por edge.
- Gráfico de probabilidad histórica por mercado.
- Panel de logs y eventos recientes.
- Tema oscuro consistente con Atlas + acceso bidireccional al
  dashboard central de atlas-core.
"""
from __future__ import annotations

import asyncio
import json
import os
from collections import deque
from pathlib import Path
from typing import Any, Deque, Optional

from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse
from pydantic import BaseModel, Field, field_validator

from ..config import RadarSettings, get_settings
from ..brain import BrainDecision
from ..executor import OrderResult
from ..risk import PositionSize
from ..state.journal import Journal
from ..metrics import compute as compute_perf, prometheus_text

# Visible en /ui/radar: si no coincide tras pull, reiniciar proceso :8791.
RADAR_UI_BUILD = "20260428-edge-labels-v2"


# ===========================================================================
# Estado en memoria compartido entre main.py y el router
# ===========================================================================
class RadarState:
    """Estado in-memory expuesto al dashboard."""

    def __init__(self, settings: Optional[RadarSettings] = None,
                 maxlen: int = 500) -> None:
        import os as _os
        self.settings = settings or get_settings()
        self.markets: dict[str, dict] = {}       # ticker -> última snapshot
        self.decisions: Deque[BrainDecision] = deque(maxlen=maxlen)
        self.orders: Deque[dict] = deque(maxlen=maxlen)
        self.events_log: Deque[str] = deque(maxlen=2000)
        # Saldo virtual paper visible al instante (antes de que arranque el orquestador)
        _live = _os.getenv("ATLAS_RADAR_LIVE", "0").strip() == "1"
        _paper = int(_os.getenv("RADAR_PAPER_BALANCE_CENTS", "1000000"))
        self.balance_cents: int = 0 if _live else max(100, _paper)
        self.session_started: float = 0.0
        self.subscribers: set[WebSocket] = set()

    # ------------------------------------------------------------------
    async def broadcast(self, payload: dict[str, Any]) -> None:
        dead = []
        msg = json.dumps(payload, default=str)
        for ws in list(self.subscribers):
            try:
                await ws.send_text(msg)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.subscribers.discard(ws)

    def push_decision(self, d: BrainDecision) -> None:
        self.decisions.append(d)

    def push_order(self, sizing: PositionSize, result: OrderResult) -> None:
        self.orders.append({
            "ts": result.ts.isoformat(),
            "sizing": sizing.model_dump(),
            "result": result.model_dump(mode="json"),
        })

    def update_market(self, ticker: str, data: dict) -> None:
        self.markets[ticker] = {**self.markets.get(ticker, {}), **data}


class RadarRuntimeConfigBody(BaseModel):
    """Carga útil para modificar entorno/modo/monto del radar."""

    environment: str = "demo"  # demo | prod
    execution_mode: str = "paper"  # paper | live
    paper_balance_usd: float = 1000.0
    api_key_id: Optional[str] = None
    private_key_path: Optional[str] = None
    persist_env: bool = True
    # Opcionales: si vienen en el JSON, se aplican en caliente (+ .env si persist_env)
    edge_threshold: Optional[float] = Field(
        default=None,
        description="Edge mínimo (0.05 = 5 pts prob.) para brain / señales.",
    )
    kelly_fraction: Optional[float] = Field(
        default=None,
        description="Fracción de Kelly en riesgo (0.25 = 25%).",
    )
    edge_net_min: Optional[float] = Field(
        default=None,
        description="Edge neto mínimo tras costos (gating), p. ej. 0.03 = 3%.",
    )

    @field_validator("environment")
    @classmethod
    def _validate_environment(cls, value: str) -> str:
        v = value.lower().strip()
        if v not in {"demo", "prod"}:
            raise ValueError("environment debe ser demo o prod")
        return v

    @field_validator("execution_mode")
    @classmethod
    def _validate_execution_mode(cls, value: str) -> str:
        v = value.lower().strip()
        if v not in {"paper", "live"}:
            raise ValueError("execution_mode debe ser paper o live")
        return v

    @field_validator("paper_balance_usd")
    @classmethod
    def _validate_paper_balance(cls, value: float) -> float:
        if value <= 0:
            raise ValueError("paper_balance_usd debe ser mayor a 0")
        return value

    @field_validator("edge_threshold")
    @classmethod
    def _v_edge_thr(cls, v: Optional[float]) -> Optional[float]:
        if v is None:
            return v
        if not 0.0005 <= v <= 0.95:
            raise ValueError("edge_threshold debe estar entre 0.0005 y 0.95")
        return v

    @field_validator("kelly_fraction")
    @classmethod
    def _v_kelly(cls, v: Optional[float]) -> Optional[float]:
        if v is None:
            return v
        if not 0.01 <= v <= 1.0:
            raise ValueError("kelly_fraction debe estar entre 0.01 y 1.0")
        return v

    @field_validator("edge_net_min")
    @classmethod
    def _v_enm(cls, v: Optional[float]) -> Optional[float]:
        if v is None:
            return v
        if not 0.0 <= v <= 0.5:
            raise ValueError("edge_net_min debe estar entre 0.0 y 0.5")
        return v


def _repo_root_from_router() -> Path:
    return Path(__file__).resolve().parents[3]


def _env_file_paths() -> list[Path]:
    repo_env = _repo_root_from_router() / ".env"
    global_env = Path(r"C:\ATLAS\config\.env")
    out = [repo_env]
    if global_env != repo_env:
        out.append(global_env)
    return out


def _mask_key(value: str) -> str:
    v = (value or "").strip()
    if not v:
        return ""
    if len(v) <= 8:
        return "****"
    return f"{v[:4]}...{v[-4:]}"


def _persist_env_updates(env_file: Path, updates: dict[str, str]) -> None:
    """Aplica reemplazo/alta de variables en un .env sin borrar comentarios."""
    env_file.parent.mkdir(parents=True, exist_ok=True)
    lines = env_file.read_text(encoding="utf-8").splitlines() \
        if env_file.exists() else []
    index_by_key: dict[str, int] = {}
    for idx, line in enumerate(lines):
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or "=" not in stripped:
            continue
        key = stripped.split("=", 1)[0].strip()
        index_by_key[key] = idx
    for key, value in updates.items():
        new_line = f"{key}={value}"
        if key in index_by_key:
            lines[index_by_key[key]] = new_line
        else:
            lines.append(new_line)
    env_file.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _runtime_snapshot(state: RadarState) -> dict[str, Any]:
    orch = getattr(state, "orchestrator", None)
    executor = getattr(orch, "executor", None) if orch else None
    exec_cfg = getattr(executor, "cfg", None) if executor else None
    live_enabled = bool(getattr(exec_cfg, "enable_live", False))
    paper_balance_cents = int(
        getattr(executor, "_paper_balance_cents", max(100, state.balance_cents))
    )
    key_id = (state.settings.kalshi_api_key_id or "").strip()
    key_path = Path(state.settings.kalshi_private_key_path)
    key_exists = bool(key_id)
    key_path_exists = key_path.expanduser().exists()
    api_ready = key_exists and key_path_exists
    scanner = getattr(orch, "scanner", None) if orch else None
    public_feed = bool(
        getattr(scanner, "uses_public_rest_feed", False)
    )
    market_data_ok = api_ready or public_feed
    s_or_orch = getattr(orch, "settings", state.settings) if orch else state.settings
    polymarket_enabled = bool(getattr(s_or_orch, "polymarket_enabled", False))
    orders_are_virtual = not live_enabled
    if api_ready:
        api_hint = ""
    elif market_data_ok:
        api_hint = (
            "Datos de mercado vía API pública Kalshi (REST, sin firma). "
            "Opcional: KALSHI_API_KEY + PEM para WebSocket firmado y órdenes live."
        )
    else:
        api_hint = (
            "Falta KALSHI_API_KEY (UUID) y/o el archivo PEM en KALSHI_PRIVATE_KEY_PATH, "
            "y el feed público REST está desactivado (RADAR_DISABLE_PUBLIC_MARKET_DATA=1). "
            "Copia .env.example → .env o configura la Bóveda y reinicia :8791."
        )
    return {
        "environment": state.settings.kalshi_environment,
        "execution_mode": "live" if live_enabled else "paper",
        "live_enabled": live_enabled,
        "orders_are_virtual": orders_are_virtual,
        "paper_balance_cents": paper_balance_cents,
        "paper_balance_usd": paper_balance_cents / 100.0,
        "api_key_masked": _mask_key(key_id),
        "api_key_present": key_exists,
        "private_key_path": str(key_path),
        "private_key_exists": key_path_exists,
        "api_ready": api_ready,
        "market_data_ok": market_data_ok,
        "kalshi_public_feed": public_feed,
        "api_hint": api_hint,
        "polymarket_enabled": polymarket_enabled,
        "data_feeds": {
            "kalshi": {
                "api_ready": api_ready,
                "market_data_ok": market_data_ok,
                "public_rest_feed": public_feed,
                "environment": state.settings.kalshi_environment,
            },
            "polymarket": {
                "enabled": polymarket_enabled,
            },
        },
        "radar_ui_build": RADAR_UI_BUILD,
        "edge_threshold": state.settings.edge_threshold,
        "kelly_fraction": state.settings.kelly_fraction,
        "edge_net_min": (
            float(orch.gating.cfg.edge_net_min)
            if orch is not None and getattr(orch, "gating", None) is not None
            else float(os.getenv("RADAR_EDGE_NET_MIN", "0.03"))
        ),
    }


async def _ensure_orchestrator_started(state: RadarState) -> None:
    """Arranca el orquestador si está montado pero aún no corriendo."""
    if not bool(getattr(state, "runtime_enabled", True)):
        return
    orch = getattr(state, "orchestrator", None)
    if orch is None:
        return
    task = getattr(state, "orchestrator_task", None)
    if task is not None and not task.done():
        return
    state.orchestrator_task = asyncio.create_task(
        orch.start(), name="atlas-radar-kalshi-orch-runtime"
    )


# ===========================================================================
# Router builder
# ===========================================================================
def build_router(state: Optional[RadarState] = None) -> APIRouter:
    """Crea un APIRouter listo para `app.include_router(router)`."""
    state = state or RadarState()
    router = APIRouter(tags=["atlas_radar_kalshi"])

    # ---------------- API REST ----------------
    _no_store = {"Cache-Control": "no-store, must-revalidate, max-age=0"}

    @router.get("/api/radar/status")
    async def status() -> JSONResponse:
        await _ensure_orchestrator_started(state)
        runtime = _runtime_snapshot(state)
        payload = {
            "ok": True,
            "module": "atlas_radar_kalshi",
            "environment": runtime["environment"],
            "execution_mode": runtime["execution_mode"],
            "live_enabled": runtime["live_enabled"],
            "orders_are_virtual": runtime["orders_are_virtual"],
            "data_feeds": runtime["data_feeds"],
            "polymarket_enabled": runtime["polymarket_enabled"],
            "paper_balance_usd": runtime["paper_balance_usd"],
            "base_url": state.settings.base_url,
            "ws_url": state.settings.ws_url,
            "ollama": state.settings.ollama_endpoint,
            "balance_usd": state.balance_cents / 100.0,
            "markets_tracked": len(state.markets),
            "decisions_logged": len(state.decisions),
            "orders_logged": len(state.orders),
            "queue_depth": int(getattr(state, "queue_depth", 0)),
            "queue_dropped": int(getattr(state, "queue_dropped", 0)),
            "edge_threshold": runtime["edge_threshold"],
            "kelly_fraction": runtime["kelly_fraction"],
            "edge_net_min": runtime["edge_net_min"],
            "api_ready": runtime["api_ready"],
            "market_data_ok": runtime["market_data_ok"],
            "kalshi_public_feed": runtime["kalshi_public_feed"],
            "api_hint": runtime["api_hint"],
            "api_key_masked": runtime["api_key_masked"],
            "private_key_path": runtime["private_key_path"],
            "private_key_exists": runtime["private_key_exists"],
            "radar_ui_build": runtime["radar_ui_build"],
        }
        return JSONResponse(content=payload, headers=_no_store)

    @router.get("/api/radar/config")
    async def config_get() -> JSONResponse:
        await _ensure_orchestrator_started(state)
        return JSONResponse(
            content={"ok": True, **_runtime_snapshot(state)},
            headers=_no_store,
        )

    @router.post("/api/radar/config")
    async def config_set(payload: RadarRuntimeConfigBody) -> dict:
        await _ensure_orchestrator_started(state)
        environment = payload.environment
        execution_mode = payload.execution_mode
        paper_balance_cents = int(round(payload.paper_balance_usd * 100))
        api_key_id = (
            payload.api_key_id.strip()
            if payload.api_key_id is not None
            else (state.settings.kalshi_api_key_id or "").strip()
        )
        private_key_path = Path(
            payload.private_key_path.strip()
            if payload.private_key_path
            else str(state.settings.kalshi_private_key_path)
        ).expanduser()

        if execution_mode == "live":
            if not api_key_id:
                raise HTTPException(
                    status_code=400,
                    detail="Modo live requiere API key configurada.",
                )
            if not private_key_path.exists():
                raise HTTPException(
                    status_code=400,
                    detail=(
                        "Modo live requiere private key PEM existente en: "
                        f"{private_key_path}"
                    ),
                )

        # Aplica al estado base
        state.settings.kalshi_environment = environment
        state.settings.kalshi_api_key_id = api_key_id
        state.settings.kalshi_private_key_path = private_key_path
        if execution_mode == "paper":
            state.balance_cents = max(100, paper_balance_cents)

        # Aplica al orquestador en runtime (si existe)
        orch = getattr(state, "orchestrator", None)
        if orch is not None:
            orch.settings.kalshi_environment = environment
            orch.settings.kalshi_api_key_id = api_key_id
            orch.settings.kalshi_private_key_path = private_key_path

            orch.executor.settings.kalshi_environment = environment
            orch.executor.settings.kalshi_api_key_id = api_key_id
            orch.executor.settings.kalshi_private_key_path = private_key_path
            orch.executor.cfg.enable_live = execution_mode == "live"
            orch.executor._signer = None
            applied_paper_cents = orch.executor.set_paper_balance(
                paper_balance_cents
            )

            orch.scanner.settings.kalshi_environment = environment
            orch.scanner.settings.kalshi_api_key_id = api_key_id
            orch.scanner.settings.kalshi_private_key_path = private_key_path
            orch.scanner._signer = None
            try:
                orch.scanner.refresh_feed_mode()
            except Exception:
                pass
            try:
                await orch.scanner.reconnect()
            except Exception:
                pass

            if execution_mode == "paper":
                orch.risk.update_balance(applied_paper_cents)
                state.balance_cents = orch.risk.state.balance_cents

        # Umbrales opcionales: brain (settings) + gating (edge neto) + risk (Kelly)
        if payload.edge_threshold is not None:
            state.settings.edge_threshold = payload.edge_threshold
        if payload.kelly_fraction is not None:
            state.settings.kelly_fraction = payload.kelly_fraction
            if orch is not None:
                orch.risk.limits = orch.risk.limits.model_copy(
                    update={"kelly_fraction": payload.kelly_fraction}
                )
        if payload.edge_net_min is not None:
            if orch is not None:
                orch.gating.cfg = orch.gating.cfg.model_copy(
                    update={"edge_net_min": payload.edge_net_min}
                )

        if payload.persist_env:
            updates = {
                "ATLAS_ENABLE_RADAR_KALSHI": "1",
                "KALSHI_ENV": environment,
                "ATLAS_RADAR_EXECUTION_MODE": execution_mode,
                "ATLAS_RADAR_LIVE": "1" if execution_mode == "live" else "0",
                "RADAR_PAPER_BALANCE_CENTS": str(max(100, paper_balance_cents)),
                "KALSHI_API_KEY_ID": api_key_id,
                # Alias de compatibilidad usado en config.py
                "KALSHI_API_KEY": api_key_id,
                "KALSHI_PRIVATE_KEY_PATH": str(private_key_path),
            }
            if payload.edge_threshold is not None:
                updates["RADAR_EDGE_THRESHOLD"] = str(payload.edge_threshold)
            if payload.kelly_fraction is not None:
                updates["RADAR_KELLY_FRACTION"] = str(payload.kelly_fraction)
            if payload.edge_net_min is not None:
                updates["RADAR_EDGE_NET_MIN"] = str(payload.edge_net_min)
            try:
                for env_path in _env_file_paths():
                    _persist_env_updates(env_path, updates)
            except Exception as exc:
                raise HTTPException(
                    status_code=500,
                    detail=f"No se pudo persistir .env: {exc}",
                ) from exc

        return {
            "ok": True,
            "message": "Configuración de radar aplicada.",
            "applied": _runtime_snapshot(state),
            "persisted_env": payload.persist_env,
        }

    @router.get("/api/radar/markets")
    def markets() -> dict:
        rows = []
        for t, m in state.markets.items():
            rows.append({"ticker": t, **m})
        rows.sort(key=lambda r: abs(r.get("edge", 0.0)), reverse=True)
        return {"ok": True, "rows": rows[:200]}

    @router.get("/api/radar/decisions")
    def decisions(limit: int = 100) -> dict:
        items = list(state.decisions)[-limit:]
        return {"ok": True, "items": [d.model_dump() for d in items]}

    @router.get("/api/radar/orders")
    def orders(limit: int = 100) -> dict:
        return {"ok": True, "items": list(state.orders)[-limit:]}

    @router.get("/api/radar/metrics")
    def metrics() -> dict:
        edges = [d.edge for d in state.decisions]
        actionable = sum(1 for d in state.decisions if d.side)
        # performance from journal
        journal = Journal(state.settings.log_dir)
        perf = compute_perf(journal)
        exec_metrics = dict(getattr(state, "exec_metrics", {}) or {})
        risk_status = dict(getattr(state, "risk_status", {}) or {})
        risk_status.setdefault(
            "kill_switch", bool(getattr(state, "kill_requested", False))
        )
        risk_status.setdefault("safe_mode", False)
        h = getattr(state, "health", None)
        if h is not None:
            health_out = {
                "degraded": bool(getattr(h, "degraded", False)),
                "last_event_ts": float(getattr(h, "last_event_ts", 0.0)),
                "last_decision_ts": float(getattr(h, "last_decision_ts", 0.0)),
                "last_order_ts": float(getattr(h, "last_order_ts", 0.0)),
            }
        else:
            health_out = {
                "degraded": False,
                "last_event_ts": 0.0,
                "last_decision_ts": 0.0,
                "last_order_ts": 0.0,
            }
        session_out = {
            "decisions": len(state.decisions),
            "orders": len(state.orders),
            "queue_depth": int(getattr(state, "queue_depth", 0)),
            "queue_dropped": int(getattr(state, "queue_dropped", 0)),
        }
        return {
            "ok": True,
            "edges_avg": (sum(edges) / len(edges)) if edges else 0.0,
            "edges_max": max(edges) if edges else 0.0,
            "edges_min": min(edges) if edges else 0.0,
            "actionable_pct": (actionable / len(state.decisions))
            if state.decisions else 0.0,
            "orders_total": len(state.orders),
            "balance_usd": state.balance_cents / 100.0,
            "performance": perf.model_dump(),
            "execution": exec_metrics,
            "risk": risk_status,
            "health": health_out,
            "session": session_out,
        }

    @router.get("/api/radar/risk")
    def risk_endpoint() -> dict:
        risk_status = dict(getattr(state, "risk_status", {}) or {})
        risk_status.setdefault(
            "kill_switch", bool(getattr(state, "kill_requested", False))
        )
        risk_status.setdefault("safe_mode", False)
        return {"ok": True, "risk": risk_status}

    @router.get("/api/radar/health")
    def health() -> dict:
        h = getattr(state, "health", None)
        return {
            "ok": True,
            "degraded": bool(getattr(h, "degraded", False)),
            "last_event_ts": getattr(h, "last_event_ts", 0.0),
            "last_decision_ts": getattr(h, "last_decision_ts", 0.0),
            "last_order_ts": getattr(h, "last_order_ts", 0.0),
        }

    @router.get("/api/radar/prometheus")
    def prom() -> Any:
        from fastapi.responses import PlainTextResponse
        journal = Journal(state.settings.log_dir)
        perf = compute_perf(journal)
        text = prometheus_text(
            perf,
            getattr(state, "exec_metrics", {}) or {},
            getattr(state, "risk_status", {}) or {},
        )
        return PlainTextResponse(text, media_type="text/plain; version=0.0.4")

    @router.post("/api/radar/kill")
    def kill() -> dict:
        state.kill_requested = True
        risk_status = dict(getattr(state, "risk_status", {}) or {})
        risk_status["kill_switch"] = True
        state.risk_status = risk_status
        return {"ok": True, "kill_switch": True}

    @router.post("/api/radar/resume")
    def resume() -> dict:
        state.kill_requested = False
        risk_status = dict(getattr(state, "risk_status", {}) or {})
        risk_status["kill_switch"] = False
        state.risk_status = risk_status
        return {"ok": True, "kill_switch": False}

    # ---------------- WebSocket ----------------
    @router.websocket("/api/radar/stream")
    async def stream(ws: WebSocket) -> None:
        await _ensure_orchestrator_started(state)
        await ws.accept()
        state.subscribers.add(ws)
        try:
            await ws.send_json({"type": "hello",
                                "atlas_dashboard": state.settings.atlas_dashboard_url,
                                "module": "atlas_radar_kalshi"})
            while True:
                # heartbeat para mantener viva la conexión
                await asyncio.sleep(15)
                await ws.send_json({"type": "ping"})
        except WebSocketDisconnect:
            pass
        finally:
            state.subscribers.discard(ws)

    # ---------------- UI estática ----------------
    @router.get("/ui/radar", response_class=HTMLResponse)
    def ui() -> HTMLResponse:
        return HTMLResponse(
            content=_RADAR_HTML,
            headers={"Cache-Control": "no-store, must-revalidate, max-age=0"},
        )

    # ---------------- Cross-link ----------------
    @router.get("/api/radar/atlas-link")
    def atlas_link() -> dict:
        """Devuelve enlaces bidireccionales con el dashboard central."""
        return {
            "atlas_status": f"{state.settings.atlas_dashboard_url}/status",
            "atlas_modules": f"{state.settings.atlas_dashboard_url}/modules",
            "radar_ui": f"{state.settings.atlas_dashboard_url}/ui/radar",
        }

    return router


# ===========================================================================
# HTML embebido (SPA mínima, dark theme, sin dependencias externas
# salvo Chart.js por CDN). Pensada para ser servida desde :8791.
# ===========================================================================
_RADAR_HTML = r"""<!doctype html>
<html lang="es">
<head>
<meta charset="utf-8" />
<title>Atlas Radar — Kalshi</title>
<meta name="viewport" content="width=device-width,initial-scale=1" />
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.1/dist/chart.umd.min.js"></script>
<style>
  :root{
    --bg:#0b0f14; --bg2:#10161d; --line:#1d2733; --fg:#e6edf3;
    --muted:#8b97a8; --accent:#3fb950; --neg:#f85149; --warn:#d29922;
    --blue:#58a6ff;
  }
  .grid{grid-template-columns:repeat(4,1fr) !important}
  @media (min-width:1280px){.grid{grid-template-columns:repeat(8,1fr) !important}}
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--fg);
       font:14px/1.4 -apple-system,Segoe UI,Inter,Roboto,sans-serif}
  header{display:flex;align-items:center;justify-content:space-between;
         padding:14px 20px;background:var(--bg2);border-bottom:1px solid var(--line)}
  header h1{margin:0;font-size:16px;letter-spacing:.5px}
  header .pill{padding:3px 10px;border:1px solid var(--line);border-radius:999px;
               color:var(--muted);font-size:12px;margin-left:8px}
  header .pill.danger{color:var(--neg);border-color:rgba(248,81,73,.4)}
  header a{color:var(--blue);text-decoration:none;margin-left:14px;font-size:13px}
  .grid{display:grid;gap:14px;padding:14px;
        grid-template-columns:repeat(4,1fr)}
  .tile{background:var(--bg2);border:1px solid var(--line);border-radius:10px;padding:14px}
  .tile h3{margin:0 0 6px 0;font-size:11px;color:var(--muted);
          text-transform:uppercase;letter-spacing:.8px}
  .tile .v{font-size:22px;font-weight:600}
  .pos{color:var(--accent)} .neg{color:var(--neg)} .mut{color:var(--muted)}
  main{display:grid;grid-template-columns:2fr 1fr;gap:14px;padding:0 14px 14px}
  .card{background:var(--bg2);border:1px solid var(--line);border-radius:10px;
        padding:12px}
  table{width:100%;border-collapse:collapse;font-size:13px}
  th,td{padding:6px 8px;border-bottom:1px solid var(--line);text-align:right}
  th:first-child,td:first-child{text-align:left}
  th{color:var(--muted);font-weight:500}
  tr.act{background:rgba(63,185,80,.06)}
  #log{height:260px;overflow:auto;font:12px/1.45 ui-monospace,Consolas,monospace;
       background:#06090d;border:1px solid var(--line);border-radius:8px;padding:8px}
  #log .e{color:var(--muted)} #log .ok{color:var(--accent)}
  #log .ko{color:var(--neg)} #log .w{color:var(--warn)}
  .row{display:flex;gap:10px;align-items:center;justify-content:space-between;
       margin-bottom:8px}
  .badge{padding:2px 8px;border-radius:999px;font-size:11px;border:1px solid var(--line)}
  .badge.ok{color:var(--accent);border-color:rgba(63,185,80,.4)}
  .badge.ko{color:var(--neg);border-color:rgba(248,81,73,.4)}
  .ctrl{display:flex;flex-wrap:wrap;gap:8px;align-items:center}
  .ctrl label{font-size:12px;color:var(--muted)}
  .ctrl select,.ctrl input{
    background:#0f1620;border:1px solid var(--line);color:var(--fg);
    border-radius:6px;padding:6px 8px;font-size:12px
  }
  .ctrl input[readonly]{opacity:.85;min-width:320px}
  .btn{
    background:#0f2a44;border:1px solid #1f6fb2;color:#9cd1ff;
    border-radius:6px;padding:6px 10px;cursor:pointer
  }
  .msg-ok{color:var(--accent)} .msg-ko{color:var(--neg)}
  .exec-banner{padding:10px 20px;border-bottom:1px solid var(--line);font-size:13px;line-height:1.45}
  .exec-banner.paper{background:rgba(63,185,80,.1);border-color:rgba(63,185,80,.28)}
  .exec-banner.live{background:rgba(248,81,73,.12);border-color:rgba(248,81,73,.35);color:#f0c4c2}
  select#cfg_mode.mode-live{border-color:rgba(248,81,73,.7)!important;color:#ffb4b0}
  select#cfg_mode.mode-paper{border-color:rgba(63,185,80,.35)!important}
  .tile.sm h3{font-size:10px;letter-spacing:.5px}
  .tile.sm .v{font-size:17px}
  .tile .sub{font-size:11px;margin-top:4px}
  .tile .smtxt{font-size:12px;font-weight:500;line-height:1.3;word-break:break-word}
  @media (min-width:1200px){
    #rt_metrics_section{grid-template-columns:repeat(4,1fr) !important}
  }
  .threshold-box{
    margin-top:12px;padding:12px 14px;border-radius:8px;
    border:1px solid rgba(88,166,255,.4);
    background:linear-gradient(180deg, rgba(15,24,40,.95), rgba(8,12,20,.9));
    box-shadow:0 0 0 1px rgba(0,0,0,.2) inset;
  }
  .threshold-box .threshold-title{
    font-size:12px;font-weight:600;color:#9cd1ff;letter-spacing:.03em;
    margin-bottom:2px;
  }
  .threshold-box .mut-hint{font-size:11px;color:var(--muted);font-weight:400}
  .threshold-box .ctrl{gap:10px}
  .threshold-box input{min-width:4.5em}
</style>
</head>
<body>
<header>
  <div>
    <h1>Atlas Radar · Kalshi</h1>
    <span class="pill" id="env">demo</span>
    <span class="pill" id="mode">paper</span>
    <span class="pill" id="feed_pill" title="Cotizaciones / mercado">DATOS: …</span>
    <span class="pill" id="order_pill" title="Simulado vs real">ÓRDENES: …</span>
    <span class="pill" id="ws">WS: …</span>
    <span class="mut" id="radar_ui_build" style="font-size:11px;margin-left:6px" title="Build del panel; si no coincide tras pull, reinicia :8791">—</span>
  </div>
  <nav>
    <a href="/ui">Volver a 8791/ui</a>
    <a href="/status">/status</a>
    <a href="/modules">/modules</a>
  </nav>
</header>
<div id="exec_banner" class="exec-banner paper" role="status" aria-live="polite">
  Cargando estado (paper = datos reales + órdenes virtuales)…
</div>

<section class="grid" style="grid-template-columns:repeat(4,1fr)">
  <div class="tile" style="grid-column:span 4">
    <div class="row">
      <strong>Control operativo</strong>
      <span class="mut" id="api_state">API no validada</span>
    </div>
    <div class="ctrl">
      <label for="cfg_env">Entorno</label>
      <select id="cfg_env">
        <option value="demo">demo</option>
        <option value="prod">prod</option>
      </select>
      <label for="cfg_mode" title="Interruptor paper (virtual) / live (real)">Modo</label>
      <select id="cfg_mode" class="mode-paper" title="paper = saldo virtual; live = órdenes reales" onchange="updateModeSelectClass()">
        <option value="paper">paper (virtual)</option>
        <option value="live">live (real)</option>
      </select>
      <label for="cfg_paper">Paper USD</label>
      <input id="cfg_paper" type="number" min="1" step="1" value="1000" />
      <label for="cfg_pem">PEM</label>
      <input id="cfg_pem" type="text" readonly value="-" />
      <button class="btn" onclick="applyConfig()">Aplicar</button>
    </div>
    <div class="threshold-box" role="group" aria-label="Umbrales editables">
      <div class="threshold-title">Umbrales configurables
        <span class="mut-hint">· Aplicar = fila superior · abajo: edge <em>bruto</em> (brain) vs <em>neto</em> (tras costos, gating)</span>
      </div>
      <div class="ctrl" style="margin-top:6px;align-items:flex-end">
        <label for="cfg_edge_thr" title="Edge en puntos de probabilidad (p_model − p_mercado), antes de costos. Filtra la señal del brain.">Brain: edge bruto mín. %</label>
        <input id="cfg_edge_thr" type="number" min="0" max="95" step="0.1" value="5" inputmode="decimal" />
        <label for="cfg_edge_net" title="Edge neto estimado tras fees/slippage; el gating exige al menos esto para operar.">Gating: edge neto mín. %</label>
        <input id="cfg_edge_net" type="number" min="0" max="50" step="0.1" value="3" inputmode="decimal" />
        <label for="cfg_kelly" title="Fracción de Kelly (riesgo)">Kelly %</label>
        <input id="cfg_kelly" type="number" min="1" max="100" step="1" value="25" inputmode="numeric" />
      </div>
    </div>
    <div class="mut" id="cfg_msg" style="margin-top:8px"></div>
  </div>
</section>

<section class="grid">
  <div class="tile"><h3>PnL neto</h3><div class="v" id="pnl">$0.00</div></div>
  <div class="tile"><h3>Hit rate</h3><div class="v" id="hit">0.0%</div></div>
  <div class="tile"><h3>Profit factor</h3><div class="v" id="pf">0.00</div></div>
  <div class="tile"><h3>Drawdown</h3><div class="v neg" id="dd">$0.00</div></div>
  <div class="tile"><h3>Expectancy</h3><div class="v" id="exp">$0.00</div></div>
  <div class="tile"><h3>Latencia p95</h3><div class="v" id="p95">0 ms</div></div>
  <div class="tile"><h3>Fill ratio</h3><div class="v" id="fr">0.00%</div></div>
  <div class="tile"><h3>Exposición</h3><div class="v" id="exp_t">$0.00</div></div>
</section>

<section class="grid" id="rt_metrics_section" style="grid-template-columns:repeat(4,1fr)">
  <div class="tile sm"><h3>Salud scanner</h3><div class="v" id="hx_degraded">—</div>
    <div class="sub mut" id="hx_last">últ. evento —</div></div>
  <div class="tile sm"><h3>Cola eventos</h3><div class="v" id="q_line">0 / 0</div>
    <div class="sub mut">profund · descart.</div></div>
  <div class="tile sm"><h3>Edge min / max (sesión)</h3><div class="v" id="edge_mm">0 / 0</div></div>
  <div class="tile sm"><h3>Señales accionables</h3><div class="v" id="act_pct">0%</div></div>
  <div class="tile sm"><h3>Trades (diario)</h3><div class="v" id="j_trades">0</div>
    <div class="sub mut" id="j_wl">W0 · L0</div></div>
  <div class="tile sm"><h3>Latencia p50</h3><div class="v" id="p50">0 ms</div></div>
  <div class="tile sm"><h3>Slippage (ejec / libro)</h3><div class="v" id="slip_b">0 / 0 ¢</div></div>
  <div class="tile sm"><h3>Drawdown actual</h3><div class="v" id="dd_cur">$0.00</div></div>
  <div class="tile sm"><h3>Brain bruto / Kelly / neto gating <span class="mut" style="font-weight:400">(lectura)</span></h3><div class="v" id="cfg_thr" style="font-size:15px">—</div>
    <div class="sub mut" id="cfg_thr_hint">Valores en vivo; edita arriba (Umbrales) y Aplicar</div></div>
  <div class="tile sm" style="grid-column:span 2"><h3>Breakers (riesgo)</h3><div class="v smtxt" id="brk_line">—</div></div>
  <div class="tile sm"><h3>Última decisión / orden</h3><div class="v smtxt" id="age_do">—</div></div>
  <div class="tile sm"><h3>Exec intentos / errores</h3><div class="v" id="x_ae">0 / 0</div></div>
</section>

<section class="grid" style="grid-template-columns:repeat(4,1fr)">
  <div class="tile"><h3>Balance</h3><div class="v" id="bal">$0.00</div></div>
  <div class="tile"><h3>Mercados</h3><div class="v" id="mkt">0</div></div>
  <div class="tile"><h3>Edge medio</h3><div class="v" id="edge">0.00%</div></div>
  <div class="tile">
    <h3>Riesgo</h3>
    <div class="v" id="risk_v">healthy</div>
    <div style="margin-top:6px">
      <button onclick="kill()" style="background:#311;border:1px solid #6b1f1f;color:#f85149;padding:4px 8px;border-radius:6px;cursor:pointer">Kill</button>
      <button onclick="resume()" style="background:#0e1f12;border:1px solid #1f6b3a;color:#3fb950;padding:4px 8px;border-radius:6px;cursor:pointer;margin-left:6px">Resume</button>
    </div>
  </div>
</section>

<main>
  <section class="card">
    <div class="row"><strong>Mercados con mayor edge</strong>
      <span class="mut" id="updated">—</span></div>
    <table id="tbl"><thead><tr>
      <th>Ticker</th><th>p_mkt</th><th>p_model</th><th>Edge</th><th>Side</th>
      <th>Conf.</th><th>MC win</th></tr></thead><tbody></tbody></table>
  </section>

  <section class="card">
    <div class="row"><strong>Eventos en vivo</strong>
      <span class="badge ok" id="conn">conectando…</span></div>
    <div id="log"></div>
    <canvas id="chart" height="160"></canvas>
  </section>
</main>

<script>
const $ = q => document.querySelector(q);
const fmtPct = x => (100*x).toFixed(2)+'%';
const fmtUSD = x => '$'+x.toFixed(2);
const fmtPF = v => (typeof v==='number' && Number.isFinite(v)) ? v.toFixed(2) : (v>900?'∞':'—');
const fmtRel = ts => {
  const t = Number(ts) || 0;
  if(t<=0) return '—';
  const age = Date.now()/1000 - t;
  if(age<0) return '0s';
  if(age<60) return Math.floor(age)+'s';
  if(age<3600) return Math.floor(age/60)+'m';
  if(age<86400) return Math.floor(age/3600)+'h';
  return Math.floor(age/86400)+'d';
};
let _rtDebounce = null;
function scheduleRTRefresh(){
  if(_rtDebounce) clearTimeout(_rtDebounce);
  _rtDebounce = setTimeout(()=>{ _rtDebounce=null; refresh(); }, 280);
}
const log = (cls, msg) => {
  const el = $('#log'); const d = document.createElement('div');
  d.className = cls; d.textContent = '['+new Date().toLocaleTimeString()+'] '+msg;
  el.prepend(d);
};

function setCfgMsg(msg, ok=false){
  const el = $('#cfg_msg');
  if(!el) return;
  el.textContent = msg || '';
  el.className = ok ? 'msg-ok' : 'msg-ko';
}

function updateModeSelectClass(){
  const sel = $('#cfg_mode');
  if (!sel) return;
  sel.className = (sel.value === 'live') ? 'mode-live' : 'mode-paper';
}

function updateExecBanner(s){
  const ban = $('#exec_banner');
  if (!ban) return;
  const paper = (s.execution_mode || 'paper') === 'paper';
  const k = (s.data_feeds && s.data_feeds.kalshi) || {};
  const p = (s.data_feeds && s.data_feeds.polymarket) || {};
  const credOk = k.api_ready === true;
  const marketOk = k.market_data_ok === true;
  const pub = k.public_rest_feed === true;
  const poly = p.enabled ? 'Polymarket encendido' : 'Polymarket apagado (sólo Kalshi si aplica)';
  const ks = credOk
    ? ('API Kalshi firmada (WS + REST en ' + (k.environment || 'demo') + ')')
    : (marketOk
        ? ('Datos reales Kalshi vía REST público' + (pub ? ' (sin key/PEM)' : '') + ' en ' + (k.environment || 'demo'))
        : 'Sin datos de mercado: configura key+PEM o deja activo el feed REST público (por defecto en paper).');
  ban.className = 'exec-banner ' + (paper ? 'paper' : 'live');
  if (paper) {
    ban.textContent = 'Fase validación: ' + ks + ' · ' + poly + ' · órdenes PAPER = dinero virtual (' + fmtUSD(s.paper_balance_usd || 0) + '). Cuando quieras riesgo real, cambia Modo a live (doble confirmación).';
  } else {
    ban.textContent = 'LIVE: órdenes reales en plataformas. Revisa exposición, límites y kill switch; volver a paper en Modo desactiva envío real.';
  }
  const fp = $('#feed_pill');
  if (fp) {
    fp.textContent = marketOk ? 'DATOS: reales' : 'DATOS: incompleto';
    fp.title = ks;
  }
  const op = $('#order_pill');
  if (op) {
    op.textContent = paper ? 'ÓRDENES: virtual' : 'ÓRDENES: REAL';
    op.className = paper ? 'pill' : 'pill danger';
    op.title = paper ? 'No se envían órdenes a matching con dinero real' : 'Se envían órdenes reales';
  }
}

async function loadConfig(){
  try{
    const res = await fetch('/api/radar/config');
    const cfg = await res.json();
    if(!res.ok || !cfg.ok){ throw new Error(cfg.detail || 'No se pudo leer config'); }
    $('#cfg_env').value = cfg.environment || 'demo';
    $('#cfg_mode').value = cfg.execution_mode || 'paper';
    $('#cfg_paper').value = Math.max(1, Math.round(cfg.paper_balance_usd || 0));
    $('#cfg_pem').value = cfg.private_key_path || '-';
    $('#mode').textContent = cfg.execution_mode || 'paper';
    const credOk = !!cfg.api_ready;
    const marketOk = cfg.market_data_ok === true;
    const apiHint = (cfg.api_hint || '').trim();
    $('#api_state').textContent = credOk
      ? `API firmada · key ${cfg.api_key_masked || '***'}`
      : (marketOk
          ? 'Mercado OK (REST público). Key+PEM opcionales en paper.'
          : (apiHint || 'Sin datos de mercado (key/PEM o feed público).'));
    $('#api_state').setAttribute('title', credOk ? 'Kalshi firmado listo' : apiHint);
    $('#api_state').className = (credOk || marketOk) ? 'msg-ok' : 'msg-ko';
    const ub = $('#radar_ui_build');
    if (ub) ub.textContent = cfg.radar_ui_build || '—';
    if ($('#cfg_edge_thr')) {
      const et = (cfg.edge_threshold != null) ? Number(cfg.edge_threshold) : 0.05;
      const en = (cfg.edge_net_min != null) ? Number(cfg.edge_net_min) : 0.03;
      const kf = (cfg.kelly_fraction != null) ? Number(cfg.kelly_fraction) : 0.25;
      $('#cfg_edge_thr').value = (et * 100).toFixed(2);
      $('#cfg_edge_net').value = (en * 100).toFixed(2);
      $('#cfg_kelly').value = String(Math.round(kf * 100));
    }
    updateModeSelectClass();
    updateExecBanner(cfg);
  }catch(err){
    setCfgMsg('Error leyendo configuración: '+err, false);
  }
}

async function applyConfig(){
  if ($('#cfg_mode').value === 'live') {
    if (!window.confirm('LIVE: se enviarán órdenes reales a Kalshi (y Polymarket si está activo) con riesgo de capital. Requiere API key + PEM. ¿Seguir?')) {
      return;
    }
    if (!window.confirm('Confirmación final: activar envío de órdenes LIVE ahora.')) {
      return;
    }
  }
  const payload = {
    environment: $('#cfg_env').value,
    execution_mode: $('#cfg_mode').value,
    paper_balance_usd: Number($('#cfg_paper').value || 0),
    private_key_path: $('#cfg_pem').value || null,
    edge_threshold: Number($('#cfg_edge_thr').value || 5) / 100,
    edge_net_min: Number($('#cfg_edge_net').value || 3) / 100,
    kelly_fraction: Number($('#cfg_kelly').value || 25) / 100,
    persist_env: true,
  };
  setCfgMsg('Aplicando configuración...', true);
  try{
    const res = await fetch('/api/radar/config', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify(payload),
    });
    const data = await res.json();
    if(!res.ok || !data.ok){
      throw new Error((data && data.detail) ? data.detail : 'No se pudo aplicar');
    }
    setCfgMsg('Configuración aplicada correctamente.', true);
    await loadConfig();
    await refresh();
  }catch(err){
    setCfgMsg('No se pudo aplicar configuración: '+err, false);
  }
}

async function kill(){await fetch('/api/radar/kill',{method:'POST'}); refresh();}
async function resume(){await fetch('/api/radar/resume',{method:'POST'}); refresh();}

async function refresh(){
  try {
    const [s, m, mk] = await Promise.all([
      fetch('/api/radar/status').then(r => r.json()),
      fetch('/api/radar/metrics').then(r => r.json()),
      fetch('/api/radar/markets').then(r => r.json()),
    ]);
    $('#env').textContent = s.environment;
    $('#mode').textContent = s.execution_mode || 'paper';
    updateModeSelectClass();
    updateExecBanner(s);
    $('#bal').textContent = fmtUSD(s.balance_usd||0);
    $('#mkt').textContent = s.markets_tracked;
    const e = m.edges_avg||0;
    $('#edge').textContent = fmtPct(e);
    $('#edge').className = 'v '+(e>=0?'pos':'neg');
    const p = m.performance||{}; const x = m.execution||{}; const r = m.risk||{};
    const hh = m.health||{}; const ses = m.session||{};
    $('#pnl').textContent = fmtUSD((p.pnl_net_cents||0)/100);
    $('#pnl').className = 'v '+((p.pnl_net_cents||0)>=0?'pos':'neg');
    $('#hit').textContent = fmtPct(p.hit_rate||0);
    $('#pf').textContent = fmtPF(p.profit_factor);
    $('#dd').textContent = fmtUSD(-(p.max_drawdown_cents||0)/100);
    $('#exp').textContent = fmtUSD((p.expectancy_cents||0)/100);
    $('#p95').textContent = (x.p95_ms||0).toFixed(0)+' ms';
    $('#fr').textContent = fmtPct(x.fill_ratio||0);
    $('#exp_t').textContent = fmtUSD((r.exposure_cents||0)/100);
    const breakers = (r.breakers||[]);
    $('#risk_v').textContent = r.kill_switch?'KILL':(r.safe_mode?'SAFE':'healthy');
    $('#risk_v').className = 'v '+(r.kill_switch||r.safe_mode?'neg':'pos');
    if($('#hx_degraded')){
      const deg = !!hh.degraded;
      $('#hx_degraded').textContent = deg ? 'DEGRADED' : 'OK';
      $('#hx_degraded').className = 'v ' + (deg ? 'neg' : 'pos');
      const hel = $('#hx_last');
      if(hel) hel.textContent = 'últ. evento hace ' + fmtRel(hh.last_event_ts);
    }
    if($('#q_line')) $('#q_line').textContent = (ses.queue_depth??0) + ' / ' + (ses.queue_dropped??0);
    if($('#edge_mm')){
      const emi = m.edges_min ?? 0, ema = m.edges_max ?? 0;
      $('#edge_mm').textContent = (100*emi).toFixed(2) + ' / ' + (100*ema).toFixed(2) + ' pts';
    }
    if($('#act_pct')) $('#act_pct').textContent = fmtPct(m.actionable_pct ?? 0);
    if($('#j_trades')) $('#j_trades').textContent = String(p.trades ?? 0);
    if($('#j_wl')) $('#j_wl').textContent = 'W' + (p.wins??0) + ' · L' + (p.losses??0);
    if($('#p50')) $('#p50').textContent = (x.p50_ms ?? 0).toFixed(0) + ' ms';
    if($('#slip_b')) $('#slip_b').textContent = (x.avg_slippage_cents ?? 0).toFixed(1) + ' / ' + (p.avg_slippage_cents ?? 0).toFixed(1) + ' ¢';
    if($('#dd_cur')){ const cd = p.current_drawdown_cents ?? 0; const el=$('#dd_cur'); el.textContent=fmtUSD(-cd/100); el.className='v '+(cd>0?'neg':'pos'); }
    if($('#cfg_thr')) {
      const en = s.edge_net_min;
      const enT = (en != null && Number.isFinite(Number(en))) ? fmtPct(Number(en)) : '—';
      $('#cfg_thr').textContent = fmtPct(s.edge_threshold ?? 0) + ' / ' + fmtPct(s.kelly_fraction ?? 0) + ' / net≥' + enT;
    }
    if($('#brk_line')) $('#brk_line').textContent = (breakers.length ? breakers.slice(-4).join(' · ') : 'ninguno');
    if($('#age_do')) $('#age_do').textContent = 'dec ' + fmtRel(hh.last_decision_ts) + ' · ord ' + fmtRel(hh.last_order_ts);
    if($('#x_ae')) $('#x_ae').textContent = (x.attempts??0) + ' / ' + (x.errors??0);
    const tb = $('#tbl tbody'); tb.innerHTML='';
    (mk.rows||[]).slice(0,40).forEach(row=>{
      const tr = document.createElement('tr');
      if(row.side) tr.className='act';
      tr.innerHTML = `<td>${row.ticker}</td>
        <td>${(row.p_market??0).toFixed(3)}</td>
        <td>${(row.p_model??0).toFixed(3)}</td>
        <td class="${(row.edge||0)>=0?'pos':'neg'}">${(row.edge??0).toFixed(3)}</td>
        <td>${row.side??''}</td>
        <td>${(row.confidence??0).toFixed(2)}</td>
        <td>${(row.mc_winrate??0).toFixed(2)}</td>`;
      tb.appendChild(tr);
    });
    $('#updated').textContent = new Date().toLocaleTimeString();
    const ub = $('#radar_ui_build');
    if (ub && s.radar_ui_build) ub.textContent = s.radar_ui_build;
  } catch(e){ log('ko', 'refresh failed: '+e); }
}

const ctx = document.getElementById('chart');
const chart = new Chart(ctx, {
  type:'line',
  data:{labels:[],datasets:[{label:'edge medio',data:[],
        borderColor:'#58a6ff',backgroundColor:'rgba(88,166,255,.15)',
        fill:true,tension:.3,pointRadius:0}]},
  options:{plugins:{legend:{display:false}},scales:{
    x:{ticks:{color:'#8b97a8'},grid:{color:'#1d2733'}},
    y:{ticks:{color:'#8b97a8'},grid:{color:'#1d2733'}}}}
});

function connectWS(){
  const proto = location.protocol==='https:'?'wss':'ws';
  const ws = new WebSocket(`${proto}://${location.host}/api/radar/stream`);
  ws.onopen = ()=>{$('#conn').textContent='live'; $('#conn').className='badge ok';
                  $('#ws').textContent='WS: live'; log('ok','WS conectado')};
  ws.onclose = ()=>{$('#conn').textContent='offline'; $('#conn').className='badge ko';
                   setTimeout(connectWS,2000)};
  ws.onmessage = ev =>{
    try{
      const m = JSON.parse(ev.data);
      if(m.type==='ping'){
        if(typeof m.degraded==='boolean'){
          const he = $('#hx_degraded');
          if(he){
            he.textContent = m.degraded ? 'DEGRADED' : 'OK';
            he.className = 'v ' + (m.degraded ? 'neg' : 'pos');
          }
        }
        return;
      }
      if(m.type==='decision'){
        log(m.side?'ok':'e', `${m.ticker} edge=${(+m.edge).toFixed(3)} side=${m.side??'-'}`);
        chart.data.labels.push(new Date().toLocaleTimeString());
        chart.data.datasets[0].data.push(+m.edge);
        if(chart.data.labels.length>60){chart.data.labels.shift();
          chart.data.datasets[0].data.shift()}
        chart.update();
        scheduleRTRefresh();
      } else if(m.type==='order'){
        log(m.ok?'ok':'ko', `ORDER ${m.ticker} ${m.side} ${m.contracts}@${m.price} -> ${m.status}`);
        scheduleRTRefresh();
      } else if(m.type==='error'){ log('ko', 'ERR: '+m.error); }
    }catch(e){}
  };
}

loadConfig(); refresh();
setInterval(refresh, 5000);
connectWS();
</script>
</body></html>
"""
