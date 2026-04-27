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
from collections import deque
from pathlib import Path
from typing import Any, Deque, Optional

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse
from pydantic import BaseModel

from ..config import RadarSettings, get_settings
from ..brain import BrainDecision
from ..executor import OrderResult
from ..risk import PositionSize


# ===========================================================================
# Estado en memoria compartido entre main.py y el router
# ===========================================================================
class RadarState:
    """Estado in-memory expuesto al dashboard."""

    def __init__(self, settings: Optional[RadarSettings] = None,
                 maxlen: int = 500) -> None:
        self.settings = settings or get_settings()
        self.markets: dict[str, dict] = {}       # ticker -> última snapshot
        self.decisions: Deque[BrainDecision] = deque(maxlen=maxlen)
        self.orders: Deque[dict] = deque(maxlen=maxlen)
        self.events_log: Deque[str] = deque(maxlen=2000)
        self.balance_cents: int = 0
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


# ===========================================================================
# Router builder
# ===========================================================================
def build_router(state: Optional[RadarState] = None) -> APIRouter:
    """Crea un APIRouter listo para `app.include_router(router)`."""
    state = state or RadarState()
    router = APIRouter(tags=["atlas_radar_kalshi"])

    # ---------------- API REST ----------------
    @router.get("/api/radar/status")
    def status() -> dict:
        return {
            "ok": True,
            "module": "atlas_radar_kalshi",
            "environment": state.settings.kalshi_environment,
            "base_url": state.settings.base_url,
            "ws_url": state.settings.ws_url,
            "ollama": state.settings.ollama_endpoint,
            "balance_usd": state.balance_cents / 100.0,
            "markets_tracked": len(state.markets),
            "decisions_logged": len(state.decisions),
            "orders_logged": len(state.orders),
            "edge_threshold": state.settings.edge_threshold,
            "kelly_fraction": state.settings.kelly_fraction,
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
        if not state.decisions:
            return {"ok": True, "edges_avg": 0.0, "actionable_pct": 0.0,
                    "orders_total": len(state.orders)}
        edges = [d.edge for d in state.decisions]
        actionable = sum(1 for d in state.decisions if d.side)
        return {
            "ok": True,
            "edges_avg": sum(edges) / len(edges),
            "edges_max": max(edges),
            "edges_min": min(edges),
            "actionable_pct": actionable / len(state.decisions),
            "orders_total": len(state.orders),
            "balance_usd": state.balance_cents / 100.0,
        }

    # ---------------- WebSocket ----------------
    @router.websocket("/api/radar/stream")
    async def stream(ws: WebSocket) -> None:
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
    def ui() -> str:
        return _RADAR_HTML

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
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--fg);
       font:14px/1.4 -apple-system,Segoe UI,Inter,Roboto,sans-serif}
  header{display:flex;align-items:center;justify-content:space-between;
         padding:14px 20px;background:var(--bg2);border-bottom:1px solid var(--line)}
  header h1{margin:0;font-size:16px;letter-spacing:.5px}
  header .pill{padding:3px 10px;border:1px solid var(--line);border-radius:999px;
               color:var(--muted);font-size:12px;margin-left:8px}
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
</style>
</head>
<body>
<header>
  <div>
    <h1>Atlas Radar · Kalshi</h1>
    <span class="pill" id="env">demo</span>
    <span class="pill" id="ws">WS: …</span>
  </div>
  <nav>
    <a href="/" >Atlas Core</a>
    <a href="/status">/status</a>
    <a href="/modules">/modules</a>
  </nav>
</header>

<section class="grid">
  <div class="tile"><h3>Balance</h3><div class="v" id="bal">$0.00</div></div>
  <div class="tile"><h3>Mercados</h3><div class="v" id="mkt">0</div></div>
  <div class="tile"><h3>Edge medio</h3><div class="v" id="edge">0.00%</div></div>
  <div class="tile"><h3>Órdenes</h3><div class="v" id="ord">0</div></div>
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
const log = (cls, msg) => {
  const el = $('#log'); const d = document.createElement('div');
  d.className = cls; d.textContent = '['+new Date().toLocaleTimeString()+'] '+msg;
  el.prepend(d);
};

async function refresh(){
  try {
    const s = await (await fetch('/api/radar/status')).json();
    $('#env').textContent = s.environment;
    $('#bal').textContent = fmtUSD(s.balance_usd||0);
    $('#mkt').textContent = s.markets_tracked;
    $('#ord').textContent = s.orders_logged;
    const m = await (await fetch('/api/radar/metrics')).json();
    const e = m.edges_avg||0;
    $('#edge').textContent = fmtPct(e);
    $('#edge').className = 'v '+(e>=0?'pos':'neg');
    const r = await (await fetch('/api/radar/markets')).json();
    const tb = $('#tbl tbody'); tb.innerHTML='';
    r.rows.slice(0,40).forEach(row=>{
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
      if(m.type==='ping') return;
      if(m.type==='decision'){
        log(m.side?'ok':'e', `${m.ticker} edge=${(+m.edge).toFixed(3)} side=${m.side??'-'}`);
        chart.data.labels.push(new Date().toLocaleTimeString());
        chart.data.datasets[0].data.push(+m.edge);
        if(chart.data.labels.length>60){chart.data.labels.shift();
          chart.data.datasets[0].data.shift()}
        chart.update();
      } else if(m.type==='order'){
        log(m.ok?'ok':'ko', `ORDER ${m.ticker} ${m.side} ${m.contracts}@${m.price} -> ${m.status}`);
      } else if(m.type==='error'){ log('ko', 'ERR: '+m.error); }
    }catch(e){}
  };
}

refresh(); setInterval(refresh, 5000); connectWS();
</script>
</body></html>
"""
