/**
 * ATLAS v4.3 — Atlas Code-Quant
 * Shortcut y panel de estado del sistema de trading algorítmico.
 * API interna en puerto 8792.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID  = 'atlas-quant-module';
const MONITOR_POLL_ID = 'atlas-quant-monitor';
const QUANT_API = 'http://127.0.0.1:8792';
const API_KEY   = 'atlas-quant-local';

function _esc(s) {
  const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML;
}

function _headers() {
  return { 'X-Api-Key': API_KEY, 'Content-Type': 'application/json' };
}

function _fmtMoney(v) {
  const n = Number(v || 0);
  const sign = n > 0 ? '+' : '';
  return `${sign}$${n.toFixed(2)}`;
}

function _fmtPct(v, digits = 1) {
  if (v === null || v === undefined || Number.isNaN(Number(v))) return '—';
  return `${Number(v).toFixed(digits)}%`;
}

function _fmtNum(v, digits = 2) {
  if (v === null || v === undefined || Number.isNaN(Number(v))) return '—';
  return Number(v).toFixed(digits);
}

function _moneyColor(v) {
  const n = Number(v || 0);
  return n >= 0 ? 'var(--accent-green)' : 'var(--accent-red)';
}

function _monitorParams(container) {
  const scope = container.querySelector('#aq-account-scope')?.value || 'paper';
  const accountId = container.querySelector('#aq-account-id')?.value?.trim() || '';
  const params = new URLSearchParams();
  if (scope) params.set('account_scope', scope);
  if (accountId) params.set('account_id', accountId);
  return params.toString();
}

function _curveSvg(points = []) {
  if (!Array.isArray(points) || points.length === 0) return '';
  const width = 220;
  const height = 84;
  const prices = points.map(p => Number(p.price || 0));
  const profits = points.map(p => Number(p.profit || 0));
  const minX = Math.min(...prices);
  const maxX = Math.max(...prices);
  const minY = Math.min(...profits);
  const maxY = Math.max(...profits);
  const rangeX = Math.max(maxX - minX, 1e-6);
  const rangeY = Math.max(maxY - minY, 1e-6);
  const coords = points.map(p => {
    const x = ((Number(p.price || 0) - minX) / rangeX) * width;
    const y = height - (((Number(p.profit || 0) - minY) / rangeY) * height);
    return `${x.toFixed(1)},${y.toFixed(1)}`;
  }).join(' ');
  const zeroY = height - (((0 - minY) / rangeY) * height);
  return `
    <svg viewBox="0 0 ${width} ${height}" width="100%" height="84" style="display:block">
      <line x1="0" y1="${zeroY.toFixed(1)}" x2="${width}" y2="${zeroY.toFixed(1)}" stroke="rgba(255,255,255,0.12)" stroke-dasharray="4 4"></line>
      <polyline fill="none" stroke="var(--accent)" stroke-width="2" points="${coords}"></polyline>
    </svg>
  `;
}

async function _fetchHealth(container) {
  const dot  = container.querySelector('#aq-dot');
  const stat = container.querySelector('#aq-status');
  const up   = container.querySelector('#aq-uptime');
  const strat = container.querySelector('#aq-strategies');
  const pos  = container.querySelector('#aq-positions');
  try {
    const r = await fetch(`${QUANT_API}/health`, { headers: _headers() });
    const d = await r.json();
    if (dot) dot.className = 'provider-dot ok';
    if (stat) { stat.textContent = 'ONLINE'; stat.style.color = 'var(--accent-green)'; }
    if (up)    up.textContent   = `${Math.floor((d.uptime_sec||0)/60)} min`;
    if (strat) strat.textContent = (d.active_strategies||[]).join(', ') || '—';
    if (pos)   pos.textContent  = d.open_positions ?? 0;
  } catch {
    if (dot) dot.className = 'provider-dot down';
    if (stat) { stat.textContent = 'OFFLINE'; stat.style.color = 'var(--accent-red)'; }
  }
}

async function _fetchPositions(container) {
  const el = container.querySelector('#aq-pos-list');
  if (!el) return;
  try {
    const r = await fetch(`${QUANT_API}/positions`, { headers: _headers() });
    const d = await r.json();
    const positions = d?.data?.positions ?? [];
    if (positions.length === 0) {
      el.innerHTML = `<div class="empty-state" style="padding:12px 0">
        <div class="empty-sub">Sin posiciones abiertas</div></div>`;
      return;
    }
    el.innerHTML = positions.map(p => {
      const pnlColor = p.pnl >= 0 ? 'var(--accent-green)' : 'var(--accent-red)';
      const sign = p.pnl >= 0 ? '+' : '';
      return `<div class="approval-card" style="padding:10px 14px;margin-bottom:6px">
        <div style="display:flex;align-items:center;justify-content:space-between;gap:8px">
          <div>
            <span class="chip blue" style="font-size:10px">${_esc(p.side.toUpperCase())}</span>
            <span style="font-weight:600;margin-left:8px">${_esc(p.symbol)}</span>
          </div>
          <span style="font-size:13px;font-weight:700;color:${pnlColor}">
            ${sign}${p.pnl?.toFixed(2)} (${sign}${p.pnl_pct?.toFixed(2)}%)
          </span>
        </div>
        <div style="font-size:11px;color:var(--text-muted);margin-top:4px">
          Entrada: ${p.entry_price} · Actual: ${p.current_price} · Size: ${p.size}
          ${p.stop_loss ? ` · SL: ${p.stop_loss}` : ''}
          ${p.take_profit ? ` · TP: ${p.take_profit}` : ''}
        </div>
      </div>`;
    }).join('');
  } catch {}
}

async function _fetchMonitorSummary(container) {
  const summaryEl = container.querySelector('#aq-monitor-summary');
  const alertsEl = container.querySelector('#aq-monitor-alerts');
  const listEl = container.querySelector('#aq-monitor-list');
  if (!summaryEl || !alertsEl || !listEl) return;
  const qs = _monitorParams(container);
  try {
    const r = await fetch(`${QUANT_API}/monitor/summary${qs ? `?${qs}` : ''}`, { headers: _headers() });
    const d = await r.json();
    if (!d?.ok || !d?.data) throw new Error(d?.error || 'No se pudo cargar monitor');
    const m = d.data;
    const acct = m.account_session || {};
    const pdt = m.pdt_status || {};
    const alerts = Array.isArray(m.alerts) ? m.alerts : [];
    const strategies = Array.isArray(m.strategies) ? m.strategies : [];
    const pdtChip = pdt.blocked_opening
      ? '<span class="chip red">PDT bloqueando aperturas</span>'
      : `<span class="chip ${acct.classification === 'live' ? 'orange' : 'green'}">${acct.classification === 'live' ? 'LIVE vigilado' : 'PAPER'}</span>`;
    summaryEl.innerHTML = `
      <div class="stat-row" style="margin-bottom:12px">
        <div class="stat-card hero">
          <div class="stat-card-label">Cuenta Tradier</div>
          <div class="stat-card-value" style="font-size:16px">${_esc(acct.account_id || '—')}</div>
          <div class="stat-card-sub">${_esc(acct.classification || 'sin sesión')} · ${_esc(acct.scope || '—')}</div>
        </div>
        <div class="stat-card">
          <div class="stat-card-label">Equity</div>
          <div class="stat-card-value ${Number(acct.total_equity || 0) >= 25000 ? 'green' : 'orange'}">${acct.total_equity ? `$${Number(acct.total_equity).toFixed(2)}` : '—'}</div>
        </div>
        <div class="stat-card">
          <div class="stat-card-label">PDT</div>
          <div style="display:flex;flex-direction:column;gap:6px;margin-top:6px">
            ${pdtChip}
            <div class="stat-card-sub">${_esc(pdt.reason || 'sin observaciones')}</div>
          </div>
        </div>
        <div class="stat-card">
          <div class="stat-card-label">Alertas</div>
          <div class="stat-card-value ${alerts.length > 0 ? 'red' : 'green'}">${alerts.length}</div>
          <div class="stat-card-sub">refresh ${Math.round((m.refresh_interval_sec || 300) / 60)} min</div>
        </div>
      </div>
      <div style="display:flex;gap:10px;flex-wrap:wrap">
        <span class="chip blue">Posiciones: ${_esc(String(m?.totals?.positions ?? '0'))}</span>
        <span class="chip accent">Estrategias: ${_esc(String(m?.totals?.strategies ?? '0'))}</span>
        <span class="chip ${Number(m?.totals?.open_pnl || 0) >= 0 ? 'green' : 'red'}">PnL abierto: ${_esc(_fmtMoney(m?.totals?.open_pnl || 0))}</span>
        <span class="chip ${pdt.blocked_opening ? 'red' : 'blue'}">Day trades 5d: ${_esc(String(pdt.day_trades_last_window ?? 0))}</span>
      </div>
    `;
    alertsEl.innerHTML = alerts.length === 0
      ? `<div class="chip green">Sin alertas de probabilidad</div>`
      : alerts.map(a => `<span class="chip red">${_esc(a.underlying)} · ${_esc(a.strategy_type)} · ${_fmtPct(a.win_rate_pct)}</span>`).join(' ');
    listEl.innerHTML = strategies.length === 0
      ? `<div class="empty-state" style="padding:16px 0"><div class="empty-sub">Sin estrategias abiertas en Tradier</div></div>`
      : strategies.map(s => `
        <div class="approval-card" style="padding:14px;margin-bottom:10px">
          <div style="display:flex;align-items:flex-start;justify-content:space-between;gap:12px;flex-wrap:wrap">
            <div>
              <div style="display:flex;align-items:center;gap:8px;flex-wrap:wrap">
                <span class="chip blue">${_esc(s.strategy_type || '—')}</span>
                <span style="font-weight:700">${_esc(s.underlying || '—')}</span>
                ${s.alert ? '<span class="chip red">Probabilidad < 50%</span>' : ''}
              </div>
              <div style="font-size:11px;color:var(--text-muted);margin-top:6px">
                ${_esc(String((s.positions || []).length))} patas · actualizado ${_esc(s.probability_updated_at || '—')}
              </div>
            </div>
            <div style="text-align:right">
              <div style="font-size:18px;font-weight:700;color:${_moneyColor(s.open_pnl)}">${_fmtMoney(s.open_pnl)}</div>
              <div style="font-size:11px;color:var(--text-muted)">PnL abierto</div>
            </div>
          </div>
          <div style="display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;margin-top:12px">
            <div class="provider-card" style="padding:10px">
              <div class="provider-role">Win Rate</div>
              <div class="provider-name" style="font-size:18px;color:${Number(s.win_rate_pct || 0) >= 50 ? 'var(--accent-green)' : 'var(--accent-red)'}">${_fmtPct(s.win_rate_pct)}</div>
            </div>
            <div class="provider-card" style="padding:10px">
              <div class="provider-role">Delta Neto</div>
              <div class="provider-name" style="font-size:18px">${_fmtNum(s.net_delta, 2)}</div>
            </div>
            <div class="provider-card" style="padding:10px">
              <div class="provider-role">Theta Diario</div>
              <div class="provider-name" style="font-size:18px">${_fmtMoney(s.theta_daily)}</div>
            </div>
            <div class="provider-card" style="padding:10px">
              <div class="provider-role">Driver</div>
              <div class="provider-name" style="font-size:14px">${_esc(s?.attribution?.dominant_driver || '—')}</div>
            </div>
          </div>
          <div style="margin-top:12px;padding:10px;border:1px solid var(--border);border-radius:10px;background:rgba(255,255,255,0.02)">
            <div style="display:flex;justify-content:space-between;gap:10px;font-size:11px;color:var(--text-muted);margin-bottom:6px">
              <span>Curva de riesgo</span>
              <span>Spot: ${_fmtNum(s.spot, 2)}</span>
            </div>
            ${_curveSvg(s.payoff_curve || [])}
          </div>
          <div style="display:flex;gap:8px;flex-wrap:wrap;margin-top:10px">
            <span class="chip ${Number(s?.attribution?.delta_pnl_est || 0) >= 0 ? 'green' : 'red'}">Delta: ${_fmtMoney(s?.attribution?.delta_pnl_est || 0)}</span>
            <span class="chip ${Number(s?.attribution?.theta_pnl_est || 0) >= 0 ? 'green' : 'red'}">Theta: ${_fmtMoney(s?.attribution?.theta_pnl_est || 0)}</span>
            <span class="chip ${Number(s?.attribution?.vega_pnl_est || 0) >= 0 ? 'green' : 'red'}">Vega: ${_fmtMoney(s?.attribution?.vega_pnl_est || 0)}</span>
          </div>
        </div>
      `).join('');
  } catch (e) {
    summaryEl.innerHTML = `<div class="empty-state" style="padding:12px 0"><div class="empty-title" style="color:var(--accent-red)">Monitor no disponible</div><div class="empty-sub">${_esc(e.message || 'Error')}</div></div>`;
    alertsEl.innerHTML = '';
    listEl.innerHTML = '';
  }
}

export default {
  id: 'atlas-quant',
  label: 'Atlas Code-Quant',
  icon: 'trending-up',
  category: 'trading',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Dashboard
          </button>
          <h2>Atlas Code-Quant</h2>
          <div style="margin-left:auto;display:flex;align-items:center;gap:8px">
            <span class="chip blue" style="font-size:10px">v0.1.0</span>
            <span class="live-badge">LIVE</span>
          </div>
        </div>

        <div class="module-body">

          <!-- Estado del servicio -->
          <div class="stat-row" style="margin-bottom:20px">
            <div class="stat-card hero">
              <div class="stat-card-label">Motor de Trading</div>
              <div style="display:flex;align-items:center;gap:8px;margin-top:6px">
                <div class="provider-dot down" id="aq-dot"></div>
                <span style="font-size:18px;font-weight:700" id="aq-status">—</span>
              </div>
              <div class="stat-card-sub">puerto 8792</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Uptime</div>
              <div class="stat-card-value" id="aq-uptime" style="font-size:16px">—</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Estrategias activas</div>
              <div class="stat-card-value" id="aq-strategies" style="font-size:13px">—</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Posiciones abiertas</div>
              <div class="stat-card-value orange" id="aq-positions">—</div>
            </div>
          </div>

          <!-- Acciones rápidas -->
          <div class="action-bar" style="padding-top:0;margin-bottom:20px">
            <button class="action-btn primary" id="aq-btn-signal">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="22 7 13.5 15.5 8.5 10.5 2 17"/><polyline points="16 7 22 7 22 13"/></svg>
              Evaluar señal
            </button>
            <button class="action-btn" id="aq-btn-positions">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="2" y="3" width="20" height="14" rx="2"/><path d="M8 21h8M12 17v4"/></svg>
              Actualizar posiciones
            </button>
            <button class="action-btn" id="aq-btn-monitor">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M3 12h4l3 8 4-16 3 8h4"/></svg>
              Actualizar monitor
            </button>
            <a href="${QUANT_API}/docs" target="_blank" class="action-btn" style="text-decoration:none">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M14 2H6a2 2 0 00-2 2v16a2 2 0 002 2h12a2 2 0 002-2V8z"/><path d="M14 2v6h6"/></svg>
              API Docs
            </a>
          </div>

          <div class="section-title" style="margin-bottom:10px">
            Monitor avanzado
            <span class="chip accent" style="font-size:10px;margin-left:6px">Tradier</span>
          </div>
          <div class="approval-card" style="padding:14px;margin-bottom:20px">
            <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-bottom:12px">
              <select id="aq-account-scope" class="config-input" style="width:120px">
                <option value="paper" selected>Paper</option>
                <option value="live">Live</option>
              </select>
              <input id="aq-account-id" class="config-input" placeholder="Account ID (opcional)" style="width:220px">
              <div id="aq-monitor-alerts" style="display:flex;gap:8px;flex-wrap:wrap"></div>
            </div>
            <div id="aq-monitor-summary">
              <div style="padding:16px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
            </div>
            <div id="aq-monitor-list" style="margin-top:14px"></div>
          </div>

          <!-- Eval señal rápida -->
          <div id="aq-signal-panel" style="display:none;margin-bottom:20px">
            <div class="section-title" style="margin-bottom:10px">Evaluar señal rápida</div>
            <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center">
              <input id="aq-symbol-input" class="config-input" placeholder="Símbolo (ej: BTC/USDT)"
                style="width:180px" value="BTC/USDT">
              <select id="aq-tf-select" class="config-input" style="width:100px">
                <option>1m</option><option>5m</option><option selected>1h</option>
                <option>4h</option><option>1d</option>
              </select>
              <button class="action-btn primary" id="aq-btn-run-signal">Ejecutar</button>
            </div>
            <div id="aq-signal-result" style="margin-top:10px"></div>
          </div>

          <!-- Backtesting panel -->
          <div class="section-title" style="margin-bottom:10px">
            Backtesting
            <span class="chip orange" style="font-size:10px;margin-left:6px">Simulación</span>
          </div>
          <div class="approval-card" style="padding:14px;margin-bottom:20px">
            <div style="display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-bottom:10px">
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Símbolo</div>
                <input id="aq-bt-symbol" class="config-input" value="BTC/USDT" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Timeframe</div>
                <select id="aq-bt-tf" class="config-input" style="width:100%">
                  <option>1h</option><option>4h</option><option>1d</option>
                </select>
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Estrategia</div>
                <select id="aq-bt-strategy" class="config-input" style="width:100%">
                  <option value="ma_cross">MA Cross</option>
                  <option value="ml_rf">ML (Random Forest)</option>
                </select>
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Velas</div>
                <input id="aq-bt-limit" class="config-input" value="500" type="number" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Capital ($)</div>
                <input id="aq-bt-capital" class="config-input" value="10000" type="number" style="width:100%">
              </div>
              <div style="display:flex;align-items:flex-end">
                <button class="action-btn primary" id="aq-btn-backtest" style="width:100%">
                  ▶ Ejecutar
                </button>
              </div>
            </div>
            <div id="aq-bt-result"></div>
          </div>

          <!-- Posiciones -->
          <div class="section-title" style="margin-bottom:10px">
            Posiciones abiertas
            <span class="live-badge">LIVE</span>
          </div>
          <div id="aq-pos-list">
            <div style="padding:16px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
          </div>

          <!-- Info arquitectura -->
          <div class="section-title" style="margin-top:24px;margin-bottom:10px">Arquitectura</div>
          <div style="display:grid;grid-template-columns:repeat(auto-fill,minmax(160px,1fr));gap:8px">
            ${[
              ['📊','data/','Ingestión OHLCV'],
              ['🧠','models/','ML Señales'],
              ['⚙️','strategies/','Event-driven'],
              ['💼','execution/','Portfolio & Risk'],
              ['🌐','api/','REST :8792'],
              ['🔬','backtesting/','Backtest engine'],
            ].map(([icon,name,desc]) => `
              <div class="provider-card" style="padding:12px">
                <div style="font-size:20px;margin-bottom:4px">${icon}</div>
                <div class="provider-name" style="font-size:12px">${name}</div>
                <div class="provider-role">${desc}</div>
              </div>
            `).join('')}
          </div>

        </div>
      </div>
    `;

    // toggle panel señal
    container.querySelector('#aq-btn-signal')?.addEventListener('click', () => {
      const p = container.querySelector('#aq-signal-panel');
      if (p) p.style.display = p.style.display === 'none' ? 'block' : 'none';
    });

    // actualizar posiciones manual
    container.querySelector('#aq-btn-positions')?.addEventListener('click', () => {
      _fetchPositions(container);
      _fetchMonitorSummary(container);
      window.AtlasToast?.show('Posiciones actualizadas', 'info');
    });

    container.querySelector('#aq-btn-monitor')?.addEventListener('click', () => {
      _fetchMonitorSummary(container);
      window.AtlasToast?.show('Monitor avanzado actualizado', 'info');
    });

    container.querySelector('#aq-account-scope')?.addEventListener('change', () => {
      _fetchMonitorSummary(container);
    });

    container.querySelector('#aq-account-id')?.addEventListener('change', () => {
      _fetchMonitorSummary(container);
    });

    // ejecutar señal rápida
    container.querySelector('#aq-btn-run-signal')?.addEventListener('click', async () => {
      const symbol = container.querySelector('#aq-symbol-input')?.value?.trim() || 'BTC/USDT';
      const tf     = container.querySelector('#aq-tf-select')?.value || '1h';
      const resEl  = container.querySelector('#aq-signal-result');
      if (resEl) resEl.innerHTML = '<div class="spinner" style="margin:0 auto;display:block"></div>';
      try {
        const r = await fetch(`${QUANT_API}/signal`, {
          method: 'POST', headers: _headers(),
          body: JSON.stringify({ symbol, timeframe: tf }),
        });
        const d = await r.json();
        const sig = d?.data;
        if (!d.ok || !sig) throw new Error(d.error || 'Sin respuesta');
        const clr = sig.signal === 'BUY' ? 'green' : sig.signal === 'SELL' ? 'red' : 'orange';
        if (resEl) resEl.innerHTML = `
          <div class="approval-card" style="padding:12px">
            <div style="display:flex;gap:10px;align-items:center">
              <span class="chip ${clr}">${_esc(sig.signal)}</span>
              <span style="font-weight:600">${_esc(sig.symbol)}</span>
              <span style="color:var(--text-muted);font-size:12px">conf: ${(sig.confidence*100).toFixed(1)}%</span>
            </div>
            <div style="font-size:11px;color:var(--text-muted);margin-top:6px">
              Precio: ${sig.price}
              ${sig.stop_loss ? ` · SL: ${sig.stop_loss}` : ''}
              ${sig.take_profit ? ` · TP: ${sig.take_profit}` : ''}
            </div>
          </div>`;
      } catch (e) {
        if (resEl) resEl.innerHTML = `<div style="color:var(--accent-red);font-size:12px">${_esc(e.message)}</div>`;
      }
    });

    // ejecutar backtest
    container.querySelector('#aq-btn-backtest')?.addEventListener('click', async () => {
      const symbol   = container.querySelector('#aq-bt-symbol')?.value?.trim() || 'BTC/USDT';
      const tf       = container.querySelector('#aq-bt-tf')?.value || '1h';
      const strategy = container.querySelector('#aq-bt-strategy')?.value || 'ma_cross';
      const limit    = parseInt(container.querySelector('#aq-bt-limit')?.value) || 500;
      const capital  = parseFloat(container.querySelector('#aq-bt-capital')?.value) || 10000;
      const resEl    = container.querySelector('#aq-bt-result');
      if (resEl) resEl.innerHTML = '<div style="text-align:center;padding:12px"><div class="spinner" style="margin:0 auto;display:block"></div><div style="color:var(--text-muted);font-size:11px;margin-top:6px">Procesando backtest…</div></div>';
      try {
        const r = await fetch(`${QUANT_API}/backtest`, {
          method: 'POST', headers: _headers(),
          body: JSON.stringify({ symbol, timeframe: tf, strategy, limit, capital }),
        });
        const d = await r.json();
        if (!d.ok) throw new Error(d.error || 'Error en backtest');
        const m = d.data.metrics;
        const retClr = m.total_return_pct >= 0 ? 'var(--accent-green)' : 'var(--accent-red)';
        const sign = m.total_return_pct >= 0 ? '+' : '';
        if (resEl) resEl.innerHTML = `
          <div style="display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-top:8px">
            <div class="stat-card" style="padding:10px">
              <div class="stat-card-label">Retorno</div>
              <div style="font-size:16px;font-weight:700;color:${retClr}">${sign}${m.total_return_pct?.toFixed(2)}%</div>
            </div>
            <div class="stat-card" style="padding:10px">
              <div class="stat-card-label">Sharpe</div>
              <div class="stat-card-value">${m.sharpe_ratio?.toFixed(3)}</div>
            </div>
            <div class="stat-card" style="padding:10px">
              <div class="stat-card-label">Drawdown</div>
              <div style="font-size:16px;font-weight:700;color:var(--accent-red)">${m.max_drawdown_pct?.toFixed(2)}%</div>
            </div>
            <div class="stat-card" style="padding:10px">
              <div class="stat-card-label">Win Rate</div>
              <div class="stat-card-value">${m.win_rate_pct?.toFixed(1)}%</div>
            </div>
            <div class="stat-card" style="padding:10px">
              <div class="stat-card-label">Trades</div>
              <div class="stat-card-value">${m.total_trades}</div>
            </div>
            <div class="stat-card" style="padding:10px">
              <div class="stat-card-label">Profit Factor</div>
              <div class="stat-card-value">${m.profit_factor?.toFixed(2)}</div>
            </div>
            <div class="stat-card" style="padding:10px">
              <div class="stat-card-label">Capital final</div>
              <div class="stat-card-value green">$${m.final_capital?.toFixed(0)}</div>
            </div>
            <div class="stat-card" style="padding:10px">
              <div class="stat-card-label">Tiempo/trade</div>
              <div class="stat-card-value">${m.avg_duration_h?.toFixed(1)}h</div>
            </div>
          </div>
          <div style="font-size:11px;color:var(--text-muted);margin-top:6px;text-align:right">
            ${d.data.bars_processed} velas · ${d.ms}ms
          </div>`;
        window.AtlasToast?.show('Backtest completado', 'success');
      } catch (e) {
        if (resEl) resEl.innerHTML = `<div style="color:var(--accent-red);font-size:12px;padding:8px">${_esc(e.message)}</div>`;
      }
    });

    _fetchHealth(container);
    _fetchPositions(container);
    _fetchMonitorSummary(container);

    poll(POLL_ID, `${QUANT_API}/health`, 15000, () => _fetchHealth(container));
    poll(MONITOR_POLL_ID, `${QUANT_API}/health`, 60000, () => _fetchMonitorSummary(container));
  },

  destroy() {
    stop(POLL_ID);
    stop(MONITOR_POLL_ID);
  },

  badge() { return null; },
};

window.AtlasModuleQuantBot = { id: 'atlas-quant' };
