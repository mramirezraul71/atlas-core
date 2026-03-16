/**
 * ATLAS v4.3 — Atlas Code-Quant
 * Shortcut y panel de estado del sistema de trading algorítmico.
 * API interna en puerto 8792.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID  = 'atlas-quant-module';
const QUANT_API = 'http://127.0.0.1:8792';
const API_KEY   = 'atlas-quant-local';

function _esc(s) {
  const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML;
}

function _headers() {
  return { 'X-Api-Key': API_KEY, 'Content-Type': 'application/json' };
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
            <a href="${QUANT_API}/docs" target="_blank" class="action-btn" style="text-decoration:none">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M14 2H6a2 2 0 00-2 2v16a2 2 0 002 2h12a2 2 0 002-2V8z"/><path d="M14 2v6h6"/></svg>
              API Docs
            </a>
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
      window.AtlasToast?.show('Posiciones actualizadas', 'info');
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

    poll(POLL_ID, `${QUANT_API}/health`, 15000, () => _fetchHealth(container));
  },

  destroy() {
    stop(POLL_ID);
  },

  badge() { return null; },
};

window.AtlasModuleQuantBot = { id: 'atlas-quant' };
