/* ================================================================
   Atlas Code-Quant — Paper Trading Platform
   UI completa: equity curve, posiciones, blotter, órdenes manuales
   ================================================================ */

// ── API helpers ────────────────────────────────────────────────────
const PaperAPI = {
  account:     () => apiGet('/paper/account'),
  positions:   () => apiGet('/paper/positions'),
  orders:      (limit = 200) => apiGet('/paper/orders', { limit }),
  equityCurve: (limit = 500) => apiGet('/paper/equity-curve', { limit }),
  fill:        (body) => apiPost('/paper/fill', body),
  close:       (body) => apiPost('/paper/close', body),
  reset:       (capital = 0) => apiPost('/paper/reset', { initial_capital: capital }),
};

// ── Formato helpers ────────────────────────────────────────────────
const _pp = {
  usd: (v) => {
    if (v == null || isNaN(v)) return '--';
    const abs = Math.abs(v), fmt = abs >= 1000
      ? '$' + abs.toLocaleString('en-US', { minimumFractionDigits: 2, maximumFractionDigits: 2 })
      : '$' + abs.toFixed(2);
    return (v < 0 ? '-' : '') + fmt;
  },
  pct: (v) => (v == null || isNaN(v)) ? '--' : (v > 0 ? '+' : '') + v.toFixed(2) + '%',
  num: (v, d = 2) => (v == null || isNaN(v)) ? '--' : Number(v).toFixed(d),
  ts:  (v) => {
    if (!v) return '--';
    const d = new Date(v * 1000);
    return d.toLocaleTimeString('es', { hour: '2-digit', minute: '2-digit', second: '2-digit' });
  },
  color: (v) => v > 0 ? 'style="color:#00d4aa"' : v < 0 ? 'style="color:#ff6b6b"' : '',
};

// ── Equity Curve Chart (lightweight-charts) ────────────────────────
let _ppChart = null;
let _ppSeries = null;

function _ppInitChart() {
  const container = document.getElementById('pp-chart-equity');
  if (!container || _ppChart) return;

  if (typeof LightweightCharts === 'undefined') {
    container.innerHTML = '<div class="empty-state" style="padding:40px">LightweightCharts no disponible</div>';
    return;
  }

  _ppChart = LightweightCharts.createChart(container, {
    width:  container.clientWidth || 700,
    height: 180,
    layout: { background: { color: '#0d1117' }, textColor: '#8b949e' },
    grid:   { vertLines: { color: '#1c2128' }, horzLines: { color: '#1c2128' } },
    crosshair: { mode: LightweightCharts.CrosshairMode.Normal },
    rightPriceScale: { borderColor: '#1c2128' },
    timeScale: { borderColor: '#1c2128', timeVisible: true },
  });

  _ppSeries = _ppChart.addAreaSeries({
    lineColor:    '#00d4aa',
    topColor:     'rgba(0,212,170,0.25)',
    bottomColor:  'rgba(0,212,170,0.02)',
    lineWidth:    2,
    priceFormat:  { type: 'price', precision: 2, minMove: 0.01 },
  });

  window.addEventListener('resize', () => {
    if (_ppChart && container.clientWidth)
      _ppChart.applyOptions({ width: container.clientWidth });
  });
}

function _ppRenderCurve(points) {
  if (!_ppSeries || !points || !points.length) return;
  const data = points.map(p => ({
    time:  Math.floor(p.ts),
    value: parseFloat(p.equity),
  })).filter(p => !isNaN(p.value));

  if (!data.length) return;
  // lightweight-charts requiere timestamps únicos y ordenados
  const seen = new Set();
  const unique = data.filter(p => {
    if (seen.has(p.time)) return false;
    seen.add(p.time);
    return true;
  });
  _ppSeries.setData(unique);
  _ppChart.timeScale().fitContent();

  const el = document.getElementById('pp-curve-range');
  if (el) el.textContent = `${unique.length} puntos`;
}

// ── Render KPIs ────────────────────────────────────────────────────
function _ppRenderAccount(d) {
  if (!d) return;

  const equity = parseFloat(d.equity);
  const pct    = parseFloat(d.total_pnl_pct);
  const rpnl   = parseFloat(d.realized_pnl);
  const upnl   = parseFloat(d.unrealized_pnl);

  const eq = document.getElementById('pp-equity');
  if (eq) {
    eq.textContent = _pp.usd(equity);
    eq.className   = 'kpi-value ' + (pct >= 0 ? 'accent' : 'red');
  }
  _setText('pp-equity-pct', _pp.pct(pct));
  _setText('pp-cash',       _pp.usd(d.cash));
  _setText('pp-rpnl',       _pp.usd(rpnl));
  const rpnlEl = document.getElementById('pp-rpnl');
  if (rpnlEl) rpnlEl.className = 'kpi-value ' + (rpnl >= 0 ? 'accent' : 'red');
  _setText('pp-upnl-sub',   'no real. ' + _pp.usd(upnl));
  _setText('pp-wr',         d.win_rate_pct != null ? d.win_rate_pct.toFixed(1) + '%' : '--');
  _setText('pp-trades-sub', `${d.winning_trades || 0}W / ${d.losing_trades || 0}L`);
  _setText('pp-dd',         d.max_drawdown_pct != null ? '-' + d.max_drawdown_pct.toFixed(2) + '%' : '--');
  _setText('pp-peak-sub',   'peak ' + _pp.usd(d.peak_equity));
  _setText('pp-open-pos',   d.open_positions ?? '--');

  // Config panel
  _setText('pp-cfg-capital',    _pp.usd(d.initial_capital));
  _setText('pp-cfg-peak',       _pp.usd(d.peak_equity));
  _setText('pp-cfg-total-pnl',  _pp.usd(d.total_pnl));
  _setText('pp-cfg-wins',       d.winning_trades ?? '--');
  _setText('pp-cfg-losses',     d.losing_trades  ?? '--');
}

function _setText(id, val) {
  const el = document.getElementById(id);
  if (el) el.textContent = val;
}

// ── Render Positions ───────────────────────────────────────────────
function _ppRenderPositions(positions) {
  const body  = document.getElementById('pp-positions-body');
  const count = document.getElementById('pp-pos-count');
  if (!body) return;

  if (!positions || !positions.length) {
    body.innerHTML = '<tr><td colspan="11" class="empty-state">Sin posiciones abiertas</td></tr>';
    if (count) count.textContent = '0';
    return;
  }

  if (count) count.textContent = positions.length;

  body.innerHTML = positions.map(p => {
    const pnl = parseFloat(p.unrealized_pnl);
    const pnlColor = pnl > 0 ? '#00d4aa' : pnl < 0 ? '#ff6b6b' : '#8b949e';
    return `<tr>
      <td><strong>${p.symbol}</strong></td>
      <td><span class="badge ${p.side === 'long' ? 'badge-green' : 'badge-red'}">${p.side.toUpperCase()}</span></td>
      <td>${p.qty}</td>
      <td>${_pp.usd(p.avg_price)}</td>
      <td>${_pp.usd(p.current_price)}</td>
      <td>${_pp.usd(p.market_value)}</td>
      <td style="color:${pnlColor};font-weight:600">${_pp.usd(pnl)} (${_pp.pct(p.unrealized_pnl_pct)})</td>
      <td><span class="chip">${p.strategy || '--'}</span></td>
      <td style="font-size:11px">${_pp.num(p.signal_score, 3)}</td>
      <td><span class="mode-badge ${_ppTierClass(p.score_tier)}">${p.score_tier || '--'}</span></td>
      <td>
        <button class="btn-xs" style="color:#ff6b6b"
          onclick="ppClosePosition('${p.symbol}', ${p.current_price}, ${p.qty})">
          &#x2715; Cerrar
        </button>
      </td>
    </tr>`;
  }).join('');
}

// ── Render Orders (Blotter) ────────────────────────────────────────
function _ppRenderOrders(orders, filter = 'all') {
  const body  = document.getElementById('pp-orders-body');
  const count = document.getElementById('pp-blotter-count');
  if (!body) return;

  const filtered = filter === 'all' ? orders : orders.filter(o => o.status === filter);
  if (count) count.textContent = `${filtered.length} órdenes`;

  if (!filtered.length) {
    body.innerHTML = '<tr><td colspan="13" class="empty-state">Sin órdenes</td></tr>';
    return;
  }

  body.innerHTML = filtered.map(o => {
    const pnl      = parseFloat(o.pnl);
    const pnlColor = pnl > 0 ? '#00d4aa' : pnl < 0 ? '#ff6b6b' : '#8b949e';
    const statusBadge = o.status === 'filled'
      ? '<span class="live-badge">FILLED</span>'
      : '<span class="mode-badge" style="background:#ff6b6b22;color:#ff6b6b">REJ</span>';
    return `<tr>
      <td style="font-size:11px;color:#8b949e">${_pp.ts(o.ts)}</td>
      <td><strong>${o.symbol}</strong></td>
      <td><span class="badge ${['buy','buy_to_cover'].includes(o.side) ? 'badge-green' : 'badge-red'}">${o.side.toUpperCase()}</span></td>
      <td>${o.qty}</td>
      <td>${_pp.usd(o.fill_price)}</td>
      <td style="color:#8b949e">$${parseFloat(o.commission || 0).toFixed(2)}</td>
      <td style="color:${pnlColor};font-weight:${pnl !== 0 ? '600' : '400'}">${pnl !== 0 ? _pp.usd(pnl) : '--'}</td>
      <td><span class="chip">${o.strategy || '--'}</span></td>
      <td style="font-size:11px;color:#f0b429">${o.option_strategy || '--'}</td>
      <td style="font-size:11px">${_pp.num(o.signal_score, 3)}</td>
      <td><span class="mode-badge ${_ppTierClass(o.score_tier)}">${o.score_tier || '--'}</span></td>
      <td>${statusBadge}</td>
      <td style="font-size:10px;color:#8b949e;max-width:120px;overflow:hidden;text-overflow:ellipsis">${o.reject_reason || ''}</td>
    </tr>`;
  }).join('');
}

function _ppTierClass(tier) {
  const map = { FULL: 'accent', NORMAL: '', SMALL: 'yellow', SKIP: 'red' };
  return map[tier] || '';
}

// ── Carga principal ────────────────────────────────────────────────
let _ppOrders = [];

async function loadPaperPlatform() {
  try {
    _ppInitChart();

    const [accR, posR, ordR, curveR] = await Promise.all([
      PaperAPI.account(),
      PaperAPI.positions(),
      PaperAPI.orders(200),
      PaperAPI.equityCurve(500),
    ]);

    if (accR.ok)   _ppRenderAccount(accR.data);
    if (posR.ok)   _ppRenderPositions(posR.data);
    if (curveR.ok) _ppRenderCurve(curveR.data);

    if (ordR.ok) {
      _ppOrders = ordR.data || [];
      const filter = document.getElementById('pp-blotter-filter')?.value || 'all';
      _ppRenderOrders(_ppOrders, filter);
    }

  } catch (e) {
    console.error('[PaperPlatform]', e);
    toast('Error cargando Paper Platform: ' + e.message, 'error');
  }
}

// ── Cierre de posición ─────────────────────────────────────────────
async function ppClosePosition(symbol, price, qty) {
  if (!confirm(`¿Cerrar posición ${symbol} (${qty} unidades) a $${price}?`)) return;
  try {
    const r = await PaperAPI.close({ symbol, price, qty });
    if (r.ok) {
      toast(`${symbol} cerrado — P&L: ${_pp.usd(r.data.pnl)}`, r.data.pnl >= 0 ? 'success' : 'info');
      await loadPaperPlatform();
    } else {
      toast('Error al cerrar: ' + (r.error || 'desconocido'), 'error');
    }
  } catch (e) {
    toast('Error: ' + e.message, 'error');
  }
}

// ── Event listeners ────────────────────────────────────────────────
document.addEventListener('DOMContentLoaded', () => {

  // Refresh
  document.getElementById('pp-refresh-btn')?.addEventListener('click', loadPaperPlatform);

  // Filtro blotter
  document.getElementById('pp-blotter-filter')?.addEventListener('change', (e) => {
    _ppRenderOrders(_ppOrders, e.target.value);
  });

  // Orden manual
  document.getElementById('pp-ord-submit')?.addEventListener('click', async () => {
    const symbol   = document.getElementById('pp-ord-symbol')?.value?.trim().toUpperCase();
    const side     = document.getElementById('pp-ord-side')?.value;
    const qty      = parseInt(document.getElementById('pp-ord-qty')?.value);
    const price    = parseFloat(document.getElementById('pp-ord-price')?.value);
    const strategy = document.getElementById('pp-ord-strategy')?.value?.trim();
    const score    = parseFloat(document.getElementById('pp-ord-score')?.value);

    if (!symbol || !qty || !price || isNaN(qty) || isNaN(price)) {
      toast('Completa símbolo, cantidad y precio', 'warning');
      return;
    }

    try {
      const r = await PaperAPI.fill({
        symbol, side, qty, price,
        strategy: strategy || 'manual',
        signal_score: isNaN(score) ? 0 : score,
        score_tier: score >= 0.75 ? 'FULL' : score >= 0.65 ? 'NORMAL' : score >= 0.55 ? 'SMALL' : 'SKIP',
      });
      if (r.ok && r.data.status === 'filled') {
        toast(`${side.toUpperCase()} ${symbol} — fill @$${r.data.fill_price}`, 'success');
        await loadPaperPlatform();
      } else {
        toast(`Orden rechazada: ${r.data?.reject_reason || r.error || 'desconocido'}`, 'error');
      }
    } catch (e) {
      toast('Error: ' + e.message, 'error');
    }
  });

  // Reset rápido (botón topbar)
  document.getElementById('pp-reset-btn')?.addEventListener('click', () => {
    const capital = parseFloat(document.getElementById('pp-reset-capital')?.value) || 0;
    const label   = capital > 0 ? `$${capital.toLocaleString()}` : 'capital original';
    if (!confirm(`¿Resetear cuenta paper a ${label}? (historial de órdenes se conserva)`)) return;
    _ppDoReset(capital);
  });

  // Reset desde panel config
  document.getElementById('pp-reset-confirm')?.addEventListener('click', () => {
    const capital = parseFloat(document.getElementById('pp-reset-capital')?.value) || 0;
    const label   = capital > 0 ? `$${capital.toLocaleString()}` : 'capital original';
    if (!confirm(`¿Resetear cuenta paper a ${label}?`)) return;
    _ppDoReset(capital);
  });
});

async function _ppDoReset(capital = 0) {
  try {
    const r = await PaperAPI.reset(capital);
    if (r.ok) {
      _ppChart = null;  // forzar re-init del chart
      _ppSeries = null;
      const el = document.getElementById('pp-chart-equity');
      if (el) el.innerHTML = '';
      toast(`Cuenta reseteada — capital: $${r.data.initial_capital.toLocaleString()}`, 'success');
      await loadPaperPlatform();
    } else {
      toast('Error al resetear: ' + r.error, 'error');
    }
  } catch (e) {
    toast('Error: ' + e.message, 'error');
  }
}

// ── Auto-refresh cada 30s cuando la vista está activa ──────────────
let _ppAutoRefreshId = null;

function _ppStartAutoRefresh() {
  _ppStopAutoRefresh();
  _ppAutoRefreshId = setInterval(loadPaperPlatform, 30_000);
}

function _ppStopAutoRefresh() {
  if (_ppAutoRefreshId) { clearInterval(_ppAutoRefreshId); _ppAutoRefreshId = null; }
}

// Exponer para que app.js pueda llamarlos al cambiar de vista
window.loadPaperPlatform   = loadPaperPlatform;
window.ppStartAutoRefresh  = _ppStartAutoRefresh;
window.ppStopAutoRefresh   = _ppStopAutoRefresh;
window.ppClosePosition     = ppClosePosition;
