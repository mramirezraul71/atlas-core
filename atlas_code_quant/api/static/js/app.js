/* ================================================================
   Atlas Code-Quant Dashboard — v1.0.0
   Core app logic: navigation, data loading, UI updates
   ================================================================ */

// ── Navigation ────────────────────────────────────────────────────
function initNav() {
  document.querySelectorAll('.nav-item').forEach(item => {
    item.addEventListener('click', () => {
      const view = item.dataset.view;
      document.querySelectorAll('.nav-item').forEach(n => n.classList.remove('active'));
      document.querySelectorAll('.view').forEach(v => v.classList.remove('active'));
      item.classList.add('active');
      document.getElementById(`view-${view}`)?.classList.add('active');
      onViewActivated(view);
    });
  });
}

function onViewActivated(view) {
  switch (view) {
    case 'overview':   loadOverview(); break;
    case 'positions':  loadPositions(); break;
    case 'scanner':    loadScanner(); break;
    case 'backtest':   /* user-triggered */ break;
    case 'journal':    loadJournal(); break;
    case 'rl':         loadRL(); break;
    case 'visual':     /* user-triggered */ break;
    case 'alerts':     loadAlerts(); break;
  }
}

// ── Toast ─────────────────────────────────────────────────────────
function toast(msg, type = 'info', ms = 3500) {
  const c = document.getElementById('toast-container');
  if (!c) return;
  const el = document.createElement('div');
  el.className = `toast ${type}`;
  el.textContent = msg;
  c.appendChild(el);
  setTimeout(() => {
    el.style.animation = 'fadeOut 0.3s ease forwards';
    setTimeout(() => el.remove(), 300);
  }, ms);
}

// ── Clock ─────────────────────────────────────────────────────────
function startClock() {
  const el = document.getElementById('sidebar-time');
  function tick() {
    const now = new Date();
    if (el) el.textContent = now.toTimeString().slice(0, 8);
  }
  tick();
  setInterval(tick, 1000);
}

// ── Format helpers ────────────────────────────────────────────────
const fmt = {
  usd: v => v == null ? '--' : `$${parseFloat(v).toLocaleString('en-US', { minimumFractionDigits: 2, maximumFractionDigits: 2 })}`,
  pct: v => v == null ? '--' : `${parseFloat(v).toFixed(2)}%`,
  num: (v, d = 3) => v == null ? '--' : parseFloat(v).toFixed(d),
  ts:  v => v ? new Date(v).toLocaleTimeString() : '--',
};

const QUANT_SCOPE = window.QUANT_ACCOUNT_SCOPE || 'paper';
const QUANT_ACCOUNT_ID = window.QUANT_ACCOUNT_ID || '';

function colorClass(val) {
  if (val == null) return '';
  return parseFloat(val) >= 0 ? 'green' : 'red';
}

function syncLabel(reconciliation) {
  const state = reconciliation?.state || 'unknown';
  if (state === 'healthy') return 'SYNC OK';
  if (state === 'degraded') return 'SYNC WARN';
  if (state === 'failed') return 'SYNC FAIL';
  if (state === 'stale') return 'SYNC STALE';
  return 'SYNC --';
}

function simulatorSummary(simulators) {
  const paper = simulators?.paper_local?.open_positions;
  const optionstrat = simulators?.optionstrat?.open_positions;
  const paperText = paper == null ? 'paper --' : `paper ${paper}`;
  const optText = optionstrat == null ? 'opt --' : `opt ${optionstrat}`;
  return `${paperText} | ${optText}`;
}

function renderCanonicalMeta(snapshot) {
  if (!snapshot) return;
  const totals = snapshot.totals || {};
  const balances = snapshot.balances || {};
  const reconciliation = snapshot.reconciliation || {};
  document.getElementById('chip-source').textContent =
    `${snapshot.source_label || 'Tradier'} · ${snapshot.account_scope || QUANT_SCOPE}`;
  document.getElementById('chip-sync').textContent = syncLabel(reconciliation);
  document.getElementById('chip-positions').textContent =
    `${totals.positions || 0} pos`;
  if (balances.total_equity != null) {
    document.getElementById('chip-equity').textContent = fmt.usd(balances.total_equity);
  }
  const ovSource = document.getElementById('ov-source-badge');
  if (ovSource) ovSource.textContent = `Operativa ${snapshot.source_label || 'Tradier'}`;
  const ovSim = document.getElementById('ov-sim-badge');
  if (ovSim) ovSim.textContent = simulatorSummary(snapshot.simulators);
  const posSource = document.getElementById('pos-source-badge');
  if (posSource) posSource.textContent = `Fuente ${snapshot.source_label || 'Tradier'}`;
  const posSync = document.getElementById('pos-sync-badge');
  if (posSync) posSync.textContent = syncLabel(reconciliation);
}

function renderOverviewHistory(meta) {
  const ovHistory = document.getElementById('ov-history-badge');
  if (!ovHistory) return;
  const label = meta?.historical_source?.label || 'Histórico --';
  ovHistory.textContent = label;
}

function renderOverviewRealtime(snapshot, overview = null) {
  if (!snapshot) return;
  const balances = snapshot.balances || {};
  const totals = snapshot.totals || {};
  const monitorSummary = snapshot.monitor_summary || {};
  const equity = balances.total_equity;
  const openPnl = totals.open_pnl;
  const positions = totals.positions || 0;

  setKPI('kpi-equity', fmt.usd(equity));
  document.getElementById('kpi-trades').textContent = `${positions} pos abiertas`;
  const pnlEl = document.getElementById('kpi-equity-delta');
  if (pnlEl) {
    pnlEl.textContent = `${openPnl >= 0 ? '+' : ''}${fmt.usd(openPnl)} abierto`;
    pnlEl.style.color = openPnl >= 0 ? 'var(--green)' : 'var(--red)';
  }
  if (overview) {
    setKPI('kpi-sharpe', fmt.num(overview.sharpe_ratio, 2));
    setKPI('kpi-dd', overview.max_drawdown_pct != null ? `${fmt.num(overview.max_drawdown_pct, 2)}%` : '--');
    setKPI('kpi-wr', fmt.pct(overview.win_rate_pct));
    setKPI('kpi-calmar', fmt.num(overview.calmar_ratio, 2));
  } else if (monitorSummary.generated_at) {
    renderOverviewHistory({ historical_source: { label: 'Histórico journal + operativa Tradier' } });
  }
}

function renderPositionsRealtime(snapshot) {
  if (!snapshot) return;
  const positions = snapshot.positions || [];
  const tbody = document.getElementById('positions-body');
  if (!tbody) return;

  document.getElementById('pos-count').textContent = positions.length;
  if (!positions.length) {
    tbody.innerHTML = '<tr class="empty-row"><td colspan="11">Sin posiciones abiertas</td></tr>';
    document.getElementById('pos-unrealized').textContent = fmt.usd(0);
    document.getElementById('pos-exposure').textContent = fmt.usd(snapshot.gross_exposure || 0);
    document.getElementById('pos-cb').textContent = syncLabel(snapshot.reconciliation);
    return;
  }

  let totalUnrealized = 0;
  tbody.innerHTML = positions.map((p) => {
    const pnl = p.unrealized_pnl || 0;
    totalUnrealized += pnl;
    const cls = pnl >= 0 ? 'green' : 'red';
    return `<tr>
      <td class="accent">${p.symbol}</td>
      <td>${p.side || '--'}</td>
      <td>${fmt.usd(p.entry_price)}</td>
      <td>${fmt.usd(p.current_price)}</td>
      <td>${fmt.num(p.quantity, 4)}</td>
      <td class="${cls}">${fmt.usd(pnl)}</td>
      <td class="${cls}">${fmt.pct(p.pnl_pct)}</td>
      <td>${fmt.num(p.log_return, 4)}</td>
      <td class="red">${fmt.usd(p.stop_loss)}</td>
      <td class="green">${fmt.usd(p.take_profit)}</td>
      <td>${fmt.num(p.atr, 4)}</td>
    </tr>`;
  }).join('');

  document.getElementById('pos-unrealized').textContent = fmt.usd(totalUnrealized);
  document.getElementById('pos-unrealized').className =
    `kpi-value ${totalUnrealized >= 0 ? 'green' : 'red'}`;
  document.getElementById('pos-exposure').textContent = fmt.usd(snapshot.gross_exposure || 0);
  document.getElementById('pos-cb').textContent = syncLabel(snapshot.reconciliation);
}

// ── Backend online/offline state ─────────────────────────────────
window._backendOnline = null;  // null=unknown, true=online, false=offline
let _backendOnline = null;

function _setBackendState(online) {
  if (_backendOnline === online) return;
  _backendOnline = online;
  window._backendOnline = online;
  let banner = document.getElementById('offline-banner');
  if (!online) {
    if (!banner) {
      banner = document.createElement('div');
      banner.id = 'offline-banner';
      banner.style.cssText = [
        'position:fixed', 'top:0', 'left:0', 'right:0', 'z-index:9999',
        'background:#ff3b5c', 'color:#fff', 'text-align:center',
        'padding:6px 12px', 'font-size:13px', 'font-weight:600',
        'letter-spacing:.5px',
      ].join(';');
      banner.textContent = '⚠ Backend offline — puerto 8795 no disponible. Reintentando…';
      document.body.prepend(banner);
    }
    document.getElementById('chip-uptime')?.setAttribute('data-offline', '1');
  } else {
    banner?.remove();
    document.getElementById('chip-uptime')?.removeAttribute('data-offline');
    // Recargar overview al volver online
    loadOverview();
  }
}

// ── Health / topbar ───────────────────────────────────────────────
async function pollHealth() {
  window._lastHealthAt = Date.now();
  try {
    const [health, status, canonical] = await Promise.all([
      QuantAPI.health(),
      QuantAPI.status(QUANT_SCOPE).catch(() => null),
      QuantAPI.canonicalSnapshot(QUANT_SCOPE, QUANT_ACCOUNT_ID).catch(() => null),
    ]);
    _setBackendState(true);
    document.getElementById('chip-uptime').textContent =
      `${Math.floor((health.uptime_sec || 0) / 60)}m up`;
    const canonicalData = canonical?.data || null;
    if (canonicalData) renderCanonicalMeta(canonicalData);
    const openPositions =
      canonicalData?.totals?.positions ??
      status?.data?.open_positions ??
      health.open_positions ??
      0;
    document.getElementById('chip-positions').textContent =
      `${openPositions || 0} pos`;
    const totalEquity =
      canonicalData?.balances?.total_equity ??
      status?.data?.balances?.total_equity ??
      status?.data?.account_session?.total_equity;
    if (totalEquity != null) {
      document.getElementById('chip-equity').textContent = fmt.usd(totalEquity);
    }
  } catch (_) {
    _setBackendState(false);
  }
}

// ── OVERVIEW ──────────────────────────────────────────────────────
let _equityChart = null, _ddChart = null;

async function loadOverview() {
  try {
    const health = await QuantAPI.health().catch(() => null);
    if (!health) { _setBackendState(false); return; }
    _setBackendState(true);
    const [ov, canonical] = await Promise.all([
      QuantAPI.dashboardOverview(QUANT_SCOPE, QUANT_ACCOUNT_ID).catch(() => null),
      QuantAPI.canonicalSnapshot(QUANT_SCOPE, QUANT_ACCOUNT_ID).catch(() => null),
    ]);

    const m = ov?.data || {};
    const canonicalData = canonical?.data || m;
    const brokerEquity =
      canonicalData?.balances?.total_equity ??
      m?.balances?.total_equity ??
      null;
    const brokerOpenPnl =
      canonicalData?.totals?.open_pnl ??
      m?.totals?.open_pnl ??
      null;
    const brokerPositions =
      canonicalData?.totals?.positions ??
      m?.open_positions ??
      0;
    const effectiveEquity = brokerEquity ?? m.equity;
    const effectiveUnrealized = brokerOpenPnl ?? m.unrealized_pnl;
    renderCanonicalMeta(canonicalData);
    renderOverviewHistory(m);

    // ── KPIs ──
    setKPI('kpi-equity',  fmt.usd(effectiveEquity));
    setKPI('kpi-sharpe',  fmt.num(m.sharpe_ratio, 2));
    // max_drawdown_pct ya viene en % (negativo)
    const ddPct = m.max_drawdown_pct;
    setKPI('kpi-dd', ddPct != null ? fmt.num(ddPct, 2) + '%' : '--');
    document.getElementById('kpi-dd').className = 'kpi-value red';
    setKPI('kpi-wr',     fmt.pct(m.win_rate_pct));
    setKPI('kpi-kelly',  '--');   // Kelly viene del backtest, no del journal
    setKPI('kpi-calmar', fmt.num(m.calmar_ratio, 2));
    document.getElementById('kpi-trades').textContent =
      m.total_trades
        ? `${m.total_trades || 0} trades`
        : `${brokerPositions || 0} pos abiertas`;

    // PnL delta
    const pnl = effectiveUnrealized;
    const el = document.getElementById('kpi-equity-delta');
    if (el && pnl != null) {
      el.textContent = `${pnl >= 0 ? '+' : ''}${fmt.usd(pnl)} abierto`;
      el.style.color = pnl >= 0 ? 'var(--green)' : 'var(--red)';
    }

    // Actualizar topbar equity
    if (effectiveEquity != null) document.getElementById('chip-equity').textContent = fmt.usd(effectiveEquity);
    document.getElementById('chip-positions').textContent = `${brokerPositions || 0} pos`;

    // ── Equity curve (TradingView) ──
    const eq = m.equity_curve || [];
    if (eq.length) {
      if (!_equityChart) {
        _equityChart = QuantCharts.renderEquityChart('chart-equity', eq);
      } else {
        _equityChart.series.setData(eq);
      }
    }

    // ── Drawdown ──
    const dd = m.drawdown_curve || [];
    if (dd.length) {
      if (!_ddChart) {
        _ddChart = QuantCharts.renderDrawdownChart('chart-drawdown', dd);
      } else {
        _ddChart.series.setData(dd);
      }
    }

    // ── PnL distribution ──
    const pnls = m.trade_pnls || [];
    if (pnls.length) QuantCharts.renderPnlDistChart('chart-pnl-dist', pnls);

    // ── Monte Carlo ──
    QuantCharts.renderMonteCarloChart('chart-mc', m.monte_carlo || null);

    // ── Win/Loss streak ──
    const trades = m.recent_trades || [];
    if (trades.length) QuantCharts.renderStreakChart('chart-streak', trades);

    // ── Heatmap de performance (día × hora) ──
    if (m.heatmap?.length) {
      QuantCharts.renderHeatmapChart('chart-heatmap', m.heatmap);
    }

    // ── Rolling Sharpe ──
    if (eq.length > 22) {
      QuantCharts.renderRollingSharpeChart('chart-rolling-sharpe', eq);
    }

  } catch (e) {
    toast('Error cargando overview: ' + e.message, 'error');
  }
}

function setKPI(id, val, subId) {
  const el = document.getElementById(id);
  if (el) el.textContent = val;
}

// Chart range buttons
document.addEventListener('click', e => {
  if (e.target.dataset.range) {
    document.querySelectorAll('.chart-controls .btn-xs').forEach(b => b.classList.remove('active'));
    e.target.classList.add('active');
    // TODO: filter equity data by range
  }
});

// ── POSITIONS ─────────────────────────────────────────────────────
async function loadPositions() {
  try {
    const r = await QuantAPI.positions(QUANT_SCOPE, QUANT_ACCOUNT_ID);
    const positions = r.data?.positions || r.data || [];
    if (r.data) renderCanonicalMeta(r.data);
    document.getElementById('pos-count').textContent = positions.length;

    const tbody = document.getElementById('positions-body');
    if (!positions.length) {
      tbody.innerHTML = '<tr class="empty-row"><td colspan="11">Sin posiciones abiertas</td></tr>';
      return;
    }

    let totalUnrealized = 0;
    tbody.innerHTML = positions.map(p => {
      const pnl = p.unrealized_pnl || 0;
      totalUnrealized += pnl;
      const cls = pnl >= 0 ? 'green' : 'red';
      return `<tr>
        <td class="accent">${p.symbol}</td>
        <td>${p.side || '--'}</td>
        <td>${fmt.usd(p.entry_price)}</td>
        <td>${fmt.usd(p.current_price)}</td>
        <td>${fmt.num(p.quantity, 4)}</td>
        <td class="${cls}">${fmt.usd(pnl)}</td>
        <td class="${cls}">${fmt.pct(p.pnl_pct)}</td>
        <td>${fmt.num(p.log_return, 4)}</td>
        <td class="red">${fmt.usd(p.stop_loss)}</td>
        <td class="green">${fmt.usd(p.take_profit)}</td>
        <td>${fmt.num(p.atr, 4)}</td>
      </tr>`;
    }).join('');

    document.getElementById('pos-unrealized').textContent = fmt.usd(totalUnrealized);
    document.getElementById('pos-unrealized').className =
      `kpi-value ${totalUnrealized >= 0 ? 'green' : 'red'}`;
    if (r.data?.gross_exposure != null) {
      document.getElementById('pos-exposure').textContent = fmt.usd(r.data.gross_exposure);
    }
    document.getElementById('pos-cb').textContent = syncLabel(r.data?.reconciliation);
  } catch (e) {
    toast('Error cargando posiciones: ' + e.message, 'error');
  }
}

// ── SCANNER ───────────────────────────────────────────────────────
async function loadScanner() {
  try {
    const [status, report] = await Promise.all([
      QuantAPI.scannerStatus().catch(() => null),
      QuantAPI.scannerReport().catch(() => null),
    ]);

    const st = status?.data || {};
    document.getElementById('scanner-status-chip').textContent =
      st.running ? 'ACTIVO' : 'INACTIVO';

    const opps = report?.data?.opportunities || [];
    const tbody = document.getElementById('scanner-body');
    if (!opps.length) {
      tbody.innerHTML = '<tr class="empty-row"><td colspan="7">Sin oportunidades detectadas</td></tr>';
      return;
    }
    tbody.innerHTML = opps.map(o => `<tr>
      <td class="accent">${o.symbol}</td>
      <td>${o.strategy}</td>
      <td class="${parseFloat(o.score || 0) >= 0.7 ? 'accent' : ''}">${fmt.num(o.score, 2)}</td>
      <td>${o.signal || '--'}</td>
      <td>${fmt.pct(o.win_probability)}</td>
      <td class="red">${fmt.pct(o.var_95)}</td>
      <td>${fmt.ts(o.timestamp)}</td>
    </tr>`).join('');
  } catch (e) {
    toast('Error cargando scanner: ' + e.message, 'error');
  }
}

document.getElementById('scanner-start')?.addEventListener('click', async () => {
  await QuantAPI.scannerStart({});
  loadScanner();
  toast('Scanner iniciado', 'success');
});
document.getElementById('scanner-stop')?.addEventListener('click', async () => {
  await QuantAPI.scannerStop();
  loadScanner();
  toast('Scanner detenido');
});

// ── BACKTEST ──────────────────────────────────────────────────────
let _btEquityChart = null;

document.getElementById('bt-run')?.addEventListener('click', async () => {
  const symbol   = document.getElementById('bt-symbol').value.trim();
  const tf       = document.getElementById('bt-tf').value;
  const strategy = document.getElementById('bt-strategy').value;
  const capital  = parseFloat(document.getElementById('bt-capital').value);
  const bars     = parseInt(document.getElementById('bt-bars').value);
  const folds    = parseInt(document.getElementById('bt-folds').value);
  const kelly    = document.getElementById('bt-kelly').checked;
  const atr      = document.getElementById('bt-atr').checked;
  const mc       = document.getElementById('bt-montecarlo').checked;

  document.getElementById('bt-status').textContent = 'running…';
  document.getElementById('bt-run').disabled = true;

  try {
    const r = await QuantAPI.runBacktest({
      symbol, timeframe: tf, strategy_type: strategy,
      initial_capital: capital, bars, walk_forward_folds: folds,
      kelly_sizing: kelly, atr_stops: atr, monte_carlo: mc,
      generate_html: true,
    });

    document.getElementById('bt-status').textContent = r.ok ? 'ok' : 'error';

    if (r.ok && r.data) {
      const m = r.data.metrics || {};
      setBM('bm-total-ret', fmt.pct(m.total_return_pct));
      setBM('bm-log-ret',   fmt.num(m.cumulative_log_return, 4));
      setBM('bm-sharpe',    fmt.num(m.sharpe_ratio, 2));
      setBM('bm-sortino',   fmt.num(m.sortino_ratio, 2));
      setBM('bm-calmar',    fmt.num(m.calmar_ratio, 2));
      setBM('bm-maxdd',     fmt.pct(m.max_drawdown_pct ? m.max_drawdown_pct * 100 : null));
      setBM('bm-wr',        fmt.pct(m.win_rate_pct));
      setBM('bm-pf',        fmt.num(m.profit_factor, 2));
      setBM('bm-trades',    m.total_trades);
      setBM('bm-var',       fmt.pct(m.mc_var_95));
      setBM('bm-cvar',      fmt.pct(m.mc_cvar_95));
      setBM('bm-kelly',     fmt.pct(m.kelly_quarter ? m.kelly_quarter * 100 : null));

      // Equity curve inside backtest panel
      const eq = m.equity_curve || r.data.equity_curve || [];
      if (eq.length) {
        if (_btEquityChart) { _btEquityChart.series.setData(eq); }
        else { _btEquityChart = QuantCharts.renderEquityChart('bt-chart-equity', eq); }
      }

      // Trades table
      const trades = r.data.trades || [];
      const panel = document.getElementById('bt-trades-panel');
      if (trades.length && panel) {
        panel.style.display = 'block';
        document.getElementById('bt-trades-body').innerHTML = trades.map((t, i) => `<tr>
          <td>${i + 1}</td>
          <td class="accent">${t.symbol || symbol}</td>
          <td>${t.side || '--'}</td>
          <td>${fmt.usd(t.entry_price)}</td>
          <td>${fmt.usd(t.exit_price)}</td>
          <td class="${(t.pnl || 0) >= 0 ? 'green' : 'red'}">${fmt.usd(t.pnl)}</td>
          <td class="${(t.pnl_pct || 0) >= 0 ? 'green' : 'red'}">${fmt.pct(t.pnl_pct)}</td>
          <td>${fmt.num(t.log_return, 4)}</td>
          <td>${t.exit_reason || '--'}</td>
        </tr>`).join('');
      }

      // Report link
      const rp = r.data.report_path;
      if (rp) {
        const link = document.getElementById('bt-report-link');
        if (link) link.href = `/reports/${rp.split('/').pop()}`;
      }

      toast('Backtest completado', 'success');
    } else {
      toast('Error en backtest: ' + (r.error || 'desconocido'), 'error');
    }
  } catch (e) {
    toast('Error: ' + e.message, 'error');
  } finally {
    document.getElementById('bt-run').disabled = false;
  }
});

function setBM(id, val) {
  const el = document.getElementById(id);
  if (el) el.textContent = val ?? '--';
}

// ── JOURNAL ───────────────────────────────────────────────────────
async function loadJournal() {
  try {
    const [stats, entries] = await Promise.all([
      QuantAPI.journalStats(),
      QuantAPI.journalEntries(100),
    ]);

    const s = stats?.data || {};
    const kpiRow = document.getElementById('journal-kpi');
    kpiRow.innerHTML = `
      <div class="kpi-card"><span class="kpi-label">Total PnL</span>
        <span class="kpi-value ${(s.total_pnl || 0) >= 0 ? 'green' : 'red'}">${fmt.usd(s.total_pnl)}</span></div>
      <div class="kpi-card"><span class="kpi-label">Win Rate</span>
        <span class="kpi-value">${fmt.pct(s.win_rate_pct)}</span></div>
      <div class="kpi-card"><span class="kpi-label">Trades</span>
        <span class="kpi-value">${s.total_trades || 0}</span></div>
      <div class="kpi-card"><span class="kpi-label">Avg PnL</span>
        <span class="kpi-value">${fmt.usd(s.avg_pnl)}</span></div>
    `;

    const list = entries?.data || [];
    QuantCharts.renderJournalPnlChart('chart-journal-pnl', list);
    QuantCharts.renderJournalStratChart('chart-journal-strat', list);

    document.getElementById('journal-body').innerHTML = list.length
      ? list.map(e => `<tr>
          <td>${e.closed_at ? e.closed_at.slice(0, 16) : '--'}</td>
          <td class="accent">${e.symbol}</td>
          <td>${e.strategy || '--'}</td>
          <td>${e.side || '--'}</td>
          <td class="${(e.pnl || 0) >= 0 ? 'green' : 'red'}">${fmt.usd(e.pnl)}</td>
          <td>${e.duration_min ? e.duration_min + 'm' : '--'}</td>
          <td>${e.exit_reason || '--'}</td>
        </tr>`).join('')
      : '<tr class="empty-row"><td colspan="7">Sin entradas</td></tr>';
  } catch (e) {
    toast('Error cargando journal: ' + e.message, 'error');
  }
}

document.getElementById('journal-refresh')?.addEventListener('click', loadJournal);

// ── RL ENGINE ─────────────────────────────────────────────────────
async function loadRL() {
  const symbol = document.getElementById('rl-symbol-sel')?.value || 'BTC/USDT';
  try {
    const r = await QuantAPI.retrainingStatus(symbol);
    const d = r.data || {};
    document.getElementById('rl-symbol').textContent = d.symbol || symbol;
    document.getElementById('rl-state').textContent  = d.running ? 'ACTIVO' : 'INACTIVO';
    document.getElementById('rl-last').textContent   = d.last_retrain_at
      ? new Date(d.last_retrain_at * 1000).toLocaleString() : 'Nunca';
    document.getElementById('rl-next').textContent   = d.next_retrain_in_h
      ? `en ${d.next_retrain_in_h.toFixed(1)}h` : '--';
    document.getElementById('rl-sharpe').textContent = fmt.num(d.best_sharpe, 3);
    document.getElementById('rl-trials').textContent = d.optuna_trials || '--';

    // Model list
    const models = d.models || [];
    const mlEl = document.getElementById('rl-models-list');
    mlEl.innerHTML = models.length
      ? models.map((m, i) => `<div class="model-entry ${i === 0 ? 'best' : ''}">
          <span class="model-name">${m.name || m}</span>
          <span class="model-sharpe">Sharpe ${fmt.num(m.sharpe, 3)}</span>
        </div>`).join('')
      : '<span class="empty-text">Sin modelos entrenados</span>';
  } catch (e) {
    toast('Error cargando RL status: ' + e.message, 'error');
  }
}

document.getElementById('rl-status-btn')?.addEventListener('click', loadRL);
document.getElementById('rl-trigger-btn')?.addEventListener('click', async () => {
  const symbol = document.getElementById('rl-symbol-sel')?.value || 'BTC/USDT';
  const r = await QuantAPI.retrainingTrigger(symbol);
  if (r.ok) {
    toast(`Retraining iniciado para ${symbol}`, 'success');
    appendLog('rl-log-box', `[${new Date().toLocaleTimeString()}] Ciclo de retraining forzado para ${symbol}`);
  } else {
    toast('Error: ' + (r.error || 'desconocido'), 'error');
  }
  loadRL();
});

// ── VISUAL ────────────────────────────────────────────────────────
document.getElementById('visual-capture')?.addEventListener('click', async () => {
  try {
    const r = await QuantAPI.visualState();
    if (!r.ok) { toast('Error capturando visual: ' + r.error, 'error'); return; }
    const d = r.data;
    document.getElementById('vs-brightness').textContent = fmt.num(d.brightness, 1);
    document.getElementById('vs-sharpness').textContent  = fmt.num(d.sharpness, 1);
    document.getElementById('vs-screens').textContent    = d.screens_detected;
    document.getElementById('vs-color').textContent      = d.chart_color || '--';
    document.getElementById('vs-prices').textContent     = (d.ocr_prices || []).join(', ') || 'ninguno';
    document.getElementById('vs-pattern').textContent    = d.pattern_detected || 'ninguno';

    const badge = document.getElementById('visual-safety-badge');
    badge.textContent  = d.safe_mode ? 'SAFE MODE' : 'OK';
    badge.className    = `mode-badge ${d.safe_mode ? 'safe' : 'ok'}`;

    if (d.feature_vector?.length) {
      QuantCharts.renderVisualFeaturesChart('chart-visual-features', d.feature_vector);
    }
    toast('Estado visual capturado', 'success');
  } catch (e) {
    toast('Error: ' + e.message, 'error');
  }
});

// ── ALERTS ────────────────────────────────────────────────────────
async function loadAlerts() {
  try {
    const r = await QuantAPI.alertsStatus();
    const d = r.data || {};
    document.getElementById('al-telegram').textContent  = d.telegram_enabled  ? 'ON' : 'OFF';
    document.getElementById('al-whatsapp').textContent  = d.whatsapp_enabled  ? 'ON' : 'OFF';
    document.getElementById('al-queue').textContent     = d.queue_size ?? '--';
    document.getElementById('al-sent').textContent      = d.sent_today ?? '--';
    document.getElementById('al-level').textContent     = d.min_level   || '--';
    document.getElementById('al-cooldown').textContent  = d.cooldown_s ? `${d.cooldown_s}s` : '--';
  } catch (e) {
    toast('Error cargando alertas: ' + e.message, 'error');
  }
}

document.getElementById('alerts-refresh-btn')?.addEventListener('click', loadAlerts);
document.getElementById('alerts-test-btn')?.addEventListener('click', async () => {
  const r = await QuantAPI.alertsTest('Test desde Atlas Code-Quant Dashboard v1.0.0');
  if (r.ok) {
    appendLog('alerts-log', `[${new Date().toLocaleTimeString()}] Alerta de prueba enviada`);
    toast('Alerta enviada', 'success');
  } else {
    toast('Error: ' + (r.error || '?'), 'error');
  }
});

// ── Log helper ────────────────────────────────────────────────────
function appendLog(id, line) {
  const el = document.getElementById(id);
  if (!el) return;
  if (el.textContent === 'Esperando actividad…' || el.textContent === 'Sin alertas registradas') {
    el.textContent = '';
  }
  el.textContent = el.textContent + line + '\n';
  el.scrollTop = el.scrollHeight;
}

// ── WebSocket feed ────────────────────────────────────────────────
function initWS() {
  quantWS.on('quant.live_update', (msg) => {
    if (msg.canonical_snapshot) {
      renderCanonicalMeta(msg.canonical_snapshot);
      renderOverviewRealtime(msg.canonical_snapshot);
      if (document.getElementById('view-positions')?.classList.contains('active')) {
        renderPositionsRealtime({
          ...msg.canonical_snapshot,
          gross_exposure: msg.canonical_snapshot?.balances?.gross_exposure ?? 0,
        });
      }
    }
  });

  quantWS.on('signal', (msg) => {
    const tbody = document.getElementById('signal-feed-body');
    if (!tbody) return;
    if (tbody.querySelector('.empty-row')) tbody.innerHTML = '';
    const cls = msg.signal === 'BUY' ? 'green' : msg.signal === 'SELL' ? 'red' : '';
    const row = document.createElement('tr');
    row.innerHTML = `
      <td>${new Date().toLocaleTimeString()}</td>
      <td class="accent">${msg.symbol}</td>
      <td>${msg.strategy}</td>
      <td class="${cls}">${msg.signal}</td>
      <td>${fmt.pct(msg.confidence ? msg.confidence * 100 : null)}</td>
      <td>${fmt.pct(msg.kelly_pct)}</td>
      <td>${fmt.usd(msg.atr_sl)} / ${fmt.usd(msg.atr_tp)}</td>
    `;
    tbody.insertBefore(row, tbody.firstChild);
    if (tbody.children.length > 30) tbody.lastChild.remove();

    // update topbar equity chip if available
    if (msg.equity) {
      document.getElementById('chip-equity').textContent = fmt.usd(msg.equity);
    }
  });

  quantWS.on('trade', (msg) => {
    const dir = msg.side === 'BUY' ? 'compra' : 'venta';
    appendLog('rl-log-box', `[${new Date().toLocaleTimeString()}] Trade ${dir} ${msg.symbol} @ ${fmt.usd(msg.price)}`);
    pollHealth();
  });

  quantWS.connect();
}

// ── Emergency stop ────────────────────────────────────────────────
document.getElementById('btn-emergency')?.addEventListener('click', () => {
  if (!confirm('¿Activar Emergency Stop? Cerrará todas las posiciones.')) return;
  apiPost('/emergency/stop', { reason: 'manual_dashboard' }).then(r => {
    toast(r.ok ? 'Emergency Stop activado' : 'Error: ' + r.error, r.ok ? 'error' : 'error', 5000);
  });
});

// ── Refresh buttons ───────────────────────────────────────────────
document.getElementById('ov-refresh')?.addEventListener('click', loadOverview);
document.getElementById('pos-refresh')?.addEventListener('click', loadPositions);

// ── Init ──────────────────────────────────────────────────────────
document.addEventListener('DOMContentLoaded', () => {
  initNav();
  startClock();
  initWS();
  loadOverview();

  // Primer health check inmediato, luego adaptativo
  pollHealth();
  // Cuando offline: sondear cada 2s. Cuando online: cada 10s como respaldo del WS.
  setInterval(() => {
    const interval = _backendOnline === false ? 2_000 : 10_000;
    if (!window._lastHealthAt || Date.now() - window._lastHealthAt >= interval) {
      window._lastHealthAt = Date.now();
      pollHealth();
    }
  }, 2_000);
});
