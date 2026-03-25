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
  // Detener auto-refresh si salimos de posiciones
  if (view !== 'positions') _stopPosAutoRefresh();
  if (view !== 'paper' && window.ppStopAutoRefresh) window.ppStopAutoRefresh();
  switch (view) {
    case 'overview':   loadOverview(); break;
    case 'positions':  loadPositions(); _startPosAutoRefresh(); break;
    case 'scanner':    loadScanner(); break;
    case 'backtest':   /* user-triggered */ break;
    case 'journal':    loadJournal(); break;
    case 'rl':         loadRL(); break;
    case 'visual':     /* user-triggered */ break;
    case 'alerts':     loadAlerts(); break;
    case 'paper':
      if (window.loadPaperPlatform) {
        window.loadPaperPlatform();
        if (window.ppStartAutoRefresh) window.ppStartAutoRefresh();
      }
      break;
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

function colorClass(val) {
  if (val == null) return '';
  return parseFloat(val) >= 0 ? 'green' : 'red';
}

// ── Health / topbar ───────────────────────────────────────────────
async function pollHealth() {
  try {
    const r = await QuantAPI.health();
    document.getElementById('chip-uptime').textContent =
      `${Math.floor((r.uptime_sec || 0) / 60)}m up`;
    document.getElementById('chip-positions').textContent =
      `${r.open_positions || 0} pos`;
  } catch (_) {}
}

// ── OVERVIEW ──────────────────────────────────────────────────────
let _equityChart = null, _ddChart = null;

async function loadOverview() {
  try {
    const [health, ov] = await Promise.all([
      QuantAPI.health(),
      QuantAPI.dashboardOverview().catch(() => null),
    ]);

    const m = ov?.data || {};

    // ── KPIs ──
    setKPI('kpi-equity',  fmt.usd(m.equity));
    setKPI('kpi-sharpe',  fmt.num(m.sharpe_ratio, 2));
    // max_drawdown_pct ya viene en % (negativo)
    const ddPct = m.max_drawdown_pct;
    setKPI('kpi-dd', ddPct != null ? fmt.num(ddPct, 2) + '%' : '--');
    document.getElementById('kpi-dd').className = 'kpi-value red';
    setKPI('kpi-wr',     fmt.pct(m.win_rate_pct));
    setKPI('kpi-kelly',  '--');   // Kelly viene del backtest, no del journal
    setKPI('kpi-calmar', fmt.num(m.calmar_ratio, 2));
    document.getElementById('kpi-trades').textContent = `${m.total_trades || 0} trades`;

    // PnL delta
    const pnl = m.realized_pnl;
    const el = document.getElementById('kpi-equity-delta');
    if (el && pnl != null) {
      el.textContent = `${pnl >= 0 ? '+' : ''}${fmt.usd(pnl)} realizado`;
      el.style.color = pnl >= 0 ? 'var(--green)' : 'var(--red)';
    }

    // Actualizar topbar equity
    if (m.equity) document.getElementById('chip-equity').textContent = fmt.usd(m.equity);

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
let _posRefreshInterval = null;

function _posTabInit() {
  document.querySelectorAll('[data-pos-tab]').forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('[data-pos-tab]').forEach(b => b.classList.remove('active'));
      document.querySelectorAll('.tab-pane').forEach(p => p.classList.remove('active'));
      btn.classList.add('active');
      document.getElementById(`pos-tab-${btn.dataset.posTab}`)?.classList.add('active');
      if (btn.dataset.posTab === 'history') loadPosHistory();
    });
  });
}

function _posDuration(openedAtMs) {
  if (!openedAtMs) return '--';
  const diffMin = Math.floor((Date.now() - openedAtMs) / 60000);
  if (diffMin < 60) return `${diffMin}m`;
  return `${Math.floor(diffMin / 60)}h ${diffMin % 60}m`;
}

function _posProgress(entry, current, sl, tp) {
  if (!sl || !tp || !entry) return '';
  const range = tp - sl;
  if (range <= 0) return '';
  const pct = Math.max(0, Math.min(100, ((current - sl) / range) * 100));
  const color = pct >= 66 ? 'var(--green)' : pct >= 33 ? 'var(--yellow)' : 'var(--red)';
  return `<div class="sl-tp-bar">
    <div class="sl-tp-fill" style="width:${pct.toFixed(1)}%;background:${color}"></div>
  </div>
  <span style="font-size:10px;color:var(--text-muted)">${pct.toFixed(0)}%</span>`;
}

async function loadPositions() {
  try {
    const [posR, healthR] = await Promise.all([
      QuantAPI.positions().catch(() => null),
      QuantAPI.health().catch(() => null),
    ]);

    const data      = posR?.data || {};
    const positions = data.positions || (Array.isArray(posR?.data) ? posR.data : []);
    const health    = healthR || {};

    // ── KPIs ──
    document.getElementById('pos-count').textContent = positions.length;

    const mode = (health.mode || 'paper').toUpperCase();
    const modeBadge = document.getElementById('pos-mode-badge');
    modeBadge.textContent  = mode;
    modeBadge.style.color  = mode === 'LIVE' ? 'var(--red)' : 'var(--accent)';

    let totalUnrealized = 0;
    positions.forEach(p => { totalUnrealized += p.unrealized_pnl || 0; });
    const unEl = document.getElementById('pos-unrealized');
    unEl.textContent  = fmt.usd(totalUnrealized);
    unEl.className    = `kpi-value ${totalUnrealized >= 0 ? 'green' : 'red'}`;

    const realized = data.realized_pnl_today ?? health.realized_pnl ?? null;
    const realEl = document.getElementById('pos-realized');
    realEl.textContent = fmt.usd(realized);
    realEl.className   = `kpi-value ${(realized || 0) >= 0 ? 'green' : 'red'}`;

    const exposure = data.total_exposure ?? null;
    document.getElementById('pos-exposure').textContent =
      exposure != null ? fmt.usd(exposure) : '--';

    const dtLeft = health.day_trades_remaining ?? data.day_trades_remaining ?? '--';
    const dtUsed = health.days_trades_used     ?? data.day_trades_used     ?? '--';
    document.getElementById('pos-dt-left').textContent = dtLeft;
    document.getElementById('pos-dt-used').textContent = `${dtUsed} usados`;

    const cb = health.circuit_breaker_active ?? data.circuit_breaker_active;
    const cbEl = document.getElementById('pos-cb');
    cbEl.textContent = cb ? '⛔ ACTIVO' : '✓ OK';
    cbEl.className   = `kpi-value ${cb ? 'red' : 'green'}`;

    // ── Tabla abiertas ──
    const tbody = document.getElementById('positions-body');
    if (!positions.length) {
      tbody.innerHTML = '<tr class="empty-row"><td colspan="11">Sin posiciones abiertas</td></tr>';
      return;
    }

    tbody.innerHTML = positions.map(p => {
      const pnl    = p.unrealized_pnl || 0;
      const pnlCls = pnl >= 0 ? 'green' : 'red';
      const side   = (p.side || '--').toUpperCase();
      const sideCls = side === 'LONG' || side === 'BUY' ? 'green' : 'red';
      const dur    = _posDuration(p.opened_at ? p.opened_at * 1000 : null);
      const prog   = _posProgress(p.entry_price, p.current_price, p.stop_loss, p.take_profit);
      const broker = p.broker || 'local';
      const brokerColor = broker.includes('alpaca') ? '#9b59b6' :
                          broker.includes('tradier') ? '#00d4aa' : 'var(--text-muted)';
      return `<tr class="pos-row" data-symbol="${p.symbol}">
        <td class="accent" style="font-family:var(--font-mono)">${p.symbol}</td>
        <td class="${sideCls}" style="font-weight:600">${side}</td>
        <td style="font-family:var(--font-mono)">${fmt.usd(p.entry_price)}</td>
        <td style="font-family:var(--font-mono)">${fmt.usd(p.current_price)}</td>
        <td>${fmt.num(p.quantity, 0)}</td>
        <td class="${pnlCls}" style="font-family:var(--font-mono);font-weight:600">${fmt.usd(pnl)}</td>
        <td class="${pnlCls}">${fmt.pct(p.pnl_pct)}</td>
        <td style="font-size:11px;font-family:var(--font-mono)">
          <span class="red">${fmt.usd(p.stop_loss)}</span>
          <span style="color:var(--text-muted)"> / </span>
          <span class="green">${fmt.usd(p.take_profit)}</span>
        </td>
        <td style="min-width:80px">${prog}</td>
        <td style="color:var(--text-muted);font-size:11px">${dur}</td>
        <td>
          <button class="btn-xs red pos-close-btn" data-symbol="${p.symbol}"
                  style="border-color:rgba(245,101,101,0.4)"
                  title="Cerrar posición ${p.symbol}">✕</button>
        </td>
      </tr>`;
    }).join('');

    // Close buttons
    document.querySelectorAll('.pos-close-btn').forEach(btn => {
      btn.addEventListener('click', async () => {
        const sym = btn.dataset.symbol;
        if (!confirm(`¿Cerrar posición ${sym}?`)) return;
        try {
          const r = await apiPost('/positions/close', { symbol: sym, reason: 'manual_dashboard' });
          if (r.ok) {
            toast(`Cierre enviado: ${sym}`, 'success');
            setTimeout(loadPositions, 1000);
          } else {
            toast(`Error cerrando ${sym}: ${r.error}`, 'error');
          }
        } catch (e) {
          toast(`Error: ${e.message}`, 'error');
        }
      });
    });

  } catch (e) {
    toast('Error cargando posiciones: ' + e.message, 'error');
  }
}

async function loadPosHistory() {
  try {
    const r = await QuantAPI.journalEntries(50).catch(() => null);
    const entries = r?.data?.entries || r?.data || [];
    const today = new Date().toISOString().slice(0, 10);
    const todayEntries = entries.filter(e => (e.closed_at || '').startsWith(today));

    const tbody = document.getElementById('pos-history-body');
    if (!todayEntries.length) {
      tbody.innerHTML = '<tr class="empty-row"><td colspan="10">Sin operaciones cerradas hoy</td></tr>';
      return;
    }
    tbody.innerHTML = todayEntries.map(e => {
      const pnl    = e.pnl || 0;
      const pnlCls = pnl >= 0 ? 'green' : 'red';
      const broker = e.broker || 'local';
      const brokerLabel = broker.includes('alpaca') ? '🟣 Alpaca' :
                          broker.includes('tradier') ? '🟢 Tradier' : '⚪ Local';
      return `<tr>
        <td style="font-size:11px;color:var(--text-muted)">${(e.closed_at || '').slice(11, 16)}</td>
        <td class="accent" style="font-family:var(--font-mono)">${e.symbol}</td>
        <td>${(e.side || '--').toUpperCase()}</td>
        <td style="font-family:var(--font-mono)">${fmt.usd(e.entry_price)}</td>
        <td style="font-family:var(--font-mono)">${fmt.usd(e.exit_price)}</td>
        <td class="${pnlCls}" style="font-weight:600">${fmt.usd(pnl)}</td>
        <td class="${pnlCls}">${fmt.pct(e.pnl_pct)}</td>
        <td style="color:var(--text-muted);font-size:11px">${e.duration_min ? e.duration_min + 'm' : '--'}</td>
        <td style="font-size:11px">${e.exit_reason || '--'}</td>
        <td style="font-size:11px">${brokerLabel}</td>
      </tr>`;
    }).join('');
  } catch (e) {
    toast('Error cargando historial: ' + e.message, 'error');
  }
}

function _startPosAutoRefresh() {
  document.getElementById('pos-auto-badge').style.display = 'inline-flex';
  _posRefreshInterval = setInterval(loadPositions, 5000);
}

function _stopPosAutoRefresh() {
  document.getElementById('pos-auto-badge').style.display = 'none';
  clearInterval(_posRefreshInterval);
  _posRefreshInterval = null;
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

  document.getElementById('bt-status').textContent = 'ejecutando…';
  document.getElementById('bt-run').disabled = true;

  try {
    const r = await QuantAPI.runBacktest({
      symbol, timeframe: tf, strategy_type: strategy,
      initial_capital: capital, bars, walk_forward_folds: folds,
      kelly_sizing: kelly, atr_stops: atr, monte_carlo: mc,
      generate_html: true,
    });

    document.getElementById('bt-status').textContent = r.ok ? 'completado' : 'error';

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
      <div class="kpi-card"><span class="kpi-label">PnL Total</span>
        <span class="kpi-value ${(s.total_pnl || 0) >= 0 ? 'green' : 'red'}">${fmt.usd(s.total_pnl)}</span></div>
      <div class="kpi-card"><span class="kpi-label">Tasa de Éxito</span>
        <span class="kpi-value">${fmt.pct(s.win_rate_pct)}</span></div>
      <div class="kpi-card"><span class="kpi-label">Operaciones</span>
        <span class="kpi-value">${s.total_trades || 0}</span></div>
      <div class="kpi-card"><span class="kpi-label">PnL Promedio</span>
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
    badge.textContent  = d.safe_mode ? 'MODO SEGURO' : 'OK';
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
  _posTabInit();
  startClock();
  initWS();
  loadOverview();
  pollHealth();
  setInterval(pollHealth, 30_000);
});
