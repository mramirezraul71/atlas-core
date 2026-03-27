/* ================================================================
   Atlas Code-Quant Dashboard - v1.1.0
   Core app logic aligned with the current Quant backend
   ================================================================ */

const fmt = {
  usd: (v) => v == null || Number.isNaN(Number(v))
    ? '--'
    : `$${Number(v).toLocaleString('en-US', { minimumFractionDigits: 2, maximumFractionDigits: 2 })}`,
  pct: (v) => v == null || Number.isNaN(Number(v)) ? '--' : `${Number(v).toFixed(2)}%`,
  num: (v, d = 3) => v == null || Number.isNaN(Number(v)) ? '--' : Number(v).toFixed(d),
  ts: (v) => v ? new Date(v).toLocaleTimeString() : '--',
};

const QUANT_SCOPE = window.QUANT_ACCOUNT_SCOPE || 'paper';
const QUANT_ACCOUNT_ID = window.QUANT_ACCOUNT_ID || '';

let _equityChart = null;
let _ddChart = null;
let _btEquityChart = null;
let _posRefreshInterval = null;

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

function setText(id, value) {
  const el = document.getElementById(id);
  if (el) el.textContent = value;
}

function setKPI(id, value) {
  const el = document.getElementById(id);
  if (el) el.textContent = value;
}

function syncLabel(reconciliation) {
  const state = reconciliation?.state || 'unknown';
  if (state === 'healthy') return 'SYNC OK';
  if (state === 'degraded') return 'SYNC WARN';
  if (state === 'failed') return 'SYNC FAIL';
  if (state === 'stale') return 'SYNC STALE';
  return 'SYNC --';
}

function renderCanonicalMeta(snapshot) {
  if (!snapshot) return;
  const balances = snapshot.balances || {};
  const totals = snapshot.totals || {};
  setText('chip-source', `${snapshot.source_label || 'Tradier'} · ${snapshot.account_scope || QUANT_SCOPE}`);
  setText('chip-sync', syncLabel(snapshot.reconciliation));
  setText('chip-positions', `${totals.positions || 0} pos`);
  if (balances.total_equity != null) setText('chip-equity', fmt.usd(balances.total_equity));
}

function renderPositionsRealtime(snapshot) {
  if (!snapshot) return;
  const positions = Array.isArray(snapshot.positions) ? snapshot.positions : [];
  const tbody = document.getElementById('positions-body');
  if (!tbody) return;
  setText('pos-count', String(positions.length));
  setText('pos-mode-badge', String(snapshot.account_scope || QUANT_SCOPE).toUpperCase());
  setText('pos-unrealized', fmt.usd(snapshot.total_pnl ?? 0));
  setText('pos-exposure', fmt.usd(snapshot.gross_exposure ?? 0));
  setText('pos-cb', syncLabel(snapshot.reconciliation));
  if (!positions.length) {
    tbody.innerHTML = '<tr class="empty-row"><td colspan="11">Sin posiciones abiertas</td></tr>';
    return;
  }
  tbody.innerHTML = positions.map((position) => {
    const qty = Math.abs(Number(position.quantity || position.signed_qty || 0));
    const pnl = Number(position.unrealized_pnl ?? position.current_pnl ?? 0);
    const pnlCls = pnl >= 0 ? 'green' : 'red';
    const side = String(position.side || '--').toUpperCase();
    const sideCls = side.includes('SHORT') ? 'red' : 'green';
    const strategyLabel = `${position.asset_class || '--'} / ${position.underlying || '--'}`;
    return `<tr title="${strategyLabel}">
      <td class="accent" style="font-family:var(--font-mono)">${position.symbol || position.underlying || '--'}</td>
      <td class="${sideCls}" style="font-weight:600">${side}</td>
      <td style="font-family:var(--font-mono)">${fmt.usd(position.entry_price)}</td>
      <td style="font-family:var(--font-mono)">${fmt.usd(position.current_price)}</td>
      <td>${fmt.num(qty, 0)}</td>
      <td class="${pnlCls}" style="font-family:var(--font-mono);font-weight:600">${fmt.usd(pnl)}</td>
      <td class="${pnlCls}">${fmt.pct(position.pnl_pct)}</td>
      <td style="font-size:11px">${strategyLabel}</td>
      <td>--</td>
      <td>--</td>
      <td style="font-size:11px;color:var(--text-muted)">canonico Tradier</td>
    </tr>`;
  }).join('');
}

function startClock() {
  const el = document.getElementById('sidebar-time');
  const tick = () => {
    if (el) el.textContent = new Date().toTimeString().slice(0, 8);
  };
  tick();
  setInterval(tick, 1000);
}

function initNav() {
  document.querySelectorAll('.nav-item').forEach((item) => {
    item.addEventListener('click', () => {
      const view = item.dataset.view;
      document.querySelectorAll('.nav-item').forEach((n) => n.classList.remove('active'));
      document.querySelectorAll('.view').forEach((v) => v.classList.remove('active'));
      item.classList.add('active');
      document.getElementById(`view-${view}`)?.classList.add('active');
      onViewActivated(view);
    });
  });
}

function onViewActivated(view) {
  if (view !== 'positions') _stopPosAutoRefresh();
  if (view !== 'paper' && window.ppStopAutoRefresh) window.ppStopAutoRefresh();

  switch (view) {
    case 'overview':
      loadOverview();
      break;
    case 'positions':
      loadPositions();
      _startPosAutoRefresh();
      break;
    case 'scanner':
      loadScanner();
      break;
    case 'journal':
      loadJournal();
      break;
    case 'rl':
      loadRL();
      break;
    case 'alerts':
      loadAlerts();
      break;
    case 'paper':
      if (window.loadPaperPlatform) {
        window.loadPaperPlatform();
        if (window.ppStartAutoRefresh) window.ppStartAutoRefresh();
      }
      break;
    default:
      break;
  }
}

function _posTabInit() {
  document.querySelectorAll('[data-pos-tab]').forEach((btn) => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('[data-pos-tab]').forEach((b) => b.classList.remove('active'));
      document.querySelectorAll('.tab-pane').forEach((p) => p.classList.remove('active'));
      btn.classList.add('active');
      document.getElementById(`pos-tab-${btn.dataset.posTab}`)?.classList.add('active');
      if (btn.dataset.posTab === 'history') loadPosHistory();
    });
  });
}

function _startPosAutoRefresh() {
  const badge = document.getElementById('pos-auto-badge');
  if (badge) badge.style.display = 'inline-flex';
  if (_posRefreshInterval) return;
  _posRefreshInterval = setInterval(loadPositions, 5000);
}

function _stopPosAutoRefresh() {
  const badge = document.getElementById('pos-auto-badge');
  if (badge) badge.style.display = 'none';
  if (_posRefreshInterval) clearInterval(_posRefreshInterval);
  _posRefreshInterval = null;
}

async function pollHealth() {
  try {
    const [statusR, monitorR, canonicalR] = await Promise.all([
      QuantAPI.status().catch(() => null),
      QuantAPI.monitorSummary().catch(() => null),
      QuantAPI.canonicalSnapshot(QUANT_SCOPE, QUANT_ACCOUNT_ID).catch(() => null),
    ]);
    const status = statusR?.data || {};
    const monitor = monitorR?.data || {};
    const canonical = canonicalR?.data || null;
    const uptimeSec = status.uptime_sec || 0;
    const equity = canonical?.balances?.total_equity ?? status.account_session?.total_equity ?? monitor.balances?.total_equity ?? null;
    const positions = canonical?.totals?.positions ?? monitor.totals?.positions ?? status.open_positions ?? 0;
    setText('chip-uptime', `${Math.floor(uptimeSec / 60)}m up`);
    if (equity != null) setText('chip-equity', fmt.usd(equity));
    setText('chip-positions', `${positions} pos`);
    if (canonical) renderCanonicalMeta(canonical);
  } catch (_) {
    setText('chip-uptime', 'offline');
  }
}

async function loadOverview() {
  try {
    const [overviewR, statusR, monitorR] = await Promise.all([
      QuantAPI.dashboardOverview().catch(() => null),
      QuantAPI.status().catch(() => null),
      QuantAPI.monitorSummary().catch(() => null),
    ]);

    const m = overviewR?.data || {};
    const status = statusR?.data || {};
    const monitor = monitorR?.data || {};
    const equity = status.account_session?.total_equity ?? monitor.balances?.total_equity ?? m.equity ?? null;
    const positionsCount = monitor.totals?.positions ?? 0;
    const realizedPnl = Number(m.realized_pnl || 0);

    setKPI('kpi-equity', fmt.usd(equity));
    setKPI('kpi-sharpe', fmt.num(m.sharpe_ratio, 2));
    setKPI('kpi-dd', m.max_drawdown_pct != null ? `${fmt.num(m.max_drawdown_pct, 2)}%` : '--');
    setKPI('kpi-wr', fmt.pct(m.win_rate_pct));
    setKPI('kpi-kelly', '--');
    setKPI('kpi-calmar', fmt.num(m.calmar_ratio, 2));
    setText('kpi-trades', `${positionsCount} posiciones`);

    const ddEl = document.getElementById('kpi-dd');
    if (ddEl) ddEl.className = 'kpi-value red';

    const pnlEl = document.getElementById('kpi-equity-delta');
    if (pnlEl) {
      pnlEl.textContent = `${realizedPnl >= 0 ? '+' : ''}${fmt.usd(realizedPnl)} realizado`;
      pnlEl.style.color = realizedPnl >= 0 ? 'var(--green)' : 'var(--red)';
    }

    if (equity != null) setText('chip-equity', fmt.usd(equity));

    const eq = Array.isArray(m.equity_curve) ? m.equity_curve : [];
    if (eq.length) {
      if (!_equityChart) _equityChart = QuantCharts.renderEquityChart('chart-equity', eq);
      else _equityChart.series.setData(eq);
    }

    const dd = Array.isArray(m.drawdown_curve) ? m.drawdown_curve : [];
    if (dd.length) {
      if (!_ddChart) _ddChart = QuantCharts.renderDrawdownChart('chart-drawdown', dd);
      else _ddChart.series.setData(dd);
    }

    if (Array.isArray(m.trade_pnls) && m.trade_pnls.length) {
      QuantCharts.renderPnlDistChart('chart-pnl-dist', m.trade_pnls);
    }
    QuantCharts.renderMonteCarloChart('chart-mc', m.monte_carlo || null);
    if (Array.isArray(m.recent_trades) && m.recent_trades.length) {
      QuantCharts.renderStreakChart('chart-streak', m.recent_trades);
    }
    if (Array.isArray(m.heatmap) && m.heatmap.length) {
      QuantCharts.renderHeatmapChart('chart-heatmap', m.heatmap);
    }
    if (eq.length > 22) {
      QuantCharts.renderRollingSharpeChart('chart-rolling-sharpe', eq);
    }
  } catch (e) {
    toast(`Error cargando overview: ${e.message}`, 'error');
  }
}

function _flattenStrategyPositions(summary) {
  const strategies = Array.isArray(summary?.strategies) ? summary.strategies : [];
  return strategies.flatMap((strategy) =>
    (strategy.positions || []).map((position) => ({
      ...position,
      strategy_type: strategy.strategy_type,
      underlying: strategy.underlying,
    }))
  );
}

async function loadPositions() {
  try {
    const [summaryR, statusR] = await Promise.all([
      QuantAPI.monitorSummary().catch(() => null),
      QuantAPI.status().catch(() => null),
    ]);

    const summary = summaryR?.data || {};
    const status = statusR?.data || {};
    const positions = _flattenStrategyPositions(summary);
    const tbody = document.getElementById('positions-body');
    if (!tbody) return;

    setText('pos-count', String(positions.length));
    setText('pos-mode-badge', String(status.account_session?.scope || 'paper').toUpperCase());
    setText('pos-unrealized', fmt.usd(summary.totals?.open_pnl ?? 0));
    setText('pos-realized', '--');

    const totalExposure = positions.reduce((acc, position) => {
      const qty = Math.abs(Number(position.signed_qty || 0));
      const multiplier = position.asset_class === 'option' ? 100 : 1;
      return acc + Math.abs(Number(position.current_price || 0) * qty * multiplier);
    }, 0);
    setText('pos-exposure', fmt.usd(totalExposure));

    const dtUsed = Number(status.days_trades_used ?? 0);
    const dtLeft = Math.max(0, 3 - dtUsed);
    setText('pos-dt-left', String(dtLeft));
    setText('pos-dt-used', `${dtUsed} usados`);

    const blocked = Boolean(status.pdt_status?.blocked_opening);
    const cbEl = document.getElementById('pos-cb');
    if (cbEl) {
      cbEl.textContent = blocked ? 'BLOQUEADO' : 'OK';
      cbEl.className = `kpi-value ${blocked ? 'red' : 'green'}`;
    }

    const modeBadge = document.getElementById('pos-mode-badge');
    if (modeBadge) {
      modeBadge.style.color = status.account_session?.scope === 'live' ? 'var(--red)' : 'var(--accent)';
    }

    if (!positions.length) {
      tbody.innerHTML = '<tr class="empty-row"><td colspan="11">Sin posiciones abiertas</td></tr>';
      return;
    }

    tbody.innerHTML = positions.map((position) => {
      const qty = Math.abs(Number(position.signed_qty || 0));
      const entry = Number(position.entry_price || 0);
      const current = Number(position.current_price || 0);
      const multiplier = position.asset_class === 'option' ? 100 : 1;
      const entryValue = Math.abs(entry * qty * multiplier);
      const pnl = Number(position.current_pnl || 0);
      const pnlPct = entryValue > 0 ? (pnl / entryValue) * 100 : null;
      const side = String(position.side || '--').toUpperCase();
      const sideCls = side.includes('SHORT') ? 'red' : 'green';
      const pnlCls = pnl >= 0 ? 'green' : 'red';
      const strategyLabel = `${position.strategy_type || '--'} / ${position.underlying || '--'}`;
      return `<tr title="${strategyLabel}">
        <td class="accent" style="font-family:var(--font-mono)">${position.symbol || position.underlying || '--'}</td>
        <td class="${sideCls}" style="font-weight:600">${side}</td>
        <td style="font-family:var(--font-mono)">${fmt.usd(entry)}</td>
        <td style="font-family:var(--font-mono)">${fmt.usd(current)}</td>
        <td>${fmt.num(qty, 0)}</td>
        <td class="${pnlCls}" style="font-family:var(--font-mono);font-weight:600">${fmt.usd(pnl)}</td>
        <td class="${pnlCls}">${fmt.pct(pnlPct)}</td>
        <td style="font-size:11px">${strategyLabel}</td>
        <td>--</td>
        <td>--</td>
        <td style="font-size:11px;color:var(--text-muted)">usar Paper tab</td>
      </tr>`;
    }).join('');
  } catch (e) {
    toast(`Error cargando posiciones: ${e.message}`, 'error');
  }
}

async function loadPosHistory() {
  const tbody = document.getElementById('pos-history-body');
  if (!tbody) return;
  try {
    const response = await QuantAPI.journalEntries(50, 'closed').catch(() => null);
    if (response && response.ok === false) {
      tbody.innerHTML = `<tr class="empty-row"><td colspan="10">Journal no disponible: ${response.error || 'sin detalle'}</td></tr>`;
      return;
    }
    const items = response?.data?.items || response?.data || [];
    const today = new Date().toISOString().slice(0, 10);
    const todayItems = items.filter((item) => String(item.exit_time || item.closed_at || '').startsWith(today));

    if (!todayItems.length) {
      tbody.innerHTML = '<tr class="empty-row"><td colspan="10">Sin operaciones cerradas hoy</td></tr>';
      return;
    }

    tbody.innerHTML = todayItems.map((item) => {
      const pnl = Number(item.realized_pnl ?? item.pnl ?? 0);
      const pnlCls = pnl >= 0 ? 'green' : 'red';
      const entryTime = item.entry_time || item.opened_at || null;
      const exitTime = item.exit_time || item.closed_at || null;
      const durationMin = entryTime && exitTime
        ? Math.max(0, Math.round((new Date(exitTime) - new Date(entryTime)) / 60000))
        : null;
      return `<tr>
        <td style="font-size:11px;color:var(--text-muted)">${exitTime ? exitTime.slice(11, 16) : '--'}</td>
        <td class="accent" style="font-family:var(--font-mono)">${item.symbol || '--'}</td>
        <td>${String(item.strategy_type || item.side || '--').toUpperCase()}</td>
        <td style="font-family:var(--font-mono)">${fmt.usd(item.entry_price)}</td>
        <td style="font-family:var(--font-mono)">${fmt.usd(item.exit_price)}</td>
        <td class="${pnlCls}" style="font-weight:600">${fmt.usd(pnl)}</td>
        <td class="${pnlCls}">${item.entry_notional ? fmt.pct((pnl / Math.max(Math.abs(item.entry_notional), 1)) * 100) : '--'}</td>
        <td style="font-size:11px">${durationMin != null ? `${durationMin}m` : '--'}</td>
        <td style="font-size:11px">${item.post_mortem_text || '--'}</td>
        <td style="font-size:11px">${item.account_type || 'paper'}</td>
      </tr>`;
    }).join('');
  } catch (e) {
    toast(`Error cargando historial: ${e.message}`, 'error');
  }
}

async function loadScanner() {
  try {
    const [statusR, reportR] = await Promise.all([
      QuantAPI.scannerStatus().catch(() => null),
      QuantAPI.scannerReport().catch(() => null),
    ]);
    const status = statusR?.data || {};
    const report = reportR?.data || {};
    const opportunities = report.candidates || report.opportunities || [];
    setText('scanner-status-chip', status.running ? 'ACTIVO' : 'INACTIVO');
    const tbody = document.getElementById('scanner-body');
    if (!tbody) return;
    if (!opportunities.length) {
      tbody.innerHTML = '<tr class="empty-row"><td colspan="7">Sin oportunidades detectadas</td></tr>';
      return;
    }
    tbody.innerHTML = opportunities.map((op) => {
      const score = Number(op.selection_score ?? op.score ?? 0);
      const winRate = Number(op.win_rate_pct ?? op.win_probability ?? 0);
      const risk = Number(op.var_95 ?? op.risk_pct ?? 0);
      const signal = op.side || op.direction || op.signal || '--';
      const strategy = op.strategy_type || op.strategy || '--';
      return `<tr>
        <td class="accent">${op.symbol || '--'}</td>
        <td>${strategy}</td>
        <td class="${score >= 0.7 ? 'accent' : ''}">${fmt.num(score, 2)}</td>
        <td>${String(signal).toUpperCase()}</td>
        <td>${fmt.pct(winRate)}</td>
        <td class="red">${fmt.pct(risk)}</td>
        <td>${fmt.ts(op.generated_at || op.timestamp)}</td>
      </tr>`;
    }).join('');
  } catch (e) {
    toast(`Error cargando scanner: ${e.message}`, 'error');
  }
}

function _normalizedBacktestStrategy(selected) {
  const map = {
    momentum: 'ma_cross',
    mean_reversion: 'ma_cross',
    breakout: 'ma_cross',
    rl: 'ml_rf',
  };
  return map[selected] || selected || 'ma_cross';
}

async function runBacktest() {
  const symbol = document.getElementById('bt-symbol')?.value?.trim() || 'SPY';
  const timeframe = document.getElementById('bt-tf')?.value || '1h';
  const strategy = _normalizedBacktestStrategy(document.getElementById('bt-strategy')?.value);
  const capital = parseFloat(document.getElementById('bt-capital')?.value || '10000');
  const limit = parseInt(document.getElementById('bt-bars')?.value || '500', 10);
  const statusEl = document.getElementById('bt-status');
  const btn = document.getElementById('bt-run');

  if (statusEl) statusEl.textContent = 'ejecutando...';
  if (btn) btn.disabled = true;

  try {
    const response = await QuantAPI.runBacktest({
      symbol,
      timeframe,
      strategy,
      capital,
      limit,
      generate_html: true,
    });

    if (statusEl) statusEl.textContent = response.ok ? 'completado' : 'error';
    if (!response.ok || !response.data) {
      toast(`Error en backtest: ${response.error || 'desconocido'}`, 'error');
      return;
    }

    const m = response.data.metrics || {};
    setKPI('bm-total-ret', fmt.pct(m.total_return_pct));
    setKPI('bm-log-ret', fmt.num(m.cumulative_log_return, 4));
    setKPI('bm-sharpe', fmt.num(m.sharpe_ratio, 2));
    setKPI('bm-sortino', fmt.num(m.sortino_ratio, 2));
    setKPI('bm-calmar', fmt.num(m.calmar_ratio, 2));
    setKPI('bm-maxdd', fmt.pct(m.max_drawdown_pct != null ? m.max_drawdown_pct * 100 : null));
    setKPI('bm-wr', fmt.pct(m.win_rate_pct));
    setKPI('bm-pf', fmt.num(m.profit_factor, 2));
    setKPI('bm-trades', m.total_trades ?? '--');
    setKPI('bm-var', fmt.pct(m.mc_var_95));
    setKPI('bm-cvar', fmt.pct(m.mc_cvar_95));
    setKPI('bm-kelly', fmt.pct(m.kelly_quarter ? m.kelly_quarter * 100 : null));

    const eq = m.equity_curve || response.data.equity_curve || [];
    if (eq.length) {
      if (_btEquityChart) _btEquityChart.series.setData(eq);
      else _btEquityChart = QuantCharts.renderEquityChart('bt-chart-equity', eq);
    }

    const panel = document.getElementById('bt-trades-panel');
    const tbody = document.getElementById('bt-trades-body');
    const trades = Array.isArray(response.data.trades) ? response.data.trades : [];
    if (panel && tbody) {
      panel.style.display = trades.length ? 'block' : 'none';
      tbody.innerHTML = trades.map((trade, index) => {
        const pnl = Number(trade.pnl || 0);
        const pnlCls = pnl >= 0 ? 'green' : 'red';
        return `<tr>
          <td>${index + 1}</td>
          <td class="accent">${trade.symbol || symbol}</td>
          <td>${trade.side || '--'}</td>
          <td>${fmt.usd(trade.entry_price)}</td>
          <td>${fmt.usd(trade.exit_price)}</td>
          <td class="${pnlCls}">${fmt.usd(pnl)}</td>
          <td class="${pnlCls}">${fmt.pct(trade.pnl_pct)}</td>
          <td>${fmt.num(trade.log_return, 4)}</td>
          <td>${trade.exit_reason || '--'}</td>
        </tr>`;
      }).join('');
    }

    const reportPath = response.data.report_path;
    const link = document.getElementById('bt-report-link');
    if (link && reportPath) link.href = `/reports/${reportPath.split('/').pop()}`;
    toast('Backtest completado', 'success');
  } catch (e) {
    if (statusEl) statusEl.textContent = 'error';
    toast(`Error en backtest: ${e.message}`, 'error');
  } finally {
    if (btn) btn.disabled = false;
  }
}

async function loadJournal() {
  try {
    const [statsR, entriesR] = await Promise.all([
      QuantAPI.journalStats().catch(() => null),
      QuantAPI.journalEntries(100).catch(() => null),
    ]);

    const paper = statsR?.data?.accounts?.paper || {};
    const items = entriesR?.data?.items || entriesR?.data || [];
    const chartEntries = items.map((item) => ({
      closed_at: item.exit_time || item.closed_at,
      pnl: item.realized_pnl || item.pnl,
      strategy: item.strategy_type || item.strategy,
    }));

    const kpiRow = document.getElementById('journal-kpi');
    if (kpiRow) {
      kpiRow.innerHTML = `
        <div class="kpi-card"><span class="kpi-label">PnL Total</span>
          <span class="kpi-value ${(paper.realized_pnl || 0) >= 0 ? 'green' : 'red'}">${fmt.usd(paper.realized_pnl)}</span></div>
        <div class="kpi-card"><span class="kpi-label">Tasa de Exito</span>
          <span class="kpi-value">${fmt.pct(paper.win_rate_pct)}</span></div>
        <div class="kpi-card"><span class="kpi-label">Cerradas</span>
          <span class="kpi-value">${paper.trades_closed || 0}</span></div>
        <div class="kpi-card"><span class="kpi-label">Abiertas</span>
          <span class="kpi-value">${paper.trades_open || 0}</span></div>
      `;
    }

    if (chartEntries.length) {
      QuantCharts.renderJournalPnlChart('chart-journal-pnl', chartEntries);
      QuantCharts.renderJournalStratChart('chart-journal-strat', chartEntries);
    }

    const tbody = document.getElementById('journal-body');
    if (!tbody) return;
    tbody.innerHTML = items.length
      ? items.map((item) => {
        const pnl = Number(item.realized_pnl || item.pnl || 0);
        const pnlCls = pnl >= 0 ? 'green' : 'red';
        const entryTime = item.entry_time || null;
        const exitTime = item.exit_time || item.closed_at || null;
        const durationMin = entryTime && exitTime
          ? Math.max(0, Math.round((new Date(exitTime) - new Date(entryTime)) / 60000))
          : null;
        return `<tr>
          <td>${exitTime ? exitTime.slice(0, 16) : '--'}</td>
          <td class="accent">${item.symbol || '--'}</td>
          <td>${item.strategy_type || item.strategy || '--'}</td>
          <td>${item.account_type || '--'}</td>
          <td class="${pnlCls}">${fmt.usd(pnl)}</td>
          <td>${durationMin != null ? `${durationMin}m` : '--'}</td>
          <td>${item.post_mortem_text || '--'}</td>
        </tr>`;
      }).join('')
      : '<tr class="empty-row"><td colspan="7">Sin entradas</td></tr>';
  } catch (e) {
    toast(`Error cargando journal: ${e.message}`, 'error');
  }
}

async function loadRL() {
  const symbol = document.getElementById('rl-symbol-sel')?.value || 'BTC/USDT';
  try {
    const r = await QuantAPI.retrainingStatus(symbol);
    const d = r.data || {};
    setText('rl-symbol', d.symbol || symbol);
    setText('rl-state', d.running ? 'ACTIVO' : 'INACTIVO');
    setText('rl-last', d.last_retrain_at ? new Date(d.last_retrain_at * 1000).toLocaleString() : 'Nunca');
    setText('rl-next', d.next_retrain_in_h ? `en ${d.next_retrain_in_h.toFixed(1)}h` : '--');
    setText('rl-sharpe', fmt.num(d.best_sharpe, 3));
    setText('rl-trials', d.optuna_trials || '--');

    const models = d.models || [];
    const mlEl = document.getElementById('rl-models-list');
    if (!mlEl) return;
    mlEl.innerHTML = models.length
      ? models.map((m, i) => `<div class="model-entry ${i === 0 ? 'best' : ''}">
          <span class="model-name">${m.name || m}</span>
          <span class="model-sharpe">Sharpe ${fmt.num(m.sharpe, 3)}</span>
        </div>`).join('')
      : '<span class="empty-text">Sin modelos entrenados</span>';
  } catch (e) {
    toast(`Error cargando RL status: ${e.message}`, 'error');
  }
}

async function captureVisualState() {
  try {
    const r = await QuantAPI.visualState();
    if (!r.ok) {
      const fallback = await QuantAPI.visionCalibrationStatus().catch(() => null);
      if (fallback?.ok) {
        const d = fallback.data || {};
        setText('vs-brightness', '--');
        setText('vs-sharpness', '--');
        setText('vs-screens', d.monitor ? '1' : '--');
        setText('vs-color', d.calibration_exists ? 'calibrado' : '--');
        setText('vs-prices', 'no disponible');
        setText('vs-pattern', d.label || 'calibracion');
        const badge = document.getElementById('visual-safety-badge');
        if (badge) {
          badge.textContent = d.session_active ? 'CALIBRADO' : 'SIN SESION';
          badge.className = `mode-badge ${d.session_active ? 'ok' : 'safe'}`;
        }
        toast('Estado visual no disponible; se mostró estado de calibración', 'warning');
        return;
      }
      toast(`Error capturando visual: ${r.error}`, 'error');
      return;
    }
    const d = r.data || {};
    setText('vs-brightness', fmt.num(d.brightness, 1));
    setText('vs-sharpness', fmt.num(d.sharpness, 1));
    setText('vs-screens', d.screens_detected ?? '--');
    setText('vs-color', d.chart_color || '--');
    setText('vs-prices', (d.ocr_prices || d.prices || []).join(', ') || 'ninguno');
    setText('vs-pattern', d.pattern_detected || 'ninguno');
    const badge = document.getElementById('visual-safety-badge');
    if (badge) {
      const safeMode = Boolean(d.safe_mode);
      badge.textContent = safeMode ? 'MODO SEGURO' : 'OK';
      badge.className = `mode-badge ${safeMode ? 'safe' : 'ok'}`;
    }
    if (d.feature_vector?.length) {
      QuantCharts.renderVisualFeaturesChart('chart-visual-features', d.feature_vector);
    }
    toast('Estado visual capturado', 'success');
  } catch (e) {
    toast(`Error: ${e.message}`, 'error');
  }
}

async function loadAlerts() {
  try {
    const r = await QuantAPI.alertsStatus();
    const d = r.data || {};
    setText('al-telegram', d.telegram_enabled ? 'ON' : 'OFF');
    setText('al-whatsapp', d.whatsapp_enabled ? 'ON' : 'OFF');
    setText('al-queue', d.queue_size ?? '--');
    setText('al-sent', d.sent_today ?? d.sent_count ?? '--');
    setText('al-level', d.min_level || '--');
    setText('al-cooldown', d.cooldown_s ? `${d.cooldown_s}s` : '--');
  } catch (e) {
    toast(`Error cargando alertas: ${e.message}`, 'error');
  }
}

function appendLog(id, line) {
  const el = document.getElementById(id);
  if (!el) return;
  if (el.textContent === 'Esperando actividad...' || el.textContent === 'Sin alertas registradas') {
    el.textContent = '';
  }
  el.textContent += `${line}\n`;
  el.scrollTop = el.scrollHeight;
}

function initWS() {
  quantWS.on('quant.live_update', (msg) => {
    const canonical = msg.canonical_snapshot || null;
    const status = msg.status || {};
    const monitor = msg.monitor_summary || {};
    const equity = canonical?.balances?.total_equity ?? status.account_session?.total_equity ?? monitor.balances?.total_equity ?? null;
    const positions = canonical?.totals?.positions ?? monitor.totals?.positions ?? status.open_positions ?? 0;
    if (equity != null) setText('chip-equity', fmt.usd(equity));
    setText('chip-positions', `${positions} pos`);
    if (canonical) {
      renderCanonicalMeta(canonical);
      if (document.getElementById('view-positions')?.classList.contains('active')) {
        renderPositionsRealtime({
          ...canonical,
          total_pnl: canonical?.totals?.open_pnl ?? 0,
          gross_exposure: canonical?.balances?.gross_exposure ?? 0,
        });
      }
    }
  });

  quantWS.on('quant.live_error', (msg) => {
    document.getElementById('ws-dot')?.classList.add('error');
    appendLog('alerts-log', `[${new Date().toLocaleTimeString()}] WS error: ${msg.error || 'desconocido'}`);
  });

  quantWS.connect();
}

document.addEventListener('click', (e) => {
  if (e.target.dataset.range) {
    document.querySelectorAll('.chart-controls .btn-xs').forEach((b) => b.classList.remove('active'));
    e.target.classList.add('active');
  }
});

document.getElementById('scanner-start')?.addEventListener('click', async () => {
  await QuantAPI.scannerStart();
  await loadScanner();
  toast('Scanner iniciado', 'success');
});

document.getElementById('scanner-stop')?.addEventListener('click', async () => {
  await QuantAPI.scannerStop();
  await loadScanner();
  toast('Scanner detenido');
});

document.getElementById('bt-run')?.addEventListener('click', runBacktest);
document.getElementById('journal-refresh')?.addEventListener('click', loadJournal);
document.getElementById('rl-status-btn')?.addEventListener('click', loadRL);
document.getElementById('rl-trigger-btn')?.addEventListener('click', async () => {
  const symbol = document.getElementById('rl-symbol-sel')?.value || 'BTC/USDT';
  const r = await QuantAPI.retrainingTrigger(symbol);
  if (r.ok) {
    toast(`Retraining iniciado para ${symbol}`, 'success');
    appendLog('rl-log-box', `[${new Date().toLocaleTimeString()}] Ciclo de retraining forzado para ${symbol}`);
  } else {
    toast(`Error: ${r.error || 'desconocido'}`, 'error');
  }
  loadRL();
});

document.getElementById('visual-capture')?.addEventListener('click', captureVisualState);
document.getElementById('alerts-refresh-btn')?.addEventListener('click', loadAlerts);
document.getElementById('alerts-test-btn')?.addEventListener('click', async () => {
  const r = await QuantAPI.alertsTest('Test desde Atlas Code-Quant Dashboard v1.1.0');
  if (r.ok) {
    appendLog('alerts-log', `[${new Date().toLocaleTimeString()}] Alerta de prueba enviada`);
    toast('Alerta enviada', 'success');
  } else {
    toast(`Error: ${r.error || '?'}`, 'error');
  }
});

document.getElementById('btn-emergency')?.addEventListener('click', async () => {
  if (!confirm('Activar Emergency Stop? Cerrara las posiciones autonomas activas.')) return;
  try {
    const r = await apiPost('/api/v2/quant/emergency/stop', { reason: 'manual_dashboard' });
    toast(r.ok ? 'Emergency Stop activado' : `Error: ${r.error}`, r.ok ? 'error' : 'error', 5000);
  } catch (e) {
    toast(`Error: ${e.message}`, 'error', 5000);
  }
});

document.getElementById('ov-refresh')?.addEventListener('click', loadOverview);
document.getElementById('pos-refresh')?.addEventListener('click', loadPositions);

document.addEventListener('DOMContentLoaded', () => {
  initNav();
  _posTabInit();
  startClock();
  initWS();
  loadOverview();
  pollHealth();
  setInterval(pollHealth, 10000);
});
