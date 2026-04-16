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
  if (window.AtlasCharts && view !== 'analytics') {
    AtlasCharts.hide();
  }
  switch (view) {
    case 'overview':   loadOverview(); break;
    case 'positions':  loadPositions(); break;
    case 'scanner':    loadScanner(); break;
    case 'backtest':   /* user-triggered */ break;
    case 'journal':    loadJournal(); break;
    case 'rl':         loadRL(); break;
    case 'visual':     loadVisual(); break;
    case 'alerts':     loadAlerts(); break;
    case 'analytics':
      if (window.AtlasCharts) {
        if (window.__atlasChartsStarted) AtlasCharts.show();
        else {
          window.__atlasChartsStarted = true;
          AtlasCharts.init();
        }
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

function setElText(id, text) {
  const el = document.getElementById(id);
  if (el) el.textContent = text;
}

const QUANT_SCOPE = window.QUANT_ACCOUNT_SCOPE || 'paper';
const QUANT_ACCOUNT_ID = window.QUANT_ACCOUNT_ID || '';
window._quantWsConnected = false;
let _latestCanonicalSnapshot = null;
let _lastViewRefreshAt = 0;
let _healthRequest = null;
let _overviewRequest = null;
let _journalRequest = null;
let _positionsRequest = null;
let _scannerRequest = null;
let _rlRequest = null;
let _alertsRequest = null;
let _visualRequest = null;
let _latestPositionsSnapshot = null;
let _latestScannerState = null;
let _latestJournalState = null;
let _latestAlertsState = null;
let _latestVisualState = null;
let _lastOverviewSuccessAt = 0;
let _healthFailures = 0;
let _forcePollingMode = false;
window._quantPollingMode = false;
let _lastSnapshotWasLightweight = false;
let _lastReconciliation = null;
let _lastCanonicalGeneratedAt = null;
let _realtimeEquityCurve = [];
const _realtimeMaxPoints = 360;

function colorClass(val) {
  if (val == null) return '';
  return parseFloat(val) >= 0 ? 'green' : 'red';
}

function syncLabel(reconciliation, snapshot = null) {
  const state = reconciliation?.state || 'unknown';
  const lightweight = isLightweightSnapshot(snapshot);
  if (lightweight && (state === 'failed' || state === 'stale' || state === 'unknown')) {
    return 'SYNC LITE';
  }
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

function isSparseOperationalSnapshot(snapshot) {
  if (!snapshot || typeof snapshot !== 'object') return true;
  const hasBalances = snapshot?.balances && Object.keys(snapshot.balances).length > 0;
  const hasTotals = snapshot?.totals && Object.keys(snapshot.totals).length > 0;
  const hasPositions = Array.isArray(snapshot?.positions) && snapshot.positions.length > 0;
  return !hasBalances && !hasTotals && !hasPositions;
}

function renderCanonicalMeta(snapshot) {
  if (!snapshot) return;
  _lastSnapshotWasLightweight = isLightweightSnapshot(snapshot);
  const sparseSnapshot = isSparseOperationalSnapshot(snapshot);
  if (!sparseSnapshot) {
    _latestCanonicalSnapshot = snapshot;
  } else if (_latestCanonicalSnapshot && !isSparseOperationalSnapshot(_latestCanonicalSnapshot)) {
    snapshot = {
      ..._latestCanonicalSnapshot,
      ...snapshot,
      balances: _latestCanonicalSnapshot.balances,
      totals: _latestCanonicalSnapshot.totals,
      positions: _latestCanonicalSnapshot.positions,
      simulators: snapshot.simulators || _latestCanonicalSnapshot.simulators,
      reconciliation: snapshot.reconciliation || _latestCanonicalSnapshot.reconciliation,
    };
  }
  const totals = snapshot.totals || {};
  const balances = snapshot.balances || {};
  const reconciliation = snapshot.reconciliation || {};
  _lastReconciliation = reconciliation;
  _lastCanonicalGeneratedAt = snapshot.generated_at || _lastCanonicalGeneratedAt;
  const sourceLabel = snapshot.lightweight_mode
    ? `${snapshot.source_label || 'Quant'} · lightweight`
    : (snapshot.source_label || 'Tradier');
  setElText('chip-source', `${sourceLabel} · ${snapshot.account_scope || QUANT_SCOPE}`);
  updateSyncChipState(syncLabel(reconciliation, snapshot));
  updateWsModeChip();
  setElText('chip-positions', `${totals.positions || 0} pos`);
  if (balances.total_equity != null) {
    setElText('chip-equity', fmt.usd(balances.total_equity));
  }
  const ovSource = document.getElementById('ov-source-badge');
  if (ovSource) ovSource.textContent = `Operativa ${snapshot.source_label || 'Tradier'}`;
  const ovSim = document.getElementById('ov-sim-badge');
  if (ovSim) ovSim.textContent = simulatorSummary(snapshot.simulators);
  const posSource = document.getElementById('pos-source-badge');
  if (posSource) posSource.textContent = `Fuente ${snapshot.source_label || 'Tradier'}`;
  const posSync = document.getElementById('pos-sync-badge');
  if (posSync) posSync.textContent = syncLabel(reconciliation, snapshot);
}

function isLightweightSnapshot(snapshot) {
  return Boolean(snapshot?.lightweight_mode || snapshot?.source === 'lightweight');
}

function updateWsModeChip() {
  const chip = document.getElementById('chip-ws-mode');
  if (!chip) return;
  let text = 'WS: --';
  let klass = 'stat-chip';
  const inferredLitePolling = !window._quantWsConnected && _lastSnapshotWasLightweight;
  if (window._quantPollingMode || inferredLitePolling) {
    text = _lastSnapshotWasLightweight ? 'WS: POLLING (LITE)' : 'WS: POLLING';
    klass = 'stat-chip warn';
    const wsDot = document.getElementById('ws-dot');
    if (wsDot && !wsDot.classList.contains('connected')) {
      wsDot.classList.remove('error');
      wsDot.classList.add('polling');
      wsDot.title = 'Modo polling activo';
    }
  } else if (window._quantWsConnected) {
    text = 'WS: LIVE';
    klass = 'stat-chip accent';
  } else {
    text = 'WS: RECONNECT';
  }
  chip.className = klass;
  chip.textContent = text;
  updateSyncChipState();
}

function updateSyncChipState(nextText = null) {
  const chip = document.getElementById('chip-sync');
  if (!chip) return;

  let text = String(nextText || chip.textContent || 'SYNC --').trim() || 'SYNC --';
  let klass = 'stat-chip';

  if (_backendOnline === false) {
    text = 'SYNC OFFLINE';
    klass = 'stat-chip danger';
  } else if (window._quantPollingMode) {
    if (_lastSnapshotWasLightweight && (text === 'SYNC --' || text === 'SYNC FAIL' || text === 'SYNC STALE')) {
      text = 'SYNC LITE';
    } else if (!_lastSnapshotWasLightweight && text === 'SYNC --') {
      text = 'SYNC WARN';
    }
    klass = 'stat-chip warn';
  } else {
    if (text === 'SYNC --' && window._quantWsConnected) {
      text = 'SYNC OK';
    }
    if (text.includes('FAIL') || text.includes('OFFLINE') || text.includes('STALE')) {
      klass = 'stat-chip danger';
    } else if (text.includes('WARN') || text.includes('LITE')) {
      klass = 'stat-chip warn';
    } else if (text.includes('OK')) {
      klass = 'stat-chip accent';
    }
  }

  chip.className = klass;
  chip.textContent = text;
  chip.title = buildSyncChipTooltip(text);
}

function buildSyncChipTooltip(syncText) {
  const rec = _lastReconciliation || {};
  if (_backendOnline === false) {
    return 'Backend offline en puerto 8795. Reintentando health-check y reconexion WS.';
  }

  const details = [];
  details.push(`Estado: ${syncText}`);

  if (window._quantPollingMode) {
    details.push(_lastSnapshotWasLightweight ? 'Transporte: polling (lightweight)' : 'Transporte: polling');
  } else if (window._quantWsConnected) {
    details.push('Transporte: websocket live');
  } else {
    details.push('Transporte: reconnect');
  }

  if (rec.state) details.push(`Reconciliation: ${rec.state}`);
  const reason = rec.reason || rec.reason_code || rec.message;
  if (reason) details.push(`Motivo: ${reason}`);

  const lagCandidate = rec.lag_ms ?? rec.staleness_ms ?? rec.age_ms;
  const lagValue = Number(lagCandidate);
  if (Number.isFinite(lagValue)) details.push(`Lag: ${Math.round(lagValue)} ms`);

  const updatedAt = rec.updated_at || rec.generated_at || rec.last_success_at || _lastCanonicalGeneratedAt;
  if (updatedAt) {
    const parsed = new Date(updatedAt);
    if (!Number.isNaN(parsed.getTime())) {
      details.push(`Actualizado: ${parsed.toLocaleString()}`);
    }
  }

  return details.join(' | ');
}

function setEmptyRow(tbodyId, colspan, message) {
  const tbody = document.getElementById(tbodyId);
  if (!tbody) return;
  tbody.innerHTML = `<tr class="empty-row"><td colspan="${colspan}">${message}</td></tr>`;
}

function markDegradedChip(id, label) {
  const el = document.getElementById(id);
  if (el) el.textContent = label;
}

function useCachedModuleState(message) {
  toast(message, 'warning', 2500);
}

function enablePollingOnlyMode(reason = 'lightweight_mode') {
  if (_forcePollingMode) return;
  _forcePollingMode = true;
  window._quantPollingMode = true;
  _lastSnapshotWasLightweight = true;
  const wsDot = document.getElementById('ws-dot');
  if (wsDot) {
    wsDot.classList.remove('connected', 'error');
    wsDot.classList.add('polling');
    wsDot.title = 'Modo polling activo (WS deshabilitado por snapshot lightweight)';
  }
  updateWsModeChip();
  try {
    quantWS.close();
  } catch (_) {}
  console.warn(`Atlas dashboard switched to polling-only mode: ${reason}`);
}

function notifyAtlas(method, ...args) {
  try {
    const notifier = window.AtlasNotifications;
    if (!notifier || !notifier.enabled || typeof notifier[method] !== 'function') return false;
    return Boolean(notifier[method](...args));
  } catch (_) {
    return false;
  }
}

function hasInFlightModuleRequest() {
  return Boolean(
    _overviewRequest ||
    _journalRequest ||
    _positionsRequest ||
    _scannerRequest ||
    _rlRequest ||
    _alertsRequest ||
    _visualRequest
  );
}

let _lightweightWarned = false;
function maybeWarnLightweight(snapshot) {
  if (!isLightweightSnapshot(snapshot) || _lightweightWarned) return;
  enablePollingOnlyMode('lightweight_snapshot');
  _lightweightWarned = true;
  toast('Quant está en modo lightweight: la analítica rica queda diferida hasta relanzar el motor completo.', 'error', 6000);
}

function exitRecommendationLabel(value) {
  const normalized = String(value || 'hold').toLowerCase();
  if (normalized === 'exit_now') return 'Salir ya';
  if (normalized === 'de_risk') return 'Reducir';
  if (normalized === 'take_profit') return 'Tomar profit';
  return 'Mantener';
}

function urgencyLabel(value) {
  const normalized = String(value || 'low').toLowerCase();
  if (normalized === 'high') return 'Alta';
  if (normalized === 'medium') return 'Media';
  return 'Baja';
}

function exitReasonLabel(value, reasons = []) {
  const normalized = String(value || '').toLowerCase();
  if (normalized === 'hard_stop_loss_r') return 'Stop R';
  if (normalized === 'hard_dollar_loss') return 'Pérdida USD';
  if (normalized === 'thesis_invalidated') return 'Tesis rota';
  if (normalized === 'time_stop') return 'Time stop';
  if (normalized === 'profit_target') return 'Take profit';
  if (normalized === 'book_concentration') return 'Concentración';
  if (normalized === 'protect_open_profit') return 'Proteger ganancia';
  if (Array.isArray(reasons) && reasons.length) return reasons.join(', ');
  return '--';
}

function renderPositionsTable(snapshot) {
  const positions = snapshot?.positions || [];
  const tbody = document.getElementById('positions-body');
  if (!tbody) return;
  if (Array.isArray(snapshot?.positions)) {
    _latestPositionsSnapshot = snapshot;
  }

  const exitSummary = snapshot?.exit_governance?.summary || {};
  document.getElementById('pos-count').textContent = positions.length;
  document.getElementById('pos-exit-now').textContent = exitSummary.exit_now_count || 0;
  document.getElementById('pos-de-risk').textContent = exitSummary.de_risk_count || 0;

  if (!positions.length) {
    tbody.innerHTML = '<tr class="empty-row"><td colspan="15">Sin posiciones abiertas</td></tr>';
    document.getElementById('pos-unrealized').textContent = fmt.usd(0);
    document.getElementById('pos-exposure').textContent = fmt.usd(snapshot?.gross_exposure || 0);
    document.getElementById('pos-cb').textContent = syncLabel(snapshot?.reconciliation, snapshot);
    return;
  }

  let totalUnrealized = 0;
  tbody.innerHTML = positions.map((p) => {
    const pnl = p.unrealized_pnl || 0;
    totalUnrealized += pnl;
    const pnlCls = pnl >= 0 ? 'green' : 'red';
    const recommendation = exitRecommendationLabel(p.exit_recommendation);
    const urgency = urgencyLabel(p.exit_urgency);
    const reason = exitReasonLabel(p.exit_reason, p.alert_reasons || []);
    const openR = p.open_r_multiple == null ? '--' : fmt.num(p.open_r_multiple, 2);
    const recommendationClass =
      String(p.exit_recommendation || '').toLowerCase() === 'exit_now'
        ? 'red'
        : String(p.exit_recommendation || '').toLowerCase() === 'de_risk'
          ? 'accent'
          : String(p.exit_recommendation || '').toLowerCase() === 'take_profit'
            ? 'green'
            : '';
    return `<tr>
      <td class="accent">${p.symbol}</td>
      <td>${p.side || '--'}</td>
      <td>${fmt.usd(p.entry_price)}</td>
      <td>${fmt.usd(p.current_price)}</td>
      <td>${fmt.num(p.quantity, 4)}</td>
      <td class="${pnlCls}">${fmt.usd(pnl)}</td>
      <td class="${pnlCls}">${fmt.pct(p.pnl_pct)}</td>
      <td>${fmt.num(p.log_return, 4)}</td>
      <td class="red">${fmt.usd(p.stop_loss)}</td>
      <td class="green">${fmt.usd(p.take_profit)}</td>
      <td>${fmt.num(p.atr, 4)}</td>
      <td>${openR}</td>
      <td class="${recommendationClass}">${recommendation}</td>
      <td>${urgency}</td>
      <td title="${reason}">${reason}</td>
    </tr>`;
  }).join('');

  document.getElementById('pos-unrealized').textContent = fmt.usd(totalUnrealized);
  document.getElementById('pos-unrealized').className =
    `kpi-value ${totalUnrealized >= 0 ? 'green' : 'red'}`;
  document.getElementById('pos-exposure').textContent = fmt.usd(snapshot?.gross_exposure || 0);
  document.getElementById('pos-cb').textContent = syncLabel(snapshot?.reconciliation, snapshot);
}

function renderOverviewHistory(meta) {
  const ovHistory = document.getElementById('ov-history-badge');
  if (!ovHistory) return;
  const label = meta?.historical_source?.label || 'Histórico --';
  ovHistory.textContent = label;
}

function pickJournalAccount(statsPayload, scope = QUANT_SCOPE) {
  const accounts = statsPayload?.accounts || {};
  return accounts?.[scope] || accounts?.paper || accounts?.live || {};
}

function normalizeJournalEntries(entriesPayload) {
  const items = Array.isArray(entriesPayload?.items)
    ? entriesPayload.items
    : Array.isArray(entriesPayload)
      ? entriesPayload
      : [];

  return items.map((entry) => {
    const realized = Number(entry?.realized_pnl ?? entry?.pnl ?? 0);
    const unrealized = Number(entry?.unrealized_pnl ?? 0);
    return {
      ...entry,
      strategy: entry?.strategy || entry?.strategy_id || entry?.strategy_type || '--',
      side: entry?.side || '--',
      pnl: Number.isFinite(realized) ? realized : 0,
      total_pnl: (Number.isFinite(realized) ? realized : 0) + (Number.isFinite(unrealized) ? unrealized : 0),
      closed_at: entry?.closed_at || entry?.exit_time || entry?.updated_at || entry?.entry_time || '',
      duration_min: entry?.duration_min ?? null,
      exit_reason: entry?.exit_reason || entry?.status || '--',
    };
  });
}

function buildOverviewFromJournal(statsPayload, chartPayload, canonicalData = null) {
  const acct = pickJournalAccount(statsPayload, QUANT_SCOPE);
  const trades = Array.isArray(chartPayload?.trades) ? chartPayload.trades : [];
  const equityCurve = [];
  const drawdownCurve = [];
  const tradePnls = [];
  let maxDrawdownPct = null;

  trades.forEach((trade) => {
    const exitTime = String(trade?.exit_time || '').trim();
    if (!exitTime) return;
    const parsed = new Date(exitTime);
    if (Number.isNaN(parsed.getTime())) return;
    const equityValue = Number(trade?.equity ?? 0);
    const drawdownValue = Number(trade?.drawdown_pct ?? 0);
    const pnlValue = Number(trade?.pnl ?? 0);
    const unixTs = Math.floor(parsed.getTime() / 1000);
    equityCurve.push({ time: unixTs, value: Number(equityValue.toFixed(2)) });
    drawdownCurve.push({ time: unixTs, value: Number(drawdownValue.toFixed(4)) });
    tradePnls.push(Number(pnlValue.toFixed(2)));
    if (maxDrawdownPct == null || drawdownValue < maxDrawdownPct) {
      maxDrawdownPct = drawdownValue;
    }
  });

  const totals = canonicalData?.totals || {
    positions: acct?.trades_open || 0,
    open_pnl: acct?.unrealized_pnl || 0,
  };
  const balances = canonicalData?.balances || {};
  const realizedPnl = Number(acct?.realized_pnl ?? 0);
  const unrealizedPnl = Number(totals?.open_pnl ?? acct?.unrealized_pnl ?? 0);
  let calmarRatio = null;
  if (maxDrawdownPct != null && maxDrawdownPct < -0.001) {
    calmarRatio = Number((realizedPnl / Math.abs(maxDrawdownPct)).toFixed(3));
  }

  return {
    generated_at: canonicalData?.generated_at || chartPayload?.generated_at || new Date().toISOString(),
    account_scope: QUANT_SCOPE,
    account_id: QUANT_ACCOUNT_ID,
    equity: balances?.total_equity ?? null,
    realized_pnl: Number(realizedPnl.toFixed(2)),
    unrealized_pnl: Number(unrealizedPnl.toFixed(2)),
    sharpe_ratio: acct?.sharpe_ratio ?? null,
    win_rate_pct: acct?.win_rate_pct ?? null,
    profit_factor: acct?.profit_factor ?? null,
    expectancy: acct?.expectancy ?? null,
    total_trades: acct?.trades_closed ?? trades.length,
    open_positions: totals?.positions ?? 0,
    max_drawdown_pct: maxDrawdownPct != null ? Number(maxDrawdownPct.toFixed(3)) : null,
    calmar_ratio: calmarRatio,
    equity_curve: equityCurve,
    drawdown_curve: drawdownCurve,
    trade_pnls: tradePnls,
    monte_carlo: null,
    recent_trades: trades.slice(-30).map((trade) => ({ pnl: trade?.pnl ?? 0 })),
    heatmap: acct?.heatmap || [],
    daily_pnl_pct: null,
    source: canonicalData?.source || 'journal_fallback',
    source_label: canonicalData?.source_label || 'Journal fallback',
    balances,
    totals,
    reconciliation: canonicalData?.reconciliation || { state: 'degraded', reason: 'overview_timeout' },
    simulators: canonicalData?.simulators || {},
    historical_source: { label: `Journal ${QUANT_SCOPE} + fallback`, kind: 'journal' },
    position_management: canonicalData?.position_management || {},
    exit_governance: canonicalData?.exit_governance || {},
    monitor_summary: canonicalData?.monitor_summary || {},
  };
}

function isWarmupOverviewPayload(payload) {
  if (!payload || typeof payload !== 'object') return true;
  if (payload.source === 'warmup') return true;
  if (payload.cache?.last_error === 'refresh_pending') return true;
  const hasSeries = Array.isArray(payload.equity_curve) && payload.equity_curve.length > 0;
  const hasTrades = Number(payload.total_trades || 0) > 0;
  const hasPositions = Number(payload.open_positions || 0) > 0;
  return !hasSeries && !hasTrades && !hasPositions;
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

  const hasHistoricalSignal = Boolean(
    overview &&
    overview.stats_signal_ok !== false &&
    Array.isArray(overview.equity_curve) &&
    overview.equity_curve.length > 1
  );
  const rtCurve = recordRealtimeEquityPoint(snapshot);
  if (!hasHistoricalSignal && rtCurve.length > 1) {
    if (!_equityChart) {
      _equityChart = QuantCharts.renderEquityChart('chart-equity', rtCurve);
    } else {
      QuantCharts.safeSetSeriesData(_equityChart.series, rtCurve);
    }
    const rtDrawdown = deriveDrawdownFromCurve(rtCurve);
    if (!_ddChart) {
      _ddChart = QuantCharts.renderDrawdownChart('chart-drawdown', rtDrawdown);
    } else {
      QuantCharts.safeSetSeriesData(_ddChart.series, rtDrawdown);
    }
  }
}

function recordRealtimeEquityPoint(snapshot) {
  const equity = Number(snapshot?.balances?.total_equity);
  if (!Number.isFinite(equity)) return _realtimeEquityCurve;
  const tsRaw = snapshot?.generated_at || new Date().toISOString();
  const tsParsed = Date.parse(tsRaw);
  let unix = Number.isFinite(tsParsed) ? Math.floor(tsParsed / 1000) : Math.floor(Date.now() / 1000);
  if (_realtimeEquityCurve.length) {
    const last = _realtimeEquityCurve[_realtimeEquityCurve.length - 1];
    if (unix < last.time) unix = last.time;
    if (unix === last.time) {
      last.value = Number(equity.toFixed(2));
      return _realtimeEquityCurve;
    }
  }
  _realtimeEquityCurve.push({ time: unix, value: Number(equity.toFixed(2)) });
  if (_realtimeEquityCurve.length > _realtimeMaxPoints) {
    _realtimeEquityCurve = _realtimeEquityCurve.slice(-_realtimeMaxPoints);
  }
  return _realtimeEquityCurve;
}

function deriveDrawdownFromCurve(curve = []) {
  if (!Array.isArray(curve) || curve.length === 0) return [];
  let peak = Number(curve[0]?.value || 0);
  return curve.map((point) => {
    const value = Number(point?.value || 0);
    if (value > peak) peak = value;
    const dd = peak > 0 ? ((value - peak) / peak) * 100 : 0;
    return { time: point.time, value: Number(dd.toFixed(4)) };
  });
}

function renderPositionsRealtime(snapshot) {
  if (!snapshot) return;
  renderPositionsTable(snapshot);
}

// ── Backend online/offline state ─────────────────────────────────
window._backendOnline = null;  // null=unknown, true=online, false=offline
let _backendOnline = null;

function _setBackendState(online) {
  if (_backendOnline === online) return;
  const wasExplicitlyOffline = _backendOnline === false;
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
    updateSyncChipState('SYNC OFFLINE');
  } else {
    banner?.remove();
    document.getElementById('chip-uptime')?.removeAttribute('data-offline');
    updateSyncChipState();
    // Evitar doble loadOverview al arranque (null → true); solo refrescar tras caída real
    if (wasExplicitlyOffline) {
      loadOverview();
    }
  }
}

// ── Health / topbar ───────────────────────────────────────────────
async function pollHealth() {
  if (_healthRequest) return _healthRequest;
  window._lastHealthAt = Date.now();
  _healthRequest = (async () => {
    let backendReachable = false;
    try {
      const health = await QuantAPI.health();
      if (health && (health.ok !== false || health.status)) {
        backendReachable = true;
        _healthFailures = 0;
        _setBackendState(true);
      }
      const canonicalData = _latestCanonicalSnapshot;
      if (canonicalData) {
        renderCanonicalMeta(canonicalData);
        const generatedAt = canonicalData?.generated_at ? new Date(canonicalData.generated_at).toLocaleTimeString() : null;
        setElText('chip-uptime', generatedAt ? `sync ${generatedAt}` : 'sync ok');
        const openPositions =
          canonicalData?.totals?.positions ??
          0;
        setElText('chip-positions', `${openPositions || 0} pos`);
        const totalEquity =
          canonicalData?.balances?.total_equity;
        if (totalEquity != null) {
          setElText('chip-equity', fmt.usd(totalEquity));
        }
      }
    } catch (error) {
      const recentlyRenderedOverview = _lastOverviewSuccessAt && (Date.now() - _lastOverviewSuccessAt) < 45000;
      _healthFailures += 1;
      if (window._quantWsConnected || backendReachable || recentlyRenderedOverview) {
        _setBackendState(true);
        return;
      }
      if (_healthFailures < 3) {
        return;
      }
      console.error('pollHealth failed', error);
      _setBackendState(false);
    } finally {
      _healthRequest = null;
    }
  })();

  return _healthRequest;
}

// ── OVERVIEW ──────────────────────────────────────────────────────
let _equityChart = null, _ddChart = null;

async function loadOverview() {
  if (_overviewRequest) return _overviewRequest;
  _overviewRequest = (async () => {
  try {
    const overviewPromise = QuantAPI.dashboardOverview(QUANT_SCOPE, QUANT_ACCOUNT_ID).catch(() => null);
    let canonicalData = _latestCanonicalSnapshot || null;
    const canonicalPromise = canonicalData
      ? Promise.resolve({ data: canonicalData })
      : QuantAPI.canonicalSnapshot(QUANT_SCOPE, QUANT_ACCOUNT_ID)
          .then((resp) => {
            if (resp?.data) {
              renderCanonicalMeta(resp.data);
              return resp;
            }
            return null;
          })
          .catch(() => null);

    const overviewResp = await overviewPromise;
    const canonicalResp = canonicalData ? { data: canonicalData } : await Promise.race([
      canonicalPromise,
      new Promise((resolve) => window.setTimeout(() => resolve(null), 1600)),
    ]);

    canonicalData = canonicalResp?.data || _latestCanonicalSnapshot || null;
    if (isSparseOperationalSnapshot(canonicalData) && _latestCanonicalSnapshot && !isSparseOperationalSnapshot(_latestCanonicalSnapshot)) {
      canonicalData = _latestCanonicalSnapshot;
    }
    let m = overviewResp?.data || null;

    if (isWarmupOverviewPayload(m)) {
      const statsResp = await QuantAPI.journalStats().catch(() => null);
      const chartResp = await QuantAPI.journalChartData(500, QUANT_SCOPE).catch(() => null);
      if (statsResp?.data || chartResp?.data) {
        m = buildOverviewFromJournal(statsResp?.data, chartResp?.data, canonicalData);
      }
    }

    if (!m) {
      if (canonicalData) {
        renderCanonicalMeta(canonicalData);
        renderOverviewRealtime(canonicalData);
        _lastOverviewSuccessAt = Date.now();
        if (window._quantWsConnected || window._backendOnline !== false) {
          _setBackendState(true);
        }
        return;
      }
      if (!window._quantWsConnected) _setBackendState(false);
      return;
    }

    _setBackendState(true);
    _lastOverviewSuccessAt = Date.now();
    maybeWarnLightweight(m);
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
    renderCanonicalMeta(canonicalData || m);
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
      m.total_trades != null
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
    if (effectiveEquity != null) setElText('chip-equity', fmt.usd(effectiveEquity));
    setElText('chip-positions', `${brokerPositions || 0} pos`);
    notifyAtlas('handleDashboardUpdate', canonicalData || m);

    // ── Equity curve (TradingView) ──
    const hasHistoricalSignal = m?.stats_signal_ok !== false && Array.isArray(m?.equity_curve) && m.equity_curve.length > 1;
    const eq = hasHistoricalSignal
      ? QuantCharts.normalizeSeriesData(m.equity_curve || [])
      : QuantCharts.normalizeSeriesData(_realtimeEquityCurve || []);
    if (!_equityChart) {
      _equityChart = QuantCharts.renderEquityChart('chart-equity', eq);
    } else {
      QuantCharts.safeSetSeriesData(_equityChart.series, eq);
    }

    // ── Drawdown ──
    const dd = hasHistoricalSignal
      ? QuantCharts.normalizeSeriesData(m.drawdown_curve || [])
      : QuantCharts.normalizeSeriesData(deriveDrawdownFromCurve(eq));
    if (!_ddChart) {
      _ddChart = QuantCharts.renderDrawdownChart('chart-drawdown', dd);
    } else {
      QuantCharts.safeSetSeriesData(_ddChart.series, dd);
    }

    // ── PnL distribution ──
    const pnls = QuantCharts.normalizeNumericArray(m.trade_pnls || []);
    QuantCharts.renderPnlDistChart('chart-pnl-dist', pnls);

    // ── Monte Carlo ──
    QuantCharts.renderMonteCarloChart('chart-mc', m.monte_carlo || null);

    // ── Win/Loss streak ──
    const trades = m.recent_trades || [];
    QuantCharts.renderStreakChart('chart-streak', trades);

    // ── Heatmap de performance (día × hora) ──
    QuantCharts.renderHeatmapChart('chart-heatmap', m.heatmap || []);

    // ── Rolling Sharpe ──
    QuantCharts.renderRollingSharpeChart('chart-rolling-sharpe', eq);

  } catch (e) {
    console.error('loadOverview failed', e);
    toast('Error cargando overview: ' + e.message, 'error');
  } finally {
    _overviewRequest = null;
  }
  })();

  return _overviewRequest;
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
  if (_positionsRequest) return _positionsRequest;
  _positionsRequest = (async () => {
    try {
      const cachedSnapshot = _latestPositionsSnapshot || _latestCanonicalSnapshot || null;
      if (Array.isArray(cachedSnapshot?.positions) && cachedSnapshot.positions.length) {
        renderCanonicalMeta(cachedSnapshot);
        maybeWarnLightweight(cachedSnapshot);
        renderPositionsTable({
          ...cachedSnapshot,
          gross_exposure: cachedSnapshot?.gross_exposure ?? cachedSnapshot?.balances?.gross_exposure ?? 0,
        });
      }

      let positionsPayload = await QuantAPI.positions(QUANT_SCOPE, QUANT_ACCOUNT_ID).catch(() => null);
      let data = positionsPayload?.data || cachedSnapshot || {};
      if (!Array.isArray(data?.positions)) {
        data = _latestCanonicalSnapshot || data || {};
      }
      if (!Array.isArray(data?.positions)) {
        const canonical = await QuantAPI.canonicalSnapshot(QUANT_SCOPE, QUANT_ACCOUNT_ID).catch(() => null);
        data = canonical?.data || data || cachedSnapshot || {};
      }
      const positions = data?.positions || [];
      if (data) {
        renderCanonicalMeta(data);
        maybeWarnLightweight(data);
      }
      renderPositionsTable({
        ...data,
        positions,
        gross_exposure: data?.gross_exposure ?? data?.balances?.gross_exposure ?? 0,
      });
    } catch (e) {
      const cachedSnapshot = _latestPositionsSnapshot || _latestCanonicalSnapshot || null;
      if (Array.isArray(cachedSnapshot?.positions)) {
        renderCanonicalMeta(cachedSnapshot);
        renderPositionsTable({
          ...cachedSnapshot,
          gross_exposure: cachedSnapshot?.gross_exposure ?? cachedSnapshot?.balances?.gross_exposure ?? 0,
        });
        useCachedModuleState('Posiciones degradadas: mostrando ultimo snapshot canonico.');
      } else {
        setEmptyRow('positions-body', 15, 'Posiciones no disponibles: backend degradado o sin respuesta.');
        markDegradedChip('pos-source-badge', 'Fuente degradada');
        markDegradedChip('pos-sync-badge', 'SYNC WARN');
        toast('Error cargando posiciones: ' + e.message, 'error');
      }
    } finally {
      _positionsRequest = null;
    }
  })();

  return _positionsRequest;
}

// ── SCANNER ───────────────────────────────────────────────────────
async function loadScanner() {
  if (_scannerRequest) return _scannerRequest;
  _scannerRequest = (async () => {
    try {
      const status = await QuantAPI.scannerStatus().catch(() => null);
      const report = await QuantAPI.scannerReport().catch(() => null);

      const st = status?.data || _latestScannerState?.status || {};
      document.getElementById('scanner-status-chip').textContent =
        st.running ? 'ACTIVO' : (_latestScannerState ? 'CACHE' : 'INACTIVO');

      const opps = report?.data?.opportunities || report?.data?.candidates || _latestScannerState?.opportunities || [];
      if (status?.data || report?.data) {
        _latestScannerState = { status: st, opportunities: opps };
      }
      const tbody = document.getElementById('scanner-body');
      if (!opps.length) {
        tbody.innerHTML = `<tr class="empty-row"><td colspan="7">${_latestScannerState ? 'Sin oportunidades detectadas' : 'Scanner degradado o sin oportunidades'}</td></tr>`;
        return;
      }
      tbody.innerHTML = opps.map(o => `<tr>
        <td class="accent">${o.symbol}</td>
        <td>${o.strategy || o.strategy_type || '--'}</td>
        <td class="${parseFloat(o.score ?? o.selection_score ?? 0) >= 0.7 ? 'accent' : ''}">${fmt.num(o.score ?? o.selection_score, 2)}</td>
        <td>${o.signal || o.direction || '--'}</td>
        <td>${fmt.pct(o.win_probability ?? o.win_rate_pct)}</td>
        <td class="red">${fmt.pct(o.var_95)}</td>
        <td>${fmt.ts(o.timestamp || o.generated_at)}</td>
      </tr>`).join('');
    } catch (e) {
      const cached = _latestScannerState;
      if (cached) {
        document.getElementById('scanner-status-chip').textContent = 'CACHE';
        const tbody = document.getElementById('scanner-body');
        tbody.innerHTML = cached.opportunities.length
          ? cached.opportunities.map(o => `<tr>
        <td class="accent">${o.symbol}</td>
        <td>${o.strategy || o.strategy_type || '--'}</td>
        <td class="${parseFloat(o.score ?? o.selection_score ?? 0) >= 0.7 ? 'accent' : ''}">${fmt.num(o.score ?? o.selection_score, 2)}</td>
        <td>${o.signal || o.direction || '--'}</td>
        <td>${fmt.pct(o.win_probability ?? o.win_rate_pct)}</td>
        <td class="red">${fmt.pct(o.var_95)}</td>
        <td>${fmt.ts(o.timestamp || o.generated_at)}</td>
      </tr>`).join('')
          : '<tr class="empty-row"><td colspan="7">Sin oportunidades detectadas</td></tr>';
        useCachedModuleState('Scanner degradado: mostrando el ultimo estado disponible.');
      } else {
        document.getElementById('scanner-status-chip').textContent = 'DEGRADED';
        setEmptyRow('scanner-body', 7, 'Scanner no disponible: backend degradado o sin respuesta.');
        toast('Error cargando scanner: ' + e.message, 'error');
      }
    } finally {
      _scannerRequest = null;
    }
  })();

  return _scannerRequest;
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
      symbol,
      source: symbol.includes('/') ? 'ccxt' : 'yfinance',
      exchange: 'binance',
      timeframe: tf,
      strategy,
      strategy_type: strategy,
      capital,
      initial_capital: capital,
      limit: bars,
      bars,
      walk_forward_folds: folds,
      kelly_sizing: kelly,
      atr_stops: atr,
      monte_carlo: mc,
      period: tf === '1d' ? '2y' : '1y',
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
      const eq = QuantCharts.normalizeSeriesData(m.equity_curve || r.data.equity_curve || []);
      if (_btEquityChart) { QuantCharts.safeSetSeriesData(_btEquityChart.series, eq); }
      else { _btEquityChart = QuantCharts.renderEquityChart('bt-chart-equity', eq); }

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
        if (link) link.href = r.data.report_html || `/reports/${rp.split('/').pop()}`;
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
  if (_journalRequest) return _journalRequest;
  _journalRequest = (async () => {
  try {
    const stats = await QuantAPI.journalStats().catch(() => null);
    const entries = await QuantAPI.journalEntries(100, QUANT_SCOPE).catch(() => null);

    const statsPayload = stats?.data || _latestJournalState?.statsPayload || {};
    const s = pickJournalAccount(statsPayload, QUANT_SCOPE);
    const list = normalizeJournalEntries(entries?.data || _latestJournalState?.entriesPayload || []);
    if (stats?.data || entries?.data) {
      _latestJournalState = {
        statsPayload,
        entriesPayload: entries?.data || _latestJournalState?.entriesPayload || [],
      };
    }
    const totalPnl = Number(s?.realized_pnl || 0) + Number(s?.unrealized_pnl || 0);
    const totalTrades = Number(s?.trades_closed || 0);
    const avgPnl = totalTrades ? totalPnl / totalTrades : 0;
    const kpiRow = document.getElementById('journal-kpi');
    kpiRow.innerHTML = `
      <div class="kpi-card"><span class="kpi-label">Total PnL</span>
        <span class="kpi-value ${totalPnl >= 0 ? 'green' : 'red'}">${fmt.usd(totalPnl)}</span></div>
      <div class="kpi-card"><span class="kpi-label">Win Rate</span>
        <span class="kpi-value">${fmt.pct(s?.win_rate_pct)}</span></div>
      <div class="kpi-card"><span class="kpi-label">Trades</span>
        <span class="kpi-value">${totalTrades}</span></div>
      <div class="kpi-card"><span class="kpi-label">Avg PnL</span>
        <span class="kpi-value">${fmt.usd(avgPnl)}</span></div>
    `;

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
    const cached = _latestJournalState;
    if (cached) {
      const s = pickJournalAccount(cached.statsPayload || {}, QUANT_SCOPE);
      const list = normalizeJournalEntries(cached.entriesPayload || []);
      const totalPnl = Number(s?.realized_pnl || 0) + Number(s?.unrealized_pnl || 0);
      const totalTrades = Number(s?.trades_closed || 0);
      const avgPnl = totalTrades ? totalPnl / totalTrades : 0;
      const kpiRow = document.getElementById('journal-kpi');
      kpiRow.innerHTML = `
      <div class="kpi-card"><span class="kpi-label">Total PnL</span>
        <span class="kpi-value ${totalPnl >= 0 ? 'green' : 'red'}">${fmt.usd(totalPnl)}</span></div>
      <div class="kpi-card"><span class="kpi-label">Win Rate</span>
        <span class="kpi-value">${fmt.pct(s?.win_rate_pct)}</span></div>
      <div class="kpi-card"><span class="kpi-label">Trades</span>
        <span class="kpi-value">${totalTrades}</span></div>
      <div class="kpi-card"><span class="kpi-label">Avg PnL</span>
        <span class="kpi-value">${fmt.usd(avgPnl)}</span></div>
    `;
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
      useCachedModuleState('Journal degradado: mostrando el ultimo estado disponible.');
    } else {
      setEmptyRow('journal-body', 7, 'Journal no disponible: backend degradado o sin respuesta.');
      toast('Error cargando journal: ' + e.message, 'error');
    }
  } finally {
    _journalRequest = null;
  }
  })();

  return _journalRequest;
}

document.getElementById('journal-refresh')?.addEventListener('click', loadJournal);

// ── RL ENGINE ─────────────────────────────────────────────────────
async function loadRL() {
  const symbol = document.getElementById('rl-symbol-sel')?.value || 'BTC/USDT';
  if (_rlRequest) return _rlRequest;
  _rlRequest = (async () => {
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
    } finally {
      _rlRequest = null;
    }
  })();

  return _rlRequest;
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
async function loadVisual() {
  if (_visualRequest) return _visualRequest;
  _visualRequest = (async () => {
    try {
      const r = await QuantAPI.visualState();
      if (!r.ok) { toast('Error capturando visual: ' + r.error, 'error'); return; }
      const d = r.data || {};
      _latestVisualState = d;
      document.getElementById('vs-brightness').textContent = fmt.num(d.brightness, 1);
      document.getElementById('vs-sharpness').textContent  = fmt.num(d.sharpness, 1);
      document.getElementById('vs-screens').textContent    = d.screens_detected ?? '--';
      document.getElementById('vs-color').textContent      = d.chart_color || '--';
      document.getElementById('vs-prices').textContent     = (d.ocr_prices || []).join(', ') || 'ninguno';
      document.getElementById('vs-pattern').textContent    = d.pattern_detected || 'ninguno';

      const badge = document.getElementById('visual-safety-badge');
      badge.textContent  = d.safe_mode ? 'SAFE MODE' : 'OK';
      badge.className    = `mode-badge ${d.safe_mode ? 'safe' : 'ok'}`;

      if (d.feature_vector?.length) {
        QuantCharts.renderVisualFeaturesChart('chart-visual-features', d.feature_vector);
      }
      return d;
    } catch (e) {
      const d = _latestVisualState;
      if (d) {
        document.getElementById('vs-brightness').textContent = fmt.num(d.brightness, 1);
        document.getElementById('vs-sharpness').textContent  = fmt.num(d.sharpness, 1);
        document.getElementById('vs-screens').textContent    = d.screens_detected ?? '--';
        document.getElementById('vs-color').textContent      = d.chart_color || '--';
        document.getElementById('vs-prices').textContent     = (d.ocr_prices || []).join(', ') || 'ninguno';
        document.getElementById('vs-pattern').textContent    = d.pattern_detected || 'ninguno';
        const badge = document.getElementById('visual-safety-badge');
        badge.textContent = 'CACHE';
        badge.className = 'mode-badge safe';
        useCachedModuleState('Visual degradado: mostrando la ultima captura disponible.');
        return d;
      }
      toast('Error: ' + e.message, 'error');
      throw e;
    } finally {
      _visualRequest = null;
    }
  })();

  return _visualRequest;
}

document.getElementById('visual-capture')?.addEventListener('click', async () => {
  const data = await loadVisual().catch(() => null);
  if (data) toast('Estado visual capturado', 'success');
});

// ── ALERTS ────────────────────────────────────────────────────────
async function loadAlerts() {
  if (_alertsRequest) return _alertsRequest;
  _alertsRequest = (async () => {
    try {
      const r = await QuantAPI.alertsStatus();
      const d = r.data || {};
      _latestAlertsState = d;
      document.getElementById('al-telegram').textContent  = d.telegram_enabled === true ? 'ON' : d.telegram_enabled === false ? 'OFF' : '--';
      document.getElementById('al-whatsapp').textContent  = d.whatsapp_enabled === true ? 'ON' : d.whatsapp_enabled === false ? 'OFF' : '--';
      document.getElementById('al-queue').textContent     = d.queue_size ?? '--';
      document.getElementById('al-sent').textContent      = d.sent_today ?? d.sent_count ?? '--';
      document.getElementById('al-level').textContent     = d.min_level || (d.worker_alive === true ? 'worker_alive' : d.worker_alive === false ? 'worker_down' : '--');
  document.getElementById('al-cooldown').textContent  = d.cooldown_s ? `${d.cooldown_s}s` : (d.error_count != null ? `errors ${d.error_count}` : '--');
      if (d.any_channel_enabled === false) {
        appendLog('alerts-log', `[${new Date().toLocaleTimeString()}] Sin canales de alerta activos: revisa Telegram/WhatsApp en backend.`);
      }
    } catch (e) {
      const d = _latestAlertsState;
      if (d) {
        document.getElementById('al-telegram').textContent  = d.telegram_enabled === true ? 'ON' : d.telegram_enabled === false ? 'OFF' : '--';
        document.getElementById('al-whatsapp').textContent  = d.whatsapp_enabled === true ? 'ON' : d.whatsapp_enabled === false ? 'OFF' : '--';
        document.getElementById('al-queue').textContent     = d.queue_size ?? '--';
        document.getElementById('al-sent').textContent      = d.sent_today ?? d.sent_count ?? '--';
        document.getElementById('al-level').textContent     = 'CACHE';
        document.getElementById('al-cooldown').textContent  = d.cooldown_s ? `${d.cooldown_s}s` : (d.error_count != null ? `errors ${d.error_count}` : '--');
        appendLog('alerts-log', `[${new Date().toLocaleTimeString()}] Estado degradado: se mantiene el ultimo snapshot disponible.`);
        useCachedModuleState('Alertas degradadas: mostrando el ultimo estado disponible.');
      } else {
        document.getElementById('al-level').textContent = 'DEGRADED';
        appendLog('alerts-log', `[${new Date().toLocaleTimeString()}] Alertas no disponibles: backend degradado o sin respuesta.`);
        toast('Error cargando alertas: ' + e.message, 'error');
      }
    } finally {
      _alertsRequest = null;
    }
  })();

  return _alertsRequest;
}

document.getElementById('alerts-refresh-btn')?.addEventListener('click', loadAlerts);
document.getElementById('alerts-test-btn')?.addEventListener('click', async () => {
  const r = await QuantAPI.alertsTest('Test desde Atlas Code-Quant Dashboard v1.0.0');
  if (r.ok && r.data?.delivered_any) {
    appendLog('alerts-log', `[${new Date().toLocaleTimeString()}] Alerta de prueba entregada · Telegram=${r.data.telegram_delivered} WhatsApp=${r.data.whatsapp_delivered}`);
    toast('Alerta entregada', 'success');
    notifyAtlas('send', '🔔 Prueba local', 'El sistema local de notificaciones está funcionando.', 'success');
  } else if (r.ok) {
    appendLog('alerts-log', `[${new Date().toLocaleTimeString()}] Prueba sin entrega efectiva · Telegram=${r.data?.telegram_delivered} WhatsApp=${r.data?.whatsapp_delivered}`);
    toast('Prueba ejecutada sin entrega efectiva', 'warning');
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
  if (window.__quantWsHandlersBound) return;
  window.__quantWsHandlersBound = true;
  quantWS.on('connected', () => {
    window._quantPollingMode = false;
    const wsDot = document.getElementById('ws-dot');
    if (wsDot) {
      wsDot.classList.remove('polling');
      wsDot.title = 'WebSocket conectado';
    }
    window._quantWsConnected = true;
    updateWsModeChip();
    _setBackendState(true);
    pollHealth();
  });

  quantWS.on('disconnected', () => {
    window._quantWsConnected = false;
    updateWsModeChip();
  });

  quantWS.on('quant.live_update', (msg) => {
    if (msg.canonical_snapshot) {
      if (isLightweightSnapshot(msg.canonical_snapshot)) {
        enablePollingOnlyMode('lightweight_live_update');
      }
      _setBackendState(true);
      renderCanonicalMeta(msg.canonical_snapshot);
      renderOverviewRealtime(msg.canonical_snapshot);
      notifyAtlas('handleDashboardUpdate', msg.canonical_snapshot);
      const canRenderPositionsRealtime =
        !msg.canonical_snapshot.positions_truncated &&
        Array.isArray(msg.canonical_snapshot.positions);
      if (canRenderPositionsRealtime && document.getElementById('view-positions')?.classList.contains('active')) {
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
      setElText('chip-equity', fmt.usd(msg.equity));
    }
    notifyAtlas('handleSignal', msg.symbol, msg.signal, msg.confidence || 0);
  });

  quantWS.on('trade', (msg) => {
    const dir = msg.side === 'BUY' ? 'compra' : 'venta';
    appendLog('rl-log-box', `[${new Date().toLocaleTimeString()}] Trade ${dir} ${msg.symbol} @ ${fmt.usd(msg.price)}`);
    notifyAtlas('handleTrade', msg);
    pollHealth();
  });

  if (!_forcePollingMode) {
    quantWS.connect();
  }
}

function getActiveView() {
  return document.querySelector('.nav-item.active')?.dataset?.view || 'overview';
}

async function refreshActiveView() {
  _lastViewRefreshAt = Date.now();
  const view = getActiveView();
  if (view === 'overview') return loadOverview();
  if (view === 'positions') return loadPositions();
  if (view === 'scanner') return loadScanner();
  if (view === 'journal') return loadJournal();
  if (view === 'visual') return loadVisual();
  if (view === 'alerts') return loadAlerts();
  if (view === 'rl') return loadRL();
  if (view === 'analytics' && window.AtlasCharts) return AtlasCharts.refresh();
}

// ── Emergency stop ────────────────────────────────────────────────
document.getElementById('btn-emergency')?.addEventListener('click', () => {
  if (!confirm('¿Activar Emergency Stop? Cerrará todas las posiciones.')) return;
  notifyAtlas('handleEmergencyStop');
  apiPost('/emergency/stop', { reason: 'manual_dashboard' }).then(r => {
    toast(r.ok ? 'Emergency Stop activado' : 'Error: ' + r.error, r.ok ? 'error' : 'error', 5000);
  });
});

// ── Refresh buttons ───────────────────────────────────────────────
document.getElementById('ov-refresh')?.addEventListener('click', loadOverview);
document.getElementById('pos-refresh')?.addEventListener('click', loadPositions);
document.getElementById('ac-refresh')?.addEventListener('click', () => { if (window.AtlasCharts) AtlasCharts.refresh(); });

// ── Init ──────────────────────────────────────────────────────────
function initDashboardApp() {
  if (window.__atlasDashboardStarted) return;
  window.__atlasDashboardStarted = true;
  updateWsModeChip();
  updateSyncChipState();
  initNav();
  startClock();
  loadOverview();

  // Primer health check inmediato, luego adaptativo
  pollHealth();
  setTimeout(initWS, 1200);
  // Cuando offline: health cada 5s. Cuando online: health cada 15s como respaldo del WS.
  document.addEventListener('visibilitychange', () => {
    if (document.visibilityState === 'hidden') {
      quantWS.close();
      return;
    }
    if (!_forcePollingMode) {
      quantWS.connect();
    }
    pollHealth();
  });

  setInterval(() => {
    if (document.visibilityState === 'hidden') return;
    const now = Date.now();
    const healthInterval = _backendOnline === false ? 5_000 : 15_000;
    if (!hasInFlightModuleRequest() && (!window._lastHealthAt || now - window._lastHealthAt >= healthInterval)) {
      pollHealth();
    }

    const activeView = getActiveView();
    const needsPollingView =
      !window._quantWsConnected ||
      ['scanner', 'journal', 'alerts', 'rl', 'analytics'].includes(activeView);
    const viewRefreshInterval = window._quantWsConnected ? 60_000 : 30_000;
    if (_backendOnline !== false && !hasInFlightModuleRequest() && needsPollingView && (!_lastViewRefreshAt || now - _lastViewRefreshAt >= viewRefreshInterval)) {
      refreshActiveView();
    }
  }, 5_000);
}

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', initDashboardApp, { once: true });
} else {
  queueMicrotask(initDashboardApp);
}
