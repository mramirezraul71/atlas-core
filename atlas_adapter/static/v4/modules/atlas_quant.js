/**
 * ATLAS v4.3 — Atlas Code-Quant
 * Shortcut y panel de estado del sistema de trading algorítmico.
 * API interna en puerto 8792.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID  = 'atlas-quant-module';
const MONITOR_POLL_ID = 'atlas-quant-monitor';
const QUANT_API = 'http://127.0.0.1:8792';
const QUANT_API_V2 = `${QUANT_API}/api/v2/quant`;
const API_KEY   = 'atlas-quant-local';
const QUANT_WS = `${QUANT_API.replace(/^http/i, 'ws')}/api/v2/quant/ws/live-updates`;
const WS_RECONNECT_MS = 4000;
let ACTIVE_CONTAINER = null;
let LIVE_SOCKET = null;
let LIVE_SOCKET_RETRY = null;
let LIVE_SOCKET_MANUAL_CLOSE = false;
const ORDER_PRESETS = {
  custom: null,
  long_call: {
    assetClass: 'option',
    side: 'buy',
    positionEffect: 'open',
    orderType: 'market',
    duration: 'day',
    strategyType: 'long_call',
    legs: [],
  },
  bull_call_debit_spread: {
    assetClass: 'multileg',
    side: 'buy',
    positionEffect: 'open',
    orderType: 'debit',
    duration: 'day',
    strategyType: 'bull_call_debit_spread',
    legs: [
      { option_symbol: 'AAPL260417C00200000', side: 'buy', quantity: 1 },
      { option_symbol: 'AAPL260417C00210000', side: 'sell', quantity: 1 },
    ],
  },
  bear_put_debit_spread: {
    assetClass: 'multileg',
    side: 'buy',
    positionEffect: 'open',
    orderType: 'debit',
    duration: 'day',
    strategyType: 'bear_put_debit_spread',
    legs: [
      { option_symbol: 'AAPL260417P00210000', side: 'buy', quantity: 1 },
      { option_symbol: 'AAPL260417P00200000', side: 'sell', quantity: 1 },
    ],
  },
  bear_call_credit_spread: {
    assetClass: 'multileg',
    side: 'sell',
    positionEffect: 'open',
    orderType: 'credit',
    duration: 'day',
    strategyType: 'bear_call_credit_spread',
    legs: [
      { option_symbol: 'AAPL260417C00200000', side: 'sell', quantity: 1 },
      { option_symbol: 'AAPL260417C00210000', side: 'buy', quantity: 1 },
    ],
  },
  bull_put_credit_spread: {
    assetClass: 'multileg',
    side: 'sell',
    positionEffect: 'open',
    orderType: 'credit',
    duration: 'day',
    strategyType: 'bull_put_credit_spread',
    legs: [
      { option_symbol: 'AAPL260417P00200000', side: 'sell', quantity: 1 },
      { option_symbol: 'AAPL260417P00190000', side: 'buy', quantity: 1 },
    ],
  },
  iron_condor: {
    assetClass: 'multileg',
    side: 'sell',
    positionEffect: 'open',
    orderType: 'credit',
    duration: 'day',
    strategyType: 'iron_condor',
    legs: [
      { option_symbol: 'AAPL260417P00190000', side: 'buy', quantity: 1 },
      { option_symbol: 'AAPL260417P00200000', side: 'sell', quantity: 1 },
      { option_symbol: 'AAPL260417C00210000', side: 'sell', quantity: 1 },
      { option_symbol: 'AAPL260417C00220000', side: 'buy', quantity: 1 },
    ],
  },
  iron_butterfly: {
    assetClass: 'multileg',
    side: 'sell',
    positionEffect: 'open',
    orderType: 'credit',
    duration: 'day',
    strategyType: 'iron_butterfly',
    legs: [
      { option_symbol: 'AAPL260417P00200000', side: 'sell', quantity: 1 },
      { option_symbol: 'AAPL260417C00200000', side: 'sell', quantity: 1 },
      { option_symbol: 'AAPL260417P00190000', side: 'buy', quantity: 1 },
      { option_symbol: 'AAPL260417C00210000', side: 'buy', quantity: 1 },
    ],
  },
  call_calendar_spread: {
    assetClass: 'multileg',
    side: 'buy',
    positionEffect: 'open',
    orderType: 'debit',
    duration: 'day',
    strategyType: 'call_calendar_spread',
    legs: [
      { option_symbol: 'AAPL260417C00200000', side: 'sell', quantity: 1 },
      { option_symbol: 'AAPL260515C00200000', side: 'buy', quantity: 1 },
    ],
  },
  call_diagonal_debit_spread: {
    assetClass: 'multileg',
    side: 'buy',
    positionEffect: 'open',
    orderType: 'debit',
    duration: 'day',
    strategyType: 'call_diagonal_debit_spread',
    legs: [
      { option_symbol: 'AAPL260417C00210000', side: 'sell', quantity: 1 },
      { option_symbol: 'AAPL260515C00200000', side: 'buy', quantity: 1 },
    ],
  },
  covered_call: {
    assetClass: 'combo',
    side: 'sell',
    positionEffect: 'open',
    orderType: 'credit',
    duration: 'day',
    strategyType: 'covered_call',
    legs: [
      { instrument_type: 'equity', side: 'buy', quantity: 100 },
      { option_symbol: 'AAPL260417C00210000', side: 'sell', quantity: 1 },
    ],
  },
};

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

function _wsUrl(container) {
  const params = new URLSearchParams();
  params.set('api_key', API_KEY);
  const scope = _orderScope(container);
  const accountId = _orderAccountId(container);
  if (scope) params.set('account_scope', scope);
  if (accountId) params.set('account_id', accountId);
  return `${QUANT_WS}?${params.toString()}`;
}

function _setFeedState(container, state, detail = '') {
  const el = container.querySelector('#aq-live-feed');
  if (!el) return;
  const tone = {
    connecting: 'orange',
    live: 'green',
    fallback: 'blue',
    error: 'red',
  }[state] || 'blue';
  const label = {
    connecting: 'WS conectando',
    live: 'WS en vivo',
    fallback: 'REST fallback',
    error: 'WS con error',
  }[state] || 'Feed';
  el.className = `chip ${tone}`;
  el.textContent = detail ? `${label} · ${detail}` : label;
}

function _startFallbackPolling(container) {
  stop(POLL_ID);
  stop(MONITOR_POLL_ID);
  poll(POLL_ID, `${QUANT_API_V2}/status`, 15000, () => _fetchHealth(container));
  poll(MONITOR_POLL_ID, `${QUANT_API_V2}/monitor/summary`, 60000, () => _fetchMonitorSummary(container));
}

function _stopLiveSocket() {
  clearTimeout(LIVE_SOCKET_RETRY);
  LIVE_SOCKET_RETRY = null;
  LIVE_SOCKET_MANUAL_CLOSE = true;
  if (LIVE_SOCKET) {
    try { LIVE_SOCKET.__atlasManualClose = true; } catch {}
    try { LIVE_SOCKET.close(); } catch {}
    LIVE_SOCKET = null;
  }
}

function _readJson(value, fallback) {
  const text = String(value || '').trim();
  if (!text) return fallback;
  return JSON.parse(text);
}

function _orderScope(container) {
  return container.querySelector('#aq-account-scope')?.value || 'paper';
}

function _orderAccountId(container) {
  return container.querySelector('#aq-account-id')?.value?.trim() || null;
}

function _buildOrderPayload(container, { preview = true } = {}) {
  const symbol = container.querySelector('#aq-order-symbol')?.value?.trim() || '';
  const side = container.querySelector('#aq-order-side')?.value || 'buy';
  const size = parseFloat(container.querySelector('#aq-order-size')?.value || '0');
  const orderType = container.querySelector('#aq-order-type')?.value || 'market';
  const assetClass = container.querySelector('#aq-order-asset-class')?.value || 'auto';
  const optionSymbol = container.querySelector('#aq-order-option-symbol')?.value?.trim() || null;
  const positionEffect = container.querySelector('#aq-order-position-effect')?.value || 'auto';
  const duration = container.querySelector('#aq-order-duration')?.value || 'day';
  const tag = container.querySelector('#aq-order-tag')?.value?.trim() || null;
  const strategyType = container.querySelector('#aq-order-strategy-type')?.value || null;
  const priceText = container.querySelector('#aq-order-price')?.value?.trim() || '';
  const stopText = container.querySelector('#aq-order-stop-price')?.value?.trim() || '';
  const legsText = container.querySelector('#aq-order-legs')?.value || '';
  const gateEnabled = !!container.querySelector('#aq-order-probability-enabled')?.checked;
  const gateMin = parseFloat(container.querySelector('#aq-order-min-win-rate')?.value || '50');

  if (!symbol) throw new Error('Indica el simbolo base');
  if (!Number.isFinite(size) || size <= 0) throw new Error('Size invalido');

  const payload = {
    symbol,
    side,
    size,
    order_type: orderType,
    asset_class: assetClass,
    option_symbol: optionSymbol,
    position_effect: positionEffect,
    duration,
    preview,
    tag,
    account_scope: _orderScope(container),
    account_id: _orderAccountId(container),
  };

  if (priceText) payload.price = parseFloat(priceText);
  if (stopText) payload.stop_price = parseFloat(stopText);
  if (strategyType) payload.strategy_type = strategyType;

  const parsedLegs = _readJson(legsText, []);
  if (Array.isArray(parsedLegs) && parsedLegs.length > 0) payload.legs = parsedLegs;

  if (gateEnabled) {
    if (!strategyType) throw new Error('Selecciona strategy type para activar el gate');
    payload.probability_gate = {
      symbol,
      strategy_type: strategyType,
      min_win_rate_pct: Number.isFinite(gateMin) ? gateMin : 50,
      account_scope: _orderScope(container),
      account_id: _orderAccountId(container),
    };
  }

  return payload;
}

function _prettyJson(value) {
  return _esc(JSON.stringify(value, null, 2));
}

function _renderPdtBreakdown(pdt = {}, { compact = false } = {}) {
  if (!pdt || typeof pdt !== 'object') return '';
  const applicable = !!pdt.applicable;
  const blocked = !!pdt.blocked_opening;
  const canOpen = pdt.can_open !== false;
  const combined = Number(pdt.day_trades_last_window ?? pdt.combined_day_trades_last_window ?? 0);
  const remaining = Number(pdt.day_trades_remaining ?? 0);
  const broker = Number(pdt.broker_day_trades_last_window ?? combined);
  const ledger = Number(pdt.ledger_day_trades_today ?? 0);
  const brokerPreview = pdt.broker_preview_day_trades;
  const details = Array.isArray(pdt.day_trade_details) ? pdt.day_trade_details : [];
  const chipRow = `
    <div style="display:flex;gap:8px;flex-wrap:wrap">
      <span class="chip ${blocked ? 'red' : (applicable ? 'orange' : 'green')}">${blocked ? 'Aperturas bloqueadas' : (applicable ? 'PDT vigilado' : 'PDT no aplica')}</span>
      <span class="chip blue">5d: ${_esc(String(combined))}</span>
      <span class="chip ${remaining > 0 ? 'green' : 'red'}">remanente: ${_esc(String(remaining))}</span>
      <span class="chip accent">broker: ${_esc(String(broker))}</span>
      <span class="chip orange">ledger hoy: ${_esc(String(ledger))}</span>
      ${brokerPreview !== undefined && brokerPreview !== null ? `<span class="chip ${Number(brokerPreview) >= 3 ? 'red' : 'blue'}">preview broker: ${_esc(String(brokerPreview))}</span>` : ''}
      <span class="chip ${canOpen ? 'green' : 'red'}">${canOpen ? 'puede abrir' : 'no puede abrir'}</span>
    </div>
  `;
  if (compact) {
    return `
      <div style="display:flex;flex-direction:column;gap:8px">
        ${chipRow}
        <div class="stat-card-sub">${_esc(pdt.reason || 'sin observaciones')}</div>
      </div>
    `;
  }
  return `
    <div style="margin-top:12px;padding:10px;border:1px solid var(--border);border-radius:10px;background:rgba(255,255,255,0.02)">
      <div style="font-size:11px;color:var(--text-muted);margin-bottom:8px">Desglose PDT</div>
      ${chipRow}
      <div style="font-size:11px;color:var(--text-muted);margin-top:8px">${_esc(pdt.reason || 'sin observaciones')}</div>
      ${details.length > 0 ? `<div style="display:flex;gap:8px;flex-wrap:wrap;margin-top:8px">${details.slice(0, 6).map((item) => `<span class="chip blue">${_esc(item.date || '—')} · ${_esc(item.symbol || item.security_key || '—')}</span>`).join(' ')}</div>` : ''}
    </div>
  `;
}

function _applyOrderPreset(container, presetId) {
  const preset = ORDER_PRESETS[presetId];
  if (!preset) return;
  const setValue = (selector, value) => {
    const el = container.querySelector(selector);
    if (el && value !== undefined) el.value = value;
  };
  setValue('#aq-order-asset-class', preset.assetClass);
  setValue('#aq-order-side', preset.side);
  setValue('#aq-order-position-effect', preset.positionEffect);
  setValue('#aq-order-type', preset.orderType);
  setValue('#aq-order-duration', preset.duration);
  setValue('#aq-order-strategy-type', preset.strategyType);
  setValue('#aq-order-legs', JSON.stringify(preset.legs || [], null, 2));
}

function _strategyExecutionMeta(strategyType) {
  if (!strategyType) return null;
  if (strategyType === 'covered_call') {
    return { assetClass: 'combo', side: 'sell', positionEffect: 'open', orderType: 'credit' };
  }
  if (strategyType === 'cash_secured_put') {
    return { assetClass: 'option', side: 'sell', positionEffect: 'open', orderType: 'market' };
  }
  if (strategyType === 'long_call' || strategyType === 'long_put') {
    return { assetClass: 'option', side: 'buy', positionEffect: 'open', orderType: 'market' };
  }
  if ([
    'bear_call_credit_spread',
    'bull_put_credit_spread',
    'call_credit_butterfly',
    'put_credit_butterfly',
    'call_credit_condor',
    'put_credit_condor',
    'iron_condor',
    'iron_butterfly',
  ].includes(strategyType)) {
    return { assetClass: 'multileg', side: 'sell', positionEffect: 'open', orderType: 'credit' };
  }
  return { assetClass: 'multileg', side: 'buy', positionEffect: 'open', orderType: 'debit' };
}

function _priceFromNetPremium(netPremium) {
  const value = Math.abs(Number(netPremium || 0));
  return value > 0 ? Number(value.toFixed(2)) : null;
}

function _intrinsicAt(price, leg) {
  const strike = Number(leg.strike || 0);
  if (leg.option_type === 'call') return Math.max(price - strike, 0);
  return Math.max(strike - price, 0);
}

function _previewPayoffAt(price, data) {
  const strategyType = data?.strategy_type || '';
  const spot = Number(data?.market_snapshot?.spot || 0);
  const legs = Array.isArray(data?.selected_legs) ? data.selected_legs : [];
  let pnl = 0;
  for (const leg of legs) {
    const intrinsic = _intrinsicAt(price, leg);
    const premium = Number(leg.premium_mid || 0);
    pnl += leg.side === 'long' ? (intrinsic - premium) : (premium - intrinsic);
  }
  if (strategyType === 'covered_call') pnl += (price - spot);
  return pnl;
}

function _buildProposalCurve(data) {
  const spot = Number(data?.market_snapshot?.spot || 0);
  if (!spot) return [];
  const points = [];
  for (let idx = 0; idx <= 24; idx += 1) {
    const price = spot * (0.75 + (idx / 24) * 0.5);
    points.push({ price, profit: _previewPayoffAt(price, data) });
  }
  return points;
}

function _sortedUniquePositive(values) {
  return [...new Set((values || []).map((value) => Number(value || 0)).filter((value) => value > 0))].sort((a, b) => a - b);
}

function _maxAdjacentWidth(strikes) {
  if (!Array.isArray(strikes) || strikes.length < 2) return 0;
  let width = 0;
  for (let idx = 1; idx < strikes.length; idx += 1) {
    width = Math.max(width, Number(strikes[idx] || 0) - Number(strikes[idx - 1] || 0));
  }
  return Number(width.toFixed(2));
}

function _proposalWidth(data) {
  const strikes = _sortedUniquePositive((data?.selected_legs || []).map((leg) => leg?.strike));
  return _maxAdjacentWidth(strikes);
}

function _proposalCapitalAtRisk(data) {
  const strategyType = data?.strategy_type || '';
  const legs = Array.isArray(data?.selected_legs) ? data.selected_legs : [];
  const spot = Number(data?.market_snapshot?.spot || 0);
  const netDebit = Number(data?.net_premium || 0);
  const width = _proposalWidth(data);

  if ([
    'long_call',
    'long_put',
    'bull_call_debit_spread',
    'bear_put_debit_spread',
    'call_debit_butterfly',
    'put_debit_butterfly',
    'call_debit_condor',
    'put_debit_condor',
    'long_straddle',
    'long_strangle',
    'calendar_spread',
    'call_calendar_spread',
    'put_calendar_spread',
    'call_diagonal_debit_spread',
    'put_diagonal_debit_spread',
  ].includes(strategyType)) {
    return Math.max(netDebit, 1e-6) * 100;
  }
  if (strategyType === 'covered_call') {
    return Math.max(spot + Math.min(netDebit, 0), 1e-6) * 100;
  }
  if (strategyType === 'cash_secured_put') {
    const shortPut = legs.find((leg) => leg?.option_type === 'put' && leg?.side === 'short');
    return Math.max(Number(shortPut?.strike || 0) + Math.min(netDebit, 0), 1e-6) * 100;
  }
  if ([
    'bear_call_credit_spread',
    'bull_put_credit_spread',
    'call_credit_butterfly',
    'put_credit_butterfly',
    'call_credit_condor',
    'put_credit_condor',
  ].includes(strategyType)) {
    return Math.max(width + Math.min(netDebit, 0), 1e-6) * 100;
  }
  if (['iron_condor', 'iron_butterfly'].includes(strategyType)) {
    const callStrikes = _sortedUniquePositive(legs.filter((leg) => leg?.option_type === 'call').map((leg) => leg?.strike));
    const putStrikes = _sortedUniquePositive(legs.filter((leg) => leg?.option_type === 'put').map((leg) => leg?.strike));
    const callWidth = callStrikes.length >= 2 ? Number(callStrikes[callStrikes.length - 1] || 0) - Number(callStrikes[0] || 0) : 0;
    const putWidth = putStrikes.length >= 2 ? Number(putStrikes[putStrikes.length - 1] || 0) - Number(putStrikes[0] || 0) : 0;
    return Math.max(Math.max(callWidth, putWidth) + Math.min(netDebit, 0), 1e-6) * 100;
  }
  return Math.max(Math.abs(netDebit), 1e-6) * 100;
}

function _buildSizingPlan(container, data) {
  const monitor = container?.__atlasQuantMonitorSummary || null;
  const balances = monitor?.balances || {};
  const account = monitor?.account_session || {};
  const riskPctInput = parseFloat(container?.querySelector('#aq-order-risk-pct')?.value || '1');
  const riskUsdInput = parseFloat(container?.querySelector('#aq-order-risk-usd')?.value || '0');
  const totalEquity = Number(account.total_equity || balances.total_equity || 0);
  const optionBuyingPower = Number(balances.option_buying_power || 0);
  const cash = Number(balances.cash || 0);
  const capitalBase = totalEquity > 0 ? totalEquity : Math.max(optionBuyingPower, cash, 0);
  const availableCapital = Math.max(optionBuyingPower, cash, totalEquity, 0);
  const riskPct = Number.isFinite(riskPctInput) && riskPctInput > 0 ? riskPctInput : 1;
  const riskBudget = Number.isFinite(riskUsdInput) && riskUsdInput > 0
    ? riskUsdInput
    : (capitalBase > 0 ? capitalBase * (riskPct / 100) : 0);
  const perUnitRisk = _proposalCapitalAtRisk(data);
  const entryPerUnit = Math.abs(Number(data?.net_premium || 0)) * 100;
  const widthPoints = _proposalWidth(data);
  const maxByRisk = riskBudget > 0 && perUnitRisk > 0 ? Math.floor(riskBudget / perUnitRisk) : null;
  const maxByCapital = availableCapital > 0 && perUnitRisk > 0 ? Math.floor(availableCapital / perUnitRisk) : null;
  const candidates = [maxByRisk, maxByCapital].filter((value) => Number.isFinite(value));
  const suggestedSize = candidates.length > 0 ? Math.max(0, Math.min(...candidates)) : 0;

  let warning = '';
  if (perUnitRisk > 0 && suggestedSize < 1) {
    if (riskBudget > 0 && riskBudget < perUnitRisk) {
      warning = 'El riesgo por lote supera el presupuesto configurado.';
    } else if (availableCapital > 0 && availableCapital < perUnitRisk) {
      warning = 'La cuenta activa no cubre un lote con el capital disponible.';
    } else if (!monitor) {
      warning = 'Falta monitor de cuenta para dimensionar el size.';
    }
  }

  return {
    hasMonitor: !!monitor,
    scope: account.scope || '',
    totalEquity,
    optionBuyingPower,
    cash,
    availableCapital,
    capitalBase,
    riskPct,
    riskBudget: Number(riskBudget.toFixed(2)),
    perUnitRisk: Number(perUnitRisk.toFixed(2)),
    entryPerUnit: Number(entryPerUnit.toFixed(2)),
    widthPoints,
    suggestedSize,
    estimatedRiskUsed: Number((suggestedSize * perUnitRisk).toFixed(2)),
    estimatedEntryValue: Number((suggestedSize * entryPerUnit).toFixed(2)),
    warning,
  };
}

function _renderProbabilityProposal(el, data, plan = null) {
  if (!el) return;
  if (!data) {
    el.innerHTML = '';
    return;
  }
  const curve = _buildProposalCurve(data);
  const legs = Array.isArray(data.selected_legs) ? data.selected_legs : [];
  const isTimeSpread = ['calendar_spread', 'call_calendar_spread', 'put_calendar_spread', 'call_diagonal_debit_spread', 'put_diagonal_debit_spread'].includes(data.strategy_type);
  const sizing = plan || null;
  el.innerHTML = `
    <div class="approval-card" style="padding:14px">
      <div style="display:flex;align-items:flex-start;justify-content:space-between;gap:12px;flex-wrap:wrap">
        <div>
          <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center">
            <span class="chip blue">${_esc(data.strategy_type || '—')}</span>
            <span style="font-weight:700">${_esc(data.symbol || '—')}</span>
            <span class="chip ${Number(data.win_rate_pct || 0) >= 50 ? 'green' : 'red'}">Win ${_fmtPct(data.win_rate_pct)}</span>
          </div>
          <div style="font-size:11px;color:var(--text-muted);margin-top:6px">
            Spot ${_fmtNum(data?.market_snapshot?.spot, 2)} · ROI esperado ${_fmtPct(data.expected_roi_pct)} · PnL esperado ${_fmtMoney(data.expected_pnl)}
          </div>
        </div>
        <div style="text-align:right">
          <div style="font-size:18px;font-weight:700;color:${Number(data.net_premium || 0) <= 0 ? 'var(--accent-green)' : 'var(--text-primary)'}">${_fmtMoney(-Number(data.net_premium || 0))}</div>
          <div style="font-size:11px;color:var(--text-muted)">credito/debito neto</div>
        </div>
      </div>
      <div style="display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;margin-top:12px">
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Riesgo 1 lote</div>
          <div class="provider-name" style="font-size:16px">${sizing ? _fmtMoney(sizing.perUnitRisk) : '—'}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Budget</div>
          <div class="provider-name" style="font-size:16px">${sizing ? _fmtMoney(sizing.riskBudget) : '—'}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Size sugerido</div>
          <div class="provider-name" style="font-size:16px;color:${sizing && sizing.suggestedSize >= 1 ? 'var(--accent-green)' : 'var(--text-primary)'}">${sizing ? _esc(String(sizing.suggestedSize || 0)) : '—'}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Width</div>
          <div class="provider-name" style="font-size:16px">${sizing && sizing.widthPoints > 0 ? _fmtNum(sizing.widthPoints, 2) : '—'}</div>
        </div>
      </div>
      <div style="display:flex;gap:8px;flex-wrap:wrap;margin-top:10px">
        ${legs.map((leg) => `<span class="chip ${leg.side === 'long' ? 'green' : 'orange'}">${_esc(leg.side)} ${_esc(leg.option_type)} ${_esc(String(leg.strike))}</span>`).join(' ')}
        ${sizing && sizing.suggestedSize >= 1 ? `<span class="chip green">size ${_esc(String(sizing.suggestedSize))}</span>` : ''}
        ${sizing ? `<span class="chip blue">riesgo usado ${_esc(_fmtMoney(sizing.estimatedRiskUsed))}</span>` : ''}
        ${sizing && sizing.entryPerUnit > 0 ? `<span class="chip accent">prima/lote ${_esc(_fmtMoney(sizing.entryPerUnit))}</span>` : ''}
        ${sizing && !sizing.hasMonitor ? '<span class="chip orange">Sizing parcial sin monitor de cuenta</span>' : ''}
        ${sizing?.warning ? `<span class="chip red">${_esc(sizing.warning)}</span>` : ''}
        ${isTimeSpread ? '<span class="chip orange">Curva simplificada para time spread</span>' : ''}
      </div>
      <div style="margin-top:12px;padding:10px;border:1px solid var(--border);border-radius:10px;background:rgba(255,255,255,0.02)">
        <div style="display:flex;justify-content:space-between;gap:10px;font-size:11px;color:var(--text-muted);margin-bottom:6px">
          <span>Curva previa simplificada</span>
          <span>Contrato lider: ${_esc(data?.selected_contract?.symbol || '—')}</span>
        </div>
        ${_curveSvg(curve)}
      </div>
    </div>
  `;
}

function _applyProbabilityProposal(container, data, plan = null) {
  const meta = _strategyExecutionMeta(data?.strategy_type);
  if (!meta) throw new Error('No se pudo mapear la estrategia seleccionada');
  const setValue = (selector, value) => {
    const el = container.querySelector(selector);
    if (el && value !== undefined && value !== null) el.value = value;
  };
  setValue('#aq-order-asset-class', meta.assetClass);
  setValue('#aq-order-side', meta.side);
  setValue('#aq-order-position-effect', meta.positionEffect);
  setValue('#aq-order-type', meta.orderType);
  setValue('#aq-order-duration', 'day');
  setValue('#aq-order-symbol', data.symbol || '');
  setValue('#aq-order-strategy-type', data.strategy_type || '');
  const price = _priceFromNetPremium(data.net_premium);
  setValue('#aq-order-price', price ?? '');
  if (plan && Number(plan.suggestedSize || 0) >= 1) {
    setValue('#aq-order-size', String(plan.suggestedSize));
  }

  if (meta.assetClass === 'option') {
    setValue('#aq-order-option-symbol', data?.selected_contract?.symbol || data?.selected_legs?.[0]?.symbol || '');
    setValue('#aq-order-legs', '');
    return;
  }

  const legs = [];
  if (data.strategy_type === 'covered_call') {
    legs.push({ instrument_type: 'equity', side: 'buy', quantity: 100 });
  }
  for (const leg of (data.selected_legs || [])) {
    legs.push({
      option_symbol: leg.symbol,
      side: leg.side === 'long' ? 'buy' : 'sell',
      quantity: 1,
    });
  }
  setValue('#aq-order-option-symbol', '');
  setValue('#aq-order-legs', JSON.stringify(legs, null, 2));
}

function _renderOrderResult(el, response, errorText = '') {
  if (!el) return;
  if (!response) {
    el.innerHTML = `<div style="color:var(--accent-red);font-size:12px;padding:8px">${_esc(errorText || 'Sin respuesta')}</div>`;
    return;
  }
  const ok = !!response.ok;
  const data = response.data || {};
  const tradier = data.tradier_response || {};
  const session = data.account_session || {};
  const pdt = data.pdt_status || {};
  const probability = data.probability_gate || null;
  const pdtPanel = _renderPdtBreakdown(pdt);
  el.innerHTML = `
    <div class="approval-card" style="padding:14px">
      <div style="display:flex;align-items:center;justify-content:space-between;gap:10px;flex-wrap:wrap">
        <div style="display:flex;gap:8px;align-items:center;flex-wrap:wrap">
          <span class="chip ${ok ? 'green' : 'red'}">${ok ? 'OK' : 'BLOQUEADA'}</span>
          <span class="chip blue">${_esc(data.mode || 'local')}</span>
          ${data.order_class ? `<span class="chip accent">${_esc(data.order_class)}</span>` : ''}
          ${session.account_id ? `<span class="chip orange">${_esc(session.account_id)}</span>` : ''}
        </div>
        <div style="font-size:12px;color:var(--text-muted)">${_esc(response.error || '')}</div>
      </div>
      <div style="display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px;margin-top:12px">
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Scope</div>
          <div class="provider-name" style="font-size:15px">${_esc(session.scope || '—')}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">PDT</div>
          <div class="provider-name" style="font-size:15px;color:${pdt.blocked_opening ? 'var(--accent-red)' : 'var(--text-primary)'}">${_esc(pdt.reason || 'sin bloqueo')}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Broker status</div>
          <div class="provider-name" style="font-size:15px">${_esc(String(tradier.status || tradier.order_status || tradier.id || 'recibido'))}</div>
        </div>
      </div>
      ${probability ? `<div style="margin-top:12px"><span class="chip ${Number(probability.win_rate_pct || 0) >= Number(probability.min_win_rate_pct || 50) ? 'green' : 'red'}">Gate ${_fmtPct(probability.win_rate_pct)}${probability.min_win_rate_pct !== undefined ? ` / min ${_fmtPct(probability.min_win_rate_pct)}` : ''}</span></div>` : ''}
      ${pdtPanel}
      <div style="margin-top:12px;font-size:11px;color:var(--text-muted)">Payload enviado a Tradier</div>
      <pre style="margin-top:6px;white-space:pre-wrap;word-break:break-word;background:rgba(255,255,255,0.03);border:1px solid var(--border);border-radius:10px;padding:10px;font-size:11px;max-height:220px;overflow:auto">${_prettyJson(data.request_payload || data)}</pre>
      <div style="margin-top:12px;font-size:11px;color:var(--text-muted)">Respuesta del broker</div>
      <pre style="margin-top:6px;white-space:pre-wrap;word-break:break-word;background:rgba(255,255,255,0.03);border:1px solid var(--border);border-radius:10px;padding:10px;font-size:11px;max-height:240px;overflow:auto">${_prettyJson(tradier)}</pre>
    </div>
  `;
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

function _renderStatusSummary(container, payload = {}) {
  const dot  = container.querySelector('#aq-dot');
  const stat = container.querySelector('#aq-status');
  const up   = container.querySelector('#aq-uptime');
  const strat = container.querySelector('#aq-strategies');
  const pos  = container.querySelector('#aq-positions');
  if (dot) dot.className = 'provider-dot ok';
  if (stat) {
    stat.textContent = payload.service_status || 'ONLINE';
    stat.style.color = payload.service_status === 'OFFLINE' ? 'var(--accent-red)' : 'var(--accent-green)';
  }
  if (up) up.textContent = `${Math.floor((payload.uptime_sec || 0) / 60)} min`;
  if (strat) {
    const items = Array.isArray(payload.active_strategies) ? payload.active_strategies : [];
    strat.textContent = items.join(', ') || '—';
  }
  if (pos) pos.textContent = payload.open_positions ?? 0;
}

async function _fetchHealth(container) {
  try {
    const qs = _monitorParams(container);
    const r = await fetch(`${QUANT_API_V2}/status${qs ? `?${qs}` : ''}`, { headers: _headers() });
    const d = await r.json();
    if (!d?.ok || !d?.data) throw new Error(d?.error || 'Status no disponible');
    _renderStatusSummary(container, d.data);
  } catch {
    const dot  = container.querySelector('#aq-dot');
    const stat = container.querySelector('#aq-status');
    if (dot) dot.className = 'provider-dot down';
    if (stat) { stat.textContent = 'OFFLINE'; stat.style.color = 'var(--accent-red)'; }
  }
}

async function _fetchPositions(container) {
  const el = container.querySelector('#aq-pos-list');
  if (!el) return;
  try {
    const r = await fetch(`${QUANT_API_V2}/positions`, { headers: _headers() });
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

function _renderMonitorSummary(container, m = {}) {
  const summaryEl = container.querySelector('#aq-monitor-summary');
  const alertsEl = container.querySelector('#aq-monitor-alerts');
  const listEl = container.querySelector('#aq-monitor-list');
  if (!summaryEl || !alertsEl || !listEl) return;
  container.__atlasQuantMonitorSummary = m;
  const acct = m.account_session || {};
  const pdt = m.pdt_status || {};
  const alerts = Array.isArray(m.alerts) ? m.alerts : [];
  const strategies = Array.isArray(m.strategies) ? m.strategies : [];
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
        <div style="margin-top:6px">
          ${_renderPdtBreakdown({ ...pdt, reason: pdt.reason || 'sin observaciones' }, { compact: true })}
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
      <span class="chip ${Number(pdt.day_trades_remaining ?? 0) > 0 ? 'green' : 'red'}">Remanente: ${_esc(String(pdt.day_trades_remaining ?? 0))}</span>
      <span class="chip accent">Broker: ${_esc(String(pdt.broker_day_trades_last_window ?? 0))}</span>
      <span class="chip orange">Ledger hoy: ${_esc(String(pdt.ledger_day_trades_today ?? 0))}</span>
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
  if (container.__atlasQuantLastProposal) {
    const sizingPlan = _buildSizingPlan(container, container.__atlasQuantLastProposal);
    if (Number(sizingPlan.suggestedSize || 0) >= 1) {
      const sizeEl = container.querySelector('#aq-order-size');
      if (sizeEl) sizeEl.value = String(sizingPlan.suggestedSize);
    }
    _renderProbabilityProposal(
      container.querySelector('#aq-order-proposal'),
      container.__atlasQuantLastProposal,
      sizingPlan,
    );
  }
}

async function _fetchMonitorSummary(container) {
  const summaryEl = container.querySelector('#aq-monitor-summary');
  const alertsEl = container.querySelector('#aq-monitor-alerts');
  const listEl = container.querySelector('#aq-monitor-list');
  if (!summaryEl || !alertsEl || !listEl) return;
  const qs = _monitorParams(container);
  try {
    const r = await fetch(`${QUANT_API_V2}/monitor/summary${qs ? `?${qs}` : ''}`, { headers: _headers() });
    const d = await r.json();
    if (!d?.ok || !d?.data) throw new Error(d?.error || 'No se pudo cargar monitor');
    _renderMonitorSummary(container, d.data);
    return d.data;
  } catch (e) {
    container.__atlasQuantMonitorSummary = null;
    summaryEl.innerHTML = `<div class="empty-state" style="padding:12px 0"><div class="empty-title" style="color:var(--accent-red)">Monitor no disponible</div><div class="empty-sub">${_esc(e.message || 'Error')}</div></div>`;
    alertsEl.innerHTML = '';
    listEl.innerHTML = '';
    return null;
  }
}

function _scheduleSocketReconnect(container) {
  clearTimeout(LIVE_SOCKET_RETRY);
  LIVE_SOCKET_RETRY = setTimeout(() => {
    if (ACTIVE_CONTAINER === container) _connectLiveUpdates(container);
  }, WS_RECONNECT_MS);
}

function _connectLiveUpdates(container) {
  ACTIVE_CONTAINER = container;
  clearTimeout(LIVE_SOCKET_RETRY);
  LIVE_SOCKET_RETRY = null;
  _stopLiveSocket();
  LIVE_SOCKET_MANUAL_CLOSE = false;
  _setFeedState(container, 'connecting');
  let socket;
  try {
    socket = new window.WebSocket(_wsUrl(container));
  } catch {
    _setFeedState(container, 'fallback');
    _startFallbackPolling(container);
    _scheduleSocketReconnect(container);
    return;
  }
  LIVE_SOCKET = socket;

  socket.addEventListener('open', () => {
    if (LIVE_SOCKET !== socket || ACTIVE_CONTAINER !== container) return;
    stop(POLL_ID);
    stop(MONITOR_POLL_ID);
    _setFeedState(container, 'live');
  });

  socket.addEventListener('message', (event) => {
    if (LIVE_SOCKET !== socket || ACTIVE_CONTAINER !== container) return;
    try {
      const message = JSON.parse(event.data || '{}');
      if (message.type === 'quant.live_update') {
        if (message.status) _renderStatusSummary(container, message.status);
        if (message.monitor_summary) _renderMonitorSummary(container, message.monitor_summary);
        _setFeedState(container, 'live', message.generated_at || '');
        return;
      }
      if (message.type === 'quant.live_error') {
        _setFeedState(container, 'error', message.error || 'socket');
      }
    } catch {
      _setFeedState(container, 'error', 'payload');
    }
  });

  socket.addEventListener('error', () => {
    if (LIVE_SOCKET !== socket || ACTIVE_CONTAINER !== container) return;
    _setFeedState(container, 'error');
  });

  socket.addEventListener('close', () => {
    if (LIVE_SOCKET === socket) LIVE_SOCKET = null;
    if (ACTIVE_CONTAINER !== container || LIVE_SOCKET_MANUAL_CLOSE || socket.__atlasManualClose) return;
    _setFeedState(container, 'fallback');
    _startFallbackPolling(container);
    _scheduleSocketReconnect(container);
  });
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
            <span class="chip orange" id="aq-live-feed">REST fallback</span>
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

          <div class="section-title" style="margin-bottom:10px">
            Orden Tradier
            <span class="chip orange" style="font-size:10px;margin-left:6px">Preview por defecto</span>
          </div>
          <div class="approval-card" style="padding:14px;margin-bottom:20px">
            <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-bottom:10px">
              <select id="aq-order-preset" class="config-input" style="width:280px">
                <option value="custom" selected>Preset rapido: custom</option>
                <option value="long_call">long_call</option>
                <option value="bull_call_debit_spread">bull_call_debit_spread</option>
                <option value="bear_put_debit_spread">bear_put_debit_spread</option>
                <option value="bear_call_credit_spread">bear_call_credit_spread</option>
                <option value="bull_put_credit_spread">bull_put_credit_spread</option>
                <option value="iron_condor">iron_condor</option>
                <option value="iron_butterfly">iron_butterfly</option>
                <option value="call_calendar_spread">call_calendar_spread</option>
                <option value="call_diagonal_debit_spread">call_diagonal_debit_spread</option>
                <option value="covered_call">covered_call</option>
              </select>
              <span class="chip blue">Rellena estructura y legs de ejemplo</span>
            </div>
            <div style="display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;margin-bottom:10px">
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Simbolo base</div>
                <input id="aq-order-symbol" class="config-input" value="AAPL" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Asset class</div>
                <select id="aq-order-asset-class" class="config-input" style="width:100%">
                  <option value="option" selected>Option</option>
                  <option value="equity">Equity</option>
                  <option value="multileg">Multileg</option>
                  <option value="combo">Combo</option>
                  <option value="auto">Auto</option>
                </select>
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Side</div>
                <select id="aq-order-side" class="config-input" style="width:100%">
                  <option value="buy">buy</option>
                  <option value="sell">sell</option>
                  <option value="buy_to_open">buy_to_open</option>
                  <option value="sell_to_open">sell_to_open</option>
                  <option value="buy_to_close">buy_to_close</option>
                  <option value="sell_to_close">sell_to_close</option>
                </select>
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Position effect</div>
                <select id="aq-order-position-effect" class="config-input" style="width:100%">
                  <option value="auto" selected>auto</option>
                  <option value="open">open</option>
                  <option value="close">close</option>
                </select>
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Option symbol</div>
                <input id="aq-order-option-symbol" class="config-input" placeholder="AAPL260417C00200000" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Size</div>
                <input id="aq-order-size" class="config-input" type="number" value="1" min="0.01" step="0.01" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Order type</div>
                <select id="aq-order-type" class="config-input" style="width:100%">
                  <option value="market" selected>market</option>
                  <option value="limit">limit</option>
                  <option value="stop">stop</option>
                  <option value="stop_limit">stop_limit</option>
                  <option value="debit">debit</option>
                  <option value="credit">credit</option>
                  <option value="even">even</option>
                </select>
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Duration</div>
                <select id="aq-order-duration" class="config-input" style="width:100%">
                  <option value="day" selected>day</option>
                  <option value="gtc">gtc</option>
                  <option value="pre">pre</option>
                  <option value="post">post</option>
                </select>
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Price</div>
                <input id="aq-order-price" class="config-input" type="number" step="0.01" placeholder="2.15" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Stop price</div>
                <input id="aq-order-stop-price" class="config-input" type="number" step="0.01" placeholder="1.95" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Strategy type</div>
                <select id="aq-order-strategy-type" class="config-input" style="width:100%">
                  <option value="">sin gate</option>
                  <option value="long_call">long_call</option>
                  <option value="long_put">long_put</option>
                  <option value="covered_call">covered_call</option>
                  <option value="cash_secured_put">cash_secured_put</option>
                  <option value="bull_call_debit_spread">bull_call_debit_spread</option>
                  <option value="bear_put_debit_spread">bear_put_debit_spread</option>
                  <option value="bear_call_credit_spread">bear_call_credit_spread</option>
                  <option value="bull_put_credit_spread">bull_put_credit_spread</option>
                  <option value="call_debit_butterfly">call_debit_butterfly</option>
                  <option value="put_debit_butterfly">put_debit_butterfly</option>
                  <option value="call_credit_butterfly">call_credit_butterfly</option>
                  <option value="put_credit_butterfly">put_credit_butterfly</option>
                  <option value="call_debit_condor">call_debit_condor</option>
                  <option value="put_debit_condor">put_debit_condor</option>
                  <option value="call_credit_condor">call_credit_condor</option>
                  <option value="put_credit_condor">put_credit_condor</option>
                  <option value="long_straddle">long_straddle</option>
                  <option value="long_strangle">long_strangle</option>
                  <option value="iron_condor">iron_condor</option>
                  <option value="iron_butterfly">iron_butterfly</option>
                  <option value="calendar_spread">calendar_spread</option>
                  <option value="call_calendar_spread">call_calendar_spread</option>
                  <option value="put_calendar_spread">put_calendar_spread</option>
                  <option value="call_diagonal_debit_spread">call_diagonal_debit_spread</option>
                  <option value="put_diagonal_debit_spread">put_diagonal_debit_spread</option>
                </select>
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Min win rate</div>
                <input id="aq-order-min-win-rate" class="config-input" type="number" value="55" min="0" max="100" step="0.1" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Tag</div>
                <input id="aq-order-tag" class="config-input" placeholder="atlas-ui" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Risk % idea</div>
                <input id="aq-order-risk-pct" class="config-input" type="number" value="1" min="0.1" max="25" step="0.1" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Tope riesgo $</div>
                <input id="aq-order-risk-usd" class="config-input" type="number" min="0" step="1" placeholder="auto por equity" style="width:100%">
              </div>
            </div>
            <div style="display:flex;align-items:center;gap:10px;flex-wrap:wrap;margin-bottom:10px">
              <label style="display:flex;align-items:center;gap:6px;font-size:12px;color:var(--text-muted)">
                <input id="aq-order-probability-enabled" type="checkbox" checked>
                Gate de probabilidad
              </label>
              <span class="chip blue">Usa la cuenta activa del monitor</span>
            </div>
            <div style="display:flex;gap:8px;flex-wrap:wrap;margin-bottom:10px">
              <button class="action-btn" id="aq-btn-order-build">Autoarmar desde estrategia</button>
              <span class="chip accent">Consulta /probability/options y rellena contratos reales</span>
            </div>
            <div id="aq-order-proposal" style="margin-bottom:12px"></div>
            <div>
              <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Legs JSON para multileg/combo</div>
              <textarea id="aq-order-legs" class="config-input" style="width:100%;min-height:84px;resize:vertical" placeholder='[{"option_symbol":"AAPL260417C00200000","side":"buy","quantity":1}]'></textarea>
            </div>
            <div style="display:flex;gap:8px;flex-wrap:wrap;margin-top:12px">
              <button class="action-btn" id="aq-btn-order-preview">Preview Tradier</button>
              <button class="action-btn primary" id="aq-btn-order-submit">Enviar real</button>
            </div>
            <div id="aq-order-result" style="margin-top:12px"></div>
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
      _fetchHealth(container);
      _fetchMonitorSummary(container);
      window.AtlasToast?.show('Monitor avanzado actualizado', 'info');
    });

    container.querySelector('#aq-account-scope')?.addEventListener('change', () => {
      _fetchHealth(container);
      _fetchMonitorSummary(container);
      _connectLiveUpdates(container);
    });

    container.querySelector('#aq-account-id')?.addEventListener('change', () => {
      _fetchHealth(container);
      _fetchMonitorSummary(container);
      _connectLiveUpdates(container);
    });

    container.querySelector('#aq-order-preset')?.addEventListener('change', (ev) => {
      _applyOrderPreset(container, ev.target?.value || 'custom');
    });

    const refreshProposalSizing = () => {
      if (!container.__atlasQuantLastProposal) return;
      const sizingPlan = _buildSizingPlan(container, container.__atlasQuantLastProposal);
      if (Number(sizingPlan.suggestedSize || 0) >= 1) {
        const sizeEl = container.querySelector('#aq-order-size');
        if (sizeEl) sizeEl.value = String(sizingPlan.suggestedSize);
      }
      _renderProbabilityProposal(
        container.querySelector('#aq-order-proposal'),
        container.__atlasQuantLastProposal,
        sizingPlan,
      );
    };

    container.querySelector('#aq-order-risk-pct')?.addEventListener('input', refreshProposalSizing);
    container.querySelector('#aq-order-risk-usd')?.addEventListener('input', refreshProposalSizing);

    const buildOrderFromStrategy = async () => {
      const resultEl = container.querySelector('#aq-order-result');
      const proposalEl = container.querySelector('#aq-order-proposal');
      const symbol = container.querySelector('#aq-order-symbol')?.value?.trim() || '';
      const strategyType = container.querySelector('#aq-order-strategy-type')?.value || '';
      if (!symbol) throw new Error('Indica el simbolo base');
      if (!strategyType) throw new Error('Selecciona strategy type');
      if (proposalEl) {
        proposalEl.innerHTML = '<div style="text-align:center;padding:12px"><div class="spinner" style="margin:0 auto;display:block"></div><div style="color:var(--text-muted);font-size:11px;margin-top:6px">Calculando estructura optima...</div></div>';
      }
      const payload = {
        symbol,
        strategy_type: strategyType,
        account_scope: _orderScope(container),
        account_id: _orderAccountId(container),
      };
      const r = await fetch(`${QUANT_API_V2}/probability/options`, {
        method: 'POST',
        headers: _headers(),
        body: JSON.stringify(payload),
      });
      const d = await r.json();
      if (!d?.ok || !d?.data) throw new Error(d?.error || 'No se pudo construir la estrategia');
      await _fetchMonitorSummary(container);
      container.__atlasQuantLastProposal = d.data;
      const sizingPlan = _buildSizingPlan(container, d.data);
      _applyProbabilityProposal(container, d.data, sizingPlan);
      _renderProbabilityProposal(proposalEl, d.data, sizingPlan);
      if (resultEl) resultEl.innerHTML = '';
      window.AtlasToast?.show('Estructura cargada desde el motor de probabilidad', 'success');
    };

    container.querySelector('#aq-btn-order-build')?.addEventListener('click', async () => {
      try {
        await buildOrderFromStrategy();
      } catch (e) {
        container.__atlasQuantLastProposal = null;
        _renderProbabilityProposal(container.querySelector('#aq-order-proposal'), null);
        const errorEl = container.querySelector('#aq-order-result');
        if (errorEl) {
          errorEl.innerHTML = `<div style="color:var(--accent-red);font-size:12px;padding:8px">${_esc(e.message || 'No se pudo construir la estrategia')}</div>`;
        }
        window.AtlasToast?.show(e.message || 'No se pudo construir la estrategia', 'error');
      }
    });

    const runOrder = async (preview) => {
      const resEl = container.querySelector('#aq-order-result');
      if (resEl) {
        resEl.innerHTML = '<div style="text-align:center;padding:12px"><div class="spinner" style="margin:0 auto;display:block"></div><div style="color:var(--text-muted);font-size:11px;margin-top:6px">Enviando a Tradier...</div></div>';
      }
      try {
        const payload = _buildOrderPayload(container, { preview });
        if (!preview && payload.account_scope === 'live') {
          const ok = window.confirm(
            `Confirmar envio REAL a Tradier\nCuenta: ${payload.account_id || 'auto'}\nSimbolo: ${payload.symbol}\nEstrategia: ${payload.strategy_type || payload.asset_class}\nSide: ${payload.side}\nSize: ${payload.size}`
          );
          if (!ok) {
            if (resEl) resEl.innerHTML = '<div class="chip orange">Envio cancelado por el usuario</div>';
            return;
          }
          payload.live_confirmed = true;
        }
        const r = await fetch(`${QUANT_API_V2}/order`, {
          method: 'POST',
          headers: _headers(),
          body: JSON.stringify(payload),
        });
        const d = await r.json();
        _renderOrderResult(resEl, d, d?.error || 'Orden sin respuesta');
        if (d?.ok) {
          window.AtlasToast?.show(preview ? 'Preview Tradier generado' : 'Orden enviada a Tradier', 'success');
          _fetchMonitorSummary(container);
        } else {
          window.AtlasToast?.show(d?.error || 'La orden fue bloqueada', 'warning');
        }
      } catch (e) {
        _renderOrderResult(resEl, null, e.message || 'No se pudo enviar la orden');
        window.AtlasToast?.show(e.message || 'No se pudo enviar la orden', 'error');
      }
    };

    container.querySelector('#aq-btn-order-preview')?.addEventListener('click', () => {
      runOrder(true);
    });

    container.querySelector('#aq-btn-order-submit')?.addEventListener('click', () => {
      runOrder(false);
    });

    // ejecutar señal rápida
    container.querySelector('#aq-btn-run-signal')?.addEventListener('click', async () => {
      const symbol = container.querySelector('#aq-symbol-input')?.value?.trim() || 'BTC/USDT';
      const tf     = container.querySelector('#aq-tf-select')?.value || '1h';
      const resEl  = container.querySelector('#aq-signal-result');
      if (resEl) resEl.innerHTML = '<div class="spinner" style="margin:0 auto;display:block"></div>';
      try {
        const r = await fetch(`${QUANT_API_V2}/signal`, {
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
        const r = await fetch(`${QUANT_API_V2}/backtest`, {
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
    _startFallbackPolling(container);
    _connectLiveUpdates(container);
  },

  destroy() {
    stop(POLL_ID);
    stop(MONITOR_POLL_ID);
    ACTIVE_CONTAINER = null;
    _stopLiveSocket();
  },

  badge() { return null; },
};

window.AtlasModuleQuantBot = { id: 'atlas-quant' };
