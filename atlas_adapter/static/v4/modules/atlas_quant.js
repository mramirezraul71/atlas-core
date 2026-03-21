/**
 * ATLAS v4.3 — Atlas Code-Quant
 * Shortcut y panel de estado del sistema de trading algorítmico.
 * API interna en puerto 8792.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID  = 'atlas-quant-module';
const MONITOR_POLL_ID = 'atlas-quant-monitor';
const JOURNAL_POLL_ID = 'atlas-quant-journal';
const OPERATION_POLL_ID = 'atlas-quant-operation';
const SCANNER_POLL_ID = 'atlas-quant-scanner';
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
  equity_buy: {
    assetClass: 'equity',
    side: 'buy',
    positionEffect: 'open',
    orderType: 'market',
    duration: 'day',
    strategyType: '',
    size: 1,
    optionSymbol: '',
    legs: [],
    probabilityEnabled: false,
  },
  equity_sell: {
    assetClass: 'equity',
    side: 'sell',
    positionEffect: 'close',
    orderType: 'market',
    duration: 'day',
    strategyType: '',
    size: 1,
    optionSymbol: '',
    legs: [],
    probabilityEnabled: false,
  },
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

const OPERATION_PROFILES = {
  paper_safe: {
    account_scope: 'paper',
    paper_only: true,
    auton_mode: 'off',
    executor_mode: 'disabled',
    vision_mode: 'direct_nexus',
    require_operator_present: true,
    operator_present: true,
    screen_integrity_ok: true,
    sentiment_score: 0,
    min_auton_win_rate_pct: 75,
    max_level4_bpr_pct: 10,
    auto_pause_on_operational_errors: true,
    operational_error_limit: 2,
    notes: 'paper safe camara directa',
  },
  paper_supervised: {
    account_scope: 'paper',
    paper_only: true,
    auton_mode: 'paper_supervised',
    executor_mode: 'paper_api',
    vision_mode: 'direct_nexus',
    require_operator_present: false,
    operator_present: true,
    screen_integrity_ok: true,
    sentiment_score: 0.15,
    min_auton_win_rate_pct: 65,
    max_level4_bpr_pct: 20,
    auto_pause_on_operational_errors: true,
    operational_error_limit: 3,
    notes: 'paper pilot camara directa',
  },
  paper_autonomous: {
    account_scope: 'paper',
    paper_only: true,
    auton_mode: 'paper_autonomous',
    executor_mode: 'paper_api',
    vision_mode: 'direct_nexus',
    require_operator_present: false,
    operator_present: true,
    screen_integrity_ok: true,
    sentiment_score: 0.2,
    min_auton_win_rate_pct: 72,
    max_level4_bpr_pct: 15,
    auto_pause_on_operational_errors: true,
    operational_error_limit: 3,
    notes: 'paper autonomous trial camara directa',
  },
};

function _esc(s) {
  const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML;
}

function _headers() {
  return { 'X-Api-Key': API_KEY, 'Content-Type': 'application/json' };
}

function _sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function _fetchJsonRetry(url, options = {}, { retries = 2, retryDelayMs = 350 } = {}) {
  let lastError = null;
  for (let attempt = 0; attempt <= retries; attempt += 1) {
    try {
      const response = await fetch(url, options);
      const raw = await response.text();
      let data = {};
      try {
        data = raw ? JSON.parse(raw) : {};
      } catch {
        throw new Error(response.ok ? 'Respuesta invalida del servicio' : `HTTP ${response.status}`);
      }
      if (!response.ok) {
        const message = data?.error || data?.detail || `HTTP ${response.status}`;
        const transientStatus = [502, 503, 504].includes(response.status);
        if (transientStatus && attempt < retries) {
          await _sleep(retryDelayMs * (attempt + 1));
          continue;
        }
        throw new Error(message);
      }
      return data;
    } catch (error) {
      lastError = error;
      const text = String(error?.message || error || '');
      const transientError =
        text.includes('Failed to fetch') ||
        text.includes('NetworkError') ||
        text.includes('ERR_CONNECTION') ||
        text.includes('Load failed') ||
        text.includes('fetch');
      if (transientError && attempt < retries) {
        await _sleep(retryDelayMs * (attempt + 1));
        continue;
      }
      throw error;
    }
  }
  throw lastError || new Error('No se pudo contactar el servicio');
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

function _fmtPrice(v) {
  if (v === null || v === undefined || Number.isNaN(Number(v))) return 'â€”';
  return `$${Number(v).toFixed(2)}`;
}

function _moneyColor(v) {
  const n = Number(v || 0);
  return n >= 0 ? 'var(--accent-green)' : 'var(--accent-red)';
}

function _labelScope(value) {
  return value === 'live' ? 'real' : value === 'paper' ? 'simulada' : (value || '—');
}

function _labelOperationMode(value) {
  return {
    off: 'apagado',
    paper_supervised: 'simulada supervisada',
    paper_autonomous: 'simulada autonoma',
  }[value] || (value || '—');
}

function _labelExecutorMode(value) {
  return {
    disabled: 'desactivado',
    paper_api: 'API simulada',
    desktop_dry_run: 'simulación de escritorio',
  }[value] || (value || '—');
}

function _labelVisionMode(value) {
  return {
    off: 'apagada',
    manual: 'manual',
    desktop_capture: 'captura de escritorio',
    direct_nexus: 'Insta360 activa por robot',
    atlas_push_bridge: 'Insta360 via puente PUSH',
    insta360_pending: 'Insta360 nativa deshabilitada',
  }[value] || (value || '—');
}

function _labelProfile(value) {
  return {
    paper_safe: 'simulada segura',
    paper_supervised: 'simulada supervisada',
    paper_autonomous: 'simulada autonoma',
  }[value] || (value || '—');
}

function _labelTradeStatus(value) {
  return {
    open: 'abierta',
    closed: 'cerrada',
  }[value] || (value || '—');
}

function _labelSide(value) {
  return {
    buy: 'compra',
    sell: 'venta',
    sell_short: 'venta en corto',
    buy_to_cover: 'compra para cubrir',
    buy_to_open: 'compra para abrir',
    sell_to_open: 'venta para abrir',
    buy_to_close: 'compra para cerrar',
    sell_to_close: 'venta para cerrar',
    long: 'larga',
    short: 'corta',
  }[value] || (value || '—');
}

function _translateText(text) {
  let value = String(text || '');
  if (!value) return value;
  const replacements = [
    [/^PDT guard rail only applies to live accounts\.?$/i, 'La protección PDT solo aplica a cuentas reales.'],
    [/^Emergency stop is active\.?$/i, 'El paro de emergencia está activo.'],
    [/^Emergency stop active$/i, 'El paro de emergencia está activo.'],
    [/^Control plane locked to paper-first mode\.?$/i, 'El plano de control está bloqueado en modo solo simulada.'],
    [/^Operator presence is required by current policy\.?$/i, 'La política actual exige presencia del operador.'],
    [/^Screen integrity check is not OK\.?$/i, 'La verificación de pantallas no es correcta.'],
    [/^Autonomous mode is OFF\.?$/i, 'El modo autónomo está apagado.'],
    [/^PDT blocked opening trades\.?$/i, 'PDT bloqueó nuevas aperturas.'],
    [/^Winning probability below gate \((.+)\)\.?$/i, 'La probabilidad de éxito está por debajo del umbral ($1).'],
    [/^Level 4 credit structure uses (.+)\.$/i, 'La estructura de crédito de nivel 4 consume $1.'],
    [/^Live execution remains disabled while paper_only is true\.?$/i, 'La ejecución real sigue deshabilitada mientras solo simulada este activa.'],
    [/^Executor blocked operation\.?$/i, 'El ejecutor bloqueó la operación.'],
    [/^Unsupported executor mode '([^']+)'\.?$/i, 'Modo de ejecutor no soportado: $1.'],
    [/^strategy_type is missing; autonomous gate is operating without Monte Carlo validation\.?$/i, 'Falta el tipo de estrategia; el filtro autónomo opera sin validación Monte Carlo.'],
    [/^Manual score for paper-first trials until Grok sentiment is wired\.?$/i, 'Puntaje manual para pruebas simuladas hasta conectar Grok.'],
    [/^Manual provider active; no automatic capture attempted\.?$/i, 'Proveedor manual activo; no se intentó una captura automática.'],
    [/^Insta360 provider not wired yet; pending SDK bridge\.?$/i, 'El proveedor Insta360 aún no está integrado; falta el puente con el SDK.'],
    [/^Context snapshot captured$/i, 'Captura de contexto registrada.'],
    [/^Paper-first executor\. Desktop automation is dry-run only\.?$/i, 'Ejecutor orientado a simulada. La automatizacion de escritorio esta solo en simulacion.'],
    [/^PyAutoGUI supports fail-safe corners and built-in pause; desktop execution remains dry-run in this prototype\.?$/i, 'PyAutoGUI permite esquinas de seguridad y pausa integrada; la ejecución de escritorio sigue en simulación en este prototipo.'],
    [/^evaluate$/i, 'evaluar'],
    [/^preview$/i, 'previsualizar'],
    [/^submit$/i, 'enviar'],
    [/^eligible$/i, 'elegible'],
    [/^preview_ready$/i, 'previsualizacion lista'],
    [/^submit_ready$/i, 'envio listo'],
    [/^allowed$/i, 'permitido'],
    [/^blocked$/i, 'bloqueado'],
    [/^ONLINE$/i, 'EN LINEA'],
    [/^OFFLINE$/i, 'FUERA DE LINEA'],
    [/^ok$/i, 'OK'],
    [/^delta$/i, 'delta'],
    [/^theta$/i, 'theta'],
    [/^vega$/i, 'vega'],
    [/^direction$/i, 'direccion'],
    [/^volatility$/i, 'volatilidad'],
    [/^time$/i, 'tiempo'],
    [/^mixed$/i, 'mixto'],
    [/^paper pilot preview$/i, 'prueba previa en simulada'],
    [/^paper pilot$/i, 'piloto en simulada'],
    [/^paper safe$/i, 'simulada segura'],
    [/^paper autonomous trial$/i, 'prueba autonoma en simulada'],
    [/^manual_stop$/i, 'paro manual'],
    [/^emergency_stop$/i, 'paro de emergencia'],
    [/^emergency_reset$/i, 'reinicio del paro'],
  ];
  for (const [pattern, replacement] of replacements) {
    value = value.replace(pattern, replacement);
  }
  return value;
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
    fallback: 'REST de respaldo',
    error: 'WS con error',
  }[state] || 'Feed';
  el.className = `chip ${tone}`;
  el.textContent = detail ? `${label} · ${detail}` : label;
}

function _startFallbackPolling(container) {
  stop(POLL_ID);
  stop(MONITOR_POLL_ID);
  stop(OPERATION_POLL_ID);
  stop(SCANNER_POLL_ID);
  poll(POLL_ID, null, 15000, () => _fetchHealth(container));
  poll(MONITOR_POLL_ID, null, 60000, () => _fetchMonitorSummary(container));
  poll(OPERATION_POLL_ID, null, 20000, () => _fetchOperationStatus(container));
  poll(SCANNER_POLL_ID, null, 45000, () => _fetchScannerReport(container));
}

function _startJournalPolling(container) {
  stop(JOURNAL_POLL_ID);
  poll(JOURNAL_POLL_ID, null, 120000, () => _refreshJournalPanel(container));
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

function _activateQuantView(container, view = 'scanner') {
  if (!container) return;
  const normalized = String(view || 'scanner');
  container.__atlasQuantView = normalized;
  container.querySelectorAll('[data-quant-view]').forEach((el) => {
    const shouldShow = String(el.getAttribute('data-quant-view') || '') === normalized;
    el.style.display = shouldShow ? '' : 'none';
  });
  container.querySelectorAll('[data-quant-view-btn]').forEach((btn) => {
    const isActive = String(btn.getAttribute('data-quant-view-btn') || '') === normalized;
    btn.classList.toggle('primary', isActive);
  });
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
  if (!Number.isFinite(size) || size <= 0) throw new Error('Tamaño inválido');

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
    if (!strategyType) throw new Error('Selecciona un tipo de estrategia para activar el filtro');
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
      ${brokerPreview !== undefined && brokerPreview !== null ? `<span class="chip ${Number(brokerPreview) >= 3 ? 'red' : 'blue'}">previsualizacion del broker: ${_esc(String(brokerPreview))}</span>` : ''}
      <span class="chip ${canOpen ? 'green' : 'red'}">${canOpen ? 'puede abrir' : 'no puede abrir'}</span>
    </div>
  `;
  if (compact) {
    return `
      <div style="display:flex;flex-direction:column;gap:8px">
        ${chipRow}
        <div class="stat-card-sub">${_esc(_translateText(pdt.reason || 'sin observaciones'))}</div>
      </div>
    `;
  }
  return `
    <div style="margin-top:12px;padding:10px;border:1px solid var(--border);border-radius:10px;background:rgba(255,255,255,0.02)">
      <div style="font-size:11px;color:var(--text-muted);margin-bottom:8px">Desglose PDT</div>
      ${chipRow}
      <div style="font-size:11px;color:var(--text-muted);margin-top:8px">${_esc(_translateText(pdt.reason || 'sin observaciones'))}</div>
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
  const setChecked = (selector, value) => {
    const el = container.querySelector(selector);
    if (el) el.checked = !!value;
  };
  setValue('#aq-order-asset-class', preset.assetClass);
  setValue('#aq-order-side', preset.side);
  setValue('#aq-order-position-effect', preset.positionEffect);
  setValue('#aq-order-type', preset.orderType);
  setValue('#aq-order-duration', preset.duration);
  setValue('#aq-order-strategy-type', preset.strategyType || '');
  setValue('#aq-order-option-symbol', preset.optionSymbol || '');
  if (preset.size !== undefined) setValue('#aq-order-size', String(preset.size));
  setValue('#aq-order-legs', JSON.stringify(preset.legs || [], null, 2));
  if (preset.probabilityEnabled !== undefined) {
    setChecked('#aq-order-probability-enabled', preset.probabilityEnabled);
  }
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

function _buildSelectorRiskCurve(payload = null) {
  if (!payload) return [];
  if (payload.probability) return _buildProposalCurve(payload.probability);
  const candidate = payload.candidate || {};
  const sizePlan = payload.size_plan || {};
  const entryPlan = payload.entry_plan || {};
  const spot = Number(candidate.price || 0);
  const perUnitRisk = Number(sizePlan.per_unit_risk_usd || 0);
  const size = Number(sizePlan.suggested_size || 0);
  if (!spot || !perUnitRisk || !size) return [];
  const stopDistance = Math.max(perUnitRisk, spot * 0.01);
  const expectedMovePct = Math.max(Number(entryPlan.expected_move_pct || 1.5), 1.0) / 100.0;
  const targetDistance = spot * expectedMovePct;
  const direction = String(candidate.direction || '').toLowerCase() === 'bajista' ? -1 : 1;
  const points = [];
  for (let idx = 0; idx <= 24; idx += 1) {
    const price = spot * (0.75 + (idx / 24) * 0.5);
    const move = (price - spot) * direction;
    const pnl = move * size;
    const capped = move < 0 ? Math.max(pnl, -(stopDistance * size)) : Math.min(pnl, targetDistance * size * 1.15);
    points.push({ price, profit: capped });
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
            <span class="chip ${Number(data.win_rate_pct || 0) >= 50 ? 'green' : 'red'}">Prob. exito ${_fmtPct(data.win_rate_pct)}</span>
          </div>
          <div style="font-size:11px;color:var(--text-muted);margin-top:6px">
            Precio spot ${_fmtNum(data?.market_snapshot?.spot, 2)} · ROI esperado ${_fmtPct(data.expected_roi_pct)} · PnL esperado ${_fmtMoney(data.expected_pnl)}
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
          <div class="provider-role">Presupuesto</div>
          <div class="provider-name" style="font-size:16px">${sizing ? _fmtMoney(sizing.riskBudget) : '—'}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Tamano sugerido</div>
          <div class="provider-name" style="font-size:16px;color:${sizing && sizing.suggestedSize >= 1 ? 'var(--accent-green)' : 'var(--text-primary)'}">${sizing ? _esc(String(sizing.suggestedSize || 0)) : '—'}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Ancho</div>
          <div class="provider-name" style="font-size:16px">${sizing && sizing.widthPoints > 0 ? _fmtNum(sizing.widthPoints, 2) : '—'}</div>
        </div>
      </div>
      <div style="display:flex;gap:8px;flex-wrap:wrap;margin-top:10px">
        ${legs.map((leg) => `<span class="chip ${leg.side === 'long' ? 'green' : 'orange'}">${_esc(_labelSide(leg.side))} ${_esc(leg.option_type)} ${_esc(String(leg.strike))}</span>`).join(' ')}
        ${sizing && sizing.suggestedSize >= 1 ? `<span class="chip green">tamano ${_esc(String(sizing.suggestedSize))}</span>` : ''}
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

async function _buildOrderFromStrategy(container, { silent = false } = {}) {
  const resultEl = container.querySelector('#aq-order-result');
  const proposalEl = container.querySelector('#aq-order-proposal');
  const symbol = container.querySelector('#aq-order-symbol')?.value?.trim() || '';
  const strategyType = container.querySelector('#aq-order-strategy-type')?.value || '';
  if (!symbol) throw new Error('Indica el simbolo base');
  if (!strategyType) throw new Error('Selecciona un tipo de estrategia');
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
  if (!silent) {
    window.AtlasToast?.show('Estructura cargada desde el motor de probabilidad', 'success');
  }
  return d.data;
}

function _forcePaperScopeForScannerBridge(container) {
  const paperOnly = !!container.querySelector('#aq-op-paper-only')?.checked;
  if (!paperOnly) return;
  const monitorScopeEl = container.querySelector('#aq-account-scope');
  const opScopeEl = container.querySelector('#aq-op-scope');
  if (monitorScopeEl) monitorScopeEl.value = 'paper';
  if (opScopeEl) opScopeEl.value = 'paper';
  _syncPaperPhaseState(container);
}

function _applyScannerCandidateSeed(container, candidate, plan) {
  const meta = _strategyExecutionMeta(plan?.strategyType);
  if (!meta) throw new Error('No se pudo derivar una plantilla operativa desde el escaner');
  const setValue = (selector, value) => {
    const el = container.querySelector(selector);
    if (el && value !== undefined && value !== null) el.value = String(value);
  };
  const setChecked = (selector, value) => {
    const el = container.querySelector(selector);
    if (el) el.checked = !!value;
  };

  _forcePaperScopeForScannerBridge(container);
  setValue('#aq-order-preset', 'custom');
  setValue('#aq-order-symbol', candidate?.symbol || '');
  setValue('#aq-order-strategy-type', plan.strategyType);
  setValue('#aq-order-asset-class', meta.assetClass);
  setValue('#aq-order-side', meta.side);
  setValue('#aq-order-position-effect', meta.positionEffect);
  setValue('#aq-order-type', meta.orderType);
  setValue('#aq-order-duration', 'day');
  setValue('#aq-order-tag', plan.tag);
  setValue('#aq-order-option-symbol', '');
  setValue('#aq-order-legs', '');
  setValue('#aq-order-price', '');
  setValue('#aq-order-stop-price', '');
  setChecked('#aq-order-probability-enabled', true);

  const opMinWin = parseFloat(container.querySelector('#aq-op-min-win-rate')?.value || '65');
  const scannerWin = Number(candidate?.local_win_rate_pct || 0);
  const derivedMin = Math.max(50, Math.min(90, Math.max(opMinWin, scannerWin - 3)));
  setValue('#aq-order-min-win-rate', Number(derivedMin.toFixed(1)));

  const riskPct = ['5m'].includes(String(candidate?.timeframe || '')) ? 0.5 : ['15m', '1h'].includes(String(candidate?.timeframe || '')) ? 0.75 : 1.0;
  setValue('#aq-order-risk-pct', riskPct);
}

async function _bridgeScannerCandidate(container, candidate, action = 'load') {
  if (!candidate) throw new Error('No hay candidato del escaner para cargar');
  const selectorPayload = await _fetchSelectorProposal(container, candidate, { autoOpenCharts: false });
  const selectedType = selectorPayload?.selected?.strategy_type || selectorPayload?.selected?.meta?.label || '';

  const bridgeState = {
    action,
    symbol: candidate.symbol,
    timeframe: candidate.timeframe,
    strategy_key: candidate.strategy_key,
    strategy_type: selectedType,
    timestamp: new Date().toISOString(),
    ok: true,
  };
  if (_selectorConfig(container).open_charts_after_prepare) {
    try { _openSelectorCharts(container); } catch {}
  }

  if (action === 'evaluate' || action === 'preview') {
    await _applySelectorToTicket(container, { runBuilder: true });
    await _saveOperationConfig(container);
    const result = await _runOperationCycle(container, action === 'preview' ? 'preview' : 'evaluate');
    bridgeState.ok = !!result?.allowed;
    bridgeState.action = action === 'preview' ? 'previsualizada' : 'evaluada';
    bridgeState.decision = result?.decision || null;
    container.__atlasScannerBridgeState = bridgeState;
    _renderScannerBridgeStatus(container, container.__atlasQuantScanner);
    _activateQuantView(container, 'operacion');
    container.querySelector('#aq-operation-section')?.scrollIntoView({ behavior: 'smooth', block: 'start' });
    window.AtlasToast?.show(
      action === 'preview'
        ? 'Idea del escaner preparada por el selector y enviada a previsualizacion'
        : 'Idea del escaner preparada por el selector y evaluada por ATLAS',
      bridgeState.ok ? 'success' : 'warning',
    );
    return result;
  }

  await _applySelectorToTicket(container, { runBuilder: true });
  container.__atlasScannerBridgeState = { ...bridgeState, action: 'ticket cargado' };
  _renderScannerBridgeStatus(container, container.__atlasQuantScanner);
  _activateQuantView(container, 'ejecucion');
  container.querySelector('#aq-order-section')?.scrollIntoView({ behavior: 'smooth', block: 'start' });
  window.AtlasToast?.show('Mejor idea del escaner preparada por el selector y cargada al ticket', 'success');
  return container.__atlasQuantLastProposal || selectorPayload;
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
        <div style="font-size:12px;color:var(--text-muted)">${_esc(_translateText(response.error || ''))}</div>
      </div>
      <div style="display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px;margin-top:12px">
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Cuenta</div>
          <div class="provider-name" style="font-size:15px">${_esc(_labelScope(session.scope) || '—')}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">PDT</div>
          <div class="provider-name" style="font-size:15px;color:${pdt.blocked_opening ? 'var(--accent-red)' : 'var(--text-primary)'}">${_esc(pdt.reason || 'sin bloqueo')}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Estado del broker</div>
          <div class="provider-name" style="font-size:15px">${_esc(String(tradier.status || tradier.order_status || tradier.id || 'recibido'))}</div>
        </div>
      </div>
      ${probability ? `<div style="margin-top:12px"><span class="chip ${Number(probability.win_rate_pct || 0) >= Number(probability.min_win_rate_pct || 50) ? 'green' : 'red'}">Filtro ${_fmtPct(probability.win_rate_pct)}${probability.min_win_rate_pct !== undefined ? ` / min ${_fmtPct(probability.min_win_rate_pct)}` : ''}</span></div>` : ''}
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

function _seriesSvg(points = [], { valueKey = 'equity', stroke = 'var(--accent)' } = {}) {
  if (!Array.isArray(points) || points.length === 0) return '';
  const values = points
    .map((point) => Number(point?.[valueKey]))
    .filter((value) => Number.isFinite(value));
  if (values.length === 0) return '';
  const width = 220;
  const height = 84;
  const minY = Math.min(...values);
  const maxY = Math.max(...values);
  const rangeY = Math.max(maxY - minY, 1e-6);
  const coords = values.map((value, index) => {
    const x = values.length === 1 ? width / 2 : (index / (values.length - 1)) * width;
    const y = height - (((value - minY) / rangeY) * height);
    return `${x.toFixed(1)},${y.toFixed(1)}`;
  }).join(' ');
  const zeroInRange = minY <= 0 && maxY >= 0;
  const zeroY = height - (((0 - minY) / rangeY) * height);
  return `
    <svg viewBox="0 0 ${width} ${height}" width="100%" height="${height}" style="display:block">
      ${zeroInRange ? `<line x1="0" y1="${zeroY.toFixed(1)}" x2="${width}" y2="${zeroY.toFixed(1)}" stroke="rgba(255,255,255,0.12)" stroke-dasharray="4 4"></line>` : ''}
      <polyline fill="none" stroke="${stroke}" stroke-width="2" points="${coords}"></polyline>
    </svg>
  `;
}

function _weekdayLabel(weekday) {
  return ['Lun', 'Mar', 'Mie', 'Jue', 'Vie', 'Sab', 'Dom'][Number(weekday)] || `D${weekday}`;
}

function _heatCellStyle(cell, maxTrades) {
  const trades = Number(cell?.trades || 0);
  if (trades <= 0) {
    return 'background:rgba(255,255,255,0.03);color:var(--text-muted);';
  }
  const rate = Math.max(0, Math.min(100, Number(cell?.success_rate_pct || 0)));
  const intensity = Math.max(0.18, trades / Math.max(maxTrades, 1));
  const alpha = Math.min(0.6, 0.12 + intensity * 0.42);
  if (rate >= 50) {
    return `background:rgba(34,197,94,${alpha.toFixed(2)});color:#eafff1;`;
  }
  return `background:rgba(248,113,113,${alpha.toFixed(2)});color:#fff1f1;`;
}

function _renderJournalHeatmap(label, cells = []) {
  if (!Array.isArray(cells) || cells.length === 0) {
    return `
      <div class="provider-card" style="padding:14px">
        <div class="provider-name" style="font-size:14px">${_esc(label)}</div>
        <div class="empty-state" style="padding:18px 0 6px 0">
          <div class="empty-sub">Sin trades cerrados para construir heatmap</div>
        </div>
      </div>
    `;
  }
  const hours = [...new Set(cells.map((cell) => Number(cell?.hour)).filter((hour) => Number.isFinite(hour)))].sort((a, b) => a - b);
  const maxTrades = Math.max(...cells.map((cell) => Number(cell?.trades || 0)), 1);
  const cellMap = new Map(cells.map((cell) => [`${cell.weekday}:${cell.hour}`, cell]));
  const topWindows = [...cells]
    .sort((a, b) => ((Number(b.success_rate_pct || 0) * Math.max(Number(b.trades || 0), 1)) - (Number(a.success_rate_pct || 0) * Math.max(Number(a.trades || 0), 1))) || (Number(b.trades || 0) - Number(a.trades || 0)))
    .slice(0, 4);
  return `
    <div class="provider-card" style="padding:14px">
      <div style="display:flex;align-items:center;justify-content:space-between;gap:10px;flex-wrap:wrap">
        <div class="provider-name" style="font-size:14px">${_esc(label)}</div>
        <span class="chip blue">${_esc(String(cells.length))} bloques</span>
      </div>
      <div style="display:flex;gap:8px;flex-wrap:wrap;margin-top:10px">
        ${topWindows.map((cell) => `<span class="chip ${Number(cell.success_rate_pct || 0) >= 50 ? 'green' : 'red'}">${_esc(_weekdayLabel(cell.weekday))} ${_esc(String(cell.hour).padStart(2, '0'))}:00 · ${_fmtPct(cell.success_rate_pct)} · ${_esc(String(cell.trades))} trades</span>`).join('')}
      </div>
      <div style="overflow:auto;margin-top:12px">
        <table style="width:100%;border-collapse:separate;border-spacing:4px;min-width:${Math.max(360, hours.length * 72)}px">
          <thead>
            <tr>
              <th style="font-size:10px;color:var(--text-muted);font-weight:600;text-align:left;padding:4px 6px">Dia</th>
              ${hours.map((hour) => `<th style="font-size:10px;color:var(--text-muted);font-weight:600;text-align:center;padding:4px 6px">${_esc(String(hour).padStart(2, '0'))}</th>`).join('')}
            </tr>
          </thead>
          <tbody>
            ${[0, 1, 2, 3, 4, 5, 6].map((weekday) => `
              <tr>
                <td style="font-size:11px;color:var(--text-primary);padding:4px 6px">${_esc(_weekdayLabel(weekday))}</td>
                ${hours.map((hour) => {
                  const cell = cellMap.get(`${weekday}:${hour}`);
                  return `<td style="padding:6px;border-radius:10px;text-align:center;font-size:10px;min-width:64px;${_heatCellStyle(cell, maxTrades)}">
                    ${cell ? `<div style="font-weight:700">${_fmtPct(cell.success_rate_pct, 0)}</div><div style="opacity:0.85">${_esc(String(cell.trades))} t</div>` : '<div>—</div><div>0 t</div>'}
                  </td>`;
                }).join('')}
              </tr>
            `).join('')}
          </tbody>
        </table>
      </div>
    </div>
  `;
}

function _renderJournalAccountCard(label, data = {}, tone = 'blue') {
  const realized = Number(data?.realized_pnl || 0);
  const unrealized = Number(data?.unrealized_pnl || 0);
  const equityCurve = Array.isArray(data?.equity_curve) ? data.equity_curve : [];
  const stroke = tone === 'green' ? 'var(--accent-green)' : 'var(--accent)';
  return `
    <div class="approval-card" style="padding:14px">
      <div style="display:flex;align-items:flex-start;justify-content:space-between;gap:12px;flex-wrap:wrap">
        <div>
          <div style="display:flex;align-items:center;gap:8px;flex-wrap:wrap">
            <span class="chip ${tone}">${_esc(label)}</span>
            <span class="chip accent">cerrados ${_esc(String(data?.trades_closed ?? 0))}</span>
            <span class="chip blue">abiertos ${_esc(String(data?.trades_open ?? 0))}</span>
          </div>
          <div style="font-size:11px;color:var(--text-muted);margin-top:6px">
            Tasa de exito ${_fmtPct(data?.win_rate_pct)} · Factor de ganancia ${data?.profit_factor ?? '—'} · Sharpe ${data?.sharpe_ratio ?? '—'}
          </div>
        </div>
        <div style="text-align:right">
          <div style="font-size:18px;font-weight:700;color:${_moneyColor(realized)}">${_fmtMoney(realized)}</div>
          <div style="font-size:11px;color:var(--text-muted)">PnL realizado</div>
        </div>
      </div>
      <div style="display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;margin-top:12px">
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">No realizado</div>
          <div class="provider-name" style="font-size:16px;color:${_moneyColor(unrealized)}">${_fmtMoney(unrealized)}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Expectativa</div>
          <div class="provider-name" style="font-size:16px;color:${_moneyColor(data?.expectancy || 0)}">${_fmtMoney(data?.expectancy || 0)}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Ganancias brutas</div>
          <div class="provider-name" style="font-size:16px;color:var(--accent-green)">${_fmtMoney(data?.gross_wins || 0)}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Perdidas brutas</div>
          <div class="provider-name" style="font-size:16px;color:var(--accent-red)">${_fmtMoney(data?.gross_losses || 0)}</div>
        </div>
      </div>
      <div style="margin-top:12px;padding:10px;border:1px solid var(--border);border-radius:10px;background:rgba(255,255,255,0.02)">
        <div style="display:flex;justify-content:space-between;gap:10px;font-size:11px;color:var(--text-muted);margin-bottom:6px">
          <span>Curva de equity</span>
          <span>${equityCurve.length} puntos</span>
        </div>
        ${_seriesSvg(equityCurve, { valueKey: 'equity', stroke })}
        ${equityCurve.length === 0 ? '<div class="empty-sub" style="padding:6px 0 2px 0">Aun sin historico cerrado</div>' : ''}
      </div>
    </div>
  `;
}

function _journalLegLabel(leg = {}) {
  if (String(leg?.asset_class || '').toLowerCase() === 'equity') {
    return `${_labelSide(leg.side) || 'acción'} acción ${leg.symbol || ''}`.trim();
  }
  const optionType = String(leg?.option_type || '').toUpperCase();
  const strike = leg?.strike !== undefined && leg?.strike !== null ? String(leg.strike) : '';
  const expiry = leg?.expiration ? ` ${leg.expiration}` : '';
  return `${_labelSide(leg.side) || 'pata'} ${optionType} ${strike}${expiry}`.trim();
}

function _renderJournalEntryCard(entry = {}) {
  const legs = Array.isArray(entry?.legs) ? entry.legs : [];
  const greeks = entry?.greeks || {};
  const attribution = entry?.attribution || {};
  const postMortem = entry?.post_mortem || {};
  const pnl = entry?.status === 'closed' ? Number(entry?.realized_pnl || 0) : Number(entry?.unrealized_pnl || 0);
  const pnlLabel = entry?.status === 'closed' ? 'PnL realizado' : 'PnL abierto';
  const summaryChips = [
    `<span class="chip ${entry?.account_type === 'live' ? 'green' : 'blue'}">${_esc(_labelScope(entry?.account_type) || '—')}</span>`,
    `<span class="chip ${entry?.status === 'closed' ? 'accent' : 'orange'}">${_esc(_labelTradeStatus(entry?.status) || '—')}</span>`,
    entry?.is_level4 ? '<span class="chip red">Nivel 4</span>' : '',
    entry?.strategy_type ? `<span class="chip blue">${_esc(entry.strategy_type)}</span>` : '',
    entry?.current_win_rate_pct !== null && entry?.current_win_rate_pct !== undefined ? `<span class="chip ${Number(entry.current_win_rate_pct || 0) >= 50 ? 'green' : 'red'}">Prob. actual ${_fmtPct(entry.current_win_rate_pct)}</span>` : '',
  ].filter(Boolean).join(' ');
  return `
    <details class="approval-card" style="padding:14px;margin-bottom:10px">
      <summary style="list-style:none;cursor:pointer;display:flex;align-items:flex-start;justify-content:space-between;gap:12px;flex-wrap:wrap">
        <div>
          <div style="display:flex;align-items:center;gap:8px;flex-wrap:wrap">
            ${summaryChips}
            <span style="font-weight:700">${_esc(entry?.symbol || '—')}</span>
          </div>
          <div style="font-size:11px;color:var(--text-muted);margin-top:6px">
            ${_esc(entry?.strategy_id || '—')} · entrada ${_esc(entry?.entry_time || '—')}
            ${entry?.exit_time ? ` · salida ${_esc(entry.exit_time)}` : ''}
          </div>
        </div>
        <div style="text-align:right">
          <div style="font-size:18px;font-weight:700;color:${_moneyColor(pnl)}">${_fmtMoney(pnl)}</div>
          <div style="font-size:11px;color:var(--text-muted)">${pnlLabel}</div>
        </div>
      </summary>
      <div style="margin-top:12px">
        <div style="display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px">
          <div class="provider-card" style="padding:10px">
            <div class="provider-role">Prob. al entrar</div>
            <div class="provider-name" style="font-size:16px">${_fmtPct(entry?.win_rate_at_entry)}</div>
          </div>
          <div class="provider-card" style="padding:10px">
            <div class="provider-role">Rango IV</div>
            <div class="provider-name" style="font-size:16px">${_fmtPct(entry?.iv_rank)}</div>
          </div>
          <div class="provider-card" style="padding:10px">
            <div class="provider-role">Nominal</div>
            <div class="provider-name" style="font-size:16px">${_fmtMoney(entry?.entry_notional || 0)}</div>
          </div>
          <div class="provider-card" style="padding:10px">
            <div class="provider-role">Riesgo entrada</div>
            <div class="provider-name" style="font-size:16px">${_fmtMoney(entry?.risk_at_entry || 0)}</div>
          </div>
        </div>
        <div style="display:flex;gap:8px;flex-wrap:wrap;margin-top:12px">
          ${legs.map((leg) => `<span class="chip ${String(leg?.side || '').toLowerCase() === 'long' ? 'green' : 'orange'}">${_esc(_journalLegLabel(leg))}</span>`).join('')}
          ${legs.length === 0 ? '<span class="chip blue">Sin patas persistidas</span>' : ''}
        </div>
        <div style="display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;margin-top:12px">
          <div class="provider-card" style="padding:10px">
            <div class="provider-role">Delta</div>
            <div class="provider-name" style="font-size:15px">${_fmtNum(greeks?.delta, 2)}</div>
          </div>
          <div class="provider-card" style="padding:10px">
            <div class="provider-role">Gamma</div>
            <div class="provider-name" style="font-size:15px">${_fmtNum(greeks?.gamma, 4)}</div>
          </div>
          <div class="provider-card" style="padding:10px">
            <div class="provider-role">Theta</div>
            <div class="provider-name" style="font-size:15px">${_fmtMoney(greeks?.theta || 0)}</div>
          </div>
          <div class="provider-card" style="padding:10px">
            <div class="provider-role">Vega</div>
            <div class="provider-name" style="font-size:15px">${_fmtNum(greeks?.vega, 2)}</div>
          </div>
        </div>
        <div style="margin-top:12px;padding:12px;border:1px solid var(--border);border-radius:10px;background:rgba(255,255,255,0.02)">
          <div style="font-size:11px;color:var(--text-muted);margin-bottom:6px">Tesis</div>
          <div style="font-size:12px;line-height:1.55">${entry?.thesis_rich_text || '<span style="color:var(--text-muted)">Sin tesis persistida</span>'}</div>
        </div>
        <div style="display:flex;gap:8px;flex-wrap:wrap;margin-top:12px">
          ${attribution?.dominant_driver ? `<span class="chip accent">motor ${_esc(_translateText(attribution.dominant_driver))}</span>` : ''}
          ${postMortem?.root_cause ? `<span class="chip ${String(postMortem.root_cause).toLowerCase() === 'mixto' ? 'orange' : 'red'}">causa ${_esc(_translateText(postMortem.root_cause))}</span>` : ''}
          ${entry?.fees ? `<span class="chip blue">comisiones ${_fmtMoney(entry.fees)}</span>` : '<span class="chip blue">comisiones $0.00</span>'}
          ${entry?.spot_price ? `<span class="chip accent">spot ${_fmtNum(entry.spot_price, 2)}</span>` : ''}
        </div>
        ${entry?.post_mortem_text ? `
          <div style="margin-top:12px;padding:12px;border:1px solid rgba(255,122,89,0.25);border-radius:10px;background:rgba(255,122,89,0.07)">
            <div style="font-size:11px;color:var(--text-muted);margin-bottom:6px">Post-mortem</div>
            <div style="font-size:12px;line-height:1.55">${_esc(entry.post_mortem_text)}</div>
          </div>
        ` : ''}
      </div>
    </details>
  `;
}

function _renderJournalStats(container, payload = {}) {
  const summaryEl = container.querySelector('#aq-journal-summary');
  const accountsEl = container.querySelector('#aq-journal-accounts');
  const heatmapEl = container.querySelector('#aq-journal-heatmaps');
  const statusEl = container.querySelector('#aq-journal-status');
  if (!summaryEl || !accountsEl || !heatmapEl || !statusEl) return;
  container.__atlasQuantJournalStats = payload;
  const live = payload?.accounts?.live || {};
  const paper = payload?.accounts?.paper || {};
  const comparison = payload?.comparison || {};
  const totalClosed = Number(live?.trades_closed || 0) + Number(paper?.trades_closed || 0);
  const totalOpen = Number(live?.trades_open || 0) + Number(paper?.trades_open || 0);
  statusEl.innerHTML = `
    <span class="chip blue">Actualizado ${_esc(payload?.generated_at || '—')}</span>
    <span class="chip accent">cerrados ${_esc(String(totalClosed))}</span>
    <span class="chip orange">abiertos ${_esc(String(totalOpen))}</span>
  `;
  summaryEl.innerHTML = `
    <div class="stat-row" style="margin-bottom:12px">
      <div class="stat-card hero">
        <div class="stat-card-label">Gap PnL realizado</div>
        <div class="stat-card-value" style="font-size:16px;color:${_moneyColor(comparison?.realized_pnl_gap || 0)}">${_fmtMoney(comparison?.realized_pnl_gap || 0)}</div>
        <div class="stat-card-sub">Real menos simulada</div>
      </div>
      <div class="stat-card">
        <div class="stat-card-label">Gap tasa de exito</div>
        <div class="stat-card-value ${Number(comparison?.win_rate_gap_pct || 0) >= 0 ? 'green' : 'red'}">${_fmtPct(comparison?.win_rate_gap_pct || 0, 2)}</div>
      </div>
      <div class="stat-card">
        <div class="stat-card-label">Gap expectativa</div>
        <div class="stat-card-value ${Number(comparison?.expectancy_gap || 0) >= 0 ? 'green' : 'red'}">${_fmtMoney(comparison?.expectancy_gap || 0)}</div>
      </div>
      <div class="stat-card">
        <div class="stat-card-label">Libro de operaciones</div>
        <div class="stat-card-value">${_esc(String(totalClosed))}</div>
        <div class="stat-card-sub">${_esc(String(totalOpen))} estructuras abiertas</div>
      </div>
    </div>
  `;
  accountsEl.innerHTML = `
    <div style="display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:12px">
      ${_renderJournalAccountCard('Cuenta Real', live, 'green')}
      ${_renderJournalAccountCard('Cuenta simulada', paper, 'blue')}
    </div>
  `;
  heatmapEl.innerHTML = `
    <div style="display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:12px">
      ${_renderJournalHeatmap('Mapa de calor real', live?.heatmap || [])}
      ${_renderJournalHeatmap('Mapa de calor simulada', paper?.heatmap || [])}
    </div>
  `;
}

function _renderJournalEntries(container, payload = {}) {
  const entriesEl = container.querySelector('#aq-journal-entries');
  if (!entriesEl) return;
  const items = Array.isArray(payload?.items) ? payload.items : [];
  entriesEl.innerHTML = items.length === 0
    ? `<div class="empty-state" style="padding:16px 0"><div class="empty-sub">Sin entradas recientes en el diario</div></div>`
    : items.map((entry) => _renderJournalEntryCard(entry)).join('');
}

async function _fetchJournalStats(container) {
  const summaryEl = container.querySelector('#aq-journal-summary');
  const accountsEl = container.querySelector('#aq-journal-accounts');
  const heatmapEl = container.querySelector('#aq-journal-heatmaps');
  const statusEl = container.querySelector('#aq-journal-status');
  if (!summaryEl || !accountsEl || !heatmapEl || !statusEl) return null;
  try {
    const d = await _fetchJsonRetry(`${QUANT_API_V2}/journal/stats`, { headers: _headers() });
    if (!d?.ok || !d?.data) throw new Error(d?.error || 'No se pudo cargar el diario');
    _renderJournalStats(container, d.data);
    return d.data;
  } catch (e) {
    container.__atlasQuantJournalStats = null;
    statusEl.innerHTML = '<span class="chip red">Diario no disponible</span>';
    summaryEl.innerHTML = `<div class="empty-state" style="padding:12px 0"><div class="empty-title" style="color:var(--accent-red)">Diario no disponible</div><div class="empty-sub">${_esc(e.message || 'Error')}</div></div>`;
    accountsEl.innerHTML = '';
    heatmapEl.innerHTML = '';
    return null;
  }
}

async function _fetchJournalEntries(container, { limit = 18 } = {}) {
  const entriesEl = container.querySelector('#aq-journal-entries');
  if (!entriesEl) return null;
  try {
    const params = new URLSearchParams();
    params.set('limit', String(limit));
    const d = await _fetchJsonRetry(`${QUANT_API_V2}/journal/entries?${params.toString()}`, { headers: _headers() });
    if (!d?.ok || !d?.data) throw new Error(d?.error || 'No se pudo cargar el detalle del diario');
    container.__atlasQuantJournalEntries = d.data;
    _renderJournalEntries(container, d.data);
    return d.data;
  } catch (e) {
    container.__atlasQuantJournalEntries = null;
    if (entriesEl) {
      entriesEl.innerHTML = `<div class="empty-state" style="padding:12px 0"><div class="empty-title" style="color:var(--accent-red)">Detalle no disponible</div><div class="empty-sub">${_esc(e.message || 'Error')}</div></div>`;
    }
    return null;
  }
}

async function _refreshJournalPanel(container) {
  const [stats, entries] = await Promise.allSettled([
    _fetchJournalStats(container),
    _fetchJournalEntries(container),
  ]);
  return {
    stats: stats.status === 'fulfilled' ? stats.value : null,
    entries: entries.status === 'fulfilled' ? entries.value : null,
  };
}

async function _syncJournal(container) {
  const statusEl = container.querySelector('#aq-journal-status');
  if (statusEl) statusEl.innerHTML = '<span class="chip orange">Sincronizando diario...</span>';
  const r = await fetch(`${QUANT_API_V2}/journal/sync/refresh`, {
    method: 'POST',
    headers: _headers(),
  });
  const d = await r.json();
  if (!d?.ok) throw new Error(d?.error || 'No se pudo sincronizar el diario');
  const results = Array.isArray(d?.data?.results) ? d.data.results : [];
  await _refreshJournalPanel(container);
  if (statusEl && results.length > 0) {
    statusEl.innerHTML += results.map((item) => {
      if (item?.error) return `<span class="chip red">${_esc(item.scope || 'scope')} error</span>`;
      return `<span class="chip green">${_esc(item.scope || 'scope')} c${_esc(String(item.created ?? 0))} u${_esc(String(item.updated ?? 0))} x${_esc(String(item.closed ?? 0))}</span>`;
    }).join(' ');
  }
  return d.data;
}

function _syncOperationForm(container, config = {}) {
  if (!container || !config) return;
  const setValue = (selector, value) => {
    const el = container.querySelector(selector);
    if (el && value !== undefined && value !== null) el.value = String(value);
  };
  const setChecked = (selector, value) => {
    const el = container.querySelector(selector);
    if (el) el.checked = !!value;
  };
  setValue('#aq-op-scope', config.account_scope || 'paper');
  setValue('#aq-op-auton-mode', config.auton_mode || 'paper_supervised');
  setValue('#aq-op-executor-mode', config.executor_mode || 'paper_api');
  setValue('#aq-op-vision-mode', config.vision_mode || 'direct_nexus');
  setValue('#aq-op-min-win-rate', config.min_auton_win_rate_pct ?? 65);
  setValue('#aq-op-max-bpr', config.max_level4_bpr_pct ?? 20);
  setValue('#aq-op-failsafe-limit', config.operational_error_limit ?? 3);
  setValue('#aq-op-sentiment-score', config.sentiment_score ?? 0);
  setValue('#aq-op-notes', _translateText(config.notes || ''));
  setChecked('#aq-op-paper-only', config.paper_only !== false);
  setChecked('#aq-op-require-operator', !!config.require_operator_present);
  setChecked('#aq-op-operator-present', config.operator_present !== false);
  setChecked('#aq-op-screen-ok', config.screen_integrity_ok !== false);
  setChecked('#aq-op-auto-pause-errors', config.auto_pause_on_operational_errors !== false);
  container.__atlasOperationHydrated = true;
  _syncPaperPhaseState(container);
  _highlightOperationProfile(container);
}

function _syncPaperPhaseState(container) {
  if (!container) return;
  const paperOnly = !!container.querySelector('#aq-op-paper-only')?.checked;
  const opScopeEl = container.querySelector('#aq-op-scope');
  const monitorScopeEl = container.querySelector('#aq-account-scope');
  const opPreviewBtn = container.querySelector('#aq-btn-op-preview');
  const opSubmitBtn = container.querySelector('#aq-btn-op-submit');
  const orderPreviewBtn = container.querySelector('#aq-btn-order-preview');
  const orderSubmitBtn = container.querySelector('#aq-btn-order-submit');
  const opNoteEl = container.querySelector('#aq-op-phase-note');
  const orderNoteEl = container.querySelector('#aq-order-phase-note');

  const opLiveOption = opScopeEl?.querySelector('option[value="live"]');
  if (opLiveOption) opLiveOption.disabled = paperOnly;
  if (paperOnly && opScopeEl && opScopeEl.value !== 'paper') {
    opScopeEl.value = 'paper';
  }

  const opScope = opScopeEl?.value || 'paper';
  const monitorScope = monitorScopeEl?.value || 'paper';
  const opLiveBlocked = paperOnly && opScope !== 'paper';
  const orderLiveBlocked = paperOnly && monitorScope === 'live';

  if (opPreviewBtn) {
    opPreviewBtn.textContent = `Previsualizar ${_labelScope(opScope)}`;
  }
  if (opSubmitBtn) {
    opSubmitBtn.disabled = opLiveBlocked;
    opSubmitBtn.textContent = opScope === 'paper' ? 'Ejecutar en simulada' : 'Ejecutar en real';
    opSubmitBtn.title = opLiveBlocked ? 'La cuenta real esta bloqueada mientras solo simulada este activa.' : '';
    opSubmitBtn.style.opacity = opLiveBlocked ? '0.6' : '1';
    opSubmitBtn.style.cursor = opLiveBlocked ? 'not-allowed' : 'pointer';
  }
  if (orderPreviewBtn) {
    orderPreviewBtn.textContent = `Previsualizar ${_labelScope(monitorScope)}`;
  }
  if (orderSubmitBtn) {
    orderSubmitBtn.disabled = orderLiveBlocked;
    orderSubmitBtn.textContent = monitorScope === 'paper' ? 'Enviar a simulada' : 'Enviar a real';
    orderSubmitBtn.title = orderLiveBlocked ? 'La cuenta real esta bloqueada visualmente durante la fase 1 simulada.' : '';
    orderSubmitBtn.style.opacity = orderLiveBlocked ? '0.6' : '1';
    orderSubmitBtn.style.cursor = orderLiveBlocked ? 'not-allowed' : 'pointer';
  }
  if (opNoteEl) {
    opNoteEl.innerHTML = paperOnly
      ? `
            <span class="chip green">Fase 1 simulada activa</span>
        <span class="chip blue">Plano de control fijado al entorno sandbox</span>
        <span class="chip orange">La cuenta real queda fuera hasta la siguiente fase</span>
      `
      : `
        <span class="chip orange">Modo solo simulada desactivado</span>
        <span class="chip red">Revisa gobernanza antes de habilitar live</span>
      `;
  }
  if (orderNoteEl) {
    orderNoteEl.innerHTML = paperOnly
      ? (monitorScope === 'paper'
          ? `
            <span class="chip green">Cuenta activa: sandbox simulado</span>
            <span class="chip blue">Envio listo solo para simulada</span>
          `
          : `
            <span class="chip orange">Cuenta activa: real</span>
            <span class="chip red">El envio real esta bloqueado por la fase 1 simulada</span>
            <span class="chip blue">Puedes usar previsualizacion o volver a simulada</span>
          `)
      : `
        <span class="chip accent">Cuenta activa: ${_esc(_labelScope(monitorScope))}</span>
      `;
  }
}

function _highlightOperationProfile(container) {
  if (!container) return;
  const current = _buildOperationConfig(container);
  let activeProfile = null;
  for (const [profileId, profile] of Object.entries(OPERATION_PROFILES)) {
    const matches =
      String(current.account_scope) === String(profile.account_scope) &&
      !!current.paper_only === !!profile.paper_only &&
      String(current.auton_mode) === String(profile.auton_mode) &&
      String(current.executor_mode) === String(profile.executor_mode) &&
      String(current.vision_mode) === String(profile.vision_mode) &&
      !!current.require_operator_present === !!profile.require_operator_present &&
      !!current.auto_pause_on_operational_errors === !!profile.auto_pause_on_operational_errors &&
      Number(current.operational_error_limit || 0) === Number(profile.operational_error_limit || 0) &&
      Math.abs(Number(current.min_auton_win_rate_pct || 0) - Number(profile.min_auton_win_rate_pct || 0)) < 0.001 &&
      Math.abs(Number(current.max_level4_bpr_pct || 0) - Number(profile.max_level4_bpr_pct || 0)) < 0.001;
    if (matches) {
      activeProfile = profileId;
      break;
    }
  }
  container.querySelectorAll('[data-op-profile]').forEach((el) => {
    el.classList.toggle('primary', !!activeProfile && el.getAttribute('data-op-profile') === activeProfile);
  });
}

function _applyOperationProfile(container, profileId) {
  const profile = OPERATION_PROFILES[profileId];
  if (!container || !profile) return false;
  const setValue = (selector, value) => {
    const el = container.querySelector(selector);
    if (el && value !== undefined && value !== null) el.value = String(value);
  };
  const setChecked = (selector, value) => {
    const el = container.querySelector(selector);
    if (el) el.checked = !!value;
  };
  setValue('#aq-op-scope', profile.account_scope);
  setValue('#aq-op-auton-mode', profile.auton_mode);
  setValue('#aq-op-executor-mode', profile.executor_mode);
  setValue('#aq-op-vision-mode', profile.vision_mode);
  setValue('#aq-op-min-win-rate', profile.min_auton_win_rate_pct);
  setValue('#aq-op-max-bpr', profile.max_level4_bpr_pct);
  setValue('#aq-op-failsafe-limit', profile.operational_error_limit);
  setValue('#aq-op-sentiment-score', profile.sentiment_score);
  setValue('#aq-op-notes', _translateText(profile.notes));
  setChecked('#aq-op-paper-only', profile.paper_only);
  setChecked('#aq-op-require-operator', profile.require_operator_present);
  setChecked('#aq-op-operator-present', profile.operator_present);
  setChecked('#aq-op-screen-ok', profile.screen_integrity_ok);
  setChecked('#aq-op-auto-pause-errors', profile.auto_pause_on_operational_errors);

  const monitorScopeEl = container.querySelector('#aq-account-scope');
  if (monitorScopeEl) monitorScopeEl.value = String(profile.account_scope || 'paper');

  _syncPaperPhaseState(container);
  _highlightOperationProfile(container);
  return true;
}

function _buildOperationConfig(container) {
  return {
    account_scope: container.querySelector('#aq-op-scope')?.value || 'paper',
    paper_only: !!container.querySelector('#aq-op-paper-only')?.checked,
    auton_mode: container.querySelector('#aq-op-auton-mode')?.value || 'paper_supervised',
    executor_mode: container.querySelector('#aq-op-executor-mode')?.value || 'paper_api',
    vision_mode: container.querySelector('#aq-op-vision-mode')?.value || 'direct_nexus',
    require_operator_present: !!container.querySelector('#aq-op-require-operator')?.checked,
    operator_present: !!container.querySelector('#aq-op-operator-present')?.checked,
    screen_integrity_ok: !!container.querySelector('#aq-op-screen-ok')?.checked,
    auto_pause_on_operational_errors: !!container.querySelector('#aq-op-auto-pause-errors')?.checked,
    operational_error_limit: parseInt(container.querySelector('#aq-op-failsafe-limit')?.value || '3', 10) || 3,
    sentiment_score: parseFloat(container.querySelector('#aq-op-sentiment-score')?.value || '0'),
    sentiment_source: 'manual',
    min_auton_win_rate_pct: parseFloat(container.querySelector('#aq-op-min-win-rate')?.value || '65'),
    max_level4_bpr_pct: parseFloat(container.querySelector('#aq-op-max-bpr')?.value || '20'),
    notes: container.querySelector('#aq-op-notes')?.value?.trim() || '',
    kill_switch_active: !!container.__atlasOperationStatus?.executor?.kill_switch_active,
  };
}

function _buildOperationRequest(container, action = 'evaluate') {
  const order = _buildOrderPayload(container, { preview: action !== 'submit' });
  order.account_scope = container.querySelector('#aq-op-scope')?.value || order.account_scope || 'paper';
  return {
    order,
    action,
    capture_context: true,
  };
}

function _renderOperationResult(el, payload, errorText = '') {
  if (!el) return;
  if (!payload) {
    el.innerHTML = `<div style="color:var(--accent-red);font-size:12px;padding:8px">${_esc(errorText || 'Sin respuesta operacional')}</div>`;
    return;
  }
  const gates = payload.gates || {};
  const reasons = Array.isArray(payload.reasons) ? payload.reasons : [];
  const warnings = Array.isArray(payload.warnings) ? payload.warnings : [];
  const execution = payload.execution || null;
  const snapshot = payload.vision_snapshot || null;
  const whatIf = payload.what_if || {};
  el.innerHTML = `
    <div class="approval-card" style="padding:14px">
      <div style="display:flex;align-items:center;justify-content:space-between;gap:12px;flex-wrap:wrap">
        <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center">
          <span class="chip ${payload.allowed ? 'green' : 'red'}">${_esc(_translateText(payload.decision || '—'))}</span>
          <span class="chip blue">${_esc(_translateText(payload.action || 'evaluate'))}</span>
          <span class="chip accent">${_esc(_labelScope(String(gates.scope || 'paper')))}</span>
          ${gates.win_rate_pct !== undefined && gates.win_rate_pct !== null ? `<span class="chip ${Number(gates.win_rate_pct || 0) >= Number(gates.min_win_rate_pct || 65) ? 'green' : 'red'}">Prob. exito ${_fmtPct(gates.win_rate_pct)}</span>` : ''}
          ${gates.level4_bpr_pct !== undefined && gates.level4_bpr_pct !== null ? `<span class="chip ${Number(gates.level4_bpr_pct || 0) <= Number(gates.max_level4_bpr_pct || 20) ? 'green' : 'red'}">BPR ${_fmtPct(gates.level4_bpr_pct, 2)}</span>` : ''}
        </div>
        <div style="font-size:12px;color:var(--text-muted)">${_esc(snapshot?.captured_at || payload.generated_at || '')}</div>
      </div>
      ${reasons.length ? `<div style="margin-top:10px;display:flex;gap:8px;flex-wrap:wrap">${reasons.map((reason) => `<span class="chip red">${_esc(_translateText(reason))}</span>`).join('')}</div>` : ''}
      ${warnings.length ? `<div style="margin-top:10px;display:flex;gap:8px;flex-wrap:wrap">${warnings.map((warning) => `<span class="chip orange">${_esc(_translateText(warning))}</span>`).join('')}</div>` : ''}
      <div style="display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;margin-top:12px">
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Min. exito</div>
          <div class="provider-name" style="font-size:15px">${_fmtPct(gates.min_win_rate_pct)}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Riesgo</div>
          <div class="provider-name" style="font-size:15px">${_fmtMoney(gates.risk_dollars || 0)}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Max BPR</div>
          <div class="provider-name" style="font-size:15px">${_fmtPct(gates.max_level4_bpr_pct, 2)}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Sentimiento</div>
          <div class="provider-name" style="font-size:15px">${_fmtNum(payload?.sentiment?.score, 2)}</div>
        </div>
      </div>
      <div class="approval-card" style="padding:12px;margin-top:12px;background:rgba(255,255,255,0.02)">
        <div class="section-title" style="font-size:14px;margin:0 0 10px">What-if operativo</div>
        <div style="display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px">
          <div class="provider-card" style="padding:10px">
            <div class="provider-role">BP actual</div>
            <div class="provider-name" style="font-size:15px">${_fmtMoney(whatIf.current_buying_power || 0)}</div>
          </div>
          <div class="provider-card" style="padding:10px">
            <div class="provider-role">BP despues</div>
            <div class="provider-name" style="font-size:15px">${_fmtMoney(whatIf.projected_buying_power || 0)}</div>
          </div>
          <div class="provider-card" style="padding:10px">
            <div class="provider-role">Uso de riesgo</div>
            <div class="provider-name" style="font-size:15px">${whatIf.projected_risk_usage_pct !== undefined && whatIf.projected_risk_usage_pct !== null ? _fmtPct(whatIf.projected_risk_usage_pct, 2) : '—'}</div>
          </div>
          <div class="provider-card" style="padding:10px">
            <div class="provider-role">Carril</div>
            <div class="provider-name" style="font-size:15px">${_esc(_translateText(whatIf.approval_lane || '—'))}</div>
          </div>
        </div>
      </div>
      ${execution ? `
        <div style="margin-top:12px;font-size:11px;color:var(--text-muted)">Resultado del ejecutor</div>
        <pre style="margin-top:6px;white-space:pre-wrap;word-break:break-word;background:rgba(255,255,255,0.03);border:1px solid var(--border);border-radius:10px;padding:10px;font-size:11px;max-height:220px;overflow:auto">${_prettyJson(execution)}</pre>
      ` : ''}
    </div>
  `;
}

function _renderOperationStatus(container, payload = {}, { syncForm = false } = {}) {
  const summaryEl = container.querySelector('#aq-op-summary');
  const statusEl = container.querySelector('#aq-op-state');
  const listEl = container.querySelector('#aq-op-strategies');
  if (!summaryEl || !statusEl || !listEl) return;
  container.__atlasOperationStatus = payload;
  const config = payload.config || {};
  const vision = payload.vision || {};
  const executor = payload.executor || {};
  const brain = payload.brain || {};
  const learning = payload.learning || {};
  const failsafe = payload.failsafe || {};
  const sentiment = payload.ai_sentiment || {};
  const positions = Array.isArray(payload.win_rate_positions) ? payload.win_rate_positions : [];
  const lastDecision = payload.last_decision || {};
  const balances = payload?.monitor_summary?.balances || {};
  if (syncForm || !container.__atlasOperationHydrated) {
    _syncOperationForm(container, config);
  }
  _syncPaperPhaseState(container);
  statusEl.innerHTML = `
    <span class="chip ${payload.auton_mode_active ? 'green' : 'orange'}">${payload.auton_mode_active ? 'Auton activo' : 'Auton en espera'}</span>
    <span class="chip ${executor.kill_switch_active ? 'red' : 'blue'}">${executor.kill_switch_active ? 'Parada total ON' : 'Parada total OFF'}</span>
    <span class="chip accent">${_esc(_labelScope(config.account_scope || 'paper'))}</span>
    <span class="chip ${vision.operator_present ? 'green' : 'orange'}">Operador ${vision.operator_present ? 'presente' : 'ausente'}</span>
    <span class="chip ${vision.screen_integrity_ok ? 'green' : 'red'}">Pantallas ${vision.screen_integrity_ok ? 'OK' : 'alerta'}</span>
    <span class="chip ${brain.last_delivery_ok ? 'green' : 'orange'}">${brain.last_delivery_ok ? 'Cerebro ATLAS enlazado' : 'Cerebro en cola local'}</span>
    <span class="chip ${learning.enabled ? 'blue' : 'orange'}">${learning.enabled ? `Aprendizaje activo (${_esc(String(learning.sample_count || 0))})` : 'Aprendizaje en espera'}</span>
    <span class="chip ${failsafe.active ? 'red' : 'blue'}">${failsafe.active ? 'Failsafe activo' : 'Failsafe listo'}</span>
  `;
  summaryEl.innerHTML = `
    <div class="stat-row" style="margin-bottom:12px">
      <div class="stat-card hero">
        <div class="stat-card-label">Modo operacional</div>
        <div class="stat-card-value" style="font-size:16px">${_esc(_labelOperationMode(config.auton_mode || 'off'))}</div>
        <div class="stat-card-sub">${_esc(_labelExecutorMode(config.executor_mode || executor.mode || 'disabled'))}</div>
      </div>
      <div class="stat-card">
        <div class="stat-card-label">Sentimiento</div>
        <div class="stat-card-value ${Number(sentiment.score || 0) >= 0 ? 'green' : 'red'}">${_fmtNum(sentiment.score, 2)}</div>
      </div>
      <div class="stat-card">
        <div class="stat-card-label">BP de opciones</div>
        <div class="stat-card-value">${_fmtMoney(balances.option_buying_power || 0)}</div>
      </div>
      <div class="stat-card">
        <div class="stat-card-label">Ultima decision</div>
        <div class="stat-card-value" style="font-size:13px">${_esc(_translateText(lastDecision.decision || '—'))}</div>
        <div class="stat-card-sub">${_esc(_translateText(lastDecision.reason || ''))}</div>
      </div>
    </div>
    <div style="display:flex;gap:8px;flex-wrap:wrap">
      <span class="chip blue">Filtro de exito ${_fmtPct(config.min_auton_win_rate_pct)}</span>
      <span class="chip orange">Max BPR ${_fmtPct(config.max_level4_bpr_pct, 2)}</span>
      <span class="chip ${config.paper_only !== false ? 'green' : 'red'}">${config.paper_only !== false ? 'Solo simulada' : 'Real permitido'}</span>
      <span class="chip accent">Vision ${_esc(_labelVisionMode(vision.provider || config.vision_mode || 'manual'))}</span>
      ${vision.last_capture_at ? `<span class="chip blue">Ultima captura ${_esc(vision.last_capture_at)}</span>` : ''}
      ${learning.risk_multiplier !== undefined ? `<span class="chip blue">Riesgo adaptativo ${_fmtNum(learning.risk_multiplier, 2)}x</span>` : ''}
      <span class="chip ${failsafe.active ? 'red' : 'orange'}">errores ${_esc(String(failsafe.operational_error_count || 0))}/${_esc(String(failsafe.operational_error_limit || 3))}</span>
    </div>
  `;
  summaryEl.innerHTML += `
    <div style="display:flex;gap:8px;flex-wrap:wrap;margin-top:10px">
      ${brain.total_events !== undefined ? `<span class="chip blue">Eventos cerebro ${_esc(String(brain.total_events || 0))}</span>` : ''}
      ${brain.queued_local_only_events ? `<span class="chip orange">En cola ${_esc(String(brain.queued_local_only_events || 0))}</span>` : ''}
      <span class="chip ${brain.last_delivery_ok ? 'green' : 'orange'}">${brain.last_delivery_ok ? 'Cerebro ATLAS conectado' : 'Cerebro local / pendiente'}</span>
      ${learning.top_positive?.[0] ? `<span class="chip green">Edge ${_esc(learning.top_positive[0].key || '—')}</span>` : ''}
      ${learning.top_negative?.[0] ? `<span class="chip orange">Enfriado ${_esc(learning.top_negative[0].key || '—')}</span>` : ''}
    </div>
    <div class="approval-card" style="padding:12px;margin-top:12px;background:rgba(255,255,255,0.02)">
      <div class="section-title" style="font-size:14px;margin:0 0 10px">Failsafe operativo</div>
      <div style="display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px">
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Estado</div>
          <div class="provider-name" style="font-size:13px">${failsafe.active ? 'pausado' : 'normal'}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Conteo</div>
          <div class="provider-name" style="font-size:13px">${_esc(String(failsafe.operational_error_count || 0))}/${_esc(String(failsafe.operational_error_limit || 3))}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Auto pausa</div>
          <div class="provider-name" style="font-size:13px">${failsafe.auto_pause_on_operational_errors ? 'si' : 'no'}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Ultimo error</div>
          <div class="provider-name" style="font-size:13px">${_esc(_translateText(failsafe.last_operational_error?.reason || 'sin error'))}</div>
        </div>
      </div>
      ${failsafe.reason ? `<div style="margin-top:10px;font-size:11px;color:var(--accent-orange)">Motivo actual: ${_esc(_translateText(failsafe.reason))}</div>` : ''}
    </div>
    <div class="approval-card" style="padding:12px;margin-top:12px;background:rgba(255,255,255,0.02)">
      <div class="section-title" style="font-size:14px;margin:0 0 10px">Enlace con cerebro ATLAS</div>
      <div style="display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px">
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Estado</div>
          <div class="provider-name" style="font-size:13px">${brain.last_delivery_ok ? 'conectado' : 'degradado'}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Ultimo evento</div>
          <div class="provider-name" style="font-size:13px">${_esc(brain.last_event_kind || 'sin eventos')}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Destino</div>
          <div class="provider-name" style="font-size:13px">${_esc(brain.atlas_base_url || 'no configurado')}</div>
        </div>
      </div>
      ${brain.last_error ? `<div style="margin-top:10px;font-size:11px;color:var(--accent-orange)">Ultimo error de enlace: ${_esc(_translateText(brain.last_error))}</div>` : ''}
    </div>
    <div class="approval-card" style="padding:12px;margin-top:12px;background:rgba(255,255,255,0.02)">
      <div class="section-title" style="font-size:14px;margin:0 0 10px">Aprendizaje adaptativo</div>
      <div style="display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px">
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Muestras</div>
          <div class="provider-name" style="font-size:13px">${_esc(String(learning.sample_count || 0))}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Ventana</div>
          <div class="provider-name" style="font-size:13px">${_esc(String(learning.window_days || '—'))} d</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Profit factor</div>
          <div class="provider-name" style="font-size:13px">${_fmtNum(learning.profit_factor, 2)}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Expectancia</div>
          <div class="provider-name" style="font-size:13px">${_fmtPct(learning.expectancy_pct, 2)}</div>
        </div>
      </div>
      <div style="display:flex;gap:8px;flex-wrap:wrap;margin-top:10px">
        ${Array.isArray(learning.top_positive) ? learning.top_positive.map((item) => `<span class="chip green">favorece ${_esc(item.key || '—')} (${_fmtNum(item.bias_score, 2)})</span>`).join(' ') : ''}
        ${Array.isArray(learning.top_negative) ? learning.top_negative.map((item) => `<span class="chip orange">enfria ${_esc(item.key || '—')} (${_fmtNum(item.bias_score, 2)})</span>`).join(' ') : ''}
      </div>
    </div>
  `;
  listEl.innerHTML = positions.length === 0
    ? `<div class="empty-state" style="padding:16px 0"><div class="empty-sub">Sin estrategias activas para telemetria operacional</div></div>`
    : positions.map((item) => `
      <div class="approval-card" style="padding:12px;margin-bottom:8px">
        <div style="display:flex;align-items:center;justify-content:space-between;gap:12px;flex-wrap:wrap">
          <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center">
            <span class="chip blue">${_esc(item.strategy_type || '—')}</span>
            <span style="font-weight:700">${_esc(item.underlying || '—')}</span>
            ${item.alert ? '<span class="chip red">alerta</span>' : ''}
          </div>
          <div style="display:flex;gap:8px;flex-wrap:wrap">
            <span class="chip ${Number(item.win_rate_pct || 0) >= 50 ? 'green' : 'red'}">Prob. exito ${_fmtPct(item.win_rate_pct)}</span>
            <span class="chip ${Number(item.open_pnl || 0) >= 0 ? 'green' : 'red'}">PnL ${_fmtMoney(item.open_pnl || 0)}</span>
            <span class="chip accent">BPR ${_fmtMoney(item.bpr || 0)}</span>
          </div>
        </div>
      </div>
    `).join('');
}

async function _fetchOperationStatus(container) {
  const summaryEl = container.querySelector('#aq-op-summary');
  const statusEl = container.querySelector('#aq-op-state');
  const listEl = container.querySelector('#aq-op-strategies');
  if (!summaryEl || !statusEl || !listEl) return null;
  try {
    const d = await _fetchJsonRetry(`${QUANT_API_V2}/operation/status`, { headers: _headers() });
    if (!d?.ok || !d?.data) throw new Error(d?.error || 'No se pudo cargar estado operacional');
    _renderOperationStatus(container, d.data);
    return d.data;
  } catch (e) {
    statusEl.innerHTML = '<span class="chip red">Operacion no disponible</span>';
    summaryEl.innerHTML = `<div class="empty-state" style="padding:12px 0"><div class="empty-title" style="color:var(--accent-red)">Operacion no disponible</div><div class="empty-sub">${_esc(e.message || 'Error')}</div></div>`;
    listEl.innerHTML = '';
    return null;
  }
}

async function _saveOperationConfig(container) {
  const payload = _buildOperationConfig(container);
  const r = await fetch(`${QUANT_API_V2}/operation/config`, {
    method: 'POST',
    headers: _headers(),
    body: JSON.stringify(payload),
  });
  const d = await r.json();
  if (!d?.ok || !d?.data) throw new Error(d?.error || 'No se pudo guardar configuracion operacional');
  _renderOperationStatus(container, d.data, { syncForm: true });
  return d.data;
}

async function _runOperationCycle(container, action = 'evaluate') {
  const resultEl = container.querySelector('#aq-op-result');
  if (resultEl) {
    resultEl.innerHTML = '<div style="text-align:center;padding:12px"><div class="spinner" style="margin:0 auto;display:block"></div><div style="color:var(--text-muted);font-size:11px;margin-top:6px">ATLAS operacional procesando...</div></div>';
  }
  const payload = _buildOperationRequest(container, action);
  const r = await fetch(`${QUANT_API_V2}/operation/test-cycle`, {
    method: 'POST',
    headers: _headers(),
    body: JSON.stringify(payload),
  });
  const d = await r.json();
  if (!d?.ok || !d?.data) throw new Error(d?.error || 'No se pudo ejecutar el ciclo operacional');
  _renderOperationResult(resultEl, d.data);
  if (d.data?.operation_status) {
    _renderOperationStatus(container, d.data.operation_status, { syncForm: false });
  } else {
    await _fetchOperationStatus(container);
  }
  return d.data;
}

async function _sendEmergency(container, reset = false) {
  const url = reset ? `${QUANT_API_V2}/emergency/reset` : `${QUANT_API_V2}/emergency/stop`;
  const options = reset
    ? { method: 'POST', headers: _headers() }
    : { method: 'POST', headers: _headers(), body: JSON.stringify({ reason: 'dashboard_manual_stop' }) };
  const r = await fetch(url, options);
  const d = await r.json();
  if (!d?.ok || !d?.data) throw new Error(d?.error || 'No se pudo actualizar el kill switch');
  _renderOperationStatus(container, d.data, { syncForm: true });
  return d.data;
}

function _populateVisionCalibrationPointSelect(container, payload = {}) {
  const selectEl = container.querySelector('#aq-calib-point-id');
  if (!selectEl) return;
  const points = Array.isArray(payload.points) ? payload.points : [];
  const selected = selectEl.value;
  selectEl.innerHTML = ['<option value="">siguiente punto sugerido</option>'].concat(points.map((point) => {
    const pid = _esc(String(point.point_id || ''));
    const label = `${pid} · (${Number(point.x || 0).toFixed(0)}, ${Number(point.y || 0).toFixed(0)})`;
    return `<option value="${pid}">${_esc(label)}</option>`;
  })).join('');
  if (selected && points.some((point) => String(point.point_id) === selected)) {
    selectEl.value = selected;
  }
}

function _renderVisionCalibration(container, payload = {}) {
  const stateEl = container.querySelector('#aq-calib-state');
  const pointsEl = container.querySelector('#aq-calib-points');
  if (!stateEl || !pointsEl) return;
  const points = Array.isArray(payload.points) ? payload.points : [];
  const samples = Array.isArray(payload.samples) ? payload.samples : [];
  const sampledIds = new Set(samples.map((item) => String(item.point_id || '')));
  _populateVisionCalibrationPointSelect(container, payload);

  stateEl.innerHTML = `
    <span class="chip ${payload.session_active ? 'green' : 'orange'}">${payload.session_active ? 'Sesion activa' : 'Sesion inactiva'}</span>
    <span class="chip blue">Muestras ${_esc(String(payload.sample_count || 0))}</span>
    <span class="chip ${payload.calibration_exists ? 'green' : 'orange'}">${payload.calibration_exists ? 'Calibracion lista' : 'Sin calibracion guardada'}</span>
    ${payload.active_calibration_path ? `<span class="chip accent">${_esc(payload.active_calibration_path)}</span>` : ''}
    ${payload.last_fit?.saved_at ? `<span class="chip blue">Ultimo ajuste ${_esc(payload.last_fit.saved_at)}</span>` : ''}
  `;

  pointsEl.innerHTML = points.length === 0
    ? '<div class="empty-state" style="padding:10px 0"><div class="empty-sub">Inicia una grilla para calibrar la camara.</div></div>'
    : `
      <div style="display:flex;gap:8px;flex-wrap:wrap">
        ${points.map((point) => {
          const sampled = sampledIds.has(String(point.point_id || ''));
          return `<span class="chip ${sampled ? 'green' : 'blue'}">${_esc(String(point.point_id || ''))}</span>`;
        }).join('')}
      </div>
      <div style="margin-top:8px;font-size:11px;color:var(--text-muted)">
        ${payload.notes ? _esc(String(payload.notes)) : 'Usa mover cuello + guardar muestra hasta cubrir la rejilla.'}
      </div>
    `;

  const widthEl = container.querySelector('#aq-calib-width');
  const heightEl = container.querySelector('#aq-calib-height');
  const rowsEl = container.querySelector('#aq-calib-rows');
  const colsEl = container.querySelector('#aq-calib-cols');
  const zoomGridEl = container.querySelector('#aq-calib-grid-zoom');
  if (widthEl && payload.monitor?.width) widthEl.value = String(payload.monitor.width);
  if (heightEl && payload.monitor?.height) heightEl.value = String(payload.monitor.height);
  if (rowsEl && payload.monitor?.rows) rowsEl.value = String(payload.monitor.rows);
  if (colsEl && payload.monitor?.cols) colsEl.value = String(payload.monitor.cols);
  if (zoomGridEl && payload.monitor?.zoom_center) zoomGridEl.value = String(payload.monitor.zoom_center);

  const yawEl = container.querySelector('#aq-calib-yaw');
  const pitchEl = container.querySelector('#aq-calib-pitch');
  const zoomEl = container.querySelector('#aq-calib-zoom');
  if (yawEl && payload.last_pose?.yaw !== undefined) yawEl.value = String(Number(payload.last_pose.yaw).toFixed(4));
  if (pitchEl && payload.last_pose?.pitch !== undefined) pitchEl.value = String(Number(payload.last_pose.pitch).toFixed(4));
  if (zoomEl && payload.last_pose?.zoom !== undefined) zoomEl.value = String(Number(payload.last_pose.zoom).toFixed(2));
}

async function _fetchVisionCalibrationStatus(container) {
  const d = await _fetchJsonRetry(`${QUANT_API_V2}/vision/calibration/status`, { headers: _headers() });
  if (!d?.ok || !d?.data) throw new Error(d?.error || 'No se pudo cargar la calibracion de camara');
  _renderVisionCalibration(container, d.data);
  return d.data;
}

async function _startVisionCalibration(container) {
  const payload = {
    width: parseInt(container.querySelector('#aq-calib-width')?.value || '1920', 10),
    height: parseInt(container.querySelector('#aq-calib-height')?.value || '1080', 10),
    rows: parseInt(container.querySelector('#aq-calib-rows')?.value || '3', 10),
    cols: parseInt(container.querySelector('#aq-calib-cols')?.value || '3', 10),
    zoom_center: parseFloat(container.querySelector('#aq-calib-grid-zoom')?.value || '1'),
    label: container.querySelector('#aq-calib-label')?.value?.trim() || 'calibracion_principal',
    save_path: container.querySelector('#aq-calib-save-path')?.value?.trim() || null,
  };
  const r = await fetch(`${QUANT_API_V2}/vision/calibration/start`, {
    method: 'POST',
    headers: _headers(),
    body: JSON.stringify(payload),
  });
  const d = await r.json();
  if (!d?.ok || !d?.data) throw new Error(d?.error || 'No se pudo iniciar la calibracion');
  _renderVisionCalibration(container, d.data);
  return d.data;
}

async function _moveVisionCalibrationPose(container) {
  const payload = {
    yaw: parseFloat(container.querySelector('#aq-calib-yaw')?.value || '0'),
    pitch: parseFloat(container.querySelector('#aq-calib-pitch')?.value || '0'),
    zoom: parseFloat(container.querySelector('#aq-calib-zoom')?.value || '1'),
    source: 'camera',
  };
  const r = await fetch(`${QUANT_API_V2}/vision/calibration/move-pose`, {
    method: 'POST',
    headers: _headers(),
    body: JSON.stringify(payload),
  });
  const d = await r.json();
  if (!d?.ok || !d?.data) throw new Error(d?.error || 'No se pudo mover el cuello');
  _renderVisionCalibration(container, d.data);
  return d.data;
}

async function _saveVisionCalibrationSample(container) {
  const payload = {
    point_id: container.querySelector('#aq-calib-point-id')?.value || null,
    use_current_pose: !!container.querySelector('#aq-calib-use-current')?.checked,
    note: container.querySelector('#aq-calib-note')?.value?.trim() || '',
  };
  if (!payload.use_current_pose) {
    payload.yaw = parseFloat(container.querySelector('#aq-calib-yaw')?.value || '0');
    payload.pitch = parseFloat(container.querySelector('#aq-calib-pitch')?.value || '0');
    payload.zoom = parseFloat(container.querySelector('#aq-calib-zoom')?.value || '1');
  }
  const r = await fetch(`${QUANT_API_V2}/vision/calibration/sample`, {
    method: 'POST',
    headers: _headers(),
    body: JSON.stringify(payload),
  });
  const d = await r.json();
  if (!d?.ok || !d?.data) throw new Error(d?.error || 'No se pudo guardar la muestra');
  _renderVisionCalibration(container, d.data);
  return d.data;
}

async function _fitVisionCalibration(container) {
  const savePath = container.querySelector('#aq-calib-save-path')?.value?.trim() || null;
  const r = await fetch(`${QUANT_API_V2}/vision/calibration/fit`, {
    method: 'POST',
    headers: _headers(),
    body: JSON.stringify({ save_path: savePath }),
  });
  const d = await r.json();
  if (!d?.ok || !d?.data) throw new Error(d?.error || 'No se pudo ajustar la calibracion');
  _renderVisionCalibration(container, d.data);
  return d.data;
}

async function _resetVisionCalibration(container) {
  const r = await fetch(`${QUANT_API_V2}/vision/calibration/reset`, {
    method: 'POST',
    headers: _headers(),
    body: JSON.stringify({ clear_active: false }),
  });
  const d = await r.json();
  if (!d?.ok || !d?.data) throw new Error(d?.error || 'No se pudo reiniciar la calibracion');
  _renderVisionCalibration(container, d.data);
  return d.data;
}

function _renderStatusSummary(container, payload = {}) {
  const dot  = container.querySelector('#aq-dot');
  const stat = container.querySelector('#aq-status');
  const up   = container.querySelector('#aq-uptime');
  const strat = container.querySelector('#aq-strategies');
  const pos  = container.querySelector('#aq-positions');
  if (dot) dot.className = 'provider-dot ok';
  if (stat) {
    stat.textContent = _translateText(payload.service_status || 'ONLINE');
    stat.style.color = String(payload.service_status || '').toUpperCase() === 'OFFLINE' ? 'var(--accent-red)' : 'var(--accent-green)';
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
    const d = await _fetchJsonRetry(`${QUANT_API_V2}/status${qs ? `?${qs}` : ''}`, { headers: _headers() });
    if (!d?.ok || !d?.data) throw new Error(d?.error || 'Estado no disponible');
    _renderStatusSummary(container, d.data);
  } catch {
    const dot  = container.querySelector('#aq-dot');
    const stat = container.querySelector('#aq-status');
    if (dot) dot.className = 'provider-dot down';
    if (stat) { stat.textContent = 'FUERA DE LINEA'; stat.style.color = 'var(--accent-red)'; }
  }
}

async function _fetchPositions(container) {
  const el = container.querySelector('#aq-pos-list');
  if (!el) return;
  try {
    const d = await _fetchJsonRetry(`${QUANT_API_V2}/positions`, { headers: _headers() });
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
          Entrada: ${p.entry_price} · Actual: ${p.current_price} · Tamano: ${p.size}
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
        <div class="stat-card-sub">${_esc(_labelScope(acct.classification) || 'sin sesion')} · ${_esc(_labelScope(acct.scope) || '—')}</div>
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
        <div class="stat-card-sub">actualiza cada ${Math.round((m.refresh_interval_sec || 300) / 60)} min</div>
      </div>
    </div>
    <div style="display:flex;gap:10px;flex-wrap:wrap">
      <span class="chip blue">Posiciones: ${_esc(String(m?.totals?.positions ?? '0'))}</span>
      <span class="chip accent">Estrategias: ${_esc(String(m?.totals?.strategies ?? '0'))}</span>
      <span class="chip ${Number(m?.totals?.open_pnl || 0) >= 0 ? 'green' : 'red'}">PnL abierto: ${_esc(_fmtMoney(m?.totals?.open_pnl || 0))}</span>
      <span class="chip ${pdt.blocked_opening ? 'red' : 'blue'}">Operaciones intradia 5d: ${_esc(String(pdt.day_trades_last_window ?? 0))}</span>
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
            <div class="provider-role">Prob. exito</div>
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
            <div class="provider-role">Motor dominante</div>
            <div class="provider-name" style="font-size:14px">${_esc(_translateText(s?.attribution?.dominant_driver || '—'))}</div>
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
    const d = await _fetchJsonRetry(`${QUANT_API_V2}/monitor/summary${qs ? `?${qs}` : ''}`, { headers: _headers() });
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

function _syncScannerForm(container, config = {}) {
  const setValue = (selector, value) => {
    const el = container.querySelector(selector);
    if (el && value !== undefined && value !== null) el.value = Array.isArray(value) ? value.join(', ') : value;
  };
  const setChecked = (selector, value) => {
    const el = container.querySelector(selector);
    if (el) el.checked = !!value;
  };
  setChecked('#aq-scanner-enabled', config.enabled !== false);
  setValue('#aq-scanner-source', config.source || 'yfinance');
  setValue('#aq-scanner-interval', config.scan_interval_sec || 180);
  setValue('#aq-scanner-min-strength', config.min_signal_strength ?? 0.55);
  setValue('#aq-scanner-min-win', config.min_local_win_rate_pct ?? 53);
  setValue('#aq-scanner-min-score', config.min_selection_score ?? 75);
  setValue('#aq-scanner-max-candidates', config.max_candidates || 8);
  setChecked('#aq-scanner-confirm-higher', config.require_higher_tf_confirmation !== false);
  setValue('#aq-scanner-universe-mode', config.universe_mode || 'us_equities_rotating');
  setValue('#aq-scanner-batch-size', config.universe_batch_size || 80);
  setValue('#aq-scanner-prefilter-count', config.prefilter_count || 20);
  setValue('#aq-scanner-timeframes', config.timeframes || ['5m', '15m', '1h', '4h', '1d']);
  setValue('#aq-scanner-universe', config.universe || []);
  setValue('#aq-scanner-notes', config.notes || '');
  _syncScannerUniverseModeUI(container);
}

function _syncScannerUniverseModeUI(container) {
  const mode = container.querySelector('#aq-scanner-universe-mode')?.value || 'us_equities_rotating';
  const manual = mode === 'manual';
  const universe = container.querySelector('#aq-scanner-universe');
  const batchSize = container.querySelector('#aq-scanner-batch-size');
  const prefilter = container.querySelector('#aq-scanner-prefilter-count');
  const hint = container.querySelector('#aq-scanner-universe-hint');
  if (universe) {
    universe.disabled = !manual;
    universe.style.opacity = manual ? '1' : '0.6';
    universe.placeholder = manual
      ? 'SPY, QQQ, IWM, AAPL...'
      : 'El scanner usa el catalogo total de acciones USA y rota por lotes automaticamente';
  }
  if (batchSize) batchSize.disabled = manual;
  if (prefilter) prefilter.disabled = manual;
  if (hint) {
    hint.textContent = manual
      ? 'Modo manual: defines la lista exacta de simbolos a barrer.'
      : 'Modo mercado total: ATLAS rota por el universo completo, prefiltra y profundiza solo sobre los mas prometedores.';
  }
}

function _buildScannerConfig(container) {
  return {
    enabled: !!container.querySelector('#aq-scanner-enabled')?.checked,
    source: container.querySelector('#aq-scanner-source')?.value || 'yfinance',
    scan_interval_sec: parseInt(container.querySelector('#aq-scanner-interval')?.value || '180', 10) || 180,
    min_signal_strength: parseFloat(container.querySelector('#aq-scanner-min-strength')?.value || '0.55'),
    min_local_win_rate_pct: parseFloat(container.querySelector('#aq-scanner-min-win')?.value || '53'),
    min_selection_score: parseFloat(container.querySelector('#aq-scanner-min-score')?.value || '75'),
    max_candidates: parseInt(container.querySelector('#aq-scanner-max-candidates')?.value || '8', 10) || 8,
    require_higher_tf_confirmation: !!container.querySelector('#aq-scanner-confirm-higher')?.checked,
    universe_mode: container.querySelector('#aq-scanner-universe-mode')?.value || 'us_equities_rotating',
    universe_batch_size: parseInt(container.querySelector('#aq-scanner-batch-size')?.value || '80', 10) || 80,
    prefilter_count: parseInt(container.querySelector('#aq-scanner-prefilter-count')?.value || '20', 10) || 20,
    timeframes: String(container.querySelector('#aq-scanner-timeframes')?.value || '5m, 15m, 1h, 4h, 1d')
      .split(',')
      .map((item) => item.trim())
      .filter(Boolean),
    universe: String(container.querySelector('#aq-scanner-universe')?.value || '')
      .split(',')
      .map((item) => item.trim().toUpperCase())
      .filter(Boolean),
    notes: container.querySelector('#aq-scanner-notes')?.value?.trim() || '',
  };
}

function _scannerTemplateMode(container) {
  return container.querySelector('#aq-scanner-order-template')?.value || 'auto';
}

function _scannerDirectionBias(candidate = {}) {
  return String(candidate?.direction || '').toLowerCase() === 'bajista' ? 'bearish' : 'bullish';
}

function _scannerTemplateLabel(mode) {
  return {
    auto: 'automatico',
    simple: 'opcion simple',
    debit_spread: 'spread de debito',
    credit_spread: 'spread de credito',
  }[mode] || (mode || 'automatico');
}

function _recommendScannerOrderPlan(container, candidate = {}) {
  const mode = _scannerTemplateMode(container);
  const bias = _scannerDirectionBias(candidate);
  const timeframe = String(candidate?.timeframe || '');
  const key = String(candidate?.strategy_key || '');
  const direction = String(candidate?.direction || '').toLowerCase();
  const higherDirection = String(candidate?.confirmation?.direction || '').toLowerCase();
  const confirmed = !!higherDirection && higherDirection === direction;
  const localWinRate = Number(candidate?.local_win_rate_pct || 0);
  const selectionScore = Number(candidate?.selection_score || 0);
  const predictedMovePct = Number(candidate?.predicted_move_pct || 0);
  const fastTimeframe = ['5m', '15m'].includes(timeframe);
  const slowTimeframe = ['1h', '4h', '1d'].includes(timeframe);
  const breakoutLike = ['breakout_donchian', 'ml_directional'].includes(key);

  let strategyType = '';
  let rationale = '';
  if (mode === 'simple') {
    strategyType = bias === 'bullish' ? 'long_call' : 'long_put';
    rationale = 'Plantilla simple: riesgo totalmente definido para probar la direccion del setup.';
  } else if (mode === 'debit_spread') {
    strategyType = bias === 'bullish' ? 'bull_call_debit_spread' : 'bear_put_debit_spread';
    rationale = 'Plantilla de debito: acompana el movimiento esperado con riesgo acotado y menor costo relativo.';
  } else if (mode === 'credit_spread') {
    strategyType = bias === 'bullish' ? 'bull_put_credit_spread' : 'bear_call_credit_spread';
    rationale = 'Plantilla de credito: vende prima de forma definida cuando la confirmacion es suficiente.';
  } else if (breakoutLike || fastTimeframe) {
    strategyType = bias === 'bullish' ? 'long_call' : 'long_put';
    rationale = 'Modo automatico: la senal es rapida o tipo breakout/ML, se privilegia opcion simple para capturar desplazamiento.';
  } else if (confirmed && slowTimeframe && localWinRate >= 62 && selectionScore >= 70 && predictedMovePct <= 2.6) {
    strategyType = bias === 'bullish' ? 'bull_put_credit_spread' : 'bear_call_credit_spread';
    rationale = 'Modo automatico: confirmacion multi-temporal alta y desplazamiento moderado, se propone spread de credito acotado.';
  } else {
    strategyType = bias === 'bullish' ? 'bull_call_debit_spread' : 'bear_put_debit_spread';
    rationale = 'Modo automatico: setup direccional confirmado, se propone spread de debito con riesgo definido.';
  }

  return {
    mode,
    mode_label: _scannerTemplateLabel(mode),
    strategyType,
    direction_bias: bias,
    rationale,
    tag: `scanner:${key || 'idea'}:${timeframe || 'tf'}`,
  };
}

function _selectorConfig(container) {
  return {
    chart_provider: container.querySelector('#aq-selector-chart-provider')?.value || 'tradingview',
    prefer_defined_risk: !!container.querySelector('#aq-selector-defined-risk')?.checked,
    allow_equity: !!container.querySelector('#aq-selector-allow-equity')?.checked,
    allow_credit: !!container.querySelector('#aq-selector-allow-credit')?.checked,
    risk_budget_pct: parseFloat(container.querySelector('#aq-selector-risk-budget')?.value || '0.75') || 0.75,
    open_charts_after_prepare: !!container.querySelector('#aq-selector-open-charts')?.checked,
  };
}

function _selectorRequest(container, candidate = {}) {
  const cfg = _selectorConfig(container);
  return {
    candidate: {
      symbol: candidate.symbol,
      timeframe: candidate.timeframe,
      direction: candidate.direction,
      price: candidate.price,
      strategy_key: candidate.strategy_key,
      strategy_label: candidate.strategy_label,
      selection_score: candidate.selection_score,
      local_win_rate_pct: candidate.local_win_rate_pct,
      predicted_move_pct: candidate.predicted_move_pct,
      relative_strength_pct: candidate.relative_strength_pct,
      confirmation: candidate.confirmation || {},
      why_selected: Array.isArray(candidate.why_selected) ? candidate.why_selected : [],
    },
    account_scope: _orderScope(container),
    account_id: _orderAccountId(container),
    chart_provider: cfg.chart_provider,
    prefer_defined_risk: cfg.prefer_defined_risk,
    allow_equity: cfg.allow_equity,
    allow_credit: cfg.allow_credit,
    risk_budget_pct: cfg.risk_budget_pct,
  };
}

function _renderSelectorSummary(container, payload = null, errorText = '') {
  const el = container.querySelector('#aq-selector-summary');
  const stateEl = container.querySelector('#aq-selector-state');
  if (!el) return;
  if (stateEl) stateEl.innerHTML = '';
  if (errorText) {
    el.innerHTML = `<div style="color:var(--accent-red);font-size:12px;padding:8px">${_esc(errorText)}</div>`;
    if (stateEl) stateEl.innerHTML = '<span class="chip red">selector con error</span>';
    return;
  }
  if (!payload) {
    el.innerHTML = `
      <div class="empty-state" style="padding:16px 0">
        <div class="empty-title">Sin propuesta todavia</div>
        <div class="empty-sub">Prepara una idea desde el escaner y aqui veras estructura, lotaje, graficos y plan de salida.</div>
      </div>
    `;
    if (stateEl) stateEl.innerHTML = '<span class="chip blue">esperando idea</span>';
    return;
  }
  const selected = payload.selected || {};
  const meta = selected.meta || {};
  const candidate = payload.candidate || {};
  const sizePlan = payload.size_plan || {};
  const chartPlan = payload.chart_plan || {};
  const cameraPlan = payload.camera_plan || {};
  const entryPlan = payload.entry_plan || {};
  const confidence = payload.confidence_breakdown || {};
  const adaptive = payload.adaptive_context || {};
  const riskProfile = payload.risk_profile || {};
  const automation = payload.automation_ready || {};
  const exitPlan = payload.exit_plan || {};
  const playbook = payload.playbook || {};
  const alternatives = Array.isArray(payload.alternatives) ? payload.alternatives : [];
  const warnings = Array.isArray(payload.warnings) ? payload.warnings : [];
  const riskCurve = _buildSelectorRiskCurve(payload);
  const breakEvenText = Array.isArray(riskProfile.break_even_points) && riskProfile.break_even_points.length
    ? riskProfile.break_even_points.map((value) => _fmtPrice(value)).join(' · ')
    : 'sin punto claro';
  if (stateEl) {
    stateEl.innerHTML = `
      <span class="chip green">${_esc(candidate.symbol || '—')}</span>
      <span class="chip blue">${_esc(candidate.timeframe || '—')}</span>
      <span class="chip accent">${_esc(meta.label || selected.strategy_type || '—')}</span>
    `;
  }
  el.innerHTML = `
    <div class="stat-row" style="margin-bottom:12px">
      <div class="stat-card hero">
        <div class="stat-card-label">Estructura elegida</div>
        <div class="stat-card-value" style="font-size:16px">${_esc(meta.label || selected.strategy_type || '—')}</div>
        <div class="stat-card-sub">${_esc((selected.why || [])[0] || 'sin razon principal')}</div>
      </div>
      <div class="stat-card">
        <div class="stat-card-label">Activo</div>
        <div class="stat-card-value" style="font-size:16px">${_esc(candidate.symbol || '—')}</div>
        <div class="stat-card-sub">${_esc(candidate.timeframe || '—')} · ${_esc(candidate.direction || '—')}</div>
      </div>
      <div class="stat-card">
        <div class="stat-card-label">Tamano sugerido</div>
        <div class="stat-card-value green">${_esc(String(sizePlan.suggested_size ?? '—'))}</div>
        <div class="stat-card-sub">${_esc(sizePlan.capital_source || 'capital')}</div>
      </div>
      <div class="stat-card">
        <div class="stat-card-label">Validacion visual</div>
        <div class="stat-card-value blue">${_fmtPct(cameraPlan.visual_fit_pct, 1)}</div>
        <div class="stat-card-sub">${_esc(cameraPlan.provider || '—')}</div>
      </div>
    </div>
    <div style="display:grid;grid-template-columns:1.1fr .9fr;gap:12px">
      <div class="approval-card" style="padding:12px;background:rgba(255,255,255,0.02)">
        <div class="section-title" style="font-size:14px;margin:0 0 10px">Decision del selector IA</div>
        <div style="display:flex;gap:8px;flex-wrap:wrap;margin-bottom:10px">
          <span class="chip blue">${_esc(candidate.strategy_label || candidate.strategy_key || 'setup')}</span>
          <span class="chip accent">${_esc(selected.strategy_type || '—')}</span>
          <span class="chip green">score ${_fmtNum(selected.score, 1)}</span>
          ${candidate?.order_flow?.available ? `<span class="chip ${candidate.order_flow.direction === candidate.direction ? 'green' : candidate.order_flow.direction === 'neutral' ? 'blue' : 'orange'}">order flow ${_esc(candidate.order_flow.direction || 'neutral')} ${_fmtPct(candidate.order_flow.score_pct, 1)}</span>` : ''}
          ${payload.probability?.win_rate_pct !== undefined ? `<span class="chip blue">win ${_fmtPct(payload.probability.win_rate_pct, 1)}</span>` : ''}
        </div>
        <div style="font-size:12px;line-height:1.55;color:var(--text-secondary)">${_esc((selected.why || []).join(' · ') || 'sin detalle')}</div>
        <div style="display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px;margin-top:12px">
          <div class="provider-card" style="padding:10px"><div class="provider-role">Presupuesto de riesgo</div><div class="provider-name" style="font-size:15px">${_fmtMoney(sizePlan.risk_budget_usd)}</div></div>
          <div class="provider-card" style="padding:10px"><div class="provider-role">Riesgo por unidad</div><div class="provider-name" style="font-size:15px">${_fmtMoney(sizePlan.per_unit_risk_usd)}</div></div>
          <div class="provider-card" style="padding:10px"><div class="provider-role">Notional estimado</div><div class="provider-name" style="font-size:15px">${_fmtMoney(sizePlan.estimated_notional_usd)}</div></div>
        </div>
        <div style="font-size:11px;color:var(--text-muted);margin-top:8px">${_esc(sizePlan.notes || '')}</div>
      </div>
      <div class="approval-card" style="padding:12px;background:rgba(13,23,45,.42)">
        <div class="section-title" style="font-size:14px;margin:0 0 10px">Graficos y camara</div>
        <div style="display:flex;flex-direction:column;gap:8px">
          ${(chartPlan.targets || []).map((item) => `
            <div class="provider-card" style="padding:10px">
              <div class="provider-name" style="font-size:12px">${_esc(item.title || item.timeframe || 'grafico')}</div>
              <div class="provider-role">${_esc(item.url || '')}</div>
            </div>
          `).join('')}
        </div>
        <div style="display:flex;gap:6px;flex-wrap:wrap;margin-top:10px">
          ${(cameraPlan.checkpoints || []).map((item) => `<span class="chip blue">${_esc(item)}</span>`).join(' ')}
        </div>
      </div>
    </div>
    <div class="approval-card" style="padding:12px;background:rgba(255,255,255,0.02);margin-top:12px">
      <div style="display:flex;justify-content:space-between;gap:10px;font-size:11px;color:var(--text-muted);margin-bottom:6px">
        <span>Perfil de riesgo previo del selector</span>
        <span>${payload.probability ? 'basado en estructura y payoffs' : 'estimado para accion / estructura sin opciones'}</span>
      </div>
      ${_curveSvg(riskCurve)}
      <div style="display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;margin-top:12px">
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Break-even</div>
          <div class="provider-name" style="font-size:13px">${_esc(breakEvenText)}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Riesgo maximo</div>
          <div class="provider-name" style="font-size:13px;color:var(--accent-red)">${_fmtMoney(riskProfile.max_loss_usd)}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Potencial</div>
          <div class="provider-name" style="font-size:13px;color:var(--accent-green)">${riskProfile.max_profit_open_ended ? 'abierto' : _fmtMoney(riskProfile.max_profit_usd)}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Relacion R/B</div>
          <div class="provider-name" style="font-size:13px">${riskProfile.reward_to_risk !== undefined && riskProfile.reward_to_risk !== null ? `${_fmtNum(riskProfile.reward_to_risk, 2)}x` : 'â€”'}</div>
        </div>
      </div>
      ${(riskProfile.scenarios || []).length ? `
        <div style="display:grid;grid-template-columns:repeat(5,minmax(0,1fr));gap:8px;margin-top:12px">
          ${(riskProfile.scenarios || []).map((item) => `
            <div class="provider-card" style="padding:10px">
              <div class="provider-role">${_esc(item.label || 'escenario')}</div>
              <div class="provider-name" style="font-size:13px">${_fmtPrice(item.price)}</div>
              <div style="font-size:12px;color:${_moneyColor(item.pnl_usd)};margin-top:4px">${_fmtMoney(item.pnl_usd)}</div>
            </div>
          `).join('')}
        </div>
      ` : ''}
      ${(riskProfile.notes || []).length ? `<div style="display:flex;gap:6px;flex-wrap:wrap;margin-top:10px">${(riskProfile.notes || []).map((item) => `<span class="chip blue">${_esc(item)}</span>`).join(' ')}</div>` : ''}
    </div>
    <div style="display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-top:12px">
      <div class="approval-card" style="padding:12px;background:rgba(255,255,255,0.02)">
        <div class="section-title" style="font-size:14px;margin:0 0 10px">Plan de entrada</div>
        <div style="display:flex;flex-direction:column;gap:8px;font-size:12px;color:var(--text-secondary)">
          <div><strong>Estilo:</strong> ${_esc(entryPlan.entry_style || '—')}</div>
          <div><strong>Disparo:</strong> ${_esc(entryPlan.trigger_rule || '—')}</div>
          <div><strong>Confirmacion:</strong> ${_esc(entryPlan.confirmation_rule || '—')}</div>
          <div><strong>Foco en grafico:</strong> ${_esc(entryPlan.chart_focus || '—')}</div>
          <div><strong>DTE sugerido:</strong> ${_esc(entryPlan.dte_window || '—')}</div>
        </div>
      </div>
      <div class="approval-card" style="padding:12px;background:rgba(255,255,255,0.02)">
        <div class="section-title" style="font-size:14px;margin:0 0 10px">Confianza y automatizacion</div>
        <div style="display:flex;gap:6px;flex-wrap:wrap;margin-bottom:10px">
          <span class="chip blue">selector ${_fmtPct(confidence.selector_score_pct, 1)}</span>
          <span class="chip blue">local ${_fmtPct(confidence.local_win_rate_pct, 1)}</span>
          <span class="chip blue">confirmacion ${_fmtPct(confidence.confirmation_confidence_pct, 1)}</span>
          ${confidence.order_flow_score_pct !== undefined ? `<span class="chip blue">flow ${_fmtPct(confidence.order_flow_score_pct, 1)}</span>` : ''}
          ${confidence.order_flow_confidence_pct !== undefined ? `<span class="chip blue">flow conf ${_fmtPct(confidence.order_flow_confidence_pct, 1)}</span>` : ''}
          <span class="chip blue">camara ${_fmtPct(confidence.camera_visual_fit_pct, 1)}</span>
          ${confidence.adaptive_bias_pct !== undefined ? `<span class="chip ${Number(confidence.adaptive_bias_pct || 0) >= 0 ? 'green' : 'orange'}">aprendizaje ${_fmtNum(confidence.adaptive_bias_pct, 2)}</span>` : ''}
          ${confidence.probability_win_rate_pct !== undefined ? `<span class="chip green">probabilidad ${_fmtPct(confidence.probability_win_rate_pct, 1)}</span>` : ''}
        </div>
        <div style="display:flex;gap:6px;flex-wrap:wrap">
          <span class="chip ${automation.charts_ready ? 'green' : 'orange'}">graficos ${automation.charts_ready ? 'listos' : 'pendientes'}</span>
          <span class="chip ${automation.camera_ready ? 'green' : 'orange'}">camara ${automation.camera_ready ? 'lista' : 'revisar'}</span>
          <span class="chip ${automation.ticket_ready ? 'green' : 'orange'}">ticket ${automation.ticket_ready ? 'listo' : 'pendiente'}</span>
          <span class="chip ${automation.operation_preview_ready ? 'green' : 'orange'}">preview ${automation.operation_preview_ready ? 'listo' : 'pendiente'}</span>
          <span class="chip ${automation.paper_submit_ready ? 'green' : 'orange'}">paper ${automation.paper_submit_ready ? 'habilitado' : 'no'}</span>
        </div>
        <div style="font-size:11px;color:var(--text-muted);margin-top:8px">${_esc(automation.recommended_autonomous_path || '')}</div>
      </div>
    </div>
    <div class="approval-card" style="padding:12px;background:rgba(255,255,255,0.02);margin-top:12px">
      <div class="section-title" style="font-size:14px;margin:0 0 10px">Aprendizaje aplicado</div>
      <div style="display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px">
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Bias total</div>
          <div class="provider-name" style="font-size:13px;color:${Number(adaptive.total_bias || 0) >= 0 ? 'var(--accent-green)' : 'var(--accent-orange)'}">${_fmtNum(adaptive.total_bias, 2)}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Bias simbolo</div>
          <div class="provider-name" style="font-size:13px">${_fmtNum(adaptive.symbol_bias, 2)}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Bias direccion</div>
          <div class="provider-name" style="font-size:13px">${_fmtNum(adaptive.directional_symbol_bias, 2)}</div>
        </div>
        <div class="provider-card" style="padding:10px">
          <div class="provider-role">Bias estructura</div>
          <div class="provider-name" style="font-size:13px">${_fmtNum(adaptive.strategy_bias, 2)}</div>
        </div>
      </div>
      <div style="display:flex;gap:6px;flex-wrap:wrap;margin-top:10px">
        ${adaptive.risk_multiplier !== undefined ? `<span class="chip blue">multiplicador de riesgo ${_fmtNum(adaptive.risk_multiplier, 2)}x</span>` : ''}
        ${Array.isArray(adaptive.notes) ? adaptive.notes.map((item) => `<span class="chip ${Number(adaptive.total_bias || 0) >= 0 ? 'green' : 'orange'}">${_esc(item)}</span>`).join(' ') : ''}
      </div>
    </div>
    <div style="display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-top:12px">
      <div class="approval-card" style="padding:12px;background:rgba(255,255,255,0.02)">
        <div class="section-title" style="font-size:14px;margin:0 0 10px">Plan de salida</div>
        <div style="display:flex;flex-direction:column;gap:8px;font-size:12px;color:var(--text-secondary)">
          <div><strong>Objetivo:</strong> ${_esc(exitPlan.primary_take_profit || '—')}</div>
          <div><strong>Invalidacion:</strong> ${_esc(exitPlan.risk_invalidation || '—')}</div>
          <div><strong>Tiempo:</strong> ${_esc(exitPlan.time_stop || '—')}</div>
        </div>
      </div>
      <div class="approval-card" style="padding:12px;background:rgba(255,255,255,0.02)">
        <div class="section-title" style="font-size:14px;margin:0 0 10px">Alternativas consideradas</div>
        <div style="display:flex;flex-direction:column;gap:8px">
          ${alternatives.slice(0, 3).map((item) => `
            <div class="provider-card" style="padding:10px">
              <div style="display:flex;justify-content:space-between;gap:8px;align-items:center">
                <div class="provider-name" style="font-size:12px">${_esc(item.strategy_type || '—')}</div>
                <span class="chip blue">${_fmtNum(item.score, 1)}</span>
              </div>
              <div class="provider-role" style="margin-top:4px">${_esc((item.why || []).join(' · ') || 'sin detalle')}</div>
            </div>
          `).join('')}
        </div>
      </div>
      <div class="approval-card" style="padding:12px;background:rgba(255,255,255,0.02)">
        <div class="section-title" style="font-size:14px;margin:0 0 10px">Playbook operativo</div>
        <div style="display:flex;gap:6px;flex-wrap:wrap;margin-bottom:10px">
          <span class="chip accent">${_esc(playbook.family || 'sin familia')}</span>
          <span class="chip blue">${_esc(playbook.direction || '—')}</span>
          <span class="chip blue">${_esc(playbook.timeframe || '—')}</span>
          ${playbook?.autonomy_policy?.autonomous_chart_opening ? '<span class="chip green">graficos autonomos</span>' : ''}
          ${playbook?.autonomy_policy?.can_submit_without_supervision ? '<span class="chip green">paper autonomo listo</span>' : '<span class="chip orange">supervision recomendada</span>'}
        </div>
        <div style="display:flex;flex-direction:column;gap:8px;font-size:12px;color:var(--text-secondary)">
          <div><strong>Entrada:</strong> ${_esc(playbook?.entry_protocol?.style || '—')}</div>
          <div><strong>Disparo:</strong> ${_esc(playbook?.entry_protocol?.trigger || '—')}</div>
          <div><strong>Confirmacion:</strong> ${_esc(playbook?.entry_protocol?.confirmation || '—')}</div>
          <div><strong>Foco grafico:</strong> ${_esc(playbook?.entry_protocol?.chart_focus || '—')}</div>
        </div>
        <div style="display:flex;gap:6px;flex-wrap:wrap;margin-top:10px">
          ${(playbook.pre_ticket_checks || []).map((item) => `<span class="chip blue">${_esc(item)}</span>`).join(' ')}
        </div>
      </div>
    </div>
    <div style="display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-top:12px">
      <div class="approval-card" style="padding:12px;background:rgba(255,255,255,0.02)">
        <div class="section-title" style="font-size:14px;margin:0 0 10px">Gestion de posicion</div>
        <div style="display:flex;gap:6px;flex-wrap:wrap">
          ${(playbook.manage_rules || []).map((item) => `<span class="chip blue">${_esc(item)}</span>`).join(' ')}
        </div>
      </div>
      <div class="approval-card" style="padding:12px;background:rgba(255,255,255,0.02)">
        <div class="section-title" style="font-size:14px;margin:0 0 10px">Disparadores de revision</div>
        <div style="display:flex;gap:6px;flex-wrap:wrap">
          ${(playbook.review_triggers || []).map((item) => `<span class="chip orange">${_esc(item)}</span>`).join(' ')}
        </div>
        <div style="display:flex;gap:6px;flex-wrap:wrap;margin-top:10px">
          ${playbook.adaptive_note ? `<span class="chip ${Number(adaptive.total_bias || 0) >= 0 ? 'green' : 'orange'}">${_esc(playbook.adaptive_note)}</span>` : ''}
        </div>
      </div>
    </div>
    ${warnings.length ? `<div style="display:flex;gap:8px;flex-wrap:wrap;margin-top:12px">${warnings.map((item) => `<span class="chip orange">${_esc(item)}</span>`).join(' ')}</div>` : ''}
  `;
}

async function _fetchSelectorProposal(container, candidate, { autoOpenCharts = false } = {}) {
  if (!candidate) throw new Error('No hay candidato del escaner para preparar');
  const summaryEl = container.querySelector('#aq-selector-summary');
  if (summaryEl) {
    summaryEl.innerHTML = '<div style="text-align:center;padding:12px"><div class="spinner" style="margin:0 auto;display:block"></div><div style="color:var(--text-muted);font-size:11px;margin-top:6px">Seleccionando estructura, sizing y graficos...</div></div>';
  }
  const payload = _selectorRequest(container, candidate);
  const r = await fetch(`${QUANT_API_V2}/selector/proposal`, {
    method: 'POST',
    headers: _headers(),
    body: JSON.stringify(payload),
  });
  const d = await r.json();
  if (!d?.ok || !d?.data) throw new Error(d?.error || 'No se pudo generar la propuesta del selector');
  container.__atlasSelectorCandidate = candidate;
  container.__atlasSelectorProposal = d.data;
  _renderSelectorSummary(container, d.data);
  _activateQuantView(container, 'selector');
  if (autoOpenCharts) _openSelectorCharts(container);
  return d.data;
}

function _openSelectorCharts(container) {
  const proposal = container.__atlasSelectorProposal || null;
  const targets = Array.isArray(proposal?.chart_plan?.targets) ? proposal.chart_plan.targets : [];
  if (!targets.length) throw new Error('No hay graficos preparados para abrir');
  targets.forEach((item) => {
    if (item?.url) window.open(item.url, '_blank', 'noopener,noreferrer');
  });
  return targets.length;
}

async function _applySelectorToTicket(container, { runBuilder = true } = {}) {
  const proposal = container.__atlasSelectorProposal || null;
  const candidate = container.__atlasSelectorCandidate || null;
  if (!proposal || !candidate) throw new Error('No hay propuesta del selector para cargar');
  const seed = proposal.order_seed || {};
  const setValue = (selector, value) => {
    const el = container.querySelector(selector);
    if (el && value !== undefined && value !== null) el.value = String(value);
  };
  const setChecked = (selector, value) => {
    const el = container.querySelector(selector);
    if (el) el.checked = !!value;
  };
  _forcePaperScopeForScannerBridge(container);
  setValue('#aq-order-preset', 'custom');
  setValue('#aq-order-symbol', seed.symbol || candidate.symbol || '');
  setValue('#aq-order-asset-class', seed.asset_class || 'auto');
  setValue('#aq-order-side', seed.side || 'buy');
  setValue('#aq-order-position-effect', seed.position_effect || 'open');
  setValue('#aq-order-type', seed.order_type || 'market');
  setValue('#aq-order-duration', seed.duration || 'day');
  setValue('#aq-order-size', seed.size || 1);
  setValue('#aq-order-tag', seed.tag || '');
  setValue('#aq-order-strategy-type', seed.strategy_type || '');
  setValue('#aq-order-option-symbol', '');
  setValue('#aq-order-legs', '');
  setValue('#aq-order-price', '');
  setValue('#aq-order-stop-price', '');
  setChecked('#aq-order-probability-enabled', !!seed.strategy_type);
  setValue('#aq-order-min-win-rate', proposal?.probability?.win_rate_pct ? Math.max(50, Number(proposal.probability.win_rate_pct) - 2).toFixed(1) : '55');
  setValue('#aq-order-risk-pct', proposal?.size_plan?.risk_budget_pct || 0.75);
  if (runBuilder && seed.strategy_type) {
    await _buildOrderFromStrategy(container, { silent: true });
  } else {
    container.__atlasQuantLastProposal = null;
    _renderProbabilityProposal(container.querySelector('#aq-order-proposal'), null);
  }
  _activateQuantView(container, 'ejecucion');
}

async function _selectorValidateWithCamera(container) {
  await _applySelectorToTicket(container, { runBuilder: true });
  await _saveOperationConfig(container);
  const result = await _runOperationCycle(container, 'preview');
  _activateQuantView(container, 'operacion');
  return result;
}

function _renderScannerBridgeStatus(container, payload = {}) {
  const el = container.querySelector('#aq-scanner-bridge-status');
  if (!el) return;
  const candidates = Array.isArray(payload?.candidates) ? payload.candidates : [];
  const best = candidates[0] || null;
  const last = container.__atlasScannerBridgeState || null;
  if (!best) {
    el.innerHTML = `
      <div class="empty-state" style="padding:10px 0">
        <div class="empty-title">Sin mejor idea todavia</div>
        <div class="empty-sub">Ejecuta un ciclo del escaner y cuando haya una oportunidad podras cargarla directo al ticket.</div>
      </div>
    `;
    return;
  }
  const plan = _recommendScannerOrderPlan(container, best);
  const lastSummary = last
    ? `
      <div style="display:flex;gap:8px;flex-wrap:wrap;margin-top:10px">
        <span class="chip blue">Ultima accion ${_esc(_translateText(last.action || 'cargada'))}</span>
        <span class="chip accent">${_esc(last.symbol || '—')}</span>
        <span class="chip ${last.ok === false ? 'red' : 'green'}">${last.ok === false ? 'con observaciones' : 'lista'}</span>
      </div>
    `
    : '';
  el.innerHTML = `
    <div style="display:flex;align-items:flex-start;justify-content:space-between;gap:12px;flex-wrap:wrap">
      <div>
        <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center">
          <span class="chip ${best.direction === 'alcista' ? 'green' : 'red'}">${_esc(best.direction || '—')}</span>
          <span style="font-weight:700">${_esc(best.symbol || '—')}</span>
          <span class="chip blue">${_esc(best.timeframe || '—')}</span>
          <span class="chip accent">${_esc(plan.strategyType)}</span>
        </div>
        <div style="font-size:11px;color:var(--text-muted);margin-top:6px">
          Score ${_fmtNum(best.selection_score, 1)} · win local ${_fmtPct(best.local_win_rate_pct, 1)} · mov. esperado ${_fmtPct(best.predicted_move_pct, 2)}
        </div>
        ${best?.order_flow?.available ? `<div style="font-size:11px;color:var(--text-muted);margin-top:4px">order flow ${_esc(best.order_flow.direction || 'neutral')} · score ${_fmtPct(best.order_flow.score_pct, 1)} · presión ${_fmtPct(best.order_flow.net_pressure_pct, 1)}</div>` : ''}
        <div style="font-size:12px;line-height:1.5;margin-top:8px;color:var(--text-secondary)">
          ${_esc(plan.rationale)}
        </div>
        ${lastSummary}
      </div>
      <div style="display:flex;gap:8px;flex-wrap:wrap">
        <span class="chip blue">plantilla ${_esc(plan.mode_label)}</span>
        <span class="chip ${String(best?.confirmation?.direction || '').toLowerCase() === String(best.direction || '').toLowerCase() ? 'green' : 'orange'}">
          confirmacion ${_esc(best?.confirmation?.higher_timeframe || '—')}
        </span>
      </div>
    </div>
  `;
}

function _renderScannerReport(container, payload = {}) {
  const summaryEl = container.querySelector('#aq-scanner-summary');
  const criteriaEl = container.querySelector('#aq-scanner-criteria');
  const candidatesEl = container.querySelector('#aq-scanner-candidates');
  const rejectedEl = container.querySelector('#aq-scanner-rejections');
  const activityEl = container.querySelector('#aq-scanner-activity');
  if (!summaryEl || !criteriaEl || !candidatesEl || !rejectedEl || !activityEl) return;
  container.__atlasQuantScanner = payload;
  const status = payload.status || {};
  const summary = payload.summary || {};
  const learning = payload.learning || {};
  const universe = payload.universe || {};
  const criteria = Array.isArray(payload.criteria) ? payload.criteria : [];
  const candidates = Array.isArray(payload.candidates) ? payload.candidates : [];
  const rejections = Array.isArray(payload.rejections) ? payload.rejections : [];
  const activity = Array.isArray(payload.activity) ? payload.activity : [];
  const current = payload.current_work || {};
  if (!container.__atlasScannerHydrated && status.config) {
    _syncScannerForm(container, status.config);
    container.__atlasScannerHydrated = true;
  }
  _renderScannerBridgeStatus(container, payload);
  summaryEl.innerHTML = `
    <div class="stat-row" style="margin-bottom:12px">
      <div class="stat-card hero">
        <div class="stat-card-label">Estado del escaner</div>
        <div class="stat-card-value" style="font-size:16px">${status.running ? 'escaneando' : 'en espera'}</div>
        <div class="stat-card-sub">${_esc(current.step || status.current_step || 'sin actividad')}</div>
      </div>
      <div class="stat-card">
        <div class="stat-card-label">Activo actual</div>
        <div class="stat-card-value" style="font-size:16px">${_esc(status.current_symbol || current.symbol || '—')}</div>
        <div class="stat-card-sub">${_esc(status.current_timeframe || current.timeframe || '—')}</div>
      </div>
      <div class="stat-card">
        <div class="stat-card-label">Ciclo</div>
        <div class="stat-card-value">${_esc(String(summary.cycle_count ?? status.cycle_count ?? 0))}</div>
        <div class="stat-card-sub">${summary.last_cycle_ms || status.last_cycle_ms ? `${_fmtNum(summary.last_cycle_ms || status.last_cycle_ms, 0)} ms` : 'sin ciclo'}</div>
      </div>
      <div class="stat-card">
        <div class="stat-card-label">Candidatos</div>
        <div class="stat-card-value ${candidates.length > 0 ? 'green' : 'orange'}">${_esc(String(candidates.length))}</div>
        <div class="stat-card-sub">rechazados: ${_esc(String(rejections.length))}</div>
      </div>
    </div>
    <div style="display:flex;gap:8px;flex-wrap:wrap">
      <span class="chip ${status.running ? 'green' : 'orange'}">${status.running ? 'ciclo activo' : 'ciclo inactivo'}</span>
      <span class="chip blue">intervalo ${_esc(String(status?.config?.scan_interval_sec ?? 180))}s</span>
      <span class="chip accent">modo ${_esc(summary.universe_mode === 'manual' ? 'manual' : 'mercado total')}</span>
      <span class="chip accent">universo ${_esc(String(summary.universe_total ?? summary.universe_size ?? status?.config?.universe?.length ?? 0))}</span>
      <span class="chip blue">lote ${_esc(String(summary.batch_size ?? 0))}</span>
      <span class="chip blue">prefiltro ${_esc(String(summary.prefilter_selected ?? 0))}</span>
      <span class="chip blue">profundo ${_esc(String(summary.deep_scan_symbols ?? 0))}</span>
      <span class="chip ${Number(summary.dynamic_min_selection_score ?? 75) > 75 ? 'orange' : 'green'}">umbral ${_fmtPct(summary.dynamic_min_selection_score ?? 75, 1)}</span>
      <span class="chip blue">previos ${_esc(String(summary.provisional_candidates ?? 0))}</span>
      <span class="chip blue">temporalidades ${_esc((status?.config?.timeframes || []).join(', ') || '—')}</span>
      <span class="chip ${status?.config?.require_higher_tf_confirmation !== false ? 'green' : 'orange'}">${status?.config?.require_higher_tf_confirmation !== false ? 'confirmacion superior ON' : 'confirmacion superior OFF'}</span>
      ${learning.enabled ? `<span class="chip blue">aprendizaje ${_esc(String(learning.sample_count || 0))} muestras</span>` : ''}
      ${learning.risk_multiplier !== undefined ? `<span class="chip accent">riesgo ${_fmtNum(learning.risk_multiplier, 2)}x</span>` : ''}
    </div>
    <div style="display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;margin-top:12px">
      <div class="provider-card" style="padding:10px">
        <div class="provider-role">Cobertura</div>
        <div class="provider-name" style="font-size:14px">${_esc(String((summary.batch_start ?? 0) + 1))} - ${_esc(String(summary.batch_end ?? 0))}</div>
      </div>
      <div class="provider-card" style="padding:10px">
        <div class="provider-role">Catalogo</div>
        <div class="provider-name" style="font-size:14px">${_esc(universe.catalog_source || 'manual')}</div>
      </div>
      <div class="provider-card" style="padding:10px">
        <div class="provider-role">Scored diario</div>
        <div class="provider-name" style="font-size:14px">${_esc(String(summary.prefilter_scored ?? 0))}</div>
      </div>
      <div class="provider-card" style="padding:10px">
        <div class="provider-role">Seleccion previa</div>
        <div class="provider-name" style="font-size:14px">${_esc(String(summary.prefilter_selected ?? 0))}</div>
      </div>
      <div class="provider-card" style="padding:10px">
        <div class="provider-role">Umbral dinamico</div>
        <div class="provider-name" style="font-size:14px">${_fmtPct(summary.dynamic_min_selection_score ?? 75, 1)}</div>
      </div>
    </div>
  `;
  criteriaEl.innerHTML = criteria.map((item) => `
    <div class="provider-card" style="padding:12px">
      <div class="provider-name" style="font-size:13px">${_esc(item.label || item.key || 'criterio')}</div>
      <div class="provider-role" style="margin-top:4px">${_esc(item.description || '')}</div>
      <div style="display:flex;gap:6px;flex-wrap:wrap;margin-top:8px">
        <span class="chip blue">evidencia ${_fmtPct(item.evidence_score, 1)}</span>
        <span class="chip accent">${_esc(item.family || '—')}</span>
      </div>
      ${(item.sources || []).length ? `<div style="margin-top:8px;font-size:11px;color:var(--text-muted)">${(item.sources || []).slice(0, 2).map((src) => src.url ? `<a href="${_esc(src.url)}" target="_blank" style="color:var(--accent-primary);text-decoration:none">${_esc(src.title || 'fuente')}</a>` : _esc(src.title || 'fuente')).join(' · ')}</div>` : ''}
    </div>
  `).join('');
  candidatesEl.innerHTML = candidates.length === 0
    ? `<div class="empty-state" style="padding:16px 0"><div class="empty-sub">Todavia no hay oportunidades que superen el filtro</div></div>`
    : candidates.map((item, idx) => `
      <div class="approval-card" style="padding:14px;margin-bottom:10px">
        <div style="display:flex;align-items:flex-start;justify-content:space-between;gap:10px;flex-wrap:wrap">
          <div>
            <div style="display:flex;align-items:center;gap:8px;flex-wrap:wrap">
              <span class="chip ${item.direction === 'alcista' ? 'green' : 'red'}">${_esc(item.direction)}</span>
              <span style="font-weight:700">${_esc(item.symbol || '—')}</span>
              <span class="chip blue">${_esc(item.timeframe || '—')}</span>
              <span class="chip accent">${_esc(item.strategy_label || item.strategy_key || '—')}</span>
            </div>
            <div style="font-size:11px;color:var(--text-muted);margin-top:6px">${(item.why_selected || []).map((reason) => _esc(reason)).join(' · ')}</div>
          </div>
          <div style="display:flex;gap:8px;flex-wrap:wrap">
            <span class="chip green">score ${_fmtNum(item.selection_score, 1)}</span>
            <span class="chip blue">win ${_fmtPct(item.local_win_rate_pct, 1)}</span>
            <span class="chip orange">mov ${_fmtPct(item.predicted_move_pct, 2)}</span>
            ${item?.order_flow?.available ? `<span class="chip ${item.order_flow.direction === item.direction ? 'green' : item.order_flow.direction === 'neutral' ? 'blue' : 'orange'}">flow ${_esc(item.order_flow.direction || 'neutral')} ${_fmtPct(item.order_flow.score_pct, 1)}</span>` : ''}
            ${item?.adaptive_context?.total_bias !== undefined ? `<span class="chip ${Number(item.adaptive_context.total_bias || 0) >= 0 ? 'green' : 'orange'}">aprendizaje ${_fmtNum(item.adaptive_context.total_bias, 2)}</span>` : ''}
          </div>
        </div>
        <div style="display:flex;gap:8px;flex-wrap:wrap;margin-top:10px">
          <button class="action-btn" data-scanner-bridge-action="selector" data-scanner-index="${idx}">Preparar selector IA</button>
          <button class="action-btn" data-scanner-bridge-action="load" data-scanner-index="${idx}">Cargar ticket</button>
          <button class="action-btn" data-scanner-bridge-action="evaluate" data-scanner-index="${idx}">Evaluar con ATLAS</button>
          <button class="action-btn" data-scanner-bridge-action="preview" data-scanner-index="${idx}">Previsualizar idea</button>
        </div>
        <div style="display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;margin-top:10px">
          <div class="provider-card" style="padding:10px"><div class="provider-role">Precio</div><div class="provider-name" style="font-size:16px">${_fmtNum(item.price, 2)}</div></div>
          <div class="provider-card" style="padding:10px"><div class="provider-role">Fuerza</div><div class="provider-name" style="font-size:16px">${_fmtPct(item.signal_strength_pct, 1)}</div></div>
          <div class="provider-card" style="padding:10px"><div class="provider-role">Fuerza relativa</div><div class="provider-name" style="font-size:16px">${_fmtPct(item.relative_strength_pct, 1)}</div></div>
          <div class="provider-card" style="padding:10px"><div class="provider-role">Confirmacion</div><div class="provider-name" style="font-size:14px">${_esc(item?.confirmation?.higher_timeframe || '—')} · ${_esc(item?.confirmation?.direction || '—')}</div></div>
          ${item?.order_flow?.available ? `<div class="provider-card" style="padding:10px"><div class="provider-role">Order flow</div><div class="provider-name" style="font-size:14px">${_esc(item.order_flow.direction || 'neutral')} · ${_fmtPct(item.order_flow.score_pct, 1)}</div><div class="provider-role">VWAP ${_fmtPct(item.order_flow.price_vs_vwap_pct, 2)} · presión ${_fmtPct(item.order_flow.net_pressure_pct, 1)}</div></div>` : ''}
        </div>
      </div>
    `).join('');
  rejectedEl.innerHTML = rejections.length === 0
    ? `<div class="chip green">Sin rechazos recientes</div>`
    : rejections.slice(0, 8).map((item) => `<span class="chip orange">${_esc(item.symbol || '—')} ${_esc(item.timeframe || '')} · ${_esc((item.reasons || [item.reason || 'descartado'])[0])}</span>`).join(' ');
  activityEl.innerHTML = activity.length === 0
    ? `<div class="empty-state" style="padding:16px 0"><div class="empty-sub">Sin actividad registrada todavia</div></div>`
    : activity.slice().reverse().slice(0, 18).map((item) => `
      <div style="padding:8px 10px;border-bottom:1px solid var(--border);font-size:12px">
        <div style="display:flex;align-items:center;justify-content:space-between;gap:8px;flex-wrap:wrap">
          <div style="display:flex;gap:8px;align-items:center;flex-wrap:wrap">
            <span class="chip ${item.level === 'error' ? 'red' : item.level === 'warn' ? 'orange' : 'blue'}">${_esc(item.level || 'info')}</span>
            <span>${_esc(item.message || 'evento')}</span>
          </div>
          <span style="color:var(--text-muted);font-size:11px">${_esc(item.timestamp || '')}</span>
        </div>
        <div style="font-size:11px;color:var(--text-muted);margin-top:4px">${[item.symbol, item.timeframe, item.strategy ? `metodo ${item.strategy}` : '', item.score ? `score ${_fmtNum(item.score, 1)}` : ''].filter(Boolean).map((value) => _esc(value)).join(' · ')}</div>
      </div>
    `).join('');
}

async function _fetchScannerReport(container) {
  const summaryEl = container.querySelector('#aq-scanner-summary');
  if (!summaryEl) return null;
  try {
    const d = await _fetchJsonRetry(`${QUANT_API_V2}/scanner/report?activity_limit=48`, { headers: _headers() });
    if (!d?.ok || !d?.data) throw new Error(d?.error || 'No se pudo cargar el escaner');
    _renderScannerReport(container, d.data);
    return d.data;
  } catch (e) {
    summaryEl.innerHTML = `<div class="empty-state" style="padding:12px 0"><div class="empty-title" style="color:var(--accent-red)">Escaner no disponible</div><div class="empty-sub">${_esc(e.message || 'Error')}</div></div>`;
    return null;
  }
}

async function _saveScannerConfig(container) {
  const payload = _buildScannerConfig(container);
  const r = await fetch(`${QUANT_API_V2}/scanner/config`, {
    method: 'POST',
    headers: _headers(),
    body: JSON.stringify(payload),
  });
  const d = await r.json();
  if (!d?.ok || !d?.data) throw new Error(d?.error || 'No se pudo guardar la configuracion del escaner');
  _renderScannerReport(container, d.data);
  return d.data;
}

async function _controlScanner(container, action) {
  const r = await fetch(`${QUANT_API_V2}/scanner/control`, {
    method: 'POST',
    headers: _headers(),
    body: JSON.stringify({ action }),
  });
  const d = await r.json();
  if (!d?.ok || !d?.data) throw new Error(d?.error || 'No se pudo controlar el escaner');
  _renderScannerReport(container, d.data);
  return d.data;
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
    stop(SCANNER_POLL_ID);
    _setFeedState(container, 'live');
  });

  socket.addEventListener('message', (event) => {
    if (LIVE_SOCKET !== socket || ACTIVE_CONTAINER !== container) return;
    try {
      const message = JSON.parse(event.data || '{}');
      if (message.type === 'quant.live_update') {
        if (message.status) _renderStatusSummary(container, message.status);
        if (message.monitor_summary) _renderMonitorSummary(container, message.monitor_summary);
        if (message.operation_status) _renderOperationStatus(container, message.operation_status);
        if (message.scanner_report) _renderScannerReport(container, message.scanner_report);
        _setFeedState(container, 'live', message.generated_at || '');
        return;
      }
      if (message.type === 'quant.operation_update') {
        if (message.operation_status) _renderOperationStatus(container, message.operation_status);
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
            Inicio
          </button>
          <h2>Atlas Code-Quant</h2>
          <div style="margin-left:auto;display:flex;align-items:center;gap:8px">
            <span class="chip blue" style="font-size:10px">v0.1.0</span>
            <span class="live-badge">EN LINEA</span>
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
              <div class="stat-card-label">Tiempo activo</div>
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
            <button class="action-btn" id="aq-btn-scanner-view">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="11" cy="11" r="7"/><line x1="21" y1="21" x2="16.65" y2="16.65"/><path d="M11 8v6M8 11h6"/></svg>
              Ver escaner
            </button>
            <button class="action-btn" id="aq-btn-journal">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M12 20V10"/><path d="M18 20V4"/><path d="M6 20v-6"/></svg>
              Actualizar diario
            </button>
            <span class="chip orange" id="aq-live-feed">REST de respaldo</span>
            <a href="${QUANT_API}/docs" target="_blank" class="action-btn" style="text-decoration:none">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M14 2H6a2 2 0 00-2 2v16a2 2 0 002 2h12a2 2 0 002-2V8z"/><path d="M14 2v6h6"/></svg>
              Documentacion API
            </a>
          </div>

          <div class="approval-card" style="padding:12px;margin-bottom:20px;background:rgba(255,255,255,0.02)">
            <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center">
              <span class="chip blue">Espacios de trabajo</span>
              <button class="action-btn primary" data-quant-view-btn="centro">Centro</button>
              <button class="action-btn" data-quant-view-btn="scanner">Scanner</button>
              <button class="action-btn" data-quant-view-btn="selector">Selector IA</button>
              <button class="action-btn" data-quant-view-btn="operacion">Operacion</button>
              <button class="action-btn" data-quant-view-btn="diario">Diario</button>
              <button class="action-btn" data-quant-view-btn="real">Cuenta real</button>
              <button class="action-btn" data-quant-view-btn="ejecucion">Ejecucion</button>
              <a href="/quant-ui" target="_blank" class="action-btn" style="display:inline-flex;align-items:center;gap:5px;text-decoration:none;background:rgba(0,212,170,0.12);border-color:rgba(0,212,170,0.4);color:#00d4aa;margin-left:auto">
                <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="22 12 18 12 15 21 9 3 6 12 2 12"/></svg>
                Dashboard Gr&#225;fico &#8599;
              </a>
            </div>
          </div>

          <div data-quant-view="centro">
          <div class="section-title" style="margin-bottom:10px">
            Monitor avanzado
            <span class="chip accent" style="font-size:10px;margin-left:6px">Tradier</span>
          </div>
          <div class="approval-card" style="padding:14px;margin-bottom:20px">
            <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-bottom:12px">
              <select id="aq-account-scope" class="config-input" style="width:120px">
                <option value="paper" selected>Simulada</option>
                <option value="live">Real</option>
              </select>
              <input id="aq-account-id" class="config-input" placeholder="ID de cuenta (opcional)" style="width:220px">
              <div id="aq-monitor-alerts" style="display:flex;gap:8px;flex-wrap:wrap"></div>
            </div>
            <div id="aq-monitor-summary">
              <div style="padding:16px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
            </div>
            <div id="aq-monitor-list" style="margin-top:14px"></div>
          </div>
          </div>

          <div data-quant-view="scanner">
          <div class="section-title" style="margin-bottom:10px" id="aq-scanner-section">
            Escaner permanente de oportunidades
            <span class="chip green" style="font-size:10px;margin-left:6px">paper first</span>
          </div>
          <div class="approval-card" style="padding:14px;margin-bottom:20px">
            <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center;justify-content:space-between;margin-bottom:12px">
              <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center">
                <button class="action-btn" id="aq-btn-scanner-refresh">Actualizar escaner</button>
                <button class="action-btn" id="aq-btn-scanner-save">Guardar configuracion</button>
                <button class="action-btn" id="aq-btn-scanner-run">Ciclo ahora</button>
                <button class="action-btn" id="aq-btn-scanner-start">Iniciar</button>
                <button class="action-btn" id="aq-btn-scanner-stop">Detener</button>
              </div>
              <div id="aq-scanner-rejections" style="display:flex;gap:8px;flex-wrap:wrap"></div>
            </div>
            <div style="display:grid;grid-template-columns:repeat(5,minmax(0,1fr));gap:8px;margin-bottom:12px">
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Activo</div>
                <label style="display:flex;align-items:center;gap:6px;font-size:12px;color:var(--text-muted)">
                  <input id="aq-scanner-enabled" type="checkbox" checked>
                  escaner habilitado
                </label>
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Fuente</div>
                <select id="aq-scanner-source" class="config-input" style="width:100%">
                  <option value="yfinance" selected>yfinance</option>
                  <option value="ccxt">ccxt</option>
                </select>
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Modo de universo</div>
                <select id="aq-scanner-universe-mode" class="config-input" style="width:100%">
                  <option value="us_equities_rotating" selected>acciones USA rotativas</option>
                  <option value="manual">manual</option>
                </select>
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Intervalo (s)</div>
                <input id="aq-scanner-interval" class="config-input" type="number" min="15" max="3600" step="5" value="180" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Min fuerza</div>
                <input id="aq-scanner-min-strength" class="config-input" type="number" min="0" max="1" step="0.01" value="0.55" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Min win rate %</div>
                <input id="aq-scanner-min-win" class="config-input" type="number" min="0" max="100" step="0.1" value="53" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Min score</div>
                <input id="aq-scanner-min-score" class="config-input" type="number" min="75" max="100" step="0.1" value="75" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Max candidatos</div>
                <input id="aq-scanner-max-candidates" class="config-input" type="number" min="1" max="50" step="1" value="8" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Lote rotativo</div>
                <input id="aq-scanner-batch-size" class="config-input" type="number" min="20" max="500" step="10" value="80" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Prefiltro diario</div>
                <input id="aq-scanner-prefilter-count" class="config-input" type="number" min="8" max="80" step="1" value="20" style="width:100%">
              </div>
              <div style="grid-column:span 2">
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Temporalidades</div>
                <input id="aq-scanner-timeframes" class="config-input" value="5m, 15m, 1h, 4h, 1d" style="width:100%">
              </div>
              <div style="display:flex;align-items:flex-end">
                <label style="display:flex;align-items:center;gap:6px;font-size:12px;color:var(--text-muted)">
                  <input id="aq-scanner-confirm-higher" type="checkbox" checked>
                  exigir confirmacion superior
                </label>
              </div>
            </div>
            <div style="margin-bottom:12px">
              <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Universo a barrer</div>
              <textarea id="aq-scanner-universe" class="config-input" style="width:100%;min-height:54px;resize:vertical">SPY, QQQ, IWM, AAPL, MSFT, NVDA, AMZN, META, AMD, TSLA</textarea>
              <div id="aq-scanner-universe-hint" style="font-size:11px;color:var(--text-muted);margin-top:6px">Modo mercado total: ATLAS rota por el universo completo, prefiltra y profundiza solo sobre los mas prometedores.</div>
            </div>
            <div style="margin-bottom:12px">
              <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Notas del escaner</div>
              <input id="aq-scanner-notes" class="config-input" value="fase paper: escaner explicable y no ejecutor" style="width:100%">
            </div>
            <div style="margin-bottom:12px;padding:12px;border:1px solid var(--border);border-radius:12px;background:rgba(255,255,255,0.02)">
              <div style="display:flex;align-items:flex-start;justify-content:space-between;gap:12px;flex-wrap:wrap">
                <div>
                  <div class="section-title" style="margin-bottom:8px;font-size:15px">Puente escaner -> operacion</div>
                  <div id="aq-scanner-bridge-status">
                    <div class="empty-state" style="padding:8px 0"><div class="empty-sub">Esperando la primera oportunidad valida</div></div>
                  </div>
                </div>
                <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:flex-end">
                  <div>
                    <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Plantilla operativa</div>
                    <select id="aq-scanner-order-template" class="config-input" style="width:220px">
                      <option value="auto" selected>automatico segun setup</option>
                      <option value="simple">opcion simple</option>
                      <option value="debit_spread">spread de debito</option>
                      <option value="credit_spread">spread de credito</option>
                    </select>
                  </div>
                  <button class="action-btn" id="aq-btn-scanner-select-best">Preparar selector IA</button>
                  <button class="action-btn" id="aq-btn-scanner-load-best">Cargar mejor idea</button>
                  <button class="action-btn" id="aq-btn-scanner-evaluate-best">Evaluar mejor idea</button>
                  <button class="action-btn" id="aq-btn-scanner-preview-best">Previsualizar mejor idea</button>
                </div>
              </div>
            </div>
            <div id="aq-scanner-summary">
              <div style="padding:16px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
            </div>
            <div style="margin-top:14px">
              <div class="section-title" style="margin-bottom:10px;font-size:15px">Lo que esta buscando</div>
              <div id="aq-scanner-criteria" style="display:grid;grid-template-columns:repeat(auto-fill,minmax(220px,1fr));gap:8px"></div>
            </div>
            <div style="margin-top:14px">
              <div class="section-title" style="margin-bottom:10px;font-size:15px">Activos seleccionados</div>
              <div id="aq-scanner-candidates">
                <div style="padding:16px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
              </div>
            </div>
            <div style="margin-top:14px">
              <div class="section-title" style="margin-bottom:10px;font-size:15px">Bitacora del escaner</div>
              <div id="aq-scanner-activity" style="border:1px solid var(--border);border-radius:12px;background:rgba(255,255,255,0.02);overflow:hidden">
                <div style="padding:16px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
              </div>
            </div>
          </div>
          </div>

          <div data-quant-view="selector">
          <div class="section-title" style="margin-bottom:10px" id="aq-selector-section">
            Selector IA de estrategia
            <span class="chip blue" style="font-size:10px;margin-left:6px">decision + graficos + camara</span>
          </div>
          <div class="approval-card" style="padding:14px;margin-bottom:20px">
            <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center;justify-content:space-between;margin-bottom:12px">
              <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center">
                <button class="action-btn" id="aq-btn-selector-best">Preparar desde mejor idea</button>
                <button class="action-btn" id="aq-btn-selector-open-charts">Abrir graficos</button>
                <button class="action-btn" id="aq-btn-selector-ticket">Cargar al ticket</button>
                <button class="action-btn primary" id="aq-btn-selector-validate">Validar con camara</button>
              </div>
              <div id="aq-selector-state" style="display:flex;gap:8px;flex-wrap:wrap"></div>
            </div>
            <div style="display:grid;grid-template-columns:repeat(5,minmax(0,1fr));gap:8px;margin-bottom:12px">
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Proveedor de graficos</div>
                <select id="aq-selector-chart-provider" class="config-input" style="width:100%">
                  <option value="tradingview" selected>tradingview web</option>
                  <option value="yahoo">yahoo finance</option>
                </select>
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Riesgo por idea %</div>
                <input id="aq-selector-risk-budget" class="config-input" type="number" min="0.25" max="2" step="0.05" value="0.75" style="width:100%">
              </div>
              <div style="display:flex;align-items:flex-end">
                <label style="display:flex;align-items:center;gap:6px;font-size:12px;color:var(--text-muted)">
                  <input id="aq-selector-defined-risk" type="checkbox" checked>
                  priorizar riesgo definido
                </label>
              </div>
              <div style="display:flex;align-items:flex-end">
                <label style="display:flex;align-items:center;gap:6px;font-size:12px;color:var(--text-muted)">
                  <input id="aq-selector-allow-equity" type="checkbox" checked>
                  permitir acciones
                </label>
              </div>
              <div style="display:flex;align-items:flex-end">
                <label style="display:flex;align-items:center;gap:6px;font-size:12px;color:var(--text-muted)">
                  <input id="aq-selector-allow-credit" type="checkbox" checked>
                  permitir credito
                </label>
              </div>
            </div>
            <div style="display:flex;gap:12px;flex-wrap:wrap;align-items:center;margin-bottom:12px">
              <label style="display:flex;align-items:center;gap:6px;font-size:12px;color:var(--text-muted)">
                <input id="aq-selector-open-charts" type="checkbox">
                abrir graficos tras preparar
              </label>
              <span class="chip blue">usa el balance real de la cuenta seleccionada</span>
              <span class="chip accent">elige estructura + sizing + salida</span>
              <span class="chip green">prepara la validacion visual con la camara</span>
            </div>
            <div id="aq-selector-summary">
              <div class="empty-state" style="padding:16px 0"><div class="empty-sub">Sin propuesta del selector todavia</div></div>
            </div>
          </div>
          </div>

          <div data-quant-view="operacion">
          <div class="section-title" style="margin-bottom:10px" id="aq-operation-section">
            ATLAS Operacional
            <span class="chip green" style="font-size:10px;margin-left:6px">fase simulada</span>
          </div>
          <div class="approval-card" style="padding:14px;margin-bottom:20px">
            <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center;justify-content:space-between;margin-bottom:12px">
              <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center">
                <button class="action-btn" id="aq-btn-op-refresh">
                  <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M21 12a9 9 0 1 1-2.64-6.36"/><polyline points="21 3 21 9 15 9"/></svg>
                  Actualizar operacion
                </button>
                <button class="action-btn" id="aq-btn-op-save">Guardar plano de control</button>
                <button class="action-btn" id="aq-btn-op-evaluate">Probar ciclo</button>
                <button class="action-btn" id="aq-btn-op-preview">Previsualizar simulada</button>
                <button class="action-btn primary" id="aq-btn-op-submit">Ejecutar en simulada</button>
                <button class="action-btn" id="aq-btn-op-stop">Parada total</button>
                <button class="action-btn" id="aq-btn-op-reset">Reactivar control</button>
              </div>
              <div id="aq-op-state" style="display:flex;gap:8px;flex-wrap:wrap"></div>
            </div>
            <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-bottom:12px">
              <span class="chip blue">Perfiles simulados</span>
              <button class="action-btn" data-op-profile="paper_safe">simulada segura</button>
              <button class="action-btn primary" data-op-profile="paper_supervised">simulada supervisada</button>
              <button class="action-btn" data-op-profile="paper_autonomous">simulada autonoma</button>
            </div>
            <div id="aq-op-phase-note" style="display:flex;gap:8px;flex-wrap:wrap;margin-bottom:12px"></div>
            <div style="display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;margin-bottom:12px">
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Cuenta</div>
                <select id="aq-op-scope" class="config-input" style="width:100%">
                  <option value="paper" selected>simulada</option>
                  <option value="live">real</option>
                </select>
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Modo autonomo</div>
                <select id="aq-op-auton-mode" class="config-input" style="width:100%">
                  <option value="off">apagado</option>
                  <option value="paper_supervised" selected>simulada supervisada</option>
                  <option value="paper_autonomous">simulada autonoma</option>
                </select>
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Ejecutor</div>
                <select id="aq-op-executor-mode" class="config-input" style="width:100%">
                  <option value="disabled">desactivado</option>
                  <option value="paper_api" selected>API simulada</option>
                  <option value="desktop_dry_run">simulacion de escritorio</option>
                </select>
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Vision</div>
                <select id="aq-op-vision-mode" class="config-input" style="width:100%">
                  <option value="off">apagada</option>
                  <option value="manual">manual</option>
                  <option value="desktop_capture">captura de escritorio</option>
                  <option value="direct_nexus" selected>Insta360 activa por robot</option>
                  <option value="atlas_push_bridge">Insta360 via puente PUSH</option>
                </select>
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Min. tasa de exito</div>
                <input id="aq-op-min-win-rate" class="config-input" type="number" value="65" min="0" max="100" step="0.1" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Max BPR % L4</div>
                <input id="aq-op-max-bpr" class="config-input" type="number" value="20" min="0" max="100" step="0.1" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Limite de errores</div>
                <input id="aq-op-failsafe-limit" class="config-input" type="number" value="3" min="1" max="10" step="1" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Sentimiento</div>
                <input id="aq-op-sentiment-score" class="config-input" type="number" value="0" min="-1" max="1" step="0.05" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Notas</div>
                <input id="aq-op-notes" class="config-input" placeholder="plano de control simulado" style="width:100%">
              </div>
            </div>
            <div style="display:flex;gap:14px;flex-wrap:wrap;align-items:center;margin-bottom:12px">
              <label style="display:flex;align-items:center;gap:6px;font-size:12px;color:var(--text-muted)">
                <input id="aq-op-paper-only" type="checkbox" checked>
                solo simulada
              </label>
              <label style="display:flex;align-items:center;gap:6px;font-size:12px;color:var(--text-muted)">
                <input id="aq-op-require-operator" type="checkbox">
                exigir operador
              </label>
              <label style="display:flex;align-items:center;gap:6px;font-size:12px;color:var(--text-muted)">
                <input id="aq-op-operator-present" type="checkbox" checked>
                operador presente
              </label>
              <label style="display:flex;align-items:center;gap:6px;font-size:12px;color:var(--text-muted)">
                <input id="aq-op-screen-ok" type="checkbox" checked>
                pantallas OK
              </label>
              <label style="display:flex;align-items:center;gap:6px;font-size:12px;color:var(--text-muted)">
                <input id="aq-op-auto-pause-errors" type="checkbox" checked>
                auto pausa por errores
              </label>
            </div>
            <div class="approval-card" style="padding:12px;margin-bottom:12px;background:rgba(13,23,45,.45);border:1px solid rgba(61,205,255,.12)">
              <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center;justify-content:space-between;margin-bottom:10px">
                <div class="section-title" style="font-size:14px;margin:0">Calibracion de camara</div>
                <div id="aq-calib-state" style="display:flex;gap:8px;flex-wrap:wrap"></div>
              </div>
              <div style="display:grid;grid-template-columns:repeat(6,minmax(0,1fr));gap:8px;margin-bottom:10px">
                <div>
                  <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Ancho monitor</div>
                  <input id="aq-calib-width" class="config-input" type="number" value="1920" min="1" step="1" style="width:100%">
                </div>
                <div>
                  <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Alto monitor</div>
                  <input id="aq-calib-height" class="config-input" type="number" value="1080" min="1" step="1" style="width:100%">
                </div>
                <div>
                  <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Filas</div>
                  <input id="aq-calib-rows" class="config-input" type="number" value="3" min="1" max="9" step="1" style="width:100%">
                </div>
                <div>
                  <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Columnas</div>
                  <input id="aq-calib-cols" class="config-input" type="number" value="3" min="1" max="9" step="1" style="width:100%">
                </div>
                <div>
                  <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Zoom base</div>
                  <input id="aq-calib-grid-zoom" class="config-input" type="number" value="1" min="1" max="4" step="0.1" style="width:100%">
                </div>
                <div>
                  <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Etiqueta</div>
                  <input id="aq-calib-label" class="config-input" value="calibracion_principal" style="width:100%">
                </div>
              </div>
              <div style="display:grid;grid-template-columns:repeat(6,minmax(0,1fr));gap:8px;margin-bottom:10px">
                <div>
                  <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Punto</div>
                  <select id="aq-calib-point-id" class="config-input" style="width:100%">
                    <option value="">siguiente punto sugerido</option>
                  </select>
                </div>
                <div>
                  <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Yaw</div>
                  <input id="aq-calib-yaw" class="config-input" type="number" value="0" min="-1" max="1" step="0.0001" style="width:100%">
                </div>
                <div>
                  <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Pitch</div>
                  <input id="aq-calib-pitch" class="config-input" type="number" value="0" min="-1" max="1" step="0.0001" style="width:100%">
                </div>
                <div>
                  <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Zoom actual</div>
                  <input id="aq-calib-zoom" class="config-input" type="number" value="1" min="1" max="4" step="0.1" style="width:100%">
                </div>
                <div>
                  <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Nota</div>
                  <input id="aq-calib-note" class="config-input" placeholder="muestra actual" style="width:100%">
                </div>
                <div>
                  <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Ruta guardado</div>
                  <input id="aq-calib-save-path" class="config-input" placeholder="config/screen_gaze_calibration.json" style="width:100%">
                </div>
              </div>
              <div style="display:flex;gap:12px;flex-wrap:wrap;align-items:center;margin-bottom:10px">
                <label style="display:flex;align-items:center;gap:6px;font-size:12px;color:var(--text-muted)">
                  <input id="aq-calib-use-current" type="checkbox" checked>
                  usar pose actual del cuello
                </label>
                <button class="action-btn" id="aq-btn-calib-start">Iniciar grilla</button>
                <button class="action-btn" id="aq-btn-calib-move">Mover cuello</button>
                <button class="action-btn" id="aq-btn-calib-sample">Guardar muestra</button>
                <button class="action-btn primary" id="aq-btn-calib-fit">Ajustar y guardar</button>
                <button class="action-btn" id="aq-btn-calib-reset">Reiniciar sesion</button>
              </div>
              <div id="aq-calib-points"></div>
            </div>
            <div id="aq-op-summary">
              <div style="padding:16px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
            </div>
            <div id="aq-op-strategies" style="margin-top:14px"></div>
            <div id="aq-op-result" style="margin-top:14px"></div>
          </div>
          </div>

          <div data-quant-view="diario">
          <div class="section-title" style="margin-bottom:10px">
            Diario de trading
            <span class="chip blue" style="font-size:10px;margin-left:6px">Real vs simulada</span>
          </div>
          <div class="approval-card" style="padding:14px;margin-bottom:20px">
            <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center;justify-content:space-between;margin-bottom:12px">
              <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center">
                <button class="action-btn" id="aq-btn-journal-sync">
                  <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M21 12a9 9 0 1 1-2.64-6.36"/><polyline points="21 3 21 9 15 9"/></svg>
                  Sincronizar diario
                </button>
                <div id="aq-journal-status" style="display:flex;gap:8px;flex-wrap:wrap"></div>
              </div>
            </div>
            <div id="aq-journal-summary">
              <div style="padding:16px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
            </div>
            <div id="aq-journal-accounts" style="margin-top:14px"></div>
            <div id="aq-journal-heatmaps" style="margin-top:14px"></div>
            <div style="margin-top:14px">
              <div class="section-title" style="margin-bottom:10px;font-size:15px">Entradas recientes</div>
              <div id="aq-journal-entries">
                <div style="padding:16px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
              </div>
            </div>
          </div>
          </div>

          <div data-quant-view="real">
          <div class="section-title" style="margin-bottom:10px">
            Cuenta real
            <span class="chip orange" style="font-size:10px;margin-left:6px">predisenada y bloqueada</span>
          </div>
          <div class="approval-card" style="padding:14px;margin-bottom:20px">
            <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center;justify-content:space-between;margin-bottom:12px">
              <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center">
                <span class="chip red">ejecucion real deshabilitada</span>
                <span class="chip blue">fase 2 de producto</span>
                <span class="chip orange">paper sigue siendo la fuente activa de prueba</span>
              </div>
              <div style="display:flex;gap:8px;flex-wrap:wrap">
                <button class="action-btn" disabled style="opacity:.65;cursor:not-allowed">Abrir consola live</button>
                <button class="action-btn" disabled style="opacity:.65;cursor:not-allowed">Activar gate real</button>
                <button class="action-btn" disabled style="opacity:.65;cursor:not-allowed">Enviar orden real</button>
              </div>
            </div>
            <div style="display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;margin-bottom:14px">
              <div class="stat-card">
                <div class="stat-card-label">Estado live</div>
                <div class="stat-card-value orange" style="font-size:15px">Preconfigurado</div>
                <div class="stat-card-sub">Listo para integrar sin abrir la cuenta real todavia.</div>
              </div>
              <div class="stat-card">
                <div class="stat-card-label">Risk Gate base</div>
                <div class="stat-card-value blue" style="font-size:15px">Win rate + PDT + BPR</div>
                <div class="stat-card-sub">La vista real parte con bloqueos mas estrictos que paper.</div>
              </div>
              <div class="stat-card">
                <div class="stat-card-label">Supervisor</div>
                <div class="stat-card-value green" style="font-size:15px">Humano + ATLAS</div>
                <div class="stat-card-sub">El operador valida y ATLAS documenta y vigila.</div>
              </div>
              <div class="stat-card">
                <div class="stat-card-label">Objetivo</div>
                <div class="stat-card-value" style="font-size:15px">Crecimiento gobernado</div>
                <div class="stat-card-sub">Sin mezclar investigacion, scouting y ejecucion en la misma pantalla.</div>
              </div>
            </div>

            <div style="display:grid;grid-template-columns:1.15fr .85fr;gap:12px;margin-bottom:14px">
              <div class="approval-card" style="padding:12px;background:rgba(13,23,45,.42)">
                <div class="section-title" style="font-size:14px;margin:0 0 10px">Arquitectura operativa live</div>
                <div style="display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px">
                  <div class="stat-card">
                    <div class="stat-card-label">1. Scanner</div>
                    <div class="stat-card-sub">Selecciona activos con evidencia multi temporal y descarte explicable.</div>
                  </div>
                  <div class="stat-card">
                    <div class="stat-card-label">2. Selector IA</div>
                    <div class="stat-card-sub">Decide accion, debito o credito segun efectivo, IV, riesgo y DTE.</div>
                  </div>
                  <div class="stat-card">
                    <div class="stat-card-label">3. Ticket live</div>
                    <div class="stat-card-sub">Arma entrada, lotes, salida, stop y validacion broker antes del envio.</div>
                  </div>
                  <div class="stat-card">
                    <div class="stat-card-label">4. Seguimiento</div>
                    <div class="stat-card-sub">PnL, payoff, griegas, probabilidad dinamica y alertas de deterioro.</div>
                  </div>
                  <div class="stat-card">
                    <div class="stat-card-label">5. Diario Pro</div>
                    <div class="stat-card-sub">Post-mortem, atribucion, curva de equidad y comparativa live vs paper.</div>
                  </div>
                  <div class="stat-card">
                    <div class="stat-card-label">6. Vision y auditoria</div>
                    <div class="stat-card-sub">Captura de contexto, operador, pantalla y evidencia visual de cada accion.</div>
                  </div>
                </div>
              </div>
              <div class="approval-card" style="padding:12px;background:rgba(255,255,255,0.02)">
                <div class="section-title" style="font-size:14px;margin:0 0 10px">Checklist de activacion</div>
                <div style="display:flex;flex-direction:column;gap:8px">
                  <label style="display:flex;align-items:center;gap:8px;font-size:12px;color:var(--text-muted)">
                    <input type="checkbox" checked disabled>
                    credenciales live verificadas
                  </label>
                  <label style="display:flex;align-items:center;gap:8px;font-size:12px;color:var(--text-muted)">
                    <input type="checkbox" checked disabled>
                    PDT y guardas de capital preparados
                  </label>
                  <label style="display:flex;align-items:center;gap:8px;font-size:12px;color:var(--text-muted)">
                    <input type="checkbox" disabled>
                    selector IA de estrategia listo
                  </label>
                  <label style="display:flex;align-items:center;gap:8px;font-size:12px;color:var(--text-muted)">
                    <input type="checkbox" disabled>
                    politica de lotaje y sizing final lista
                  </label>
                  <label style="display:flex;align-items:center;gap:8px;font-size:12px;color:var(--text-muted)">
                    <input type="checkbox" disabled>
                    reglas de salida automatizadas listas
                  </label>
                  <label style="display:flex;align-items:center;gap:8px;font-size:12px;color:var(--text-muted)">
                    <input type="checkbox" checked disabled>
                    camara y auditoria visual operativas
                  </label>
                </div>
              </div>
            </div>

            <div style="display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px;margin-bottom:14px">
              <div class="approval-card" style="padding:12px;background:rgba(255,255,255,0.02)">
                <div class="section-title" style="font-size:14px;margin:0 0 8px">Bloque selector IA</div>
                <div class="stat-card-sub">Vista reservada para la siguiente etapa: aqui ATLAS elegira estructura, timeframe, lotes, DTE y salida esperada.</div>
                <div style="display:flex;gap:6px;flex-wrap:wrap;margin-top:10px">
                  <span class="chip blue">accion</span>
                  <span class="chip blue">opcion simple</span>
                  <span class="chip blue">debito</span>
                  <span class="chip blue">credito</span>
                  <span class="chip blue">cobertura</span>
                </div>
              </div>
              <div class="approval-card" style="padding:12px;background:rgba(255,255,255,0.02)">
                <div class="section-title" style="font-size:14px;margin:0 0 8px">Bloque risk profile</div>
                <div class="stat-card-sub">Aqui viviran la curva de riesgo, break-even, stress por escenario y consumo de buying power antes de confirmar.</div>
                <div style="display:flex;gap:6px;flex-wrap:wrap;margin-top:10px">
                  <span class="chip orange">PDT</span>
                  <span class="chip orange">BPR</span>
                  <span class="chip orange">delta</span>
                  <span class="chip orange">theta</span>
                  <span class="chip orange">vega</span>
                </div>
              </div>
              <div class="approval-card" style="padding:12px;background:rgba(255,255,255,0.02)">
                <div class="section-title" style="font-size:14px;margin:0 0 8px">Bloque de ejecucion</div>
                <div class="stat-card-sub">El flujo real quedara en dos pasos: previsualizacion obligatoria y envio confirmado con marca live.</div>
                <div style="display:flex;gap:6px;flex-wrap:wrap;margin-top:10px">
                  <span class="chip red">confirmacion live</span>
                  <span class="chip red">doble validacion</span>
                  <span class="chip red">kill switch</span>
                </div>
              </div>
            </div>

            <div class="approval-card" style="padding:12px;background:rgba(13,23,45,.42)">
              <div class="section-title" style="font-size:14px;margin:0 0 8px">Ruta recomendada para abrir live</div>
              <div class="stat-card-sub">
                Primero se cierra la fase paper con scanner, selector IA y seguimiento consistentes. Despues esta vista pasara de maqueta gobernada a consola operativa real sin rehacer la arquitectura.
              </div>
            </div>
          </div>
          </div>

          <div data-quant-view="ejecucion">
          <div class="section-title" style="margin-bottom:10px" id="aq-order-section">
            Orden Tradier
            <span class="chip orange" style="font-size:10px;margin-left:6px">Previsualizacion por defecto</span>
          </div>
          <div class="approval-card" style="padding:14px;margin-bottom:20px">
            <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-bottom:10px">
              <select id="aq-order-preset" class="config-input" style="width:280px">
                <option value="custom" selected>Preset rapido: personalizado</option>
                <option value="equity_buy">Comprar accion</option>
                <option value="equity_sell">Vender accion</option>
                <option value="long_call">Call largo</option>
                <option value="bull_call_debit_spread">Spread call alcista de debito</option>
                <option value="bear_put_debit_spread">Spread put bajista de debito</option>
                <option value="bear_call_credit_spread">Spread call bajista de credito</option>
                <option value="bull_put_credit_spread">Spread put alcista de credito</option>
                <option value="iron_condor">Condor de hierro</option>
                <option value="iron_butterfly">Mariposa de hierro</option>
                <option value="call_calendar_spread">Calendario call</option>
                <option value="call_diagonal_debit_spread">Diagonal call de debito</option>
                <option value="covered_call">Call cubierta</option>
              </select>
              <span class="chip blue">Acciones y opciones dentro del mismo ticket</span>
            </div>
            <div style="display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;margin-bottom:10px">
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Simbolo base</div>
                <input id="aq-order-symbol" class="config-input" value="AAPL" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Clase de activo</div>
                <select id="aq-order-asset-class" class="config-input" style="width:100%">
                  <option value="option" selected>Opcion</option>
                  <option value="equity">Accion</option>
                  <option value="multileg">Multileg</option>
                  <option value="combo">Combo</option>
                  <option value="auto">Automatico</option>
                </select>
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Lado</div>
                <select id="aq-order-side" class="config-input" style="width:100%">
                  <option value="buy">compra</option>
                  <option value="sell">venta</option>
                  <option value="sell_short">venta en corto</option>
                  <option value="buy_to_cover">compra para cubrir</option>
                  <option value="buy_to_open">compra para abrir</option>
                  <option value="sell_to_open">venta para abrir</option>
                  <option value="buy_to_close">compra para cerrar</option>
                  <option value="sell_to_close">venta para cerrar</option>
                </select>
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Efecto en posicion</div>
                <select id="aq-order-position-effect" class="config-input" style="width:100%">
                  <option value="auto" selected>automatico</option>
                  <option value="open">abrir</option>
                  <option value="close">cerrar</option>
                </select>
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Simbolo OCC</div>
                <input id="aq-order-option-symbol" class="config-input" placeholder="AAPL260417C00200000" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Tamano</div>
                <input id="aq-order-size" class="config-input" type="number" value="1" min="0.01" step="0.01" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Tipo de orden</div>
                <select id="aq-order-type" class="config-input" style="width:100%">
                  <option value="market" selected>mercado</option>
                  <option value="limit">limite</option>
                  <option value="stop">stop</option>
                  <option value="stop_limit">stop limite</option>
                  <option value="debit">debito</option>
                  <option value="credit">credito</option>
                  <option value="even">par</option>
                </select>
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Duracion</div>
                <select id="aq-order-duration" class="config-input" style="width:100%">
                  <option value="day" selected>day</option>
                  <option value="gtc">gtc</option>
                  <option value="pre">pre</option>
                  <option value="post">post</option>
                </select>
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Precio</div>
                <input id="aq-order-price" class="config-input" type="number" step="0.01" placeholder="2.15" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Precio stop</div>
                <input id="aq-order-stop-price" class="config-input" type="number" step="0.01" placeholder="1.95" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Tipo de estrategia</div>
                <select id="aq-order-strategy-type" class="config-input" style="width:100%">
                  <option value="">sin filtro</option>
                  <option value="long_call">Call largo</option>
                  <option value="long_put">Put largo</option>
                  <option value="covered_call">Call cubierta</option>
                  <option value="cash_secured_put">Put garantizada en efectivo</option>
                  <option value="bull_call_debit_spread">Spread call alcista de debito</option>
                  <option value="bear_put_debit_spread">Spread put bajista de debito</option>
                  <option value="bear_call_credit_spread">Spread call bajista de credito</option>
                  <option value="bull_put_credit_spread">Spread put alcista de credito</option>
                  <option value="call_debit_butterfly">Mariposa call de debito</option>
                  <option value="put_debit_butterfly">Mariposa put de debito</option>
                  <option value="call_credit_butterfly">Mariposa call de credito</option>
                  <option value="put_credit_butterfly">Mariposa put de credito</option>
                  <option value="call_debit_condor">Condor call de debito</option>
                  <option value="put_debit_condor">Condor put de debito</option>
                  <option value="call_credit_condor">Condor call de credito</option>
                  <option value="put_credit_condor">Condor put de credito</option>
                  <option value="long_straddle">Straddle largo</option>
                  <option value="long_strangle">Strangle largo</option>
                  <option value="iron_condor">Condor de hierro</option>
                  <option value="iron_butterfly">Mariposa de hierro</option>
                  <option value="calendar_spread">Spread calendario</option>
                  <option value="call_calendar_spread">Calendario call</option>
                  <option value="put_calendar_spread">Calendario put</option>
                  <option value="call_diagonal_debit_spread">Diagonal call de debito</option>
                  <option value="put_diagonal_debit_spread">Diagonal put de debito</option>
                </select>
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Min. tasa de exito</div>
                <input id="aq-order-min-win-rate" class="config-input" type="number" value="55" min="0" max="100" step="0.1" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Etiqueta</div>
                <input id="aq-order-tag" class="config-input" placeholder="atlas-ui" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">% de riesgo por idea</div>
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
                Filtro de probabilidad
              </label>
              <span class="chip blue">Usa la cuenta activa del monitor</span>
            </div>
            <div id="aq-order-phase-note" style="display:flex;gap:8px;flex-wrap:wrap;margin-bottom:10px"></div>
            <div style="display:flex;gap:8px;flex-wrap:wrap;margin-bottom:10px">
              <button class="action-btn" id="aq-btn-order-build">Construir desde estrategia</button>
              <span class="chip accent">Consulta /probability/options y rellena contratos reales</span>
            </div>
            <div id="aq-order-proposal" style="margin-bottom:12px"></div>
            <div>
              <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Legs JSON para multileg/combo</div>
              <textarea id="aq-order-legs" class="config-input" style="width:100%;min-height:84px;resize:vertical" placeholder='[{"option_symbol":"AAPL260417C00200000","side":"buy","quantity":1}]'></textarea>
            </div>
            <div style="display:flex;gap:8px;flex-wrap:wrap;margin-top:12px">
              <button class="action-btn" id="aq-btn-order-preview">Previsualizar en Tradier</button>
              <button class="action-btn primary" id="aq-btn-order-submit">Enviar a simulada</button>
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
            Pruebas historicas
            <span class="chip orange" style="font-size:10px;margin-left:6px">Simulación</span>
          </div>
          <div class="approval-card" style="padding:14px;margin-bottom:20px">
            <div style="display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-bottom:10px">
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Símbolo</div>
                <input id="aq-bt-symbol" class="config-input" value="BTC/USDT" style="width:100%">
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Marco temporal</div>
                <select id="aq-bt-tf" class="config-input" style="width:100%">
                  <option>1h</option><option>4h</option><option>1d</option>
                </select>
              </div>
              <div>
                <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Estrategia</div>
                <select id="aq-bt-strategy" class="config-input" style="width:100%">
                  <option value="ma_cross">Cruce de medias</option>
                  <option value="ml_rf">ML (Bosque aleatorio)</option>
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
            <span class="live-badge">EN LINEA</span>
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
              ['⚙️','strategies/','Dirigido por eventos'],
              ['💼','execution/','Portafolio y riesgo'],
              ['🌐','api/','REST :8792'],
              ['🔬','backtesting/','Motor de backtest'],
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
      </div>
    `;

    container.querySelectorAll('[data-quant-view-btn]').forEach((btn) => {
      btn.addEventListener('click', () => {
        _activateQuantView(container, btn.getAttribute('data-quant-view-btn') || 'scanner');
      });
    });

    // toggle panel señal
    container.querySelector('#aq-btn-signal')?.addEventListener('click', () => {
      const p = container.querySelector('#aq-signal-panel');
      if (p) p.style.display = p.style.display === 'none' ? 'block' : 'none';
      _activateQuantView(container, 'ejecucion');
    });

    // actualizar posiciones manual
    container.querySelector('#aq-btn-positions')?.addEventListener('click', () => {
      _activateQuantView(container, 'ejecucion');
      _fetchPositions(container);
      _fetchMonitorSummary(container);
      _refreshJournalPanel(container);
      window.AtlasToast?.show('Posiciones actualizadas', 'info');
    });

    container.querySelector('#aq-btn-monitor')?.addEventListener('click', () => {
      _activateQuantView(container, 'centro');
      _fetchHealth(container);
      _fetchMonitorSummary(container);
      _fetchScannerReport(container);
      _refreshJournalPanel(container);
      window.AtlasToast?.show('Monitor avanzado actualizado', 'info');
    });

    container.querySelector('#aq-btn-scanner-view')?.addEventListener('click', () => {
      _activateQuantView(container, 'scanner');
      container.querySelector('#aq-scanner-section')?.scrollIntoView({ behavior: 'smooth', block: 'start' });
    });

    container.querySelector('#aq-btn-scanner-refresh')?.addEventListener('click', async () => {
      await _fetchScannerReport(container);
      window.AtlasToast?.show('Escaner actualizado', 'info');
    });

    container.querySelector('#aq-btn-scanner-save')?.addEventListener('click', async () => {
      try {
        await _saveScannerConfig(container);
        window.AtlasToast?.show('Configuracion del escaner guardada', 'success');
      } catch (e) {
        window.AtlasToast?.show(e.message || 'No se pudo guardar el escaner', 'error');
      }
    });

    container.querySelector('#aq-btn-scanner-run')?.addEventListener('click', async () => {
      try {
        await _saveScannerConfig(container);
        await _controlScanner(container, 'run_once');
        window.AtlasToast?.show('Ciclo del escaner ejecutado', 'success');
      } catch (e) {
        window.AtlasToast?.show(e.message || 'No se pudo ejecutar el ciclo del escaner', 'error');
      }
    });

    container.querySelector('#aq-btn-scanner-start')?.addEventListener('click', async () => {
      try {
        await _saveScannerConfig(container);
        await _controlScanner(container, 'start');
        window.AtlasToast?.show('Escaner iniciado', 'success');
      } catch (e) {
        window.AtlasToast?.show(e.message || 'No se pudo iniciar el escaner', 'error');
      }
    });

    container.querySelector('#aq-btn-scanner-stop')?.addEventListener('click', async () => {
      try {
        await _controlScanner(container, 'stop');
        window.AtlasToast?.show('Escaner detenido', 'warning');
      } catch (e) {
        window.AtlasToast?.show(e.message || 'No se pudo detener el escaner', 'error');
      }
    });

    container.querySelector('#aq-scanner-order-template')?.addEventListener('change', () => {
      _renderScannerBridgeStatus(container, container.__atlasQuantScanner || {});
    });

    container.querySelector('#aq-scanner-universe-mode')?.addEventListener('change', () => {
      _syncScannerUniverseModeUI(container);
    });

    container.querySelector('#aq-btn-scanner-select-best')?.addEventListener('click', async () => {
      try {
        const best = container.__atlasQuantScanner?.candidates?.[0];
        await _fetchSelectorProposal(container, best, { autoOpenCharts: _selectorConfig(container).open_charts_after_prepare });
        container.querySelector('#aq-selector-section')?.scrollIntoView({ behavior: 'smooth', block: 'start' });
        window.AtlasToast?.show('Selector IA preparado desde la mejor idea', 'success');
      } catch (e) {
        _renderSelectorSummary(container, null, e.message || 'No se pudo preparar el selector');
        window.AtlasToast?.show(e.message || 'No se pudo preparar el selector', 'error');
      }
    });

    container.querySelector('#aq-btn-scanner-load-best')?.addEventListener('click', async () => {
      try {
        const best = container.__atlasQuantScanner?.candidates?.[0];
        await _bridgeScannerCandidate(container, best, 'load');
      } catch (e) {
        window.AtlasToast?.show(e.message || 'No se pudo cargar la mejor idea del escaner', 'error');
      }
    });

    container.querySelector('#aq-btn-scanner-evaluate-best')?.addEventListener('click', async () => {
      try {
        const best = container.__atlasQuantScanner?.candidates?.[0];
        await _bridgeScannerCandidate(container, best, 'evaluate');
      } catch (e) {
        window.AtlasToast?.show(e.message || 'No se pudo evaluar la mejor idea del escaner', 'error');
      }
    });

    container.querySelector('#aq-btn-scanner-preview-best')?.addEventListener('click', async () => {
      try {
        const best = container.__atlasQuantScanner?.candidates?.[0];
        await _bridgeScannerCandidate(container, best, 'preview');
      } catch (e) {
        window.AtlasToast?.show(e.message || 'No se pudo previsualizar la mejor idea del escaner', 'error');
      }
    });

    container.addEventListener('click', async (event) => {
      const btn = event.target?.closest?.('[data-scanner-bridge-action]');
      if (!btn || !container.contains(btn)) return;
      try {
        const index = Number(btn.getAttribute('data-scanner-index') || '-1');
        const action = btn.getAttribute('data-scanner-bridge-action') || 'load';
        const candidate = Array.isArray(container.__atlasQuantScanner?.candidates)
          ? container.__atlasQuantScanner.candidates[index]
          : null;
        if (action === 'selector') {
          await _fetchSelectorProposal(container, candidate, { autoOpenCharts: _selectorConfig(container).open_charts_after_prepare });
          container.querySelector('#aq-selector-section')?.scrollIntoView({ behavior: 'smooth', block: 'start' });
          window.AtlasToast?.show('Idea enviada al selector IA', 'success');
          return;
        }
        await _bridgeScannerCandidate(container, candidate, action);
      } catch (e) {
        window.AtlasToast?.show(e.message || 'No se pudo usar la idea del escaner', 'error');
      }
    });

    container.querySelector('#aq-btn-selector-best')?.addEventListener('click', async () => {
      try {
        const best = container.__atlasQuantScanner?.candidates?.[0];
        await _fetchSelectorProposal(container, best, { autoOpenCharts: _selectorConfig(container).open_charts_after_prepare });
        window.AtlasToast?.show('Selector IA actualizado desde la mejor idea', 'success');
      } catch (e) {
        _renderSelectorSummary(container, null, e.message || 'No se pudo actualizar el selector');
        window.AtlasToast?.show(e.message || 'No se pudo actualizar el selector', 'error');
      }
    });

    container.querySelector('#aq-btn-selector-open-charts')?.addEventListener('click', () => {
      try {
        const opened = _openSelectorCharts(container);
        window.AtlasToast?.show(`Graficos abiertos: ${opened}`, 'success');
      } catch (e) {
        window.AtlasToast?.show(e.message || 'No se pudieron abrir los graficos', 'error');
      }
    });

    container.querySelector('#aq-btn-selector-ticket')?.addEventListener('click', async () => {
      try {
        await _applySelectorToTicket(container, { runBuilder: true });
        container.querySelector('#aq-order-section')?.scrollIntoView({ behavior: 'smooth', block: 'start' });
        window.AtlasToast?.show('Propuesta cargada al ticket', 'success');
      } catch (e) {
        window.AtlasToast?.show(e.message || 'No se pudo cargar el ticket desde el selector', 'error');
      }
    });

    container.querySelector('#aq-btn-selector-validate')?.addEventListener('click', async () => {
      try {
        await _selectorValidateWithCamera(container);
        container.querySelector('#aq-operation-section')?.scrollIntoView({ behavior: 'smooth', block: 'start' });
        window.AtlasToast?.show('Selector validado con camara y plano operacional', 'success');
      } catch (e) {
        window.AtlasToast?.show(e.message || 'No se pudo validar con camara', 'error');
      }
    });

    container.querySelector('#aq-btn-journal')?.addEventListener('click', () => {
      _activateQuantView(container, 'diario');
      _refreshJournalPanel(container);
      window.AtlasToast?.show('Journal actualizado', 'info');
    });

    container.querySelector('#aq-btn-journal-sync')?.addEventListener('click', async () => {
      try {
        await _syncJournal(container);
        window.AtlasToast?.show('Diario sincronizado con Tradier', 'success');
      } catch (e) {
        window.AtlasToast?.show(e.message || 'No se pudo sincronizar el diario', 'error');
      }
    });

    container.querySelector('#aq-btn-op-refresh')?.addEventListener('click', () => {
      _fetchOperationStatus(container);
      window.AtlasToast?.show('Operacion actualizada', 'info');
    });

    container.querySelector('#aq-btn-op-save')?.addEventListener('click', async () => {
      try {
        await _saveOperationConfig(container);
        window.AtlasToast?.show('Plano de control guardado', 'success');
      } catch (e) {
        window.AtlasToast?.show(e.message || 'No se pudo guardar configuracion operacional', 'error');
      }
    });

    container.querySelectorAll('[data-op-profile]').forEach((btn) => {
      btn.addEventListener('click', async () => {
        try {
          const profileId = btn.getAttribute('data-op-profile');
          if (!_applyOperationProfile(container, profileId)) return;
          await _saveOperationConfig(container);
          _fetchHealth(container);
          _fetchMonitorSummary(container);
          _connectLiveUpdates(container);
          window.AtlasToast?.show(`Perfil ${_labelProfile(profileId)} cargado`, 'success');
        } catch (e) {
          window.AtlasToast?.show(e.message || 'No se pudo cargar el perfil simulado', 'error');
        }
      });
    });

    container.querySelector('#aq-btn-op-evaluate')?.addEventListener('click', async () => {
      try {
        await _saveOperationConfig(container);
        await _runOperationCycle(container, 'evaluate');
        window.AtlasToast?.show('Ciclo operacional evaluado', 'success');
      } catch (e) {
        _renderOperationResult(container.querySelector('#aq-op-result'), null, e.message || 'No se pudo evaluar el ciclo');
        window.AtlasToast?.show(e.message || 'No se pudo evaluar el ciclo', 'error');
      }
    });

    container.querySelector('#aq-btn-op-preview')?.addEventListener('click', async () => {
      try {
        await _saveOperationConfig(container);
        await _runOperationCycle(container, 'preview');
        window.AtlasToast?.show('Previsualizacion simulada enviada desde el plano de control', 'success');
      } catch (e) {
        _renderOperationResult(container.querySelector('#aq-op-result'), null, e.message || 'No se pudo lanzar la previsualizacion simulada');
        window.AtlasToast?.show(e.message || 'No se pudo lanzar la previsualizacion simulada', 'error');
      }
    });

    container.querySelector('#aq-btn-op-submit')?.addEventListener('click', async () => {
      try {
        await _saveOperationConfig(container);
        const scope = container.querySelector('#aq-op-scope')?.value || 'paper';
        const ok = window.confirm(`Enviar a ${_labelScope(scope)} desde ATLAS Operacional usando el ticket actual?`);
        if (!ok) return;
        await _runOperationCycle(container, 'submit');
        window.AtlasToast?.show(`Operacion enviada a ${_labelScope(scope)}`, 'success');
      } catch (e) {
        _renderOperationResult(container.querySelector('#aq-op-result'), null, e.message || 'No se pudo enviar la operacion');
        window.AtlasToast?.show(e.message || 'No se pudo enviar la operacion', 'error');
      }
    });

    container.querySelector('#aq-btn-op-stop')?.addEventListener('click', async () => {
      try {
        await _sendEmergency(container, false);
        window.AtlasToast?.show('Parada total activada', 'warning');
      } catch (e) {
        window.AtlasToast?.show(e.message || 'No se pudo activar kill switch', 'error');
      }
    });

    container.querySelector('#aq-btn-op-reset')?.addEventListener('click', async () => {
      try {
        await _sendEmergency(container, true);
        window.AtlasToast?.show('Control reactivado', 'success');
      } catch (e) {
        window.AtlasToast?.show(e.message || 'No se pudo resetear kill switch', 'error');
      }
    });

    container.querySelector('#aq-account-scope')?.addEventListener('change', () => {
      _syncPaperPhaseState(container);
      _fetchHealth(container);
      _fetchMonitorSummary(container);
      _fetchScannerReport(container);
      _connectLiveUpdates(container);
    });

    container.querySelector('#aq-account-id')?.addEventListener('change', () => {
      _fetchHealth(container);
      _fetchMonitorSummary(container);
      _fetchScannerReport(container);
      _connectLiveUpdates(container);
    });

    container.querySelector('#aq-op-paper-only')?.addEventListener('change', () => {
      _syncPaperPhaseState(container);
      _highlightOperationProfile(container);
    });

    container.querySelector('#aq-op-scope')?.addEventListener('change', () => {
      _syncPaperPhaseState(container);
      _highlightOperationProfile(container);
    });

    container.querySelector('#aq-btn-calib-start')?.addEventListener('click', async () => {
      try {
        await _startVisionCalibration(container);
        window.AtlasToast?.show('Grilla de calibracion iniciada', 'success');
      } catch (e) {
        window.AtlasToast?.show(e.message || 'No se pudo iniciar la calibracion', 'error');
      }
    });

    container.querySelector('#aq-btn-calib-move')?.addEventListener('click', async () => {
      try {
        await _moveVisionCalibrationPose(container);
        window.AtlasToast?.show('Cuello movido', 'success');
      } catch (e) {
        window.AtlasToast?.show(e.message || 'No se pudo mover el cuello', 'error');
      }
    });

    container.querySelector('#aq-btn-calib-sample')?.addEventListener('click', async () => {
      try {
        await _saveVisionCalibrationSample(container);
        window.AtlasToast?.show('Muestra de calibracion guardada', 'success');
      } catch (e) {
        window.AtlasToast?.show(e.message || 'No se pudo guardar la muestra', 'error');
      }
    });

    container.querySelector('#aq-btn-calib-fit')?.addEventListener('click', async () => {
      try {
        await _fitVisionCalibration(container);
        window.AtlasToast?.show('Calibracion ajustada y guardada', 'success');
      } catch (e) {
        window.AtlasToast?.show(e.message || 'No se pudo ajustar la calibracion', 'error');
      }
    });

    container.querySelector('#aq-btn-calib-reset')?.addEventListener('click', async () => {
      try {
        await _resetVisionCalibration(container);
        window.AtlasToast?.show('Sesion de calibracion reiniciada', 'warning');
      } catch (e) {
        window.AtlasToast?.show(e.message || 'No se pudo reiniciar la calibracion', 'error');
      }
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

    container.querySelector('#aq-btn-order-build')?.addEventListener('click', async () => {
      try {
        await _buildOrderFromStrategy(container);
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
            `Confirmar envio REAL a Tradier\nCuenta: ${payload.account_id || 'auto'}\nSimbolo: ${payload.symbol}\nEstrategia: ${payload.strategy_type || payload.asset_class}\nLado: ${payload.side}\nTamano: ${payload.size}`
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
          window.AtlasToast?.show(preview ? 'Previsualizacion de Tradier generada' : 'Orden enviada a Tradier', 'success');
          _fetchMonitorSummary(container);
          _refreshJournalPanel(container);
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
              <div class="stat-card-label">Tasa de exito</div>
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
    _fetchScannerReport(container);
    _fetchOperationStatus(container);
    _fetchVisionCalibrationStatus(container).catch(() => {});
    _refreshJournalPanel(container);
    _activateQuantView(container, 'scanner');
    _syncPaperPhaseState(container);
    _highlightOperationProfile(container);
    _startFallbackPolling(container);
    _startJournalPolling(container);
    _connectLiveUpdates(container);
  },

  destroy() {
    stop(POLL_ID);
    stop(MONITOR_POLL_ID);
    stop(JOURNAL_POLL_ID);
    stop(OPERATION_POLL_ID);
    stop(SCANNER_POLL_ID);
    ACTIVE_CONTAINER = null;
    _stopLiveSocket();
  },

  badge() { return null; },
};

window.AtlasModuleQuantBot = { id: 'atlas-quant' };
