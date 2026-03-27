/* ================================================================
   Atlas Code-Quant — API Client
   Wrapper REST + WebSocket para todos los endpoints del backend
   ================================================================ */

// Si se sirve desde Atlas (8791), las llamadas van directamente a 8795.
// Si se sirve desde el propio servidor quant (8795), same-origin funciona igual.
const API_BASE = (location.port === '8791' || location.port === '443')
  ? 'http://127.0.0.1:8795'
  : '';
const API_KEY = (() => {
  const saved = localStorage.getItem('quant_api_key');
  if (saved && saved.trim()) return saved.trim();
  const fallback = 'atlas-quant-local';
  localStorage.setItem('quant_api_key', fallback);
  return fallback;
})();

// ── HTTP helper ────────────────────────────────────────────────────
const _FETCH_TIMEOUT = 8000;  // 8 s máximo por petición

function _fetchWithTimeout(url, opts = {}, timeoutMs = _FETCH_TIMEOUT) {
  const ctrl = new AbortController();
  const id = setTimeout(() => ctrl.abort(), timeoutMs);
  return fetch(url, { ...opts, signal: ctrl.signal })
    .finally(() => clearTimeout(id));
}

async function _parseResponse(r) {
  let payload = null;
  try {
    payload = await r.json();
  } catch (_) {
    payload = { ok: false, error: `HTTP ${r.status}` };
  }
  if (payload && typeof payload === 'object') {
    payload._httpStatus = r.status;
    if (!r.ok && payload.ok == null) payload.ok = false;
    if (!r.ok && !payload.error) payload.error = payload.detail || `HTTP ${r.status}`;
  }
  return payload;
}

async function apiGet(path, params = {}, options = {}) {
  const url = new URL(API_BASE + path, window.location.origin);
  Object.entries(params).forEach(([k, v]) => url.searchParams.set(k, v));
  const r = await _fetchWithTimeout(url, {
    headers: { 'x-api-key': API_KEY, 'Accept': 'application/json' }
  }, options.timeoutMs);
  return _parseResponse(r);
}

async function apiPost(path, body = {}, options = {}) {
  const r = await _fetchWithTimeout(API_BASE + path, {
    method: 'POST',
    headers: {
      'x-api-key': API_KEY,
      'Content-Type': 'application/json',
      'Accept': 'application/json'
    },
    body: JSON.stringify(body)
  }, options.timeoutMs);
  return _parseResponse(r);
}

async function apiGetFirst(paths, params = {}, options = {}) {
  let lastResponse = null;
  for (const path of paths) {
    const response = await apiGet(path, params, options);
    lastResponse = response;
    if (!response || response._httpStatus !== 404) return response;
  }
  return lastResponse || { ok: false, error: 'Endpoint no disponible' };
}

// ── Endpoint wrappers ─────────────────────────────────────────────
const QuantAPI = {

  health: () => apiGetFirst(['/api/v2/quant/health', '/health']),

  status: (scope = 'paper') => apiGetFirst(['/api/v2/quant/status', '/status'], { account_scope: scope }),

  canonicalSnapshot: (scope = 'paper', accountId = '') =>
    apiGetFirst(['/api/v2/quant/canonical/snapshot', '/canonical/snapshot'], {
      account_scope: scope,
      ...(accountId ? { account_id: accountId } : {}),
    }),

  dashboardOverview: (scope = 'paper') => apiGetFirst(['/api/v2/quant/dashboard/overview'], { account_scope: scope }),

  positions: (scope = 'paper') => apiGetFirst(['/api/v2/quant/monitor/summary', '/monitor/summary'], { account_scope: scope }),

  // Backtest
  runBacktest: (payload) => apiPost('/api/v2/quant/backtest', payload),
  listReports: () => apiGetFirst(['/api/v2/quant/backtest/reports', '/backtest/reports']),

  // Scanner
  scannerStatus: () => apiGetFirst(['/api/v2/quant/scanner/status', '/scanner/status']),
  scannerStart:  () => apiPost('/api/v2/quant/scanner/control', { action: 'start' }),
  scannerStop:   () => apiPost('/api/v2/quant/scanner/control', { action: 'stop' }),
  scannerReport: () => apiGetFirst(['/api/v2/quant/scanner/report', '/scanner/report']),

  // Journal
  journalStats:   () => apiGetFirst(['/api/v2/quant/journal/stats', '/journal/stats']),
  journalEntries: (limit = 50, status = '') =>
    apiGetFirst(['/api/v2/quant/journal/entries', '/journal/entries'], { limit, status }),

  // Phase 3 — Alerts
  alertsStatus: () => apiGetFirst(['/api/v2/quant/alerts/status']),
  alertsTest:   (message) => apiPost(`/api/v2/quant/alerts/test?message=${encodeURIComponent(message)}`),

  // Phase 3 — Visual
  visualState: () => apiGetFirst(
    ['/api/v2/quant/visual/state'],
    {},
    { timeoutMs: 15000 },
  ),
  visionCalibrationStatus: () => apiGetFirst([
    '/api/v2/quant/vision/calibration/status',
    '/vision/calibration/status',
  ]),

  // Phase 3 — Retraining
  retrainingStatus:  (symbol) => apiGetFirst(['/api/v2/quant/retraining/status'], { symbol }),
  retrainingTrigger: (symbol) => apiPost(`/api/v2/quant/retraining/trigger?symbol=${encodeURIComponent(symbol)}`),

  // Monitor
  monitorSummary: (scope = 'paper') => apiGetFirst(['/api/v2/quant/monitor/summary', '/monitor/summary'], { account_scope: scope }),

  // Paper Trading
  paperAccount: () => apiGetFirst(['/api/v2/quant/paper/account', '/paper/account']),
  paperPositions: () => apiGetFirst(['/api/v2/quant/paper/positions', '/paper/positions']),
  paperOrders: (limit = 200) => apiGetFirst(['/api/v2/quant/paper/orders', '/paper/orders'], { limit }),
  paperEquityCurve: (limit = 500) => apiGetFirst(['/api/v2/quant/paper/equity-curve', '/paper/equity-curve'], { limit }),
  paperFill: (body) => apiPost('/api/v2/quant/paper/fill', body),
  paperClose: (body) => apiPost('/paper/close', body),
  paperReset: (capital = 0) => apiPost('/api/v2/quant/paper/reset', { initial_capital: capital }),
};

// ── WebSocket manager ─────────────────────────────────────────────
class QuantWS {
  constructor() {
    this._ws = null;
    this._listeners = {};
    this._reconnectDelay = 2000;
    this._maxDelay = 30000;
  }

  connect() {
    const scope = window.QUANT_ACCOUNT_SCOPE || 'paper';
    const accountId = window.QUANT_ACCOUNT_ID || '';
    const protocol = location.protocol === 'https:' ? 'wss' : 'ws';
    const wsHost = (location.port === '8791' || location.port === '443')
      ? '127.0.0.1:8795'
      : location.host;
    const qs = new URLSearchParams({ api_key: API_KEY, account_scope: scope });
    if (accountId) qs.set('account_id', accountId);
    const url = `${protocol}://${wsHost}/api/v2/quant/ws/live-updates?${qs.toString()}`;
    this._ws = new WebSocket(url);

    this._ws.onopen = () => {
      this._reconnectDelay = 2000;
      document.getElementById('ws-dot')?.classList.add('connected');
      document.getElementById('ws-dot')?.classList.remove('error');
      this._emit('connected');
    };

    this._ws.onmessage = (e) => {
      try {
        const msg = JSON.parse(e.data);
        this._emit('message', msg);
        if (msg.type) this._emit(msg.type, msg);
      } catch (_) {}
    };

    this._ws.onclose = () => {
      document.getElementById('ws-dot')?.classList.remove('connected');
      this._emit('disconnected');
      setTimeout(() => {
        this._reconnectDelay = Math.min(this._reconnectDelay * 1.5, this._maxDelay);
        this.connect();
      }, this._reconnectDelay);
    };

    this._ws.onerror = () => {
      document.getElementById('ws-dot')?.classList.add('error');
    };
  }

  on(event, cb) {
    if (!this._listeners[event]) this._listeners[event] = [];
    this._listeners[event].push(cb);
    return this;
  }

  off(event, cb) {
    if (!this._listeners[event]) return;
    this._listeners[event] = this._listeners[event].filter(l => l !== cb);
  }

  _emit(event, data) {
    (this._listeners[event] || []).forEach(cb => cb(data));
  }
}

window.QuantAPI = QuantAPI;
window.quantWS  = new QuantWS();
