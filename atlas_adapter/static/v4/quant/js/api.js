/* ================================================================
   Atlas Code-Quant — API Client
   Wrapper REST + WebSocket para todos los endpoints del backend
   ================================================================ */

// Si se sirve desde Atlas (8791), las llamadas van directamente a 8795.
// Si se sirve desde el propio servidor quant (8795), same-origin funciona igual.
const API_BASE = (location.port === '8791' || location.port === '443')
  ? 'http://127.0.0.1:8795'
  : '';
const API_KEY  = localStorage.getItem('quant_api_key') || '';

// ── HTTP helper ────────────────────────────────────────────────────
const _FETCH_TIMEOUT = 8000;  // 8 s máximo por petición

function _fetchWithTimeout(url, opts = {}) {
  const ctrl = new AbortController();
  const id = setTimeout(() => ctrl.abort(), _FETCH_TIMEOUT);
  return fetch(url, { ...opts, signal: ctrl.signal })
    .finally(() => clearTimeout(id));
}

async function apiGet(path, params = {}) {
  const url = new URL(API_BASE + path, window.location.origin);
  Object.entries(params).forEach(([k, v]) => url.searchParams.set(k, v));
  const r = await _fetchWithTimeout(url, {
    headers: { 'x-api-key': API_KEY, 'Accept': 'application/json' }
  });
  return r.json();
}

async function apiPost(path, body = {}) {
  const r = await _fetchWithTimeout(API_BASE + path, {
    method: 'POST',
    headers: {
      'x-api-key': API_KEY,
      'Content-Type': 'application/json',
      'Accept': 'application/json'
    },
    body: JSON.stringify(body)
  });
  return r.json();
}

// ── Endpoint wrappers ─────────────────────────────────────────────
const QuantAPI = {

  health: () => apiGet('/health'),

  status: (scope = 'paper') => apiGet('/status', { account_scope: scope }),

  dashboardOverview: (scope = 'paper') => apiGet('/api/v2/quant/dashboard/overview', { account_scope: scope }),

  positions: () => apiGet('/positions'),

  // Backtest
  runBacktest: (payload) => apiPost('/backtest', payload),
  listReports: () => apiGet('/backtest/reports'),

  // Scanner
  scannerStatus: () => apiGet('/api/v2/quant/scanner/status'),
  scannerStart:  () => apiPost('/api/v2/quant/scanner/control', { action: 'start' }),
  scannerStop:   () => apiPost('/api/v2/quant/scanner/control', { action: 'stop' }),
  scannerReport: () => apiGet('/api/v2/quant/scanner/report'),

  // Journal
  journalStats:   (scope = 'paper') => apiGet('/journal/stats', { account_scope: scope }),
  journalEntries: (limit = 50, scope = 'paper') =>
    apiGet('/journal/entries', { limit, account_scope: scope }),

  // Phase 3 — Alerts
  alertsStatus: () => apiGet('/api/v2/quant/alerts/status'),
  alertsTest:   (message) => apiPost(`/api/v2/quant/alerts/test?message=${encodeURIComponent(message)}`),

  // Phase 3 — Visual
  visualState: () => apiGet('/api/v2/quant/visual/state'),

  // Phase 3 — Retraining
  retrainingStatus:  (symbol) => apiGet('/api/v2/quant/retraining/status', { symbol }),
  retrainingTrigger: (symbol) => apiPost(`/api/v2/quant/retraining/trigger?symbol=${encodeURIComponent(symbol)}`),

  // Monitor
  monitorSummary: (scope = 'paper') => apiGet('/monitor/summary', { account_scope: scope }),
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
    const protocol = location.protocol === 'https:' ? 'wss' : 'ws';
    const wsHost = (location.port === '8791' || location.port === '443')
      ? '127.0.0.1:8795'
      : location.host;
    const url = `${protocol}://${wsHost}/ws/live-updates`;
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
