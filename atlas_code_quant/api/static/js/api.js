/* ================================================================
   Atlas Code-Quant — API Client
   Wrapper REST + WebSocket para todos los endpoints del backend
   ================================================================ */

// Si se sirve desde Atlas (8791/443), enruta al API quant en 8795 usando el
// mismo hostname del navegador para soportar localhost, LAN IP o dominio.
// Si se sirve desde el propio servidor quant (8795), same-origin funciona igual.
const QUANT_API_ORIGIN = `${location.protocol}//${location.hostname}:8795`;
const API_BASE = (location.port === '8791' || location.port === '443')
  ? QUANT_API_ORIGIN
  : '';
const IS_LOCALHOST =
  location.hostname === '127.0.0.1' || location.hostname === 'localhost';
const DEFAULT_LOCAL_API_KEY = IS_LOCALHOST ? 'atlas-quant-local' : '';
let API_KEY = localStorage.getItem('quant_api_key') || DEFAULT_LOCAL_API_KEY;
if (!localStorage.getItem('quant_api_key') && DEFAULT_LOCAL_API_KEY) {
  localStorage.setItem('quant_api_key', DEFAULT_LOCAL_API_KEY);
}

function persistApiKey(value) {
  API_KEY = (value || '').trim();
  try {
    if (API_KEY) {
      localStorage.setItem('quant_api_key', API_KEY);
    } else {
      localStorage.removeItem('quant_api_key');
    }
  } catch (_) {}
  return API_KEY;
}

function buildAuthHeaders(extraHeaders = {}) {
  return {
    'Accept': 'application/json',
    'Cache-Control': 'no-cache, no-store, must-revalidate',
    'Pragma': 'no-cache',
    ...(API_KEY ? { 'x-api-key': API_KEY } : {}),
    ...extraHeaders,
  };
}

function shouldRetryWithLocalKey(response) {
  return response?.status === 401 && IS_LOCALHOST && DEFAULT_LOCAL_API_KEY && API_KEY !== DEFAULT_LOCAL_API_KEY;
}

// ── HTTP helper ────────────────────────────────────────────────────
const _FETCH_TIMEOUT_MS = 12000;

function _fetchWithTimeout(url, options = {}) {
  const ctrl = new AbortController();
  const tid = setTimeout(() => ctrl.abort(), _FETCH_TIMEOUT_MS);
  return fetch(url, { cache: 'no-store', ...options, signal: ctrl.signal })
    .finally(() => clearTimeout(tid));
}

async function _parseResponse(r) {
  const contentType = r.headers.get('content-type') || '';
  const payload = contentType.includes('application/json')
    ? await r.json()
    : await r.text();
  if (!r.ok) {
    const detail = typeof payload === 'string'
      ? payload
      : payload?.detail || payload?.error || `HTTP ${r.status}`;
    throw new Error(detail);
  }
  return payload;
}

async function apiGet(path, params = {}) {
  const url = new URL(API_BASE + path, window.location.origin);
  Object.entries(params).forEach(([k, v]) => url.searchParams.set(k, v));
  let r = await _fetchWithTimeout(url, {
    headers: buildAuthHeaders()
  });
  if (shouldRetryWithLocalKey(r)) {
    persistApiKey(DEFAULT_LOCAL_API_KEY);
    r = await _fetchWithTimeout(url, {
      headers: buildAuthHeaders()
    });
  }
  return _parseResponse(r);
}

async function apiPost(path, body = {}) {
  let r = await _fetchWithTimeout(API_BASE + path, {
    method: 'POST',
    headers: buildAuthHeaders({ 'Content-Type': 'application/json' }),
    body: JSON.stringify(body)
  });
  if (shouldRetryWithLocalKey(r)) {
    persistApiKey(DEFAULT_LOCAL_API_KEY);
    r = await _fetchWithTimeout(API_BASE + path, {
      method: 'POST',
      headers: buildAuthHeaders({ 'Content-Type': 'application/json' }),
      body: JSON.stringify(body)
    });
  }
  return _parseResponse(r);
}

// ── Endpoint wrappers ─────────────────────────────────────────────
const QuantAPI = {

  health: () => apiGet('/health'),

  status: (scope = 'paper') => apiGet('/status', { account_scope: scope }),

  canonicalSnapshot: (scope = 'paper', accountId = '') =>
    apiGet('/api/v2/quant/canonical/snapshot', { account_scope: scope, ...(accountId ? { account_id: accountId } : {}) }),

  dashboardOverview: (scope = 'paper', accountId = '') =>
    apiGet('/api/v2/quant/dashboard/overview', { account_scope: scope, ...(accountId ? { account_id: accountId } : {}) }),

  positions: (scope = 'paper', accountId = '') =>
    apiGet('/api/v2/quant/positions', { account_scope: scope, ...(accountId ? { account_id: accountId } : {}) }),

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

  journalChartData: (limit = 500, scope = 'paper') =>
    apiGet('/journal/chart-data', { limit, account_scope: scope }),

  learningICSummary: (method = '') =>
    apiGet('/learning/ic/summary', method ? { method } : {}),

  learningOrchestratorStatus: () =>
    apiGet('/learning/orchestrator/status'),

  // DuckDB Analytics
  journalAnalytics: (scope = 'paper') =>
    apiGet('/journal/analytics', { account_scope: scope }),
  journalAnalyticsRisk: (scope = 'paper') =>
    apiGet('/journal/analytics/risk', { account_scope: scope }),

  // Event Store
  eventStoreStats: () => apiGet('/events/stats'),
  eventStoreQuery: (topic, limit = 100) =>
    apiGet('/events/query', { topic, limit }),

  // Strategy Evolution
  evolutionRun: (symbols = 'SPY,QQQ,AAPL', generations = 20, population = 30) =>
    apiPost(`/evolution/run?symbols=${encodeURIComponent(symbols)}&generations=${generations}&population=${population}`),
  evolutionResults: () => apiGet('/evolution/results'),

  // Phase 3 — Alerts
  alertsStatus: () => apiGet('/api/v2/quant/alerts/status'),
  alertsTest:   (message) => apiPost(`/api/v2/quant/alerts/test?message=${encodeURIComponent(message)}`),

  // Phase 3 — Visual
  visualState: () => apiGet('/api/v2/quant/visual/state'),

  // Phase 3 — Retraining
  retrainingStatus:  (symbol) => apiGet('/api/v2/quant/retraining/status', { symbol }),
  retrainingTrigger: (symbol) => apiPost(`/api/v2/quant/retraining/trigger?symbol=${encodeURIComponent(symbol)}`),

  // Monitor
  monitorSummary: (scope = 'paper', accountId = '') =>
    apiGet('/monitor/summary', { account_scope: scope, ...(accountId ? { account_id: accountId } : {}) }),
};

// ── WebSocket manager ─────────────────────────────────────────────
class QuantWS {
  constructor() {
    this._ws = null;
    this._listeners = {};
    this._reconnectDelay = 2000;
    this._maxDelay = 30000;
    this._connectTimer = null;
    this._explicitlyClosed = false;
    this._everOpened = false;
    this._failedConnects = 0;
  }

  connect() {
    if (document.visibilityState === 'hidden') return;
    const readyState = this._ws?.readyState;
    if (readyState === WebSocket.OPEN || readyState === WebSocket.CONNECTING) return;
    this._explicitlyClosed = false;
    if (this._connectTimer) {
      clearTimeout(this._connectTimer);
      this._connectTimer = null;
    }
    const scope = window.QUANT_ACCOUNT_SCOPE || 'paper';
    const accountId = window.QUANT_ACCOUNT_ID || '';
    const protocol = location.protocol === 'https:' ? 'wss' : 'ws';
    const wsHost = (location.port === '8791' || location.port === '443')
      ? `${location.hostname}:8795`
      : location.host;
    const wsUrl = new URL(`${protocol}://${wsHost}/ws/live-updates`);
    if (API_KEY) wsUrl.searchParams.set('api_key', API_KEY);
    if (scope) wsUrl.searchParams.set('account_scope', scope);
    if (accountId) wsUrl.searchParams.set('account_id', accountId);
    const url = wsUrl.toString();
    this._everOpened = false;
    this._ws = new WebSocket(url);

    this._ws.onopen = () => {
      this._reconnectDelay = 2000;
      this._everOpened = true;
      this._failedConnects = 0;
      window._quantWsConnected = true;
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
      window._quantWsConnected = false;
      document.getElementById('ws-dot')?.classList.remove('connected');
      this._emit('disconnected');
      this._ws = null;
      if (this._explicitlyClosed) return;
      if (!this._everOpened) {
        this._failedConnects += 1;
      } else {
        this._failedConnects = 0;
      }
      // Si el backend está offline usa delay mayor para no saturar la consola
      const backendDown = window._backendOnline === false;
      this._reconnectDelay = Math.min(
        this._reconnectDelay * (backendDown ? 2 : 1.5),
        this._maxDelay
      );
      if (this._failedConnects >= 5) {
        this._connectTimer = setTimeout(() => this.connect(), this._maxDelay);
        return;
      }
      const jitter = Math.floor(Math.random() * 900);
      this._connectTimer = setTimeout(() => this.connect(), this._reconnectDelay + jitter);
    };

    this._ws.onerror = () => {
      document.getElementById('ws-dot')?.classList.add('error');
    };
  }

  close() {
    this._explicitlyClosed = true;
    window._quantWsConnected = false;
    if (this._connectTimer) {
      clearTimeout(this._connectTimer);
      this._connectTimer = null;
    }
    if (this._ws && (this._ws.readyState === WebSocket.OPEN || this._ws.readyState === WebSocket.CONNECTING)) {
      this._ws.close(1000, 'Dashboard hidden');
    }
    this._ws = null;
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
