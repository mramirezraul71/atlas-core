/**
 * ATLAS v4 — WebSocket Manager
 * Auto-reconnect, heartbeat, and message routing.
 */
let _ws = null;
let _reconnectTimer = null;
let _heartbeatTimer = null;
const _handlers = new Map();
let _url = null;
let _reconnectAttempts = 0;
const MAX_RECONNECT = 10;

export function connect(url) {
  _url = url;
  _reconnectAttempts = 0;
  _open();
}

function _open() {
  if (_ws && (_ws.readyState === WebSocket.OPEN || _ws.readyState === WebSocket.CONNECTING)) return;

  try {
    _ws = new WebSocket(_url);
  } catch (e) {
    _scheduleReconnect();
    return;
  }

  _ws.onopen = () => {
    _reconnectAttempts = 0;
    _startHeartbeat();
    _emit('_open', {});
  };

  _ws.onmessage = (evt) => {
    try {
      const msg = JSON.parse(evt.data);
      const type = msg.type || msg.event || '_message';
      _emit(type, msg);
      _emit('_message', msg);
    } catch {
      _emit('_raw', evt.data);
    }
  };

  _ws.onclose = () => {
    _stopHeartbeat();
    _emit('_close', {});
    _scheduleReconnect();
  };

  _ws.onerror = () => {};
}

function _scheduleReconnect() {
  if (_reconnectAttempts >= MAX_RECONNECT) return;
  const delay = Math.min(1000 * Math.pow(2, _reconnectAttempts), 30000);
  _reconnectAttempts++;
  _reconnectTimer = setTimeout(_open, delay);
}

function _startHeartbeat() {
  _heartbeatTimer = setInterval(() => {
    if (_ws?.readyState === WebSocket.OPEN) {
      _ws.send(JSON.stringify({ type: 'ping' }));
    }
  }, 15000);
}

function _stopHeartbeat() {
  clearInterval(_heartbeatTimer);
}

function _emit(type, data) {
  const fns = _handlers.get(type);
  if (fns) fns.forEach(fn => { try { fn(data); } catch (e) { console.error('ws handler error:', e); } });
}

export function on(type, fn) {
  if (!_handlers.has(type)) _handlers.set(type, new Set());
  _handlers.get(type).add(fn);
  return () => _handlers.get(type).delete(fn);
}

export function send(data) {
  if (_ws?.readyState === WebSocket.OPEN) {
    _ws.send(typeof data === 'string' ? data : JSON.stringify(data));
  }
}

export function disconnect() {
  clearTimeout(_reconnectTimer);
  _stopHeartbeat();
  if (_ws) _ws.close();
  _ws = null;
}

export function isConnected() {
  return _ws?.readyState === WebSocket.OPEN;
}

window.AtlasWS = { connect, on, send, disconnect, isConnected };
