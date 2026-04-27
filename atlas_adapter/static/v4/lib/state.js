/**
 * ATLAS v4 — Reactive State Store (pub/sub)
 * Lightweight state management with change subscriptions.
 */
const _store = {
  health: null,
  model: null,
  provider: null,
  theme: localStorage.getItem('atlas-theme') || 'cyan',
  version: '4.3.2-autonomy2',
  uptime: null,
  lastActivity: null,
  route: '/',
  menuOpen: false,
  assistantBusy: false,
};

const _listeners = new Map();

export function set(key, value) {
  const prev = _store[key];
  _store[key] = value;
  if (prev !== value) {
    const fns = _listeners.get(key);
    if (fns) fns.forEach(fn => { try { fn(value, prev); } catch(e) { console.error('state listener error:', e); } });
    const all = _listeners.get('*');
    if (all) all.forEach(fn => { try { fn(key, value, prev); } catch(e) {} });
  }
}

export function get(key) {
  return _store[key];
}

export function getAll() {
  return { ..._store };
}

export function on(key, fn) {
  if (!_listeners.has(key)) _listeners.set(key, new Set());
  _listeners.get(key).add(fn);
  const cur = _store[key];
  if (cur !== undefined && cur !== null) {
    try {
      fn(cur, cur);
    } catch (e) {
      console.error('state listener error:', e);
    }
  }
  return () => _listeners.get(key).delete(fn);
}

export function batch(updates) {
  for (const [k, v] of Object.entries(updates)) {
    set(k, v);
  }
}

window.AtlasState = { set, get, getAll, on, batch };
