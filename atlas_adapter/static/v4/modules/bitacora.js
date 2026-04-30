/**
 * ATLAS v4.2 — Bitacora Module
 * Timeline coloreada con filtro por nivel + contador de entradas.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'bitacora-module';
let _activeFilter = 'all';

function _esc(s) { const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML; }

function _level(entry) {
  const explicit = ((entry.level || entry.type || entry.severity) + '').toLowerCase();
  if (explicit && explicit !== 'undefined') return explicit;
  // Audit entries: ok=false → error, warn en error field
  if (entry.ok === false || entry.error) return 'error';
  return 'info';
}

function _tlClass(lvl) {
  if (lvl === 'error' || lvl === 'critical') return 'error';
  if (lvl === 'warn' || lvl === 'warning') return 'warn';
  if (lvl === 'success' || lvl === 'ok') return 'success';
  if (lvl === 'debug') return 'debug';
  return 'info';
}

function _icon(cls) {
  return { info: 'I', warn: '!', error: '✕', success: '✓', debug: 'D' }[cls] || 'I';
}

function _shortTime(ts) {
  if (!ts) return '--:--';
  try { return new Date(ts).toLocaleTimeString(); } catch { return ts; }
}

function _extractEntries(data) {
  const p = data?.data ?? data;
  return p?.entries || p?.items || p?.logs || (Array.isArray(p) ? p : []);
}

export default {
  id: 'bitacora',
  label: 'Bitácora',
  icon: 'file-text',
  category: 'monitoring',

  render(container) {
    // Lee filtro inicial desde URL param: #/bitacora?f=error
    const urlParam = new URLSearchParams(location.hash.split('?')[1] || '');
    _activeFilter = urlParam.get('f') || 'all';

    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Dashboard
          </button>
          <h2>Bitácora ANS</h2>
          <div style="margin-left:auto;display:flex;gap:8px;align-items:center">
            ${_activeFilter !== 'all' ? `<span class="chip orange">Filtro: ${_activeFilter}</span>` : ''}
            <span class="live-badge">LIVE</span>
          </div>
        </div>
        <div class="module-body">
          <div class="action-bar" style="padding-top:0;margin-bottom:8px">
            <button class="action-btn ${_activeFilter === 'all'   ? 'primary' : ''}" id="f-all"  >Todo</button>
            <button class="action-btn ${_activeFilter === 'info'  ? 'primary' : ''}" id="f-info" >Info</button>
            <button class="action-btn ${_activeFilter === 'warn'  ? 'primary' : ''}" id="f-warn" >Warn</button>
            <button class="action-btn ${_activeFilter === 'error' ? 'primary' : ''}" id="f-error">Error</button>
            <span class="chip" style="margin-left:auto" id="bit-count">-- entradas</span>
          </div>
          <div class="section-title">Actividad del Sistema</div>
          <div class="timeline" id="bit-list">
            <div style="padding:20px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
          </div>
        </div>
      </div>
    `;

    // Botones de filtro (sin onclick inline para evitar globals)
    ['all','info','warn','error'].forEach(f => {
      container.querySelector(`#f-${f}`)?.addEventListener('click', () => _setFilter(f, container));
    });

    window._bitFilter = (f) => _setFilter(f, container);

    _fetchAndRender(container);
    poll(POLL_ID, '/audit/tail?n=80', 7000, (data) => {
      if (data) _renderEntries(container, data);
    });
  },

  destroy() { stop(POLL_ID); delete window._bitFilter; },
  badge() { return null; },
};

function _setFilter(f, container) {
  _activeFilter = f;
  ['all','info','warn','error'].forEach(k => {
    const btn = container.querySelector(`#f-${k}`);
    if (btn) btn.className = 'action-btn' + (k === f ? ' primary' : '');
  });
  const list = container.querySelector('#bit-list');
  if (list && list._data) _renderEntries(container, list._data);
}

async function _fetchAndRender(container) {
  try {
    const r = await fetch('/audit/tail?n=80');
    const data = await r.json();
    _renderEntries(container, data);
  } catch (e) {
    const el = container.querySelector('#bit-list');
    if (el) el.innerHTML = `<div class="tl-entry error"><div class="tl-icon">✕</div><div class="tl-body"><div class="tl-msg">Error: ${_esc(e.message)}</div></div></div>`;
  }
}

function _renderEntries(container, data) {
  const list = container.querySelector('#bit-list');
  if (!list) return;
  list._data = data;

  let entries = _extractEntries(data);
  const total = entries.length;

  if (_activeFilter !== 'all') {
    entries = entries.filter(e => {
      const cls = _tlClass(_level(e));
      if (_activeFilter === 'warn')  return cls === 'warn';
      if (_activeFilter === 'error') return cls === 'error';
      return cls === 'info' || cls === 'success' || cls === 'debug';
    });
  }

  const countEl = container.querySelector('#bit-count');
  if (countEl) countEl.textContent = `${total} entradas`;

  if (entries.length === 0) {
    list.innerHTML = `<div class="empty-state">
      <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5"><path d="M14 2H6a2 2 0 00-2 2v16a2 2 0 002 2h12a2 2 0 002-2V8z"/><polyline points="14 2 14 8 20 8"/></svg>
      <div class="empty-title">Sin entradas</div>
      <div class="empty-sub">La bitácora no tiene registros para el filtro seleccionado</div>
    </div>`;
    return;
  }

  list.innerHTML = [...entries].reverse().map(e => {
    const cls = _tlClass(_level(e));
    // Audit entries have action+actor; generic entries have message/text
    const msg = _esc(e.action ? `[${e.actor || e.role || 'sys'}] ${e.action}${e.error ? ' — ' + e.error : ''}` : (e.message || e.msg || e.text || e.event || JSON.stringify(e)));
    const ts   = _shortTime(e.timestamp || e.ts || e.time);
    const sub  = e.module || e.subsystem || e.source || '';
    const meta = e.ms != null ? `${e.ms}ms` : '';
    return `<div class="tl-entry ${cls}">
      <div class="tl-time">${_esc(ts)}</div>
      <div class="tl-icon">${_icon(cls)}</div>
      <div class="tl-body">
        <div class="tl-msg">${msg}</div>
        ${sub || meta ? `<div class="tl-meta">${_esc([sub, meta].filter(Boolean).join(' · '))}</div>` : ''}
      </div>
    </div>`;
  }).join('');
}

window.AtlasModuleBitacora = { id: 'bitacora' };
