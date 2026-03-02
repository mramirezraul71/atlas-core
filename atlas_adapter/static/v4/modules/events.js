/**
 * ATLAS v4.2 — Event Bus Module
 * Timeline del bus de eventos del kernel con tipo/fuente.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'events-module';

function _esc(s) { const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML; }

function _eventClass(type) {
  const t = (type + '').toLowerCase();
  if (t.includes('error') || t.includes('fail'))   return 'error';
  if (t.includes('warn'))                           return 'warn';
  if (t.includes('success') || t.includes('done') || t.includes('ok')) return 'success';
  if (t.includes('debug') || t.includes('trace'))  return 'debug';
  return 'info';
}

function _icon(cls) {
  return { info: '⚡', warn: '!', error: '✕', success: '✓', debug: 'D' }[cls] || '⚡';
}

function _shortTime(ts) {
  if (!ts) return '--:--';
  try { return new Date(ts).toLocaleTimeString(); } catch { return String(ts).slice(0, 8); }
}

function _extractEvents(data) {
  const p = data?.data ?? data;
  return p?.items || p?.events || p?.history || (Array.isArray(p) ? p : []);
}

export default {
  id: 'events',
  label: 'Event Bus',
  icon: 'zap',
  category: 'monitoring',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Inicio
          </button>
          <h2>Event Bus</h2>
          <div style="margin-left:auto"><span class="live-badge">LIVE</span></div>
        </div>
        <div class="module-body">
          <div class="stat-row" style="margin-bottom:16px">
            <div class="stat-card">
              <div class="stat-card-label">Eventos recientes</div>
              <div class="stat-card-value accent" id="ev-count">--</div>
              <div class="stat-card-sub">Últimos 30</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Errores</div>
              <div class="stat-card-value red" id="ev-errors">0</div>
              <div class="stat-card-sub">En esta carga</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Última actividad</div>
              <div class="stat-card-value" id="ev-last" style="font-size:14px">--</div>
            </div>
          </div>
          <div class="section-title">Historial de Eventos</div>
          <div class="timeline" id="ev-timeline">
            <div style="padding:20px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
          </div>
        </div>
      </div>
    `;

    _fetchAndRender(container);
    poll(POLL_ID, '/api/kernel/event-bus/history?limit=30', 5000, (data) => {
      if (data) _render(container, data);
    });
  },

  destroy() { stop(POLL_ID); },
};

async function _fetchAndRender(container) {
  try {
    const r = await fetch('/api/kernel/event-bus/history?limit=30');
    const data = await r.json();
    _render(container, data);
  } catch (e) {
    const el = container.querySelector('#ev-timeline');
    if (el) el.innerHTML = `<div class="tl-entry error">
      <div class="tl-time">--:--</div><div class="tl-icon">✕</div>
      <div class="tl-body"><div class="tl-msg">Error: ${_esc(e.message)}</div></div>
    </div>`;
  }
}

function _render(container, data) {
  const tl = container.querySelector('#ev-timeline');
  if (!tl) return;

  const events = _extractEvents(data);
  const errors = events.filter(e => _eventClass(e.type || e.event_type || '') === 'error').length;

  const txt = (id, v) => { const el = container.querySelector(id); if (el) el.textContent = v; };
  txt('#ev-count', events.length);
  txt('#ev-errors', errors);
  if (events.length > 0) {
    const last = events[events.length - 1];
    txt('#ev-last', _shortTime(last.timestamp || last.ts));
  }

  if (events.length === 0) {
    tl.innerHTML = `<div class="empty-state">
      <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5"><polygon points="13 2 3 14 12 14 11 22 21 10 12 10 13 2"/></svg>
      <div class="empty-title">Bus de eventos silencioso</div>
      <div class="empty-sub">No hay eventos registrados recientemente</div>
    </div>`;
    return;
  }

  tl.innerHTML = [...events].reverse().map(ev => {
    const type = ev.type || ev.event_type || ev.kind || 'event';
    const cls  = _eventClass(type);
    const ts   = _shortTime(ev.timestamp || ev.ts);
    const src  = ev.source || ev.origin || ev.subsystem || '';
    const msg  = ev.message || ev.msg || ev.data?.message || JSON.stringify(ev.data || {}).slice(0, 120);
    return `<div class="tl-entry ${cls}">
      <div class="tl-time">${_esc(ts)}</div>
      <div class="tl-icon">${_icon(cls)}</div>
      <div class="tl-body">
        <div class="tl-msg">
          <span class="tl-lvl ${cls}">${_esc(type)}</span>${_esc(msg)}
        </div>
        ${src ? `<div class="tl-meta">${_esc(src)}</div>` : ''}
      </div>
    </div>`;
  }).join('');
}

window.AtlasModuleEvents = { id: 'events' };
