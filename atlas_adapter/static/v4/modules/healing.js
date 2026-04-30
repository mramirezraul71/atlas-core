/**
 * ATLAS v4.2 — Healing Module
 * Timeline de acciones de auto-curación con estadísticas.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'healing-module';

function _esc(s) { const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML; }

function _healClass(item) {
  const status = (item.status || item.result || '').toLowerCase();
  if (status === 'success' || status === 'ok' || status === 'healed') return 'success';
  if (status === 'failed' || status === 'error') return 'error';
  if (status === 'running' || status === 'in_progress') return 'info';
  return 'warn';
}

function _icon(cls) {
  return { success: '✓', error: '✕', info: '↻', warn: '!' }[cls] || '!';
}

function _shortTime(ts) {
  if (!ts) return '--:--';
  try { return new Date(ts).toLocaleTimeString(); } catch { return String(ts).slice(0, 8); }
}

function _extractItems(data) {
  const p = data?.data ?? data;
  return p?.items || p?.history || p?.events || p?.actions || (Array.isArray(p) ? p : []);
}

export default {
  id: 'healing',
  label: 'Auto-Healing',
  icon: 'activity',
  category: 'monitoring',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Inicio
          </button>
          <h2>Auto-Healing</h2>
          <div style="margin-left:auto"><span class="live-badge">LIVE</span></div>
        </div>
        <div class="module-body">
          <div class="stat-row" style="margin-bottom:16px">
            <div class="stat-card">
              <div class="stat-card-label">Acciones totales</div>
              <div class="stat-card-value" id="h-total">--</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Exitosas</div>
              <div class="stat-card-value green" id="h-ok">0</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Fallidas</div>
              <div class="stat-card-value red" id="h-fail">0</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Última acción</div>
              <div class="stat-card-value" id="h-last" style="font-size:14px">--</div>
            </div>
          </div>
          <div class="section-title">Historial de Curación</div>
          <div class="timeline" id="heal-timeline">
            <div style="padding:20px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
          </div>
        </div>
      </div>
    `;

    _fetchAndRender(container);
    poll(POLL_ID, '/api/healing/history?limit=20', 6000, (data) => {
      if (data) _render(container, data);
    });
  },

  destroy() { stop(POLL_ID); },
  badge() { return null; },
};

async function _fetchAndRender(container) {
  try {
    const r = await fetch('/api/healing/history?limit=20');
    const data = await r.json();
    _render(container, data);
  } catch (e) {
    const el = container.querySelector('#heal-timeline');
    if (el) el.innerHTML = `<div class="tl-entry error">
      <div class="tl-time">--:--</div><div class="tl-icon">✕</div>
      <div class="tl-body"><div class="tl-msg">Error: ${_esc(e.message)}</div></div>
    </div>`;
  }
}

function _render(container, data) {
  const tl = container.querySelector('#heal-timeline');
  if (!tl) return;

  const items = _extractItems(data);
  const okCount   = items.filter(i => _healClass(i) === 'success').length;
  const failCount = items.filter(i => _healClass(i) === 'error').length;

  const txt = (id, v) => { const el = container.querySelector(id); if (el) el.textContent = v; };
  txt('#h-total', items.length);
  txt('#h-ok', okCount);
  txt('#h-fail', failCount);
  if (items.length > 0) {
    const last = items[items.length - 1];
    txt('#h-last', _shortTime(last.timestamp || last.ts || last.time));
  }

  if (items.length === 0) {
    tl.innerHTML = `<div class="empty-state">
      <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5"><path d="M22 11.08V12a10 10 0 11-5.93-9.14"/><polyline points="22 4 12 14.01 9 11.01"/></svg>
      <div class="empty-title">Sin acciones de curación</div>
      <div class="empty-sub">El sistema no ha necesitado auto-healing</div>
    </div>`;
    return;
  }

  tl.innerHTML = [...items].reverse().map(item => {
    const cls    = _healClass(item);
    const ts     = _shortTime(item.timestamp || item.ts || item.time);
    const action = item.action || item.type || item.name || 'heal';
    const target = item.target || item.component || item.module || '';
    const detail = item.detail || item.message || item.reason || '';
    return `<div class="tl-entry ${cls}">
      <div class="tl-time">${_esc(ts)}</div>
      <div class="tl-icon">${_icon(cls)}</div>
      <div class="tl-body">
        <div class="tl-msg">
          <span class="tl-lvl ${cls}">${_esc(action)}</span>
          ${target ? `<strong>${_esc(target)}</strong> — ` : ''}${_esc(detail)}
        </div>
        ${item.duration_ms ? `<div class="tl-meta">${item.duration_ms}ms</div>` : ''}
      </div>
    </div>`;
  }).join('');
}

window.AtlasModuleHealing = { id: 'healing' };
