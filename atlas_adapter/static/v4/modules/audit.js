/**
 * ATLAS v4.2 — Audit Tail Module
 * Timeline de eventos de auditoría con colores por nivel.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'audit-module';

function _esc(s) { const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML; }

function _parseLevel(line) {
  const u = (line + '').toUpperCase();
  if (u.includes('ERROR') || u.includes('CRITICAL')) return 'error';
  if (u.includes('WARN'))    return 'warn';
  if (u.includes('SUCCESS')) return 'success';
  if (u.includes('DEBUG'))   return 'debug';
  return 'info';
}

function _icon(cls) {
  return { info: 'A', warn: '!', error: '✕', success: '✓', debug: 'D' }[cls] || 'A';
}

function _extractLines(data) {
  const p = data?.data ?? data;
  const raw = p?.lines || p?.items || p?.events || p?.audit || p;
  if (Array.isArray(raw)) return raw;
  if (typeof raw === 'string') return raw.split('\n').filter(Boolean);
  return [];
}

export default {
  id: 'audit',
  label: 'Audit Trail',
  icon: 'shield',
  category: 'monitoring',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Inicio
          </button>
          <h2>Audit Trail</h2>
          <div style="margin-left:auto"><span class="live-badge">LIVE</span></div>
        </div>
        <div class="module-body">
          <div class="section-title">
            Registros de Auditoría
            <span class="count" id="audit-count"></span>
          </div>
          <div class="timeline" id="audit-timeline">
            <div style="padding:20px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
          </div>
        </div>
      </div>
    `;

    _fetchAndRender(container);
    poll(POLL_ID, '/audit/tail?n=60', 4000, (data) => {
      if (data) _render(container, data);
    });
  },

  destroy() { stop(POLL_ID); },
};

async function _fetchAndRender(container) {
  try {
    const r = await fetch('/audit/tail?n=60');
    const data = await r.json();
    _render(container, data);
  } catch (e) {
    const el = container.querySelector('#audit-timeline');
    if (el) el.innerHTML = `<div class="tl-entry error">
      <div class="tl-time">--:--</div>
      <div class="tl-icon">✕</div>
      <div class="tl-body"><div class="tl-msg">Error: ${_esc(e.message)}</div></div>
    </div>`;
  }
}

function _render(container, data) {
  const tl = container.querySelector('#audit-timeline');
  if (!tl) return;

  const lines = _extractLines(data);
  const countEl = container.querySelector('#audit-count');
  if (countEl) countEl.textContent = `${lines.length} registros`;

  if (lines.length === 0) {
    tl.innerHTML = `<div class="empty-state">
      <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5"><path d="M12 22s8-4 8-10V5l-8-3-8 3v7c0 6 8 10 8 10z"/></svg>
      <div class="empty-title">Sin registros de auditoría</div>
      <div class="empty-sub">El sistema no ha registrado eventos aún</div>
    </div>`;
    return;
  }

  tl.innerHTML = [...lines].reverse().map(line => {
    const text  = typeof line === 'string' ? line : JSON.stringify(line);
    const cls   = _parseLevel(text);
    // intenta extraer timestamp tipo [HH:MM:SS] o ISO
    const tsMatch = text.match(/\[?(\d{2}:\d{2}:\d{2})\]?/) || text.match(/(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2})/);
    const ts = tsMatch ? tsMatch[1].slice(0, 8) : '--:--';
    const clean = text.replace(/\[?\d{2}:\d{2}:\d{2}\]?\s*/, '').replace(/\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\S*\s*/, '');
    return `<div class="tl-entry ${cls}">
      <div class="tl-time">${_esc(ts)}</div>
      <div class="tl-icon">${_icon(cls)}</div>
      <div class="tl-body"><div class="tl-msg">${_esc(clean || text)}</div></div>
    </div>`;
  }).join('');
}

window.AtlasModuleAudit = { id: 'audit' };
