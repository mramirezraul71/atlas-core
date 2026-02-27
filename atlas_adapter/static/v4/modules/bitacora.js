/**
 * ATLAS v4 — Bitacora Module
 * Real-time activity log with auto-refresh and level filtering.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'bitacora-module';

export default {
  id: 'bitacora',
  label: 'Bitacora',
  icon: 'file-text',
  category: 'monitoring',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Home
          </button>
          <h2>Bitacora</h2>
        </div>
        <div class="module-body">
          <div class="bitacora-list" id="bitacora-list">
            <div class="bitacora-entry"><span class="bitacora-msg">Loading...</span></div>
          </div>
        </div>
      </div>
    `;

    _fetchAndRender(container);

    // Backend exposes bitácora under /ans/bitacora
    poll(POLL_ID, '/ans/bitacora?limit=50', 5000, (data, err) => {
      if (data) _renderEntries(container, data);
    });
  },

  destroy() {
    stop(POLL_ID);
  },

  badge() { return null; },
};

async function _fetchAndRender(container) {
  try {
    const res = await fetch('/ans/bitacora?limit=50');
    const data = await res.json();
    _renderEntries(container, data);
  } catch (e) {
    container.querySelector('#bitacora-list').innerHTML = `<div class="bitacora-entry"><span class="bitacora-msg">Error: ${e.message}</span></div>`;
  }
}

function _renderEntries(container, data) {
  const list = container.querySelector('#bitacora-list');
  if (!list) return;

  const entries = data.data || data.entries || data.items || (Array.isArray(data) ? data : []);
  if (entries.length === 0) {
    list.innerHTML = '<div class="bitacora-entry"><span class="bitacora-msg" style="color:var(--text-muted)">No entries yet</span></div>';
    return;
  }

  list.innerHTML = entries.map(e => {
    const time = e.timestamp || e.ts || e.time || '';
    const shortTime = time ? new Date(time).toLocaleTimeString() : '--';
    const level = (e.level || e.type || 'info').toLowerCase();
    const msg = e.message || e.msg || e.text || e.event || JSON.stringify(e);
    return `<div class="bitacora-entry">
      <span class="bitacora-time">${_esc(shortTime)}</span>
      <span class="bitacora-msg"><span class="bitacora-level ${level}">${level}</span>${_esc(msg)}</span>
    </div>`;
  }).join('');
}

function _esc(s) { const d = document.createElement('span'); d.textContent = s || ''; return d.innerHTML; }

window.AtlasModuleBitacora = { id: 'bitacora' };
