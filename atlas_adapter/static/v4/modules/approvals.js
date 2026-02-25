/**
 * ATLAS v4 — Approvals Module
 * Displays pending approvals and basic details.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'approvals-module';

export default {
  id: 'approvals',
  label: 'Approvals',
  icon: 'shield',
  category: 'control',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Home
          </button>
          <h2>Approvals</h2>
        </div>
        <div class="module-body">
          <div class="config-section">
            <div class="config-section-title">Pending</div>
            <div id="approvals-list" class="bitacora-list"></div>
          </div>
        </div>
      </div>
    `;

    _refresh(container);
    poll(POLL_ID, '/approvals/pending?limit=50', 5000, (data) => {
      if (data) _render(container, data);
    });
  },

  destroy() {
    stop(POLL_ID);
  },
};

async function _refresh(container) {
  try {
    const res = await fetch('/approvals/pending?limit=50');
    const data = await res.json();
    _render(container, data);
  } catch (e) {
    const el = container.querySelector('#approvals-list');
    if (el) el.innerHTML = `<div class="bitacora-entry"><span class="bitacora-msg">Error: ${_esc(e.message)}</span></div>`;
  }
}

function _render(container, data) {
  const el = container.querySelector('#approvals-list');
  if (!el) return;

  const payload = data.data || data;
  const items = payload.items || payload.approvals || payload.pending || (Array.isArray(payload) ? payload : []);

  if (!Array.isArray(items) || items.length === 0) {
    el.innerHTML = `<div class="bitacora-entry"><span class="bitacora-msg" style="color:var(--text-muted)">No pending approvals</span></div>`;
    return;
  }

  el.innerHTML = items.slice(0, 80).map(a => {
    const id = a.id || a.approval_id || a.task_id || '--';
    const kind = a.kind || a.type || a.module || 'approval';
    const created = a.created_at || a.ts || a.timestamp || '';
    const shortTime = created ? new Date(created).toLocaleTimeString() : '--';
    const summary = a.summary || a.title || a.reason || a.description || '';
    return `<div class="bitacora-entry">
      <span class="bitacora-time">${_esc(shortTime)}</span>
      <span class="bitacora-msg"><span class="bitacora-level warn">${_esc(kind)}</span>${_esc(`${id} ${summary}`)}</span>
    </div>`;
  }).join('');
}

function _esc(s) { const d = document.createElement('span'); d.textContent = s || ''; return d.innerHTML; }

window.AtlasModuleApprovals = { id: 'approvals' };
