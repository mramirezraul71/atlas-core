/**
 * ATLAS v4 — Audit Tail Module
 * Shows recent audit log lines via /audit/tail.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'audit-module';

export default {
  id: 'audit',
  label: 'Audit Tail',
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
          <h2>Audit Tail</h2>
        </div>
        <div class="module-body">
          <div class="config-section">
            <div class="config-section-title">Latest</div>
            <pre class="codebox" id="audit-box">Loading...</pre>
          </div>
        </div>
      </div>
    `;

    _refresh(container);
    poll(POLL_ID, '/audit/tail?n=60', 4000, (data) => {
      if (data) _render(container, data);
    });
  },

  destroy() {
    stop(POLL_ID);
  },
};

async function _refresh(container) {
  try {
    const res = await fetch('/audit/tail?n=60');
    const data = await res.json();
    _render(container, data);
  } catch (e) {
    const el = container.querySelector('#audit-box');
    if (el) el.textContent = `Error: ${e.message}`;
  }
}

function _render(container, data) {
  const el = container.querySelector('#audit-box');
  if (!el) return;
  const payload = data.data || data;
  const lines = payload.lines || payload.items || payload.events || payload || [];
  if (Array.isArray(lines)) {
    el.textContent = lines.map(x => typeof x === 'string' ? x : JSON.stringify(x)).join('\n');
  } else if (typeof lines === 'string') {
    el.textContent = lines;
  } else {
    el.textContent = JSON.stringify(lines, null, 2);
  }
}

window.AtlasModuleAudit = { id: 'audit' };

