/**
 * ATLAS v4 — Healing History Module
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'healing-module';

export default {
  id: 'healing',
  label: 'Healing',
  icon: 'activity',
  category: 'monitoring',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Home
          </button>
          <h2>Healing</h2>
        </div>
        <div class="module-body">
          <div class="config-section">
            <div class="config-section-title">Recent healing actions</div>
            <pre class="codebox" id="healing-box">Loading...</pre>
          </div>
        </div>
      </div>
    `;
    _refresh(container);
    poll(POLL_ID, '/api/healing/history?limit=10', 6000, (data) => {
      if (data) _render(container, data);
    });
  },

  destroy() { stop(POLL_ID); },
};

async function _refresh(container) {
  try {
    const res = await fetch('/api/healing/history?limit=10');
    const data = await res.json();
    _render(container, data);
  } catch (e) {
    const el = container.querySelector('#healing-box');
    if (el) el.textContent = `Error: ${e.message}`;
  }
}

function _render(container, data) {
  const el = container.querySelector('#healing-box');
  if (!el) return;
  const payload = data.data || data;
  const items = payload.items || payload.history || payload.events || (Array.isArray(payload) ? payload : []);
  el.textContent = JSON.stringify(items, null, 2);
}

window.AtlasModuleHealing = { id: 'healing' };
