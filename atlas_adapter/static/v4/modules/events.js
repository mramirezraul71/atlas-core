/**
 * ATLAS v4 — Event Bus History Module
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'events-module';

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
            Home
          </button>
          <h2>Event Bus</h2>
        </div>
        <div class="module-body">
          <div class="config-section">
            <div class="config-section-title">Recent events</div>
            <pre class="codebox" id="events-box">Loading...</pre>
          </div>
        </div>
      </div>
    `;
    _refresh(container);
    poll(POLL_ID, '/api/kernel/event-bus/history?limit=30', 5000, (data) => {
      if (data) _render(container, data);
    });
  },

  destroy() { stop(POLL_ID); },
};

async function _refresh(container) {
  try {
    const res = await fetch('/api/kernel/event-bus/history?limit=30');
    const data = await res.json();
    _render(container, data);
  } catch (e) {
    const el = container.querySelector('#events-box');
    if (el) el.textContent = `Error: ${e.message}`;
  }
}

function _render(container, data) {
  const el = container.querySelector('#events-box');
  if (!el) return;
  const payload = data.data || data;
  const events = payload.items || payload.events || payload.history || (Array.isArray(payload) ? payload : []);
  el.textContent = JSON.stringify(events, null, 2);
}

window.AtlasModuleEvents = { id: 'events' };

