/**
 * ATLAS v4 — Memory Status Module
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'memory-module';

export default {
  id: 'memory',
  label: 'Cognitive Memory',
  icon: 'brain',
  category: 'monitoring',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Home
          </button>
          <h2>Cognitive Memory</h2>
        </div>
        <div class="module-body">
          <div class="config-section">
            <div class="config-section-title">Status</div>
            <div id="memory-cards" class="health-grid"></div>
          </div>
        </div>
      </div>
    `;

    _refresh(container);
    poll(POLL_ID, '/api/cognitive-memory/lifelog/status', 8000, (d) => { if (d) _render(container); });
  },

  destroy() { stop(POLL_ID); },
};

async function _refresh(container) {
  try {
    await _render(container);
  } catch (e) {
    const el = container.querySelector('#memory-cards');
    if (el) el.innerHTML = `<div class="health-card"><div class="health-card-title">Error</div><div class="health-card-sub">${_esc(e.message)}</div></div>`;
  }
}

async function _render(container) {
  const grid = container.querySelector('#memory-cards');
  if (!grid) return;

  const [lifelog, world, auto] = await Promise.all([
    fetch('/api/cognitive-memory/lifelog/status').then(r => r.json()).catch(() => null),
    fetch('/api/cognitive-memory/world-model/status').then(r => r.json()).catch(() => null),
    fetch('/api/cognitive-memory/autobiographical/status').then(r => r.json()).catch(() => null),
  ]);

  const cards = [];
  cards.push(_card('Lifelog', lifelog));
  cards.push(_card('World Model', world));
  cards.push(_card('Autobiographical', auto));
  grid.innerHTML = cards.join('');
}

function _card(title, data) {
  if (!data) return `<div class="health-card"><div class="health-card-title">${_esc(title)}</div><div class="health-card-sub">Unavailable</div></div>`;
  const ok = data.ok !== false;
  const payload = data.data || data;
  const detail = payload.status || payload.state || payload.mode || (ok ? 'ok' : 'warn');
  const extra = Object.entries(payload).slice(0, 4).map(([k, v]) => `${k}=${typeof v === 'object' ? '[obj]' : String(v)}`).join(' • ');
  return `<div class="health-card">
    <div class="health-card-title">${_esc(title)}</div>
    <div class="health-card-value" style="font-size:20px"><span class="health-status-dot ${ok ? 'ok' : 'warn'}"></span>${_esc(String(detail))}</div>
    <div class="health-card-sub">${_esc(extra)}</div>
  </div>`;
}

function _esc(s) { const d = document.createElement('span'); d.textContent = s || ''; return d.innerHTML; }

window.AtlasModuleMemory = { id: 'memory' };

