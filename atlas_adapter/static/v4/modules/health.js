/**
 * ATLAS v4 — Health Module
 * Real-time system health monitoring with auto-refresh.
 */
import { poll, stop } from '../lib/polling.js';
import { set } from '../lib/state.js';

const POLL_ID = 'health-module';

export default {
  id: 'health',
  label: 'System Health',
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
          <h2>System Health</h2>
        </div>
        <div class="module-body">
          <div class="health-grid" id="health-grid">
            <div class="health-card"><div class="health-card-title">Loading...</div></div>
          </div>
        </div>
      </div>
    `;

    _fetchAndRender(container);

    poll(POLL_ID, '/health', 5000, (data, err) => {
      if (data) _renderCards(container, data);
    });
  },

  destroy() {
    stop(POLL_ID);
  },

  badge() { return null; },
};

async function _fetchAndRender(container) {
  try {
    const res = await fetch('/health');
    const data = await res.json();
    _renderCards(container, data);
    set('health', data);
  } catch (e) {
    container.querySelector('#health-grid').innerHTML = `<div class="health-card"><div class="health-card-title">Error loading health</div><div class="health-card-sub">${e.message}</div></div>`;
  }
}

function _renderCards(container, data) {
  const grid = container.querySelector('#health-grid');
  if (!grid) return;

  const status = data.status || data.ok ? 'ok' : 'warn';
  const uptime = data.uptime || data.uptime_human || '--';
  const version = data.version || '--';
  const modules = data.modules || {};

  let cards = `
    <div class="health-card">
      <div class="health-card-title">Overall Status</div>
      <div class="health-card-value"><span class="health-status-dot ${status === 'ok' || data.ok ? 'ok' : 'warn'}"></span>${data.ok ? 'Healthy' : 'Degraded'}</div>
      <div class="health-card-sub">Last checked: ${new Date().toLocaleTimeString()}</div>
    </div>
    <div class="health-card">
      <div class="health-card-title">Uptime</div>
      <div class="health-card-value">${_esc(uptime)}</div>
    </div>
    <div class="health-card">
      <div class="health-card-title">Version</div>
      <div class="health-card-value" style="font-size:20px">${_esc(version)}</div>
    </div>
  `;

  if (data.cpu_percent !== undefined) {
    cards += `<div class="health-card"><div class="health-card-title">CPU</div><div class="health-card-value">${data.cpu_percent}%</div></div>`;
  }
  if (data.memory_mb !== undefined || data.mem_percent !== undefined) {
    cards += `<div class="health-card"><div class="health-card-title">Memory</div><div class="health-card-value">${data.mem_percent || '--'}%</div><div class="health-card-sub">${data.memory_mb || '--'} MB</div></div>`;
  }

  for (const [name, info] of Object.entries(modules)) {
    const mStatus = typeof info === 'object' ? (info.status || (info.ok ? 'ok' : 'warn')) : info;
    const dotClass = mStatus === 'ok' || mStatus === true ? 'ok' : mStatus === 'warn' ? 'warn' : 'error';
    cards += `<div class="health-card">
      <div class="health-card-title">${_esc(name)}</div>
      <div class="health-card-value" style="font-size:16px"><span class="health-status-dot ${dotClass}"></span>${typeof info === 'object' ? _esc(info.status || 'active') : _esc(String(info))}</div>
    </div>`;
  }

  grid.innerHTML = cards;
  set('health', { uptime, ok: data.ok });
}

function _esc(s) {
  const d = document.createElement('span');
  d.textContent = s || '';
  return d.innerHTML;
}

window.AtlasModuleHealth = { id: 'health' };
