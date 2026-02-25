/**
 * ATLAS v4 — Autonomy & Governance Module
 * Unified control surface for autonomy daemon, tasks, and governance mode.
 */
import { poll, stop } from '../lib/polling.js';
import { get, set } from '../lib/state.js';

const POLL_ID = 'autonomy-module';

export default {
  id: 'autonomy',
  label: 'Autonomy & Governance',
  icon: 'cpu',
  category: 'control',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Home
          </button>
          <h2>Autonomy & Governance</h2>
        </div>
        <div class="module-body">
          <div class="config-section">
            <div class="config-section-title">Status</div>
            <div id="autonomy-status" class="config-row"><span class="config-key">Loading...</span><span class="config-val">--</span></div>
            <div id="governance-status" class="config-row"><span class="config-key">Governance</span><span class="config-val">--</span></div>
          </div>

          <div class="config-section">
            <div class="config-section-title">Actions</div>
            <div style="display:flex;gap:10px;flex-wrap:wrap">
              <button class="chip-btn" id="btn-daemon-start">Start daemon</button>
              <button class="chip-btn" id="btn-daemon-stop">Stop daemon</button>
              <button class="chip-btn" id="btn-refresh">Refresh</button>
            </div>
            <div style="margin-top:10px;color:var(--text-muted);font-size:12px" id="autonomy-msg"></div>
          </div>

          <div class="config-section">
            <div class="config-section-title">Tasks (top)</div>
            <div id="autonomy-tasks" class="bitacora-list"></div>
          </div>
        </div>
      </div>
    `;

    container.querySelector('#btn-daemon-start')?.addEventListener('click', () => _daemon('start'));
    container.querySelector('#btn-daemon-stop')?.addEventListener('click', () => _daemon('stop'));
    container.querySelector('#btn-refresh')?.addEventListener('click', () => _refresh(container));

    _refresh(container);

    poll(POLL_ID, '/api/autonomy/status', 5000, (data) => {
      if (data) _renderAutonomy(container, data);
    });
    poll(POLL_ID + ':gov', '/governance/status', 5000, (data) => {
      if (data) _renderGovernance(container, data);
    });
    poll(POLL_ID + ':tasks', '/api/autonomy/tasks', 7000, (data) => {
      if (data) _renderTasks(container, data);
    });
  },

  destroy() {
    stop(POLL_ID);
    stop(POLL_ID + ':gov');
    stop(POLL_ID + ':tasks');
  },
};

async function _refresh(container) {
  try {
    const [a, g, t] = await Promise.all([
      fetch('/api/autonomy/status').then(r => r.json()).catch(() => null),
      fetch('/governance/status').then(r => r.json()).catch(() => null),
      fetch('/api/autonomy/tasks').then(r => r.json()).catch(() => null),
    ]);
    if (a) _renderAutonomy(container, a);
    if (g) _renderGovernance(container, g);
    if (t) _renderTasks(container, t);
  } catch (e) {
    _msg(container, `Error: ${e.message}`, 'error');
  }
}

function _msg(container, text, type = 'info') {
  const el = container.querySelector('#autonomy-msg');
  if (!el) return;
  el.textContent = text || '';
  if (text) window.AtlasToast?.show(text, type === 'error' ? 'error' : 'info');
}

async function _daemon(action) {
  try {
    const url = action === 'start' ? '/api/autonomy/daemon/start' : '/api/autonomy/daemon/stop';
    const res = await fetch(url, { method: 'POST' });
    const data = await res.json().catch(() => ({}));
    if (!res.ok || data.ok === false) throw new Error(data.error || `HTTP ${res.status}`);
    window.AtlasToast?.show(`Daemon ${action}: OK`, 'success');
  } catch (e) {
    window.AtlasToast?.show(`Daemon ${action} failed: ${e.message}`, 'error');
  }
}

function _renderAutonomy(container, data) {
  const el = container.querySelector('#autonomy-status');
  if (!el) return;
  const ok = data.ok !== false;
  const mode = (data.data || data).mode || (data.data || data).state || '--';
  const daemon = (data.data || data).daemon || (data.data || data).daemon_status || {};
  const dState = typeof daemon === 'string' ? daemon : (daemon.running ? 'running' : (daemon.status || '--'));
  el.innerHTML = `<span class="config-key">Autonomy</span><span class="config-val">${_esc(`${ok ? 'ok' : 'warn'} / daemon:${dState} / mode:${mode}`)}</span>`;
  set('lastActivity', `Autonomy: ${ok ? 'OK' : 'WARN'}`);
}

function _renderGovernance(container, data) {
  const el = container.querySelector('#governance-status');
  if (!el) return;
  const mode = (data.data || data).mode || data.mode || '--';
  const emergency = (data.data || data).emergency || false;
  el.innerHTML = `<span class="config-key">Governance</span><span class="config-val">${_esc(`${mode}${emergency ? ' (emergency)' : ''}`)}</span>`;
}

function _renderTasks(container, data) {
  const el = container.querySelector('#autonomy-tasks');
  if (!el) return;
  const items = (data.data || data).tasks || (data.data || data).items || data.tasks || [];
  if (!Array.isArray(items) || items.length === 0) {
    el.innerHTML = `<div class="bitacora-entry"><span class="bitacora-msg" style="color:var(--text-muted)">No tasks</span></div>`;
    return;
  }
  el.innerHTML = items.slice(0, 12).map(t => {
    const id = t.id || t.task_id || '--';
    const status = t.status || t.state || 'unknown';
    const title = t.title || t.name || t.kind || '';
    return `<div class="bitacora-entry">
      <span class="bitacora-time">${_esc(String(status).slice(0, 8))}</span>
      <span class="bitacora-msg"><span class="bitacora-level info">${_esc(String(status))}</span>${_esc(`${id} ${title}`)}</span>
    </div>`;
  }).join('');
}

function _esc(s) { const d = document.createElement('span'); d.textContent = s || ''; return d.innerHTML; }

window.AtlasModuleAutonomy = { id: 'autonomy' };
