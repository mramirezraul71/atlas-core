/**
 * ATLAS v4.2 — Autonomy & Governance Module
 * Panel de control con mode badge, tasks list y acciones del daemon.
 */
import { poll, stop } from '../lib/polling.js';
import { set } from '../lib/state.js';

const POLL_ID = 'autonomy-module';

function _esc(s) { const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML; }

function _modeBadge(mode) {
  const m = (mode + '').toLowerCase();
  if (m.includes('emergency')) return `<span class="mode-badge emergency">⚠ Emergency</span>`;
  if (m.includes('auto'))      return `<span class="mode-badge autonomous">◎ Autónomo</span>`;
  if (m.includes('supervised') || m.includes('semi')) return `<span class="mode-badge supervised">◑ Supervisado</span>`;
  return `<span class="mode-badge manual">◻ Manual</span>`;
}

function _taskStatus(status) {
  const s = (status + '').toLowerCase();
  if (s === 'running' || s === 'in_progress') return 'running';
  if (s === 'done' || s === 'completed' || s === 'success') return 'done';
  if (s === 'failed' || s === 'error') return 'failed';
  return 'pending';
}

export default {
  id: 'autonomy',
  label: 'Autonomy',
  icon: 'cpu',
  category: 'control',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Inicio
          </button>
          <h2>Autonomy &amp; Governance</h2>
        </div>
        <div class="module-body">

          <!-- Status row -->
          <div class="stat-row" style="margin-bottom:16px">
            <div class="stat-card hero">
              <div class="stat-card-label">Modo de Gobierno</div>
              <div id="gov-mode" style="margin-top:6px">
                <span class="mode-badge manual">Cargando...</span>
              </div>
              <div class="stat-card-sub" id="gov-extra" style="margin-top:8px"></div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Daemon ANS</div>
              <div class="stat-card-value" id="daemon-state">--</div>
              <div class="stat-card-sub" id="daemon-sub"></div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Tareas Activas</div>
              <div class="stat-card-value accent" id="task-count">--</div>
              <div class="stat-card-sub">En cola</div>
            </div>
          </div>

          <!-- Actions -->
          <div class="section-title">Acciones del Daemon</div>
          <div class="action-bar">
            <button class="action-btn success" id="btn-start">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polygon points="5 3 19 12 5 21 5 3"/></svg>
              Iniciar
            </button>
            <button class="action-btn danger" id="btn-stop">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="3" y="3" width="18" height="18" rx="2"/></svg>
              Detener
            </button>
            <button class="action-btn" id="btn-refresh">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="23 4 23 10 17 10"/><path d="M20.49 15a9 9 0 11-2.12-9.36L23 10"/></svg>
              Actualizar
            </button>
            <div id="au-msg" style="font-size:11px;color:var(--text-muted);align-self:center"></div>
          </div>

          <!-- Task list -->
          <div class="section-title" style="margin-top:20px">
            Cola de Tareas
            <span class="count" id="task-count-lbl"></span>
          </div>
          <div class="task-list" id="autonomy-tasks">
            <div style="padding:20px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
          </div>

        </div>
      </div>
    `;

    container.querySelector('#btn-start')?.addEventListener('click', () => _daemon('start', container));
    container.querySelector('#btn-stop')?.addEventListener('click',  () => _daemon('stop', container));
    container.querySelector('#btn-refresh')?.addEventListener('click', () => _refresh(container));

    _refresh(container);
    poll(POLL_ID,          '/api/autonomy/status',  5000, (d) => { if (d) _renderAutonomy(container, d); });
    poll(POLL_ID + ':gov', '/governance/status',    5000, (d) => { if (d) _renderGovernance(container, d); });
    poll(POLL_ID + ':t',   '/api/autonomy/tasks',   7000, (d) => { if (d) _renderTasks(container, d); });
  },

  destroy() {
    stop(POLL_ID);
    stop(POLL_ID + ':gov');
    stop(POLL_ID + ':t');
  },
};

async function _refresh(container) {
  const [a, g, t] = await Promise.all([
    fetch('/api/autonomy/status').then(r => r.json()).catch(() => null),
    fetch('/governance/status').then(r => r.json()).catch(() => null),
    fetch('/api/autonomy/tasks').then(r => r.json()).catch(() => null),
  ]);
  if (a) _renderAutonomy(container, a);
  if (g) _renderGovernance(container, g);
  if (t) _renderTasks(container, t);
}

async function _daemon(action, container) {
  const btn = container.querySelector(action === 'start' ? '#btn-start' : '#btn-stop');
  if (btn) btn.disabled = true;
  const msg = container.querySelector('#au-msg');
  if (msg) msg.textContent = `${action === 'start' ? 'Iniciando' : 'Deteniendo'}...`;
  try {
    const url = action === 'start' ? '/api/autonomy/daemon/start' : '/api/autonomy/daemon/stop';
    const r = await fetch(url, { method: 'POST' });
    const data = await r.json().catch(() => ({}));
    if (!r.ok || data.ok === false) throw new Error(data.error || `HTTP ${r.status}`);
    window.AtlasToast?.show(`Daemon ${action}: OK`, 'success');
    if (msg) msg.textContent = `Daemon ${action}: OK`;
    setTimeout(() => _refresh(container), 1500);
  } catch (e) {
    window.AtlasToast?.show(e.message, 'error');
    if (msg) msg.textContent = `Error: ${e.message}`;
  } finally {
    if (btn) btn.disabled = false;
  }
}

function _renderAutonomy(container, data) {
  const p = data?.data ?? data;
  const daemon = p?.daemon || p?.daemon_status || {};
  const running = typeof daemon === 'string' ? (daemon === 'running') : (daemon.running === true || daemon.status === 'running');
  const txt = (id, v) => { const el = container.querySelector(id); if (el) el.textContent = v; };
  txt('#daemon-state', running ? 'Activo' : 'Detenido');
  const stateEl = container.querySelector('#daemon-state');
  if (stateEl) stateEl.style.color = running ? 'var(--accent-green)' : 'var(--accent-red)';
  txt('#daemon-sub', p?.uptime ? `Uptime: ${p.uptime}` : '');
  set('lastActivity', `Autonomy: ${running ? 'running' : 'stopped'}`);
}

function _renderGovernance(container, data) {
  const p = data?.data ?? data;
  const mode = p?.mode || p?.governance_mode || 'manual';
  const emergency = p?.emergency === true;
  const modeEl = container.querySelector('#gov-mode');
  if (modeEl) modeEl.innerHTML = _modeBadge(emergency ? 'emergency' : mode);
  const extraEl = container.querySelector('#gov-extra');
  if (extraEl) extraEl.textContent = emergency ? '⚠ Modo emergencia activo' : (p?.reason || '');
}

function _renderTasks(container, data) {
  const p = data?.data ?? data;
  const items = p?.tasks || p?.items || (Array.isArray(p) ? p : []);
  const el = container.querySelector('#autonomy-tasks');
  const countEl = container.querySelector('#task-count');
  const countLbl = container.querySelector('#task-count-lbl');
  if (countEl) countEl.textContent = items.length;
  if (countLbl) countLbl.textContent = `${items.length} tareas`;

  if (!el) return;
  if (items.length === 0) {
    el.innerHTML = `<div class="empty-state">
      <svg width="28" height="28" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5"><path d="M9 11l3 3L22 4"/><path d="M21 12v7a2 2 0 01-2 2H5a2 2 0 01-2-2V5a2 2 0 012-2h11"/></svg>
      <div class="empty-title">Sin tareas en cola</div>
      <div class="empty-sub">El daemon no tiene tareas pendientes</div>
    </div>`;
    return;
  }

  el.innerHTML = items.slice(0, 20).map(t => {
    const st = _taskStatus(t.status || t.state);
    const name = _esc(t.title || t.name || t.kind || t.id || '--');
    const detail = _esc(t.description || t.reason || t.id || '');
    const ts = t.created_at || t.ts || t.timestamp || '';
    const tsStr = ts ? new Date(ts).toLocaleTimeString() : '';
    return `<div class="task-item">
      <div class="task-status-dot ${st}"></div>
      <div class="task-info">
        <div class="task-name">${name}</div>
        ${detail ? `<div class="task-detail">${detail}</div>` : ''}
      </div>
      ${tsStr ? `<div class="task-ts">${_esc(tsStr)}</div>` : ''}
    </div>`;
  }).join('');
}

window.AtlasModuleAutonomy = { id: 'autonomy' };
