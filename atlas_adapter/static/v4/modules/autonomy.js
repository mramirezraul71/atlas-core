/**
 * ATLAS v4.2 — Autonomy & Governance Module
 * Panel completo: modo de gobierno, emergency stop, daemon ANS,
 * modo agresivo, reglas de gobernanza, log de gobernanza y cola de tareas.
 */
import { poll, stop } from '../lib/polling.js';
import { set } from '../lib/state.js';

const POLL_ID = 'autonomy-module';

function _esc(s) { const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML; }
function _time(ts) { if (!ts) return '--'; try { return new Date(ts).toLocaleTimeString(); } catch { return String(ts); } }

function _modeBadge(mode) {
  const m = (mode + '').toLowerCase();
  if (m.includes('emergency')) return `<span class="mode-badge emergency">⚠ Emergency</span>`;
  if (m.includes('auto') || m === 'growth') return `<span class="mode-badge autonomous">◎ GROWTH / Autónomo</span>`;
  if (m.includes('governed') || m === 'governed') return `<span class="mode-badge supervised">◑ GOVERNED / Supervisado</span>`;
  if (m.includes('learning')) return `<span class="mode-badge supervised" style="background:rgba(163,113,247,.15);color:var(--accent-purple)">📚 LEARNING</span>`;
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
          <button class="back-btn" id="au-back">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Dashboard
          </button>
          <h2>Autonomy &amp; Governance</h2>
          <div style="margin-left:auto;display:flex;gap:8px;align-items:center">
            <span class="live-badge">LIVE</span>
          </div>
        </div>

        <div class="module-body">

          <!-- KPIs superiores -->
          <div class="stat-row" style="margin-bottom:20px">
            <div class="stat-card hero">
              <div class="stat-card-label">Modo de Gobierno</div>
              <div id="gov-mode" style="margin-top:6px"><span class="mode-badge manual">Cargando...</span></div>
              <div class="stat-card-sub" id="gov-reason" style="margin-top:6px"></div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Daemon ANS</div>
              <div class="stat-card-value" id="daemon-state">--</div>
              <div class="stat-card-sub" id="daemon-uptime"></div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Tareas en cola</div>
              <div class="stat-card-value accent" id="task-count">--</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Modo agresivo</div>
              <div class="stat-card-value" id="aggr-state">--</div>
              <div class="stat-card-sub" id="aggr-sub"></div>
            </div>
          </div>

          <!-- === SECCIÓN 1: CONTROL DE GOBERNANZA === -->
          <div class="section-title">Control de Gobernanza</div>
          <div class="action-bar" style="flex-wrap:wrap;gap:8px;margin-bottom:8px">
            <button class="action-btn primary" id="btn-gov-growth">
              <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="22 7 13.5 15.5 8.5 10.5 2 17"/><polyline points="16 7 22 7 22 13"/></svg>
              GROWTH
            </button>
            <button class="action-btn" id="btn-gov-governed" style="color:var(--accent-orange);border-color:var(--accent-orange)">
              <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M12 22s8-4 8-10V5l-8-3-8 3v7c0 6 8 10 8 10z"/></svg>
              GOVERNED
            </button>
            <button class="action-btn" id="btn-gov-learning" style="color:var(--accent-purple);border-color:var(--accent-purple)">
              <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M2 3h6a4 4 0 014 4v14a3 3 0 00-3-3H2z"/><path d="M22 3h-6a4 4 0 00-4 4v14a3 3 0 013-3h7z"/></svg>
              LEARNING
            </button>
            <div style="flex:1"></div>
            <button class="action-btn danger" id="btn-emergency" style="font-weight:700;letter-spacing:.5px">
              <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M10.29 3.86L1.82 18a2 2 0 001.71 3h16.94a2 2 0 001.71-3L13.71 3.86a2 2 0 00-3.42 0z"/><line x1="12" y1="9" x2="12" y2="13"/><line x1="12" y1="17" x2="12.01" y2="17"/></svg>
              EMERGENCY STOP
            </button>
          </div>
          <div id="gov-msg" style="font-size:11px;color:var(--text-muted);padding:4px 0 8px"></div>

          <!-- Reglas de gobernanza (desplegable) -->
          <details style="margin-bottom:16px">
            <summary style="cursor:pointer;font-size:12px;color:var(--text-muted);padding:4px 0;user-select:none">
              Ver reglas de gobernanza
            </summary>
            <div id="gov-rules" style="margin-top:8px">
              <div style="padding:12px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
            </div>
          </details>

          <!-- Log de gobernanza -->
          <details style="margin-bottom:20px">
            <summary style="cursor:pointer;font-size:12px;color:var(--text-muted);padding:4px 0;user-select:none">
              Log de gobernanza
            </summary>
            <div id="gov-log" class="timeline" style="margin-top:8px;max-height:200px;overflow-y:auto">
              <div style="padding:12px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
            </div>
          </details>

          <!-- === SECCIÓN 2: DAEMON ANS === -->
          <div class="section-title">Control del Daemon ANS</div>
          <div class="action-bar" style="margin-bottom:20px">
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
            <span id="au-msg" style="font-size:11px;color:var(--text-muted);align-self:center"></span>
          </div>

          <!-- === SECCIÓN 3: MODO AGRESIVO === -->
          <div class="section-title">Modo Agresivo de Autonomía</div>
          <div style="margin-bottom:20px">
            <div id="aggr-config-view">
              <div style="padding:12px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
            </div>
            <div style="display:flex;gap:8px;flex-wrap:wrap;margin-top:12px" id="aggr-form" style="display:none">
              <div style="display:flex;flex-direction:column;gap:4px">
                <label style="font-size:11px;color:var(--text-muted)">Idle segundos</label>
                <input id="aggr-idle" type="number" min="10" max="3600" style="width:100px;padding:6px 10px;background:var(--surface-1);border:1px solid var(--border);border-radius:6px;color:var(--text-primary);font-size:13px">
              </div>
              <div style="display:flex;flex-direction:column;gap:4px">
                <label style="font-size:11px;color:var(--text-muted)">Intervalo segundos</label>
                <input id="aggr-interval" type="number" min="5" max="600" style="width:100px;padding:6px 10px;background:var(--surface-1);border:1px solid var(--border);border-radius:6px;color:var(--text-primary);font-size:13px">
              </div>
              <div style="display:flex;align-items:flex-end;gap:6px">
                <button class="action-btn primary" id="btn-aggr-save">Guardar</button>
                <button class="action-btn" id="btn-aggr-toggle">Toggle ON/OFF</button>
              </div>
            </div>
          </div>

          <!-- === SECCIÓN 4: COLA DE TAREAS === -->
          <div class="section-title">
            Cola de Tareas
            <span class="count" id="task-count-lbl"></span>
          </div>
          <div class="task-list" id="autonomy-tasks">
            <div style="padding:20px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
          </div>

        </div>
      </div>
    `;

    // Navegación
    container.querySelector('#au-back')?.addEventListener('click', () => { location.hash = '/'; });

    // Governance mode
    container.querySelector('#btn-gov-growth')?.addEventListener('click',    () => _setMode('growth', container));
    container.querySelector('#btn-gov-governed')?.addEventListener('click',  () => _setMode('governed', container));
    container.querySelector('#btn-gov-learning')?.addEventListener('click',  () => _setMode('learning', container));
    container.querySelector('#btn-emergency')?.addEventListener('click',     () => _emergency(container));

    // Daemon
    container.querySelector('#btn-start')?.addEventListener('click',   () => _daemon('start', container));
    container.querySelector('#btn-stop')?.addEventListener('click',    () => _daemon('stop', container));
    container.querySelector('#btn-refresh')?.addEventListener('click', () => _refresh(container));

    // Aggressive mode
    container.querySelector('#btn-aggr-save')?.addEventListener('click',   () => _saveAggr(container));
    container.querySelector('#btn-aggr-toggle')?.addEventListener('click', () => _toggleAggr(container));

    // Lazy-load rules/log on details open
    container.querySelector('details:first-of-type')?.addEventListener('toggle', e => {
      if (e.target.open) _loadRules(container);
    });
    container.querySelectorAll('details')[1]?.addEventListener('toggle', e => {
      if (e.target.open) _loadGovLog(container);
    });

    _refresh(container);
    _loadAggrConfig(container);

    poll(POLL_ID,          '/api/autonomy/status',              5000, (d) => { if (d) _renderAutonomy(container, d); });
    poll(POLL_ID + ':gov', '/governance/status',                5000, (d) => { if (d) _renderGovernance(container, d); });
    poll(POLL_ID + ':t',   '/api/autonomy/tasks',               7000, (d) => { if (d) _renderTasks(container, d); });
    poll(POLL_ID + ':ag',  '/api/autonomy/aggressive/config',   10000,(d) => { if (d) _renderAggrConfig(container, d); });
  },

  destroy() {
    stop(POLL_ID);
    stop(POLL_ID + ':gov');
    stop(POLL_ID + ':t');
    stop(POLL_ID + ':ag');
  },
};

/* ─── Refresh general ───────────────────────────────────────────────── */

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

/* ─── Renderizadores de estado ──────────────────────────────────────── */

function _renderAutonomy(container, data) {
  const p = data?.data ?? data;
  const daemon = p?.daemon || p?.daemon_status || {};
  const running = typeof daemon === 'string'
    ? daemon === 'running'
    : (daemon.running === true || daemon.status === 'running');
  const stateEl = container.querySelector('#daemon-state');
  if (stateEl) {
    stateEl.textContent = running ? 'Activo' : 'Detenido';
    stateEl.style.color = running ? 'var(--accent-green)' : 'var(--accent-red)';
  }
  const upEl = container.querySelector('#daemon-uptime');
  if (upEl) upEl.textContent = p?.uptime ? `Uptime: ${p.uptime}` : '';
  set('lastActivity', `Autonomy: ${running ? 'running' : 'stopped'}`);
}

function _renderGovernance(container, data) {
  const p = data?.data ?? data;
  const mode = p?.mode || p?.governance_mode || p?.current_mode || 'manual';
  const emergency = p?.emergency === true || p?.emergency_stop === true;
  const modeEl = container.querySelector('#gov-mode');
  if (modeEl) modeEl.innerHTML = _modeBadge(emergency ? 'emergency' : mode);
  const reasonEl = container.querySelector('#gov-reason');
  if (reasonEl) reasonEl.textContent = emergency ? '⚠ Modo emergencia activo' : (p?.reason || p?.description || '');
}

function _renderTasks(container, data) {
  const p = data?.data ?? data;
  const items = p?.tasks || p?.items || (Array.isArray(p) ? p : []);
  const countEl = container.querySelector('#task-count');
  const countLbl = container.querySelector('#task-count-lbl');
  if (countEl) countEl.textContent = items.length;
  if (countLbl) countLbl.textContent = `${items.length} tareas`;

  const el = container.querySelector('#autonomy-tasks');
  if (!el) return;

  if (items.length === 0) {
    el.innerHTML = `<div class="empty-state">
      <svg width="28" height="28" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5"><path d="M9 11l3 3L22 4"/><path d="M21 12v7a2 2 0 01-2 2H5a2 2 0 01-2-2V5a2 2 0 012-2h11"/></svg>
      <div class="empty-title">Sin tareas en cola</div>
      <div class="empty-sub">El daemon no tiene tareas pendientes</div>
    </div>`;
    return;
  }

  el.innerHTML = items.slice(0, 25).map(t => {
    const st     = _taskStatus(t.status || t.state);
    const name   = _esc(t.title || t.name || t.kind || t.id || '--');
    const detail = _esc(t.description || t.reason || t.id || '');
    const ts     = _time(t.created_at || t.ts || t.timestamp);
    return `<div class="task-item">
      <div class="task-status-dot ${st}"></div>
      <div class="task-info">
        <div class="task-name">${name}</div>
        ${detail ? `<div class="task-detail">${detail}</div>` : ''}
      </div>
      ${ts ? `<div class="task-ts">${_esc(ts)}</div>` : ''}
    </div>`;
  }).join('');
}

/* ─── Aggressive config ─────────────────────────────────────────────── */

async function _loadAggrConfig(container) {
  try {
    const r = await fetch('/api/autonomy/aggressive/config');
    const data = await r.json().catch(() => null);
    if (data) _renderAggrConfig(container, data);
  } catch {}
}

function _renderAggrConfig(container, data) {
  const p = data?.data ?? data;
  const enabled = p?.enabled === true || p?.active === true;
  const idle = p?.idle_sec ?? p?.idle_seconds ?? 120;
  const interval = p?.interval_sec ?? p?.check_interval ?? 30;

  const stateEl = container.querySelector('#aggr-state');
  const subEl   = container.querySelector('#aggr-sub');
  if (stateEl) {
    stateEl.textContent = enabled ? 'Activo' : 'Inactivo';
    stateEl.style.color = enabled ? 'var(--accent-green)' : 'var(--text-muted)';
  }
  if (subEl) subEl.textContent = enabled ? `idle: ${idle}s / check: ${interval}s` : '';

  // Populate form
  const idleInput = container.querySelector('#aggr-idle');
  const intInput  = container.querySelector('#aggr-interval');
  if (idleInput && !idleInput.value) idleInput.value = idle;
  if (intInput && !intInput.value)   intInput.value  = interval;

  // Show form
  const form = container.querySelector('#aggr-form');
  if (form) form.style.display = 'flex';

  // Render current config as cards
  const view = container.querySelector('#aggr-config-view');
  if (view) {
    view.innerHTML = `<div class="stat-row" style="flex-wrap:wrap;gap:8px">
      <div class="stat-card" style="min-width:110px;flex:1 1 110px">
        <div class="stat-card-label">Estado</div>
        <div class="stat-card-value ${enabled ? 'green' : ''}" style="font-size:16px">${enabled ? 'Activo' : 'Inactivo'}</div>
      </div>
      <div class="stat-card" style="min-width:110px;flex:1 1 110px">
        <div class="stat-card-label">Idle (s)</div>
        <div class="stat-card-value accent" style="font-size:16px">${idle}</div>
      </div>
      <div class="stat-card" style="min-width:110px;flex:1 1 110px">
        <div class="stat-card-label">Intervalo (s)</div>
        <div class="stat-card-value accent" style="font-size:16px">${interval}</div>
      </div>
      ${p?.pages ? `<div class="stat-card" style="min-width:200px;flex:2 1 200px">
        <div class="stat-card-label">Páginas monitorizadas</div>
        <div style="font-size:11px;color:var(--text-secondary);margin-top:4px;word-break:break-all">${_esc(Array.isArray(p.pages) ? p.pages.join(', ') : String(p.pages))}</div>
      </div>` : ''}
    </div>`;
  }
}

async function _saveAggr(container) {
  const idle     = parseInt(container.querySelector('#aggr-idle')?.value || 120);
  const interval = parseInt(container.querySelector('#aggr-interval')?.value || 30);
  const msg = container.querySelector('#gov-msg');
  if (msg) msg.textContent = 'Guardando config agresiva...';
  try {
    const r = await fetch('/api/autonomy/aggressive/config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ idle_sec: idle, interval_sec: interval }),
    });
    const d = await r.json().catch(() => ({}));
    if (!r.ok || d.ok === false) throw new Error(d.error || `HTTP ${r.status}`);
    window.AtlasToast?.show('Configuración agresiva guardada', 'success');
    if (msg) msg.textContent = '✓ Configuración guardada';
    _loadAggrConfig(container);
  } catch (e) {
    window.AtlasToast?.show(e.message, 'error');
    if (msg) msg.textContent = `Error: ${e.message}`;
  }
  setTimeout(() => { const m = container.querySelector('#gov-msg'); if (m) m.textContent = ''; }, 4000);
}

async function _toggleAggr(container) {
  const msg = container.querySelector('#gov-msg');
  if (msg) msg.textContent = 'Cambiando modo agresivo...';
  try {
    // Get current state first
    const r0 = await fetch('/api/autonomy/aggressive/config');
    const d0 = await r0.json().catch(() => ({}));
    const p0 = d0?.data ?? d0;
    const wasEnabled = p0?.enabled === true || p0?.active === true;

    const r = await fetch('/api/autonomy/aggressive/config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ enabled: !wasEnabled }),
    });
    const d = await r.json().catch(() => ({}));
    if (!r.ok || d.ok === false) throw new Error(d.error || `HTTP ${r.status}`);
    window.AtlasToast?.show(`Modo agresivo: ${!wasEnabled ? 'activado' : 'desactivado'}`, 'success');
    if (msg) msg.textContent = `✓ Modo agresivo ${!wasEnabled ? 'activado' : 'desactivado'}`;
    _loadAggrConfig(container);
  } catch (e) {
    window.AtlasToast?.show(e.message, 'error');
    if (msg) msg.textContent = `Error: ${e.message}`;
  }
  setTimeout(() => { const m = container.querySelector('#gov-msg'); if (m) m.textContent = ''; }, 4000);
}

/* ─── Governance actions ────────────────────────────────────────────── */

async function _setMode(mode, container) {
  if (!confirm(`¿Cambiar modo de gobierno a ${mode.toUpperCase()}?`)) return;
  const msg = container.querySelector('#gov-msg');
  if (msg) msg.textContent = `Cambiando a ${mode}...`;
  try {
    const r = await fetch('/governance/mode', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ mode }),
    });
    const d = await r.json().catch(() => ({}));
    if (!r.ok || d.ok === false) throw new Error(d.error || `HTTP ${r.status}`);
    window.AtlasToast?.show(`Modo → ${mode.toUpperCase()}`, 'success');
    if (msg) msg.textContent = `✓ Modo cambiado a ${mode.toUpperCase()}`;
    setTimeout(() => _refresh(container), 1000);
  } catch (e) {
    window.AtlasToast?.show(e.message, 'error');
    if (msg) msg.textContent = `Error: ${e.message}`;
  }
  setTimeout(() => { const m = container.querySelector('#gov-msg'); if (m) m.textContent = ''; }, 5000);
}

async function _emergency(container) {
  if (!confirm('⚠ EMERGENCY STOP — ¿Detener todas las operaciones automáticas ahora? Esta acción es reversible cambiando el modo de gobierno.')) return;
  const msg = container.querySelector('#gov-msg');
  if (msg) msg.textContent = 'Ejecutando EMERGENCY STOP...';
  try {
    const r = await fetch('/governance/emergency', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ reason: 'Manual emergency stop desde panel v4' }),
    });
    const d = await r.json().catch(() => ({}));
    if (!r.ok || d.ok === false) throw new Error(d.error || `HTTP ${r.status}`);
    window.AtlasToast?.show('⚠ EMERGENCY STOP activado', 'error');
    if (msg) msg.textContent = '⚠ EMERGENCY STOP activado';
    setTimeout(() => _refresh(container), 800);
  } catch (e) {
    window.AtlasToast?.show(e.message, 'error');
    if (msg) msg.textContent = `Error: ${e.message}`;
  }
}

/* ─── Daemon control ────────────────────────────────────────────────── */

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

/* ─── Governance rules/log (lazy) ───────────────────────────────────── */

async function _loadRules(container) {
  const el = container.querySelector('#gov-rules');
  if (!el || el.dataset.loaded) return;
  el.dataset.loaded = '1';
  try {
    const r = await fetch('/governance/rules');
    const d = await r.json().catch(() => null);
    const p = d?.data ?? d ?? {};
    const rules = Array.isArray(p) ? p : (p?.rules || p?.items || Object.entries(p).map(([k, v]) => `${k}: ${v}`));
    if (!rules.length) { el.innerHTML = `<div class="empty-state"><div class="empty-title">Sin reglas cargadas</div></div>`; return; }
    el.innerHTML = `<div style="display:flex;flex-direction:column;gap:4px">
      ${rules.map(rule => `<div class="tl-entry info" style="padding:6px 10px">
        <div class="tl-icon" style="font-size:10px">◼</div>
        <div class="tl-body"><div class="tl-msg">${_esc(typeof rule === 'string' ? rule : JSON.stringify(rule))}</div></div>
      </div>`).join('')}
    </div>`;
  } catch (e) {
    if (el) el.innerHTML = `<div style="font-size:11px;color:var(--text-muted)">Error: ${_esc(e.message)}</div>`;
  }
}

async function _loadGovLog(container) {
  const el = container.querySelector('#gov-log');
  if (!el) return;
  try {
    const r = await fetch('/governance/log');
    const d = await r.json().catch(() => null);
    const p = d?.data ?? d ?? {};
    const items = Array.isArray(p) ? p : (p?.items || p?.entries || []);
    if (!items.length) { el.innerHTML = `<div class="empty-state"><div class="empty-title">Sin entradas de log</div></div>`; return; }
    el.innerHTML = [...items].reverse().slice(0, 30).map(entry => {
      const msg = entry.message || entry.msg || entry.text || JSON.stringify(entry);
      const ts  = _time(entry.timestamp || entry.ts);
      const lvl = (entry.level || entry.type || 'info').toLowerCase();
      const cls = lvl.includes('error') ? 'error' : lvl.includes('warn') ? 'warn' : 'info';
      return `<div class="tl-entry ${cls}">
        <div class="tl-time">${_esc(ts)}</div>
        <div class="tl-icon">${cls === 'error' ? '✕' : cls === 'warn' ? '!' : 'I'}</div>
        <div class="tl-body"><div class="tl-msg">${_esc(msg)}</div></div>
      </div>`;
    }).join('');
  } catch (e) {
    if (el) el.innerHTML = `<div style="font-size:11px;color:var(--text-muted)">Error: ${_esc(e.message)}</div>`;
  }
}

window.AtlasModuleAutonomy = { id: 'autonomy' };
