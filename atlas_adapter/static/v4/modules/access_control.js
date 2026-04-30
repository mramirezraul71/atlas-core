/**
 * ATLAS v4.3 — Control de Acceso
 * Gestión de usuarios autorizados, sesiones activas y estado de emergencia.
 * Persistencia en localStorage: los datos no se pierden al recargar.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID   = 'access-control-module';
const LS_USERS  = 'atlas-access-users-v1';
const LS_NOTES  = 'atlas-access-notes-v1';

/* ─── helpers ─────────────────────────────────────────────────── */
function _esc(s) {
  const d = document.createElement('span');
  d.textContent = s ?? '';
  return d.innerHTML;
}

function _ts(iso) {
  if (!iso) return '--';
  try { return new Date(iso).toLocaleString(); } catch { return String(iso); }
}

function _shortTs(iso) {
  if (!iso) return '--';
  try { return new Date(iso).toLocaleTimeString(); } catch { return String(iso); }
}

/* ─── localStorage: usuarios ───────────────────────────────────── */
function _loadUsers() {
  try {
    const raw = localStorage.getItem(LS_USERS);
    if (raw) return JSON.parse(raw);
  } catch {}
  // Seed con usuarios base del sistema
  return [
    { id: 'owner',    name: 'Propietario Principal',  role: 'owner',   method: 'ui',      active: true,  addedAt: new Date().toISOString() },
    { id: 'atlas',    name: 'ATLAS Sistema',           role: 'system',  method: 'api',     active: true,  addedAt: new Date().toISOString() },
    { id: 'clawd',    name: 'ClawdBOT',                role: 'agent',   method: 'api',     active: true,  addedAt: new Date().toISOString() },
    { id: 'telegram', name: 'Bot Telegram',            role: 'remote',  method: 'telegram',active: true,  addedAt: new Date().toISOString() },
  ];
}

function _saveUsers(users) {
  localStorage.setItem(LS_USERS, JSON.stringify(users));
}

function _loadNotes() {
  return localStorage.getItem(LS_NOTES) || '';
}

function _saveNotes(txt) {
  localStorage.setItem(LS_NOTES, txt);
}

/* ─── render helpers ───────────────────────────────────────────── */
function _roleBadge(role) {
  const map = { owner: 'red', system: 'blue', agent: 'purple', remote: 'orange', viewer: 'accent' };
  return `<span class="chip ${map[role] || 'accent'}" style="font-size:10px;padding:2px 7px">${_esc(role)}</span>`;
}

function _methodIcon(method) {
  const map = { ui: '🖥️', api: '⚡', telegram: '📱', voice: '🎙️', windows: '🪟', face: '👤' };
  return map[method] || '🔑';
}

/* ─── renderizado de panel de sesiones ─────────────────────────── */
function _renderSessions(container, data) {
  const el = container.querySelector('#ac-sessions');
  if (!el) return;
  const p = data?.data ?? data;
  const sessions = p?.active_sessions ?? [];
  const emergency = p?.emergency ?? false;
  const chainOk   = p?.chain_valid ?? p?.chain_ok ?? true;

  // estado emergencia
  const emEl = container.querySelector('#ac-emergency');
  if (emEl) {
    emEl.className = `chip ${emergency ? 'red' : 'green'}`;
    emEl.textContent = emergency ? '🚨 Modo Emergencia: ACTIVO' : '✅ Modo Normal';
  }

  // cadena de integridad
  const chainEl = container.querySelector('#ac-chain');
  if (chainEl) {
    chainEl.className = `chip ${chainOk ? 'green' : 'orange'}`;
    chainEl.textContent = chainOk ? 'Cadena: OK' : 'Cadena: ⚠️ Revisar';
  }

  const countEl = container.querySelector('#ac-sessions-count');
  if (countEl) countEl.textContent = sessions.length;

  if (sessions.length === 0) {
    el.innerHTML = `<div class="empty-state" style="padding:16px 0">
      <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5"><circle cx="12" cy="8" r="4"/><path d="M20 21a8 8 0 10-16 0"/></svg>
      <div class="empty-sub">Sin sesiones activas</div>
    </div>`;
    return;
  }

  el.innerHTML = sessions.map((s, i) => {
    const method = s.method || 'ui';
    return `<div class="approval-card" style="padding:10px 14px;margin-bottom:6px">
      <div style="display:flex;align-items:center;gap:10px">
        <span style="font-size:20px">${_methodIcon(method)}</span>
        <div style="flex:1;min-width:0">
          <div style="font-size:12px;font-weight:600;color:var(--text-primary)">${_esc(method.toUpperCase())} · Sesión ${i + 1}</div>
          <div style="font-size:11px;color:var(--text-muted);margin-top:2px">Creada: ${_shortTs(s.created_at ? new Date(s.created_at * 1000).toISOString() : null)}</div>
          <div style="font-size:11px;color:var(--text-muted)">Expira: ${_ts(s.expires_at)}</div>
        </div>
        <span class="chip green" style="font-size:10px">ACTIVA</span>
      </div>
    </div>`;
  }).join('');
}

/* ─── renderizado de tabla de usuarios ─────────────────────────── */
function _renderUsers(container) {
  const el = container.querySelector('#ac-users-list');
  if (!el) return;
  const users = _loadUsers();
  const activeCount = users.filter(u => u.active).length;
  const cntEl = container.querySelector('#ac-users-count');
  if (cntEl) cntEl.textContent = `${activeCount} / ${users.length}`;

  if (users.length === 0) {
    el.innerHTML = `<div class="empty-state"><div class="empty-title">Sin usuarios</div></div>`;
    return;
  }

  el.innerHTML = `
    <table class="data-tbl" style="width:100%">
      <thead>
        <tr>
          <th>ID</th><th>Nombre</th><th>Rol</th><th>Método</th><th>Añadido</th><th>Estado</th><th></th>
        </tr>
      </thead>
      <tbody>
        ${users.map((u, idx) => `
          <tr>
            <td><code style="font-size:11px">${_esc(u.id)}</code></td>
            <td style="font-weight:500">${_esc(u.name)}</td>
            <td>${_roleBadge(u.role)}</td>
            <td><span title="${_esc(u.method)}">${_methodIcon(u.method)} ${_esc(u.method)}</span></td>
            <td style="font-size:11px;color:var(--text-muted)">${_ts(u.addedAt)}</td>
            <td>
              <span class="chip ${u.active ? 'green' : 'red'}" style="font-size:10px">
                ${u.active ? 'Activo' : 'Revocado'}
              </span>
            </td>
            <td>
              <button class="action-btn ${u.active ? 'danger' : 'success'}" style="font-size:11px;padding:4px 10px"
                data-toggle-user="${idx}">${u.active ? 'Revocar' : 'Activar'}</button>
              <button class="action-btn" style="font-size:11px;padding:4px 10px;margin-left:4px"
                data-remove-user="${idx}">✕</button>
            </td>
          </tr>
        `).join('')}
      </tbody>
    </table>
  `;

  // delegación de eventos en la tabla
  el.addEventListener('click', (e) => {
    const toggleBtn = e.target.closest('[data-toggle-user]');
    const removeBtn = e.target.closest('[data-remove-user]');
    if (toggleBtn) {
      const idx = parseInt(toggleBtn.dataset.toggleUser, 10);
      const users2 = _loadUsers();
      users2[idx].active = !users2[idx].active;
      _saveUsers(users2);
      _renderUsers(container);
      window.AtlasToast?.show(users2[idx].active ? `${users2[idx].name} activado` : `${users2[idx].name} revocado`, 'info');
    }
    if (removeBtn) {
      const idx = parseInt(removeBtn.dataset.removeUser, 10);
      const users2 = _loadUsers();
      const removed = users2.splice(idx, 1)[0];
      _saveUsers(users2);
      _renderUsers(container);
      window.AtlasToast?.show(`Usuario eliminado: ${removed.name}`, 'info');
    }
  }, { once: false });
}

/* ─── modal añadir usuario ─────────────────────────────────────── */
function _openAddUserModal(container) {
  const existing = document.querySelector('#ac-add-modal');
  if (existing) existing.remove();

  const overlay = document.createElement('div');
  overlay.id = 'ac-add-modal';
  overlay.style.cssText = 'position:fixed;inset:0;background:rgba(0,0,0,.7);display:flex;align-items:center;justify-content:center;z-index:9999';
  overlay.innerHTML = `
    <div style="background:var(--bg-card);border:1px solid var(--border);border-radius:12px;padding:28px;min-width:340px;max-width:90vw">
      <h3 style="margin:0 0 18px;font-size:15px;color:var(--text-primary)">Agregar Usuario Autorizado</h3>
      <div style="display:flex;flex-direction:column;gap:10px">
        <input id="ac-new-id"     class="config-input" placeholder="ID único (ej: r6957)" style="width:100%">
        <input id="ac-new-name"   class="config-input" placeholder="Nombre completo" style="width:100%">
        <select id="ac-new-role"  class="config-input" style="width:100%">
          <option value="owner">owner — Propietario</option>
          <option value="system" selected>system — Sistema</option>
          <option value="agent">agent — Agente AI</option>
          <option value="remote">remote — Acceso remoto</option>
          <option value="viewer">viewer — Solo lectura</option>
        </select>
        <select id="ac-new-method" class="config-input" style="width:100%">
          <option value="ui">ui — Dashboard</option>
          <option value="api">api — API Key</option>
          <option value="telegram">telegram — Bot Telegram</option>
          <option value="voice">voice — Voz</option>
          <option value="windows">windows — Windows Auth</option>
        </select>
      </div>
      <div style="display:flex;gap:8px;margin-top:18px;justify-content:flex-end">
        <button class="action-btn" id="ac-cancel-add">Cancelar</button>
        <button class="action-btn primary" id="ac-confirm-add">Agregar</button>
      </div>
    </div>
  `;
  document.body.appendChild(overlay);

  overlay.querySelector('#ac-cancel-add').addEventListener('click', () => overlay.remove());
  overlay.querySelector('#ac-confirm-add').addEventListener('click', () => {
    const id     = overlay.querySelector('#ac-new-id').value.trim();
    const name   = overlay.querySelector('#ac-new-name').value.trim();
    const role   = overlay.querySelector('#ac-new-role').value;
    const method = overlay.querySelector('#ac-new-method').value;
    if (!id || !name) { window.AtlasToast?.show('ID y Nombre son requeridos', 'error'); return; }
    const users = _loadUsers();
    if (users.find(u => u.id === id)) { window.AtlasToast?.show('ID ya existe', 'error'); return; }
    users.push({ id, name, role, method, active: true, addedAt: new Date().toISOString() });
    _saveUsers(users);
    _renderUsers(container);
    overlay.remove();
    window.AtlasToast?.show(`Usuario agregado: ${name}`, 'success');
  });

  overlay.addEventListener('click', (e) => { if (e.target === overlay) overlay.remove(); });
}

/* ─── exportar lista de usuarios ──────────────────────────────── */
function _exportUsers() {
  const users = _loadUsers();
  const blob = new Blob([JSON.stringify(users, null, 2)], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `atlas-usuarios-autorizados-${new Date().toISOString().slice(0,10)}.json`;
  a.click();
  URL.revokeObjectURL(a.href);
}

/* ─── módulo principal ─────────────────────────────────────────── */
export default {
  id: 'access-control',
  label: 'Control de Acceso',
  icon: 'lock',
  category: 'control',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Dashboard
          </button>
          <h2>Control de Acceso</h2>
          <div style="margin-left:auto;display:flex;align-items:center;gap:8px">
            <span class="chip blue" id="ac-emergency">--</span>
            <span class="chip blue" id="ac-chain">--</span>
            <span class="live-badge">LIVE</span>
          </div>
        </div>

        <div class="module-body">

          <!-- KPIs -->
          <div class="stat-row" style="margin-bottom:20px">
            <div class="stat-card hero">
              <div class="stat-card-label">Usuarios Autorizados</div>
              <div class="stat-card-value green" id="ac-users-count">--</div>
              <div class="stat-card-sub">activos / total</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Sesiones en Servidor</div>
              <div class="stat-card-value blue" id="ac-sessions-count">--</div>
              <div class="stat-card-sub">en este momento</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Última actualización</div>
              <div class="stat-card-value" id="ac-last-update" style="font-size:13px">--</div>
            </div>
          </div>

          <!-- Sesiones activas -->
          <div class="section-title" style="margin-bottom:10px">
            Sesiones Activas
            <span class="live-badge">LIVE</span>
          </div>
          <div id="ac-sessions" style="margin-bottom:24px">
            <div style="padding:20px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
          </div>

          <!-- Usuarios autorizados -->
          <div class="section-title" style="margin-bottom:10px">
            Usuarios Autorizados
            <span class="count" id="ac-users-count-badge"></span>
          </div>
          <div class="action-bar" style="padding-top:0;margin-bottom:12px">
            <button class="action-btn primary" id="ac-btn-add">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><line x1="12" y1="5" x2="12" y2="19"/><line x1="5" y1="12" x2="19" y2="12"/></svg>
              Agregar usuario
            </button>
            <button class="action-btn" id="ac-btn-export">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M21 15v4a2 2 0 01-2 2H5a2 2 0 01-2-2v-4"/><polyline points="7 10 12 15 17 10"/><line x1="12" y1="15" x2="12" y2="3"/></svg>
              Exportar JSON
            </button>
            <div style="font-size:11px;color:var(--text-muted);align-self:center;margin-left:auto">
              Datos guardados en este navegador · persisten al recargar
            </div>
          </div>
          <div id="ac-users-list">
            <div style="padding:20px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
          </div>

          <!-- Notas de seguridad -->
          <div class="section-title" style="margin-top:24px;margin-bottom:10px">Notas de Acceso</div>
          <textarea id="ac-notes" class="config-input"
            style="width:100%;min-height:80px;resize:vertical;font-size:12px;font-family:var(--font-mono);box-sizing:border-box"
            placeholder="Notas internas sobre políticas de acceso, IPs permitidas, etc."
          ></textarea>
          <button class="action-btn" id="ac-btn-save-notes" style="margin-top:8px">Guardar notas</button>

        </div>
      </div>
    `;

    // notas persistentes
    const notesEl = container.querySelector('#ac-notes');
    if (notesEl) notesEl.value = _loadNotes();

    container.querySelector('#ac-btn-save-notes')?.addEventListener('click', () => {
      _saveNotes(notesEl?.value || '');
      window.AtlasToast?.show('Notas guardadas', 'success');
    });

    container.querySelector('#ac-btn-add')?.addEventListener('click', () => _openAddUserModal(container));
    container.querySelector('#ac-btn-export')?.addEventListener('click', _exportUsers);

    _renderUsers(container);
    _fetchStatus(container);

    poll(POLL_ID, '/owner/status', 10000, (data) => {
      if (data) {
        _renderSessions(container, data);
        const lastEl = container.querySelector('#ac-last-update');
        if (lastEl) lastEl.textContent = new Date().toLocaleTimeString();
      }
    });
  },

  destroy() {
    stop(POLL_ID);
    document.querySelector('#ac-add-modal')?.remove();
  },

  badge() { return null; },
};

async function _fetchStatus(container) {
  try {
    const r = await fetch('/owner/status');
    const data = await r.json();
    _renderSessions(container, data);
    const lastEl = container.querySelector('#ac-last-update');
    if (lastEl) lastEl.textContent = new Date().toLocaleTimeString();
  } catch (e) {
    const el = container.querySelector('#ac-sessions');
    if (el) el.innerHTML = `<div class="empty-state" style="padding:12px 0">
      <div class="empty-sub" style="color:var(--accent-red)">Error al cargar sesiones: ${_esc(e.message)}</div>
    </div>`;
  }
}

window.AtlasModuleAccessControl = { id: 'access-control' };
