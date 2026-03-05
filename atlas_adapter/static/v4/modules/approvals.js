/**
 * ATLAS v4.3 - Approvals Module
 * Pending approvals with approve/reject actions and owner session controls.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'approvals-module';
let OWNER_TIMER = null;

function _esc(s) {
  const d = document.createElement('span');
  d.textContent = s ?? '';
  return d.innerHTML;
}

function _shortTime(ts) {
  if (!ts) return '--';
  try { return new Date(ts).toLocaleTimeString(); } catch { return String(ts); }
}

function _extractItems(data) {
  const p = data?.data ?? data;
  return p?.items || p?.approvals || p?.pending || (Array.isArray(p) ? p : []);
}

function _ownerToken() {
  return sessionStorage.getItem('atlas-owner-session-token') || '';
}

function _ownerTokenExp() {
  return sessionStorage.getItem('atlas-owner-session-exp') || '';
}

function _setOwnerToken(token, exp) {
  if (token) sessionStorage.setItem('atlas-owner-session-token', String(token));
  if (exp) sessionStorage.setItem('atlas-owner-session-exp', String(exp));
}

function _clearOwnerToken() {
  sessionStorage.removeItem('atlas-owner-session-token');
  sessionStorage.removeItem('atlas-owner-session-exp');
}

async function _ensureOwnerSessionToken() {
  let token = _ownerToken();
  if (token) return token;
  const r = await fetch('/owner/session/start', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ actor: 'dashboard', method: 'ui' }),
  });
  const d = await r.json().catch(() => ({}));
  if (!r.ok || d.ok === false || !d.session_token) {
    throw new Error(d.error || `Owner session start failed (HTTP ${r.status})`);
  }
  token = String(d.session_token || '').trim();
  if (!token) throw new Error('Owner session token is empty.');
  _setOwnerToken(token, d.expires_at || '');
  return token;
}

async function _startOwnerSession(container) {
  _clearOwnerToken();
  await _ensureOwnerSessionToken();
  await _refreshOwnerSessionPanel(container);
}

async function _endOwnerSession(container) {
  const token = _ownerToken();
  if (!token) {
    await _refreshOwnerSessionPanel(container);
    return;
  }
  const r = await fetch('/owner/session/end', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ session_token: token }),
  });
  const d = await r.json().catch(() => ({}));
  if (!r.ok || d.ok === false) {
    throw new Error(d.error || `Owner session end failed (HTTP ${r.status})`);
  }
  _clearOwnerToken();
  await _refreshOwnerSessionPanel(container);
}

async function _refreshOwnerSessionPanel(container) {
  const chip = container.querySelector('#owner-session-state');
  const meta = container.querySelector('#owner-session-meta');
  if (!chip || !meta) return;
  try {
    const r = await fetch('/owner/session/status');
    const d = await r.json().catch(() => ({}));
    const sessions = Array.isArray(d.sessions) ? d.sessions : [];
    const hasLocal = Boolean(_ownerToken());
    const exp = _ownerTokenExp();
    const expTxt = exp ? _shortTime(exp) : '--';

    chip.className = `chip ${hasLocal ? 'green' : 'orange'}`;
    chip.textContent = hasLocal ? 'Owner Session: ACTIVA' : 'Owner Session: INACTIVA';
    if (d.ok === false) {
      chip.className = 'chip red';
      chip.textContent = 'Owner Session: ERROR';
      meta.textContent = d.error || 'status unavailable';
      return;
    }
    meta.textContent = `Sesiones servidor: ${sessions.length} · Exp local: ${expTxt}`;
  } catch (e) {
    chip.className = 'chip red';
    chip.textContent = 'Owner Session: ERROR';
    meta.textContent = String(e.message || e);
  }
}

export default {
  id: 'approvals',
  label: 'Aprobaciones',
  icon: 'check-circle',
  category: 'control',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Dashboard
          </button>
          <h2>Aprobaciones</h2>
        </div>
        <div class="module-body">
          <div class="action-bar" style="padding-top:0;margin-bottom:8px">
            <span class="chip blue" id="owner-session-state">Owner Session: --</span>
            <button class="action-btn" id="owner-session-start">Iniciar/Renovar</button>
            <button class="action-btn danger" id="owner-session-end">Cerrar</button>
            <div id="owner-session-meta" style="font-size:11px;color:var(--text-muted);margin-left:auto"></div>
          </div>

          <div class="stat-row" style="margin-bottom:16px">
            <div class="stat-card hero">
              <div class="stat-card-label">Pendientes</div>
              <div class="stat-card-value orange" id="ap-count">--</div>
              <div class="stat-card-sub">Requieren tu atencion</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Ultima revision</div>
              <div class="stat-card-value" id="ap-last" style="font-size:14px">--</div>
            </div>
          </div>

          <div class="section-title">
            Cola de Aprobaciones
            <span class="live-badge">LIVE</span>
          </div>
          <div id="approvals-list">
            <div style="padding:20px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
          </div>
        </div>
      </div>
    `;

    container.querySelector('#owner-session-start')?.addEventListener('click', async () => {
      try {
        await _startOwnerSession(container);
        window.AtlasToast?.show('Owner session iniciada', 'success');
      } catch (e) {
        window.AtlasToast?.show(String(e.message || e), 'error');
      }
    });

    container.querySelector('#owner-session-end')?.addEventListener('click', async () => {
      try {
        await _endOwnerSession(container);
        window.AtlasToast?.show('Owner session cerrada', 'info');
      } catch (e) {
        window.AtlasToast?.show(String(e.message || e), 'error');
      }
    });

    _refreshOwnerSessionPanel(container);
    clearInterval(OWNER_TIMER);
    OWNER_TIMER = setInterval(() => _refreshOwnerSessionPanel(container), 12000);

    _fetchAndRender(container);
    poll(POLL_ID, '/approvals/pending?limit=50', 5000, (data) => {
      if (data) _render(container, data);
    });
  },

  destroy() {
    stop(POLL_ID);
    clearInterval(OWNER_TIMER);
    OWNER_TIMER = null;
  },
  badge() { return null; },
};

async function _fetchAndRender(container) {
  try {
    const r = await fetch('/approvals/pending?limit=50');
    const data = await r.json();
    _render(container, data);
  } catch (e) {
    const el = container.querySelector('#approvals-list');
    if (el) {
      el.innerHTML = `<div class="approval-card">
        <div class="approval-desc" style="color:var(--accent-red)">Error: ${_esc(e.message)}</div>
      </div>`;
    }
  }
}

function _render(container, data) {
  const el = container.querySelector('#approvals-list');
  if (!el) return;

  const items = _extractItems(data);
  const txt = (id, v) => { const e = container.querySelector(id); if (e) e.textContent = v; };
  txt('#ap-count', items.length);
  txt('#ap-last', new Date().toLocaleTimeString());

  if (items.length === 0) {
    el.innerHTML = `<div class="empty-state">
      <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5"><path d="M22 11.08V12a10 10 0 11-5.93-9.14"/><polyline points="22 4 12 14.01 9 11.01"/></svg>
      <div class="empty-title">Todo aprobado</div>
      <div class="empty-sub">No hay aprobaciones pendientes en este momento</div>
    </div>`;
    return;
  }

  el.innerHTML = items.slice(0, 50).map((a) => {
    const id = a.id || a.approval_id || a.task_id || '--';
    const kind = a.kind || a.type || a.module || 'approval';
    const ts = _shortTime(a.created_at || a.ts || a.timestamp);
    const summary = a.summary || a.title || a.description || a.reason || '';
    const risk = a.risk_level || a.risk || '';
    const riskCls = risk === 'high' ? 'red' : risk === 'medium' ? 'orange' : 'accent';
    const riskColor = riskCls === 'red' ? 'red' : riskCls === 'orange' ? 'orange' : 'green';

    return `<div class="approval-card">
      <div class="approval-header">
        <div class="approval-title">
          <span class="chip ${riskCls}" style="margin-right:8px">${_esc(kind)}</span>
          ${_esc(summary || id)}
        </div>
        <div class="approval-meta">${_esc(ts)}</div>
      </div>
      ${a.description && a.summary ? `<div class="approval-desc">${_esc(a.description)}</div>` : ''}
      <div class="approval-meta" style="margin-bottom:10px">
        ID: <code style="font-family:var(--font-mono);font-size:10px">${_esc(String(id).slice(0, 40))}</code>
        ${risk ? ` · Riesgo: <span style="color:var(--accent-${riskColor})">${_esc(risk)}</span>` : ''}
      </div>
      <div class="approval-actions">
        <button class="action-btn success" onclick="window._approveAction('${_esc(String(id))}', 'approve')">
          <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><polyline points="20 6 9 17 4 12"/></svg>
          Aprobar
        </button>
        <button class="action-btn danger" onclick="window._approveAction('${_esc(String(id))}', 'reject')">
          <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><line x1="18" y1="6" x2="6" y2="18"/><line x1="6" y1="6" x2="18" y2="18"/></svg>
          Rechazar
        </button>
      </div>
    </div>`;
  }).join('');

  window._approveAction = async (id, action) => {
    try {
      const url = action === 'approve' ? '/approvals/approve' : '/approvals/reject';
      let ownerToken = '';
      if (action === 'approve') ownerToken = await _ensureOwnerSessionToken();

      let r = await fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ id, session_token: ownerToken || undefined }),
      });
      let d = await r.json().catch(() => ({}));

      if (
        action === 'approve' &&
        (d?.status === 'owner_session_required' || String(d?.error || '').toLowerCase().includes('owner'))
      ) {
        _clearOwnerToken();
        ownerToken = await _ensureOwnerSessionToken();
        r = await fetch(url, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ id, session_token: ownerToken }),
        });
        d = await r.json().catch(() => ({}));
      }

      if (!r.ok || d.ok === false) throw new Error(d.error || `HTTP ${r.status}`);
      const okMsg = `${action === 'approve' ? 'Aprobado' : 'Rechazado'}: ${id.slice(0, 12)}`;
      if (window.AtlasToast?.show) window.AtlasToast.show(okMsg, 'success');
      setTimeout(() => {
        _refreshOwnerSessionPanel(container);
        _fetchAndRender(container);
      }, 700);
    } catch (e) {
      const errMsg = e?.message || 'Error en aprobacion';
      if (window.AtlasToast?.show) window.AtlasToast.show(errMsg, 'error');
    }
  };
}

window.AtlasModuleApprovals = { id: 'approvals' };
