/**
 * ATLAS v4.2 — Approvals Module
 * Tarjetas de aprobación pendiente con acciones aprobar/rechazar.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'approvals-module';

function _esc(s) { const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML; }

function _shortTime(ts) {
  if (!ts) return '--';
  try { return new Date(ts).toLocaleTimeString(); } catch { return String(ts); }
}

function _extractItems(data) {
  const p = data?.data ?? data;
  return p?.items || p?.approvals || p?.pending || (Array.isArray(p) ? p : []);
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
            Inicio
          </button>
          <h2>Aprobaciones</h2>
        </div>
        <div class="module-body">
          <div class="stat-row" style="margin-bottom:16px">
            <div class="stat-card hero">
              <div class="stat-card-label">Pendientes</div>
              <div class="stat-card-value orange" id="ap-count">--</div>
              <div class="stat-card-sub">Requieren tu atención</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Última revisión</div>
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

    _fetchAndRender(container);
    poll(POLL_ID, '/approvals/pending?limit=50', 5000, (data) => {
      if (data) _render(container, data);
    });
  },

  destroy() { stop(POLL_ID); },
  badge() { return null; },
};

async function _fetchAndRender(container) {
  try {
    const r = await fetch('/approvals/pending?limit=50');
    const data = await r.json();
    _render(container, data);
  } catch (e) {
    const el = container.querySelector('#approvals-list');
    if (el) el.innerHTML = `<div class="approval-card">
      <div class="approval-desc" style="color:var(--accent-red)">Error: ${_esc(e.message)}</div>
    </div>`;
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

  el.innerHTML = items.slice(0, 50).map(a => {
    const id      = a.id || a.approval_id || a.task_id || '--';
    const kind    = a.kind || a.type || a.module || 'approval';
    const ts      = _shortTime(a.created_at || a.ts || a.timestamp);
    const summary = a.summary || a.title || a.description || a.reason || '';
    const risk    = a.risk_level || a.risk || '';
    const riskCls = risk === 'high' ? 'red' : risk === 'medium' ? 'orange' : 'accent';
    return `<div class="approval-card">
      <div class="approval-header">
        <div class="approval-title">
          <span class="chip ${riskCls}" style="margin-right:8px">${_esc(kind)}</span>
          ${_esc(summary || id)}
        </div>
        <div class="approval-meta">${_esc(ts)}</div>
      </div>
      ${a.description && a.summary ? `<div class="approval-desc">${_esc(a.description)}</div>` : ''}
      <div class="approval-meta" style="margin-bottom:10px">ID: <code style="font-family:var(--font-mono);font-size:10px">${_esc(String(id).slice(0, 40))}</code>${risk ? ` · Riesgo: <span style="color:var(--accent-${riskCls === 'red' ? 'red' : riskCls === 'orange' ? 'orange' : ''})">${_esc(risk)}</span>` : ''}</div>
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
      const url = action === 'approve' ? `/approvals/${id}/approve` : `/approvals/${id}/reject`;
      const r = await fetch(url, { method: 'POST' });
      const d = await r.json().catch(() => ({}));
      if (!r.ok && d.ok !== true) throw new Error(d.error || `HTTP ${r.status}`);
      window.AtlasToast?.show(`${action === 'approve' ? 'Aprobado' : 'Rechazado'}: ${id.slice(0,12)}`, 'success');
      setTimeout(() => _fetchAndRender(container), 800);
    } catch (e) {
      window.AtlasToast?.show(e.message, 'error');
    }
  };
}

window.AtlasModuleApprovals = { id: 'approvals' };
