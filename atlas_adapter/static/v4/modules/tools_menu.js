/**
 * ATLAS v4.3 - Tools Menu
 * Inventario de herramientas, salud, upgrades y acciones seguras.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'tools-menu-module';
const MENU_ENDPOINT = '/api/tools/menu';
const JOB_KEY = 'atlas-tools-menu-active-job';

function _esc(s) {
  const d = document.createElement('span');
  d.textContent = s ?? '';
  return d.innerHTML;
}

function _payload(data) {
  if (!data) return null;
  return data.data ?? data;
}

function _statusDot(status) {
  if (status === 'ok') return 'ok';
  if (status === 'warn') return 'degraded';
  if (status === 'error') return 'down';
  return 'unknown';
}

function _fmt(v) {
  return v && String(v).trim() ? String(v) : '--';
}

function _isProtectedBlock(data) {
  const git = data?.git || {};
  return Boolean(git.is_protected) && !Boolean(git.allow_protected_updates);
}

function _normalizeJobStatus(job) {
  const raw = String(job?.status || 'queued').toLowerCase().trim().replace(/\s+/g, '_');
  if (raw === 'done_with_errors') return 'done_with_errors';
  if (raw === 'done' || raw === 'completed' || raw === 'ok') return 'done';
  if (raw === 'failed' || raw === 'error') return 'failed';
  if (raw === 'running' || raw === 'in_progress') return 'running';
  return 'queued';
}

export default {
  id: 'tools-menu',
  label: 'Tools Menu',
  icon: 'wrench',
  category: 'system',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Dashboard
          </button>
          <h2>Tools Menu</h2>
          <div style="margin-left:auto"><span class="live-badge">LIVE</span></div>
        </div>
        <div class="module-body">
          <div class="stat-row" style="margin-bottom: 16px;">
            <div class="stat-card hero">
              <div class="stat-card-label">Herramientas</div>
              <div class="stat-card-value green" id="tm-total">--</div>
              <div class="stat-card-sub" id="tm-scan-time">sin escaneo</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Salud OK</div>
              <div class="stat-card-value green" id="tm-ok">--</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Con advertencia</div>
              <div class="stat-card-value orange" id="tm-warn">--</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Upgrade Ready</div>
              <div class="stat-card-value accent" id="tm-upgrades">--</div>
            </div>
          </div>

          <div class="action-bar">
            <button class="action-btn primary" id="tm-btn-bootstrap">
              Instalar Base Obligatoria
            </button>
            <button class="action-btn" id="tm-btn-scan">
              Escanear Versiones
            </button>
            <button class="action-btn" id="tm-btn-update-all">
              Actualizar Todo
            </button>
            <button class="action-btn" id="tm-btn-discovery">
              Buscar Herramientas Nuevas
            </button>
            <button class="action-btn" id="tm-btn-reconcile">
              Reconciliar estados
            </button>
            <label class="tools-filter-wrap">
              <span>Filtro</span>
              <select id="tm-filter" class="tools-filter-select">
                <option value="all">Todas</option>
                <option value="upgrade">Upgrade Ready</option>
                <option value="warn">Advertencias / Error</option>
              </select>
            </label>
          </div>

          <div id="tm-alerts"></div>
          <div id="tm-progress"></div>
          <div id="tm-action" style="margin: 10px 0 14px;"></div>

          <div class="section-title">
            Inventario de Herramientas
            <span class="count" id="tm-count">--</span>
          </div>
          <div class="provider-grid" id="tm-grid">
            <div class="empty-state" style="grid-column:1/-1">
              <div class="spinner"></div>
              <div class="empty-sub" style="margin-top:8px">Cargando inventario...</div>
            </div>
          </div>

          <div class="section-title" style="margin-top: 18px;">
            Registro Maestro
            <span class="count">atlas_master_registry.json</span>
          </div>
          <div class="codebox" id="tm-meta">Preparando...</div>

          <div class="section-title" style="margin-top: 20px;">
            Expansion Hub
            <span class="count" id="tm-discovery-count">--</span>
          </div>
          <div id="tm-discovery-alerts"></div>
          <div id="tm-discovery-progress"></div>
          <div class="provider-grid" id="tm-discovery-grid">
            <div class="empty-state" style="grid-column:1/-1">
              <div class="empty-title">Discovery pendiente</div>
              <div class="empty-sub">Presiona "Buscar Herramientas Nuevas".</div>
            </div>
          </div>
        </div>
      </div>
      <div id="tm-detail-modal" style="display:none;position:fixed;inset:0;z-index:9999;background:rgba(0,0,0,.62);padding:24px;">
        <div style="max-width:980px;margin:0 auto;background:var(--surface-0);border:1px solid var(--border-subtle);border-radius:12px;overflow:hidden;">
          <div style="display:flex;align-items:center;gap:10px;padding:10px 12px;border-bottom:1px solid var(--border-subtle);">
            <strong id="tm-detail-title" style="font-size:13px;">Detalle</strong>
            <div style="margin-left:auto;display:flex;gap:8px;">
              <button class="action-btn" id="tm-detail-export">Exportar reporte</button>
              <button class="action-btn" id="tm-detail-close">Cerrar</button>
            </div>
          </div>
          <div id="tm-detail-body" style="max-height:72vh;overflow:auto;padding:12px;"></div>
        </div>
      </div>
    `;

    container.querySelector('#tm-btn-scan')?.addEventListener('click', () => _scanNow(container));
    container.querySelector('#tm-btn-bootstrap')?.addEventListener('click', () => _bootstrap(container));
    container.querySelector('#tm-btn-update-all')?.addEventListener('click', () => _updateAll(container));
    container.querySelector('#tm-btn-discovery')?.addEventListener('click', () => _runDiscovery(container));
    container.querySelector('#tm-btn-reconcile')?.addEventListener('click', () => _reconcileStates(container));
    container.querySelector('#tm-filter')?.addEventListener('change', () => _renderTools(container));
    container.querySelector('#tm-detail-close')?.addEventListener('click', () => _closeDetail(container));
    container.querySelector('#tm-detail-export')?.addEventListener('click', () => _exportCurrentJob(container));
    container.querySelector('#tm-detail-modal')?.addEventListener('click', (ev) => {
      if (ev.target?.id === 'tm-detail-modal') _closeDetail(container);
    });

    _refresh(container);
    _runDiscovery(container);
    _resumeOrDiscoverJob(container);
    poll(POLL_ID, MENU_ENDPOINT, 30000, (data) => {
      if (data) _applyData(container, _payload(data));
    });
  },

  destroy() {
    stop(POLL_ID);
  },
};

async function _refresh(container) {
  try {
    const r = await fetch(MENU_ENDPOINT);
    const d = await r.json().catch(() => ({}));
    _applyData(container, _payload(d));
  } catch (e) {
    _setAction(container, `<div style="color:var(--accent-red);font-size:12px">Error: ${_esc(e.message)}</div>`);
  }
}

function _applyData(container, data) {
  if (!data) return;
  container.__toolsMenuData = data;

  const summary = data.summary || {};
  const total = Number(summary.total || 0);
  const ok = Number(summary.ok || 0);
  const warnOnly = Number(summary.warn || 0);
  const err = Number(summary.error || 0);
  const warn = warnOnly + err;
  const upgrades = Number(summary.upgrade_ready || 0);
  const scanAt = data.generated_at ? new Date(data.generated_at).toLocaleTimeString() : '--';
  const queue = Number((data.offline_queue || {}).pending || 0);
  const git = data.git || {};
  const protectedBlock = _isProtectedBlock(data);

  _txt(container, '#tm-total', total);
  _txt(container, '#tm-ok', ok);
  _txt(container, '#tm-warn', warn);
  _txt(container, '#tm-upgrades', upgrades);
  _txt(container, '#tm-count', `${total} tools`);
  _txt(container, '#tm-scan-time', `ultimo escaneo: ${scanAt}`);

  const alerts = container.querySelector('#tm-alerts');
  if (alerts) {
    const bits = [];
    if (upgrades > 0) bits.push(`<span class="chip orange">UPGRADE_READY: ${upgrades}</span>`);
    if (err > 0) bits.push(`<span class="chip red">ERROR: ${err}</span>`);
    if (queue > 0) bits.push(`<span class="chip red">Offline Queue: ${queue}</span>`);
    if (git.branch) bits.push(`<span class="chip blue">Branch: ${_esc(git.branch)}</span>`);
    if (protectedBlock) bits.push('<span class="chip red">Bloqueado en rama protegida</span>');
    if (bits.length) {
      alerts.innerHTML = `<div class="tools-alert-box">${bits.join(' ')}</div>`;
    } else {
      alerts.innerHTML = '';
    }
  }

  const meta = container.querySelector('#tm-meta');
  if (meta) {
    meta.textContent = JSON.stringify({
      generated_at: data.generated_at,
      workspace: data.workspace,
      offline_queue: data.offline_queue,
      channels: data.channels,
      git: data.git,
      security: data.security,
    }, null, 2);
  }

  _renderTools(container);
  _renderProgress(container);
  if (container.__discoveryData) _renderDiscovery(container);
}

function _renderTools(container) {
  const data = container.__toolsMenuData || {};
  const js = container.__jobState || null;
  const rows = Array.isArray(data.tools) ? data.tools : [];
  const filter = container.querySelector('#tm-filter')?.value || 'all';
  const protectedBlock = _isProtectedBlock(data);

  let filtered = rows;
  if (filter === 'upgrade') filtered = rows.filter(r => r.update_ready);
  if (filter === 'warn') filtered = rows.filter(r => r.status === 'warn' || r.status === 'error');

  const grid = container.querySelector('#tm-grid');
  if (!grid) return;

  if (!filtered.length) {
    grid.innerHTML = `<div class="empty-state" style="grid-column:1/-1">
      <div class="empty-title">Sin herramientas para este filtro</div>
      <div class="empty-sub">Prueba "Todas" o ejecuta un nuevo escaneo.</div>
    </div>`;
    return;
  }

  grid.innerHTML = filtered.map((t) => {
    const cls = t.status === 'ok' ? 'active' : t.status === 'warn' ? 'degraded' : 'down';
    const dot = _statusDot(t.status);
    const canUpdate = !!(t.update_script && String(t.update_script).trim());
    const blocked = protectedBlock && canUpdate;
    const upgradeChip = t.update_ready ? '<span class="chip orange">UPGRADE_READY</span>' : '';
    const criticalChip = t.critical ? '<span class="chip blue">ESENCIAL</span>' : '';
    const toolProgress = _toolProgressState(js, t.id);
    const progressChip = toolProgress ? `<span class="chip ${toolProgress.cls}">${_esc(toolProgress.label)}</span>` : '';
    const latest = t.latest_version ? `<div class="provider-role">Latest: ${_esc(t.latest_version)}</div>` : '';
    const detailBtn = toolProgress ? `<button class="action-btn" data-action="detail" data-tool="${_esc(t.id)}">Ver detalle</button>` : '';
    return `<div class="provider-card ${cls}" data-tool="${_esc(t.id)}">
      <div class="provider-latency">${_esc(t.category || 'tool')}</div>
      <div class="provider-name">${_esc(t.name || t.id)}</div>
      <div class="provider-role">Version: ${_esc(_fmt(t.version))}</div>
      ${latest}
      <div class="provider-status">
        <span class="provider-dot ${dot}"></span>
        ${_esc(_fmt(t.health))}
      </div>
      <div style="margin-top:8px;display:flex;gap:6px;flex-wrap:wrap">
        ${upgradeChip}
        ${criticalChip}
        ${progressChip}
      </div>
      <div style="font-size:11px;color:var(--text-muted);margin-top:8px;min-height:30px">
        ${_esc(_fmt(t.details))}
      </div>
      <div style="margin-top:12px;display:flex;gap:6px">
        <button class="action-btn ${t.update_ready ? 'primary' : ''}" data-action="update" data-tool="${_esc(t.id)}" data-critical="${t.critical ? '1' : '0'}" ${canUpdate && !blocked ? '' : 'disabled'}>
          ${blocked ? 'BLOQUEADO' : 'ACTUALIZAR'}
        </button>
        ${detailBtn}
      </div>
    </div>`;
  }).join('');

  grid.querySelectorAll('[data-action="update"]').forEach((btn) => {
    btn.addEventListener('click', () => _updateTool(container, btn.dataset.tool, btn.dataset.critical === '1'));
  });
  grid.querySelectorAll('[data-action="detail"]').forEach((btn) => {
    btn.addEventListener('click', () => _openDetail(container, btn.dataset.tool));
  });
}

async function _scanNow(container) {
  _setAction(container, '<div class="codebox" style="font-size:11px">Escaneando versiones oficiales...</div>');
  try {
    const r = await fetch('/api/tools/watchdog/scan', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ force: true }),
    });
    const d = await r.json().catch(() => ({}));
    if (!r.ok || d.ok === false) throw new Error(d.error || `HTTP ${r.status}`);
    _setAction(container, '<div class="chip green">Escaneo completado</div>');
    _applyData(container, _payload(d));
    window.AtlasToast?.show('Watchdog actualizado', 'success');
  } catch (e) {
    _setAction(container, `<div style="color:var(--accent-red);font-size:12px">Error escaneo: ${_esc(e.message)}</div>`);
    window.AtlasToast?.show(String(e.message), 'error');
  }
}

async function _reconcileStates(container) {
  _setAction(container, '<div class="codebox" style="font-size:11px">Reconciliando estado global (inventario + jobs + discovery)...</div>');
  try {
    await _scanNow(container);
    await _refresh(container);
    await _runDiscovery(container);

    const r = await fetch('/api/tools/job/latest');
    const d = await r.json().catch(() => ({}));
    const p = _payload(d) || {};
    if (r.ok && d.ok !== false && p.found && p.job && p.job.job_id) {
      _updateJobState(container, p.job);
      _setAction(container, `<div class="chip orange">Job activo detectado: ${_esc(String(p.job.job_id))}</div>`);
    } else {
      container.__jobState = null;
      _renderProgress(container);
      _renderTools(container);
      _setAction(container, '<div class="chip green">Estado reconciliado sin jobs activos</div>');
    }
    window.AtlasToast?.show('Estados reconciliados', 'success');
  } catch (e) {
    _setAction(container, `<div style="color:var(--accent-red);font-size:12px">Error reconciliando: ${_esc(String(e.message || e))}</div>`);
    window.AtlasToast?.show(String(e.message || e), 'error');
  }
}

async function _bootstrap(container) {
  if (!confirm('Instalar dependencias base (ccxt/playwright/puppeteer) con snapshot previo?')) return;
  _setAction(container, '<div class="codebox" style="font-size:11px">Ejecutando bootstrap seguro...</div>');
  try {
    const r = await fetch('/api/tools/bootstrap', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ background: true }),
    });
    const d = await r.json().catch(() => ({}));
    const payload = _payload(d) || {};
    if (!r.ok || d.ok === false) {
      const msg = d.error || payload.error || (payload.result && payload.result.error) || (d.ok === false ? 'update_failed (server ok=false)' : `HTTP ${r.status}`);
      throw new Error(msg);
    }
    if (payload.queued && payload.job_id) {
      _setAction(container, `<div class="codebox" style="font-size:11px">Bootstrap en segundo plano (job ${_esc(payload.job_id)})</div>`);
      window.AtlasToast?.show('Bootstrap en background', 'info');
      _watchJob(container, payload.job_id, 'bootstrap', { scanOnDone: true });
      return;
    }
    const ok = payload.ok !== false;
    _setAction(container, `<div class="codebox" style="font-size:11px">${_esc(JSON.stringify(payload, null, 2))}</div>`);
    window.AtlasToast?.show(ok ? 'Bootstrap completado' : 'Bootstrap con advertencias', ok ? 'success' : 'info');
    await _refresh(container);
  } catch (e) {
    _setAction(container, `<div style="color:var(--accent-red);font-size:12px">Error bootstrap: ${_esc(e.message)}</div>`);
    window.AtlasToast?.show(String(e.message), 'error');
  }
}

async function _updateTool(container, toolId, isCritical = false) {
  if (!toolId) return;
  if (_isProtectedBlock(container.__toolsMenuData || {})) {
    _setAction(container, '<div style="color:var(--accent-red);font-size:12px">Update bloqueado: rama protegida activa.</div>');
    return;
  }
  if (!confirm(`Actualizar herramienta "${toolId}" con snapshot previo?`)) return;
  if (isCritical) {
    const phrase = prompt(`"${toolId}" es crítica. Escribe ACTUALIZAR para confirmar:`, '');
    if ((phrase || '').trim().toUpperCase() !== 'ACTUALIZAR') {
      _setAction(container, '<div class="chip orange">Actualización cancelada por confirmación incompleta.</div>');
      return;
    }
  }
  _setAction(container, `<div class="codebox" style="font-size:11px">Actualizando ${_esc(toolId)}...</div>`);
  try {
    const r = await fetch('/api/tools/update', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ tool_id: toolId, background: true }),
    });
    const d = await r.json().catch(() => ({}));
    const payload = _payload(d) || {};
    if (!r.ok || d.ok === false) {
      const msg = d.error || payload.error || (payload.result && payload.result.error) || (d.ok === false ? 'update_failed (server ok=false)' : `HTTP ${r.status}`);
      throw new Error(msg);
    }
    if (payload.queued && payload.job_id) {
      _setAction(container, `<div class="codebox" style="font-size:11px">Update en segundo plano: ${_esc(toolId)} (job ${_esc(payload.job_id)})</div>`);
      window.AtlasToast?.show(`Update en background: ${toolId}`, 'info');
      _watchJob(container, payload.job_id, toolId, { scanOnDone: true });
      return;
    }
    _setAction(container, `<div class="codebox" style="font-size:11px">${_esc(JSON.stringify(payload, null, 2))}</div>`);
    window.AtlasToast?.show(`Update ejecutado: ${toolId}`, 'success');
    await _scanNow(container);
  } catch (e) {
    _setAction(container, `<div style="color:var(--accent-red);font-size:12px">Error update: ${_esc(e.message)}</div>`);
    window.AtlasToast?.show(String(e.message), 'error');
  }
}

async function _watchJob(container, jobId, label, opts = {}) {
  try {
    localStorage.setItem(JOB_KEY, JSON.stringify({ jobId, label, ts: Date.now() }));
  } catch (_) {}
  const scanOnDone = opts.scanOnDone !== false;
  const maxPolls = 120; // ~10 minutos
  for (let i = 0; i < maxPolls; i++) {
    try {
      await new Promise((resolve) => setTimeout(resolve, 5000));
      const r = await fetch(`/api/tools/job/status/${encodeURIComponent(jobId)}`);
      const d = await r.json().catch(() => ({}));
      const p = _payload(d) || {};
      if (!r.ok || d.ok === false) continue;
      if (!p.job_id) p.job_id = String(jobId);
      p.status = _normalizeJobStatus(p);
      _updateJobState(container, p);
      if (p.status === 'running' || p.status === 'queued') {
        _setAction(container, `<div class="codebox" style="font-size:11px">Ejecutando ${_esc(label)} (job ${_esc(String(jobId))})...</div>`);
        continue;
      }
      if (p.status === 'done') {
        try { localStorage.removeItem(JOB_KEY); } catch (_) {}
        container.__jobState = p;
        _renderProgress(container);
        _renderTools(container);
        if (String(p.source || '').toLowerCase().includes('discovery')) {
          container.__discoveryJobState = p;
          _renderDiscovery(container);
        }
        _setAction(container, `<div class="codebox" style="font-size:11px">Proceso completado: ${_esc(label)} (job ${_esc(jobId)})</div>`);
        window.AtlasToast?.show(`Completado: ${label}`, 'success');
        if (scanOnDone) await _scanNow(container);
        return;
      }
      if (p.status === 'done_with_errors') {
        try { localStorage.removeItem(JOB_KEY); } catch (_) {}
        container.__jobState = p;
        _renderProgress(container);
        _renderTools(container);
        if (String(p.source || '').toLowerCase().includes('discovery')) {
          container.__discoveryJobState = p;
          _renderDiscovery(container);
        }
        if (scanOnDone) await _scanNow(container);
        const msg = p.error || (p.result && p.result.error) || 'Completado con errores';
        _setAction(container, `<div style="color:var(--accent-orange);font-size:12px">Proceso completado con errores (${_esc(String(label))}): ${_esc(String(msg))}</div>`);
        window.AtlasToast?.show('Completado con errores', 'warning');
        return;
      }
      if (p.status === 'failed') {
        try { localStorage.removeItem(JOB_KEY); } catch (_) {}
        container.__jobState = p;
        _renderProgress(container);
        _renderTools(container);
        if (String(p.source || '').toLowerCase().includes('discovery')) {
          container.__discoveryJobState = p;
          _renderDiscovery(container);
        }
        if (scanOnDone) await _scanNow(container);
        const msg = p.error || (p.result && p.result.error) || 'tools_update_failed';
        _setAction(container, `<div style="color:var(--accent-red);font-size:12px">Error update (${_esc(String(label))}): ${_esc(msg)}</div>`);
        window.AtlasToast?.show(String(msg), 'error');
        return;
      }
      const msg = p.error || (p.result && p.result.error) || 'tools_update_failed';
      _setAction(container, `<div style="color:var(--accent-red);font-size:12px">Error update: ${_esc(msg)}</div>`);
      window.AtlasToast?.show(String(msg), 'error');
      return;
    } catch (_) {
      // silent retry
    }
  }
  _setAction(container, `<div class="codebox" style="font-size:11px">Update sigue en segundo plano (${_esc(jobId)}). Reescanea en unos minutos.</div>`);
}

async function _updateAll(container) {
  if (_isProtectedBlock(container.__toolsMenuData || {})) {
    _setAction(container, '<div style="color:var(--accent-red);font-size:12px">Update bloqueado: rama protegida activa.</div>');
    return;
  }
  if (!confirm('Actualizar todas las herramientas con upgrade_ready en segundo plano?')) return;
  _setAction(container, '<div class="codebox" style="font-size:11px">Iniciando actualización masiva...</div>');
  try {
    const r = await fetch('/api/tools/update-all', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ background: true, include_critical: true }),
    });
    const d = await r.json().catch(() => ({}));
    const payload = _payload(d) || {};
    if (!r.ok || d.ok === false) {
      throw new Error(d.error || payload.error || `HTTP ${r.status}`);
    }
    if (payload.status === 'noop') {
      _setAction(container, `<div class="chip green">${_esc(payload.message || 'Sin upgrades pendientes')}</div>`);
      return;
    }
    if (payload.queued && payload.job_id) {
      const count = Number(payload.total || (Array.isArray(payload.tools) ? payload.tools.length : 0));
      _setAction(container, `<div class="codebox" style="font-size:11px">Actualización masiva en segundo plano (${count} tools, job ${_esc(payload.job_id)})</div>`);
      window.AtlasToast?.show(`Update-all en background (${count})`, 'info');
      _watchJob(container, payload.job_id, 'all', { scanOnDone: true });
      return;
    }
    _setAction(container, `<div class="codebox" style="font-size:11px">${_esc(JSON.stringify(payload, null, 2))}</div>`);
    await _scanNow(container);
  } catch (e) {
    _setAction(container, `<div style="color:var(--accent-red);font-size:12px">Error update-all: ${_esc(e.message)}</div>`);
    window.AtlasToast?.show(String(e.message), 'error');
  }
}

async function _resumeOrDiscoverJob(container) {
  try {
    const raw = localStorage.getItem(JOB_KEY);
    if (raw) {
      const s = JSON.parse(raw);
      if (s && s.jobId) {
        _setAction(container, `<div class="codebox" style="font-size:11px">Reanudando seguimiento de job ${_esc(s.jobId)}...</div>`);
        _updateJobState(container, { job_id: String(s.jobId), tool: String(s.label || 'tools'), status: 'queued' });
        _watchJob(container, String(s.jobId), String(s.label || 'tools'), { scanOnDone: true });
        return;
      }
    }
  } catch (_) {}

  try {
    const r = await fetch('/api/tools/job/latest');
    const d = await r.json().catch(() => ({}));
    const p = _payload(d) || {};
    if (!r.ok || d.ok === false) return;
    if (!p.found || !p.job || !p.job.job_id) return;
    const job = p.job;
    const label = String(job.tool || 'tools');
    _setAction(container, `<div class="codebox" style="font-size:11px">Job activo detectado (${_esc(job.job_id)}). Reanudando seguimiento...</div>`);
    _updateJobState(container, job);
    _watchJob(container, String(job.job_id), label, { scanOnDone: true });
  } catch (_) {
    // silent
  }
}

async function _runDiscovery(container) {
  const grid = container.querySelector('#tm-discovery-grid');
  if (grid) {
    grid.innerHTML = `<div class="empty-state" style="grid-column:1/-1">
      <div class="spinner"></div>
      <div class="empty-sub" style="margin-top:8px">Buscando herramientas útiles...</div>
    </div>`;
  }
  try {
    const r = await fetch('/api/tools/discovery/search');
    const d = await r.json().catch(() => ({}));
    const p = _payload(d) || {};
    if (!r.ok || d.ok === false) throw new Error(d.error || p.error || `HTTP ${r.status}`);
    container.__discoveryData = p;
    container.__discoveryJobState = null;
    _renderDiscovery(container);
  } catch (e) {
    if (grid) {
      grid.innerHTML = `<div class="empty-state" style="grid-column:1/-1">
        <div class="empty-title">Discovery no disponible</div>
        <div class="empty-sub">${_esc(String(e.message || e))}</div>
      </div>`;
    }
  }
}

function _renderDiscovery(container) {
  const data = container.__discoveryData || {};
  const dJob = container.__discoveryJobState || null;
  const allRows = Array.isArray(data.candidates) ? data.candidates : [];
  const invRows = Array.isArray((container.__toolsMenuData || {}).tools) ? (container.__toolsMenuData || {}).tools : [];
  const discoveryAliases = {
    playwright: ['playwright', 'playwright_py'],
    playwright_py: ['playwright_py', 'playwright'],
    task: ['task', 'taskwarrior'],
    taskwarrior: ['taskwarrior', 'task'],
    'yt-dlp': ['yt-dlp', 'ytdlp'],
    ytdlp: ['ytdlp', 'yt-dlp'],
  };
  const installedIds = new Set(
    invRows
      .filter(t => ['ok', 'warn'].includes(String(t?.status || '')) && String(t?.health || '').toLowerCase() !== 'not_detected')
      .map(t => String(t?.id || '').toLowerCase())
  );
  const rows = allRows.filter((r) => {
    const rid = String(r?.id || '').toLowerCase();
    if (!rid) return false;
    const candidates = discoveryAliases[rid] || [rid];
    return !candidates.some(id => installedIds.has(String(id).toLowerCase()));
  });
  const grid = container.querySelector('#tm-discovery-grid');
  const count = container.querySelector('#tm-discovery-count');
  const alerts = container.querySelector('#tm-discovery-alerts');
  const progress = container.querySelector('#tm-discovery-progress');
  if (count) count.textContent = `${rows.length} candidatos`;
  if (!grid) return;
  if (!rows.length) {
    grid.innerHTML = `<div class="empty-state" style="grid-column:1/-1">
      <div class="empty-title">Sin candidatos</div>
      <div class="empty-sub">No se encontraron herramientas nuevas.</div>
    </div>`;
    return;
  }
  const available = rows.filter(r => r.network_available).length;
  const unreachable = Math.max(0, rows.length - available);
  if (alerts) {
    alerts.innerHTML = `<div class="tools-alert-box">
      <span class="chip green">Disponibles: ${available}</span>
      <span class="chip ${unreachable > 0 ? 'orange' : 'green'}">Sin red: ${unreachable}</span>
      <button class="action-btn primary" id="tm-discovery-install-all" style="margin-left:10px;">Instalar Todo</button>
    </div>`;
  }
  if (progress) _renderDiscoveryProgress(progress, dJob);
  grid.innerHTML = rows.map((t) => {
    const cls = t.network_available ? 'active' : 'degraded';
    const dot = t.network_available ? 'ok' : 'degraded';
    const latency = t.latency_ms != null ? `${t.latency_ms}ms` : '--';
    const latest = t.latest_version ? `<div class="provider-role">Latest: ${_esc(t.latest_version)}</div>` : '';
    const netChip = t.network_available ? '<span class="chip green">NETWORK_OK</span>' : '<span class="chip orange">NETWORK_OFF</span>';
    const installDisabled = !t.network_available ? 'disabled' : '';
    const dProg = _discoveryToolProgressState(dJob, t.id);
    const dProgChip = dProg ? `<span class="chip ${dProg.cls}">${_esc(dProg.label)}</span>` : '';
    return `<div class="provider-card ${cls}">
      <div class="provider-latency">${_esc(t.category || 'tool')}</div>
      <div class="provider-name">${_esc(t.name || t.id)}</div>
      <div class="provider-role">${_esc(t.multitask_value || '--')}</div>
      <div class="provider-role">Método: ${_esc(t.install_method || '--')} · Target: ${_esc(t.install_target || '--')}</div>
      ${latest}
      <div class="provider-status">
        <span class="provider-dot ${dot}"></span>
        ${_esc(t.network_available ? `red ${latency}` : (t.network_error || 'sin conexión'))}
      </div>
      <div style="margin-top:8px;display:flex;gap:6px;flex-wrap:wrap">
        ${netChip}
        <span class="chip blue">Score ${_esc(String(t.score ?? '--'))}</span>
        ${dProgChip}
      </div>
      <div style="margin-top:10px;display:flex;gap:8px;flex-wrap:wrap">
        <button class="action-btn" data-action="install-discovery"
          data-tool="${_esc(t.id)}"
          data-method="${_esc(t.install_method)}"
          data-target="${_esc(t.install_target)}"
          ${installDisabled}>Instalar</button>
        <a class="action-btn" href="${_esc(t.docs_url || '#')}" target="_blank" rel="noopener">Docs</a>
      </div>
    </div>`;
  }).join('');

  alerts?.querySelector('#tm-discovery-install-all')?.addEventListener('click', () => _installAllDiscoveredTools(container));

  grid.querySelectorAll('[data-action="install-discovery"]').forEach((btn) => {
    btn.addEventListener('click', () => _installDiscoveredTool(
      container,
      btn.getAttribute('data-tool') || '',
      btn.getAttribute('data-method') || '',
      btn.getAttribute('data-target') || '',
    ));
  });
}

function _renderDiscoveryProgress(el, job) {
  if (!el) return;
  if (!job || !String(job.source || '').toLowerCase().includes('discovery')) {
    el.innerHTML = '';
    return;
  }
  const total = Number(job.total || (Array.isArray(job.tools) ? job.tools.length : 0) || 0);
  const done = Number(job.done || (Array.isArray(job.results) ? job.results.length : 0) || 0);
  const failed = Number(job.failed || 0);
  const pct = total > 0 ? Math.max(0, Math.min(100, Math.round((done / total) * 100))) : 0;
  const success = Math.max(0, done - failed);
  const st = _normalizeJobStatus(job);
  const status = st.toUpperCase();
  const terminalWithFailures = st === 'done_with_errors' || (st === 'failed' && total > 0 && done >= total && failed > 0);
  const statusLabel = terminalWithFailures ? 'DONE_WITH_ERRORS' : status;
  const current = String(job.current_tool || '').trim();
  const color = (st === 'done' && !terminalWithFailures)
    ? 'var(--accent-green)'
    : (terminalWithFailures ? 'var(--accent-orange)' : (st === 'failed' ? 'var(--accent-red)' : (failed > 0 ? 'var(--accent-orange)' : 'var(--accent-primary)')));
  el.innerHTML = `
    <div class="codebox" style="margin:10px 0;padding:10px;">
      <div style="display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin-bottom:8px;">
        <span class="chip blue">DISCOVERY JOB ${_esc(String(job.job_id || '--'))}</span>
        <span class="chip ${(st === 'done' && !terminalWithFailures) ? 'green' : (terminalWithFailures ? 'orange' : (st === 'failed' ? 'red' : 'orange'))}">${_esc(statusLabel)}</span>
        <span style="font-size:12px;color:var(--text-muted)">Lote: ${done}/${total} · Exitosas: ${success} · Fallos: ${failed}</span>
        ${current ? `<span style="font-size:12px;color:var(--text-muted)">Actual: ${_esc(current)}</span>` : ''}
      </div>
      <div style="height:10px;background:rgba(255,255,255,.08);border-radius:999px;overflow:hidden;">
        <div style="height:100%;width:${pct}%;background:${color};transition:width .25s ease;"></div>
      </div>
      <div style="font-size:11px;color:var(--text-muted);margin-top:6px;">${pct}% completado</div>
    </div>
  `;
}

function _discoveryToolProgressState(job, toolId) {
  if (!job || !toolId) return null;
  const tid = String(toolId).toLowerCase();
  const status = _normalizeJobStatus(job);
  const current = String(job.current_tool || '').toLowerCase();
  const listed = Array.isArray(job.tools) ? job.tools.map(x => String(x).toLowerCase()) : [];
  const rs = Array.isArray(job.results) ? job.results : [];
  const found = rs.find(r => String((r && r.tool) || '').toLowerCase() === tid);
  if (found) {
    if (found.ok === true) return { cls: 'green', label: 'INSTALADA' };
    if (found.ok === false) return { cls: 'red', label: 'FALLO' };
  }
  if (current === tid && (status === 'running' || status === 'queued')) return { cls: 'blue', label: 'EN CURSO' };
  if (listed.includes(tid) && (status === 'running' || status === 'queued')) return { cls: 'orange', label: 'PENDIENTE' };
  return null;
}

async function _installDiscoveredTool(container, toolId, method, target) {
  if (!toolId || !method || !target) return;
  if (!confirm(`Instalar "${toolId}" via ${method} (${target}) en background?`)) return;
  _setAction(container, `<div class="codebox" style="font-size:11px">Instalando herramienta descubierta: ${_esc(toolId)}...</div>`);
  try {
    const r = await fetch('/api/tools/discovery/install', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        tool_id: toolId,
        install_method: method,
        install_target: target,
        background: true,
      }),
    });
    const d = await r.json().catch(() => ({}));
    const p = _payload(d) || {};
    if (!r.ok || d.ok === false) throw new Error(d.error || p.error || `HTTP ${r.status}`);
    if (p.queued && p.job_id) {
      container.__discoveryJobState = { job_id: String(p.job_id), status: 'queued', source: 'discovery', tools: [toolId], total: 1, done: 0, failed: 0 };
      _renderDiscovery(container);
      _watchJob(container, String(p.job_id), `install:${toolId}`, { scanOnDone: true });
      return;
    }
    _setAction(container, `<div class="codebox" style="font-size:11px">${_esc(JSON.stringify(p, null, 2))}</div>`);
  } catch (e) {
    _setAction(container, `<div style="color:var(--accent-red);font-size:12px">Error instalando: ${_esc(String(e.message || e))}</div>`);
    window.AtlasToast?.show(String(e.message || e), 'error');
  }
}

async function _installAllDiscoveredTools(container) {
  const data = container.__discoveryData || {};
  const rows = Array.isArray(data.candidates) ? data.candidates : [];
  const tools = rows
    .filter(r => r && r.network_available)
    .map(r => ({ id: r.id, install_method: r.install_method, install_target: r.install_target }));
  if (!tools.length) {
    _setAction(container, '<div class="chip orange">No hay herramientas discovery disponibles para descargar.</div>');
    return;
  }
  if (!confirm(`Descargar e instalar ${tools.length} herramientas nuevas en background?`)) return;
  _setAction(container, `<div class="codebox" style="font-size:11px">Lanzando descarga masiva (${tools.length})...</div>`);
  try {
    const r = await fetch('/api/tools/discovery/install-all', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ tools, background: true }),
    });
    const d = await r.json().catch(() => ({}));
    const p = _payload(d) || {};
    if (!r.ok || d.ok === false) throw new Error(d.error || p.error || `HTTP ${r.status}`);
    if (p.status === 'noop') {
      _setAction(container, `<div class="chip green">${_esc(p.message || 'Sin pendientes')}</div>`);
      return;
    }
    if (p.queued && p.job_id) {
      container.__discoveryJobState = {
        job_id: String(p.job_id),
        status: 'queued',
        source: 'discovery-bulk',
        tools: Array.isArray(p.tools) ? p.tools : tools.map(t => t.id),
        total: Number(p.total || tools.length),
        done: 0,
        failed: 0,
      };
      _renderDiscovery(container);
      _watchJob(container, String(p.job_id), 'discovery_all', { scanOnDone: true });
      return;
    }
  } catch (e) {
    _setAction(container, `<div style="color:var(--accent-red);font-size:12px">Error descargar todo: ${_esc(String(e.message || e))}</div>`);
    window.AtlasToast?.show(String(e.message || e), 'error');
  }
}

function _toolProgressState(job, toolId) {
  if (!job || !toolId) return null;
  const status = _normalizeJobStatus(job);
  const current = String(job.current_tool || '').toLowerCase();
  const doneSet = new Set();
  const failSet = new Set();
  const rs = Array.isArray(job.results) ? job.results : [];
  rs.forEach((r) => {
    const tid = String((r && r.tool) || '').toLowerCase();
    if (!tid) return;
    if (r.ok === true) doneSet.add(tid);
    else if (r.ok === false) failSet.add(tid);
  });
  const tid = String(toolId).toLowerCase();
  if (doneSet.has(tid)) return { cls: 'green', label: 'ACTUALIZADA' };
  if (failSet.has(tid)) return { cls: 'red', label: 'FALLO' };
  if (current === tid && (status === 'running' || status === 'queued')) return { cls: 'blue', label: 'EN CURSO' };
  const listed = Array.isArray(job.tools) ? job.tools.map(x => String(x).toLowerCase()) : [];
  if (listed.includes(tid) && (status === 'running' || status === 'queued')) return { cls: 'orange', label: 'PENDIENTE' };
  return null;
}

function _getJobToolResult(job, toolId) {
  if (!job || !toolId) return null;
  const tid = String(toolId).toLowerCase();
  const rs = Array.isArray(job.results) ? job.results : [];
  for (const r of rs) {
    const rid = String((r && r.tool) || '').toLowerCase();
    if (rid === tid) return r;
  }
  return null;
}

function _openDetail(container, toolId) {
  const modal = container.querySelector('#tm-detail-modal');
  const title = container.querySelector('#tm-detail-title');
  const body = container.querySelector('#tm-detail-body');
  if (!modal || !title || !body) return;
  const job = container.__jobState || null;
  const result = _getJobToolResult(job, toolId);
  const prog = _toolProgressState(job, toolId);
  const current = String((job && job.current_tool) || '');
  title.textContent = `Detalle: ${toolId}`;

  let statusText = prog ? prog.label : 'SIN_DATOS';
  if (!result && current && current.toLowerCase() === String(toolId).toLowerCase()) {
    statusText = 'EN_CURSO';
  }

  const steps = Array.isArray(result?.steps) ? result.steps : [];
  const stepsHtml = steps.length
    ? `<div style="margin-top:10px;">
         <div style="font-size:12px;color:var(--text-muted);margin-bottom:6px;">Steps</div>
         ${steps.map((s) => `<div class="codebox" style="font-size:11px;margin-bottom:6px;">${_esc(JSON.stringify(s, null, 2))}</div>`).join('')}
       </div>`
    : '';

  body.innerHTML = `
    <div style="display:flex;gap:8px;flex-wrap:wrap;margin-bottom:10px;">
      <span class="chip blue">Tool: ${_esc(String(toolId))}</span>
      <span class="chip ${statusText === 'ACTUALIZADA' ? 'green' : (statusText === 'FALLO' ? 'red' : 'orange')}">${_esc(statusText)}</span>
      <span class="chip">Job: ${_esc(String(job?.job_id || '--'))}</span>
    </div>
    ${result ? `<div class="codebox" style="font-size:11px;">${_esc(JSON.stringify(result, null, 2))}</div>` : `<div style="font-size:12px;color:var(--text-muted)">Aún no hay resultado final para esta herramienta.</div>`}
    ${stepsHtml}
    <div id="tm-runner-log-wrap" style="margin-top:12px;">
      <div style="font-size:12px;color:var(--text-muted);margin-bottom:6px;">Runner Log</div>
      <div class="codebox" id="tm-runner-log" style="font-size:11px;max-height:220px;overflow:auto;">Cargando...</div>
    </div>
  `;
  modal.style.display = 'block';
  _loadRunnerLog(body, job?.job_id);
}

async function _loadRunnerLog(scopeEl, jobId) {
  const logEl = scopeEl?.querySelector('#tm-runner-log');
  if (!logEl) return;
  if (!jobId) {
    logEl.textContent = 'Sin job_id';
    return;
  }
  try {
    const r = await fetch(`/api/tools/job/runner-log/${encodeURIComponent(String(jobId))}`);
    const d = await r.json().catch(() => ({}));
    const p = _payload(d) || {};
    if (!r.ok || d.ok === false) throw new Error(d.error || p.error || `HTTP ${r.status}`);
    if (!p.found) {
      logEl.textContent = 'Runner log no disponible.';
      return;
    }
    logEl.textContent = String(p.tail || '').trim() || '(vacío)';
  } catch (e) {
    logEl.textContent = `Error cargando runner log: ${String(e.message || e)}`;
  }
}

function _closeDetail(container) {
  const modal = container.querySelector('#tm-detail-modal');
  if (modal) modal.style.display = 'none';
}

function _updateJobState(container, job) {
  if (!container) return;
  const normalized = job ? { ...job, status: _normalizeJobStatus(job) } : null;
  container.__jobState = normalized;
  if (normalized && String(normalized.source || '').toLowerCase().includes('discovery')) {
    container.__discoveryJobState = normalized;
    _renderDiscovery(container);
  }
  _renderProgress(container);
  _renderTools(container);
}

function _renderProgress(container) {
  const el = container.querySelector('#tm-progress');
  if (!el) return;
  const job = container.__jobState || null;
  if (!job) {
    el.innerHTML = '';
    return;
  }
  const total = Number(job.total || (Array.isArray(job.tools) ? job.tools.length : 0) || 0);
  const done = Number(job.done || (Array.isArray(job.results) ? job.results.length : 0) || 0);
  const failed = Number(job.failed || 0);
  const pct = total > 0 ? Math.max(0, Math.min(100, Math.round((done / total) * 100))) : 0;
  const success = Math.max(0, done - failed);
  const st = _normalizeJobStatus(job);
  const status = st.toUpperCase();
  const terminalWithFailures = st === 'done_with_errors' || (st === 'failed' && total > 0 && done >= total && failed > 0);
  const statusLabel = terminalWithFailures ? 'DONE_WITH_ERRORS' : status;
  const current = String(job.current_tool || '').trim();
  const stateColor = (st === 'done' && !terminalWithFailures)
    ? 'var(--accent-green)'
    : (terminalWithFailures ? 'var(--accent-orange)' : (st === 'failed' ? 'var(--accent-red)' : (failed > 0 ? 'var(--accent-orange)' : 'var(--accent-primary)')));
  el.innerHTML = `
    <div class="codebox" style="margin:10px 0;padding:10px;">
      <div style="display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin-bottom:8px;">
        <span class="chip blue">JOB ${_esc(String(job.job_id || '--'))}</span>
        <span class="chip ${(st === 'done' && !terminalWithFailures) ? 'green' : (terminalWithFailures ? 'orange' : (st === 'failed' ? 'red' : 'orange'))}">${_esc(statusLabel)}</span>
        <span style="font-size:12px;color:var(--text-muted)">Lote: ${done}/${total} · Exitosas: ${success} · Fallos: ${failed}</span>
        ${current ? `<span style="font-size:12px;color:var(--text-muted)">Actual: ${_esc(current)}</span>` : ''}
        ${(st === 'failed') && !terminalWithFailures ? `<button class="action-btn" id="tm-progress-retry">Reintentar job atascado</button>` : ''}
        <button class="action-btn" id="tm-progress-export" style="margin-left:auto;">Exportar reporte</button>
      </div>
      <div style="height:10px;background:rgba(255,255,255,.08);border-radius:999px;overflow:hidden;">
        <div style="height:100%;width:${pct}%;background:${stateColor};transition:width .25s ease;"></div>
      </div>
      <div style="font-size:11px;color:var(--text-muted);margin-top:6px;">
        ${pct}% completado
        ${pct === 0 && (st === 'running' || st === 'queued') ? ' · iniciando primer tool (sin completar aún)' : ''}
      </div>
    </div>
  `;
  el.querySelector('#tm-progress-retry')?.addEventListener('click', () => _retryCurrentJob(container));
  el.querySelector('#tm-progress-export')?.addEventListener('click', () => _exportCurrentJob(container));
}

async function _retryCurrentJob(container) {
  const job = container.__jobState || null;
  const jobId = String((job && job.job_id) || '').trim();
  if (!jobId) {
    _setAction(container, '<div style="color:var(--accent-red);font-size:12px">No hay job para reintentar.</div>');
    return;
  }
  if (!confirm(`Reintentar job ${jobId} con herramientas pendientes/fallidas?`)) return;
  _setAction(container, `<div class="codebox" style="font-size:11px">Reintentando job ${_esc(jobId)}...</div>`);
  try {
    const r = await fetch(`/api/tools/job/retry/${encodeURIComponent(jobId)}`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
    });
    const d = await r.json().catch(() => ({}));
    const p = _payload(d) || {};
    if (!r.ok || d.ok === false) throw new Error(d.error || p.error || `HTTP ${r.status}`);
    if (p.status === 'noop' || p.queued !== true || !p.job_id) {
      _setAction(container, `<div class="chip green">${_esc(p.message || 'Sin pendientes para reintento')}</div>`);
      return;
    }
    _setAction(container, `<div class="codebox" style="font-size:11px">Reintento lanzado (job ${_esc(String(p.job_id))})</div>`);
    _watchJob(container, String(p.job_id), 'retry', { scanOnDone: true });
  } catch (e) {
    _setAction(container, `<div style="color:var(--accent-red);font-size:12px">Error reintentando job: ${_esc(e.message)}</div>`);
    window.AtlasToast?.show(String(e.message), 'error');
  }
}

async function _exportCurrentJob(container) {
  const job = container.__jobState || null;
  const jobId = String((job && job.job_id) || '').trim();
  if (!jobId) {
    _setAction(container, '<div style="color:var(--accent-red);font-size:12px">No hay job activo para exportar.</div>');
    return;
  }
  _setAction(container, `<div class="codebox" style="font-size:11px">Exportando reporte del job ${_esc(jobId)}...</div>`);
  try {
    const r = await fetch(`/api/tools/job/export/${encodeURIComponent(jobId)}`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
    });
    const d = await r.json().catch(() => ({}));
    const p = _payload(d) || {};
    if (!r.ok || d.ok === false) throw new Error(d.error || p.error || `HTTP ${r.status}`);
    const path = p.report_path || p.report_url || '--';
    _setAction(container, `<div class="codebox" style="font-size:11px">Reporte exportado: ${_esc(String(path))}</div>`);
    window.AtlasToast?.show('Reporte exportado', 'success');
  } catch (e) {
    _setAction(container, `<div style="color:var(--accent-red);font-size:12px">Error exportando reporte: ${_esc(e.message)}</div>`);
    window.AtlasToast?.show(String(e.message), 'error');
  }
}

function _txt(container, selector, value) {
  const el = container.querySelector(selector);
  if (el) el.textContent = String(value ?? '--');
}

function _setAction(container, html) {
  const el = container.querySelector('#tm-action');
  if (el) el.innerHTML = html || '';
}

window.AtlasModuleToolsMenu = { id: 'tools-menu' };
