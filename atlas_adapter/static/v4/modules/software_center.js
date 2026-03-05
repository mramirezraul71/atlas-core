/**
 * ATLAS v4.3 - Software Center
 * Inventario profesional de software, drivers, stack dev y ciclos de mantenimiento.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'software-center-module';
const MENU_ENDPOINT = '/api/software/menu';
const DISCOVERY_ENDPOINT = '/api/software/discovery/search';
const JOB_KEY = 'atlas-software-center-active-job';
const MAINT_KEY = 'atlas-software-center-active-maintenance-job';

function _esc(s) {
  const d = document.createElement('span');
  d.textContent = s ?? '';
  return d.innerHTML;
}

function _payload(data) {
  if (!data) return null;
  return data.data ?? data;
}

function _txt(container, sel, val) {
  const el = container.querySelector(sel);
  if (el) el.textContent = String(val ?? '--');
}

function _statusChip(status) {
  const s = String(status || '').toLowerCase();
  if (s === 'ok') return 'green';
  if (s === 'warn') return 'orange';
  if (s === 'error') return 'red';
  if (s === 'running') return 'blue';
  if (s === 'done') return 'green';
  if (s === 'done_with_errors') return 'orange';
  if (s === 'failed') return 'red';
  return 'blue';
}

function _pct(done, total, status) {
  const d = Number(done || 0);
  const t = Number(total || 0);
  if (t > 0) return Math.max(0, Math.min(100, Math.floor((d * 100) / t)));
  return String(status || '').toLowerCase() === 'done' ? 100 : 0;
}

export default {
  id: 'software-center',
  label: 'Software Center',
  icon: 'package',
  category: 'system',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Dashboard
          </button>
          <h2>Software Center</h2>
          <div style="margin-left:auto"><span class="live-badge">LIVE</span></div>
        </div>
        <div class="module-body">
          <div class="stat-row" style="margin-bottom:16px;">
            <div class="stat-card hero">
              <div class="stat-card-label">Software</div>
              <div class="stat-card-value green" id="sc-software-total">--</div>
              <div class="stat-card-sub" id="sc-scan-time">sin escaneo</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Update Ready</div>
              <div class="stat-card-value orange" id="sc-update-ready">--</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Drivers Críticos</div>
              <div class="stat-card-value accent" id="sc-drivers-critical">--</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Disponibles en Red</div>
              <div class="stat-card-value green" id="sc-candidates-available">--</div>
            </div>
          </div>

          <div class="action-bar">
            <button class="action-btn primary" id="sc-btn-scan">Escanear Software</button>
            <button class="action-btn" id="sc-btn-discovery">Buscar en Red</button>
            <button class="action-btn" id="sc-btn-apply-all">Actualizar / Instalar Todo</button>
            <button class="action-btn" id="sc-btn-maintenance">Ciclo Mantenimiento</button>
            <button class="action-btn" id="sc-btn-open-tools">Ir a Tools Menu</button>
          </div>

          <div id="sc-alerts"></div>
          <div id="sc-progress"></div>
          <div id="sc-maint-progress"></div>
          <div id="sc-action" style="margin:10px 0 14px;"></div>

          <div class="section-title">Software Instalado y Versionado <span class="count" id="sc-software-count">--</span></div>
          <div class="provider-grid" id="sc-software-grid"></div>

          <div class="section-title" style="margin-top:18px;">Drivers Requeridos <span class="count" id="sc-driver-count">--</span></div>
          <div class="provider-grid" id="sc-driver-grid"></div>

          <div class="section-title" style="margin-top:18px;">Expansion Network <span class="count" id="sc-candidate-count">--</span></div>
          <div class="provider-grid" id="sc-candidate-grid"></div>

          <div class="section-title" style="margin-top:18px;">Stack de Desarrollo ATLAS <span class="count" id="sc-dev-count">--</span></div>
          <div class="provider-grid" id="sc-dev-grid"></div>
        </div>
      </div>
    `;

    container.querySelector('#sc-btn-scan')?.addEventListener('click', () => _scan(container));
    container.querySelector('#sc-btn-discovery')?.addEventListener('click', () => _discovery(container, true));
    container.querySelector('#sc-btn-apply-all')?.addEventListener('click', () => _applyAll(container));
    container.querySelector('#sc-btn-maintenance')?.addEventListener('click', () => _maintenance(container));
    container.querySelector('#sc-btn-open-tools')?.addEventListener('click', () => {
      location.hash = '/tools-menu';
    });

    _refresh(container);
    _resumeJobs(container);
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
    const j = await r.json().catch(() => ({}));
    if (!r.ok || !j.ok) {
      _setAction(container, `<span style="color:var(--accent-red)">Error: ${_esc(j.error || r.status)}</span>`);
      return;
    }
    _applyData(container, _payload(j));
  } catch (e) {
    _setAction(container, `<span style="color:var(--accent-red)">Error: ${_esc(e.message)}</span>`);
  }
}

function _applyData(container, data) {
  if (!data) return;
  container.__softwareData = data;
  const summary = data.summary || {};
  const scanAt = data.generated_at ? new Date(data.generated_at).toLocaleTimeString() : '--';
  _txt(container, '#sc-software-total', summary.software_total ?? '--');
  _txt(container, '#sc-update-ready', summary.software_update_ready ?? '--');
  _txt(container, '#sc-drivers-critical', summary.drivers_error ?? 0);
  _txt(container, '#sc-candidates-available', summary.candidates_available ?? 0);
  _txt(container, '#sc-scan-time', `ultimo escaneo: ${scanAt}`);

  _renderAlerts(container, data);
  _renderSoftware(container);
  _renderDrivers(container);
  _renderCandidates(container);
  _renderDevStack(container);
  _renderJobProgress(container);
  _renderMaintenanceProgress(container);
}

function _renderAlerts(container, data) {
  const summary = data.summary || {};
  const bits = [];
  if (Number(summary.software_update_ready || 0) > 0) bits.push(`<span class="chip orange">UPDATE_READY: ${Number(summary.software_update_ready || 0)}</span>`);
  if (Number(summary.software_error || 0) > 0) bits.push(`<span class="chip red">SOFTWARE_ERROR: ${Number(summary.software_error || 0)}</span>`);
  if (Number(summary.drivers_error || 0) > 0) bits.push(`<span class="chip red">DRIVERS_CRITICOS: ${Number(summary.drivers_error || 0)}</span>`);
  if (Number(summary.dev_error || 0) > 0) bits.push(`<span class="chip red">STACK_DEV_ERROR: ${Number(summary.dev_error || 0)}</span>`);
  container.querySelector('#sc-alerts').innerHTML = bits.length ? `<div class="tools-alert-box">${bits.join(' ')}</div>` : '';
}

function _cardForSoftware(row) {
  const status = String(row.status || '');
  const statusColor = _statusChip(status);
  const health = row.health || '--';
  const version = row.version || '--';
  const latest = row.latest_version || '--';
  const details = row.details || '--';
  const installed = !!row.installed;
  const net = row.network_available ? `red ${row.latency_ms || '--'}ms` : 'sin red';
  return `
    <div class="tool-card ${status === 'error' ? 'error' : (status === 'warn' ? 'warning' : '')}">
      <div class="tool-head">
        <div class="tool-name">${_esc(row.name || row.id || 'software')}</div>
        <div class="tool-category">${_esc(row.category || 'software')}</div>
      </div>
      <div class="tool-lines">
        <div>VERSION: ${_esc(version)}</div>
        <div>LATEST: ${_esc(latest)}</div>
      </div>
      <div class="tool-status-line"><span class="dot"></span> ${_esc(health)}</div>
      <div class="tool-tags">
        <span class="chip ${statusColor}">${_esc(status.toUpperCase() || 'UNKNOWN')}</span>
        ${row.update_ready ? '<span class="chip orange">UPGRADE_READY</span>' : ''}
        ${installed ? '<span class="chip green">INSTALADA</span>' : '<span class="chip blue">PENDIENTE</span>'}
      </div>
      <div class="tool-details">${_esc(details)}</div>
      <div class="tool-details">${_esc(net)}</div>
    </div>
  `;
}

function _cardForDriver(row) {
  const status = String(row.status || '');
  const statusColor = _statusChip(status);
  const version = row.version || '--';
  const count = Number(row.count || 0);
  return `
    <div class="tool-card ${status === 'error' ? 'error' : (status === 'warn' ? 'warning' : '')}">
      <div class="tool-head">
        <div class="tool-name">${_esc(row.name || row.id || 'driver')}</div>
        <div class="tool-category">driver</div>
      </div>
      <div class="tool-lines">
        <div>VERSION: ${_esc(version)}</div>
        <div>COINCIDENCIAS: ${count}</div>
      </div>
      <div class="tool-tags">
        <span class="chip ${statusColor}">${_esc((status || 'unknown').toUpperCase())}</span>
        ${row.critical ? '<span class="chip red">CRITICA</span>' : '<span class="chip blue">OPTIONAL</span>'}
      </div>
    </div>
  `;
}

function _cardForCandidate(row) {
  const ready = !!row.network_available;
  return `
    <div class="tool-card ${ready ? '' : 'warning'}">
      <div class="tool-head">
        <div class="tool-name">${_esc(row.name || row.id || 'candidate')}</div>
        <div class="tool-category">${_esc(row.category || 'candidate')}</div>
      </div>
      <div class="tool-lines">
        <div>METODO: ${_esc(row.install_method || '--')}</div>
        <div>TARGET: ${_esc(row.install_target || '--')}</div>
      </div>
      <div class="tool-tags">
        <span class="chip ${ready ? 'green' : 'orange'}">${ready ? 'NETWORK_OK' : 'SIN_RED'}</span>
        <span class="chip blue">Score ${Number(row.score || 0)}</span>
      </div>
      <div class="tool-details">${ready ? `latencia ${_esc(String(row.latency_ms || '--'))}ms` : _esc(row.network_error || '--')}</div>
    </div>
  `;
}

function _cardForDev(row) {
  const status = String(row.status || '');
  const statusColor = _statusChip(status);
  return `
    <div class="tool-card ${status === 'error' ? 'error' : (status === 'warn' ? 'warning' : '')}">
      <div class="tool-head">
        <div class="tool-name">${_esc(row.name || row.id || 'component')}</div>
        <div class="tool-category">${_esc(row.type || 'dev')}</div>
      </div>
      <div class="tool-lines">
        <div>VERSION: ${_esc(row.version || '--')}</div>
        <div>HEALTH: ${_esc(row.health || '--')}</div>
      </div>
      <div class="tool-tags">
        <span class="chip ${statusColor}">${_esc((status || 'unknown').toUpperCase())}</span>
      </div>
      <div class="tool-details">${_esc(row.details || '--')}</div>
    </div>
  `;
}

function _renderSoftware(container) {
  const rows = Array.isArray(container.__softwareData?.software) ? container.__softwareData.software : [];
  _txt(container, '#sc-software-count', `${rows.length} items`);
  const grid = container.querySelector('#sc-software-grid');
  if (!rows.length) {
    grid.innerHTML = `<div class="empty-state" style="grid-column:1/-1"><div class="empty-title">Sin software detectado</div></div>`;
    return;
  }
  grid.innerHTML = rows.map(_cardForSoftware).join('');
}

function _renderDrivers(container) {
  const rows = Array.isArray(container.__softwareData?.drivers) ? container.__softwareData.drivers : [];
  _txt(container, '#sc-driver-count', `${rows.length} drivers`);
  const grid = container.querySelector('#sc-driver-grid');
  if (!rows.length) {
    grid.innerHTML = `<div class="empty-state" style="grid-column:1/-1"><div class="empty-title">Sin requisitos de driver</div></div>`;
    return;
  }
  grid.innerHTML = rows.map(_cardForDriver).join('');
}

function _renderCandidates(container) {
  const rows = Array.isArray(container.__softwareData?.network_candidates) ? container.__softwareData.network_candidates : [];
  _txt(container, '#sc-candidate-count', `${rows.length} candidatos`);
  const grid = container.querySelector('#sc-candidate-grid');
  if (!rows.length) {
    grid.innerHTML = `<div class="empty-state" style="grid-column:1/-1"><div class="empty-title">No hay candidatos pendientes</div></div>`;
    return;
  }
  grid.innerHTML = rows.map(_cardForCandidate).join('');
}

function _renderDevStack(container) {
  const rows = Array.isArray(container.__softwareData?.development_stack) ? container.__softwareData.development_stack : [];
  _txt(container, '#sc-dev-count', `${rows.length} componentes`);
  const grid = container.querySelector('#sc-dev-grid');
  if (!rows.length) {
    grid.innerHTML = `<div class="empty-state" style="grid-column:1/-1"><div class="empty-title">Sin datos de stack</div></div>`;
    return;
  }
  grid.innerHTML = rows.map(_cardForDev).join('');
}

async function _scan(container) {
  _setAction(container, 'Escaneando software...');
  try {
    const r = await fetch('/api/software/watchdog/scan', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ force: true }),
    });
    const j = await r.json().catch(() => ({}));
    if (!r.ok || !j.ok) {
      _setAction(container, `<span style="color:var(--accent-red)">Error scan: ${_esc(j.error || r.status)}</span>`);
      return;
    }
    _setAction(container, '<span style="color:var(--accent-green)">Escaneo completado</span>');
    _applyData(container, _payload(j));
  } catch (e) {
    _setAction(container, `<span style="color:var(--accent-red)">Error scan: ${_esc(e.message)}</span>`);
  }
}

async function _discovery(container, force = false) {
  _setAction(container, 'Buscando software en red...');
  try {
    const r = await fetch(`${DISCOVERY_ENDPOINT}?force=${force ? 'true' : 'false'}`);
    const j = await r.json().catch(() => ({}));
    if (!r.ok || !j.ok) {
      _setAction(container, `<span style="color:var(--accent-red)">Error discovery: ${_esc(j.error || r.status)}</span>`);
      return;
    }
    const data = _payload(j) || {};
    _setAction(
      container,
      `<span style="color:var(--accent-green)">Discovery: ${Number(data.summary?.available || 0)} disponibles, ${Number(data.summary?.unreachable || 0)} sin red</span>`,
    );
    await _refresh(container);
  } catch (e) {
    _setAction(container, `<span style="color:var(--accent-red)">Error discovery: ${_esc(e.message)}</span>`);
  }
}

async function _applyAll(container) {
  if (!confirm('¿Aplicar instalación/actualización masiva de software pendiente?')) return;
  _setAction(container, 'Encolando actualización masiva de software...');
  try {
    const r = await fetch('/api/software/apply-all', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ background: true, include_optional: true }),
    });
    const j = await r.json().catch(() => ({}));
    if (!r.ok || !j.ok) {
      _setAction(container, `<span style="color:var(--accent-red)">Error apply-all: ${_esc(j.error || r.status)}</span>`);
      return;
    }
    const data = _payload(j) || {};
    if (!data.queued) {
      _setAction(container, `<span style="color:var(--accent-green)">${_esc(data.message || 'Sin items pendientes')}</span>`);
      return;
    }
    container.__softwareJob = data;
    localStorage.setItem(JOB_KEY, String(data.job_id || ''));
    _setAction(container, `Job software iniciado: ${_esc(data.job_id)}`);
    _watchSoftwareJob(container);
  } catch (e) {
    _setAction(container, `<span style="color:var(--accent-red)">Error apply-all: ${_esc(e.message)}</span>`);
  }
}

async function _maintenance(container) {
  if (!confirm('¿Ejecutar ciclo de mantenimiento (triada + MakePlay + repo + refresh)?')) return;
  _setAction(container, 'Encolando ciclo de mantenimiento...');
  try {
    const r = await fetch('/api/software/maintenance/cycle', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ background: true, apply_repo: false }),
    });
    const j = await r.json().catch(() => ({}));
    if (!r.ok || !j.ok) {
      _setAction(container, `<span style="color:var(--accent-red)">Error maintenance: ${_esc(j.error || r.status)}</span>`);
      return;
    }
    const data = _payload(j) || {};
    container.__maintenanceJob = data;
    localStorage.setItem(MAINT_KEY, String(data.job_id || ''));
    _setAction(container, `Mantenimiento iniciado: ${_esc(data.job_id || '--')}`);
    _watchMaintenanceJob(container);
  } catch (e) {
    _setAction(container, `<span style="color:var(--accent-red)">Error maintenance: ${_esc(e.message)}</span>`);
  }
}

async function _resumeJobs(container) {
  const softwareJobId = localStorage.getItem(JOB_KEY);
  if (softwareJobId) {
    container.__softwareJob = { job_id: softwareJobId, status: 'running' };
    _watchSoftwareJob(container);
  } else {
    try {
      const r = await fetch('/api/software/job/latest');
      const j = await r.json().catch(() => ({}));
      const data = _payload(j) || {};
      if (data.found && data.job && data.job.job_id) {
        container.__softwareJob = data.job;
        localStorage.setItem(JOB_KEY, String(data.job.job_id));
        _watchSoftwareJob(container);
      }
    } catch {}
  }

  const maintJobId = localStorage.getItem(MAINT_KEY);
  if (maintJobId) {
    container.__maintenanceJob = { job_id: maintJobId, status: 'running' };
    _watchMaintenanceJob(container);
  }
}

function _watchSoftwareJob(container) {
  if (container.__softwareJobTimer) clearInterval(container.__softwareJobTimer);
  const tick = async () => {
    const job = container.__softwareJob || {};
    const id = String(job.job_id || '').trim();
    if (!id) return;
    try {
      const r = await fetch(`/api/software/job/status/${encodeURIComponent(id)}`);
      const j = await r.json().catch(() => ({}));
      if (!r.ok || !j.ok) return;
      const state = _payload(j) || {};
      container.__softwareJob = state;
      _renderJobProgress(container);
      const status = String(state.status || '').toLowerCase();
      if (status === 'done' || status === 'done_with_errors' || status === 'failed') {
        clearInterval(container.__softwareJobTimer);
        container.__softwareJobTimer = null;
        localStorage.removeItem(JOB_KEY);
        await _refresh(container);
      }
    } catch {}
  };
  tick();
  container.__softwareJobTimer = setInterval(tick, 2500);
}

function _watchMaintenanceJob(container) {
  if (container.__maintenanceTimer) clearInterval(container.__maintenanceTimer);
  const tick = async () => {
    const job = container.__maintenanceJob || {};
    const id = String(job.job_id || '').trim();
    if (!id) return;
    try {
      const r = await fetch(`/api/software/maintenance/status/${encodeURIComponent(id)}`);
      const j = await r.json().catch(() => ({}));
      if (!r.ok || !j.ok) return;
      const state = _payload(j) || {};
      container.__maintenanceJob = state;
      _renderMaintenanceProgress(container);
      const status = String(state.status || '').toLowerCase();
      if (status === 'done' || status === 'done_with_errors' || status === 'failed') {
        clearInterval(container.__maintenanceTimer);
        container.__maintenanceTimer = null;
        localStorage.removeItem(MAINT_KEY);
        await _refresh(container);
      }
    } catch {}
  };
  tick();
  container.__maintenanceTimer = setInterval(tick, 3000);
}

function _renderJobProgress(container) {
  const box = container.querySelector('#sc-progress');
  if (!box) return;
  const job = container.__softwareJob || null;
  if (!job || !job.job_id) {
    box.innerHTML = '';
    return;
  }
  const status = String(job.status || '').toLowerCase();
  const done = Number(job.done || 0);
  const total = Number(job.total || 0);
  const failed = Number(job.failed || 0);
  const pct = _pct(done, total, status);
  const color = status === 'done' && failed === 0 ? 'var(--accent-green)' : ((status === 'failed' || failed > 0) ? 'var(--accent-red)' : 'var(--accent-blue)');
  box.innerHTML = `
    <div class="tools-progress-box">
      <div class="tools-progress-head">
        <span class="chip blue">JOB ${_esc(job.job_id || '--')}</span>
        <span class="chip ${_statusChip(status)}">${_esc((status || 'queued').toUpperCase())}</span>
        <span class="tools-progress-meta">Progreso real: ${done}/${total} · Fallos: ${failed}${job.current_item ? ` · Actual: ${_esc(job.current_item)}` : ''}</span>
      </div>
      <div class="tools-progress-track"><div class="tools-progress-fill" style="width:${pct}%;background:${color}"></div></div>
      <div class="tools-progress-sub">${pct}% completado${failed > 0 ? ' · con errores' : ''}</div>
      ${job.error ? `<div style="margin-top:8px;color:var(--accent-red)">Error: ${_esc(job.error)}</div>` : ''}
    </div>
  `;
}

function _renderMaintenanceProgress(container) {
  const box = container.querySelector('#sc-maint-progress');
  if (!box) return;
  const job = container.__maintenanceJob || null;
  if (!job || !job.job_id) {
    box.innerHTML = '';
    return;
  }
  const status = String(job.status || '').toLowerCase();
  const done = Number(job.done || 0);
  const total = Number(job.total || 0);
  const failed = Number(job.failed || 0);
  const pct = _pct(done, total, status);
  const color = status === 'done' && failed === 0 ? 'var(--accent-green)' : ((status === 'failed' || failed > 0) ? 'var(--accent-red)' : 'var(--accent-blue)');
  box.innerHTML = `
    <div class="tools-progress-box">
      <div class="tools-progress-head">
        <span class="chip blue">MAINT ${_esc(job.job_id || '--')}</span>
        <span class="chip ${_statusChip(status)}">${_esc((status || 'queued').toUpperCase())}</span>
        <span class="tools-progress-meta">Steps: ${done}/${total} · Fallos: ${failed}${job.current_step ? ` · Actual: ${_esc(job.current_step)}` : ''}</span>
      </div>
      <div class="tools-progress-track"><div class="tools-progress-fill" style="width:${pct}%;background:${color}"></div></div>
      <div class="tools-progress-sub">${pct}% completado${failed > 0 ? ' · con errores' : ''}</div>
    </div>
  `;
}

function _setAction(container, html) {
  const el = container.querySelector('#sc-action');
  if (el) el.innerHTML = html || '';
}

window.AtlasModuleSoftwareCenter = { id: 'software-center' };
