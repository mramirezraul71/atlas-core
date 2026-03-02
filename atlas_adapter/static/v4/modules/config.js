/**
 * ATLAS v4.2 — Config Module
 * Vista de configuración IA: modelos disponibles, proveedores, sistema.
 */
function _esc(s) { const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML; }

export default {
  id: 'config',
  label: 'AI Configuration',
  icon: 'sliders',
  category: 'configuration',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Inicio
          </button>
          <h2>Configuración AI</h2>
        </div>
        <div class="module-body" id="config-body">
          <div style="padding:40px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
        </div>
      </div>
    `;
    _fetchConfig(container);
  },

  destroy() {},
  badge() { return null; },
};

async function _fetchConfig(container) {
  const body = container.querySelector('#config-body');
  try {
    const [aiRes, healthRes] = await Promise.all([
      fetch('/api/workspace/interpreter/models').then(r => r.json()).catch(() => null),
      fetch('/health').then(r => r.json()).catch(() => null),
    ]);

    let html = '';

    // System info
    if (healthRes) {
      const h = healthRes?.data ?? healthRes;
      html += `<div class="stat-row" style="margin-bottom:20px">
        <div class="stat-card">
          <div class="stat-card-label">Versión</div>
          <div class="stat-card-value accent" style="font-size:18px">${_esc(h.version || healthRes.version || '--')}</div>
        </div>
        <div class="stat-card">
          <div class="stat-card-label">Estado</div>
          <div class="stat-card-value ${(h.ok ?? healthRes.ok) ? 'green' : 'orange'}" style="font-size:18px">${(h.ok ?? healthRes.ok) ? 'Saludable' : 'Degradado'}</div>
        </div>
        <div class="stat-card">
          <div class="stat-card-label">Score</div>
          <div class="stat-card-value" style="font-size:18px">${_esc(String(h.score ?? healthRes.score ?? '--'))}</div>
        </div>
        <div class="stat-card">
          <div class="stat-card-label">Uptime</div>
          <div class="stat-card-value" style="font-size:14px">${_esc(h.uptime_human || h.uptime || healthRes.uptime || '--')}</div>
        </div>
      </div>`;
    }

    // Providers grid
    const providers = aiRes?.data?.providers || aiRes?.providers || null;
    if (providers && typeof providers === 'object') {
      html += `<div class="section-title">Proveedores AI</div>
        <div class="provider-grid">`;
      for (const [prov, info] of Object.entries(providers)) {
        const available = typeof info === 'object' ? (info.available !== false && info.ok !== false) : (info === true || info === 'active' || info === 'ok');
        const latency  = typeof info === 'object' ? (info.latency_ms || info.latency || '') : '';
        const role     = typeof info === 'object' ? (info.role || info.type || '') : '';
        const dotCls   = available ? 'ok' : 'down';
        const cardCls  = available ? 'active' : 'down';
        html += `<div class="provider-card ${cardCls}">
          ${latency ? `<div class="provider-latency">${_esc(String(latency))}ms</div>` : ''}
          <div class="provider-name">${_esc(prov)}</div>
          ${role ? `<div class="provider-role">${_esc(role)}</div>` : ''}
          <div class="provider-status">
            <div class="provider-dot ${dotCls}"></div>
            ${available ? 'Activo' : 'No disponible'}
          </div>
        </div>`;
      }
      html += `</div>`;
    }

    // Models list
    const models = aiRes?.data?.models || aiRes?.models || null;
    if (Array.isArray(models) && models.length > 0) {
      html += `<div class="section-title" style="margin-top:20px">Modelos Disponibles <span class="count">${models.length}</span></div>
        <table class="data-tbl">
          <thead><tr><th>Modelo</th><th>Estado</th></tr></thead>
          <tbody>`;
      for (const m of models) {
        const name = typeof m === 'string' ? m : (m.name || m.id || JSON.stringify(m));
        html += `<tr><td class="mono">${_esc(name)}</td><td><span class="chip green">disponible</span></td></tr>`;
      }
      html += `</tbody></table>`;
    }

    if (!html) {
      html = `<div class="empty-state">
        <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5"><circle cx="12" cy="12" r="3"/><path d="M19.07 4.93a10 10 0 010 14.14M4.93 4.93a10 10 0 000 14.14"/></svg>
        <div class="empty-title">Sin datos de configuración</div>
        <div class="empty-sub">Los endpoints de configuración no están disponibles</div>
      </div>`;
    }

    body.innerHTML = html;
  } catch (e) {
    body.innerHTML = `<div class="empty-state">
      <div class="empty-title" style="color:var(--accent-red)">Error</div>
      <div class="empty-sub">${_esc(e.message)}</div>
    </div>`;
  }
}

window.AtlasModuleConfig = { id: 'config' };
