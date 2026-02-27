/**
 * ATLAS v4 — Config Module
 * AI model configuration viewer.
 */
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
            Home
          </button>
          <h2>AI Configuration</h2>
        </div>
        <div class="module-body" id="config-body">
          <div class="config-section"><div class="config-section-title">Loading...</div></div>
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

    if (healthRes) {
      html += `<div class="config-section">
        <div class="config-section-title">System</div>
        <div class="config-row"><span class="config-key">Version</span><span class="config-val">${_esc(healthRes.version || '--')}</span></div>
        <div class="config-row"><span class="config-key">Status</span><span class="config-val">${healthRes.ok ? 'Healthy' : 'Degraded'}</span></div>
        <div class="config-row"><span class="config-key">Uptime</span><span class="config-val">${_esc(healthRes.uptime || '--')}</span></div>
      </div>`;
    }

    if (aiRes?.data?.models) {
      html += `<div class="config-section"><div class="config-section-title">Available AI Models</div>`;
      for (const m of aiRes.data.models) {
        const name = typeof m === 'string' ? m : (m.name || m.id || JSON.stringify(m));
        html += `<div class="config-row"><span class="config-key">${_esc(name)}</span><span class="config-val">available</span></div>`;
      }
      html += '</div>';
    }

    if (aiRes?.data?.providers) {
      html += `<div class="config-section"><div class="config-section-title">Providers</div>`;
      for (const [prov, info] of Object.entries(aiRes.data.providers)) {
        const status = typeof info === 'object' ? (info.available ? 'active' : 'inactive') : String(info);
        html += `<div class="config-row"><span class="config-key">${_esc(prov)}</span><span class="config-val">${_esc(status)}</span></div>`;
      }
      html += '</div>';
    }

    body.innerHTML = html || '<div class="config-section"><div class="config-section-title">No configuration data available</div></div>';
  } catch (e) {
    body.innerHTML = `<div class="config-section"><div class="config-section-title">Error loading config</div><p style="color:var(--text-muted)">${e.message}</p></div>`;
  }
}

function _esc(s) { const d = document.createElement('span'); d.textContent = s || ''; return d.innerHTML; }

window.AtlasModuleConfig = { id: 'config' };
