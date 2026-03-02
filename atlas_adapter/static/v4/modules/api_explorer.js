/**
 * ATLAS v4.2 — API Explorer
 * Browser de paths OpenAPI con lista clickable + editor/respuesta.
 */
function _esc(s) { const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML; }

export default {
  id: 'api_explorer',
  label: 'API Explorer',
  icon: 'code',
  category: 'tools',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Inicio
          </button>
          <h2>API Explorer</h2>
        </div>
        <div class="module-body" style="padding:0">
          <div style="display:grid;grid-template-columns:320px 1fr;height:calc(100vh - 104px)">

            <!-- Left: path list -->
            <div style="border-right:1px solid var(--border);overflow-y:auto;padding:16px">
              <div style="display:flex;gap:8px;margin-bottom:12px">
                <input id="api-search" class="feedback-box" style="min-height:0;height:36px;resize:none;font-size:12px"
                  placeholder="Filtrar rutas...">
              </div>
              <div class="api-path-list" id="api-path-list">
                <div style="padding:16px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
              </div>
            </div>

            <!-- Right: editor + response -->
            <div style="overflow-y:auto;padding:16px;display:flex;flex-direction:column;gap:12px">
              <div class="section-title" style="margin:0 0 4px">Request</div>
              <div style="display:flex;gap:8px;align-items:center;flex-wrap:wrap">
                <select id="api-method" style="padding:8px 12px;border-radius:var(--radius-md);border:1px solid var(--border);background:var(--bg-elevated);color:var(--text);font-family:var(--font-mono);font-size:12px;font-weight:600">
                  <option>GET</option><option>POST</option><option>PUT</option><option>DELETE</option><option>PATCH</option>
                </select>
                <input id="api-path" class="feedback-box" style="flex:1;min-width:200px;min-height:0;height:38px;resize:none;font-family:var(--font-mono);font-size:12px"
                  value="/health" placeholder="/api/endpoint">
                <button id="api-send" class="action-btn primary">Enviar</button>
              </div>
              <textarea id="api-body" class="feedback-box" style="min-height:80px;max-height:160px;font-family:var(--font-mono);font-size:12px"
                placeholder='{ "key": "value" }'></textarea>

              <div class="section-title" style="margin:0">Respuesta</div>
              <pre class="codebox" id="api-out" style="flex:1;min-height:120px;overflow:auto">--</pre>
            </div>
          </div>
        </div>
      </div>
    `;

    // Load OpenAPI paths
    _loadOpenApi(container);

    // Search filter
    container.querySelector('#api-search')?.addEventListener('input', (e) => {
      const q = e.target.value.toLowerCase();
      const items = container.querySelectorAll('.api-path-item');
      items.forEach(item => {
        item.style.display = item.textContent.toLowerCase().includes(q) ? '' : 'none';
      });
    });

    container.querySelector('#api-send')?.addEventListener('click', () => _send(container));

    // Enter key on path
    container.querySelector('#api-path')?.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') _send(container);
    });
  },

  destroy() {},
};

async function _send(container) {
  const method  = (container.querySelector('#api-method')?.value || 'GET').toUpperCase();
  const path    = (container.querySelector('#api-path')?.value || '').trim() || '/health';
  const bodyTxt = (container.querySelector('#api-body')?.value || '').trim();
  const out     = container.querySelector('#api-out');
  if (out) { out.textContent = '...'; out.style.color = 'var(--text-muted)'; }

  try {
    const opts = { method, headers: {} };
    if (method !== 'GET' && method !== 'HEAD') {
      opts.headers['Content-Type'] = 'application/json';
      opts.body = bodyTxt || '{}';
    }
    const r   = await fetch(path, opts);
    const ct  = r.headers.get('content-type') || '';
    const txt = ct.includes('json')
      ? JSON.stringify(await r.json().catch(() => ({})), null, 2)
      : await r.text();
    if (out) {
      out.textContent = `HTTP ${r.status} ${r.statusText}\n\n${txt}`;
      out.style.color = r.ok ? 'var(--text-secondary)' : 'var(--accent-red)';
    }
    if (!r.ok) window.AtlasToast?.show(`HTTP ${r.status}`, 'error');
  } catch (e) {
    if (out) { out.textContent = `Error: ${e.message}`; out.style.color = 'var(--accent-red)'; }
    window.AtlasToast?.show(e.message, 'error');
  }
}

async function _loadOpenApi(container) {
  const list = container.querySelector('#api-path-list');
  try {
    const r    = await fetch('/openapi.json');
    const data = await r.json();
    const paths = Object.entries(data.paths || {}).sort(([a], [b]) => a.localeCompare(b));

    if (paths.length === 0) {
      list.innerHTML = `<div class="empty-state"><div class="empty-title">Sin paths</div></div>`;
      return;
    }

    list.innerHTML = paths.map(([path, methods]) => {
      const methodKeys = Object.keys(methods).filter(m => ['get','post','put','delete','patch','head'].includes(m));
      const tags = methodKeys.flatMap(m => methods[m]?.tags || []).filter(Boolean);
      const tag  = tags[0] || '';
      return methodKeys.map(method => `
        <div class="api-path-item" data-path="${_esc(path)}" data-method="${method.toUpperCase()}">
          <span class="api-method ${method}">${method.toUpperCase()}</span>
          <span class="api-path-url">${_esc(path)}</span>
          ${tag ? `<span class="api-path-tag">${_esc(tag.slice(0,16))}</span>` : ''}
        </div>`).join('');
    }).join('');

    // Click to fill form
    list.querySelectorAll('.api-path-item').forEach(item => {
      item.addEventListener('click', () => {
        list.querySelectorAll('.api-path-item').forEach(i => i.classList.remove('selected'));
        item.classList.add('selected');
        const pathEl   = container.querySelector('#api-path');
        const methodEl = container.querySelector('#api-method');
        if (pathEl)   pathEl.value = item.dataset.path;
        if (methodEl) methodEl.value = item.dataset.method;
      });
    });
  } catch (e) {
    if (list) list.innerHTML = `<div class="empty-state">
      <div class="empty-title" style="color:var(--accent-red)">Error</div>
      <div class="empty-sub">${_esc(e.message)}</div>
    </div>`;
  }
}

window.AtlasModuleApiExplorer = { id: 'api_explorer' };
