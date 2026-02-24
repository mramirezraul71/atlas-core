/**
 * ATLAS v4 — API Explorer
 * Safety net: browse OpenAPI paths and call GET/POST with JSON.
 */
export default {
  id: 'api',
  label: 'API Explorer',
  icon: 'code',
  category: 'tools',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Home
          </button>
          <h2>API Explorer</h2>
        </div>
        <div class="module-body">
          <div class="config-section">
            <div class="config-section-title">Request</div>
            <div style="display:flex;gap:10px;flex-wrap:wrap;align-items:center">
              <select id="api-method" class="chip-btn">
                <option>GET</option>
                <option>POST</option>
              </select>
              <input id="api-path" class="search-bar" style="flex:1;min-width:280px;margin:0" placeholder="/health">
              <button id="api-send" class="chip-btn">Send</button>
            </div>
            <div style="margin-top:10px">
              <textarea id="api-body" class="assistant-input" style="min-height:120px;max-height:240px" placeholder="{ } (JSON body for POST)"></textarea>
            </div>
            <div style="margin-top:10px;color:var(--text-muted);font-size:12px">Tip: carga OpenAPI abajo y pega una ruta.</div>
          </div>

          <div class="config-section">
            <div class="config-section-title">Response</div>
            <pre class="codebox" id="api-out">--</pre>
          </div>

          <div class="config-section">
            <div class="config-section-title">OpenAPI paths</div>
            <pre class="codebox" id="api-paths">Loading...</pre>
          </div>
        </div>
      </div>
    `;

    container.querySelector('#api-send')?.addEventListener('click', () => _send(container));
    _loadOpenApi(container);
  },

  destroy() {},
};

async function _send(container) {
  const method = (container.querySelector('#api-method')?.value || 'GET').toUpperCase();
  const path = (container.querySelector('#api-path')?.value || '').trim() || '/health';
  const bodyText = container.querySelector('#api-body')?.value || '';
  const out = container.querySelector('#api-out');
  if (out) out.textContent = '...';
  try {
    const opts = { method, headers: {} };
    if (method !== 'GET') {
      opts.headers['Content-Type'] = 'application/json';
      opts.body = bodyText && bodyText.trim() ? bodyText : '{}';
    }
    const res = await fetch(path, opts);
    const ct = res.headers.get('content-type') || '';
    const text = ct.includes('json') ? JSON.stringify(await res.json().catch(() => ({})), null, 2) : await res.text();
    if (out) out.textContent = `HTTP ${res.status}\n\n` + text;
    if (!res.ok) window.AtlasToast?.show(`HTTP ${res.status}`, 'error');
  } catch (e) {
    if (out) out.textContent = `Error: ${e.message}`;
    window.AtlasToast?.show(`Error: ${e.message}`, 'error');
  }
}

async function _loadOpenApi(container) {
  const box = container.querySelector('#api-paths');
  try {
    const res = await fetch('/openapi.json');
    const data = await res.json();
    const paths = Object.keys(data.paths || {}).sort();
    if (box) box.textContent = paths.join('\n');
  } catch (e) {
    if (box) box.textContent = `Error: ${e.message}`;
  }
}

window.AtlasModuleApiExplorer = { id: 'api' };

