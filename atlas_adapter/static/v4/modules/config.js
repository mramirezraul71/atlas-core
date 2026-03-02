/**
 * ATLAS v4.2 — AI Configuration Module
 * Panel completo: lectura/escritura de config AI, test de conexión,
 * gestión de modelos Ollama y proveedores disponibles.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'config-module';

function _esc(s) { const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML; }

const BRAIN_MODELS = [
  'claude-opus-4-6', 'claude-sonnet-4-6', 'claude-haiku-4-5',
  'gpt-4o', 'gpt-4-turbo', 'gemini-pro',
  'deepseek-chat', 'grok-2', 'ollama/llama3',
];
const CODE_MODELS = [
  'deepseek-coder', 'claude-sonnet-4-6', 'gpt-4o', 'codellama', 'ollama/codellama',
];
const REASONING_MODELS = [
  'claude-opus-4-6', 'o1-preview', 'o1-mini', 'gemini-pro', 'grok-2',
];

export default {
  id: 'config',
  label: 'AI Configuration',
  icon: 'sliders',
  category: 'configuration',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" id="cfg-back">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Dashboard
          </button>
          <h2>Configuración AI</h2>
        </div>

        <div class="module-body">

          <!-- === ESTADO DEL SISTEMA === -->
          <div class="section-title">Estado del Sistema</div>
          <div class="stat-row" style="margin-bottom:20px" id="cfg-health-row">
            <div style="padding:12px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
          </div>

          <!-- === CONFIGURACIÓN DE MODELOS === -->
          <div class="section-title">Modelos AI
            <span id="cfg-save-ts" style="font-size:10px;color:var(--text-muted);font-weight:400;margin-left:8px"></span>
          </div>
          <div style="display:grid;grid-template-columns:repeat(auto-fill,minmax(260px,1fr));gap:12px;margin-bottom:20px" id="cfg-form-grid">
            <div style="padding:12px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
          </div>

          <!-- Acciones de config -->
          <div class="action-bar" style="margin-bottom:20px">
            <button class="action-btn primary" id="btn-cfg-save">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 21H5a2 2 0 01-2-2V5a2 2 0 012-2h11l5 5v11a2 2 0 01-2 2z"/><polyline points="17 21 17 13 7 13 7 21"/><polyline points="7 3 7 8 15 8"/></svg>
              Guardar Config
            </button>
            <button class="action-btn" id="btn-cfg-test">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M9 11l3 3L22 4"/><path d="M21 12v7a2 2 0 01-2 2H5a2 2 0 01-2-2V5a2 2 0 012-2h11"/></svg>
              Test conexión
            </button>
            <button class="action-btn" id="btn-cfg-load">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="23 4 23 10 17 10"/><path d="M20.49 15a9 9 0 11-2.12-9.36L23 10"/></svg>
              Recargar
            </button>
            <span id="cfg-msg" style="font-size:11px;color:var(--text-muted);align-self:center"></span>
          </div>

          <!-- === PROVEEDORES === -->
          <div class="section-title">Proveedores AI</div>
          <div id="cfg-providers" style="margin-bottom:20px">
            <div style="padding:12px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
          </div>

          <!-- === OLLAMA (local) === -->
          <div class="section-title">Modelos Ollama (Local)</div>
          <div class="action-bar" style="margin-bottom:12px">
            <button class="action-btn" id="btn-ollama-list">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="23 4 23 10 17 10"/><path d="M20.49 15a9 9 0 11-2.12-9.36L23 10"/></svg>
              Actualizar lista
            </button>
          </div>
          <div id="cfg-ollama">
            <div style="font-size:12px;color:var(--text-muted);padding:8px 0">Haz clic en "Actualizar lista" para cargar los modelos Ollama disponibles.</div>
          </div>

        </div>
      </div>
    `;

    container.querySelector('#cfg-back')?.addEventListener('click', () => { location.hash = '/'; });
    container.querySelector('#btn-cfg-save')?.addEventListener('click',  () => _saveConfig(container));
    container.querySelector('#btn-cfg-test')?.addEventListener('click',  () => _testConn(container));
    container.querySelector('#btn-cfg-load')?.addEventListener('click',  () => _loadAll(container));
    container.querySelector('#btn-ollama-list')?.addEventListener('click', () => _loadOllama(container));

    _loadAll(container);
    poll(POLL_ID, '/health', 12000, (d) => { if (d) _renderHealth(container, d); });
  },

  destroy() { stop(POLL_ID); },
  badge() { return null; },
};

/* ─── Carga completa ─────────────────────────────────────────────────── */

async function _loadAll(container) {
  const [healthRes, aiRes, modelsRes] = await Promise.all([
    fetch('/health').then(r => r.json()).catch(() => null),
    fetch('/config/ai').then(r => r.json()).catch(() => null),
    fetch('/api/workspace/interpreter/models').then(r => r.json()).catch(() => null),
  ]);
  if (healthRes) _renderHealth(container, healthRes);
  _renderAIForm(container, aiRes);
  if (modelsRes) _renderProviders(container, modelsRes);
}

/* ─── Health row ─────────────────────────────────────────────────────── */

function _renderHealth(container, data) {
  const h = data?.data ?? data ?? {};
  const ok = h.ok ?? true;
  const row = container.querySelector('#cfg-health-row');
  if (!row) return;
  row.innerHTML = `
    <div class="stat-card">
      <div class="stat-card-label">Versión</div>
      <div class="stat-card-value accent" style="font-size:18px">${_esc(h.version || h.checks?.version || '--')}</div>
    </div>
    <div class="stat-card">
      <div class="stat-card-label">Estado</div>
      <div class="stat-card-value ${ok ? 'green' : 'orange'}" style="font-size:18px">${ok ? 'Saludable' : 'Degradado'}</div>
    </div>
    <div class="stat-card">
      <div class="stat-card-label">Score</div>
      <div class="stat-card-value" style="font-size:18px">${_esc(String(h.score ?? '--'))}</div>
    </div>
    <div class="stat-card">
      <div class="stat-card-label">Uptime</div>
      <div class="stat-card-value" style="font-size:14px">${_esc(h.uptime_human || h.uptime || '--')}</div>
    </div>
    <div class="stat-card">
      <div class="stat-card-label">PID</div>
      <div class="stat-card-value" style="font-size:16px">${_esc(String(h.pid ?? '--'))}</div>
    </div>
  `;
}

/* ─── Formulario de configuración AI ────────────────────────────────── */

function _renderAIForm(container, data) {
  const grid = container.querySelector('#cfg-form-grid');
  if (!grid) return;
  const p = data?.data ?? data ?? {};

  const _sel = (id, opts, current) => `
    <select id="${id}" style="width:100%;padding:7px 10px;background:var(--surface-1);border:1px solid var(--border);border-radius:6px;color:var(--text-primary);font-size:13px">
      ${opts.map(o => `<option value="${_esc(o)}" ${o === current ? 'selected' : ''}>${_esc(o)}</option>`).join('')}
      ${current && !opts.includes(current) ? `<option value="${_esc(current)}" selected>${_esc(current)}</option>` : ''}
    </select>`;

  const _inp = (id, placeholder, current, type = 'text') => `
    <input id="${id}" type="${type}" value="${_esc(String(current ?? ''))}" placeholder="${_esc(placeholder)}"
      style="width:100%;padding:7px 10px;background:var(--surface-1);border:1px solid var(--border);border-radius:6px;color:var(--text-primary);font-size:13px;box-sizing:border-box">`;

  grid.innerHTML = `
    <!-- Modelo del cerebro -->
    <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px">
      <div style="font-size:11px;color:var(--text-muted);margin-bottom:6px;font-weight:600;text-transform:uppercase;letter-spacing:.5px">🧠 Modelo Cerebro</div>
      ${_sel('cfg-brain-model', BRAIN_MODELS, p.brain_model || p.model || '')}
    </div>
    <!-- Modelo de código -->
    <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px">
      <div style="font-size:11px;color:var(--text-muted);margin-bottom:6px;font-weight:600;text-transform:uppercase;letter-spacing:.5px">💻 Modelo Código</div>
      ${_sel('cfg-code-model', CODE_MODELS, p.code_model || '')}
    </div>
    <!-- Modelo de razonamiento -->
    <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px">
      <div style="font-size:11px;color:var(--text-muted);margin-bottom:6px;font-weight:600;text-transform:uppercase;letter-spacing:.5px">🤖 Razonamiento</div>
      ${_sel('cfg-reasoning-model', REASONING_MODELS, p.reasoning_model || '')}
    </div>
    <!-- Ventana de contexto -->
    <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px">
      <div style="font-size:11px;color:var(--text-muted);margin-bottom:6px;font-weight:600;text-transform:uppercase;letter-spacing:.5px">📏 Ventana contexto</div>
      <input id="cfg-ctx-window" type="number" min="1024" max="128000" step="1024" value="${p.context_window || 8192}"
        style="width:100%;padding:7px 10px;background:var(--surface-0);border:1px solid var(--border);border-radius:6px;color:var(--text-primary);font-size:13px;box-sizing:border-box">
    </div>
    <!-- Temperatura -->
    <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px">
      <div style="font-size:11px;color:var(--text-muted);margin-bottom:6px;font-weight:600;text-transform:uppercase;letter-spacing:.5px">🌡️ Temperatura</div>
      <input id="cfg-temperature" type="number" min="0" max="2" step="0.05" value="${p.temperature ?? 0.7}"
        style="width:100%;padding:7px 10px;background:var(--surface-0);border:1px solid var(--border);border-radius:6px;color:var(--text-primary);font-size:13px;box-sizing:border-box">
    </div>
    <!-- Modo de contexto -->
    <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px">
      <div style="font-size:11px;color:var(--text-muted);margin-bottom:6px;font-weight:600;text-transform:uppercase;letter-spacing:.5px">💾 Modo contexto</div>
      <select id="cfg-ctx-type" style="width:100%;padding:7px 10px;background:var(--surface-0);border:1px solid var(--border);border-radius:6px;color:var(--text-primary);font-size:13px">
        <option value="short" ${(p.context_type || 'short') === 'short' ? 'selected' : ''}>Corto plazo (últimas 10)</option>
        <option value="long"  ${p.context_type === 'long'  ? 'selected' : ''}>Largo plazo (persistente)</option>
      </select>
    </div>
  `;

  // API Keys section (separate, collapsible)
  const form = container.querySelector('#cfg-form-grid');
  const keysHtml = `
    <details style="grid-column:1/-1;background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px">
      <summary style="cursor:pointer;font-size:11px;color:var(--text-muted);font-weight:600;text-transform:uppercase;letter-spacing:.5px;user-select:none">
        🔑 API Keys (clic para expandir)
      </summary>
      <div style="display:grid;grid-template-columns:repeat(auto-fill,minmax(220px,1fr));gap:8px;margin-top:12px">
        ${[
          ['OPENAI_KEY',    'OpenAI API Key',    p.openai_key    || ''],
          ['ANTHROPIC_KEY', 'Anthropic API Key', p.anthropic_key || ''],
          ['DEEPSEEK_KEY',  'DeepSeek API Key',  p.deepseek_key  || ''],
          ['GEMINI_KEY',    'Gemini API Key',    p.gemini_key    || ''],
          ['XAI_KEY',       'xAI/Grok API Key',  p.xai_key       || ''],
        ].map(([id, label, val]) => `<div>
          <div style="font-size:10px;color:var(--text-muted);margin-bottom:4px">${label}</div>
          <input id="cfg-key-${id.toLowerCase().replace('_key','')}" type="password" value="${_esc(val)}"
            placeholder="${label}..." style="width:100%;padding:6px 10px;background:var(--surface-0);border:1px solid var(--border);border-radius:6px;color:var(--text-primary);font-size:12px;box-sizing:border-box">
        </div>`).join('')}
      </div>
    </details>
  `;
  form.insertAdjacentHTML('beforeend', keysHtml);
}

/* ─── Proveedores (read-only) ────────────────────────────────────────── */

function _renderProviders(container, data) {
  const el = container.querySelector('#cfg-providers');
  if (!el) return;
  const providers = data?.data?.providers || data?.providers || null;
  if (!providers || !Object.keys(providers).length) {
    el.innerHTML = `<div class="empty-state"><div class="empty-title">Sin datos de proveedores</div></div>`;
    return;
  }
  el.innerHTML = `<div class="provider-grid">
    ${Object.entries(providers).map(([prov, info]) => {
      const available = typeof info === 'object' ? (info.available !== false && info.ok !== false) : (info === true || info === 'active');
      const latency   = typeof info === 'object' ? (info.latency_ms || info.latency || '') : '';
      const role      = typeof info === 'object' ? (info.role || info.type || '') : '';
      return `<div class="provider-card ${available ? 'active' : 'down'}">
        ${latency ? `<div class="provider-latency">${_esc(String(latency))}ms</div>` : ''}
        <div class="provider-name">${_esc(prov)}</div>
        ${role ? `<div class="provider-role">${_esc(role)}</div>` : ''}
        <div class="provider-status">
          <div class="provider-dot ${available ? 'ok' : 'down'}"></div>
          ${available ? 'Activo' : 'No disponible'}
        </div>
      </div>`;
    }).join('')}
  </div>`;
}

/* ─── Ollama models ──────────────────────────────────────────────────── */

async function _loadOllama(container) {
  const el = container.querySelector('#cfg-ollama');
  if (!el) return;
  el.innerHTML = `<div style="display:flex;gap:8px;align-items:center"><div class="spinner"></div><span style="font-size:12px;color:var(--text-muted)">Cargando modelos Ollama...</span></div>`;
  try {
    const r = await fetch('/config/ai/ollama-models');
    const d = await r.json().catch(() => ({}));
    if (!r.ok || d.ok === false) throw new Error(d.error || `HTTP ${r.status}`);
    const p = d?.data ?? d;
    const models = p?.models || p?.local_models || (Array.isArray(p) ? p : []);
    if (!models.length) {
      el.innerHTML = `<div class="empty-state"><div class="empty-title">Sin modelos Ollama locales</div><div class="empty-sub">Instala Ollama y descarga modelos para verlos aquí</div></div>`;
      return;
    }
    el.innerHTML = `
      <div style="overflow-x:auto;border-radius:8px;border:1px solid var(--border-subtle)">
        <table style="width:100%;border-collapse:collapse;font-size:12px">
          <thead><tr style="background:var(--surface-1)">
            <th style="text-align:left;padding:8px 12px;color:var(--text-muted);font-size:10px;text-transform:uppercase;border-bottom:1px solid var(--border-subtle)">Modelo</th>
            <th style="text-align:left;padding:8px 12px;color:var(--text-muted);font-size:10px;text-transform:uppercase;border-bottom:1px solid var(--border-subtle)">Tamaño</th>
            <th style="text-align:left;padding:8px 12px;color:var(--text-muted);font-size:10px;text-transform:uppercase;border-bottom:1px solid var(--border-subtle)">Estado</th>
          </tr></thead>
          <tbody>
            ${models.map((m, i) => {
              const name = typeof m === 'string' ? m : (m.name || m.model || m.id || '--');
              const size = typeof m === 'object' ? (m.size ? `${(m.size / 1e9).toFixed(1)} GB` : '--') : '--';
              return `<tr style="${i%2?'background:var(--surface-0)':''}">
                <td style="padding:7px 12px;border-bottom:1px solid var(--border-subtle);color:var(--text-primary);font-family:var(--font-mono);font-size:11px">${_esc(name)}</td>
                <td style="padding:7px 12px;border-bottom:1px solid var(--border-subtle);color:var(--text-secondary)">${size}</td>
                <td style="padding:7px 12px;border-bottom:1px solid var(--border-subtle)"><span class="chip green">disponible</span></td>
              </tr>`;
            }).join('')}
          </tbody>
        </table>
      </div>`;
  } catch (e) {
    el.innerHTML = `<div class="empty-state"><div class="empty-title" style="color:var(--accent-red)">Error</div><div class="empty-sub">${_esc(e.message)}</div></div>`;
  }
}

/* ─── Guardar config ─────────────────────────────────────────────────── */

async function _saveConfig(container) {
  const msg = container.querySelector('#cfg-msg');
  const btnSave = container.querySelector('#btn-cfg-save');
  if (btnSave) btnSave.disabled = true;
  if (msg) msg.textContent = 'Guardando...';

  const get = (id) => container.querySelector(id)?.value || '';

  const payload = {
    brain_model:      get('#cfg-brain-model'),
    code_model:       get('#cfg-code-model'),
    reasoning_model:  get('#cfg-reasoning-model'),
    context_window:   parseInt(get('#cfg-ctx-window') || '8192'),
    temperature:      parseFloat(get('#cfg-temperature') || '0.7'),
    context_type:     get('#cfg-ctx-type'),
  };
  // API keys (only include if filled)
  const keys = {
    openai:    get('#cfg-key-openai'),
    anthropic: get('#cfg-key-anthropic'),
    deepseek:  get('#cfg-key-deepseek'),
    gemini:    get('#cfg-key-gemini'),
    xai:       get('#cfg-key-xai'),
  };
  for (const [k, v] of Object.entries(keys)) { if (v) payload[`${k}_key`] = v; }

  try {
    const r = await fetch('/config/ai', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    const d = await r.json().catch(() => ({}));
    if (!r.ok || d.ok === false) throw new Error(d.error || `HTTP ${r.status}`);
    window.AtlasToast?.show('Configuración guardada', 'success');
    if (msg) msg.textContent = '✓ Guardado';
    const ts = container.querySelector('#cfg-save-ts');
    if (ts) ts.textContent = `Guardado a las ${new Date().toLocaleTimeString()}`;
  } catch (e) {
    window.AtlasToast?.show(e.message, 'error');
    if (msg) msg.textContent = `Error: ${e.message}`;
  } finally {
    if (btnSave) btnSave.disabled = false;
    setTimeout(() => { const m = container.querySelector('#cfg-msg'); if (m) m.textContent = ''; }, 4000);
  }
}

/* ─── Test de conexión ───────────────────────────────────────────────── */

async function _testConn(container) {
  const msg = container.querySelector('#cfg-msg');
  const btn = container.querySelector('#btn-cfg-test');
  if (btn) btn.disabled = true;
  if (msg) msg.textContent = 'Probando conexión...';
  try {
    const r = await fetch('/config/ai/test', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ model: container.querySelector('#cfg-brain-model')?.value || '' }),
    });
    const d = await r.json().catch(() => ({}));
    if (!r.ok || d.ok === false) throw new Error(d.error || `HTTP ${r.status}`);
    const latency = d?.data?.latency_ms || d?.latency_ms || '';
    window.AtlasToast?.show(`Conexión OK${latency ? ` (${latency}ms)` : ''}`, 'success');
    if (msg) msg.textContent = `✓ Conexión OK${latency ? ` — ${latency}ms` : ''}`;
  } catch (e) {
    window.AtlasToast?.show(`Test fallido: ${e.message}`, 'error');
    if (msg) msg.textContent = `✕ ${e.message}`;
  } finally {
    if (btn) btn.disabled = false;
    setTimeout(() => { const m = container.querySelector('#cfg-msg'); if (m) m.textContent = ''; }, 6000);
  }
}

window.AtlasModuleConfig = { id: 'config' };
