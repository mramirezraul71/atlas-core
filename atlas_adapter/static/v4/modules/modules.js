/**
 * ATLAS v4.2 — Body Modules Status
 * Vista de los 16 módulos del cuerpo de Atlas: estado conectado/error,
 * reconexión individual y verificación global.
 * Endpoint: GET /modules/check-all  POST /modules/reconnect/{id}
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'modules-module';

function _esc(s) { const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML; }

// Metadatos visuales por módulo
const MODULE_META = {
  ans:          { label: 'ANS',          icon: '❤️',  desc: 'Autonomous Nervous System' },
  governance:   { label: 'Governance',   icon: '⚖️',  desc: 'Políticas y control ético' },
  nervous:      { label: 'Nervous',      icon: '🔗',  desc: 'Sistema nervioso periférico' },
  cognitive:    { label: 'Cognitive',    icon: '🧠',  desc: 'Procesamiento cognitivo' },
  quality:      { label: 'Quality',      icon: '✅',  desc: 'Control de calidad y tutorías' },
  memory_engine:{ label: 'Memory Engine',icon: '💾',  desc: 'Motor de memoria persistente' },
  brain:        { label: 'Brain',        icon: '🧬',  desc: 'Núcleo de razonamiento' },
  hippo:        { label: 'Hippo',        icon: '🐘',  desc: 'Hipocampo — memoria episódica' },
  learning:     { label: 'Learning',     icon: '📚',  desc: 'Motor de aprendizaje' },
  comms:        { label: 'Comms',        icon: '📡',  desc: 'Hub de comunicaciones' },
  world_model:  { label: 'World Model',  icon: '🌐',  desc: 'Modelo del mundo externo' },
  libro_vida:   { label: 'Libro de Vida',icon: '📖',  desc: 'Registro biográfico continuo' },
  directives:   { label: 'Directives',   icon: '📋',  desc: 'Directivas y objetivos' },
  tools:        { label: 'Tools',        icon: '🔧',  desc: 'Capa de herramientas externas' },
  scheduler:    { label: 'Scheduler',    icon: '⏱️',  desc: 'Planificador de tareas' },
  tutorias:     { label: 'Tutorías',     icon: '🎓',  desc: 'Calidad y tutorías' },
};

export default {
  id: 'modules',
  label: 'Módulos del Cuerpo',
  icon: 'cpu',
  category: 'monitoring',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Dashboard
          </button>
          <h2>Módulos del Cuerpo</h2>
          <div style="margin-left:auto"><span class="live-badge">LIVE</span></div>
        </div>
        <div class="module-body">

          <!-- KPIs -->
          <div class="stat-row" style="margin-bottom:20px">
            <div class="stat-card hero">
              <div class="stat-card-label">Conectados</div>
              <div class="stat-card-value green" id="mod-connected">--</div>
              <div class="stat-card-sub" id="mod-total">de -- módulos</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Con error</div>
              <div class="stat-card-value red" id="mod-errors">--</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Salud del cuerpo</div>
              <div class="stat-card-value" id="mod-health" style="font-size:22px">--%</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Última verificación</div>
              <div class="stat-card-value" id="mod-ts" style="font-size:12px">--</div>
            </div>
          </div>

          <!-- Acciones -->
          <div class="action-bar" style="padding-top:0;margin-bottom:16px">
            <button class="action-btn primary" id="btn-check-all">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="23 4 23 10 17 10"/><path d="M20.49 15a9 9 0 11-2.12-9.36L23 10"/></svg>
              Verificar todos
            </button>
            <div id="mod-msg" style="font-size:11px;color:var(--text-muted);align-self:center"></div>
          </div>

          <!-- Grid de módulos -->
          <div class="section-title">
            Estado por Módulo
            <span class="count" id="mod-grid-count"></span>
          </div>
          <div id="mod-grid" style="display:grid;grid-template-columns:repeat(auto-fill,minmax(200px,1fr));gap:10px">
            <div style="grid-column:1/-1;padding:20px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
          </div>

        </div>
      </div>
    `;

    container.querySelector('#btn-check-all')?.addEventListener('click', () => _checkAll(container, true));
    _fetchAndRender(container);
    poll(POLL_ID, '/modules/check-all', 15000, (data) => {
      if (data) _render(container, data);
    });
  },

  destroy() { stop(POLL_ID); },
  badge() { return null; },
};

async function _fetchAndRender(container) {
  try {
    const r = await fetch('/modules/check-all');
    const data = await r.json();
    _render(container, data);
  } catch (e) {
    const grid = container.querySelector('#mod-grid');
    if (grid) grid.innerHTML = `<div class="empty-state" style="grid-column:1/-1">
      <div class="empty-title" style="color:var(--accent-red)">Error al cargar módulos</div>
      <div class="empty-sub">${_esc(e.message)}</div>
    </div>`;
  }
}

async function _checkAll(container, manual = false) {
  const btn = container.querySelector('#btn-check-all');
  const msg = container.querySelector('#mod-msg');
  if (btn) btn.disabled = true;
  if (msg) msg.textContent = 'Verificando…';
  try {
    const r = await fetch('/modules/check-all');
    const data = await r.json();
    _render(container, data);
    const conn = data.connected ?? 0;
    const tot  = data.total ?? 0;
    if (msg) msg.textContent = `Verificados: ${conn}/${tot} OK`;
    if (manual) window.AtlasToast?.show(`Módulos: ${conn}/${tot} conectados`, conn === tot ? 'success' : 'info');
  } catch (e) {
    if (msg) msg.textContent = `Error: ${e.message}`;
    window.AtlasToast?.show(e.message, 'error');
  } finally {
    if (btn) btn.disabled = false;
  }
}

function _render(container, data) {
  const p = data?.data ?? data;
  const modules  = p?.modules || [];
  const connected = p?.connected ?? modules.filter(m => m.status === 'connected').length;
  const total     = p?.total    ?? modules.length;
  const errors    = total - connected;
  const pct       = total > 0 ? Math.round((connected / total) * 100) : 0;

  const txt = (id, v) => { const e = container.querySelector(id); if (e) e.textContent = v; };
  txt('#mod-connected', connected);
  txt('#mod-total',     `de ${total} módulos`);
  txt('#mod-errors',    errors);
  txt('#mod-health',    `${pct}%`);
  txt('#mod-ts',        new Date().toLocaleTimeString());
  txt('#mod-grid-count', `${total} módulos`);

  const healthEl = container.querySelector('#mod-health');
  if (healthEl) healthEl.style.color = pct >= 80 ? 'var(--accent-green)' : pct >= 50 ? 'var(--accent-orange)' : 'var(--accent-red)';

  const grid = container.querySelector('#mod-grid');
  if (!grid) return;

  if (modules.length === 0) {
    grid.innerHTML = `<div class="empty-state" style="grid-column:1/-1">
      <div class="empty-title">Sin datos de módulos</div>
      <div class="empty-sub">El endpoint no devolvió módulos</div>
    </div>`;
    return;
  }

  grid.innerHTML = modules.map(m => {
    const ok   = m.status === 'connected';
    const meta = MODULE_META[m.name] || { label: m.name, icon: '⚙️', desc: m.module || '' };
    const errTip = m.error ? `title="${_esc(m.error)}"` : '';
    return `<div class="provider-card ${ok ? 'active' : 'down'}" ${errTip}>
      <div style="font-size:22px;margin-bottom:6px">${meta.icon}</div>
      <div class="provider-name">${_esc(meta.label)}</div>
      <div class="provider-role">${_esc(meta.desc)}</div>
      <div class="provider-status" style="margin-top:8px">
        <div class="provider-dot ${ok ? 'ok' : 'down'}"></div>
        ${ok ? 'Conectado' : 'Error'}
      </div>
      ${!ok ? `<button class="action-btn" style="margin-top:10px;width:100%;font-size:11px;padding:6px 8px"
        onclick="window._reconnectModule('${_esc(m.name)}', this)">
        ↻ Reconectar
      </button>` : ''}
      ${m.error ? `<div style="font-size:10px;color:var(--accent-red);margin-top:6px;word-break:break-word;max-height:40px;overflow:hidden">${_esc(m.error.slice(0, 80))}</div>` : ''}
    </div>`;
  }).join('');

  window._reconnectModule = async (moduleId, btn) => {
    if (btn) btn.disabled = true;
    try {
      const r = await fetch(`/modules/reconnect/${encodeURIComponent(moduleId)}`, { method: 'POST' });
      const d = await r.json().catch(() => ({}));
      window.AtlasToast?.show(d.ok !== false ? `Reconectado: ${moduleId}` : (d.error || 'Error'), d.ok !== false ? 'success' : 'error');
      setTimeout(() => _checkAll(container), 800);
    } catch (e) {
      window.AtlasToast?.show(e.message, 'error');
      if (btn) btn.disabled = false;
    }
  };
}

window.AtlasModuleModules = { id: 'modules' };
