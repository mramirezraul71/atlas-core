/**
 * ATLAS v4.2 — Body Module Detail
 * Vista de detalle dinámica para cualquiera de los 16 módulos del cuerpo.
 * Ruta: #/body-module/{module_name}
 * Routing: app.js lo despacha al prefix 'body-module' con el slug en el hash.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'body-module-detail';

function _esc(s) { const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML; }
function _now() { return new Date().toLocaleTimeString(); }

/**
 * Configuración por módulo:
 *   primary  → endpoint GET principal para métricas
 *   extra    → endpoint GET secundario (opcional)
 *   extraLabel → label para la sección secundaria
 */
const CFG = {
  ans: {
    label: 'ANS',           icon: '❤️',  desc: 'Autonomous Nervous System',
    primary: '/ans/status',
    extra:   '/api/autonomy/status',     extraLabel: 'Autonomía',
  },
  governance: {
    label: 'Governance',    icon: '⚖️',  desc: 'Políticas y control ético',
    primary: '/supervisor/daemon/status',
    extra:   '/supervisor/directives',   extraLabel: 'Directivas activas',
  },
  nervous: {
    label: 'Nervous',       icon: '🔗',  desc: 'Sistema nervioso periférico',
    primary: '/nervous/status',
    extra:   '/nervous/services',        extraLabel: 'Servicios',
  },
  cognitive: {
    label: 'Cognitive',     icon: '🧠',  desc: 'Procesamiento cognitivo',
    primary: '/api/brain/state',
    extra:   '/brain/status',            extraLabel: 'Estado del cerebro',
  },
  quality: {
    label: 'Quality',       icon: '✅',  desc: 'Control de calidad y tutorías',
    primary: '/quality/pots/list',       primaryLabel: 'Tests (POTS)',
    extra:   '/quality/executions/recent', extraLabel: 'Ejecuciones recientes',
  },
  memory_engine: {
    label: 'Memory Engine', icon: '💾',  desc: 'Motor de memoria persistente',
    primary: '/api/memory/stats',        primaryLabel: 'Estadísticas',
    extra:   '/memory/thread/list',      extraLabel: 'Hilos de memoria',
  },
  brain: {
    label: 'Brain',         icon: '🧬',  desc: 'Núcleo de razonamiento',
    primary: '/brain/status',
    extra:   '/api/brain/models',        extraLabel: 'Modelos disponibles',
  },
  hippo: {
    label: 'Hippo',         icon: '🐘',  desc: 'Hipocampo — memoria episódica',
    primary: '/api/memory/stats',        primaryLabel: 'Stats de memoria episódica',
    extra:   '/memory/thread/list',      extraLabel: 'Hilos episódicos',
  },
  learning: {
    label: 'Learning',      icon: '📚',  desc: 'Motor de aprendizaje y patrones',
    primary: '/api/learning/growth-metrics', primaryLabel: 'Métricas de crecimiento',
    extra:   '/api/learning/uncertainty-status', extraLabel: 'Estado de incertidumbre',
  },
  comms: {
    label: 'Comms',         icon: '📡',  desc: 'Hub de comunicaciones',
    primary: '/api/comms/status',
    extra:   '/api/comms/hub/health',    extraLabel: 'Health del Hub',
  },
  world_model: {
    label: 'World Model',   icon: '🌐',  desc: 'Modelo del mundo externo',
    primary: null,
  },
  libro_vida: {
    label: 'Libro de Vida', icon: '📖',  desc: 'Registro biográfico continuo',
    primary: '/memory/thread/list',      primaryLabel: 'Hilos de vida',
  },
  directives: {
    label: 'Directives',    icon: '📋',  desc: 'Directivas y objetivos activos',
    primary: '/supervisor/directives',   primaryLabel: 'Directivas',
    extra:   '/supervisor/policy',       extraLabel: 'Política de supervisor',
  },
  tools: {
    label: 'Tools',         icon: '🔧',  desc: 'Capa de herramientas externas',
    primary: '/tools',                   primaryLabel: 'Herramientas disponibles',
  },
  scheduler: {
    label: 'Scheduler',     icon: '⏱️',  desc: 'Planificador de tareas',
    primary: '/scheduler/jobs',          primaryLabel: 'Trabajos programados',
  },
  tutorias: {
    label: 'Tutorías',      icon: '🎓',  desc: 'Sistema de tutorías y calidad',
    primary: '/quality/pots/list',       primaryLabel: 'POTS activos',
    extra:   '/quality/executions/recent', extraLabel: 'Ejecuciones recientes',
  },
};

export default {
  id: 'body-module',
  label: 'Detalle de Módulo',

  render(container) {
    // Extraer slug del hash: #/body-module/ans → 'ans'
    const slug = (location.hash.split('/').slice(2).join('/') || '').split('?')[0];
    const cfg  = CFG[slug] || { label: slug || 'Módulo', icon: '⚙️', desc: 'Módulo del sistema', primary: null };

    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" id="bm-back">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Módulos del Cuerpo
          </button>
          <div style="display:flex;align-items:center;gap:12px">
            <span style="font-size:32px;line-height:1">${cfg.icon}</span>
            <div>
              <h2 style="margin:0;line-height:1.2">${_esc(cfg.label)}</h2>
              <div style="font-size:12px;color:var(--text-muted);margin-top:2px">${_esc(cfg.desc)}</div>
            </div>
          </div>
          <div style="margin-left:auto;display:flex;gap:8px;align-items:center">
            <span class="live-badge">LIVE</span>
          </div>
        </div>

        <div class="module-body">

          <!-- KPI de estado -->
          <div class="stat-row" style="margin-bottom:20px">
            <div class="stat-card hero">
              <div class="stat-card-label">Estado</div>
              <div class="stat-card-value" id="bm-status-val" style="font-size:20px">--</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Módulo</div>
              <div class="stat-card-value accent" style="font-size:14px">${_esc(slug || '?')}</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Última actualización</div>
              <div class="stat-card-value" id="bm-ts" style="font-size:12px">--</div>
            </div>
            <div class="stat-card" id="bm-error-card" style="display:none">
              <div class="stat-card-label">Error</div>
              <div class="stat-card-value red" id="bm-error-val" style="font-size:11px;word-break:break-all;line-height:1.4">--</div>
            </div>
          </div>

          <!-- Barra de acciones -->
          <div class="action-bar" style="padding-top:0;margin-bottom:20px">
            <button class="action-btn primary" id="bm-refresh">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="23 4 23 10 17 10"/><path d="M20.49 15a9 9 0 11-2.12-9.36L23 10"/></svg>
              Actualizar
            </button>
            <button class="action-btn" id="bm-reconnect">
              ↻ Reconectar módulo
            </button>
            <span id="bm-msg" style="font-size:11px;color:var(--text-muted);align-self:center;margin-left:4px"></span>
          </div>

          <!-- Datos primarios -->
          ${cfg.primary ? `
            <div class="section-title">${_esc(cfg.primaryLabel || 'Datos del Módulo')}</div>
            <div id="bm-primary" style="margin-bottom:24px">
              <div style="padding:24px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
            </div>
          ` : `
            <div class="section-title">Estado del Módulo</div>
            <div class="empty-state" style="margin-bottom:24px">
              <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5"><circle cx="12" cy="12" r="10"/><line x1="12" y1="8" x2="12" y2="12"/><line x1="12" y1="16" x2="12.01" y2="16"/></svg>
              <div class="empty-title">Sin endpoint de telemetría</div>
              <div class="empty-sub">Este módulo no expone datos de métricas propias.<br>Su estado se refleja en la vista de Módulos del Cuerpo.</div>
            </div>
          `}

          <!-- Datos secundarios -->
          ${cfg.extra ? `
            <div class="section-title">${_esc(cfg.extraLabel || 'Datos Secundarios')}</div>
            <div id="bm-extra" style="margin-bottom:24px">
              <div style="padding:24px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
            </div>
          ` : ''}

        </div>
      </div>
    `;

    // Eventos
    container.querySelector('#bm-back').addEventListener('click', () => { location.hash = '/modules'; });
    container.querySelector('#bm-refresh').addEventListener('click', () => _loadAll(container, slug, cfg));
    container.querySelector('#bm-reconnect').addEventListener('click', () => _reconnect(container, slug, cfg));

    // Carga inicial
    _loadAll(container, slug, cfg);

    // Poll del status global cada 15s
    poll(POLL_ID, '/modules/check-all', 15000, (data) => {
      if (data) _updateStatus(container, slug, data);
    });
  },

  destroy() { stop(POLL_ID); },
};

/* ─── Carga de datos ─────────────────────────────────────────────────── */

async function _loadAll(container, slug, cfg) {
  const btn = container.querySelector('#bm-refresh');
  if (btn) btn.disabled = true;

  try {
    const promises = [
      fetch('/modules/check-all').then(r => r.json()).catch(() => null),
      cfg.primary ? fetch(cfg.primary).then(r => r.json()).catch(e => ({ _fetchError: e.message })) : Promise.resolve(null),
      cfg.extra   ? fetch(cfg.extra).then(r => r.json()).catch(e => ({ _fetchError: e.message }))   : Promise.resolve(null),
    ];

    const [statusData, primaryData, extraData] = await Promise.all(promises);

    if (statusData) _updateStatus(container, slug, statusData);
    if (primaryData) _renderSection(container, '#bm-primary', primaryData);
    if (extraData)   _renderSection(container, '#bm-extra',   extraData);

    const ts = container.querySelector('#bm-ts');
    if (ts) ts.textContent = _now();
  } finally {
    if (btn) btn.disabled = false;
  }
}

/* ─── Actualizar estado desde check-all ─────────────────────────────── */

function _updateStatus(container, slug, data) {
  const p = data?.data ?? data;
  const modules = Array.isArray(p) ? p : (p?.modules || []);
  const mod = modules.find(m => m.name === slug);

  const statusEl = container.querySelector('#bm-status-val');
  const errorCard = container.querySelector('#bm-error-card');
  const errorVal  = container.querySelector('#bm-error-val');

  if (!mod) {
    if (statusEl) { statusEl.textContent = 'Desconocido'; statusEl.className = 'stat-card-value'; }
    return;
  }

  const ok = mod.status === 'connected';
  if (statusEl) {
    statusEl.textContent = ok ? 'Conectado' : 'Error';
    statusEl.className = `stat-card-value ${ok ? 'green' : 'red'}`;
  }

  if (errorCard) errorCard.style.display = mod.error ? '' : 'none';
  if (errorVal && mod.error) errorVal.textContent = mod.error;
}

/* ─── Renderizado de sección de datos ───────────────────────────────── */

function _renderSection(container, selector, data) {
  const el = container.querySelector(selector);
  if (!el) return;

  // Error de fetch
  if (data && data._fetchError) {
    el.innerHTML = `<div class="empty-state">
      <div class="empty-title" style="color:var(--accent-red)">Error al cargar datos</div>
      <div class="empty-sub">${_esc(data._fetchError)}</div>
    </div>`;
    return;
  }

  // Extraer payload (maneja { ok, data, ms } y respuestas planas)
  const raw = (data?.data !== undefined && data?.data !== null) ? data.data : data;

  if (raw === null || raw === undefined) {
    el.innerHTML = `<div class="empty-state"><div class="empty-title">Sin datos</div></div>`;
    return;
  }

  if (Array.isArray(raw)) {
    el.innerHTML = raw.length === 0
      ? `<div class="empty-state"><div class="empty-title">Lista vacía</div></div>`
      : _renderArr(raw);
    return;
  }

  if (typeof raw === 'object') {
    el.innerHTML = _renderObj(raw);
    return;
  }

  // Escalar
  el.innerHTML = `<div class="codebox" style="font-size:12px">${_esc(String(raw))}</div>`;
}

/* ─── Renderizado de objetos ─────────────────────────────────────────── */

function _renderObj(obj) {
  const entries = Object.entries(obj);
  if (!entries.length) return `<div class="empty-state"><div class="empty-title">Sin datos</div></div>`;

  const simple  = entries.filter(([, v]) => typeof v !== 'object' || v === null);
  const complex = entries.filter(([, v]) => typeof v === 'object' && v !== null);

  let html = '';

  // Stat-cards para valores simples
  if (simple.length) {
    html += `<div class="stat-row" style="flex-wrap:wrap;gap:10px;margin-bottom:${complex.length ? '20px' : '0'}">`;
    simple.forEach(([k, v]) => {
      const display = v === null ? 'null' : v === true ? '✓ true' : v === false ? '✗ false' : String(v);
      const isOk  = v === true || /^(ok|active|connected|running|true)$/i.test(String(v));
      const isErr = v === false || /^(error|false|down|failed)$/i.test(String(v));
      const color = isOk ? 'green' : isErr ? 'red' : '';
      const fsize = display.length > 14 ? '11' : display.length > 8 ? '14' : '18';
      html += `
        <div class="stat-card" style="min-width:130px;flex:1 1 130px">
          <div class="stat-card-label">${_esc(k.replace(/_/g, ' '))}</div>
          <div class="stat-card-value ${color}" style="font-size:${fsize}px;word-break:break-all">${_esc(display)}</div>
        </div>`;
    });
    html += '</div>';
  }

  // Secciones anidadas para valores complejos
  complex.forEach(([k, v]) => {
    html += `<div class="section-title" style="font-size:11px;margin-top:16px;margin-bottom:8px">${_esc(k.replace(/_/g, ' ').toUpperCase())}</div>`;
    if (Array.isArray(v)) {
      html += v.length
        ? `<div style="margin-bottom:4px">${_renderArr(v)}</div>`
        : `<div style="font-size:12px;color:var(--text-muted);padding:8px 0">Lista vacía</div>`;
    } else {
      // Objeto anidado: renderizar como mini-cards o JSON
      const subEntries = Object.entries(v);
      const subSimple = subEntries.filter(([, sv]) => typeof sv !== 'object');
      if (subSimple.length > 0 && subSimple.length <= 8) {
        html += `<div class="stat-row" style="flex-wrap:wrap;gap:8px;margin-bottom:4px">`;
        subSimple.forEach(([sk, sv]) => {
          const sd = String(sv ?? '--');
          html += `<div class="stat-card" style="min-width:110px;flex:1 1 110px">
            <div class="stat-card-label">${_esc(sk.replace(/_/g, ' '))}</div>
            <div class="stat-card-value" style="font-size:13px">${_esc(sd)}</div>
          </div>`;
        });
        html += '</div>';
      } else {
        html += `<div class="codebox" style="font-size:11px;max-height:200px;overflow-y:auto;margin-bottom:4px">${_esc(JSON.stringify(v, null, 2))}</div>`;
      }
    }
  });

  return html;
}

/* ─── Renderizado de arrays ──────────────────────────────────────────── */

function _renderArr(arr) {
  // Arrays de strings/numbers → chips
  if (typeof arr[0] !== 'object' || arr[0] === null) {
    return `<div style="display:flex;flex-wrap:wrap;gap:6px;padding:4px 0">
      ${arr.slice(0, 50).map(v => `<span class="chip">${_esc(String(v))}</span>`).join('')}
      ${arr.length > 50 ? `<span class="chip" style="color:var(--text-muted)">+${arr.length - 50} más</span>` : ''}
    </div>`;
  }

  // Arrays de objetos → tabla compacta
  const sample = arr[0] || {};
  const keys = Object.keys(sample).filter(k => typeof sample[k] !== 'object').slice(0, 7);
  if (!keys.length) {
    return `<div class="codebox" style="font-size:11px;max-height:300px;overflow-y:auto">${_esc(JSON.stringify(arr.slice(0, 10), null, 2))}</div>`;
  }

  return `
    <div style="overflow-x:auto;border-radius:8px;border:1px solid var(--border-subtle)">
      <table style="width:100%;border-collapse:collapse;font-size:12px">
        <thead>
          <tr style="background:var(--surface-1)">
            ${keys.map(k => `<th style="text-align:left;padding:8px 12px;color:var(--text-muted);font-weight:600;font-size:10px;text-transform:uppercase;white-space:nowrap;border-bottom:1px solid var(--border-subtle)">${_esc(k)}</th>`).join('')}
          </tr>
        </thead>
        <tbody>
          ${arr.slice(0, 40).map((row, i) => `
            <tr style="${i % 2 ? 'background:var(--surface-0)' : ''}">
              ${keys.map(k => {
                const v = String(row[k] ?? '--');
                const isOk  = /^(true|connected|active|ok|running)$/i.test(v);
                const isErr = /^(false|error|down|failed)$/i.test(v);
                const color = isOk ? 'var(--accent-green)' : isErr ? 'var(--accent-red)' : 'var(--text-secondary)';
                return `<td style="padding:7px 12px;border-bottom:1px solid var(--border-subtle);color:${color};max-width:220px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap" title="${_esc(v)}">${_esc(v)}</td>`;
              }).join('')}
            </tr>`).join('')}
        </tbody>
      </table>
      ${arr.length > 40 ? `<div style="padding:8px 12px;font-size:11px;color:var(--text-muted);border-top:1px solid var(--border-subtle)">Mostrando 40 de ${arr.length} entradas</div>` : ''}
    </div>
  `;
}

/* ─── Reconectar módulo ──────────────────────────────────────────────── */

async function _reconnect(container, slug, cfg) {
  const btn = container.querySelector('#bm-reconnect');
  const msg = container.querySelector('#bm-msg');
  if (btn) btn.disabled = true;
  if (msg) msg.textContent = 'Reconectando…';

  try {
    const r = await fetch(`/modules/reconnect/${encodeURIComponent(slug)}`, { method: 'POST' });
    const d = await r.json().catch(() => ({}));
    const ok = d.ok !== false && r.ok;
    if (msg) msg.textContent = ok ? '✓ Reconectado' : `Error: ${_esc(d.error || r.status)}`;
    window.AtlasToast?.show(
      ok ? `${cfg.label || slug}: reconectado` : (d.error || 'Error al reconectar'),
      ok ? 'success' : 'error'
    );
    if (ok) setTimeout(() => _loadAll(container, slug, cfg), 800);
  } catch (e) {
    if (msg) msg.textContent = `Error: ${e.message}`;
    window.AtlasToast?.show(e.message, 'error');
  } finally {
    if (btn) btn.disabled = false;
    // Limpiar mensaje después de 4s
    setTimeout(() => { const m = container.querySelector('#bm-msg'); if (m) m.textContent = ''; }, 4000);
  }
}

window.AtlasBodyModule = { id: 'body-module' };
