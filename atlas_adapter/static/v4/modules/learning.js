/**
 * ATLAS v4.2 — Learning Module
 * Muestra patrones aprendidos, ciclos de aprendizaje y métricas.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'learning-module';

function _esc(s) { const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML; }

function _shortTime(ts) {
  if (!ts) return '--';
  try { return new Date(ts).toLocaleTimeString(); } catch { return String(ts); }
}

const ENDPOINTS = [
  { id: 'status',   url: '/api/learning/growth-metrics',  label: 'Métricas de Crecimiento' },
  { id: 'patterns', url: '/api/learning/knowledge-base',  label: 'Base de Conocimiento' },
  { id: 'cycles',   url: null,                            label: 'Historial de Ciclos' },
];

export default {
  id: 'learning',
  label: 'Aprendizaje',
  icon: 'trending-up',
  category: 'monitoring',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Inicio
          </button>
          <h2>Motor de Aprendizaje</h2>
          <div style="margin-left:auto"><span class="live-badge">LIVE</span></div>
        </div>
        <div class="module-body">

          <div class="stat-row" style="margin-bottom:16px">
            <div class="stat-card hero">
              <div class="stat-card-label">Estado</div>
              <div class="stat-card-value" id="l-state">--</div>
              <div class="stat-card-sub" id="l-mode"></div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Patrones</div>
              <div class="stat-card-value accent" id="l-patterns">--</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Ciclos</div>
              <div class="stat-card-value" id="l-cycles">--</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Último ciclo</div>
              <div class="stat-card-value" id="l-last" style="font-size:13px">--</div>
            </div>
          </div>

          <div class="section-title">
            Patrones Aprendidos
            <span class="count" id="l-pcount"></span>
          </div>
          <div id="l-pattern-list">
            <div style="padding:20px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
          </div>

          <div class="section-title" style="margin-top:20px">
            Historial de Ciclos
            <span class="count" id="l-ccount"></span>
          </div>
          <div class="timeline" id="l-cycles-list"></div>

        </div>
      </div>
    `;

    _refresh(container);
    poll(POLL_ID, ENDPOINTS[0].url, 10000, () => _refresh(container));
  },

  destroy() { stop(POLL_ID); },
};

async function _refresh(container) {
  const [status, patterns] = await Promise.all(
    ENDPOINTS.filter(e => e.url).map(e => fetch(e.url).then(r => r.json()).catch(() => null))
  );
  _renderStatus(container, status);
  _renderPatterns(container, patterns);
  _renderCycles(container, null);
}

function _renderStatus(container, data) {
  if (!data) return;
  const p = data?.data ?? data;
  // growth-metrics format: {knowledge_base:{concepts,skills,rules}, episodic_memory:{total_episodes,...}, learning:{...}}
  const kb = p?.knowledge_base || {};
  const ep = p?.episodic_memory || {};
  const active = (ep.total_episodes ?? 0) > 0 || (kb.concepts ?? 0) > 0;
  const txt = (id, v) => { const e = container.querySelector(id); if (e) e.textContent = v; };
  txt('#l-state', active ? 'Activo' : 'Inicializando');
  const stEl = container.querySelector('#l-state');
  if (stEl) stEl.style.color = active ? 'var(--accent-green)' : 'var(--text-muted)';
  const totalPatterns = (kb.concepts ?? 0) + (kb.skills ?? 0) + (kb.rules ?? 0);
  txt('#l-patterns', totalPatterns || '--');
  txt('#l-mode', ep.overall_success_rate !== undefined ? `Éxito: ${Math.round(ep.overall_success_rate * 100)}%` : '');
  txt('#l-cycles', ep.total_episodes ?? '--');
  txt('#l-last', ep.top_task_types?.[0]?.task ? `Tarea: ${ep.top_task_types[0].task}` : '--');
}

function _renderPatterns(container, data) {
  const el = container.querySelector('#l-pattern-list');
  if (!el) return;
  const p = data?.data ?? data;
  // knowledge-base format: {concepts: {name: {...}}, skills: {name: {...}}, rules: {name: {...}}, total_concepts, total_skills}
  const pcountEl = container.querySelector('#l-pcount');

  if (!p || (!p.concepts && !p.skills)) {
    el.innerHTML = `<div class="empty-state">
      <svg width="28" height="28" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5"><polyline points="22 12 18 12 15 21 9 3 6 12 2 12"/></svg>
      <div class="empty-title">Sin conocimiento registrado</div>
      <div class="empty-sub">La base de conocimiento está vacía</div>
    </div>`;
    return;
  }

  // Flatten concepts + skills + rules into rows
  const rows = [];
  for (const [name, v] of Object.entries(p.concepts || {})) rows.push({ cat: 'concepto', name, def: v?.definition || v?.type || '' });
  for (const [name, v] of Object.entries(p.skills || {})) rows.push({ cat: 'habilidad', name, def: v?.description || '' });
  for (const [name, v] of Object.entries(p.rules || {})) rows.push({ cat: 'regla', name, def: v?.name || v?.explanation || '' });

  if (pcountEl) pcountEl.textContent = `${rows.length} entradas`;

  el.innerHTML = `<table class="data-tbl"><thead><tr><th>Categoría</th><th>Nombre</th><th>Definición</th></tr></thead><tbody>
    ${rows.slice(0, 40).map(r =>
      `<tr><td><span class="chip" style="font-size:10px">${_esc(r.cat)}</span></td><td class="accent">${_esc(r.name)}</td><td style="font-size:12px;color:var(--text-secondary)">${_esc(r.def)}</td></tr>`
    ).join('')}
  </tbody></table>`;
}

function _renderCycles(container, data) {
  const el = container.querySelector('#l-cycles-list');
  if (!el) return;
  const p = data?.data ?? data;
  const items = p?.cycles || p?.items || (Array.isArray(p) ? p : []);
  const ccountEl = container.querySelector('#l-ccount');
  if (ccountEl) ccountEl.textContent = `${items.length} ciclos`;

  if (items.length === 0) {
    el.innerHTML = `<div class="empty-state">
      <div class="empty-title">Sin ciclos</div>
    </div>`;
    return;
  }

  el.innerHTML = [...items].reverse().slice(0, 15).map(c => {
    const ok  = c.success !== false && c.status !== 'error';
    const cls = ok ? 'success' : 'error';
    const ts  = _shortTime(c.timestamp || c.ts || c.completed_at);
    const msg = c.summary || c.message || `Ciclo #${c.cycle_id || c.id || ''}`;
    const dur = c.duration_ms ? `${c.duration_ms}ms` : '';
    return `<div class="tl-entry ${cls}">
      <div class="tl-time">${_esc(ts)}</div>
      <div class="tl-icon">${ok ? '✓' : '✕'}</div>
      <div class="tl-body">
        <div class="tl-msg">${_esc(msg)}</div>
        ${dur ? `<div class="tl-meta">${_esc(dur)}</div>` : ''}
      </div>
    </div>`;
  }).join('');
}

window.AtlasModuleLearning = { id: 'learning' };
