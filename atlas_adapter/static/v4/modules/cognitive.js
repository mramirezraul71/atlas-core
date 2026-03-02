/**
 * ATLAS v4.2 — Cognitive / Nervous System Module
 * Mapa de nodos cerebrales, goals activos, memoria episódica y diagnóstico.
 * Endpoints: /cognitive/*, /nervous/*, /brain/*
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'cognitive-module';

function _esc(s) { const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML; }
function _time(ts) { if (!ts) return '--'; try { return new Date(ts).toLocaleTimeString(); } catch { return String(ts); } }

// Mapa de nodos cerebrales (igual que v3.8)
const BRAIN_NODES = [
  { id: 'cortex/frontal',   label: 'Cortex Frontal',   emoji: '🧠', desc: 'Control ejecutivo' },
  { id: 'cortex/parietal',  label: 'Cortex Parietal',  emoji: '🧠', desc: 'Procesamiento sensorial' },
  { id: 'cortex/temporal',  label: 'Cortex Temporal',  emoji: '🧠', desc: 'Audición y lenguaje' },
  { id: 'cortex/occipital', label: 'Cortex Occipital', emoji: '🧠', desc: 'Procesamiento visual' },
  { id: 'limbic',           label: 'Límbico',          emoji: '💛', desc: 'Emociones y motivación' },
  { id: 'hippo',            label: 'Hipocampo',        emoji: '🐘', desc: 'Memoria episódica' },
  { id: 'basal',            label: 'Ganglios Basales', emoji: '⚙️', desc: 'Control motor y hábitos' },
  { id: 'brainstem',        label: 'Tronco Encefálico',emoji: '🔴', desc: 'Funciones vitales' },
  { id: 'medulla',          label: 'Médula',           emoji: '🔗', desc: 'Control autónomo' },
  { id: 'motor',            label: 'Motor',            emoji: '🤖', desc: 'Control de movimiento' },
  { id: 'learning',         label: 'Learning',         emoji: '📚', desc: 'Aprendizaje activo' },
  { id: 'snp',              label: 'SNP',              emoji: '🌐', desc: 'Sistema nervioso periférico' },
];

export default {
  id: 'cognitive',
  label: 'Sistema Nervioso',
  icon: 'activity',
  category: 'monitoring',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" id="cog-back">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Dashboard
          </button>
          <h2>Sistema Nervioso / Cognitivo</h2>
          <div style="margin-left:auto;display:flex;gap:8px;align-items:center">
            <span class="live-badge">LIVE</span>
          </div>
        </div>

        <div class="module-body">

          <!-- KPIs del sistema -->
          <div class="stat-row" style="margin-bottom:20px" id="cog-kpis">
            <div style="padding:12px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
          </div>

          <!-- Acciones -->
          <div class="action-bar" style="padding-top:0;margin-bottom:20px">
            <button class="action-btn primary" id="btn-cog-refresh">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="23 4 23 10 17 10"/><path d="M20.49 15a9 9 0 11-2.12-9.36L23 10"/></svg>
              Actualizar
            </button>
            <button class="action-btn" id="btn-cog-diagnostic">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="10"/><line x1="12" y1="8" x2="12" y2="12"/><line x1="12" y1="16" x2="12.01" y2="16"/></svg>
              Diagnóstico nervioso
            </button>
            <span id="cog-msg" style="font-size:11px;color:var(--text-muted);align-self:center"></span>
          </div>

          <!-- Mapa de nodos cerebrales -->
          <div class="section-title">Nodos del Sistema Nervioso
            <span class="count" id="cog-nodes-count"></span>
          </div>
          <div id="cog-nodes-grid" style="display:grid;grid-template-columns:repeat(auto-fill,minmax(180px,1fr));gap:8px;margin-bottom:24px">
            ${BRAIN_NODES.map(n => `
              <div class="provider-card" data-node-id="${n.id}" style="cursor:pointer;transition:border-color .2s" title="${n.desc}">
                <div style="font-size:24px;margin-bottom:6px">${n.emoji}</div>
                <div class="provider-name">${_esc(n.label)}</div>
                <div class="provider-role">${_esc(n.desc)}</div>
                <div class="provider-status" id="cog-node-${n.id.replace('/', '-')}" style="margin-top:8px">
                  <div class="provider-dot"></div>
                  <span class="cog-node-status">--</span>
                </div>
              </div>`).join('')}
          </div>

          <!-- Panel de detalle de nodo (aparece al hacer clic) -->
          <div id="cog-node-detail" style="display:none;margin-bottom:24px">
            <div style="display:flex;align-items:center;gap:10px;margin-bottom:12px">
              <div id="cog-detail-emoji" style="font-size:28px"></div>
              <div>
                <div style="font-weight:600;font-size:16px" id="cog-detail-label"></div>
                <div style="font-size:11px;color:var(--text-muted)" id="cog-detail-desc"></div>
              </div>
              <button class="action-btn" id="btn-node-close" style="margin-left:auto;font-size:11px;padding:4px 10px">Cerrar ✕</button>
            </div>
            <div id="cog-detail-data" class="codebox" style="max-height:300px;overflow-y:auto;font-size:11px">Cargando...</div>
          </div>

          <!-- Goals activos -->
          <div class="section-title">Goals Activos
            <span class="count" id="cog-goals-count"></span>
          </div>
          <div id="cog-goals" style="margin-bottom:24px">
            <div style="padding:12px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
          </div>

          <!-- Memoria episódica reciente -->
          <div class="section-title">Memoria Episódica Reciente</div>
          <div class="timeline" id="cog-memory" style="margin-bottom:20px">
            <div style="padding:12px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
          </div>

          <!-- Diagnóstico expandido -->
          <details>
            <summary style="cursor:pointer;font-size:12px;color:var(--text-muted);padding:4px 0;user-select:none">
              Ver diagnóstico del sistema nervioso
            </summary>
            <div id="cog-diag" class="codebox" style="margin-top:8px;max-height:400px;overflow-y:auto;font-size:11px">
              Haz clic en "Diagnóstico nervioso" para cargar.
            </div>
          </details>

        </div>
      </div>
    `;

    container.querySelector('#cog-back')?.addEventListener('click', () => { location.hash = '/'; });
    container.querySelector('#btn-cog-refresh')?.addEventListener('click',    () => _loadAll(container));
    container.querySelector('#btn-cog-diagnostic')?.addEventListener('click', () => _loadDiagnostic(container));
    container.querySelector('#btn-node-close')?.addEventListener('click', () => {
      const panel = container.querySelector('#cog-node-detail');
      if (panel) panel.style.display = 'none';
    });

    // Click en nodos cerebrales
    container.querySelectorAll('[data-node-id]').forEach(card => {
      card.addEventListener('click', () => _inspectNode(card.dataset.nodeId, container));
    });

    _loadAll(container);

    poll(POLL_ID,         '/cognitive/status',          10000, (d) => { if (d) _renderStatus(container, d); });
    poll(POLL_ID + ':g',  '/cognitive/goals',           15000, (d) => { if (d) _renderGoals(container, d); });
    poll(POLL_ID + ':m',  '/cognitive/memory/recent',   20000, (d) => { if (d) _renderMemory(container, d); });
    poll(POLL_ID + ':ns', '/nervous/status',             10000, (d) => { if (d) _renderNervous(container, d); });
  },

  destroy() {
    stop(POLL_ID);
    stop(POLL_ID + ':g');
    stop(POLL_ID + ':m');
    stop(POLL_ID + ':ns');
  },
};

/* ─── Carga completa ─────────────────────────────────────────────────── */

async function _loadAll(container) {
  const msg = container.querySelector('#cog-msg');
  if (msg) msg.textContent = 'Actualizando...';
  const [cogStatus, goals, memory, nervStatus] = await Promise.all([
    fetch('/cognitive/status').then(r => r.json()).catch(() => null),
    fetch('/cognitive/goals').then(r => r.json()).catch(() => null),
    fetch('/cognitive/memory/recent').then(r => r.json()).catch(() => null),
    fetch('/nervous/status').then(r => r.json()).catch(() => null),
  ]);
  if (cogStatus) _renderStatus(container, cogStatus);
  if (goals)     _renderGoals(container, goals);
  if (memory)    _renderMemory(container, memory);
  if (nervStatus) _renderNervous(container, nervStatus);
  if (msg) msg.textContent = '';
}

/* ─── Status KPIs ────────────────────────────────────────────────────── */

function _renderStatus(container, data) {
  const p = data?.data ?? data ?? {};
  const kpis = container.querySelector('#cog-kpis');
  if (!kpis) return;

  kpis.innerHTML = `
    <div class="stat-card hero">
      <div class="stat-card-label">Estado cognitivo</div>
      <div class="stat-card-value ${p.ok || p.status === 'active' ? 'green' : 'orange'}" style="font-size:18px">
        ${p.status || (p.ok ? 'Activo' : 'Degradado')}
      </div>
    </div>
    <div class="stat-card">
      <div class="stat-card-label">Señales/min</div>
      <div class="stat-card-value accent">${_esc(String(p.signals_per_min ?? p.signals ?? '--'))}</div>
    </div>
    <div class="stat-card">
      <div class="stat-card-label">Latencia</div>
      <div class="stat-card-value">${p.latency_ms ? `${p.latency_ms}ms` : '--'}</div>
    </div>
    <div class="stat-card">
      <div class="stat-card-label">Goals activos</div>
      <div class="stat-card-value">${_esc(String(p.active_goals ?? p.goals ?? '--'))}</div>
    </div>
    <div class="stat-card">
      <div class="stat-card-label">Uptime</div>
      <div class="stat-card-value" style="font-size:13px">${_esc(String(p.uptime || p.uptime_human || '--'))}</div>
    </div>
  `;

  // Actualizar nodos si hay info de módulos en el status
  const modules = p?.modules || p?.cortex_modules || {};
  Object.entries(modules).forEach(([nodeId, info]) => {
    const normalized = nodeId.replace('/', '-');
    const nodeEl = container.querySelector(`#cog-node-${normalized}`);
    if (!nodeEl) return;
    const ok = typeof info === 'boolean' ? info : (info?.ok !== false && info?.status !== 'error');
    const dot = nodeEl.querySelector('.provider-dot');
    const txt = nodeEl.querySelector('.cog-node-status');
    if (dot) dot.className = `provider-dot ${ok ? 'ok' : 'down'}`;
    if (txt) txt.textContent = ok ? 'Activo' : 'Error';
  });
}

function _renderNervous(container, data) {
  const p = data?.data ?? data ?? {};
  // Update nervous signals KPI if available
  const kpis = container.querySelector('#cog-kpis');
  if (!kpis || kpis.children.length === 0) return;
  // We can update existing cards or just display nervous info elsewhere
}

/* ─── Goals ─────────────────────────────────────────────────────────── */

function _renderGoals(container, data) {
  const el = container.querySelector('#cog-goals');
  if (!el) return;
  const p = data?.data ?? data;
  const goals = Array.isArray(p) ? p : (p?.goals || p?.items || []);
  const countEl = container.querySelector('#cog-goals-count');
  if (countEl) countEl.textContent = `${goals.length} goals`;

  if (!goals.length) {
    el.innerHTML = `<div class="empty-state"><div class="empty-title">Sin goals activos</div></div>`;
    return;
  }

  el.innerHTML = `<div style="display:flex;flex-direction:column;gap:8px">
    ${goals.slice(0, 15).map(g => {
      const name    = g.name || g.title || g.id || '--';
      const pct     = g.progress !== undefined ? Math.round(g.progress * (g.progress <= 1 ? 100 : 1)) : null;
      const status  = g.status || g.state || 'active';
      const priority = g.priority || g.importance || '';
      const cls = status === 'done' || status === 'completed' ? 'green' : status === 'failed' ? 'red' : 'accent';
      return `<div style="background:var(--surface-1);border:1px solid var(--border-subtle);border-radius:8px;padding:10px">
        <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:${pct !== null ? '6px' : '0'}">
          <div>
            <span style="font-weight:600;font-size:13px">${_esc(name)}</span>
            ${priority ? `<span class="chip orange" style="margin-left:6px;font-size:10px">${_esc(priority)}</span>` : ''}
          </div>
          <span class="chip ${cls}">${_esc(status)}</span>
        </div>
        ${pct !== null ? `<div style="background:var(--surface-0);border-radius:4px;height:4px;overflow:hidden">
          <div style="width:${pct}%;height:100%;background:var(--accent-primary);border-radius:4px;transition:width .3s"></div>
        </div>
        <div style="font-size:10px;color:var(--text-muted);margin-top:3px">${pct}%</div>` : ''}
        ${g.description ? `<div style="font-size:11px;color:var(--text-secondary);margin-top:4px">${_esc(g.description.slice(0, 100))}</div>` : ''}
      </div>`;
    }).join('')}
  </div>`;
}

/* ─── Memoria episódica ──────────────────────────────────────────────── */

function _renderMemory(container, data) {
  const tl = container.querySelector('#cog-memory');
  if (!tl) return;
  const p = data?.data ?? data;
  const items = Array.isArray(p) ? p : (p?.episodes || p?.items || p?.memories || []);

  if (!items.length) {
    tl.innerHTML = `<div class="empty-state"><div class="empty-title">Sin episodios de memoria</div></div>`;
    return;
  }

  tl.innerHTML = items.slice(0, 20).map(ep => {
    const msg  = ep.summary || ep.text || ep.content || ep.event || JSON.stringify(ep).slice(0, 80);
    const ts   = _time(ep.timestamp || ep.ts || ep.time);
    const type = (ep.type || ep.category || 'memory').toLowerCase();
    const cls  = type.includes('error') ? 'error' : type.includes('warn') ? 'warn' : type.includes('success') ? 'success' : 'info';
    return `<div class="tl-entry ${cls}">
      <div class="tl-time">${_esc(ts)}</div>
      <div class="tl-icon" style="font-size:11px">🧠</div>
      <div class="tl-body">
        <div class="tl-msg">${_esc(msg)}</div>
        ${ep.source || ep.module ? `<div class="tl-meta">${_esc(ep.source || ep.module)}</div>` : ''}
      </div>
    </div>`;
  }).join('');
}

/* ─── Inspección de nodo ─────────────────────────────────────────────── */

async function _inspectNode(nodeId, container) {
  const node = BRAIN_NODES.find(n => n.id === nodeId);
  if (!node) return;

  const panel  = container.querySelector('#cog-node-detail');
  const emoji  = container.querySelector('#cog-detail-emoji');
  const label  = container.querySelector('#cog-detail-label');
  const desc   = container.querySelector('#cog-detail-desc');
  const dataEl = container.querySelector('#cog-detail-data');

  if (emoji) emoji.textContent = node.emoji;
  if (label) label.textContent = node.label;
  if (desc)  desc.textContent  = node.desc;
  if (panel) panel.style.display = '';
  if (dataEl) dataEl.textContent = 'Cargando...';

  // Scroll to panel
  panel?.scrollIntoView({ behavior: 'smooth', block: 'nearest' });

  try {
    const r = await fetch(`/cognitive/${nodeId}`);
    const d = await r.json().catch(() => null);
    if (!r.ok) throw new Error(`HTTP ${r.status}`);
    const payload = d?.data ?? d ?? {};
    if (dataEl) dataEl.textContent = JSON.stringify(payload, null, 2);

    // Actualizar el dot del nodo
    const normalized = nodeId.replace('/', '-');
    const nodeEl = container.querySelector(`#cog-node-${normalized}`);
    if (nodeEl) {
      const ok = d?.ok !== false;
      const dot = nodeEl.querySelector('.provider-dot');
      const txt = nodeEl.querySelector('.cog-node-status');
      if (dot) dot.className = `provider-dot ${ok ? 'ok' : 'down'}`;
      if (txt) txt.textContent = ok ? 'Activo' : 'Error';
      const card = nodeEl.closest('.provider-card');
      if (card) card.className = `provider-card ${ok ? 'active' : 'down'}`;
    }
  } catch (e) {
    if (dataEl) dataEl.textContent = `Error: ${e.message}`;
  }
}

/* ─── Diagnóstico completo ───────────────────────────────────────────── */

async function _loadDiagnostic(container) {
  const el  = container.querySelector('#cog-diag');
  const msg = container.querySelector('#cog-msg');
  if (msg) msg.textContent = 'Ejecutando diagnóstico...';

  // Expand details
  const details = container.querySelector('details');
  if (details) details.open = true;

  if (el) el.textContent = 'Cargando diagnóstico...';
  try {
    const r = await fetch('/nervous/diagnostic');
    const d = await r.json().catch(() => null);
    if (!r.ok) throw new Error(`HTTP ${r.status}`);
    const payload = d?.data ?? d ?? {};
    if (el) el.textContent = JSON.stringify(payload, null, 2);
    if (msg) msg.textContent = '';
    window.AtlasToast?.show('Diagnóstico completo', 'success');
  } catch (e) {
    if (el) el.textContent = `Error: ${e.message}`;
    if (msg) msg.textContent = `Error: ${e.message}`;
  }
}

window.AtlasModuleCognitive = { id: 'cognitive' };
