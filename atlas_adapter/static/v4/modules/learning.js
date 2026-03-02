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
  { id: 'status',   url: '/api/learning/status',   label: 'Estado del Motor' },
  { id: 'patterns', url: '/api/learning/patterns',  label: 'Patrones' },
  { id: 'cycles',   url: '/api/learning/cycles',    label: 'Ciclos recientes' },
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
    poll(POLL_ID, ENDPOINTS[0].url, 8000, () => _refresh(container));
  },

  destroy() { stop(POLL_ID); },
};

async function _refresh(container) {
  const [status, patterns, cycles] = await Promise.all(
    ENDPOINTS.map(e => fetch(e.url).then(r => r.json()).catch(() => null))
  );
  _renderStatus(container, status);
  _renderPatterns(container, patterns);
  _renderCycles(container, cycles);
}

function _renderStatus(container, data) {
  if (!data) return;
  const p = data?.data ?? data;
  const active = p?.active === true || p?.status === 'running' || p?.enabled === true;
  const txt = (id, v) => { const e = container.querySelector(id); if (e) e.textContent = v; };
  txt('#l-state', active ? 'Activo' : 'Inactivo');
  const stEl = container.querySelector('#l-state');
  if (stEl) stEl.style.color = active ? 'var(--accent-green)' : 'var(--text-muted)';
  txt('#l-mode', p?.mode || p?.learning_mode || '');
  txt('#l-cycles', p?.total_cycles ?? p?.cycles ?? '--');
  txt('#l-last', _shortTime(p?.last_cycle || p?.last_run));
}

function _renderPatterns(container, data) {
  const el = container.querySelector('#l-pattern-list');
  if (!el) return;
  const p = data?.data ?? data;
  const items = p?.patterns || p?.items || (Array.isArray(p) ? p : []);
  const countEl = container.querySelector('#l-patterns');
  const pcountEl = container.querySelector('#l-pcount');
  if (countEl) countEl.textContent = items.length;
  if (pcountEl) pcountEl.textContent = `${items.length} patrones`;

  if (items.length === 0) {
    el.innerHTML = `<div class="empty-state">
      <svg width="28" height="28" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5"><polyline points="22 12 18 12 15 21 9 3 6 12 2 12"/></svg>
      <div class="empty-title">Sin patrones registrados</div>
      <div class="empty-sub">El motor no ha aprendido patrones aún</div>
    </div>`;
    return;
  }

  el.innerHTML = `<table class="data-tbl"><thead><tr><th>Patrón</th><th>Confianza</th><th>Usos</th><th>Actualizado</th></tr></thead><tbody>
    ${items.slice(0, 30).map(p => {
      const name  = _esc(p.name || p.pattern || p.id || '--');
      const conf  = p.confidence !== undefined ? `${Math.round(p.confidence * 100)}%` : '--';
      const uses  = p.uses ?? p.count ?? '--';
      const ts    = _shortTime(p.updated_at || p.ts);
      return `<tr><td>${name}</td><td class="accent">${_esc(conf)}</td><td>${_esc(String(uses))}</td><td class="mono">${_esc(ts)}</td></tr>`;
    }).join('')}
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
