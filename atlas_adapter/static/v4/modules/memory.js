/**
 * ATLAS v4.2 — Cognitive Memory Module
 * Tarjetas de estado para Lifelog, World Model y Autobiographical memory.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'memory-module';

function _esc(s) { const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML; }

const MEMORIES = [
  { key: 'lifelog',        label: 'Lifelog',          url: '/api/cognitive-memory/lifelog/status',        icon: '📋' },
  { key: 'world',          label: 'World Model',       url: '/api/cognitive-memory/world-model/status',    icon: '🌐' },
  { key: 'autobio',        label: 'Autobiographical',  url: '/api/cognitive-memory/autobiographical/status', icon: '🧠' },
];

export default {
  id: 'memory',
  label: 'Memoria Cognitiva',
  icon: 'brain',
  category: 'monitoring',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Inicio
          </button>
          <h2>Memoria Cognitiva</h2>
          <div style="margin-left:auto"><span class="live-badge">LIVE</span></div>
        </div>
        <div class="module-body">
          <div class="stat-row" id="mem-stat-row">
            ${MEMORIES.map(m => `<div class="stat-card" id="mc-${m.key}">
              <div class="stat-card-label">${m.icon} ${m.label}</div>
              <div class="stat-card-value">--</div>
            </div>`).join('')}
          </div>

          <div class="section-title">Detalles</div>
          <div id="mem-details">
            <div style="padding:20px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
          </div>
        </div>
      </div>
    `;

    _refresh(container);
    poll(POLL_ID, MEMORIES[0].url, 8000, () => _refresh(container));
  },

  destroy() { stop(POLL_ID); },
};

async function _refresh(container) {
  const results = await Promise.all(
    MEMORIES.map(m => fetch(m.url).then(r => r.json()).catch(() => null))
  );

  // Update stat cards
  results.forEach((data, i) => {
    const m = MEMORIES[i];
    const card = container.querySelector(`#mc-${m.key}`);
    if (!card) return;
    const ok = data ? (data.ok !== false) : false;
    const valEl = card.querySelector('.stat-card-value');
    if (valEl) {
      valEl.textContent = data ? (ok ? 'OK' : 'Degradado') : 'N/A';
      valEl.className = 'stat-card-value ' + (data ? (ok ? 'green' : 'orange') : '');
    }
    card.style.borderColor = data ? (ok ? 'var(--accent-green)' : 'var(--accent-orange)') : 'var(--border)';
  });

  // Details table
  const details = container.querySelector('#mem-details');
  if (!details) return;

  const rows = [];
  results.forEach((data, i) => {
    const m = MEMORIES[i];
    const p = data?.data ?? data ?? {};
    rows.push(`<tr style="background:var(--bg-elevated)">
      <td class="accent" colspan="2" style="font-weight:600">${m.icon} ${m.label}</td>
    </tr>`);
    if (!data) {
      rows.push(`<tr><td colspan="2" style="color:var(--text-muted)">No disponible</td></tr>`);
    } else {
      const entries = Object.entries(p).filter(([k]) => !['ok','ms','error'].includes(k)).slice(0, 8);
      if (entries.length === 0) {
        rows.push(`<tr><td colspan="2" style="color:var(--text-muted)">Sin datos</td></tr>`);
      } else {
        entries.forEach(([k, v]) => {
          const display = typeof v === 'object' ? JSON.stringify(v).slice(0, 80) : String(v);
          rows.push(`<tr><td style="color:var(--text-secondary);width:200px">${_esc(k)}</td><td class="mono">${_esc(display)}</td></tr>`);
        });
      }
    }
  });

  details.innerHTML = `<table class="data-tbl"><thead><tr><th>Campo</th><th>Valor</th></tr></thead><tbody>${rows.join('')}</tbody></table>`;
}

window.AtlasModuleMemory = { id: 'memory' };
