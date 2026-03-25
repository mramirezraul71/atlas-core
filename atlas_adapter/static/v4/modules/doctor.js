/**
 * ATLAS DOCTOR v1.0 — Sistema Nervioso Central
 * Dashboard de salud: 15 puertos, 5 capas, anomalías, historial de reparaciones.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'doctor-module';

const TIER_COLORS = { 'CRASH': 'red', 'CRÍTICO': 'orange', 'DEGRADADO': 'yellow', 'WARNING': 'blue', 'EVOLUCIÓN': 'purple' };
const LAYER_ICONS = { api: '⚙', hardware: '🤖', vision: '👁', quant: '📈', cognitive: '🧠', comms: '💬', monitoring: '📊' };

function _esc(s) { const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML; }
function _ts(s) { if (!s) return '--:--'; try { return new Date(s).toLocaleTimeString(); } catch { return String(s).slice(0, 8); } }
function _tierBadge(t) {
  const col = TIER_COLORS[t] || 'grey';
  return `<span class="chip" style="background:var(--${col},#888);color:#fff;font-size:11px">${_esc(t)}</span>`;
}

function _portGrid(ports) {
  if (!ports || !Object.keys(ports).length) return '<div class="empty-state">Sin datos de puertos</div>';
  return `<div class="check-grid">${
    Object.entries(ports).map(([name, p]) => {
      const up = p.up;
      const icon = up ? '✓' : '✕';
      const cls  = up ? 'success' : 'error';
      return `<div class="check-item ${cls}" title="${_esc(p.description || '')}">
        <span class="check-icon">${icon}</span>
        <span class="check-label">${_esc(name)}</span>
        <span class="check-sub">:${p.port} ${LAYER_ICONS[p.layer] || ''}</span>
      </div>`;
    }).join('')
  }</div>`;
}

function _anomalyList(items) {
  if (!items?.length) return '<div class="empty-state" style="color:var(--green)">✓ Sin anomalías activas</div>';
  return `<div class="timeline">${
    items.map(a => `
      <div class="tl-entry error">
        <div class="tl-icon">!</div>
        <div class="tl-body">
          <div class="tl-title">${_esc(a.component)} ${_tierBadge(a.tier_label || 'TIER' + a.tier)}</div>
          <div class="tl-sub">${_esc(a.description)}</div>
          <div class="tl-meta">${_esc(a.layer)} · ${_ts(a.ts)}</div>
        </div>
      </div>`).join('')
  }</div>`;
}

function _historyTable(items) {
  if (!items?.length) return '<div class="empty-state">Sin eventos registrados</div>';
  return `<table class="data-tbl"><thead><tr>
    <th>Hora</th><th>Componente</th><th>Capa</th><th>Tier</th><th>Acción</th><th>Resultado</th>
  </tr></thead><tbody>${
    items.slice(0, 40).map(e => {
      const healed = e.healed === 1 || e.healed === true;
      const rowCls = healed ? 'ok' : (e.tier <= 1 ? 'error' : '');
      return `<tr class="${rowCls}">
        <td>${_ts(e.ts)}</td>
        <td>${_esc(e.component)}</td>
        <td>${LAYER_ICONS[e.layer] || ''} ${_esc(e.layer)}</td>
        <td>${_tierBadge(e.tier_label || '')}</td>
        <td>${_esc(e.action_type)}</td>
        <td>${healed ? '<span style="color:var(--green)">✓ SANADO</span>' : _esc(e.outcome || e.action_detail || '').slice(0, 60)}</td>
      </tr>`;
    }).join('')
  }</tbody></table>`;
}

export default {
  id: 'doctor',
  label: 'ATLAS DOCTOR',
  icon: 'activity',
  category: 'monitoring',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Inicio
          </button>
          <h2>ATLAS DOCTOR — Sistema Nervioso Central</h2>
          <div style="margin-left:auto;display:flex;gap:8px;align-items:center">
            <span class="live-badge">LIVE</span>
            <button class="action-btn" id="dr-diagnose-btn" title="Diagnóstico inmediato">⚡ Diagnosticar</button>
            <button class="action-btn" id="dr-emergency-btn" title="Emergency Stop" style="color:var(--red)">🚨 Emergency</button>
          </div>
        </div>
        <div class="module-body">

          <!-- Stat row -->
          <div class="stat-row" id="dr-stats">
            <div class="stat-card">
              <div class="stat-card-label">Estado</div>
              <div class="stat-card-value" id="dr-status-val">--</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Ciclo #</div>
              <div class="stat-card-value" id="dr-cycle">--</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Anomalías</div>
              <div class="stat-card-value red" id="dr-anomaly-count">0</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Sanadas</div>
              <div class="stat-card-value green" id="dr-healed-count">0</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Último ciclo</div>
              <div class="stat-card-value" id="dr-last-ts">--</div>
            </div>
          </div>

          <!-- Tabs -->
          <div class="action-bar" style="margin:12px 0 0">
            <button class="action-btn active" data-tab="ports" onclick="window._drTab(this,'ports')">🔌 Puertos (15)</button>
            <button class="action-btn" data-tab="anomalies" onclick="window._drTab(this,'anomalies')">⚠ Anomalías</button>
            <button class="action-btn" data-tab="history" onclick="window._drTab(this,'history')">📋 Historial</button>
            <button class="action-btn" data-tab="chromadb" onclick="window._drTab(this,'chromadb')">🧠 ChromaDB</button>
          </div>

          <!-- Tab: Puertos -->
          <div id="dr-tab-ports" class="dr-tab-content">
            <div id="dr-ports-grid"><div class="empty-state">Cargando…</div></div>
          </div>

          <!-- Tab: Anomalías -->
          <div id="dr-tab-anomalies" class="dr-tab-content" style="display:none">
            <div id="dr-anomalies-list"><div class="empty-state">Cargando…</div></div>
          </div>

          <!-- Tab: Historial -->
          <div id="dr-tab-history" class="dr-tab-content" style="display:none">
            <div id="dr-history-wrap"><div class="empty-state">Cargando…</div></div>
          </div>

          <!-- Tab: ChromaDB -->
          <div id="dr-tab-chromadb" class="dr-tab-content" style="display:none">
            <div id="dr-chroma-wrap"><div class="empty-state">Cargando…</div></div>
          </div>

        </div>
      </div>`;

    // Tab switcher
    window._drTab = (btn, tab) => {
      document.querySelectorAll('.dr-tab-content').forEach(el => el.style.display = 'none');
      document.querySelectorAll('[data-tab]').forEach(b => b.classList.remove('active'));
      const el = document.getElementById(`dr-tab-${tab}`);
      if (el) el.style.display = '';
      btn.classList.add('active');
    };

    // Botón diagnose
    document.getElementById('dr-diagnose-btn').onclick = async () => {
      const btn = document.getElementById('dr-diagnose-btn');
      btn.disabled = true;
      btn.textContent = '⏳ Diagnosticando…';
      try {
        const r = await fetch('/doctor/diagnose', { method: 'POST' });
        const d = await r.json();
        _applyStatus(d.data);
        btn.textContent = '✓ Listo';
      } catch (e) {
        btn.textContent = '✕ Error';
      } finally {
        setTimeout(() => { btn.disabled = false; btn.textContent = '⚡ Diagnosticar'; }, 2000);
      }
    };

    // Botón emergency
    document.getElementById('dr-emergency-btn').onclick = async () => {
      if (!confirm('¿Confirmar EMERGENCY STOP? Se detendrá el LiveLoop de Quant y se emitirá alerta.')) return;
      try {
        await fetch('/doctor/emergency', { method: 'POST' });
        alert('Emergency Stop ejecutado.');
      } catch(e) { alert('Error: ' + e); }
    };
  },

  mount() {
    // Poll status
    poll(POLL_ID, async () => {
      try {
        const [statusR, portsR, histR, chromaR] = await Promise.all([
          fetch('/doctor/status').then(r => r.json()),
          fetch('/doctor/ports').then(r => r.json()),
          fetch('/doctor/history?limit=40').then(r => r.json()),
          fetch('/doctor/chromadb').then(r => r.json()),
        ]);

        if (statusR.ok) _applyStatus(statusR.data);
        if (portsR.ok) _applyPorts(portsR.data);
        if (histR.ok)  _applyHistory(histR.data?.items || []);
        if (chromaR.ok) _applyChroma(chromaR.data);

        // Anomalías del status
        const anomalies = statusR.data?.anomalies || [];
        const el = document.getElementById('dr-anomalies-list');
        if (el) el.innerHTML = _anomalyList(anomalies);

      } catch (e) { /* silent */ }
    }, 15000);
  },

  unmount() {
    stop(POLL_ID);
    delete window._drTab;
  },
};

function _applyStatus(data) {
  if (!data) return;
  const ok = data.ok;
  const sv = document.getElementById('dr-status-val');
  if (sv) {
    sv.textContent = ok ? 'OK' : `${data.anomalies_count} anomalías`;
    sv.className = 'stat-card-value ' + (ok ? 'green' : 'red');
  }
  const cycle = document.getElementById('dr-cycle');
  if (cycle) cycle.textContent = data.cycle ?? '--';
  const ac = document.getElementById('dr-anomaly-count');
  if (ac) ac.textContent = data.anomalies_count ?? 0;
  const hc = document.getElementById('dr-healed-count');
  if (hc) hc.textContent = data.healed_count ?? 0;
  const lt = document.getElementById('dr-last-ts');
  if (lt) lt.textContent = _ts(data.last_ts);
}

function _applyPorts(data) {
  const el = document.getElementById('dr-ports-grid');
  if (el) el.innerHTML = _portGrid(data);
}

function _applyHistory(items) {
  const el = document.getElementById('dr-history-wrap');
  if (el) el.innerHTML = _historyTable(items);
}

function _applyChroma(data) {
  const el = document.getElementById('dr-chroma-wrap');
  if (!el) return;
  if (!data) { el.innerHTML = '<div class="empty-state">Sin datos</div>'; return; }
  const ok = data.status === 'ok';
  el.innerHTML = `
    <div class="stat-row">
      <div class="stat-card">
        <div class="stat-card-label">Estado ChromaDB</div>
        <div class="stat-card-value ${ok ? 'green' : 'red'}">${_esc(data.status)}</div>
      </div>
      <div class="stat-card">
        <div class="stat-card-label">Colecciones</div>
        <div class="stat-card-value">${data.collections ?? '--'}</div>
      </div>
    </div>
    ${data.names?.length ? `<div class="api-path-list">${data.names.map(n => `<span class="chip">${_esc(n)}</span>`).join(' ')}</div>` : ''}
    ${data.error ? `<div class="empty-state red">${_esc(data.error)}</div>` : ''}
  `;
}
