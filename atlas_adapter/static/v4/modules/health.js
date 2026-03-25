/**
 * ATLAS v4.2 — Health Module
 * Score dial + checks grid + system stats.
 */
import { poll, stop } from '../lib/polling.js';
import { set } from '../lib/state.js';

const POLL_ID = 'health-module';

function _esc(s) {
  const d = document.createElement('span');
  d.textContent = s ?? '';
  return d.innerHTML;
}

function _scoreColor(score) {
  if (score >= 90) return 'var(--accent-green)';
  if (score >= 70) return 'var(--accent-orange)';
  return 'var(--accent-red)';
}

function _scoreLabel(score) {
  if (score >= 90) return 'Óptimo';
  if (score >= 70) return 'Degradado';
  return 'Crítico';
}

function _fmt(val, unit = '') {
  return val !== undefined && val !== null ? `${val}${unit}` : '--';
}

export default {
  id: 'health',
  label: 'System Health',
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
          <h2>System Health</h2>
          <div style="margin-left:auto;display:flex;gap:8px;align-items:center">
            <button class="action-btn" id="btn-deep-check" style="font-size:11px;padding:4px 10px">
              <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" style="vertical-align:middle"><circle cx="12" cy="12" r="10"/><path d="M12 8v4M12 16h.01"/></svg>
              Diagnóstico
            </button>
            <button class="action-btn" onclick="location.hash='/doctor'" style="font-size:11px;padding:4px 10px;border-color:rgba(255,80,80,0.5);color:#ff5050">
              &#9829; ATLAS Doctor
            </button>
            <span class="live-badge">LIVE</span>
          </div>
        </div>
        <div class="module-body" id="health-body">
          <div class="stat-row">
            <div class="stat-card" id="hc-score" style="display:flex;align-items:center;gap:20px;grid-column:span 2">
              <div class="score-dial-wrap">
                <div class="score-dial" id="score-dial">
                  <div class="score-dial-inner">
                    <span class="value" id="score-val">--</span>
                    <span class="label">Score</span>
                  </div>
                </div>
                <div class="score-dial-sub" id="score-lbl">Cargando...</div>
              </div>
              <div style="flex:1">
                <div class="stat-card-label">Estado General</div>
                <div class="stat-card-value" id="h-status" style="font-size:20px">--</div>
                <div class="stat-card-sub" id="h-uptime"></div>
                <div class="stat-card-sub" id="h-version" style="margin-top:4px;font-family:var(--font-mono);font-size:11px"></div>
                <div class="stat-card-sub" style="margin-top:8px" id="h-ts"></div>
              </div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">CPU</div>
              <div class="stat-card-value" id="h-cpu">--</div>
              <div class="stat-card-sub">Uso del procesador</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Memoria</div>
              <div class="stat-card-value" id="h-mem">--</div>
              <div class="stat-card-sub" id="h-mem-mb"></div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">PID</div>
              <div class="stat-card-value accent" id="h-pid">--</div>
              <div class="stat-card-sub">Proceso Atlas</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">AutoRevive</div>
              <div class="stat-card-value" id="h-ar-running">--</div>
              <div class="stat-card-sub" id="h-ar-pids">--</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Último Reinicio</div>
              <div class="stat-card-value" id="h-ar-restart">--</div>
              <div class="stat-card-sub" id="h-ar-count">--</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Último Error</div>
              <div class="stat-card-value" id="h-ar-error">--</div>
              <div class="stat-card-sub" id="h-ar-streak">--</div>
            </div>
          </div>

          <div class="section-title">
            Verificaciones del Sistema
            <span class="count" id="checks-count">--</span>
          </div>
          <div class="check-grid" id="checks-grid">
            <div class="check-item" style="grid-column:1/-1">
              <div class="spinner"></div>
              <span style="font-size:12px;color:var(--text-muted)">Cargando checks...</span>
            </div>
          </div>
        </div>
      </div>
    `;

    container.querySelector('#btn-deep-check')?.addEventListener('click', () => _deepCheck(container));

    _fetchAndRender(container);
    poll(POLL_ID, '/health', 6000, async (data, err) => {
      if (data) {
        _render(container, data);
        try {
          const rAuto = await fetch('/api/push/autorevive/status');
          const ar = await rAuto.json().catch(() => null);
          _renderAutoRevive(container, ar);
        } catch (_) {
          _renderAutoRevive(container, null);
        }
      }
    });
  },

  destroy() { stop(POLL_ID); },
  badge() { return null; },
};

async function _fetchAndRender(container) {
  try {
    const [rHealth, rAuto] = await Promise.all([
      fetch('/health'),
      fetch('/api/push/autorevive/status').catch(() => null),
    ]);
    const data = await rHealth.json();
    let ar = null;
    if (rAuto && rAuto.ok) {
      ar = await rAuto.json().catch(() => null);
    }
    _render(container, data);
    _renderAutoRevive(container, ar);
  } catch (e) {
    const grid = container.querySelector('#checks-grid');
    if (grid) grid.innerHTML = `<div class="check-item error" style="grid-column:1/-1"><span style="color:var(--accent-red);font-size:12px">Error: ${_esc(e.message)}</span></div>`;
  }
}

function _render(container, data) {
  const score   = typeof data.score === 'number' ? data.score : (data.ok ? 100 : 0);
  const color   = _scoreColor(score);
  const pct     = `${(score / 100 * 360).toFixed(1)}deg`;

  // Score dial
  const dial = container.querySelector('#score-dial');
  if (dial) {
    dial.style.setProperty('--score-pct', pct);
    dial.style.setProperty('--score-color', color);
  }
  const sVal = container.querySelector('#score-val');
  if (sVal) { sVal.textContent = score; sVal.style.color = color; }
  const sLbl = container.querySelector('#score-lbl');
  if (sLbl) { sLbl.textContent = _scoreLabel(score); sLbl.style.color = color; }

  // Stats row
  _txt(container, '#h-status', data.ok ? '✓  Saludable' : '⚠  Degradado');
  const statusEl = container.querySelector('#h-status');
  if (statusEl) statusEl.style.color = data.ok ? 'var(--accent-green)' : 'var(--accent-orange)';

  _txt(container, '#h-uptime', data.uptime_human ? `Uptime: ${data.uptime_human}` : (data.uptime ? `Uptime: ${data.uptime}` : ''));
  _txt(container, '#h-version', data.version ? `v${data.version}` : '');
  _txt(container, '#h-ts', `Actualizado: ${new Date().toLocaleTimeString()}`);
  _txt(container, '#h-cpu', _fmt(data.cpu_percent, '%'));
  _txt(container, '#h-mem', _fmt(data.mem_percent, '%'));
  _txt(container, '#h-mem-mb', data.memory_mb ? `${data.memory_mb} MB` : '');
  _txt(container, '#h-pid', _fmt(data.pid));

  // Checks grid
  const checks = data.checks || {};
  const entries = Object.entries(checks);
  const grid = container.querySelector('#checks-grid');
  const countEl = container.querySelector('#checks-count');
  if (countEl) {
    const okCount = entries.filter(([,v]) => _checkOk(v)).length;
    countEl.textContent = `${okCount}/${entries.length} OK`;
  }

  if (!grid) return;
  if (entries.length === 0) {
    grid.innerHTML = `<div class="empty-state" style="grid-column:1/-1">
      <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5"><circle cx="12" cy="12" r="10"/><path d="M12 8v4M12 16h.01"/></svg>
      <div class="empty-title">Sin checks disponibles</div>
    </div>`;
    return;
  }

  grid.innerHTML = entries.map(([name, val]) => {
    const ok = _checkOk(val);
    const cls = ok ? 'ok' : (val === 'warn' || val?.status === 'warn' ? 'warn' : 'error');
    const icon = cls === 'ok' ? '✓' : cls === 'warn' ? '!' : '✕';
    const display = typeof val === 'object'
      ? (val.value !== undefined ? _esc(String(val.value)) : (val.status ? _esc(val.status) : 'active'))
      : _esc(String(val));
    return `<div class="check-item ${cls}">
      <div class="check-icon">${icon}</div>
      <div class="check-name">${_esc(name)}</div>
      <div class="check-val">${display}</div>
    </div>`;
  }).join('');

  set('health', { score, ok: data.ok });
}

function _renderAutoRevive(container, autoData) {
  const d = autoData && (autoData.data || autoData);
  if (!d) {
    _txt(container, '#h-ar-running', '--');
    _txt(container, '#h-ar-pids', '--');
    _txt(container, '#h-ar-restart', '--');
    _txt(container, '#h-ar-count', '--');
    _txt(container, '#h-ar-error', '--');
    _txt(container, '#h-ar-streak', '--');
    return;
  }
  const running = !!d.running;
  const pids = Array.isArray(d.pids) ? d.pids.join(', ') : '--';
  const lastRestart = d.last_restart_at ? new Date(d.last_restart_at).toLocaleString() : 'nunca';
  const restarts = d.restarts !== undefined && d.restarts !== null ? String(d.restarts) : '--';
  const lastError = d.last_error ? String(d.last_error).slice(0, 40) : 'sin error';
  const failStreak = d.fail_streak !== undefined && d.fail_streak !== null ? String(d.fail_streak) : '--';

  _txt(container, '#h-ar-running', running ? 'ACTIVO' : 'INACTIVO');
  const runEl = container.querySelector('#h-ar-running');
  if (runEl) runEl.style.color = running ? 'var(--accent-green)' : 'var(--accent-red)';
  _txt(container, '#h-ar-pids', pids || '--');
  _txt(container, '#h-ar-restart', lastRestart);
  _txt(container, '#h-ar-count', `reinicios: ${restarts}`);
  _txt(container, '#h-ar-error', lastError);
  _txt(container, '#h-ar-streak', `fail_streak: ${failStreak}`);
}

function _checkOk(val) {
  if (typeof val === 'boolean') return val;
  if (typeof val === 'string') return val === 'ok' || val === 'true' || val === 'healthy';
  if (typeof val === 'number') return val > 0;
  if (typeof val === 'object' && val !== null) return val.ok === true || val.status === 'ok';
  return false;
}

function _txt(container, sel, text) {
  const el = container.querySelector(sel);
  if (el) el.textContent = text;
}

async function _deepCheck(container) {
  const btn = container.querySelector('#btn-deep-check');
  if (btn) { btn.disabled = true; btn.textContent = '⟳ Analizando...'; }
  try {
    const r = await fetch('/health/deep');
    const data = await r.json();
    _render(container, data);
    window.AtlasToast?.show('Diagnóstico completo cargado', 'success');
  } catch (e) {
    window.AtlasToast?.show(`Diagnóstico fallido: ${e.message}`, 'error');
  } finally {
    if (btn) { btn.disabled = false; btn.innerHTML = '<svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" style="vertical-align:middle"><circle cx="12" cy="12" r="10"/><path d="M12 8v4M12 16h.01"/></svg> Diagnóstico'; }
  }
}

window.AtlasModuleHealth = { id: 'health' };
