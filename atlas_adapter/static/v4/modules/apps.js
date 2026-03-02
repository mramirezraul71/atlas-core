/**
 * ATLAS v4.2 — Mis Apps Module
 * Panel de Brazos Subordinados: Rauli Vision + Rauli Panadería.
 * Health en tiempo real, métricas clave y acceso directo a cada dashboard.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID_VISION    = 'apps-rauli-vision';
const POLL_ID_PANADERIA = 'apps-rauli-panaderia';

const APPS = [
  {
    id:       'rauli-vision',
    name:     'Rauli Vision',
    role:     'Brazo Sensorial — Visión Artificial',
    desc:     'Detección de objetos, tracking y análisis de imágenes en tiempo real.',
    port:     3000,
    health:   'http://127.0.0.1:3000/api/health',
    dashboard:'http://127.0.0.1:3000',
    icon:     '👁️',
    color:    'var(--accent-primary)',
    pollId:   POLL_ID_VISION,
    metrics: [
      { key: 'detections',  label: 'Detecciones hoy', path: ['detections_today', 'count'] },
      { key: 'fps',         label: 'FPS activo',       path: ['fps', 'current'] },
      { key: 'cameras',     label: 'Cámaras activas',  path: ['cameras_active', 'total'] },
    ],
  },
  {
    id:       'rauli-panaderia',
    name:     'Rauli Panadería',
    role:     'Brazo Operativo — Gestión de Negocio',
    desc:     'Inventarios, ventas, producción y control operacional de la panadería.',
    port:     3001,
    health:   'http://127.0.0.1:3001/api/health',
    dashboard:'http://127.0.0.1:3001',
    icon:     '🥖',
    color:    'var(--accent-green)',
    pollId:   POLL_ID_PANADERIA,
    metrics: [
      { key: 'ventas',      label: 'Ventas hoy',       path: ['sales_today', 'total'] },
      { key: 'inventario',  label: 'Items inventario',  path: ['inventory', 'items'] },
      { key: 'alertas',     label: 'Alertas activas',   path: ['alerts', 'active'] },
    ],
  },
];

function _esc(s) { const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML; }

export default {
  id: 'apps',
  label: 'Mis Apps',
  icon: 'layers',
  category: 'brazos',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" id="apps-back">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Dashboard
          </button>
          <h2>Mis Apps — Brazos ATLAS</h2>
          <div style="margin-left:auto;display:flex;gap:8px;align-items:center">
            <span class="live-badge">LIVE</span>
          </div>
        </div>

        <div class="module-body">

          <!-- Intro -->
          <div style="background:var(--surface-1);border:1px solid var(--border-subtle);border-radius:12px;padding:16px;margin-bottom:24px;display:flex;align-items:center;gap:14px">
            <svg width="28" height="28" viewBox="0 0 24 24" fill="none" stroke="var(--accent-primary)" stroke-width="1.5"><circle cx="12" cy="12" r="10"/><path d="M12 8v4l3 3"/><path d="M2 12h3M19 12h3M12 2v3M12 19v3"/></svg>
            <div>
              <div style="font-size:13px;font-weight:600;color:var(--text-primary)">Brazos Subordinados</div>
              <div style="font-size:12px;color:var(--text-muted);margin-top:2px">
                Los brazos operan de forma independiente. ATLAS actúa como único intermediario — nunca se comunican entre sí directamente.
              </div>
            </div>
          </div>

          <!-- Tarjetas de apps -->
          <div style="display:grid;grid-template-columns:repeat(auto-fill,minmax(440px,1fr));gap:20px">
            ${APPS.map(app => `
              <div class="apps-card" id="card-${app.id}" style="background:var(--surface-1);border:1px solid var(--border-subtle);border-radius:16px;overflow:hidden">

                <!-- Header de la tarjeta -->
                <div style="background:var(--surface-0);padding:18px 20px;border-bottom:1px solid var(--border-subtle);display:flex;align-items:center;gap:14px">
                  <div style="font-size:36px;line-height:1">${app.icon}</div>
                  <div style="flex:1;min-width:0">
                    <div style="font-size:15px;font-weight:700;color:var(--text-primary)">${_esc(app.name)}</div>
                    <div style="font-size:11px;color:${app.color};font-weight:500;margin-top:2px">${_esc(app.role)}</div>
                    <div style="font-size:11px;color:var(--text-muted);margin-top:4px">${_esc(app.desc)}</div>
                  </div>
                  <div style="display:flex;flex-direction:column;align-items:flex-end;gap:6px;flex-shrink:0">
                    <div id="status-dot-${app.id}" style="display:flex;align-items:center;gap:5px;font-size:11px;color:var(--text-muted)">
                      <div style="width:8px;height:8px;border-radius:50%;background:var(--text-muted)"></div>
                      Verificando...
                    </div>
                    <div id="latency-${app.id}" style="font-size:10px;color:var(--text-muted)"></div>
                  </div>
                </div>

                <!-- KPIs -->
                <div style="display:grid;grid-template-columns:repeat(3,1fr);gap:1px;background:var(--border-subtle)">
                  ${app.metrics.map(m => `
                    <div style="background:var(--surface-1);padding:12px 14px">
                      <div style="font-size:10px;color:var(--text-muted);margin-bottom:4px">${_esc(m.label)}</div>
                      <div id="metric-${app.id}-${m.key}" style="font-size:18px;font-weight:700;color:var(--text-primary)">--</div>
                    </div>
                  `).join('')}
                </div>

                <!-- Info técnica -->
                <div style="padding:12px 20px;display:flex;justify-content:space-between;align-items:center;border-top:1px solid var(--border-subtle)">
                  <div style="font-size:11px;color:var(--text-muted)">
                    Puerto: <code style="color:${app.color};font-size:11px">:${app.port}</code>
                    &nbsp;·&nbsp;
                    Health: <code style="font-size:10px;opacity:.7">:${app.port}/api/health</code>
                  </div>
                  <div id="uptime-${app.id}" style="font-size:11px;color:var(--text-muted)"></div>
                </div>

                <!-- Acciones -->
                <div style="padding:14px 20px 18px;display:flex;gap:8px;flex-wrap:wrap">
                  <button class="action-btn primary" data-open="${app.dashboard}" style="flex:1;justify-content:center">
                    <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M18 13v6a2 2 0 01-2 2H5a2 2 0 01-2-2V8a2 2 0 012-2h6"/><polyline points="15 3 21 3 21 9"/><line x1="10" y1="14" x2="21" y2="3"/></svg>
                    Abrir Dashboard
                  </button>
                  <button class="action-btn" data-refresh="${app.id}" style="min-width:36px;padding:0 10px" title="Actualizar estado">
                    <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="23 4 23 10 17 10"/><path d="M20.49 15a9 9 0 11-2.12-9.36L23 10"/></svg>
                  </button>
                  <button class="action-btn" data-proxy="${app.id}" style="min-width:36px;padding:0 10px" title="Ver via proxy ATLAS">
                    <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="3"/><path d="M19.07 4.93a10 10 0 010 14.14M4.93 4.93a10 10 0 000 14.14"/></svg>
                  </button>
                </div>

                <!-- Raw JSON colapsable -->
                <details style="border-top:1px solid var(--border-subtle)">
                  <summary style="padding:10px 20px;font-size:11px;color:var(--text-muted);cursor:pointer;list-style:none;display:flex;align-items:center;gap:6px">
                    <svg width="11" height="11" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="9 18 15 12 9 6"/></svg>
                    Respuesta health raw
                  </summary>
                  <pre id="raw-${app.id}" style="margin:0;padding:12px 20px 16px;font-size:10px;color:var(--text-muted);overflow-x:auto;max-height:180px;overflow-y:auto;background:var(--surface-0)"></pre>
                </details>
              </div>
            `).join('')}
          </div>

          <!-- Panel de comunicación ATLAS -->
          <div style="margin-top:28px">
            <div class="section-title">Comunicación vía ATLAS</div>
            <div style="display:grid;grid-template-columns:repeat(auto-fill,minmax(200px,1fr));gap:10px;margin-top:10px">
              <div class="stat-card" style="cursor:pointer" onclick="location.hash='/vision'">
                <div class="stat-card-label">Módulo Visión</div>
                <div class="stat-card-value" style="font-size:13px">
                  <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" style="vertical-align:middle;margin-right:4px"><path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z"/><circle cx="12" cy="12" r="3"/></svg>
                  → /vision
                </div>
              </div>
              <div class="stat-card" style="cursor:pointer" onclick="location.hash='/autonomy'">
                <div class="stat-card-label">Cortex Orchestrator</div>
                <div class="stat-card-value" style="font-size:13px">
                  <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" style="vertical-align:middle;margin-right:4px"><rect x="4" y="4" width="16" height="16" rx="2"/><rect x="9" y="9" width="6" height="6"/></svg>
                  → /autonomy
                </div>
              </div>
              <div class="stat-card" style="cursor:pointer" onclick="location.hash='/modules'">
                <div class="stat-card-label">Estado Brazos (Cuerpo)</div>
                <div class="stat-card-value" style="font-size:13px">
                  <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" style="vertical-align:middle;margin-right:4px"><rect x="3" y="3" width="7" height="7"/><rect x="14" y="3" width="7" height="7"/><rect x="3" y="14" width="7" height="7"/><rect x="14" y="14" width="7" height="7"/></svg>
                  → /modules
                </div>
              </div>
              <div class="stat-card" style="cursor:pointer" onclick="location.hash='/chat'">
                <div class="stat-card-label">Enviar comando al cerebro</div>
                <div class="stat-card-value" style="font-size:13px">
                  <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" style="vertical-align:middle;margin-right:4px"><path d="M21 15a2 2 0 01-2 2H7l-4 4V5a2 2 0 012-2h14a2 2 0 012 2z"/></svg>
                  → /chat
                </div>
              </div>
            </div>
          </div>

        </div>
      </div>
    `;

    container.querySelector('#apps-back')?.addEventListener('click', () => { location.hash = '/'; });

    // Botones abrir dashboard
    container.querySelectorAll('[data-open]').forEach(btn => {
      btn.addEventListener('click', () => window.open(btn.dataset.open, '_blank'));
    });

    // Botones refresh manual
    container.querySelectorAll('[data-refresh]').forEach(btn => {
      btn.addEventListener('click', () => _checkHealth(btn.dataset.refresh, container));
    });

    // Botones proxy (abre ruta relativa del brazo via ATLAS si existe)
    container.querySelectorAll('[data-proxy]').forEach(btn => {
      btn.addEventListener('click', () => {
        const id = btn.dataset.proxy;
        const app = APPS.find(a => a.id === id);
        if (app) window.open(app.dashboard, '_blank');
      });
    });

    // Carga inicial y polling para ambos brazos
    APPS.forEach(app => {
      _checkHealth(app.id, container);
      poll(app.pollId, null, 20000, () => _checkHealth(app.id, container));
    });
  },

  destroy() {
    APPS.forEach(app => stop(app.pollId));
  },
};

/* ─── Health check ────────────────────────────────────────────────────── */

async function _checkHealth(appId, container) {
  const app = APPS.find(a => a.id === appId);
  if (!app) return;

  const dotEl    = container.querySelector(`#status-dot-${appId}`);
  const latEl    = container.querySelector(`#latency-${appId}`);
  const rawEl    = container.querySelector(`#raw-${appId}`);
  const uptimeEl = container.querySelector(`#uptime-${appId}`);

  const t0 = performance.now();
  try {
    const r  = await fetch(app.health, { signal: AbortSignal.timeout(5000) });
    const ms = Math.round(performance.now() - t0);
    const d  = await r.json().catch(() => ({}));

    const ok = r.ok && (d.ok !== false);

    if (dotEl) dotEl.innerHTML = `
      <div style="width:8px;height:8px;border-radius:50%;background:${ok ? 'var(--accent-green)' : 'var(--accent-red)'}"></div>
      <span style="color:${ok ? 'var(--accent-green)' : 'var(--accent-red)'};font-weight:500">${ok ? 'Online' : 'Offline'}</span>`;

    if (latEl) latEl.textContent = `${ms} ms`;

    // Fill metrics
    app.metrics.forEach(m => {
      const el = container.querySelector(`#metric-${appId}-${m.key}`);
      if (!el) return;
      let val = d;
      for (const p of m.path) { val = val?.[p]; if (val === undefined) break; }
      el.textContent = val !== undefined && val !== null ? String(val) : '--';
      el.style.color = val !== undefined && val !== null ? 'var(--text-primary)' : 'var(--text-muted)';
    });

    // Uptime
    const up = d.uptime || d.data?.uptime;
    if (uptimeEl && up) uptimeEl.textContent = `Uptime: ${up}`;

    if (rawEl) rawEl.textContent = JSON.stringify(d, null, 2);

  } catch (e) {
    const ms = Math.round(performance.now() - t0);
    if (dotEl) dotEl.innerHTML = `
      <div style="width:8px;height:8px;border-radius:50%;background:var(--accent-red)"></div>
      <span style="color:var(--accent-red);font-weight:500">Sin respuesta</span>`;
    if (latEl) latEl.textContent = `${ms} ms`;
    if (rawEl) rawEl.textContent = e.message;
    app.metrics.forEach(m => {
      const el = container.querySelector(`#metric-${appId}-${m.key}`);
      if (el) { el.textContent = '--'; el.style.color = 'var(--text-muted)'; }
    });
  }
}

window.AtlasModuleApps = { id: 'apps' };
