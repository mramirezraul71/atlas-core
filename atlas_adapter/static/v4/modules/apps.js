/**
 * ATLAS v4.2 — Mis Apps Module
 * Panel de Brazos Subordinados con vista embebida y gobierno desde ATLAS.
 * Split-pane: sidebar de control (izq) + iframe del dashboard (der).
 */
import { poll, stop } from '../lib/polling.js';

const POLL_VISION    = 'apps-vision-health';
const POLL_PANADERIA = 'apps-panaderia-health';

// URLs configurables — frontend (React/Vite) y API backend son puertos distintos
// Panadería: frontend en :5173 (Vite dev), API en :3001
// Vision:    frontend en :5173/:5174 (Vite dev) o :3000 (cliente-local con static)
const APP_DEFAULTS = {
  vision:    { frontendBase: 'http://127.0.0.1:5174', apiBase: 'http://127.0.0.1:3000' },
  panaderia: { frontendBase: 'http://127.0.0.1:5173', apiBase: 'http://127.0.0.1:3001' },
};

// Carga URLs guardadas en localStorage
function _loadSavedUrls(appId) {
  try { return JSON.parse(localStorage.getItem(`atlas-apps-url-${appId}`) || 'null') || APP_DEFAULTS[appId]; }
  catch { return APP_DEFAULTS[appId]; }
}

function _buildApps() {
  const v = _loadSavedUrls('vision');
  const p = _loadSavedUrls('panaderia');
  return {
    vision: {
      id: 'vision', name: 'Rauli Vision', shortName: 'Vision',
      role: 'Brazo Sensorial', icon: '👁️', color: 'var(--accent-primary)',
      base: v.frontendBase, health: `${v.apiBase}/api/health`, pollId: POLL_VISION,
      startInstructions: [
        { label: 'Paso 1 — Iniciar proxy API', cmd: 'cd _external/RAULI-VISION/cliente-local && python simple-server.py' },
        { label: 'Paso 2 — Iniciar dashboard', cmd: 'cd _external/RAULI-VISION/dashboard && npm run dev' },
        { label: 'Resultado', cmd: `Frontend → ${v.frontendBase} · API → ${v.apiBase}` },
      ],
      navLinks: [
        { label: 'Inicio',         path: '/' },
        { label: 'Búsqueda Web',   path: '/?tab=search' },
        { label: 'Búsqueda Video', path: '/?tab=video' },
        { label: 'Chat IA',        path: '/?tab=chat' },
      ],
      govEndpoints: [
        { label: 'Health / proxy',  url: `${v.apiBase}/api/health`,       method: 'GET', id: 'vision-health' },
        { label: 'Historial chat',  url: `${v.apiBase}/api/chat/history`,  method: 'GET', id: 'vision-chat' },
      ],
    },
    panaderia: {
      id: 'panaderia', name: 'Rauli Panadería', shortName: 'Panadería',
      role: 'Brazo Operativo', icon: '🥖', color: 'var(--accent-green)',
      base: p.frontendBase, health: `${p.apiBase}/api/health`, pollId: POLL_PANADERIA,
      startInstructions: [
        { label: 'Paso 1 — Iniciar API backend', cmd: 'cd _external/rauli-panaderia/backend && node server.js' },
        { label: 'Paso 2 — Iniciar frontend',    cmd: 'cd _external/rauli-panaderia/frontend && npm run dev' },
        { label: 'Resultado', cmd: `Frontend → ${p.frontendBase} · API → ${p.apiBase}` },
      ],
      navLinks: [
        { label: 'Dashboard',    path: '/' },
        { label: 'POS / Caja',   path: '/pos' },
        { label: 'Ventas',       path: '/sales' },
        { label: 'Inventario',   path: '/inventory' },
        { label: 'Producción',   path: '/produccion' },
        { label: 'Caja',         path: '/cash' },
        { label: 'Reportes',     path: '/reports' },
        { label: 'Empleados',    path: '/employees' },
        { label: 'Contabilidad', path: '/accounting' },
        { label: 'Gerencia',     path: '/gerencia' },
        { label: 'Gastos',       path: '/expenses' },
        { label: 'Configuración',path: '/settings' },
      ],
      govEndpoints: [
        { label: 'Sentinel status',    url: `${p.apiBase}/api/sentinel/status`,                    method: 'GET', id: 'pan-sentinel' },
        { label: 'Alertas activas',    url: `${p.apiBase}/api/sentinel/alerts`,                    method: 'GET', id: 'pan-alerts' },
        { label: 'Métricas sentinel',  url: `${p.apiBase}/api/sentinel/metrics`,                   method: 'GET', id: 'pan-metrics' },
        { label: 'Ventas hoy',         url: `${p.apiBase}/api/sales/today`,                        method: 'GET', id: 'pan-sales' },
        { label: 'Inventario resumen', url: `${p.apiBase}/api/inventory/summary`,                  method: 'GET', id: 'pan-inv' },
        { label: 'Lotes por vencer',   url: `${p.apiBase}/api/inventory/lots/expiring`,            method: 'GET', id: 'pan-exp' },
        { label: 'Órdenes prod.',      url: `${p.apiBase}/api/production/production-orders`,       method: 'GET', id: 'pan-prod' },
      ],
    },
  };
}

let APPS = _buildApps();

function _esc(s) { const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML; }

let _activeApp   = 'panaderia';
let _iframeReady = false;

export default {
  id: 'apps',
  label: 'Mis Apps',
  icon: 'layers',

  render(container) {
    container.innerHTML = `
      <div class="module-view" style="height:100%;display:flex;flex-direction:column">

        <!-- HEADER -->
        <div class="module-header">
          <button class="back-btn" id="apps-back">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Dashboard
          </button>
          <h2>Mis Apps — Brazos</h2>
          <div style="margin-left:auto;display:flex;gap:8px;align-items:center">
            <!-- Status pills -->
            <div id="pill-vision"    class="live-badge" style="background:var(--surface-2);color:var(--text-muted);gap:5px;display:flex;align-items:center">
              <div id="dot-vision"   style="width:7px;height:7px;border-radius:50%;background:var(--text-muted)"></div>Vision
            </div>
            <div id="pill-panaderia" class="live-badge" style="background:var(--surface-2);color:var(--text-muted);gap:5px;display:flex;align-items:center">
              <div id="dot-panaderia" style="width:7px;height:7px;border-radius:50%;background:var(--text-muted)"></div>Panadería
            </div>
            <span class="live-badge">LIVE</span>
          </div>
        </div>

        <!-- TABS SELECTORES -->
        <div style="display:flex;gap:0;border-bottom:1px solid var(--border-subtle);flex-shrink:0;padding:0 20px">
          ${Object.values(APPS).map(a => `
            <button class="apps-tab" data-tab="${a.id}"
              style="padding:10px 20px;border:none;background:none;color:var(--text-muted);font-size:13px;font-weight:500;cursor:pointer;border-bottom:2px solid transparent;display:flex;align-items:center;gap:7px;transition:all .15s">
              <span>${a.icon}</span> ${_esc(a.name)}
              <span id="tab-status-${a.id}" style="font-size:10px;opacity:.6"></span>
            </button>
          `).join('')}
        </div>

        <!-- SPLIT PANE -->
        <div style="flex:1;display:flex;overflow:hidden;min-height:0">

          <!-- SIDEBAR DE GOBIERNO -->
          <div id="apps-sidebar"
               style="width:280px;flex-shrink:0;border-right:1px solid var(--border-subtle);overflow-y:auto;display:flex;flex-direction:column;background:var(--surface-0)">

            <!-- Bloque de estado rápido -->
            <div id="sidebar-status" style="padding:14px 16px;border-bottom:1px solid var(--border-subtle)">
              <div style="font-size:10px;color:var(--text-muted);letter-spacing:.05em;margin-bottom:8px">ESTADO</div>
              <div id="sidebar-health-row" style="font-size:12px;color:var(--text-muted)">Verificando...</div>
              <div id="sidebar-latency"    style="font-size:10px;color:var(--text-muted);margin-top:3px"></div>
              <div id="sidebar-uptime"     style="font-size:10px;color:var(--text-muted);margin-top:2px"></div>
            </div>

            <!-- Navegación dentro del iframe -->
            <div style="padding:12px 16px;border-bottom:1px solid var(--border-subtle)">
              <div style="font-size:10px;color:var(--text-muted);letter-spacing:.05em;margin-bottom:8px">NAVEGACIÓN</div>
              <div id="sidebar-nav" style="display:flex;flex-direction:column;gap:4px"></div>
            </div>

            <!-- Panel de gobierno / datos -->
            <div style="padding:12px 16px;flex:1">
              <div style="font-size:10px;color:var(--text-muted);letter-spacing:.05em;margin-bottom:8px">GOBIERNO</div>
              <div id="sidebar-gov" style="display:flex;flex-direction:column;gap:6px"></div>
            </div>

            <!-- Pie del sidebar -->
            <div style="padding:12px 16px;border-top:1px solid var(--border-subtle);display:flex;flex-direction:column;gap:6px">
              <button class="action-btn primary" id="btn-open-external" style="justify-content:center;width:100%">
                <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M18 13v6a2 2 0 01-2 2H5a2 2 0 01-2-2V8a2 2 0 012-2h6"/><polyline points="15 3 21 3 21 9"/><line x1="10" y1="14" x2="21" y2="3"/></svg>
                Abrir en pestaña
              </button>
              <button class="action-btn" id="btn-reload-iframe" style="justify-content:center;width:100%">
                <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="23 4 23 10 17 10"/><path d="M20.49 15a9 9 0 11-2.12-9.36L23 10"/></svg>
                Recargar frame
              </button>
              <!-- Config de URLs -->
              <details style="margin-top:4px">
                <summary style="font-size:10px;color:var(--text-muted);cursor:pointer;padding:4px 0;list-style:none;display:flex;align-items:center;gap:5px">
                  <svg width="10" height="10" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="3"/><path d="M12 1v4M12 19v4M4.22 4.22l2.83 2.83M16.95 16.95l2.83 2.83M1 12h4M19 12h4M4.22 19.78l2.83-2.83M16.95 7.05l2.83-2.83"/></svg>
                  Configurar URLs
                </summary>
                <div style="margin-top:8px;display:flex;flex-direction:column;gap:6px">
                  <div>
                    <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">Frontend (iframe)</div>
                    <input id="cfg-frontend-url" type="text" placeholder="http://127.0.0.1:5173"
                      style="width:100%;box-sizing:border-box;padding:6px 8px;font-size:11px;background:var(--surface-2);border:1px solid var(--border-subtle);border-radius:6px;color:var(--text-primary);font-family:monospace">
                  </div>
                  <div>
                    <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">API backend (gobierno)</div>
                    <input id="cfg-api-url" type="text" placeholder="http://127.0.0.1:3001"
                      style="width:100%;box-sizing:border-box;padding:6px 8px;font-size:11px;background:var(--surface-2);border:1px solid var(--border-subtle);border-radius:6px;color:var(--text-primary);font-family:monospace">
                  </div>
                  <button class="action-btn success" id="btn-save-urls" style="justify-content:center;width:100%;font-size:11px">
                    Guardar y aplicar
                  </button>
                </div>
              </details>
            </div>
          </div>

          <!-- IFRAME CONTAINER -->
          <div style="flex:1;position:relative;background:var(--surface-0);display:flex;flex-direction:column;min-width:0">
            <!-- URL bar -->
            <div id="iframe-urlbar"
                 style="padding:6px 12px;border-bottom:1px solid var(--border-subtle);font-size:11px;color:var(--text-muted);font-family:monospace;background:var(--surface-1);flex-shrink:0;display:flex;align-items:center;gap:8px;overflow:hidden">
              <svg width="11" height="11" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5"><circle cx="12" cy="12" r="10"/><path d="M12 8v4l3 3"/></svg>
              <span id="iframe-url-text" style="overflow:hidden;text-overflow:ellipsis;white-space:nowrap">Cargando...</span>
            </div>
            <!-- Overlay "app no iniciada" -->
            <div id="iframe-not-running"
                 style="display:none;position:absolute;inset:36px 0 0 0;background:var(--surface-0);z-index:10;align-items:center;justify-content:center;flex-direction:column;gap:12px;overflow-y:auto">
            </div>

            <!-- Aviso X-Frame-Options -->
            <div id="iframe-blocked-msg"
                 style="display:none;position:absolute;inset:40px 0 0 0;background:var(--surface-0);z-index:5;display:none;align-items:center;justify-content:center;flex-direction:column;gap:12px;text-align:center;padding:32px">
              <svg width="40" height="40" viewBox="0 0 24 24" fill="none" stroke="var(--accent-yellow,#f59e0b)" stroke-width="1.5"><circle cx="12" cy="12" r="10"/><line x1="12" y1="8" x2="12" y2="12"/><line x1="12" y1="16" x2="12.01" y2="16"/></svg>
              <div style="font-size:14px;font-weight:600;color:var(--text-primary)">Dashboard bloqueado por CORS / X-Frame-Options</div>
              <div style="font-size:12px;color:var(--text-muted);max-width:400px">
                El navegador impide mostrar esta app en un iframe desde otro origen.<br>
                Usa <b>Abrir en pestaña</b> para acceder al dashboard directamente.
              </div>
              <button class="action-btn primary" id="blocked-open-btn" style="margin-top:8px">
                <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M18 13v6a2 2 0 01-2 2H5a2 2 0 01-2-2V8a2 2 0 012-2h6"/><polyline points="15 3 21 3 21 9"/><line x1="10" y1="14" x2="21" y2="3"/></svg>
                Abrir en pestaña nueva
              </button>
            </div>
            <iframe id="apps-iframe"
                    style="flex:1;border:none;width:100%;height:100%"
                    title="Dashboard integrado"
                    sandbox="allow-scripts allow-same-origin allow-forms allow-popups allow-popups-to-escape-sandbox allow-top-navigation-by-user-activation">
            </iframe>
          </div>

        </div>
      </div>

      <style>
        .apps-tab { transition: color .15s, border-color .15s; }
        .apps-tab:hover { color: var(--text-primary); }
        .apps-tab.active { color: var(--accent-primary); border-bottom-color: var(--accent-primary) !important; }
        .apps-tab.active span:first-child { opacity:1; }
        .sidebar-nav-btn {
          width: 100%; text-align: left; padding: 7px 10px; border-radius: 7px;
          border: none; background: none; color: var(--text-muted); font-size: 12px;
          cursor: pointer; transition: background .12s, color .12s; display: flex; align-items: center; gap: 7px;
        }
        .sidebar-nav-btn:hover { background: var(--surface-2); color: var(--text-primary); }
        .sidebar-nav-btn.active { background: rgba(var(--accent-primary-rgb,0,200,200),.12); color: var(--accent-primary); }
        .gov-card {
          background: var(--surface-1); border: 1px solid var(--border-subtle);
          border-radius: 9px; padding: 10px 12px; cursor: pointer; transition: border-color .15s;
        }
        .gov-card:hover { border-color: var(--accent-primary); }
        .gov-card-label { font-size: 10px; color: var(--text-muted); margin-bottom: 4px; }
        .gov-card-value { font-size: 13px; font-weight: 600; color: var(--text-primary); word-break:break-all }
        .gov-card-value.ok    { color: var(--accent-green); }
        .gov-card-value.error { color: var(--accent-red); }
        .gov-card-value.warn  { color: #f59e0b; }
      </style>
    `;

    const iframe   = container.querySelector('#apps-iframe');
    const urlText  = container.querySelector('#iframe-url-text');
    const blockedMsg = container.querySelector('#iframe-blocked-msg');

    container.querySelector('#apps-back')?.addEventListener('click', () => { location.hash = '/'; });

    // Detectar si el iframe fue bloqueado
    iframe.addEventListener('load', () => {
      try {
        const url = iframe.contentWindow?.location?.href || '';
        if (urlText) urlText.textContent = url || iframe.src;
      } catch {
        // cross-origin load — el brazo está vivo pero bloqueó iframe
        if (urlText) urlText.textContent = iframe.src;
      }
    });

    // Botones de pie del sidebar
    container.querySelector('#btn-open-external')?.addEventListener('click', () => {
      const app = APPS[_activeApp];
      if (app) window.open(iframe.src || app.base, '_blank');
    });
    container.querySelector('#btn-reload-iframe')?.addEventListener('click', () => {
      if (iframe.src) { const s = iframe.src; iframe.src = ''; iframe.src = s; }
    });
    container.querySelector('#blocked-open-btn')?.addEventListener('click', () => {
      const app = APPS[_activeApp]; if (app) window.open(app.base, '_blank');
    });

    // Config de URLs — poblar inputs y guardar
    function _updateCfgInputs() {
      const app = APPS[_activeApp];
      if (!app) return;
      const saved = _loadSavedUrls(_activeApp);
      const fe = container.querySelector('#cfg-frontend-url');
      const api = container.querySelector('#cfg-api-url');
      if (fe)  fe.value  = saved.frontendBase;
      if (api) api.value = saved.apiBase;
    }
    _updateCfgInputs();

    container.querySelector('#btn-save-urls')?.addEventListener('click', () => {
      const fe  = container.querySelector('#cfg-frontend-url')?.value?.trim();
      const api = container.querySelector('#cfg-api-url')?.value?.trim();
      if (!fe || !api) return;
      localStorage.setItem(`atlas-apps-url-${_activeApp}`, JSON.stringify({ frontendBase: fe, apiBase: api }));
      APPS = _buildApps();
      _switchApp(_activeApp, container, iframe, urlText);
      window.AtlasToast?.show('URLs guardadas', 'success');
    });

    // Tabs
    container.querySelectorAll('.apps-tab').forEach(tab => {
      tab.addEventListener('click', () => {
        _switchApp(tab.dataset.tab, container, iframe, urlText);
        // Actualizar inputs de config para el nuevo brazo
        setTimeout(_updateCfgInputs, 0);
      });
    });

    // Polling de health para los dos brazos
    Object.values(APPS).forEach(app => {
      _checkHealth(app, container);
      poll(app.pollId, null, 20000, () => _checkHealth(app, container));
    });

    // Cargar app activa por defecto
    _switchApp(_activeApp, container, iframe, urlText);
  },

  destroy() {
    Object.values(APPS).forEach(app => stop(app.pollId));
  },
};

/* ─── Cambiar app activa ─────────────────────────────────────────────── */

function _switchApp(appId, container, iframe, urlText) {
  const app = APPS[appId];
  if (!app) return;
  _activeApp = appId;

  // Tabs UI
  container.querySelectorAll('.apps-tab').forEach(t => {
    t.classList.toggle('active', t.dataset.tab === appId);
  });

  // Sidebar: Navegación
  const navEl = container.querySelector('#sidebar-nav');
  if (navEl) {
    navEl.innerHTML = app.navLinks.map((l, i) => `
      <button class="sidebar-nav-btn${i === 0 ? ' active' : ''}" data-nav-path="${_esc(l.path)}">
        <svg width="11" height="11" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="9 18 15 12 9 6"/></svg>
        ${_esc(l.label)}
      </button>
    `).join('');

    navEl.querySelectorAll('.sidebar-nav-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        navEl.querySelectorAll('.sidebar-nav-btn').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        const url = app.base + btn.dataset.navPath;
        iframe.src = url;
        if (urlText) urlText.textContent = url;
      });
    });
  }

  // Sidebar: Gobierno (gov endpoints)
  const govEl = container.querySelector('#sidebar-gov');
  if (govEl) {
    govEl.innerHTML = app.govEndpoints.map(ep => `
      <div class="gov-card" data-ep-id="${ep.id}">
        <div class="gov-card-label">${_esc(ep.label)}</div>
        <div class="gov-card-value" id="gov-val-${ep.id}">
          <span style="opacity:.5;font-size:11px">Cargando...</span>
        </div>
      </div>
    `).join('');

    // Click en gov card → navega el iframe a la sección correspondiente
    govEl.querySelectorAll('.gov-card').forEach(card => {
      card.addEventListener('click', () => {
        const ep = app.govEndpoints.find(e => e.id === card.dataset.epId);
        if (!ep) return;
        // Navegar iframe a la sección relevante
        const sectionMap = {
          'pan-sentinel': '/sentinel',
          'pan-alerts':   '/sentinel',
          'pan-metrics':  '/sentinel',
          'pan-sales':    '/sales',
          'pan-inv':      '/inventory',
          'pan-exp':      '/inventory/lots',
          'pan-prod':     '/production',
          'vision-health':'/',
          'vision-chat':  '/?tab=chat',
        };
        const path = sectionMap[ep.id] || '/';
        const url  = app.base + path;
        iframe.src = url;
        if (urlText) urlText.textContent = url;
        if (navEl) navEl.querySelectorAll('.sidebar-nav-btn').forEach(b => b.classList.remove('active'));
      });
    });

    // Fetch gov endpoint data
    app.govEndpoints.forEach(ep => _fetchGovEndpoint(ep, container));
  }

  // Pre-check: verificar que el frontend responde antes de cargar iframe
  const url = app.base + '/';
  if (urlText) urlText.textContent = url;
  _preCheckAndLoad(app, url, container, iframe, urlText);
}

/* ─── Pre-check iframe + overlay de instrucciones ───────────────────── */

async function _preCheckAndLoad(app, url, container, iframe, urlText) {
  const overlay = container.querySelector('#iframe-not-running');

  // Mostrar spinner mientras verificamos
  if (overlay) {
    overlay.style.display = 'flex';
    overlay.innerHTML = `
      <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="var(--text-muted)" stroke-width="1.5"
           style="animation:spin 1s linear infinite">
        <path d="M21 12a9 9 0 11-6.219-8.56"/>
      </svg>
      <div style="font-size:13px;color:var(--text-muted)">Conectando con ${_esc(app.name)}...</div>
      <style>@keyframes spin{to{transform:rotate(360deg)}}</style>`;
  }

  let reachable = false;
  try {
    const r = await fetch(url, { method: 'HEAD', signal: AbortSignal.timeout(4000), mode: 'no-cors' });
    reachable = true; // no-cors: si no lanza excepción, el servidor respondió
  } catch {
    reachable = false;
  }

  if (reachable) {
    // Ocultar overlay y cargar iframe
    if (overlay) overlay.style.display = 'none';
    iframe.src = url;
    if (urlText) urlText.textContent = url;
  } else {
    // Mostrar overlay con instrucciones de inicio
    if (overlay) {
      const steps = (app.startInstructions || []).map(s => `
        <div style="margin-bottom:10px">
          <div style="font-size:10px;color:var(--text-muted);margin-bottom:3px">${_esc(s.label)}</div>
          <code style="display:block;background:var(--surface-0);border:1px solid var(--border-subtle);border-radius:6px;padding:8px 10px;font-size:11px;color:var(--accent-primary);word-break:break-all">${_esc(s.cmd)}</code>
        </div>`).join('');

      overlay.innerHTML = `
        <div style="text-align:center;max-width:480px;padding:16px">
          <div style="font-size:32px;margin-bottom:12px">${app.icon}</div>
          <div style="font-size:15px;font-weight:700;color:var(--text-primary);margin-bottom:6px">
            ${_esc(app.name)} no está iniciado
          </div>
          <div style="font-size:12px;color:var(--text-muted);margin-bottom:20px">
            El dashboard no responde en <code style="color:var(--accent-primary)">${_esc(url)}</code><br>
            Inicia la app y luego recarga el frame, o ajusta el URL en "Configurar URLs".
          </div>
          ${steps ? `
            <div style="text-align:left;margin-bottom:16px">
              <div style="font-size:10px;color:var(--text-muted);letter-spacing:.05em;margin-bottom:10px">CÓMO INICIAR</div>
              ${steps}
            </div>` : ''}
          <div style="display:flex;gap:8px;justify-content:center;flex-wrap:wrap">
            <button class="action-btn primary" id="overlay-retry-btn">
              <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="23 4 23 10 17 10"/><path d="M20.49 15a9 9 0 11-2.12-9.36L23 10"/></svg>
              Reintentar
            </button>
            <button class="action-btn" id="overlay-force-btn">
              Cargar de todos modos
            </button>
          </div>
        </div>`;

      overlay.querySelector('#overlay-retry-btn')?.addEventListener('click', () => {
        _preCheckAndLoad(app, url, container, iframe, urlText);
      });
      overlay.querySelector('#overlay-force-btn')?.addEventListener('click', () => {
        overlay.style.display = 'none';
        iframe.src = url;
        if (urlText) urlText.textContent = url;
      });
    }
    // No cargar el iframe (evita la página fea del navegador)
    iframe.src = 'about:blank';
  }
}

/* ─── Fetch gobierno endpoint ────────────────────────────────────────── */

async function _fetchGovEndpoint(ep, container) {
  const el = container.querySelector(`#gov-val-${ep.id}`);
  if (!el) return;
  try {
    const r = await fetch(ep.url, { method: ep.method, signal: AbortSignal.timeout(5000) });
    const d = await r.json().catch(() => null);

    if (r.status === 401 || r.status === 403) {
      el.innerHTML = `<span style="font-size:11px;color:var(--accent-yellow,#f59e0b)">⚠ Iniciar sesión en el brazo</span>`;
      return;
    }

    if (!r.ok) {
      el.innerHTML = `<span class="gov-card-value error" style="font-size:11px">Error ${r.status}</span>`;
      return;
    }

    const summary = _summarizeGovData(ep.id, d);
    el.innerHTML = summary;

  } catch (e) {
    el.innerHTML = `<span style="font-size:10px;color:var(--text-muted)">Sin respuesta</span>`;
  }
}

function _summarizeGovData(id, d) {
  function esc(s) { const el = document.createElement('span'); el.textContent = String(s ?? ''); return el.innerHTML; }

  if (!d) return '<span style="font-size:11px;color:var(--text-muted)">Sin datos</span>';

  if (id === 'pan-sentinel') {
    const st = d?.status || d?.data?.status || d?.data?.sentinel_status || 'unknown';
    const cls = st === 'ok' ? 'ok' : st === 'warn' ? 'warn' : 'error';
    return `<span class="gov-card-value ${cls}">${esc(st.toUpperCase())}</span>`;
  }
  if (id === 'pan-alerts') {
    const arr = d?.alerts || d?.data?.alerts || d?.items || (Array.isArray(d) ? d : []);
    const count = arr.length;
    const cls = count === 0 ? 'ok' : count < 3 ? 'warn' : 'error';
    return `<span class="gov-card-value ${cls}">${esc(count)} alerta${count !== 1 ? 's' : ''}</span>
      ${arr.slice(0,2).map(a => `<div style="font-size:10px;color:var(--text-muted);margin-top:2px">${esc(a.message || a.type || a.id || '')}</div>`).join('')}`;
  }
  if (id === 'pan-metrics') {
    const data = d?.data || d;
    const keys = Object.keys(data).slice(0, 3);
    return keys.map(k => `<div style="font-size:10px;margin-top:1px"><span style="color:var(--text-muted)">${esc(k)}:</span> ${esc(data[k])}</div>`).join('') || '<span style="font-size:11px;color:var(--text-muted)">Sin métricas</span>';
  }
  if (id === 'pan-sales') {
    const total = d?.total || d?.data?.total || d?.data?.amount || d?.amount;
    const count = d?.count || d?.data?.count || d?.data?.orders;
    return `<span class="gov-card-value ok">${total !== undefined ? `$${esc(total)}` : '--'}</span>
      ${count !== undefined ? `<div style="font-size:10px;color:var(--text-muted);margin-top:2px">${esc(count)} pedidos</div>` : ''}`;
  }
  if (id === 'pan-inv') {
    const items = d?.items || d?.data?.items || d?.total_products;
    const val   = d?.total_value || d?.data?.total_value;
    return `<span class="gov-card-value">${items !== undefined ? `${esc(items)} items` : '--'}</span>
      ${val !== undefined ? `<div style="font-size:10px;color:var(--text-muted);margin-top:2px">Valor: $${esc(val)}</div>` : ''}`;
  }
  if (id === 'pan-exp') {
    const arr = d?.lots || d?.data?.lots || d?.items || (Array.isArray(d) ? d : []);
    const cls = arr.length === 0 ? 'ok' : 'warn';
    return `<span class="gov-card-value ${cls}">${esc(arr.length)} lote${arr.length !== 1 ? 's' : ''}</span>
      ${arr.slice(0,2).map(l => `<div style="font-size:10px;color:var(--text-muted);margin-top:2px">${esc(l.product_name || l.name || l.id || '')}</div>`).join('')}`;
  }
  if (id === 'pan-prod') {
    const arr = d?.orders || d?.data?.orders || d?.items || (Array.isArray(d) ? d : []);
    const pending = arr.filter(o => o.status === 'pending' || o.status === 'in_progress').length;
    return `<span class="gov-card-value">${esc(arr.length)} total</span>
      <div style="font-size:10px;color:var(--text-muted);margin-top:2px">${esc(pending)} en proceso</div>`;
  }
  if (id === 'vision-health') {
    const proxy  = d?.proxy  || d?.status || 'ok';
    const espejo = d?.espejo || 'unknown';
    const cache  = d?.cache_entries ?? d?.cache ?? '--';
    return `<span class="gov-card-value ok">Proxy: ${esc(proxy)}</span>
      <div style="font-size:10px;color:var(--text-muted);margin-top:2px">Espejo: ${esc(espejo)}</div>
      <div style="font-size:10px;color:var(--text-muted)">Caché: ${esc(cache)} entradas</div>`;
  }
  if (id === 'vision-chat') {
    const arr = d?.items || d?.messages || (Array.isArray(d) ? d : []);
    return `<span class="gov-card-value">${esc(arr.length)} mensajes</span>
      ${arr.slice(0,2).map(m => `<div style="font-size:10px;color:var(--text-muted);margin-top:2px">[${esc(m.role || '?')}] ${esc((m.preview || m.content || '').slice(0,40))}</div>`).join('')}`;
  }
  // fallback genérico
  return `<span style="font-size:10px;color:var(--text-muted)">${JSON.stringify(d).slice(0,60)}</span>`;
}

/* ─── Health check (pills de header + sidebar status) ───────────────── */

async function _checkHealth(app, container) {
  const dot  = container.querySelector(`#dot-${app.id}`);
  const pill = container.querySelector(`#pill-${app.id}`);
  const tabSt= container.querySelector(`#tab-status-${app.id}`);

  const t0 = performance.now();
  try {
    const r  = await fetch(app.health, { signal: AbortSignal.timeout(5000) });
    const ms = Math.round(performance.now() - t0);
    const d  = await r.json().catch(() => ({}));
    const ok = r.ok && (d.ok !== false) && (d.status !== 'error');

    const color = ok ? 'var(--accent-green)' : 'var(--accent-red)';
    if (dot)  dot.style.background = color;
    if (pill) { pill.style.background = ok ? 'rgba(0,200,100,.15)' : 'rgba(255,60,60,.15)'; pill.style.color = color; }
    if (tabSt) tabSt.textContent = ok ? `${ms}ms` : 'Offline';

    // Si es el brazo activo, actualizar sidebar status
    if (app.id === _activeApp) {
      const healthRow = container.querySelector('#sidebar-health-row');
      const latRow    = container.querySelector('#sidebar-latency');
      const uptRow    = container.querySelector('#sidebar-uptime');
      if (healthRow) healthRow.innerHTML = `<span style="color:${color};font-weight:600">${ok ? '● Online' : '● Offline'}</span>`;
      if (latRow)    latRow.textContent = `Latencia: ${ms} ms`;
      if (uptRow)    uptRow.textContent = d.uptime ? `Uptime: ${d.uptime}` : '';
    }

  } catch {
    const ms = Math.round(performance.now() - t0);
    if (dot)  dot.style.background = 'var(--accent-red)';
    if (pill) { pill.style.background = 'rgba(255,60,60,.15)'; pill.style.color = 'var(--accent-red)'; }
    if (tabSt) tabSt.textContent = 'Sin respuesta';
    if (app.id === _activeApp) {
      const healthRow = container.querySelector('#sidebar-health-row');
      if (healthRow) healthRow.innerHTML = `<span style="color:var(--accent-red);font-weight:600">● Sin respuesta</span>`;
    }
  }
}

window.AtlasModuleApps = { id: 'apps' };
