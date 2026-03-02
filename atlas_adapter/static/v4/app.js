/**
 * ATLAS v4 — App Shell
 * Orchestrates router, topbar, mega menu, theme, and module registry.
 */
const THEMES = ['cyan', 'purple', 'green', 'blue', 'orange', 'pink', 'red', 'gold'];

const SVG = {
  menu: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M3 12h18M3 6h18M3 18h18"/></svg>',
  home: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M3 9l9-7 9 7v11a2 2 0 01-2 2H5a2 2 0 01-2-2z"/></svg>',
  theme: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="5"/><path d="M12 1v2M12 21v2M4.22 4.22l1.42 1.42M18.36 18.36l1.42 1.42M1 12h2M21 12h2M4.22 19.78l1.42-1.42M18.36 5.64l1.42-1.42"/></svg>',
  logo: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><polygon points="12 2 22 20 2 20"/><line x1="12" y1="8" x2="12" y2="14"/><circle cx="12" cy="17" r="0.5" fill="currentColor"/></svg>',
};

const MODULE_REGISTRY = {};
let _themeIdx = 0;
let _currentCleanup = null;

function _esc(s) { const d = document.createElement('span'); d.textContent = s || ''; return d.innerHTML; }

function _applyTheme(theme) {
  document.documentElement.setAttribute('data-theme', theme);
  localStorage.setItem('atlas-theme', theme);
  window.AtlasState?.set('theme', theme);
}

function _cycleTheme() {
  _themeIdx = (_themeIdx + 1) % THEMES.length;
  _applyTheme(THEMES[_themeIdx]);
}

function _buildTopbar(app) {
  const bar = document.createElement('header');
  bar.className = 'topbar';
  bar.innerHTML = `
    <div class="topbar-logo" id="topbar-logo">
      ${SVG.logo}
      <span>ATLAS</span>
      <div class="topbar-health-dot" id="topbar-dot"></div>
    </div>
    <div class="topbar-center"></div>
    <div class="topbar-right">
      <span class="topbar-version" id="topbar-version">v4.2.0</span>
      <button class="topbar-btn" id="btn-theme" title="Cambiar tema">${SVG.theme}</button>
      <button class="topbar-btn" id="btn-home" title="Inicio">${SVG.home}</button>
      <button class="topbar-btn" id="btn-menu" title="Menú" aria-label="Menú principal">${SVG.menu}</button>
    </div>
  `;
  app.appendChild(bar);

  const btnMenu = bar.querySelector('#btn-menu');
  btnMenu.addEventListener('click', (e) => {
    e.stopPropagation();
    window.AtlasMegaMenu?.toggle?.();
  });

  bar.querySelector('#btn-home').addEventListener('click', () => { location.hash = '/'; });
  bar.querySelector('#btn-theme').addEventListener('click', _cycleTheme);
  bar.querySelector('#topbar-logo').addEventListener('click', () => { location.hash = '/'; });
}

function _openUpdatePanel() {
  const overlay = document.createElement('div');
  overlay.className = 'update-panel-overlay';
  overlay.innerHTML = `
    <div class="update-panel">
      <button type="button" class="update-panel-close" aria-label="Cerrar">&times;</button>
      <h2>Actualizar ATLAS</h2>
      <p class="subtitle">Comprobar prerequisitos, estado del repo y aplicar actualizaciones desde el remoto. Si «Aplicar» falla por política, configura <code>POLICY_ALLOW_UPDATE_APPLY=true</code> en <code>config/atlas.env</code> y reinicia el servidor.</p>
      <div class="update-panel-section">
        <div class="update-panel-section-title">Software</div>
        <div id="update-prereqs"></div>
      </div>
      <div class="update-panel-section">
        <div class="update-panel-section-title">Repositorio</div>
        <div id="update-repo"></div>
      </div>
      <div class="update-panel-actions">
        <button type="button" class="update-panel-btn" id="update-btn-refresh">Comprobar</button>
        <button type="button" class="update-panel-btn primary" id="update-btn-apply">Aplicar actualización</button>
        <button type="button" class="update-panel-btn" id="update-btn-restart" title="Reinicia el servidor ATLAS para cargar cambios (config, código).">Reiniciar servidor</button>
      </div>
      <div class="update-panel-log" id="update-log"></div>
    </div>
  `;
  document.body.appendChild(overlay);
  const logEl = overlay.querySelector('#update-log');
  const prereqsEl = overlay.querySelector('#update-prereqs');
  const repoEl = overlay.querySelector('#update-repo');

  function log(msg, isError = false) {
    const line = (typeof msg === 'string' ? msg : JSON.stringify(msg)) + '\n';
    logEl.textContent = (logEl.textContent || '') + line;
    logEl.scrollTop = logEl.scrollHeight;
  }

  async function loadPrereqs() {
    try {
      let r = await fetch('/update/prereqs');
      if (!r.ok) r = await fetch('/api/v4/update/prereqs');
      const j = await r.json();
      const d = j.data || {};
      if (!r.ok) {
        prereqsEl.innerHTML = '<div class="update-panel-row"><span class="status-dot error"></span> Error API: ' + _esc(j.error || r.status) + '</div>';
        return;
      }
      prereqsEl.innerHTML = `
        <div class="update-panel-row"><span class="status-dot ${d.git_ok ? 'ok' : 'error'}"></span> Git: ${d.git_ok ? (d.git_version || 'OK') : 'No detectado'}</div>
        <div class="update-panel-row"><span class="status-dot ${d.python_ok ? 'ok' : 'error'}"></span> Python: ${d.python_ok ? (d.python_version || 'OK') : 'No detectado'}</div>
        <div class="update-panel-row"><span class="status-dot ${d.pip_ok ? 'ok' : 'error'}"></span> pip: ${d.pip_ok ? (d.pip_version || 'OK') : 'No detectado'}</div>
        <div class="update-panel-row"><span class="status-dot ${d.repo_has_changes ? 'warn' : 'ok'}"></span> Repo: ${_esc(d.branch || '?')} ${d.repo_has_changes ? '(cambios locales)' : 'limpio'}</div>
      `;
    } catch (e) {
      prereqsEl.innerHTML = '<div class="update-panel-row"><span class="status-dot error"></span> Error: ' + _esc(String(e)) + '. Comprueba que el servidor esté en marcha y recarga la página.</div>';
    }
  }

  async function loadRepo() {
    try {
      const r = await fetch('/update/status');
      const j = await r.json();
      const d = (j.data || j) || {};
      repoEl.innerHTML = `
        <div class="update-panel-row"><span class="status-dot ok"></span> Rama: ${_esc(d.branch || '—')}</div>
        <div class="update-panel-row">Commit actual: ${_esc((d.head_commit || '').slice(0, 8) || '—')}</div>
        <div class="update-panel-row">Remoto: ${_esc((d.remote_commit || '').slice(0, 8) || '—')}</div>
        <div class="update-panel-row"><span class="status-dot ${d.has_update ? 'warn' : 'ok'}"></span> ${d.has_update ? 'Hay actualizaciones disponibles' : 'Al día'}</div>
      `;
    } catch (e) {
      repoEl.innerHTML = '<div class="update-panel-row"><span class="status-dot error"></span> Error: ' + _esc(String(e)) + '</div>';
    }
  }

  overlay.querySelector('.update-panel-close').addEventListener('click', () => {
    overlay.classList.remove('open');
    setTimeout(() => overlay.remove(), 250);
  });
  overlay.addEventListener('click', (e) => {
    if (e.target === overlay) overlay.querySelector('.update-panel-close').click();
  });

  overlay.querySelector('#update-btn-refresh').addEventListener('click', async () => {
    log('Comprobando...');
    await loadPrereqs();
    await loadRepo();
    log('Comprobación lista.');
  });

  overlay.querySelector('#update-btn-restart').addEventListener('click', async () => {
    if (!confirm('¿Reiniciar el servidor ATLAS ahora? Se cerrará esta sesión en ~3 segundos. Deberás recargar la página cuando vuelva a estar en línea.')) return;
    const btn = overlay.querySelector('#update-btn-restart');
    btn.disabled = true;
    log('Solicitando reinicio del servidor...');
    try {
      const r = await fetch('/update/restart', { method: 'POST', headers: { 'Content-Type': 'application/json' } });
      const j = await r.json().catch(() => ({}));
      if (r.ok && j.ok) {
        log(j.data && j.data.message ? j.data.message : 'Reinicio programado.');
        log('Recarga la página en 5-10 segundos (F5).');
      } else {
        log('Error: ' + (j.error || r.status));
        if (r.status === 404) {
          log('');
          log('Este servidor no tiene la ruta de reinicio (código antiguo). Reinicio manual:');
          log('  1. Cierra esta ventana y en una terminal ejecuta:');
          log('  2. cd ' + (window.location.pathname.startsWith('/ui') ? 'tu_carpeta_ATLAS_PUSH' : 'C:\\ATLAS_PUSH'));
          log('  3. powershell -ExecutionPolicy Bypass -File scripts\\restart_push_from_api.ps1');
          log('  O mata el proceso en el puerto 8791 y vuelve a iniciar: python -m uvicorn atlas_adapter.atlas_http_api:app --host 0.0.0.0 --port 8791');
        }
      }
    } catch (e) {
      log('Error: ' + String(e) + ' (el servidor puede estar reiniciándose).');
    }
    btn.disabled = false;
  });

  overlay.querySelector('#update-btn-apply').addEventListener('click', async () => {
    if (!confirm('¿Aplicar actualización desde el remoto? (fetch + staging + smoke + promote).')) return;
    const btn = overlay.querySelector('#update-btn-apply');
    btn.disabled = true;
    log('Aplicando actualización...');
    try {
      const r = await fetch('/update/apply', { method: 'POST', headers: { 'Content-Type': 'application/json' } });
      const j = await r.json();
      const err = j.error || '';
      if (j.ok) {
        log('OK: ' + JSON.stringify(j.data));
      } else {
        log('');
        log('Error: ' + err);
        if (j.data && j.data.steps && Array.isArray(j.data.steps)) {
          const lastBad = j.data.steps.filter(s => !s.ok).pop();
          if (lastBad) log('Último paso fallido: ' + (lastBad.step || '') + (lastBad.error ? ' — ' + lastBad.error : ''));
        }
        if (err.includes('POLICY_ALLOW_UPDATE_APPLY') || err.includes('policy')) {
          log('');
          log('Para permitir aplicar actualizaciones desde este panel, configura en el entorno o en config/atlas.env:');
          log('  POLICY_ALLOW_UPDATE_APPLY=true');
          log('');
          log('Luego reinicia el servidor ATLAS.');
        }
      }
      await loadRepo();
      await loadPrereqs();
    } catch (e) {
      log('Error: ' + String(e), true);
    }
    btn.disabled = false;
  });

  overlay.classList.add('open');
  loadPrereqs();
  loadRepo();
}

function _getViewContainer() {
  return document.getElementById('app-view');
}

function _clearView() {
  if (_currentCleanup) { try { _currentCleanup(); } catch {} _currentCleanup = null; }
  const v = _getViewContainer();
  if (v) v.innerHTML = '';
  return v;
}

function _handleRoute() {
  const hash = (location.hash.slice(1) || '/').split('?')[0];
  window.AtlasCompanion?.onNavigate?.(hash);
  window.AtlasState?.set('menuOpen', false);

  const v = _clearView();
  if (!v) return;

  // Landing
  if (!hash || hash === '/') {
    const cleanup = window.AtlasLanding?.render(v);
    if (typeof cleanup === 'function') _currentCleanup = cleanup;
    return;
  }

  // Dispatch to registered module
  const moduleId = hash.replace(/^\//, '');
  const mod = MODULE_REGISTRY[moduleId];
  if (mod) {
    try {
      const cleanup = mod.render(v);
      _currentCleanup = () => {
        if (typeof cleanup === 'function') try { cleanup(); } catch {}
        try { mod.destroy?.(); } catch {}
      };
      // Estandariza texto del botón back en todos los módulos
      v.querySelectorAll('.back-btn').forEach(btn => {
        const txt = btn.textContent.trim();
        if (txt === 'Inicio' || txt === 'Home' || txt === '← Home') {
          const svg = btn.querySelector('svg');
          btn.innerHTML = (svg ? svg.outerHTML : '') + ' Dashboard';
        }
      });
    } catch (e) {
      v.innerHTML = `<div class="module-view"><div class="module-header"><button class="back-btn" onclick="location.hash='/'">← Dashboard</button><h2>Error</h2></div><div class="module-body"><p style="color:var(--accent-red)">${_esc(e.message)}</p></div></div>`;
    }
    return;
  }

  // Special routes
  if (moduleId === '_update' || moduleId === '__update') {
    location.hash = '/';
    setTimeout(() => window.AtlasUpdatePanel?.open?.(), 50);
    return;
  }

  // Unknown route → landing
  location.hash = '/';
}

async function _loadModules() {
  const MODULE_NAMES = [
    'health', 'config', 'bitacora', 'memory', 'learning',
    'autonomy', 'healing', 'approvals', 'audit', 'comms',
    'events', 'api_explorer', 'voice', 'trading',
  ];
  for (const name of MODULE_NAMES) {
    try {
      const mod = (await import(`/v4/static/modules/${name}.js`)).default;
      if (mod?.id) MODULE_REGISTRY[mod.id] = mod;
    } catch (e) {
      console.warn(`[Atlas v4.2] Module "${name}" no cargado:`, e.message);
    }
  }
}

function _startHealthPolling() {
  async function _poll() {
    try {
      const r = await fetch('/health');
      const d = await r.json();
      const uptime = d.checks?.active_port ? 'online' : '--';
      const version = d.checks?.version || d.version || '--';
      window.AtlasState?.set('health', { uptime, ok: d.ok, score: d.score });
      window.AtlasState?.set('model', version);
      const dot = document.getElementById('topbar-dot');
      if (dot) dot.className = `topbar-health-dot${d.ok ? '' : ' error'}`;
    } catch {
      const dot = document.getElementById('topbar-dot');
      if (dot) dot.className = 'topbar-health-dot error';
    }
  }
  _poll();
  setInterval(_poll, 12000);
}

async function main() {
  const savedTheme = localStorage.getItem('atlas-theme') || 'cyan';
  _themeIdx = THEMES.indexOf(savedTheme);
  if (_themeIdx < 0) _themeIdx = 0;
  _applyTheme(THEMES[_themeIdx]);

  await _loadModules();

  const app = document.getElementById('app');
  app.innerHTML = '';

  _buildTopbar(app);
  window.AtlasUpdatePanel = { open: _openUpdatePanel };
  window.AtlasMegaMenu?.mount?.(app);

  const viewContainer = document.createElement('main');
  viewContainer.id = 'app-view';
  app.appendChild(viewContainer);

  window.AtlasCompanion?.mount?.(app);

  window.addEventListener('hashchange', _handleRoute);
  _handleRoute();

  _startHealthPolling();

  let _idleTimer = null;
  document.addEventListener('mousemove', () => {
    if (window.AtlasCompanion?.getState() === 'sleeping') window.AtlasCompanion.wake();
    clearTimeout(_idleTimer);
    _idleTimer = setTimeout(() => window.AtlasCompanion?.sleep(), 120000);
  });
}

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', main);
} else {
  main();
}
