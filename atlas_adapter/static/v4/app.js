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
      <span class="topbar-version" id="topbar-version">v4.0</span>
      <button class="topbar-btn" id="btn-theme" title="Change theme">${SVG.theme}</button>
      <button class="topbar-btn" id="btn-home" title="Home">${SVG.home}</button>
      <button class="topbar-btn" id="btn-dashboard" title="Open Dashboard" style="font-weight:600;padding:8px 10px">Dashboard</button>
      <button class="topbar-btn" id="btn-workspace" title="Open Workspace" style="font-weight:600;padding:8px 10px">Workspace</button>
    </div>
  `;
  app.appendChild(bar);

  bar.querySelector('#btn-dashboard')?.addEventListener('click', () => { window.location.href = '/v3'; });
  bar.querySelector('#btn-workspace')?.addEventListener('click', () => { window.location.href = '/workspace'; });
  bar.querySelector('#btn-home').addEventListener('click', () => { location.hash = '/'; });
  bar.querySelector('#btn-theme').addEventListener('click', _cycleTheme);
  bar.querySelector('#topbar-logo').addEventListener('click', () => { location.hash = '/'; });
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

  // Landing-only: any deep link should go to the operational dashboard (v3).
  if (hash && hash !== '/' && hash !== '') {
    window.location.href = '/v3';
    return;
  }
  const v = _clearView();
  if (!v) return;

  const cleanup = window.AtlasLanding?.render(v);
  if (typeof cleanup === 'function') _currentCleanup = cleanup;
}

async function _loadModules() {
  // Landing-only.
  return;
}

function _startHealthPolling() {
  // Landing-only: keep dot green without polling.
  const dot = document.getElementById('topbar-dot');
  if (dot) dot.className = 'topbar-health-dot';
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
  // No mega menu on landing-only.

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
