/**
 * ATLAS v4 — Landing Page (Google-style)
 * Clean, centered search bar with contextual quick actions and app tiles.
 */
import { get, on } from '../lib/state.js';
import { navigate } from '../lib/router.js';

const SVG = {
  arrow: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M5 12h14M12 5l7 7-7 7"/></svg>',
  health: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M22 12h-4l-3 9L9 3l-3 9H2"/></svg>',
  workspace: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="2" y="3" width="20" height="14" rx="2"/><path d="M8 21h8M12 17v4"/></svg>',
  monitor: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="4" y="4" width="16" height="16" rx="2"/><path d="M9 9h6M9 13h6M9 17h4"/></svg>',
  bitacora: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M14 2H6a2 2 0 00-2 2v16a2 2 0 002 2h12a2 2 0 002-2V8z"/><path d="M14 2v6h6M16 13H8M16 17H8M10 9H8"/></svg>',
  config: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="3"/><path d="M12 1v4M12 19v4M4.22 4.22l2.83 2.83M16.95 16.95l2.83 2.83M1 12h4M19 12h4M4.22 19.78l2.83-2.83M16.95 7.05l2.83-2.83"/></svg>',
  trading: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="22 7 13.5 15.5 8.5 10.5 2 17"/><polyline points="16 7 22 7 22 13"/></svg>',
  governance: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M12 22s8-4 8-10V5l-8-3-8 3v7c0 6 8 10 8 10z"/></svg>',
  brain: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M12 2a7 7 0 017 7c0 3-2 5-4 6v3h-6v-3c-2-1-4-3-4-6a7 7 0 017-7z"/><path d="M9 18h6M10 21h4"/></svg>',
};

function _getGreeting() {
  const h = new Date().getHours();
  if (h < 6) return 'Trabajando de madrugada';
  if (h < 12) return 'Buenos dias';
  if (h < 18) return 'Buenas tardes';
  return 'Buenas noches';
}

export function render(container) {
  const health = get('health');
  const greeting = _getGreeting();
  const lastAct = get('lastActivity') || 'Sistema listo';

  container.innerHTML = `
    <div class="landing">
      <div class="landing-logo">
        <h1>ATLAS</h1>
        <div class="subtitle">Intelligent System Hub</div>
      </div>

      <div class="landing-greeting">
        ${greeting}. ${lastAct}
      </div>

      <div class="search-container">
        <input class="search-bar" id="landing-search"
               placeholder="Pregunta, busca o da un comando a Atlas..."
               autocomplete="off" autofocus>
        <button class="search-submit" id="landing-submit">
          ${SVG.arrow}
        </button>
      </div>

      <div class="quick-actions">
        <div class="quick-chip" data-action="health">${SVG.health} System Health</div>
        <div class="quick-chip" data-action="workspace">${SVG.workspace} Workspace</div>
        <div class="quick-chip" data-action="bitacora">${SVG.bitacora} Bitacora</div>
        <div class="quick-chip" data-action="config">${SVG.config} AI Config</div>
        <div class="quick-chip" data-action="autonomy">${SVG.governance} Autonomy</div>
      </div>

      <div class="apps-grid">
        <div class="app-tile" data-app="assistant">
          <div class="app-tile-icon">${SVG.brain}</div>
          <div class="app-tile-label">AI Assistant</div>
        </div>
        <div class="app-tile" data-app="workspace-ext">
          <div class="app-tile-icon">${SVG.workspace}</div>
          <div class="app-tile-label">Workspace</div>
        </div>
        <div class="app-tile" data-app="governance">
          <div class="app-tile-icon">${SVG.governance}</div>
          <div class="app-tile-label">Governance</div>
        </div>
        <div class="app-tile" data-app="approvals">
          <div class="app-tile-icon">${SVG.governance}</div>
          <div class="app-tile-label">Approvals</div>
        </div>
        <div class="app-tile" data-app="monitor">
          <div class="app-tile-icon">${SVG.monitor}</div>
          <div class="app-tile-label">Monitor</div>
        </div>
        <div class="app-tile" data-app="trading">
          <div class="app-tile-icon">${SVG.trading}</div>
          <div class="app-tile-label">Trading</div>
        </div>
        <div class="app-tile" data-app="config-tile">
          <div class="app-tile-icon">${SVG.config}</div>
          <div class="app-tile-label">Settings</div>
        </div>
      </div>
    </div>

    <div class="landing-footer">
      <span id="footer-uptime">Uptime: --</span>
      <span id="footer-model">Model: --</span>
      <span>v4.0.0</span>
    </div>
  `;

  const searchInput = container.querySelector('#landing-search');
  const submitBtn = container.querySelector('#landing-submit');

  function submitSearch() {
    const q = searchInput.value.trim();
    if (!q) return;
    // v4 is presentation-only. Send users to the operational dashboard (v3).
    window.location.href = `/v3`;
  }

  searchInput.addEventListener('keydown', e => { if (e.key === 'Enter') submitSearch(); });
  submitBtn.addEventListener('click', submitSearch);

  container.querySelectorAll('.quick-chip').forEach(chip => {
    chip.addEventListener('click', () => {
      window.AtlasSounds?.click();
      const action = chip.dataset.action;
      if (action === 'workspace') {
        window.open('/workspace', '_blank');
      } else {
        window.location.href = '/v3';
      }
    });
  });

  container.querySelectorAll('.app-tile').forEach(tile => {
    tile.addEventListener('click', () => {
      window.AtlasSounds?.click();
      const app = tile.dataset.app;
      if (app === 'assistant') window.location.href = '/v3';
      else if (app === 'workspace-ext') window.open('/workspace', '_blank');
      else window.location.href = '/v3';
    });
  });

  const unsubHealth = on('health', h => {
    const el = container.querySelector('#footer-uptime');
    if (el && h) el.textContent = `Uptime: ${h.uptime || '--'}`;
  });
  const unsubModel = on('model', m => {
    const el = container.querySelector('#footer-model');
    if (el) el.textContent = `Model: ${m || '--'}`;
  });

  return () => { unsubHealth(); unsubModel(); };
}

window.AtlasLanding = { render };
