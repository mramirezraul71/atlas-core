/**
 * ATLAS v4 — Landing Page (Google-style)
 * Clean, centered search bar with contextual quick actions and app tiles.
 */
import { get, on } from '../lib/state.js';
import { navigate } from '../lib/router.js';

const SVG = {
  mic: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M12 1a3 3 0 00-3 3v8a3 3 0 006 0V4a3 3 0 00-3-3z"/><path d="M19 10v2a7 7 0 01-14 0v-2M12 19v4M8 23h8"/></svg>',
  arrow: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M5 12h14M12 5l7 7-7 7"/></svg>',
  health: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M22 12h-4l-3 9L9 3l-3 9H2"/></svg>',
  workspace: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="2" y="3" width="20" height="14" rx="2"/><path d="M8 21h8M12 17v4"/></svg>',
  monitor: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="4" y="4" width="16" height="16" rx="2"/><path d="M9 9h6M9 13h6M9 17h4"/></svg>',
  bitacora: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M14 2H6a2 2 0 00-2 2v16a2 2 0 002 2h12a2 2 0 002-2V8z"/><path d="M14 2v6h6M16 13H8M16 17H8M10 9H8"/></svg>',
  config: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="3"/><path d="M12 1v4M12 19v4M4.22 4.22l2.83 2.83M16.95 16.95l2.83 2.83M1 12h4M19 12h4M4.22 19.78l2.83-2.83M16.95 7.05l2.83-2.83"/></svg>',
  trading: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="22 7 13.5 15.5 8.5 10.5 2 17"/><polyline points="16 7 22 7 22 13"/></svg>',
  apps: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polygon points="12 2 2 7 12 12 22 7 12 2"/><polyline points="2 17 12 22 22 17"/><polyline points="2 12 12 17 22 12"/></svg>',
  eye: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z"/><circle cx="12" cy="12" r="3"/></svg>',
  package: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><line x1="16.5" y1="9.4" x2="7.5" y2="4.21"/><path d="M21 16V8a2 2 0 00-1-1.73l-7-4a2 2 0 00-2 0l-7 4A2 2 0 002 8v8a2 2 0 001 1.73l7 4a2 2 0 002 0l7-4A2 2 0 0021 16z"/><polyline points="3.27 6.96 12 12.01 20.73 6.96"/><line x1="12" y1="22.08" x2="12" y2="12"/></svg>',
  governance: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M12 22s8-4 8-10V5l-8-3-8 3v7c0 6 8 10 8 10z"/></svg>',
  brain: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M12 2a7 7 0 017 7c0 3-2 5-4 6v3h-6v-3c-2-1-4-3-4-6a7 7 0 017-7z"/><path d="M9 18h6M10 21h4"/></svg>',
  grafana: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="2" y="2" width="20" height="20" rx="2"/><polyline points="6 16 10 10 14 13 18 8"/><circle cx="6" cy="16" r="1" fill="currentColor"/></svg>',
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
        <button class="search-mic" id="landing-mic" title="Hablar con Atlas">
          ${SVG.mic}
        </button>
        <button class="search-submit" id="landing-submit">
          ${SVG.arrow}
        </button>
      </div>

      <style>
        .search-mic {
          position: absolute;
          right: 52px;
          top: 50%;
          transform: translateY(-50%);
          width: 36px; height: 36px;
          display: flex; align-items: center; justify-content: center;
          background: none; border: none; cursor: pointer;
          color: var(--text-muted);
          border-radius: 50%;
          transition: color .2s, background .2s;
        }
        .search-mic svg { width: 18px; height: 18px; }
        .search-mic:hover { color: var(--accent-primary); background: rgba(255,255,255,.06); }
        .search-mic.listening {
          color: var(--accent-red, #ff4444);
          animation: mic-pulse 1s ease-in-out infinite;
        }
        @keyframes mic-pulse {
          0%, 100% { opacity: 1; transform: translateY(-50%) scale(1); }
          50%       { opacity: .6; transform: translateY(-50%) scale(1.15); }
        }
        .search-container { position: relative; }
      </style>

      <div class="quick-actions">
        <div class="quick-chip" data-action="health">${SVG.health} Salud del sistema</div>
        <div class="quick-chip" data-action="clawd-direct">${SVG.brain} ATLAS Directo</div>
        <div class="quick-chip" data-action="bitacora">${SVG.bitacora} Bitacora</div>
        <div class="quick-chip quant-chip" data-action="code-quant-dashboard" style="color:#00d4aa;border-color:rgba(0,212,170,0.4);background:rgba(0,212,170,0.07)">${SVG.trading} Code Quant</div>
        <div class="quick-chip quant-chip" data-action="atlas-quant-scanner" style="color:#00d4aa;border-color:rgba(0,212,170,0.25)">&#9670; Escaner Quant</div>
        <div class="quick-chip quant-chip" data-action="option-strat" style="color:#00d4aa;border-color:rgba(0,212,170,0.35);background:rgba(0,212,170,0.07)">&#9651; OptionStrat</div>
        <div class="quick-chip quant-chip" data-action="grafana-quant" style="color:#f46800;border-color:rgba(244,104,0,0.35)">${SVG.grafana} Grafana</div>
        <div class="quick-chip" data-action="rauli-vision">${SVG.eye} Rauli Vision</div>
        <div class="quick-chip" data-action="rauli-panaderia">${SVG.package} Rauli Panaderia</div>
        <div class="quick-chip" data-action="bety-eventos">${SVG.bitacora} Bety Eventos</div>
        <div class="quick-chip" data-action="workspace">${SVG.workspace} Workspace</div>
        <div class="quick-chip" data-action="config">${SVG.config} AI Config</div>
        <div class="quick-chip" data-action="autonomy">${SVG.governance} Autonomia</div>
        <div class="quick-chip" data-action="codex-supervisor">${SVG.governance} Codex Supervisor</div>
        <div class="quick-chip" data-action="tools-menu">${SVG.monitor} Menu de herramientas</div>
        <div class="quick-chip" data-action="software-center">${SVG.package} Centro de software</div>
        <div class="quick-chip" data-action="live-diagnostic">${SVG.health} Diagnostico Live</div>
        <div class="quick-chip" data-action="atlas-nexus">⬡ Atlas Nexus</div>
        <div class="quick-chip" data-action="robot-3d">🤖 Robot 3D</div>
      </div>

      <div class="apps-grid">

        <!-- ── QUANT GROUP ─────────────────────────────── -->
        <div class="app-tile" data-app="code-quant-dashboard" style="border-color:rgba(0,212,170,0.4);background:rgba(0,212,170,0.05)">
          <div class="app-tile-icon" style="color:#00d4aa">${SVG.trading}</div>
          <div class="app-tile-label" style="color:#00d4aa">Code Quant</div>
        </div>
        <div class="app-tile" data-app="atlas-quant-scanner" style="border-color:rgba(0,212,170,0.25)">
          <div class="app-tile-icon" style="color:#00d4aa">&#9670;</div>
          <div class="app-tile-label">Escaner Quant</div>
        </div>
        <div class="app-tile" data-app="option-strat" style="border-color:rgba(0,212,170,0.4);background:rgba(0,212,170,0.05)">
          <div class="app-tile-icon" style="color:#00d4aa">&#9651;</div>
          <div class="app-tile-label" style="color:#00d4aa">OptionStrat</div>
        </div>
        <div class="app-tile" data-app="grafana-quant" style="border-color:rgba(244,104,0,0.35)">
          <div class="app-tile-icon" style="color:#f46800">${SVG.grafana}</div>
          <div class="app-tile-label">Grafana</div>
        </div>

        <!-- ── CORE ───────────────────────────────────── -->
        <div class="app-tile" data-app="assistant">
          <div class="app-tile-icon">${SVG.brain}</div>
          <div class="app-tile-label">AI Assistant</div>
        </div>
        <div class="app-tile" data-app="monitor">
          <div class="app-tile-icon">${SVG.monitor}</div>
          <div class="app-tile-label">Monitor</div>
        </div>
        <div class="app-tile" data-app="clawd-direct">
          <div class="app-tile-icon">${SVG.brain}</div>
          <div class="app-tile-label">ATLAS Directo</div>
        </div>
        <div class="app-tile" data-app="mis-apps">
          <div class="app-tile-icon">${SVG.apps}</div>
          <div class="app-tile-label">Mis Apps</div>
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

        <!-- ── BRAZOS ─────────────────────────────────── -->
        <div class="app-tile" data-app="rauli-vision">
          <div class="app-tile-icon">${SVG.eye}</div>
          <div class="app-tile-label">Rauli Vision</div>
        </div>
        <div class="app-tile" data-app="rauli-panaderia">
          <div class="app-tile-icon">${SVG.package}</div>
          <div class="app-tile-label">Rauli Panaderia</div>
        </div>
        <div class="app-tile" data-app="bety-eventos">
          <div class="app-tile-icon">${SVG.bitacora}</div>
          <div class="app-tile-label">Bety Eventos</div>
        </div>

        <!-- ── TOOLS ──────────────────────────────────── -->
        <div class="app-tile" data-app="config-tile">
          <div class="app-tile-icon">${SVG.config}</div>
          <div class="app-tile-label">Settings</div>
        </div>
        <div class="app-tile" data-app="tools-menu">
          <div class="app-tile-icon">${SVG.monitor}</div>
          <div class="app-tile-label">Herramientas</div>
        </div>
        <div class="app-tile" data-app="software-center">
          <div class="app-tile-icon">${SVG.package}</div>
          <div class="app-tile-label">Software</div>
        </div>
        <div class="app-tile" data-app="codex-supervisor">
          <div class="app-tile-icon">${SVG.governance}</div>
          <div class="app-tile-label">Codex Supervisor</div>
        </div>
        <div class="app-tile" data-app="live-diagnostic">
          <div class="app-tile-icon">${SVG.health}</div>
          <div class="app-tile-label">Diagnostico Live</div>
        </div>
        <div class="app-tile" data-app="atlas-nexus">
          <div class="app-tile-icon">⬡</div>
          <div class="app-tile-label">Atlas Nexus</div>
        </div>
        <div class="app-tile" data-app="robot-3d">
          <div class="app-tile-icon">🤖</div>
          <div class="app-tile-label">Robot 3D</div>
        </div>
        <div class="app-tile" data-app="doctor" style="border-color:rgba(255,80,80,0.4);background:rgba(255,80,80,0.05)">
          <div class="app-tile-icon" style="color:#ff5050">${SVG.health}</div>
          <div class="app-tile-label" style="color:#ff5050">ATLAS Doctor</div>
        </div>
      </div>
    </div>

    <div class="landing-footer">
      <span id="footer-uptime">Uptime: --</span>
      <span id="footer-model">Model: --</span>
      <span>v4.3.0</span>
    </div>
  `;

  const searchInput = container.querySelector('#landing-search');
  const submitBtn = container.querySelector('#landing-submit');

  function submitSearch() {
    const q = searchInput.value.trim();
    if (!q) return;
    location.hash = `/bitacora?q=${encodeURIComponent(q)}`;
  }

  searchInput.addEventListener('keydown', e => { if (e.key === 'Enter') submitSearch(); });
  submitBtn.addEventListener('click', submitSearch);

  // ── Micrófono (Web Speech API) ──────────────────────────────────────────────
  const micBtn = container.querySelector('#landing-mic');
  const SpeechRec = window.SpeechRecognition || window.webkitSpeechRecognition;
  if (!SpeechRec) {
    if (micBtn) micBtn.style.display = 'none';
  } else {
    const rec = new SpeechRec();
    rec.lang = 'es-ES';
    rec.interimResults = true;
    rec.maxAlternatives = 1;
    let _recognizing = false;

    rec.onstart = () => {
      _recognizing = true;
      micBtn.classList.add('listening');
      micBtn.title = 'Escuchando… (clic para detener)';
    };
    rec.onend = () => {
      _recognizing = false;
      micBtn.classList.remove('listening');
      micBtn.title = 'Hablar con Atlas';
    };
    rec.onresult = (e) => {
      const transcript = Array.from(e.results).map(r => r[0].transcript).join('');
      searchInput.value = transcript;
      if (e.results[e.results.length - 1].isFinal && transcript.trim()) {
        sessionStorage.setItem('atlas-voice-query', transcript.trim());
        location.hash = '/chat';
      }
    };
    rec.onerror = (e) => {
      _recognizing = false;
      micBtn.classList.remove('listening');
      const msg = e.error === 'not-allowed'
        ? 'Permiso de micrófono denegado'
        : 'Error de micrófono: ' + e.error;
      window.AtlasToast?.show(msg, 'error');
    };

    micBtn.addEventListener('click', () => {
      if (_recognizing) { rec.stop(); return; }
      rec.start();
    });
  }
  // ───────────────────────────────────────────────────────────────────────────

  const CHIP_ROUTES = {
    health:    { hash: '/health' },
    workspace: { href: '/workspace', newTab: true },
    bitacora:  { hash: '/bitacora' },
    'rauli-vision': { hash: '/apps/vision' },
    'rauli-panaderia': { hash: '/apps/panaderia' },
    'bety-eventos': { hash: '/bety-eventos' },
    config:    { hash: '/config' },
    autonomy:  { hash: '/autonomy' },
    'clawd-direct': { hash: '/clawd-direct' },
    'codex-supervisor': { hash: '/codex-supervisor' },
    'tools-menu': { hash: '/tools-menu' },
    'software-center': { hash: '/software-center' },
    'live-diagnostic': { hash: '/live-diagnostic' },
    'atlas-quant-scanner':  { hash: '/atlas-quant' },
    'code-quant-dashboard': { href: '/v4/static/quant/index.html', newTab: true },
    'option-strat':         { href: 'http://localhost:8795/options/ui', newTab: true },
    'grafana-quant':        { href: 'http://localhost:3002', newTab: true },
    'atlas-nexus': { href: '/nexus', newTab: false },
    'robot-3d':    { href: 'http://127.0.0.1:8002/dashboard', newTab: true },
  };

  container.querySelectorAll('.quick-chip').forEach(chip => {
    chip.addEventListener('click', () => {
      window.AtlasSounds?.click();
      const r = CHIP_ROUTES[chip.dataset.action];
      if (!r) return;
      if (r.hash) location.hash = r.hash;
      else if (r.newTab) window.open(r.href, '_blank');
      else window.location.href = r.href;
    });
  });

  const TILE_ROUTES = {
    'assistant':    { hash: '/chat' },
    'workspace-ext':{ href: '/workspace', newTab: true },
    'governance':   { hash: '/autonomy' },
    'approvals':    { hash: '/approvals' },
    'monitor':      { hash: '/health' },
    'mis-apps':     { hash: '/apps' },
    'rauli-vision': { hash: '/apps/vision' },
    'rauli-panaderia': { hash: '/apps/panaderia' },
    'bety-eventos': { hash: '/bety-eventos' },
    'config-tile':  { hash: '/config' },
    'tools-menu':   { hash: '/tools-menu' },
    'software-center': { hash: '/software-center' },
    'atlas-quant-scanner':  { hash: '/atlas-quant' },
    'code-quant-dashboard': { href: '/v4/static/quant/index.html', newTab: true },
    'option-strat':         { href: 'http://localhost:8795/options/ui', newTab: true },
    'grafana-quant':        { href: 'http://localhost:3002', newTab: true },
    'clawd-direct': { hash: '/clawd-direct' },
    'codex-supervisor': { hash: '/codex-supervisor' },
    'live-diagnostic': { hash: '/live-diagnostic' },
    'atlas-nexus': { href: '/nexus', newTab: false },
    'robot-3d':    { href: 'http://127.0.0.1:8002/dashboard', newTab: true },
    'doctor':      { hash: '/doctor' },
  };

  container.querySelectorAll('.app-tile').forEach(tile => {
    tile.addEventListener('click', () => {
      window.AtlasSounds?.click();
      const r = TILE_ROUTES[tile.dataset.app];
      if (!r) return;
      if (r.hash) location.hash = r.hash;
      else if (r.newTab) window.open(r.href, '_blank');
      else window.location.href = r.href;
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
