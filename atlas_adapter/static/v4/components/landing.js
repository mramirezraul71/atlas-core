/**
 * ATLAS v4 â€” Landing Page (Google-style)
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
  radar: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="10"/><circle cx="12" cy="12" r="3"/><path d="M12 2v4M12 18v4M2 12h4M18 12h4"/></svg>',
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
        .landing-options-card {
          margin: 20px auto 0;
          width: min(980px, 100%);
          border: 1px solid rgba(0, 212, 170, 0.18);
          background: linear-gradient(180deg, rgba(8,15,22,0.95), rgba(10,18,27,0.92));
          border-radius: 18px;
          padding: 16px 18px;
          box-shadow: 0 18px 48px rgba(0,0,0,0.24);
        }
        .landing-options-head {
          display: flex;
          align-items: center;
          justify-content: space-between;
          gap: 12px;
          margin-bottom: 12px;
          flex-wrap: wrap;
        }
        .landing-options-title { font-size: 18px; font-weight: 700; color: #dcefff; }
        .landing-options-sub { font-size: 13px; color: var(--text-muted); margin-top: 4px; }
        .landing-options-badge {
          display: inline-flex;
          align-items: center;
          padding: 6px 11px;
          border-radius: 999px;
          font-size: 12px;
          font-weight: 800;
          letter-spacing: .06em;
          text-transform: uppercase;
          border: 1px solid transparent;
        }
        .landing-options-badge.go { color: #7ee787; background: rgba(46,160,67,.16); border-color: rgba(46,160,67,.35); }
        .landing-options-badge.no-go { color: #ff7b72; background: rgba(248,81,73,.16); border-color: rgba(248,81,73,.35); }
        .landing-options-badge.degraded { color: #f2cc60; background: rgba(210,153,34,.16); border-color: rgba(210,153,34,.35); }
        .landing-options-grid {
          display: grid;
          grid-template-columns: repeat(auto-fit, minmax(132px, 1fr));
          gap: 10px;
          margin-bottom: 12px;
        }
        .landing-options-stat {
          border: 1px solid rgba(82, 97, 121, 0.28);
          border-radius: 12px;
          background: rgba(12, 20, 29, 0.86);
          padding: 10px 12px;
        }
        .landing-options-stat span {
          display: block;
          font-size: 11px;
          text-transform: uppercase;
          letter-spacing: .05em;
          color: var(--text-muted);
          margin-bottom: 6px;
        }
        .landing-options-stat strong { font-size: 16px; color: #f6fbff; }
        .landing-options-note {
          border: 1px solid rgba(82, 97, 121, 0.22);
          background: rgba(12, 20, 29, 0.9);
          color: #d0dae6;
          border-radius: 12px;
          padding: 10px 12px;
          font-size: 13px;
          margin-bottom: 12px;
        }
        .landing-options-note.warn {
          color: #f2cc60;
          border-color: rgba(210,153,34,.35);
          background: rgba(210,153,34,.08);
        }
        .landing-options-meta {
          display: flex;
          align-items: center;
          gap: 8px;
          color: var(--text-muted);
          font-size: 12px;
          margin: -2px 0 12px;
        }
        .landing-options-dot {
          width: 9px;
          height: 9px;
          border-radius: 50%;
          background: #f2cc60;
          box-shadow: 0 0 0 2px rgba(255,255,255,.03);
          flex: 0 0 auto;
        }
        .landing-options-dot.go { background: #7ee787; }
        .landing-options-dot.warm,
        .landing-options-dot.degraded { background: #f2cc60; }
        .landing-options-dot.stale,
        .landing-options-dot.no-go { background: #ff7b72; }
        .landing-options-links {
          display: flex;
          gap: 10px;
          flex-wrap: wrap;
        }
        .landing-options-links a {
          display: inline-flex;
          align-items: center;
          justify-content: center;
          padding: 8px 12px;
          border-radius: 10px;
          text-decoration: none;
          border: 1px solid rgba(88,166,255,.22);
          background: rgba(88,166,255,.06);
          color: #8bc2ff;
          font-size: 13px;
        }
      </style>

      <div class="quick-actions">
        <div class="quick-chip" data-action="health">${SVG.health} Salud del sistema</div>
        <div class="quick-chip" data-action="clawd-direct">${SVG.brain} ATLAS Directo</div>
        <div class="quick-chip" data-action="bitacora">${SVG.bitacora} Bitacora</div>
        <div class="quick-chip quant-chip" data-action="code-quant-dashboard" style="color:#00d4aa;border-color:rgba(0,212,170,0.4);background:rgba(0,212,170,0.07)">${SVG.trading} Code Quant</div>
        <div class="quick-chip quant-chip" data-action="atlas-quant-scanner" style="color:#00d4aa;border-color:rgba(0,212,170,0.25)">&#9670; Escaner Quant</div>
        <div class="quick-chip quant-chip" data-action="option-strat" style="color:#00d4aa;border-color:rgba(0,212,170,0.35);background:rgba(0,212,170,0.07)">&#9651; OptionStrat</div>
        <div class="quick-chip quant-chip" data-action="institutional-radar" style="color:#79c0ff;border-color:rgba(121,192,255,0.35);background:rgba(121,192,255,0.06)">${SVG.radar} Radar Institucional</div>
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
        <div class="quick-chip" data-action="atlas-nexus">â¬¡ Atlas Nexus</div>
        <div class="quick-chip" data-action="robot-3d">ðŸ¤– Robot 3D</div>
      </div>

      <div class="landing-options-card" id="landing-options-card">
        <div class="landing-options-head">
          <div>
            <div class="landing-options-title">Options Engine</div>
            <div class="landing-options-sub" id="landing-options-sub">paper_only · pending</div>
          </div>
          <span class="landing-options-badge degraded" id="landing-options-badge">Cargando</span>
        </div>
        <div class="landing-options-grid" id="landing-options-grid">
          <div class="landing-options-stat"><span>IV Rank</span><strong>pending</strong></div>
          <div class="landing-options-stat"><span>Journal hoy</span><strong>pending</strong></div>
          <div class="landing-options-stat"><span>Sesiones</span><strong>pending</strong></div>
          <div class="landing-options-stat"><span>Cerrados hoy</span><strong>pending</strong></div>
          <div class="landing-options-stat"><span>Meta</span><strong>pending</strong></div>
          <div class="landing-options-stat"><span>WR / PF</span><strong>pending</strong></div>
        </div>
        <div class="landing-options-note" id="landing-options-note">Cargando estado operativo...</div>
        <div class="landing-options-meta" id="landing-options-meta">
          <span class="landing-options-dot degraded" id="landing-options-dot"></span>
          <span id="landing-options-refresh-text">Esperando primer snapshot...</span>
        </div>
        <div class="landing-options-links">
          <a href="#" id="landing-options-health" target="_blank" rel="noopener">Health</a>
          <a href="#" id="landing-options-signals" target="_blank" rel="noopener">Signals</a>
          <a href="#" id="landing-options-performance" target="_blank" rel="noopener">Performance</a>
          <a href="/quant-ui" id="landing-options-ui" target="_blank" rel="noopener">Options UI</a>
        </div>
      </div>

      <div class="apps-grid">

        <!-- â”€â”€ QUANT GROUP â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ -->
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
        <div class="app-tile" data-app="institutional-radar" style="border-color:rgba(121,192,255,0.35);background:rgba(121,192,255,0.05)">
          <div class="app-tile-icon" style="color:#79c0ff">${SVG.radar}</div>
          <div class="app-tile-label" style="color:#79c0ff">Radar Institucional</div>
        </div>
        <div class="app-tile" data-app="grafana-quant" style="border-color:rgba(244,104,0,0.35)">
          <div class="app-tile-icon" style="color:#f46800">${SVG.grafana}</div>
          <div class="app-tile-label">Grafana</div>
        </div>

        <!-- â”€â”€ CORE â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ -->
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

        <!-- â”€â”€ BRAZOS â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ -->
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

        <!-- â”€â”€ TOOLS â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ -->
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
          <div class="app-tile-icon">â¬¡</div>
          <div class="app-tile-label">Atlas Nexus</div>
        </div>
        <div class="app-tile" data-app="robot-3d">
          <div class="app-tile-icon">ðŸ¤–</div>
          <div class="app-tile-label">Robot 3D</div>
        </div></div>
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

  // â”€â”€ MicrÃ³fono (Web Speech API) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
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
      micBtn.title = 'Escuchandoâ€¦ (clic para detener)';
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
        ? 'Permiso de micrÃ³fono denegado'
        : 'Error de micrÃ³fono: ' + e.error;
      window.AtlasToast?.show(msg, 'error');
    };

    micBtn.addEventListener('click', () => {
      if (_recognizing) { rec.stop(); return; }
      rec.start();
    });
  }
  // â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

  const GRAFANA_PRO_URL = `http://${location.hostname}:3002/d/atlas-quant-pro-2026/7def89ea-334f-564c-b9c7-d190c3f6f69d`;

  const CHIP_ROUTES = {
    health:    { hash: '/health' },
    'institutional-radar': { href: '/radar/dashboard', newTab: false },
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
    'code-quant-dashboard': { href: '/quant-ui', newTab: true },
    'option-strat':         { href: 'http://localhost:8795/options/ui', newTab: true },
    'grafana-quant':        { href: GRAFANA_PRO_URL, newTab: true },
    'atlas-nexus': { href: '/nexus', newTab: false },
    'robot-3d':    { href: `http://${location.hostname}:8002/dashboard`, newTab: true },
  };

  function openRoute(route) {
    if (!route) return;
    if (route.hash) {
      location.hash = route.hash;
      return;
    }
    if (route.newTab) {
      const popup = window.open(route.href, '_blank', 'noopener');
      if (!popup) {
        window.location.href = route.href;
      }
      return;
    }
    window.location.href = route.href;
  }

  container.querySelectorAll('.quick-chip').forEach(chip => {
    chip.addEventListener('click', () => {
      window.AtlasSounds?.click();
      openRoute(CHIP_ROUTES[chip.dataset.action]);
    });
  });

  const TILE_ROUTES = {
    'assistant':    { hash: '/chat' },
    'institutional-radar': { href: '/radar/dashboard', newTab: false },
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
    'code-quant-dashboard': { href: '/quant-ui', newTab: true },
    'option-strat':         { href: 'http://localhost:8795/options/ui', newTab: true },
    'grafana-quant':        { href: GRAFANA_PRO_URL, newTab: true },
    'clawd-direct': { hash: '/clawd-direct' },
    'codex-supervisor': { hash: '/codex-supervisor' },
    'live-diagnostic': { hash: '/live-diagnostic' },
    'atlas-nexus': { href: '/nexus', newTab: false },
    'robot-3d':    { href: `http://${location.hostname}:8002/dashboard`, newTab: true },
  };

  container.querySelectorAll('.app-tile').forEach(tile => {
    tile.addEventListener('click', () => {
      window.AtlasSounds?.click();
      openRoute(TILE_ROUTES[tile.dataset.app]);
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

  function optionsStatusClass(status) {
    if (status === 'GO') return 'go';
    if (status === 'NO_GO' || status === 'NO-GO') return 'no-go';
    return 'degraded';
  }

  function safeMetric(value, formatter) {
    if (value === null || value === undefined || value === '' || Number.isNaN(value)) return 'pending';
    return formatter ? formatter(value) : String(value);
  }

  function deriveGrafanaUrl(seed, uid, slug) {
    if (!seed) return '#';
    try {
      const u = new URL(seed, location.origin);
      return `${u.origin}/d/${uid}/${slug}`;
    } catch {
      return '#';
    }
  }

  function formatRefreshTime(isoValue) {
    if (!isoValue) return 'Last refresh pending';
    const d = new Date(isoValue);
    if (Number.isNaN(d.getTime())) return 'Last refresh pending';
    return `Last refresh ${d.toLocaleTimeString([], { hour12: false })}`;
  }

  function journalFreshnessState(ageSeconds) {
    if (typeof ageSeconds !== 'number' || Number.isNaN(ageSeconds)) {
      return { label: 'Journal pending', dot: 'degraded' };
    }
    if (ageSeconds <= 300) return { label: 'Journal fresh', dot: 'go' };
    if (ageSeconds <= 900) return { label: 'Journal warm', dot: 'warm' };
    return { label: 'Journal stale', dot: 'stale' };
  }

  const OPTIONS_FETCH_TIMEOUT_MS = 8000;
  const OPTIONS_REFRESH_MS = 30000;
  const OPTIONS_BOOTSTRAP_RETRY_MS = 5000;

  function scheduleOptionsBootstrapRetry() {
    if (container.__optionsLastOk) return;
    if (container.__optionsBootstrapRetryTimer) return;
    container.__optionsBootstrapRetryTimer = window.setTimeout(() => {
      container.__optionsBootstrapRetryTimer = null;
      loadOptionsStatus().then(ok => {
        if (ok) container.__optionsLastOk = true;
      });
    }, OPTIONS_BOOTSTRAP_RETRY_MS);
  }

  async function loadOptionsStatus() {
    const badgeEl = container.querySelector('#landing-options-badge');
    const subEl = container.querySelector('#landing-options-sub');
    const gridEl = container.querySelector('#landing-options-grid');
    const noteEl = container.querySelector('#landing-options-note');
    const dotEl = container.querySelector('#landing-options-dot');
    const refreshTextEl = container.querySelector('#landing-options-refresh-text');
    const healthEl = container.querySelector('#landing-options-health');
    const signalsEl = container.querySelector('#landing-options-signals');
    const perfEl = container.querySelector('#landing-options-performance');
    const uiEl = container.querySelector('#landing-options-ui');
    if (!badgeEl || !subEl || !gridEl || !noteEl || !dotEl || !refreshTextEl) return false;
    if (container.__optionsLoadInFlight) return false;

    container.__optionsLoadInFlight = true;
    const controller = new AbortController();
    container.__optionsAbortController = controller;
    const timeoutId = window.setTimeout(() => {
      try { controller.abort(); } catch {}
    }, OPTIONS_FETCH_TIMEOUT_MS);
    try {
      const r = await fetch('/api/options-engine-status', {
        cache: 'no-store',
        signal: controller.signal,
        headers: { 'Accept': 'application/json' },
      });
      const d = await r.json();
      if (!r.ok || d.ok === false) throw new Error(d.error || `HTTP ${r.status}`);

      const status = d.status || d.go_nogo_label || 'DEGRADED';
      const loopMode = d.loop_mode || 'metrics unavailable';
      const automationMode = d.automation_mode || 'paper_only';
      const closedToday = d.paper_trades_closed_today ?? d.trades_closed_today ?? 0;
      const completedTotal = d.trades_completed_total ?? closedToday;
      const target = d.paper_trades_target ?? null;
      const stale = typeof d.journal_last_write_age_seconds === 'number' && d.journal_last_write_age_seconds > 300;
      const freshness = journalFreshnessState(d.journal_last_write_age_seconds);
      const wrpf = (d.wr_basic != null || d.pf_basic != null)
        ? `WR ${safeMetric(d.wr_basic, v => Number(v).toFixed(2))} · PF ${safeMetric(d.pf_basic, v => Number(v).toFixed(2))}`
        : 'pending';

      badgeEl.className = `landing-options-badge ${optionsStatusClass(status)}`;
      badgeEl.textContent = String(status).replace(/_/g, '-');
      subEl.textContent = `${automationMode} · ${loopMode}`;
      gridEl.innerHTML = [
        ['IV Rank', safeMetric(d.iv_rank_current, v => Number(v).toFixed(2))],
        ['Journal hoy', safeMetric(d.journal_events_today, v => parseInt(v, 10))],
        ['Sesiones', safeMetric(d.journal_sessions_today, v => parseInt(v, 10))],
        ['Cerrados hoy', safeMetric(closedToday, v => parseInt(v, 10))],
        ['Meta', target != null ? `${completedTotal}/${target}` : String(completedTotal)],
        ['WR / PF', wrpf],
      ].map(([label, value]) => `
        <div class="landing-options-stat">
          <span>${label}</span>
          <strong>${value}</strong>
        </div>
      `).join('');

      noteEl.className = `landing-options-note${stale ? ' warn' : ''}`;
      noteEl.textContent = stale
        ? `${d.notes || 'sin datos aun'} · Journal stale`
        : (d.notes || 'sin datos aun');

      if (healthEl) healthEl.href = d.grafana_health_dashboard_url || '#';
      if (signalsEl) signalsEl.href = d.grafana_signals_dashboard_url || deriveGrafanaUrl(d.grafana_health_dashboard_url, 'atlas-options-signals-intent', 'options-engine-signals-intent');
      if (perfEl) perfEl.href = d.grafana_paper_performance_dashboard_url || deriveGrafanaUrl(d.grafana_health_dashboard_url, 'atlas-options-paper-performance', 'options-engine-paper-performance');
      dotEl.className = `landing-options-dot ${freshness.dot}`;
      refreshTextEl.textContent = `${formatRefreshTime(d.last_updated_utc)} Â· ${freshness.label}`;
      if (uiEl) uiEl.href = d.options_ui_url || '/quant-ui';
      if (container.__optionsBootstrapRetryTimer) {
        clearTimeout(container.__optionsBootstrapRetryTimer);
        container.__optionsBootstrapRetryTimer = null;
      }
      container.__optionsLastOk = true;
      return true;
    } catch (err) {
      const errMsg = err?.name === 'AbortError' ? 'request timeout' : String(err);
      if (container.__optionsLastOk) {
        noteEl.className = 'landing-options-note warn';
        dotEl.className = 'landing-options-dot warm';
        refreshTextEl.textContent = `${refreshTextEl.textContent || 'Last refresh pending'} Â· refresh delayed`;
        noteEl.textContent = `${noteEl.textContent || 'Estado previo conservado'} Â· metrics refresh delayed`;
        return false;
      }
      badgeEl.className = 'landing-options-badge degraded';
      badgeEl.textContent = 'DEGRADED';
      subEl.textContent = 'paper_only · metrics unavailable';
      gridEl.innerHTML = [
        ['IV Rank', 'pending'],
        ['Journal hoy', 'pending'],
        ['Sesiones', 'pending'],
        ['Cerrados hoy', 'pending'],
        ['Meta', 'pending'],
        ['WR / PF', 'pending'],
      ].map(([label, value]) => `
        <div class="landing-options-stat">
          <span>${label}</span>
          <strong>${value}</strong>
        </div>
      `).join('');
      noteEl.className = 'landing-options-note warn';
      dotEl.className = 'landing-options-dot warm';
      refreshTextEl.textContent = `${refreshTextEl.textContent || 'Last refresh pending'} · refresh delayed`;
      noteEl.textContent = `metrics unavailable · ${errMsg}`;
      scheduleOptionsBootstrapRetry();
      return false;
    } finally {
      clearTimeout(timeoutId);
      if (container.__optionsAbortController === controller) {
        container.__optionsAbortController = null;
      }
      container.__optionsLoadInFlight = false;
    }
  }

  loadOptionsStatus().then(ok => { if (ok) container.__optionsLastOk = true; });
  if (container.__optionsRefreshTimer) clearInterval(container.__optionsRefreshTimer);
  container.__optionsRefreshTimer = window.setInterval(() => {
    loadOptionsStatus().then(ok => {
      if (ok) container.__optionsLastOk = true;
    });
  }, OPTIONS_REFRESH_MS);

  return () => {
    unsubHealth();
    unsubModel();
    if (container.__optionsRefreshTimer) {
      clearInterval(container.__optionsRefreshTimer);
      container.__optionsRefreshTimer = null;
    }
    if (container.__optionsBootstrapRetryTimer) {
      clearTimeout(container.__optionsBootstrapRetryTimer);
      container.__optionsBootstrapRetryTimer = null;
    }
    if (container.__optionsAbortController) {
      try { container.__optionsAbortController.abort(); } catch {}
      container.__optionsAbortController = null;
    }
    container.__optionsLoadInFlight = false;
  };
}

window.AtlasLanding = { render };



