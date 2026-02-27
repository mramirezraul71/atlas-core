/**
 * ATLAS v4 — Robot Companion
 * 3D CSS animated robot that follows the user across views.
 * States: idle, thinking, speaking, listening, celebrating, sleeping
 */

const STATES = {
  IDLE: 'idle',
  THINKING: 'thinking',
  SPEAKING: 'speaking',
  LISTENING: 'listening',
  CELEBRATING: 'celebrating',
  SLEEPING: 'sleeping',
  GREETING: 'greeting',
};

let _el = null;
let _state = STATES.IDLE;
let _mouseX = 0.5;
let _mouseY = 0.5;
let _blinkTimer = null;
let _particleTimer = null;
let _collapsed = false;
let _dragOffset = null;
let _pos = { x: null, y: null };

function _injectStyles() {
  if (document.getElementById('atlas-companion-css')) return;
  const style = document.createElement('style');
  style.id = 'atlas-companion-css';
  style.textContent = `
    /* ═══════════ COMPANION CONTAINER ═══════════ */
    .atlas-companion {
      position: fixed;
      bottom: 80px;
      right: 20px;
      z-index: 5000;
      cursor: grab;
      user-select: none;
      transition: transform 0.3s cubic-bezier(0.34, 1.56, 0.64, 1);
    }
    .atlas-companion:active { cursor: grabbing; }
    .atlas-companion.collapsed .companion-body { transform: scale(0.5); opacity: 0.4; }
    .atlas-companion.collapsed:hover .companion-body { opacity: 0.8; transform: scale(0.7); }

    /* ═══════════ ROBOT BODY — 3D scene ═══════════ */
    .companion-scene {
      width: 80px;
      height: 100px;
      perspective: 400px;
    }
    .companion-body {
      position: relative;
      width: 80px;
      height: 100px;
      transform-style: preserve-3d;
      transition: transform 0.4s ease, opacity 0.3s;
      animation: companion-float 3s ease-in-out infinite;
    }

    /* ═══ HEAD ═══ */
    .companion-head {
      position: absolute;
      top: 0; left: 10px;
      width: 60px;
      height: 50px;
      background: linear-gradient(145deg, var(--bg-elevated, #21262d), var(--bg-card, #161b22));
      border: 2px solid var(--accent, #39d3c4);
      border-radius: 14px 14px 10px 10px;
      transform-style: preserve-3d;
      box-shadow:
        0 0 20px rgba(57,211,196,0.2),
        inset 0 -8px 16px rgba(0,0,0,0.3);
      transition: border-color 0.3s, box-shadow 0.3s;
    }

    /* Visor / face plate */
    .companion-visor {
      position: absolute;
      top: 8px; left: 6px; right: 6px;
      height: 26px;
      background: rgba(0,0,0,0.5);
      border-radius: 8px;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 12px;
      overflow: hidden;
    }

    /* ═══ EYES ═══ */
    .companion-eye {
      width: 10px;
      height: 10px;
      border-radius: 50%;
      background: var(--accent, #39d3c4);
      box-shadow: 0 0 8px var(--accent, #39d3c4), 0 0 16px rgba(57,211,196,0.4);
      transition: all 0.15s ease;
      position: relative;
    }
    .companion-eye .pupil {
      position: absolute;
      width: 4px;
      height: 4px;
      background: var(--bg, #0a0e14);
      border-radius: 50%;
      top: 3px;
      left: 3px;
      transition: all 0.1s ease;
    }

    /* Eye states */
    .companion-body[data-state="thinking"] .companion-eye {
      animation: eye-think 1.5s ease-in-out infinite alternate;
    }
    .companion-body[data-state="speaking"] .companion-eye {
      animation: eye-speak 0.3s ease-in-out infinite alternate;
    }
    .companion-body[data-state="sleeping"] .companion-eye {
      height: 2px;
      border-radius: 2px;
      opacity: 0.4;
    }
    .companion-body[data-state="celebrating"] .companion-eye {
      animation: eye-celebrate 0.4s ease infinite;
    }
    .companion-body[data-state="greeting"] .companion-eye {
      width: 12px; height: 12px;
      animation: eye-greet 0.6s ease;
    }
    .companion-eye.blink {
      height: 2px !important;
      border-radius: 2px !important;
      transition: height 0.05s ease;
    }

    /* Mouth / expression bar */
    .companion-mouth {
      position: absolute;
      bottom: 6px;
      left: 50%;
      transform: translateX(-50%);
      width: 16px;
      height: 3px;
      background: var(--accent, #39d3c4);
      border-radius: 2px;
      opacity: 0.6;
      transition: all 0.2s ease;
    }
    .companion-body[data-state="speaking"] .companion-mouth {
      animation: mouth-speak 0.2s ease-in-out infinite alternate;
    }
    .companion-body[data-state="thinking"] .companion-mouth {
      width: 8px;
      opacity: 0.3;
    }
    .companion-body[data-state="celebrating"] .companion-mouth {
      width: 20px;
      height: 8px;
      border-radius: 0 0 10px 10px;
      background: transparent;
      border: 2px solid var(--accent, #39d3c4);
      border-top: none;
    }
    .companion-body[data-state="sleeping"] .companion-mouth {
      width: 6px; opacity: 0.2;
    }

    /* ═══ ANTENNA ═══ */
    .companion-antenna {
      position: absolute;
      top: -12px;
      left: 50%;
      transform: translateX(-50%);
      width: 2px;
      height: 12px;
      background: var(--accent, #39d3c4);
    }
    .companion-antenna::after {
      content: '';
      position: absolute;
      top: -4px; left: -3px;
      width: 8px;
      height: 8px;
      border-radius: 50%;
      background: var(--accent, #39d3c4);
      box-shadow: 0 0 8px var(--accent, #39d3c4);
      animation: antenna-pulse 2s ease-in-out infinite;
    }
    .companion-body[data-state="thinking"] .companion-antenna::after {
      animation: antenna-think 0.5s ease infinite;
    }

    /* ═══ TORSO ═══ */
    .companion-torso {
      position: absolute;
      top: 48px; left: 16px;
      width: 48px;
      height: 30px;
      background: linear-gradient(180deg, var(--bg-elevated, #21262d), var(--bg-card, #161b22));
      border: 1.5px solid rgba(57,211,196,0.3);
      border-top: none;
      border-radius: 4px 4px 12px 12px;
      box-shadow: 0 4px 12px rgba(0,0,0,0.3);
    }
    .companion-core {
      position: absolute;
      top: 6px; left: 50%;
      transform: translateX(-50%);
      width: 12px;
      height: 12px;
      border-radius: 50%;
      background: var(--accent, #39d3c4);
      opacity: 0.6;
      animation: core-pulse 2s ease-in-out infinite;
    }
    .companion-body[data-state="thinking"] .companion-core {
      animation: core-think 0.8s ease infinite;
    }
    .companion-body[data-state="speaking"] .companion-core {
      animation: core-speak 0.3s ease infinite;
    }

    /* ═══ ARMS ═══ */
    .companion-arm {
      position: absolute;
      top: 52px;
      width: 6px;
      height: 22px;
      background: var(--bg-elevated, #21262d);
      border: 1px solid rgba(57,211,196,0.2);
      border-radius: 3px;
      transform-origin: top center;
    }
    .companion-arm.left  { left: 6px;  animation: arm-idle-l 3s ease-in-out infinite; }
    .companion-arm.right { right: 6px; animation: arm-idle-r 3s ease-in-out infinite; }
    .companion-body[data-state="greeting"] .companion-arm.right {
      animation: arm-wave 0.6s ease 3;
    }
    .companion-body[data-state="celebrating"] .companion-arm.left  { animation: arm-celebrate-l 0.5s ease infinite; }
    .companion-body[data-state="celebrating"] .companion-arm.right { animation: arm-celebrate-r 0.5s ease infinite; }

    /* ═══ PARTICLES (around robot) ═══ */
    .companion-particles {
      position: absolute;
      inset: -20px;
      pointer-events: none;
    }
    .companion-particle {
      position: absolute;
      width: 3px; height: 3px;
      background: var(--accent, #39d3c4);
      border-radius: 50%;
      opacity: 0;
      animation: particle-float 2s ease-out forwards;
    }

    /* ═══ SPEECH BUBBLE ═══ */
    .companion-bubble {
      position: absolute;
      bottom: 115px;
      right: 0;
      background: var(--bg-card, #161b22);
      border: 1px solid var(--border, #1e2a3a);
      border-radius: 12px 12px 4px 12px;
      padding: 8px 12px;
      font-size: 11px;
      color: var(--text, #e6edf3);
      max-width: 200px;
      box-shadow: 0 4px 16px rgba(0,0,0,0.4);
      opacity: 0;
      transform: translateY(8px) scale(0.9);
      transition: all 0.3s cubic-bezier(0.34, 1.56, 0.64, 1);
      pointer-events: none;
      font-family: var(--font, 'Inter', sans-serif);
    }
    .companion-bubble.show {
      opacity: 1;
      transform: translateY(0) scale(1);
    }
    .companion-bubble::after {
      content: '';
      position: absolute;
      bottom: -6px;
      right: 20px;
      width: 12px; height: 12px;
      background: var(--bg-card, #161b22);
      border-right: 1px solid var(--border, #1e2a3a);
      border-bottom: 1px solid var(--border, #1e2a3a);
      transform: rotate(45deg);
    }

    /* ═══ SOUND TOGGLE ═══ */
    .companion-sound-btn {
      position: absolute;
      bottom: -8px; left: -8px;
      width: 20px; height: 20px;
      border-radius: 50%;
      background: var(--bg-elevated, #21262d);
      border: 1px solid var(--border, #1e2a3a);
      color: var(--text-muted, #484f58);
      font-size: 10px;
      display: flex; align-items: center; justify-content: center;
      cursor: pointer;
      transition: all 0.2s;
      z-index: 10;
    }
    .companion-sound-btn:hover { color: var(--accent, #39d3c4); border-color: var(--accent, #39d3c4); }

    /* ═══════════ ANIMATIONS ═══════════ */
    @keyframes companion-float {
      0%, 100% { transform: translateY(0) rotateX(2deg); }
      50%      { transform: translateY(-6px) rotateX(-2deg); }
    }
    @keyframes eye-think {
      0%   { transform: translateX(-2px); }
      100% { transform: translateX(2px); }
    }
    @keyframes eye-speak {
      0%   { height: 10px; }
      100% { height: 6px; }
    }
    @keyframes eye-celebrate {
      0%, 100% { transform: scale(1); }
      50%      { transform: scale(1.3); }
    }
    @keyframes eye-greet {
      0%   { transform: scale(0.8); }
      50%  { transform: scale(1.3); }
      100% { transform: scale(1); }
    }
    @keyframes mouth-speak {
      0%   { width: 16px; height: 3px; }
      100% { width: 12px; height: 6px; border-radius: 3px; }
    }
    @keyframes antenna-pulse {
      0%, 100% { opacity: 1; box-shadow: 0 0 8px var(--accent, #39d3c4); }
      50%      { opacity: 0.5; box-shadow: 0 0 16px var(--accent, #39d3c4); }
    }
    @keyframes antenna-think {
      0%, 100% { opacity: 1; background: var(--accent, #39d3c4); }
      50%      { opacity: 0.3; background: var(--accent-orange, #d29922); }
    }
    @keyframes core-pulse {
      0%, 100% { opacity: 0.4; transform: translateX(-50%) scale(1); }
      50%      { opacity: 0.8; transform: translateX(-50%) scale(1.1); }
    }
    @keyframes core-think {
      0%, 100% { background: var(--accent, #39d3c4); opacity: 0.6; }
      50%      { background: var(--accent-orange, #d29922); opacity: 1; }
    }
    @keyframes core-speak {
      0%, 100% { opacity: 0.5; transform: translateX(-50%) scale(0.9); }
      50%      { opacity: 1; transform: translateX(-50%) scale(1.2); }
    }
    @keyframes arm-idle-l {
      0%, 100% { transform: rotate(3deg); }
      50%      { transform: rotate(-2deg); }
    }
    @keyframes arm-idle-r {
      0%, 100% { transform: rotate(-3deg); }
      50%      { transform: rotate(2deg); }
    }
    @keyframes arm-wave {
      0%, 100% { transform: rotate(-10deg); }
      50%      { transform: rotate(-60deg); }
    }
    @keyframes arm-celebrate-l {
      0%, 100% { transform: rotate(-20deg); }
      50%      { transform: rotate(-70deg); }
    }
    @keyframes arm-celebrate-r {
      0%, 100% { transform: rotate(20deg); }
      50%      { transform: rotate(70deg); }
    }
    @keyframes particle-float {
      0%   { opacity: 0.8; transform: translate(0, 0) scale(1); }
      100% { opacity: 0; transform: translate(var(--px, 20px), var(--py, -30px)) scale(0); }
    }

    /* State-based head tilt */
    .companion-body[data-state="thinking"] {
      animation: companion-float 3s ease-in-out infinite, think-tilt 2s ease-in-out infinite;
    }
    @keyframes think-tilt {
      0%, 100% { transform: translateY(0) rotateZ(0deg); }
      25%      { transform: translateY(-4px) rotateZ(-5deg); }
      75%      { transform: translateY(-4px) rotateZ(5deg); }
    }

    .companion-body[data-state="celebrating"] {
      animation: celebrate-bounce 0.4s ease infinite;
    }
    @keyframes celebrate-bounce {
      0%, 100% { transform: translateY(0) scale(1); }
      50%      { transform: translateY(-12px) scale(1.05); }
    }

    .companion-body[data-state="sleeping"] {
      animation: sleep-float 4s ease-in-out infinite;
    }
    @keyframes sleep-float {
      0%, 100% { transform: translateY(0) rotateZ(-3deg); }
      50%      { transform: translateY(-3px) rotateZ(3deg); }
    }

    /* Z bubble for sleeping */
    .companion-zzz {
      position: absolute;
      top: -10px; right: -10px;
      font-size: 14px;
      opacity: 0;
      pointer-events: none;
    }
    .companion-body[data-state="sleeping"] .companion-zzz {
      animation: zzz-float 2s ease-in-out infinite;
    }
    @keyframes zzz-float {
      0%   { opacity: 0; transform: translate(0, 0) scale(0.7); }
      50%  { opacity: 0.6; transform: translate(8px, -15px) scale(1); }
      100% { opacity: 0; transform: translate(16px, -30px) scale(0.5); }
    }
  `;
  document.head.appendChild(style);
}

function _buildHTML() {
  return `
    <div class="companion-bubble" id="companion-bubble"></div>
    <div class="companion-scene">
      <div class="companion-body" data-state="idle" id="companion-body">
        <div class="companion-antenna"></div>
        <div class="companion-head">
          <div class="companion-visor">
            <div class="companion-eye" id="eye-l"><div class="pupil" id="pupil-l"></div></div>
            <div class="companion-eye" id="eye-r"><div class="pupil" id="pupil-r"></div></div>
          </div>
          <div class="companion-mouth"></div>
        </div>
        <div class="companion-arm left"></div>
        <div class="companion-arm right"></div>
        <div class="companion-torso">
          <div class="companion-core"></div>
        </div>
        <div class="companion-zzz">z</div>
        <div class="companion-particles" id="companion-particles"></div>
      </div>
    </div>
    <button class="companion-sound-btn" id="companion-sound-btn" title="Toggle sounds">
      <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
        <polygon points="11 5 6 9 2 9 2 15 6 15 11 19 11 5"/>
        <path d="M15.54 8.46a5 5 0 010 7.07" id="sound-wave1"/>
        <path d="M19.07 4.93a10 10 0 010 14.14" id="sound-wave2"/>
      </svg>
    </button>
  `;
}

const GREETINGS = [
  'Hola, estoy aqui para ayudarte',
  'Atlas listo para trabajar',
  'A tus ordenes',
  'Sistema operativo',
  'Navegando contigo',
];

const THINKING_MSGS = [
  'Procesando...',
  'Analizando...',
  'Trabajando en eso...',
  'Dame un momento...',
  'Calculando...',
];

const DONE_MSGS = [
  'Listo!',
  'Tarea completada',
  'Hecho',
  'Finalizado',
  'Operacion exitosa',
];

let _bubbleTimer = null;

function _showBubble(text, duration = 3000) {
  const bubble = document.getElementById('companion-bubble');
  if (!bubble) return;
  clearTimeout(_bubbleTimer);
  bubble.textContent = text;
  bubble.classList.add('show');
  _bubbleTimer = setTimeout(() => bubble.classList.remove('show'), duration);
}

function _spawnParticle() {
  const container = document.getElementById('companion-particles');
  if (!container) return;
  const p = document.createElement('div');
  p.className = 'companion-particle';
  const angle = Math.random() * Math.PI * 2;
  const dist = 20 + Math.random() * 30;
  p.style.setProperty('--px', `${Math.cos(angle) * dist}px`);
  p.style.setProperty('--py', `${Math.sin(angle) * dist}px`);
  p.style.left = `${40 + Math.random() * 20 - 10}px`;
  p.style.top = `${30 + Math.random() * 20 - 10}px`;
  container.appendChild(p);
  setTimeout(() => p.remove(), 2000);
}

function _startBlink() {
  if (_blinkTimer) clearInterval(_blinkTimer);
  _blinkTimer = setInterval(() => {
    if (_state === STATES.SLEEPING) return;
    const eyes = _el?.querySelectorAll('.companion-eye');
    if (!eyes) return;
    eyes.forEach(e => e.classList.add('blink'));
    setTimeout(() => eyes.forEach(e => e.classList.remove('blink')), 100);
  }, 3000 + Math.random() * 2000);
}

function _trackMouse(e) {
  const rect = _el?.getBoundingClientRect();
  if (!rect) return;
  const cx = rect.left + rect.width / 2;
  const cy = rect.top + rect.height / 2;
  _mouseX = Math.max(-1, Math.min(1, (e.clientX - cx) / 300));
  _mouseY = Math.max(-1, Math.min(1, (e.clientY - cy) / 300));

  const pl = document.getElementById('pupil-l');
  const pr = document.getElementById('pupil-r');
  if (pl && pr && _state !== STATES.SLEEPING) {
    const ox = _mouseX * 2.5;
    const oy = _mouseY * 2;
    pl.style.transform = `translate(${ox}px, ${oy}px)`;
    pr.style.transform = `translate(${ox}px, ${oy}px)`;
  }
}

function _setupDrag() {
  let dragging = false;
  let startX, startY, origX, origY;

  _el.addEventListener('mousedown', (e) => {
    if (e.target.closest('.companion-sound-btn')) return;
    dragging = true;
    const rect = _el.getBoundingClientRect();
    startX = e.clientX;
    startY = e.clientY;
    origX = rect.left;
    origY = rect.top;
    _el.style.transition = 'none';
  });

  document.addEventListener('mousemove', (e) => {
    if (!dragging) return;
    const dx = e.clientX - startX;
    const dy = e.clientY - startY;
    _el.style.left = (origX + dx) + 'px';
    _el.style.top = (origY + dy) + 'px';
    _el.style.right = 'auto';
    _el.style.bottom = 'auto';
  });

  document.addEventListener('mouseup', () => {
    if (!dragging) return;
    dragging = false;
    _el.style.transition = '';
  });
}

export function mount(parent) {
  _injectStyles();
  _el = document.createElement('div');
  _el.className = 'atlas-companion';
  _el.id = 'atlas-companion';
  _el.innerHTML = _buildHTML();
  parent.appendChild(_el);

  _startBlink();
  document.addEventListener('mousemove', _trackMouse);
  _setupDrag();

  _el.addEventListener('click', (e) => {
    if (e.target.closest('.companion-sound-btn')) {
      const enabled = !window.AtlasSounds?.isEnabled();
      window.AtlasSounds?.toggle(enabled);
      const w1 = document.getElementById('sound-wave1');
      const w2 = document.getElementById('sound-wave2');
      if (w1) w1.style.opacity = enabled ? '1' : '0.2';
      if (w2) w2.style.opacity = enabled ? '1' : '0.2';
      _showBubble(enabled ? 'Sonido activado' : 'Sonido silenciado', 2000);
      return;
    }

    if (_collapsed) {
      _collapsed = false;
      _el.classList.remove('collapsed');
      setState(STATES.GREETING);
      window.AtlasSounds?.greeting();
      _showBubble(GREETINGS[Math.floor(Math.random() * GREETINGS.length)]);
      setTimeout(() => setState(STATES.IDLE), 2000);
    } else {
      window.AtlasSounds?.click();
      _showBubble(GREETINGS[Math.floor(Math.random() * GREETINGS.length)]);
    }
  });

  _el.addEventListener('dblclick', () => {
    _collapsed = !_collapsed;
    _el.classList.toggle('collapsed', _collapsed);
    if (_collapsed) _showBubble('Zzz...', 1500);
  });

  setTimeout(() => {
    setState(STATES.GREETING);
    window.AtlasSounds?.greeting();
    _showBubble(GREETINGS[0], 4000);
    setTimeout(() => setState(STATES.IDLE), 2500);
  }, 1000);
}

export function setState(newState) {
  _state = newState;
  const body = document.getElementById('companion-body');
  if (body) body.setAttribute('data-state', newState);

  if (newState === STATES.THINKING) {
    _showBubble(THINKING_MSGS[Math.floor(Math.random() * THINKING_MSGS.length)], 5000);
    if (!_particleTimer) {
      _particleTimer = setInterval(_spawnParticle, 400);
    }
  } else {
    if (_particleTimer) { clearInterval(_particleTimer); _particleTimer = null; }
  }

  if (newState === STATES.CELEBRATING) {
    _showBubble(DONE_MSGS[Math.floor(Math.random() * DONE_MSGS.length)]);
    for (let i = 0; i < 8; i++) setTimeout(_spawnParticle, i * 100);
  }
}

export function getState() { return _state; }

export function say(text, duration = 3000) {
  _showBubble(text, duration);
}

export function onNavigate(route) {
  window.AtlasSounds?.navigate();
  const body = document.getElementById('companion-body');
  if (body) {
    body.style.animation = 'none';
    body.offsetHeight;
    body.style.animation = '';
  }
  if (route === '/') {
    say('Bienvenido al inicio', 2000);
  } else if (route.startsWith('/ask')) {
    say('Asistente IA listo', 2000);
  } else if (route.includes('health')) {
    say('Estado del sistema', 2000);
  } else if (route.includes('trading')) {
    say('Trading pronto disponible', 2000);
  }
}

export function onUserType() {
  if (_state !== STATES.LISTENING) setState(STATES.LISTENING);
  window.AtlasSounds?.typing();
}

export function onAgentThink() {
  setState(STATES.THINKING);
  window.AtlasSounds?.thinking();
}

export function onAgentSpeak() {
  setState(STATES.SPEAKING);
}

export function onAgentDone(success) {
  if (success) {
    setState(STATES.CELEBRATING);
    window.AtlasSounds?.success();
    setTimeout(() => setState(STATES.IDLE), 3000);
  } else {
    window.AtlasSounds?.error();
    say('Algo salio mal', 2000);
    setTimeout(() => setState(STATES.IDLE), 2000);
  }
}

export function sleep() { setState(STATES.SLEEPING); }
export function wake()  { setState(STATES.IDLE); window.AtlasSounds?.click(); }

window.AtlasCompanion = {
  mount, setState, getState, say, onNavigate,
  onUserType, onAgentThink, onAgentSpeak, onAgentDone,
  sleep, wake, STATES,
};
