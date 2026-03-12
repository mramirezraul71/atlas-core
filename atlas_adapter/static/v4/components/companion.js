/**
 * ATLAS v4 — Robot Companion  v2.0
 * True-3D CSS robot that follows the user across views.
 * States: idle, thinking, speaking, listening, celebrating, sleeping, greeting
 * New in v2: 3D head tracking (rotateX/Y), per-state 3D animations, Web Speech API TTS.
 */

const STATES = {
  IDLE:        'idle',
  THINKING:    'thinking',
  SPEAKING:    'speaking',
  LISTENING:   'listening',
  CELEBRATING: 'celebrating',
  SLEEPING:    'sleeping',
  GREETING:    'greeting',
};

let _el           = null;
let _state        = STATES.IDLE;
let _mouseX       = 0;
let _mouseY       = 0;
let _blinkTimer   = null;
let _particleTimer= null;
let _collapsed    = false;
let _runtimeTimer = null;
let _runtimeMode  = '';
let _soundEnabled = true;
let _bubbleTimer  = null;
let _voices       = [];

// Load TTS voices (Chrome requires voiceschanged event)
if (typeof window !== 'undefined' && window.speechSynthesis) {
  _voices = speechSynthesis.getVoices();
  speechSynthesis.addEventListener('voiceschanged', () => {
    _voices = speechSynthesis.getVoices();
  });
}

// ─── WEB SPEECH API TTS ──────────────────────────────────────────────────────
function _speak(text) {
  if (!_soundEnabled || !window.speechSynthesis) return;
  speechSynthesis.cancel();
  const utter = new SpeechSynthesisUtterance(text);
  utter.lang   = 'es-ES';
  utter.rate   = 1.05;
  utter.pitch  = 1.1;
  utter.volume = 0.85;
  const esVoice = _voices.find(v => v.lang && v.lang.startsWith('es')) || null;
  if (esVoice) utter.voice = esVoice;
  utter.onstart = () => { if (_state !== STATES.SPEAKING && _state !== STATES.CELEBRATING) setState(STATES.SPEAKING); };
  utter.onend   = () => { if (_state === STATES.SPEAKING) setState(STATES.IDLE); };
  speechSynthesis.speak(utter);
}

// ─── STYLES ───────────────────────────────────────────────────────────────────
function _injectStyles() {
  if (document.getElementById('atlas-companion-css')) return;
  const style = document.createElement('style');
  style.id = 'atlas-companion-css';
  style.textContent = `
    /* ═══════════ CONTAINER ═══════════ */
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

    /* ═══════════ 3D SCENE ═══════════ */
    .companion-scene {
      width: 80px;
      height: 100px;
      perspective: 280px;
      perspective-origin: 50% 15%;
    }
    .companion-body {
      position: relative;
      width: 80px;
      height: 100px;
      transform-style: preserve-3d;
      transition: opacity 0.3s;
      animation: companion-float 3.5s ease-in-out infinite;
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
      transform: translateZ(10px);
      box-shadow:
        0 0 20px rgba(57,211,196,0.2),
        inset 0 -8px 16px rgba(0,0,0,0.3);
      transition: border-color 0.3s, box-shadow 0.3s;
    }

    /* Per-state 3D head animations */
    .companion-body[data-state="thinking"]    .companion-head { animation: head-think-3d  2.8s ease-in-out infinite; }
    .companion-body[data-state="speaking"]    .companion-head { animation: head-speak-nod  0.5s ease-in-out infinite alternate; }
    .companion-body[data-state="sleeping"]    .companion-head { animation: head-sleep-droop 4s  ease-in-out infinite; }
    .companion-body[data-state="celebrating"] .companion-head { animation: head-celebrate   0.45s ease infinite; }

    /* ═══ VISOR / EYES ═══ */
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
    .companion-eye {
      width: 10px; height: 10px;
      border-radius: 50%;
      background: var(--accent, #39d3c4);
      box-shadow: 0 0 8px var(--accent, #39d3c4), 0 0 16px rgba(57,211,196,0.4);
      transition: all 0.15s ease;
      position: relative;
    }
    .companion-eye .pupil {
      position: absolute;
      width: 4px; height: 4px;
      background: var(--bg, #0a0e14);
      border-radius: 50%;
      top: 3px; left: 3px;
      transition: all 0.08s ease;
    }
    .companion-body[data-state="thinking"]    .companion-eye { animation: eye-think     1.5s ease-in-out infinite alternate; }
    .companion-body[data-state="speaking"]    .companion-eye { animation: eye-speak     0.3s ease-in-out infinite alternate; }
    .companion-body[data-state="sleeping"]    .companion-eye { height: 2px; border-radius: 2px; opacity: 0.4; }
    .companion-body[data-state="celebrating"] .companion-eye { animation: eye-celebrate 0.4s ease infinite; }
    .companion-body[data-state="greeting"]    .companion-eye { width: 12px; height: 12px; animation: eye-greet 0.6s ease; }
    .companion-eye.blink { height: 2px !important; border-radius: 2px !important; transition: height 0.05s ease; }

    /* ═══ MOUTH ═══ */
    .companion-mouth {
      position: absolute;
      bottom: 6px; left: 50%;
      transform: translateX(-50%);
      width: 16px; height: 3px;
      background: var(--accent, #39d3c4);
      border-radius: 2px;
      opacity: 0.6;
      transition: all 0.2s ease;
    }
    .companion-body[data-state="speaking"]    .companion-mouth { animation: mouth-speak 0.2s ease-in-out infinite alternate; }
    .companion-body[data-state="thinking"]    .companion-mouth { width: 8px; opacity: 0.3; }
    .companion-body[data-state="celebrating"] .companion-mouth { width: 20px; height: 8px; border-radius: 0 0 10px 10px; background: transparent; border: 2px solid var(--accent, #39d3c4); border-top: none; }
    .companion-body[data-state="sleeping"]    .companion-mouth { width: 6px; opacity: 0.2; }

    /* ═══ ANTENNA ═══ */
    .companion-antenna {
      position: absolute;
      top: -12px; left: 50%;
      transform: translateX(-50%) translateZ(12px);
      width: 2px; height: 12px;
      background: var(--accent, #39d3c4);
    }
    .companion-antenna::after {
      content: '';
      position: absolute;
      top: -4px; left: -3px;
      width: 8px; height: 8px;
      border-radius: 50%;
      background: var(--accent, #39d3c4);
      box-shadow: 0 0 8px var(--accent, #39d3c4);
      animation: antenna-pulse 2s ease-in-out infinite;
    }
    .companion-body[data-state="thinking"] .companion-antenna::after { animation: antenna-think 0.5s ease infinite; }

    /* ═══ TORSO ═══ */
    .companion-torso {
      position: absolute;
      top: 48px; left: 16px;
      width: 48px; height: 30px;
      background: linear-gradient(180deg, var(--bg-elevated, #21262d), var(--bg-card, #161b22));
      border: 1.5px solid rgba(57,211,196,0.3);
      border-top: none;
      border-radius: 4px 4px 12px 12px;
      box-shadow: 0 4px 12px rgba(0,0,0,0.3);
      transform: translateZ(-5px);
    }
    .companion-core {
      position: absolute;
      top: 6px; left: 50%;
      transform: translateX(-50%);
      width: 12px; height: 12px;
      border-radius: 50%;
      background: var(--accent, #39d3c4);
      opacity: 0.6;
      animation: core-pulse 2s ease-in-out infinite;
    }
    .companion-body[data-state="thinking"] .companion-core { animation: core-think 0.8s ease infinite; }
    .companion-body[data-state="speaking"] .companion-core { animation: core-speak 0.3s ease infinite; }

    /* ═══ ARMS ═══ */
    .companion-arm {
      position: absolute;
      top: 52px;
      width: 6px; height: 22px;
      background: var(--bg-elevated, #21262d);
      border: 1px solid rgba(57,211,196,0.2);
      border-radius: 3px;
      transform-origin: top center;
      transform-style: preserve-3d;
    }
    .companion-arm.left  { left: 6px;  transform: translateZ(3px) rotate(3deg);  animation: arm-idle-l 3s ease-in-out infinite; }
    .companion-arm.right { right: 6px; transform: translateZ(3px) rotate(-3deg); animation: arm-idle-r 3s ease-in-out infinite; }
    .companion-body[data-state="greeting"]    .companion-arm.right { animation: arm-wave        0.6s ease 3; }
    .companion-body[data-state="celebrating"] .companion-arm.left  { animation: arm-celebrate-l 0.5s ease infinite; }
    .companion-body[data-state="celebrating"] .companion-arm.right { animation: arm-celebrate-r 0.5s ease infinite; }

    /* ═══ PARTICLES ═══ */
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
      bottom: 115px; right: 0;
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
    .companion-bubble.show { opacity: 1; transform: translateY(0) scale(1); }
    .companion-bubble::after {
      content: '';
      position: absolute;
      bottom: -6px; right: 20px;
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

    /* ═══ ZZZ ═══ */
    .companion-zzz {
      position: absolute;
      top: -10px; right: -10px;
      font-size: 14px;
      opacity: 0;
      pointer-events: none;
    }
    .companion-body[data-state="sleeping"] .companion-zzz { animation: zzz-float 2s ease-in-out infinite; }

    /* ═══════════ KEYFRAMES ═══════════ */

    /* BODY — float with subtle 3D rotation */
    @keyframes companion-float {
      0%   { transform: translateY(0)    rotateX(1deg)  rotateY(0deg); }
      30%  { transform: translateY(-5px) rotateX(-1deg) rotateY(4deg); }
      65%  { transform: translateY(-7px) rotateX(-2deg) rotateY(-4deg); }
      100% { transform: translateY(0)    rotateX(1deg)  rotateY(0deg); }
    }

    /* HEAD — 3D per-state */
    @keyframes head-think-3d {
      0%   { transform: translateZ(10px) rotateY(-20deg) rotateX(6deg); }
      30%  { transform: translateZ(10px) rotateY(14deg)  rotateX(-4deg); }
      65%  { transform: translateZ(10px) rotateY(24deg)  rotateX(10deg); }
      100% { transform: translateZ(10px) rotateY(-20deg) rotateX(6deg); }
    }
    @keyframes head-speak-nod {
      0%   { transform: translateZ(10px) rotateX(-10deg) rotateY(-4deg); }
      100% { transform: translateZ(10px) rotateX(6deg)   rotateY(4deg); }
    }
    @keyframes head-sleep-droop {
      0%, 100% { transform: translateZ(10px) rotateX(-28deg) rotateZ(-6deg); }
      50%      { transform: translateZ(10px) rotateX(-22deg) rotateZ(6deg); }
    }
    @keyframes head-celebrate {
      0%, 100% { transform: translateZ(10px) rotateY(-12deg) rotateX(-6deg); }
      50%      { transform: translateZ(10px) rotateY(12deg)  rotateX(6deg); }
    }

    /* BODY — state overrides */
    .companion-body[data-state="thinking"] {
      animation: companion-float 3.5s ease-in-out infinite, think-sway 2.2s ease-in-out infinite;
    }
    @keyframes think-sway {
      0%, 100% { transform: translateY(0)   rotateZ(0deg); }
      25%      { transform: translateY(-4px) rotateZ(-4deg); }
      75%      { transform: translateY(-4px) rotateZ(4deg); }
    }
    .companion-body[data-state="celebrating"] {
      animation: celebrate-spin 0.8s ease-in-out infinite;
    }
    @keyframes celebrate-spin {
      0%   { transform: translateY(0)    rotateY(0deg)   scale(1); }
      50%  { transform: translateY(-14px) rotateY(180deg) scale(1.06); }
      100% { transform: translateY(0)    rotateY(360deg) scale(1); }
    }
    .companion-body[data-state="sleeping"] {
      animation: sleep-float 4.5s ease-in-out infinite;
    }
    @keyframes sleep-float {
      0%, 100% { transform: translateY(0)   rotateZ(-4deg); }
      50%      { transform: translateY(-3px) rotateZ(4deg); }
    }

    /* EYES */
    @keyframes eye-think     { 0% { transform: translateX(-2px); } 100% { transform: translateX(2px); } }
    @keyframes eye-speak     { 0% { height: 10px; } 100% { height: 6px; } }
    @keyframes eye-celebrate { 0%, 100% { transform: scale(1); } 50% { transform: scale(1.3); } }
    @keyframes eye-greet     { 0% { transform: scale(0.8); } 50% { transform: scale(1.3); } 100% { transform: scale(1); } }

    /* MOUTH */
    @keyframes mouth-speak { 0% { width: 16px; height: 3px; } 100% { width: 12px; height: 6px; border-radius: 3px; } }

    /* ANTENNA */
    @keyframes antenna-pulse { 0%, 100% { opacity: 1; box-shadow: 0 0 8px var(--accent, #39d3c4); } 50% { opacity: 0.5; box-shadow: 0 0 18px var(--accent, #39d3c4); } }
    @keyframes antenna-think { 0%, 100% { opacity: 1; background: var(--accent, #39d3c4); } 50% { opacity: 0.3; background: var(--accent-orange, #d29922); } }

    /* CORE */
    @keyframes core-pulse { 0%, 100% { opacity: 0.4; transform: translateX(-50%) scale(1); } 50% { opacity: 0.8; transform: translateX(-50%) scale(1.1); } }
    @keyframes core-think { 0%, 100% { background: var(--accent, #39d3c4); opacity: 0.6; } 50% { background: var(--accent-orange, #d29922); opacity: 1; } }
    @keyframes core-speak { 0%, 100% { opacity: 0.5; transform: translateX(-50%) scale(0.9); } 50% { opacity: 1; transform: translateX(-50%) scale(1.2); } }

    /* ARMS */
    @keyframes arm-idle-l     { 0%, 100% { transform: translateZ(3px) rotate(3deg);   } 50% { transform: translateZ(3px) rotate(-2deg); } }
    @keyframes arm-idle-r     { 0%, 100% { transform: translateZ(3px) rotate(-3deg);  } 50% { transform: translateZ(3px) rotate(2deg);  } }
    @keyframes arm-wave       { 0%, 100% { transform: translateZ(3px) rotate(-10deg); } 50% { transform: translateZ(3px) rotate(-65deg); } }
    @keyframes arm-celebrate-l{ 0%, 100% { transform: translateZ(3px) rotate(-20deg); } 50% { transform: translateZ(3px) rotate(-75deg); } }
    @keyframes arm-celebrate-r{ 0%, 100% { transform: translateZ(3px) rotate(20deg);  } 50% { transform: translateZ(3px) rotate(75deg);  } }

    /* PARTICLES */
    @keyframes particle-float { 0% { opacity: 0.8; transform: translate(0,0) scale(1); } 100% { opacity: 0; transform: translate(var(--px,20px), var(--py,-30px)) scale(0); } }

    /* ZZZ */
    @keyframes zzz-float { 0% { opacity: 0; transform: translate(0,0) scale(0.7); } 50% { opacity: 0.6; transform: translate(8px,-15px) scale(1); } 100% { opacity: 0; transform: translate(16px,-30px) scale(0.5); } }
  `;
  document.head.appendChild(style);
}

// ─── HTML TEMPLATE ────────────────────────────────────────────────────────────
function _buildHTML() {
  return `
    <div class="companion-bubble" id="companion-bubble"></div>
    <div class="companion-scene">
      <div class="companion-body" data-state="idle" id="companion-body">
        <div class="companion-antenna"></div>
        <div class="companion-head" id="companion-head">
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
    <button class="companion-sound-btn" id="companion-sound-btn" title="Toggle voz">
      <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
        <polygon points="11 5 6 9 2 9 2 15 6 15 11 19 11 5"/>
        <path d="M15.54 8.46a5 5 0 010 7.07" id="sound-wave1"/>
        <path d="M19.07 4.93a10 10 0 010 14.14" id="sound-wave2"/>
      </svg>
    </button>
  `;
}

// ─── MESSAGES ─────────────────────────────────────────────────────────────────
const GREETINGS    = ['Hola, estoy aqui para ayudarte', 'Atlas listo para trabajar', 'A tus ordenes', 'Sistema operativo', 'Navegando contigo'];
const THINKING_MSGS= ['Procesando...', 'Analizando...', 'Trabajando en eso...', 'Dame un momento...', 'Calculando...'];
const DONE_MSGS    = ['Listo!', 'Tarea completada', 'Hecho', 'Finalizado', 'Operacion exitosa'];

// ─── BUBBLE ───────────────────────────────────────────────────────────────────
function _showBubble(text, duration = 3000, speak = false) {
  const bubble = document.getElementById('companion-bubble');
  if (!bubble) return;
  clearTimeout(_bubbleTimer);
  bubble.textContent = text;
  bubble.classList.add('show');
  _bubbleTimer = setTimeout(() => bubble.classList.remove('show'), duration);
  if (speak) _speak(text);
}

// ─── PARTICLES ────────────────────────────────────────────────────────────────
function _spawnParticle() {
  const container = document.getElementById('companion-particles');
  if (!container) return;
  const p = document.createElement('div');
  p.className = 'companion-particle';
  const angle = Math.random() * Math.PI * 2;
  const dist  = 20 + Math.random() * 30;
  p.style.setProperty('--px', `${Math.cos(angle) * dist}px`);
  p.style.setProperty('--py', `${Math.sin(angle) * dist}px`);
  p.style.left = `${40 + Math.random() * 20 - 10}px`;
  p.style.top  = `${30 + Math.random() * 20 - 10}px`;
  container.appendChild(p);
  setTimeout(() => p.remove(), 2000);
}

// ─── BLINK ────────────────────────────────────────────────────────────────────
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

// ─── MOUSE TRACKING — 3D head rotation ────────────────────────────────────────
const _TRACK_STATES = new Set([STATES.IDLE, STATES.LISTENING, STATES.GREETING]);

function _trackMouse(e) {
  const rect = _el?.getBoundingClientRect();
  if (!rect) return;
  const cx = rect.left + rect.width / 2;
  const cy = rect.top  + rect.height / 2;
  _mouseX = Math.max(-1, Math.min(1, (e.clientX - cx) / 300));
  _mouseY = Math.max(-1, Math.min(1, (e.clientY - cy) / 300));

  // Pupils track mouse
  const pl = document.getElementById('pupil-l');
  const pr = document.getElementById('pupil-r');
  if (pl && pr && _state !== STATES.SLEEPING) {
    const ox = _mouseX * 2.5;
    const oy = _mouseY * 2;
    pl.style.transform = `translate(${ox}px, ${oy}px)`;
    pr.style.transform = `translate(${ox}px, ${oy}px)`;
  }

  // Head rotates in 3D toward cursor (only in non-animated states)
  if (_TRACK_STATES.has(_state)) {
    const head = document.getElementById('companion-head');
    if (head) {
      const ry = _mouseX * 24;   // ±24° horizontal
      const rx = _mouseY * 16;   // ±16° vertical
      head.style.transform = `translateZ(10px) rotateY(${ry}deg) rotateX(${rx}deg)`;
    }
  }
}

// ─── DRAG ─────────────────────────────────────────────────────────────────────
function _setupDrag() {
  let dragging = false;
  let startX, startY, origX, origY;

  _el.addEventListener('mousedown', (e) => {
    if (e.target.closest('.companion-sound-btn')) return;
    dragging = true;
    const rect = _el.getBoundingClientRect();
    startX = e.clientX; startY = e.clientY;
    origX  = rect.left; origY  = rect.top;
    _el.style.transition = 'none';
  });
  document.addEventListener('mousemove', (e) => {
    if (!dragging) return;
    _el.style.left   = (origX + e.clientX - startX) + 'px';
    _el.style.top    = (origY + e.clientY - startY) + 'px';
    _el.style.right  = 'auto';
    _el.style.bottom = 'auto';
  });
  document.addEventListener('mouseup', () => {
    if (!dragging) return;
    dragging = false;
    _el.style.transition = '';
  });
}

// ─── RUNWAY CONTEXT SYNC ──────────────────────────────────────────────────────
function _mapRunwayModeToState(mode, suggested) {
  const s = String(suggested || '').toLowerCase();
  if (Object.values(STATES).includes(s)) return s;
  if (mode === 'alert')   return STATES.SPEAKING;
  if (mode === 'focused') return STATES.THINKING;
  return STATES.IDLE;
}

function _syncRunwayContextOnce() {
  fetch('/api/avatar/runway/context')
    .then(r => { if (!r.ok) throw 0; return r.json(); })
    .then(data => {
      if (!data?.ok) return;
      const rt   = data.runtime || {};
      const mode = String(rt.mode || '').toLowerCase();
      const target = _mapRunwayModeToState(mode, rt.companion_state);
      const transient = new Set([STATES.GREETING, STATES.CELEBRATING, STATES.SLEEPING]);
      if (!transient.has(_state) && target && _state !== target) setState(target);
      if (mode && mode !== _runtimeMode) {
        _runtimeMode = mode;
        if (rt.line_template && !_collapsed) _showBubble(String(rt.line_template), 2600);
      }
    })
    .catch(() => {});
}

// ─── MOUNT ────────────────────────────────────────────────────────────────────
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
    // Sound/voice toggle
    if (e.target.closest('.companion-sound-btn')) {
      _soundEnabled = !_soundEnabled;
      if (!_soundEnabled) speechSynthesis?.cancel();
      window.AtlasSounds?.toggle(_soundEnabled);
      const w1 = document.getElementById('sound-wave1');
      const w2 = document.getElementById('sound-wave2');
      if (w1) w1.style.opacity = _soundEnabled ? '1' : '0.2';
      if (w2) w2.style.opacity = _soundEnabled ? '1' : '0.2';
      _showBubble(_soundEnabled ? 'Voz activada' : 'Voz silenciada', 2000, _soundEnabled);
      return;
    }

    if (_collapsed) {
      _collapsed = false;
      _el.classList.remove('collapsed');
      setState(STATES.GREETING);
      window.AtlasSounds?.greeting();
      _showBubble(GREETINGS[Math.floor(Math.random() * GREETINGS.length)], 3000, true);
      setTimeout(() => setState(STATES.IDLE), 2500);
    } else {
      window.AtlasSounds?.click();
      _showBubble(GREETINGS[Math.floor(Math.random() * GREETINGS.length)], 3000, true);
    }
  });

  _el.addEventListener('dblclick', () => {
    _collapsed = !_collapsed;
    _el.classList.toggle('collapsed', _collapsed);
    if (_collapsed) { speechSynthesis?.cancel(); _showBubble('Zzz...', 1500); }
  });

  // Greeting on startup
  setTimeout(() => {
    setState(STATES.GREETING);
    window.AtlasSounds?.greeting();
    _showBubble(GREETINGS[0], 4000, true);
    setTimeout(() => setState(STATES.IDLE), 2500);
  }, 1200);

  setTimeout(_syncRunwayContextOnce, 2500);
  if (_runtimeTimer) clearInterval(_runtimeTimer);
  _runtimeTimer = setInterval(_syncRunwayContextOnce, 20000);
}

// ─── SET STATE ────────────────────────────────────────────────────────────────
export function setState(newState) {
  _state = newState;
  const body = document.getElementById('companion-body');
  if (body) body.setAttribute('data-state', newState);

  // Clear inline head transform → let CSS animation take over for animated states
  const head = document.getElementById('companion-head');
  if (head) {
    const ANIM_STATES = new Set([STATES.THINKING, STATES.SPEAKING, STATES.SLEEPING, STATES.CELEBRATING]);
    if (ANIM_STATES.has(newState)) {
      head.style.transform = '';   // CSS animation takes control
    } else {
      head.style.transform = 'translateZ(10px)';  // base 3D depth for tracking
    }
  }

  if (newState === STATES.THINKING) {
    _showBubble(THINKING_MSGS[Math.floor(Math.random() * THINKING_MSGS.length)], 5000);
    if (!_particleTimer) _particleTimer = setInterval(_spawnParticle, 400);
  } else {
    if (_particleTimer) { clearInterval(_particleTimer); _particleTimer = null; }
  }

  if (newState === STATES.CELEBRATING) {
    _showBubble(DONE_MSGS[Math.floor(Math.random() * DONE_MSGS.length)], 3000, true);
    for (let i = 0; i < 8; i++) setTimeout(_spawnParticle, i * 100);
  }
}

// ─── PUBLIC API ───────────────────────────────────────────────────────────────
export function getState() { return _state; }

export function say(text, duration = 3000, speak = false) {
  _showBubble(text, duration, speak);
}

export function onNavigate(route) {
  window.AtlasSounds?.navigate();
  const body = document.getElementById('companion-body');
  if (body) { body.style.animation = 'none'; body.offsetHeight; body.style.animation = ''; }
  if (route === '/')               say('Bienvenido al inicio', 2000, true);
  else if (route.startsWith('/ask')) say('Asistente IA listo', 2000, true);
  else if (route.includes('health')) say('Estado del sistema', 2000);
  else if (route.includes('trading')) say('Trading pronto disponible', 2000);
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
  // Only set SPEAKING if TTS isn't already controlling it
  if (_state !== STATES.SPEAKING) setState(STATES.SPEAKING);
}

export function onAgentDone(success) {
  if (success) {
    setState(STATES.CELEBRATING);
    window.AtlasSounds?.success();
    setTimeout(() => setState(STATES.IDLE), 3000);
  } else {
    window.AtlasSounds?.error();
    say('Algo salio mal', 2000, true);
    setTimeout(() => setState(STATES.IDLE), 2000);
  }
}

export function sleep() { setState(STATES.SLEEPING); speechSynthesis?.cancel(); }
export function wake()  { setState(STATES.IDLE); window.AtlasSounds?.click(); }

window.AtlasCompanion = {
  mount, setState, getState, say, onNavigate,
  onUserType, onAgentThink, onAgentSpeak, onAgentDone,
  sleep, wake, STATES,
};
