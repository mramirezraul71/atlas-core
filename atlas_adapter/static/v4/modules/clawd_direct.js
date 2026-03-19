// ─── ATLAS Directo — Cursor-style AI Interface ───────────────────────────────

function _esc(s) {
  const d = document.createElement('span');
  d.textContent = s ?? '';
  return d.innerHTML;
}
function _pickReply(data) {
  if (!data || typeof data !== 'object') return '';
  return String(data.reply || data.response || data.text || data.message || '').trim();
}

// Simple markdown → HTML (code blocks, inline code, bold, italic, links)
function _md(text) {
  if (!text) return '';
  let h = _esc(text);
  // Fenced code blocks
  h = h.replace(/```(\w*)\n?([\s\S]*?)```/g, (_, lang, code) =>
    `<div class="cd-code-block"><div class="cd-code-lang">${lang || 'code'}</div><pre><code>${code.trim()}</code></pre><button class="cd-copy-btn" onclick="navigator.clipboard.writeText(this.parentElement.querySelector('code').innerText)">⎘ Copiar</button></div>`
  );
  // Inline code
  h = h.replace(/`([^`]+)`/g, '<code class="cd-inline-code">$1</code>');
  // Bold
  h = h.replace(/\*\*([^*]+)\*\*/g, '<strong>$1</strong>');
  // Italic
  h = h.replace(/\*([^*]+)\*/g, '<em>$1</em>');
  // Line breaks
  h = h.replace(/\n/g, '<br>');
  return h;
}

const CSS = `
<style id="cd-styles">
.cd-root {
  height: 100%;
  display: flex;
  flex-direction: column;
  background: #0d0d12;
  font-family: 'Inter', -apple-system, sans-serif;
  color: #e2e8f0;
  overflow: hidden;
}

/* ── Top bar ── */
.cd-topbar {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 10px 16px;
  background: #111118;
  border-bottom: 1px solid #1e1e2e;
  flex-shrink: 0;
}
.cd-back {
  display: flex;
  align-items: center;
  gap: 6px;
  color: #94a3b8;
  font-size: 13px;
  cursor: pointer;
  padding: 4px 8px;
  border-radius: 6px;
  border: none;
  background: none;
  transition: color .15s, background .15s;
}
.cd-back:hover { color: #e2e8f0; background: #1e1e2e; }
.cd-title {
  font-size: 14px;
  font-weight: 600;
  color: #e2e8f0;
  display: flex;
  align-items: center;
  gap: 8px;
}
.cd-title-icon {
  width: 28px; height: 28px;
  background: linear-gradient(135deg, #00d4aa, #0099ff);
  border-radius: 8px;
  display: flex; align-items: center; justify-content: center;
  font-size: 14px;
}
.cd-spacer { flex: 1; }
.cd-pills { display: flex; gap: 8px; }
.cd-pill {
  display: flex; align-items: center; gap: 6px;
  padding: 4px 10px;
  border-radius: 20px;
  font-size: 11px;
  font-weight: 600;
  border: 1px solid transparent;
}
.cd-pill.ok   { background: #052e1b; border-color: #14532d; color: #4ade80; }
.cd-pill.warn { background: #2d1f00; border-color: #713f12; color: #fb923c; }
.cd-pill.err  { background: #2d0a0a; border-color: #7f1d1d; color: #f87171; }
.cd-pill.dim  { background: #1a1a2e; border-color: #2e2e4e; color: #64748b; }
.cd-pill-dot  { width: 6px; height: 6px; border-radius: 50%; background: currentColor; }

/* ── Layout: sidebar + main ── */
.cd-body {
  display: flex;
  flex: 1;
  overflow: hidden;
}

/* ── Sidebar ── */
.cd-sidebar {
  width: 220px;
  min-width: 220px;
  background: #0d0d12;
  border-right: 1px solid #1e1e2e;
  display: flex;
  flex-direction: column;
  overflow-y: auto;
  flex-shrink: 0;
}
.cd-sidebar-section {
  padding: 12px 14px 6px;
  font-size: 10px;
  font-weight: 700;
  letter-spacing: .08em;
  color: #475569;
  text-transform: uppercase;
}
.cd-sidebar-item {
  display: flex; align-items: center; gap: 8px;
  padding: 7px 14px;
  font-size: 12px;
  color: #94a3b8;
  cursor: pointer;
  border-left: 2px solid transparent;
  transition: all .15s;
}
.cd-sidebar-item:hover { color: #e2e8f0; background: #111118; }
.cd-sidebar-item.active { color: #00d4aa; border-left-color: #00d4aa; background: #051c16; }
.cd-sidebar-item .cd-si-icon { font-size: 13px; width: 16px; text-align: center; }

.cd-ctx-box {
  margin: 8px 10px;
  padding: 10px;
  background: #111118;
  border: 1px solid #1e1e2e;
  border-radius: 8px;
  font-size: 11px;
  color: #64748b;
}
.cd-ctx-row { display: flex; justify-content: space-between; padding: 2px 0; }
.cd-ctx-val { color: #94a3b8; font-weight: 500; }

/* ── Chat main ── */
.cd-main {
  flex: 1;
  display: flex;
  flex-direction: column;
  overflow: hidden;
}

/* ── Messages ── */
.cd-messages {
  flex: 1;
  overflow-y: auto;
  padding: 20px 24px;
  display: flex;
  flex-direction: column;
  gap: 16px;
  scroll-behavior: smooth;
}
.cd-messages::-webkit-scrollbar { width: 4px; }
.cd-messages::-webkit-scrollbar-track { background: transparent; }
.cd-messages::-webkit-scrollbar-thumb { background: #2e2e4e; border-radius: 2px; }

/* Welcome */
.cd-welcome {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: 12px;
  padding: 40px 20px;
  color: #475569;
  text-align: center;
}
.cd-welcome-icon { font-size: 40px; }
.cd-welcome h3 { font-size: 16px; font-weight: 600; color: #64748b; margin: 0; }
.cd-welcome p { font-size: 13px; margin: 0; max-width: 300px; line-height: 1.6; }
.cd-welcome-chips { display: flex; flex-wrap: wrap; gap: 8px; justify-content: center; margin-top: 8px; }
.cd-welcome-chip {
  padding: 6px 12px;
  background: #111118;
  border: 1px solid #1e1e2e;
  border-radius: 20px;
  font-size: 12px;
  color: #94a3b8;
  cursor: pointer;
  transition: all .15s;
}
.cd-welcome-chip:hover { border-color: #00d4aa; color: #00d4aa; background: #051c16; }

/* Message bubbles */
.cd-msg { display: flex; gap: 10px; max-width: 100%; }
.cd-msg.user { flex-direction: row-reverse; }
.cd-msg-avatar {
  width: 30px; height: 30px;
  border-radius: 8px;
  display: flex; align-items: center; justify-content: center;
  font-size: 14px;
  flex-shrink: 0;
}
.cd-msg.user .cd-msg-avatar { background: linear-gradient(135deg, #6366f1, #8b5cf6); }
.cd-msg.clawd .cd-msg-avatar { background: linear-gradient(135deg, #00d4aa, #0099ff); }
.cd-msg.system .cd-msg-avatar { background: #1e1e2e; }

.cd-msg-wrap { display: flex; flex-direction: column; gap: 4px; max-width: 80%; }
.cd-msg.user .cd-msg-wrap { align-items: flex-end; }

.cd-msg-bubble {
  padding: 10px 14px;
  border-radius: 12px;
  font-size: 13px;
  line-height: 1.6;
  word-break: break-word;
}
.cd-msg.user .cd-msg-bubble {
  background: #1a1a3e;
  border: 1px solid #2e2e6e;
  color: #c7d2fe;
  border-bottom-right-radius: 4px;
}
.cd-msg.clawd .cd-msg-bubble {
  background: #0d1b1a;
  border: 1px solid #1a3330;
  color: #d1fae5;
  border-bottom-left-radius: 4px;
}
.cd-msg.system .cd-msg-bubble {
  background: #111118;
  border: 1px solid #1e1e2e;
  color: #64748b;
  font-size: 11px;
  font-family: 'Consolas', monospace;
  border-radius: 6px;
  white-space: pre-wrap;
}
.cd-msg-time {
  font-size: 10px;
  color: #334155;
  padding: 0 4px;
}
.cd-msg.clawd .cd-msg-provider {
  font-size: 10px;
  color: #00d4aa;
  padding: 0 4px;
  opacity: .7;
}

/* Code blocks inside messages */
.cd-code-block {
  position: relative;
  background: #0a0a10;
  border: 1px solid #1e1e2e;
  border-radius: 8px;
  margin: 8px 0;
  overflow: hidden;
}
.cd-code-lang {
  padding: 4px 12px;
  background: #111118;
  border-bottom: 1px solid #1e1e2e;
  font-size: 10px;
  color: #475569;
  font-family: monospace;
}
.cd-code-block pre {
  margin: 0;
  padding: 12px;
  overflow-x: auto;
  font-size: 12px;
  line-height: 1.5;
  color: #a5f3fc;
  font-family: 'Consolas', 'Monaco', monospace;
}
.cd-copy-btn {
  position: absolute;
  top: 4px; right: 8px;
  padding: 2px 8px;
  background: #1e1e2e;
  border: 1px solid #2e2e4e;
  border-radius: 4px;
  color: #94a3b8;
  font-size: 10px;
  cursor: pointer;
  transition: all .15s;
}
.cd-copy-btn:hover { color: #e2e8f0; border-color: #00d4aa; }
.cd-inline-code {
  background: #1e1e2e;
  padding: 1px 5px;
  border-radius: 4px;
  font-family: monospace;
  font-size: 12px;
  color: #a5f3fc;
}

/* Typing indicator */
.cd-typing {
  display: flex; gap: 4px; align-items: center;
  padding: 12px 14px;
}
.cd-typing span {
  width: 6px; height: 6px;
  background: #00d4aa;
  border-radius: 50%;
  animation: cd-bounce .9s infinite;
}
.cd-typing span:nth-child(2) { animation-delay: .15s; }
.cd-typing span:nth-child(3) { animation-delay: .3s; }
@keyframes cd-bounce {
  0%, 60%, 100% { transform: translateY(0); opacity: .5; }
  30% { transform: translateY(-6px); opacity: 1; }
}

/* ── Input area ── */
.cd-input-area {
  border-top: 1px solid #1e1e2e;
  background: #0d0d12;
  padding: 12px 16px;
  flex-shrink: 0;
}
.cd-input-bar {
  display: flex;
  align-items: flex-end;
  gap: 8px;
  background: #111118;
  border: 1px solid #1e1e2e;
  border-radius: 12px;
  padding: 8px 12px;
  transition: border-color .15s;
}
.cd-input-bar:focus-within { border-color: #00d4aa44; box-shadow: 0 0 0 2px #00d4aa11; }
.cd-textarea {
  flex: 1;
  background: none;
  border: none;
  outline: none;
  color: #e2e8f0;
  font-size: 13px;
  line-height: 1.5;
  resize: none;
  max-height: 140px;
  min-height: 22px;
  font-family: inherit;
}
.cd-textarea::placeholder { color: #334155; }
.cd-input-actions { display: flex; gap: 6px; align-items: center; }
.cd-ctx-toggle {
  padding: 5px 10px;
  background: none;
  border: 1px solid #1e1e2e;
  border-radius: 6px;
  color: #475569;
  font-size: 11px;
  cursor: pointer;
  transition: all .15s;
  display: flex; align-items: center; gap: 4px;
}
.cd-ctx-toggle:hover { border-color: #2e2e4e; color: #94a3b8; }
.cd-send-btn {
  width: 34px; height: 34px;
  background: linear-gradient(135deg, #00d4aa, #0099ff);
  border: none;
  border-radius: 8px;
  color: #000;
  cursor: pointer;
  display: flex; align-items: center; justify-content: center;
  font-size: 15px;
  transition: opacity .15s, transform .1s;
  flex-shrink: 0;
}
.cd-send-btn:hover { opacity: .85; transform: scale(1.05); }
.cd-send-btn:disabled { opacity: .3; cursor: not-allowed; transform: none; }
.cd-input-hints {
  display: flex;
  align-items: center;
  gap: 12px;
  margin-top: 6px;
  font-size: 11px;
  color: #334155;
}
.cd-model-tag {
  display: flex; align-items: center; gap: 4px;
  padding: 2px 8px;
  background: #111118;
  border: 1px solid #1e1e2e;
  border-radius: 10px;
  color: #475569;
  cursor: pointer;
  transition: all .15s;
}
.cd-model-tag:hover { border-color: #2e2e4e; color: #64748b; }
.cd-shortcut { color: #1e2a3a; }

/* ── Bridge panel (collapsible) ── */
.cd-bridge-panel {
  border-top: 1px solid #1e1e2e;
  background: #0a0a10;
  flex-shrink: 0;
}
.cd-bridge-toggle {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 8px 16px;
  cursor: pointer;
  font-size: 11px;
  color: #475569;
  font-weight: 600;
  letter-spacing: .06em;
  text-transform: uppercase;
  user-select: none;
  transition: color .15s;
}
.cd-bridge-toggle:hover { color: #64748b; }
.cd-bridge-toggle .cd-bridge-chevron { transition: transform .2s; }
.cd-bridge-toggle.open .cd-bridge-chevron { transform: rotate(180deg); }
.cd-bridge-body {
  max-height: 0;
  overflow: hidden;
  transition: max-height .3s ease;
}
.cd-bridge-body.open { max-height: 120px; }
.cd-bridge-inner {
  padding: 0 16px 12px;
  display: flex;
  gap: 8px;
  align-items: center;
}
.cd-bridge-input {
  flex: 1;
  padding: 7px 10px;
  background: #111118;
  border: 1px solid #1e1e2e;
  border-radius: 7px;
  color: #e2e8f0;
  font-size: 12px;
  font-family: 'Consolas', monospace;
  outline: none;
  transition: border-color .15s;
}
.cd-bridge-input:focus { border-color: #1a3330; }
.cd-bridge-input::placeholder { color: #334155; }
.cd-bridge-select {
  padding: 7px 10px;
  background: #111118;
  border: 1px solid #1e1e2e;
  border-radius: 7px;
  color: #e2e8f0;
  font-size: 12px;
  outline: none;
}
.cd-bridge-run {
  padding: 7px 14px;
  background: #1a3330;
  border: 1px solid #1a3330;
  border-radius: 7px;
  color: #00d4aa;
  font-size: 12px;
  font-weight: 600;
  cursor: pointer;
  transition: all .15s;
}
.cd-bridge-run:hover { background: #051c16; border-color: #00d4aa44; }
</style>`;

const PROMPTS = [
  '¿Cuántas ventas hubo hoy en panadería?',
  '¿Cuál es el estado del sistema?',
  'Haz un git status en el repo panadería',
  'Resume la actividad reciente de Atlas',
  '¿Qué productos tienen bajo inventario?',
];

export default {
  id: 'clawd-direct',
  label: 'ATLAS Directo',
  icon: 'message-square',
  category: 'intelligence',

  render(container) {
    if (!document.getElementById('cd-styles')) {
      document.head.insertAdjacentHTML('beforeend', CSS);
    }

    container.innerHTML = `
<div class="cd-root">

  <!-- Top bar -->
  <div class="cd-topbar">
    <button class="cd-back" onclick="location.hash='/'">
      <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
      Dashboard
    </button>
    <div class="cd-title">
      <div class="cd-title-icon">⚡</div>
      ATLAS Directo
    </div>
    <div class="cd-spacer"></div>
    <div class="cd-pills">
      <div class="cd-pill dim" id="cd-p-bridge"><span class="cd-pill-dot"></span>Bridge</div>
      <div class="cd-pill dim" id="cd-p-auth"><span class="cd-pill-dot"></span>Auth</div>
      <div class="cd-pill dim" id="cd-p-model"><span class="cd-pill-dot"></span>—</div>
    </div>
  </div>

  <!-- Body -->
  <div class="cd-body">

    <!-- Sidebar -->
    <div class="cd-sidebar">
      <div class="cd-sidebar-section">Conversación</div>
      <div class="cd-sidebar-item active" id="cd-tab-chat">
        <span class="cd-si-icon">💬</span> Chat IA
      </div>
      <div class="cd-sidebar-item" id="cd-tab-bridge">
        <span class="cd-si-icon">⚙</span> Bridge
      </div>

      <div class="cd-sidebar-section" style="margin-top:8px">Contexto</div>
      <div class="cd-ctx-box" id="cd-ctx-box">
        <div class="cd-ctx-row"><span>Modelo</span><span class="cd-ctx-val" id="cd-ctx-model">—</span></div>
        <div class="cd-ctx-row"><span>Canal</span><span class="cd-ctx-val">workspace</span></div>
        <div class="cd-ctx-row"><span>Repos</span><span class="cd-ctx-val" id="cd-ctx-repos">—</span></div>
        <div class="cd-ctx-row"><span>Mensajes</span><span class="cd-ctx-val" id="cd-ctx-count">0</span></div>
      </div>

      <div class="cd-sidebar-section" style="margin-top:4px">Acciones rápidas</div>
      <div class="cd-sidebar-item" data-prompt="git status en panadería">
        <span class="cd-si-icon">📋</span> Git status
      </div>
      <div class="cd-sidebar-item" data-prompt="Estado del inventario">
        <span class="cd-si-icon">📦</span> Inventario
      </div>
      <div class="cd-sidebar-item" data-prompt="Resumen de ventas de hoy">
        <span class="cd-si-icon">📈</span> Ventas hoy
      </div>
      <div class="cd-sidebar-item" data-prompt="Estado del sistema Atlas">
        <span class="cd-si-icon">🔍</span> Diagnóstico
      </div>
      <div class="cd-sidebar-item" id="cd-clear-btn" style="margin-top:auto; color:#ef4444">
        <span class="cd-si-icon">🗑</span> Limpiar chat
      </div>
    </div>

    <!-- Main chat -->
    <div class="cd-main">
      <div class="cd-messages" id="cd-messages">
        <div class="cd-welcome" id="cd-welcome">
          <div class="cd-welcome-icon">⚡</div>
          <h3>ATLAS Directo</h3>
          <p>Interfaz directa con ATLAS. Puedes consultar, analizar, resumir actividad y ejecutar acciones controladas mediante el bridge operativo.</p>
          <div class="cd-welcome-chips" id="cd-welcome-chips"></div>
        </div>
      </div>

      <!-- Input area -->
      <div class="cd-input-area">
        <div class="cd-input-bar">
          <textarea class="cd-textarea" id="cd-textarea" rows="1"
            placeholder="Pregunta a Atlas IA o da una instrucción..."></textarea>
          <div class="cd-input-actions">
            <button class="cd-ctx-toggle" id="cd-bridge-quick" title="Ejecutar comando en repo">⚙ Bridge</button>
            <button class="cd-send-btn" id="cd-send" title="Enviar (Ctrl+Enter)">
              <svg width="15" height="15" viewBox="0 0 24 24" fill="currentColor"><path d="M2.01 21L23 12 2.01 3 2 10l15 2-15 2z"/></svg>
            </button>
          </div>
        </div>
        <div class="cd-input-hints">
          <div class="cd-model-tag" id="cd-model-tag">
            <span>⚡</span><span id="cd-model-name">cargando...</span>
          </div>
          <span class="cd-shortcut">Ctrl+Enter para enviar · Shift+Enter nueva línea</span>
        </div>
      </div>

      <!-- Bridge panel -->
      <div class="cd-bridge-panel" id="cd-bridge-panel" style="display:none">
        <div class="cd-bridge-toggle" id="cd-bridge-toggle">
          <span>⚙ BRIDGE — Ejecutar en repositorio</span>
          <span class="cd-bridge-chevron">▲</span>
        </div>
        <div class="cd-bridge-body open" id="cd-bridge-body">
          <div class="cd-bridge-inner">
            <input class="cd-bridge-input" id="cd-bridge-cmd" placeholder="git log --oneline -5  /  git diff HEAD~1  /  ls backend/" />
            <select class="cd-bridge-select" id="cd-bridge-repo">
              <option value="panaderia">panaderia</option>
              <option value="vision">vision</option>
            </select>
            <button class="cd-bridge-run" id="cd-bridge-run">▶ Ejecutar</button>
          </div>
        </div>
      </div>
    </div>
  </div>
</div>`;

    // ── State ────────────────────────────────────────────────────────────────
    let msgCount = 0;
    let sending = false;
    let lastProvider = '—';
    const msgs = container.querySelector('#cd-messages');
    const textarea = container.querySelector('#cd-textarea');
    const sendBtn = container.querySelector('#cd-send');
    const ctxCount = container.querySelector('#cd-ctx-count');
    const ctxModel = container.querySelector('#cd-ctx-model');
    const modelName = container.querySelector('#cd-model-name');

    // ── Welcome chips ────────────────────────────────────────────────────────
    const chipsEl = container.querySelector('#cd-welcome-chips');
    PROMPTS.forEach(p => {
      const c = document.createElement('div');
      c.className = 'cd-welcome-chip';
      c.textContent = p;
      c.addEventListener('click', () => { textarea.value = p; sendMsg(); });
      chipsEl.appendChild(c);
    });

    // ── Sidebar quick prompts ────────────────────────────────────────────────
    container.querySelectorAll('[data-prompt]').forEach(el => {
      el.addEventListener('click', () => {
        textarea.value = el.dataset.prompt;
        sendMsg();
      });
    });

    // ── Clear chat ───────────────────────────────────────────────────────────
    container.querySelector('#cd-clear-btn').addEventListener('click', () => {
      msgs.innerHTML = container.querySelector('#cd-welcome') ? '' : '';
      msgs.innerHTML = `<div class="cd-welcome" id="cd-welcome">
        <div class="cd-welcome-icon">⚡</div>
        <h3>Chat limpiado</h3>
        <p>Nueva conversación iniciada.</p>
      </div>`;
      msgCount = 0;
      ctxCount.textContent = '0';
    });

    // ── Auto-resize textarea ─────────────────────────────────────────────────
    textarea.addEventListener('input', () => {
      textarea.style.height = '22px';
      textarea.style.height = Math.min(textarea.scrollHeight, 140) + 'px';
    });

    // ── Keyboard shortcuts ───────────────────────────────────────────────────
    textarea.addEventListener('keydown', e => {
      if (e.key === 'Enter' && e.ctrlKey) { e.preventDefault(); sendMsg(); }
    });
    sendBtn.addEventListener('click', sendMsg);

    // ── Bridge panel toggle ──────────────────────────────────────────────────
    const bridgePanel = container.querySelector('#cd-bridge-panel');
    const bridgeToggle = container.querySelector('#cd-bridge-toggle');
    const bridgeBody = container.querySelector('#cd-bridge-body');
    let bridgeOpen = true;

    container.querySelector('#cd-bridge-quick').addEventListener('click', () => {
      bridgePanel.style.display = bridgePanel.style.display === 'none' ? '' : 'none';
    });
    bridgeToggle.addEventListener('click', () => {
      bridgeOpen = !bridgeOpen;
      bridgeBody.classList.toggle('open', bridgeOpen);
      bridgeToggle.classList.toggle('open', bridgeOpen);
    });

    // ── Sidebar tabs ─────────────────────────────────────────────────────────
    container.querySelector('#cd-tab-chat').addEventListener('click', () => {
      container.querySelector('#cd-tab-chat').classList.add('active');
      container.querySelector('#cd-tab-bridge').classList.remove('active');
      bridgePanel.style.display = 'none';
    });
    container.querySelector('#cd-tab-bridge').addEventListener('click', () => {
      container.querySelector('#cd-tab-bridge').classList.add('active');
      container.querySelector('#cd-tab-chat').classList.remove('active');
      bridgePanel.style.display = '';
    });

    // ── Bridge run ───────────────────────────────────────────────────────────
    container.querySelector('#cd-bridge-run').addEventListener('click', async () => {
      const command = (container.querySelector('#cd-bridge-cmd').value || '').trim();
      const repo = container.querySelector('#cd-bridge-repo').value;
      if (!command) return;
      addMsg('system', `$ [${repo}] ${command}`);
      try {
        const r = await fetch('/api/clawd/bridge/action', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ action_id: `ui-${Date.now()}`, target_repo: repo, command, run_snapshot: false, timeout_sec: 60 }),
        });
        const d = await r.json();
        const out = d.output || d.result || JSON.stringify(d, null, 2);
        addMsg('system', String(out).substring(0, 2000));
      } catch (e) {
        addMsg('system', `Error: ${e.message}`);
      }
    });

    // ── Helpers ──────────────────────────────────────────────────────────────
    function removeWelcome() {
      const w = container.querySelector('#cd-welcome');
      if (w) w.remove();
    }

    function addMsg(role, text, provider = '') {
      removeWelcome();
      msgCount++;
      ctxCount.textContent = msgCount;

      const wrap = document.createElement('div');
      wrap.className = `cd-msg ${role}`;

      const avatar = document.createElement('div');
      avatar.className = 'cd-msg-avatar';
      avatar.textContent = role === 'user' ? '👤' : role === 'clawd' ? '⚡' : '⚙';

      const msgWrap = document.createElement('div');
      msgWrap.className = 'cd-msg-wrap';

      const bubble = document.createElement('div');
      bubble.className = 'cd-msg-bubble';
      if (role === 'system') {
        bubble.textContent = text;
      } else {
        bubble.innerHTML = _md(text);
      }

      const meta = document.createElement('div');
      meta.style.cssText = 'display:flex;gap:8px;align-items:center';

      const time = document.createElement('div');
      time.className = 'cd-msg-time';
      time.textContent = new Date().toLocaleTimeString('es', { hour: '2-digit', minute: '2-digit' });
      meta.appendChild(time);

      if (provider && role === 'clawd') {
        const prov = document.createElement('div');
        prov.className = 'cd-msg-provider';
        prov.textContent = formatProvider(provider);
        meta.appendChild(prov);
        lastProvider = provider;
        updateModelPill(provider);
      }

      msgWrap.appendChild(bubble);
      msgWrap.appendChild(meta);
      wrap.appendChild(avatar);
      wrap.appendChild(msgWrap);
      msgs.appendChild(wrap);
      msgs.scrollTop = msgs.scrollHeight;
      return wrap;
    }

    function addTyping() {
      removeWelcome();
      const wrap = document.createElement('div');
      wrap.className = 'cd-msg clawd';
      wrap.id = 'cd-typing-indicator';
      wrap.innerHTML = `
        <div class="cd-msg-avatar">⚡</div>
        <div class="cd-msg-bubble cd-typing">
          <span></span><span></span><span></span>
        </div>`;
      msgs.appendChild(wrap);
      msgs.scrollTop = msgs.scrollHeight;
      return wrap;
    }

    function removeTyping() {
      document.getElementById('cd-typing-indicator')?.remove();
    }

    function updateModelPill(provider) {
      const p = container.querySelector('#cd-p-model');
      const name = formatProvider(provider);
      p.className = 'cd-pill ok';
      p.innerHTML = `<span class="cd-pill-dot"></span>${name}`;
      modelName.textContent = provider.includes('anthropic') ? 'claude' : provider.includes('local') ? 'ollama' : 'atlas-ai';
      ctxModel.textContent = modelName.textContent;
    }

    function formatProvider(provider) {
      return provider
        .split(':')
        .slice(0, 2)
        .join(':')
        .replace('local_auto', '🏠 local')
        .replace('clawd_api', '⚡ atlas')
        .replace('clawd_cli', '⚡ atlas-cli');
    }

    // ── Send message ─────────────────────────────────────────────────────────
    async function sendMsg() {
      const message = (textarea.value || '').trim();
      if (!message || sending) return;

      sending = true;
      sendBtn.disabled = true;
      textarea.value = '';
      textarea.style.height = '22px';

      addMsg('user', message);
      const typing = addTyping();

      try {
        const r = await fetch('/brain/process', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ text: message }),
        });
        const d = await r.json();
        removeTyping();
        if (!d.ok) throw new Error(d.error || 'Sin respuesta del cerebro');
        const reply = _pickReply(d) || '(sin respuesta)';
        addMsg('clawd', reply, d.model_used || d.provider || '');
      } catch (e) {
        removeTyping();
        addMsg('system', `Error de conexión: ${e.message}`);
      } finally {
        sending = false;
        sendBtn.disabled = false;
        textarea.focus();
      }
    }

    // ── Bridge status ────────────────────────────────────────────────────────
    async function loadStatus() {
      try {
        const r = await fetch('/api/clawd/bridge/status');
        const d = await r.json();

        const pBridge = container.querySelector('#cd-p-bridge');
        const pAuth = container.querySelector('#cd-p-auth');
        const ctxRepos = container.querySelector('#cd-ctx-repos');

        const stable = d?.stable;
        pBridge.className = `cd-pill ${stable ? 'ok' : 'warn'}`;
        pBridge.innerHTML = `<span class="cd-pill-dot"></span>${stable ? 'Bridge OK' : 'Bridge ⚠'}`;

        const auth = d?.repo_access?.authorized;
        pAuth.className = `cd-pill ${auth ? 'ok' : 'err'}`;
        pAuth.innerHTML = `<span class="cd-pill-dot"></span>${auth ? 'Autorizado' : 'Sin auth'}`;

        const repos = Object.keys(d?.repo_access?.repos || {}).join(', ');
        if (ctxRepos) ctxRepos.textContent = repos || '—';
      } catch (e) {
        const pBridge = container.querySelector('#cd-p-bridge');
        pBridge.className = 'cd-pill err';
        pBridge.innerHTML = '<span class="cd-pill-dot"></span>Sin conexión';
      }
    }

    loadStatus();
    textarea.focus();
    return () => {};
  },
};

window.AtlasModuleClawdDirect = { id: 'clawd-direct' };
