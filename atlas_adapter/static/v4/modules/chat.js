/**
 * ATLAS v4.2 — Chat Module
 * Chat directo con el cerebro ATLAS en tiempo real.
 * Endpoint: POST /brain/process
 */

const QUICK_PROMPTS = [
  { label: '¿Estado general?',    text: '¿Cuál es tu estado general actual?' },
  { label: 'Objetivos activos',    text: '¿Cuáles son tus objetivos activos ahora mismo?' },
  { label: 'Última decisión',      text: '¿Cuál fue la última decisión autónoma que tomaste?' },
  { label: 'Salud del sistema',    text: 'Dame un resumen de la salud del sistema.' },
  { label: 'Memoria reciente',     text: '¿Qué has aprendido o procesado recientemente?' },
  { label: 'Tareas pendientes',    text: '¿Qué tareas tienes pendientes en este momento?' },
];

function _esc(s) { const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML; }
function _ts() { return new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' }); }

export default {
  id: 'chat',
  label: 'Chat Cerebro',
  icon: 'message-square',
  category: 'intelligence',

  render(container) {
    container.innerHTML = `
      <div class="module-view" style="height:100%;display:flex;flex-direction:column">
        <div class="module-header">
          <button class="back-btn" id="chat-back">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Dashboard
          </button>
          <h2>Chat con Cerebro</h2>
          <div style="margin-left:auto;display:flex;gap:8px;align-items:center">
            <span id="chat-status-badge" class="live-badge" style="background:var(--accent-green)">● CONECTADO</span>
            <span class="live-badge">BRAIN</span>
          </div>
        </div>

        <div class="module-body" style="flex:1;display:flex;flex-direction:column;overflow:hidden;padding-bottom:0">

          <!-- KPIs -->
          <div class="stat-row" style="margin-bottom:16px;flex-shrink:0">
            <div class="stat-card hero">
              <div class="stat-card-label">Mensajes enviados</div>
              <div class="stat-card-value accent" id="chat-msg-count">0</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Latencia promedio</div>
              <div class="stat-card-value" id="chat-avg-ms" style="font-size:14px">--</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Sesión activa</div>
              <div class="stat-card-value" id="chat-session-time" style="font-size:14px">--</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Modelo activo</div>
              <div class="stat-card-value" id="chat-model" style="font-size:11px">--</div>
            </div>
          </div>

          <!-- Quick prompts -->
          <div style="flex-shrink:0;margin-bottom:12px">
            <div class="section-title" style="margin-bottom:8px">Accesos Rápidos</div>
            <div style="display:flex;flex-wrap:wrap;gap:6px" id="chat-quick-btns">
              ${QUICK_PROMPTS.map((q, i) => `
                <button class="action-btn" data-qi="${i}" style="font-size:11px;padding:5px 10px">
                  ${_esc(q.label)}
                </button>
              `).join('')}
              <button class="action-btn danger" id="chat-clear-btn" style="font-size:11px;padding:5px 10px;margin-left:auto">
                <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="3 6 5 6 21 6"/><path d="M19 6v14a2 2 0 01-2 2H7a2 2 0 01-2-2V6M8 6V4a2 2 0 012-2h4a2 2 0 012 2v2"/></polyline></svg>
                Limpiar
              </button>
            </div>
          </div>

          <!-- Ventana de chat -->
          <div id="chat-window"
               style="flex:1;overflow-y:auto;background:var(--surface-0);border:1px solid var(--border-subtle);border-radius:12px;padding:16px;margin-bottom:12px;min-height:200px;scroll-behavior:smooth">
            <div id="chat-messages">
              <!-- Mensaje de bienvenida -->
              <div class="chat-msg-system">
                <div class="chat-bubble system">
                  <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polygon points="13 2 3 14 12 14 11 22 21 10 12 10 13 2"/></svg>
                  Canal de comunicación directa con el Cerebro ATLAS iniciado. Escribe un mensaje o usa los accesos rápidos para interactuar.
                </div>
              </div>
            </div>
          </div>

          <!-- Input de mensaje -->
          <div style="flex-shrink:0;padding-bottom:16px">
            <div style="display:flex;gap:8px;align-items:flex-end">
              <textarea id="chat-input"
                placeholder="Escribe o habla con el cerebro ATLAS..."
                style="flex:1;resize:vertical;min-height:52px;max-height:120px;padding:12px 14px;background:var(--surface-1);border:1px solid var(--border-subtle);border-radius:10px;color:var(--text-primary);font-size:13px;font-family:inherit;line-height:1.4;outline:none;transition:border-color .2s"
                rows="2"></textarea>
              <button id="chat-mic-btn" title="Hablar (micrófono)"
                style="height:52px;width:48px;background:var(--surface-2,var(--surface-1));color:var(--text-muted);border:1px solid var(--border-subtle);border-radius:10px;cursor:pointer;display:flex;align-items:center;justify-content:center;flex-shrink:0;transition:color .2s,background .2s">
                <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M12 1a3 3 0 00-3 3v8a3 3 0 006 0V4a3 3 0 00-3-3z"/><path d="M19 10v2a7 7 0 01-14 0v-2M12 19v4M8 23h8"/></svg>
              </button>
              <button id="chat-send-btn"
                style="height:52px;padding:0 20px;background:var(--accent-primary);color:#000;border:none;border-radius:10px;font-size:13px;font-weight:600;cursor:pointer;display:flex;align-items:center;gap:6px;white-space:nowrap;transition:opacity .2s">
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><line x1="22" y1="2" x2="11" y2="13"/><polygon points="22 2 15 22 11 13 2 9 22 2"/></svg>
                Enviar
              </button>
            </div>
            <div id="chat-hint" style="font-size:11px;color:var(--text-muted);margin-top:6px;text-align:right">
              Enter para nueva línea · Ctrl+Enter para enviar · 🎤 clic en micrófono para dictar
            </div>
          </div>

        </div>
      </div>

      <style>
        .chat-msg { margin-bottom: 12px; }
        .chat-msg-user { display: flex; justify-content: flex-end; }
        .chat-msg-atlas { display: flex; justify-content: flex-start; }
        .chat-msg-system { display: flex; justify-content: center; margin-bottom: 8px; }
        .chat-bubble {
          max-width: 75%;
          padding: 10px 14px;
          border-radius: 12px;
          font-size: 13px;
          line-height: 1.5;
          word-break: break-word;
          white-space: pre-wrap;
        }
        .chat-bubble.user {
          background: var(--accent-primary);
          color: #000;
          border-bottom-right-radius: 4px;
        }
        .chat-bubble.atlas {
          background: var(--surface-2, var(--surface-1));
          color: var(--text-primary);
          border-bottom-left-radius: 4px;
          border: 1px solid var(--border-subtle);
        }
        .chat-bubble.error-bubble {
          background: rgba(255,60,60,.12);
          color: var(--accent-red);
          border: 1px solid rgba(255,60,60,.25);
          border-bottom-left-radius: 4px;
        }
        .chat-bubble.system {
          background: rgba(255,255,255,.05);
          color: var(--text-muted);
          font-size: 11px;
          display: flex;
          align-items: center;
          gap: 6px;
          padding: 6px 12px;
          border-radius: 20px;
          border: 1px solid var(--border-subtle);
          max-width: 90%;
        }
        .chat-bubble.thinking {
          background: var(--surface-2, var(--surface-1));
          color: var(--text-muted);
          font-style: italic;
          border: 1px solid var(--border-subtle);
          border-bottom-left-radius: 4px;
          animation: chatPulse 1.2s ease-in-out infinite;
        }
        @keyframes chatPulse {
          0%, 100% { opacity: .7 } 50% { opacity: 1 }
        }
        .chat-meta {
          font-size: 10px;
          color: var(--text-muted);
          margin-top: 3px;
          padding: 0 2px;
        }
        .chat-meta.right { text-align: right; }
        #chat-input:focus { border-color: var(--accent-primary); }
        #chat-send-btn:hover { opacity: .85; }
        #chat-send-btn:disabled { opacity: .4; cursor: not-allowed; }
      </style>
    `;

    let msgCount = 0;
    let totalMs = 0;
    let sessionStart = Date.now();
    const messagesEl = container.querySelector('#chat-messages');
    const windowEl   = container.querySelector('#chat-window');
    const inputEl    = container.querySelector('#chat-input');
    const sendBtn    = container.querySelector('#chat-send-btn');
    const countEl    = container.querySelector('#chat-msg-count');
    const avgMsEl    = container.querySelector('#chat-avg-ms');
    const modelEl    = container.querySelector('#chat-model');
    const sessionEl  = container.querySelector('#chat-session-time');

    container.querySelector('#chat-back')?.addEventListener('click', () => { location.hash = '/'; });

    // ── Micrófono ────────────────────────────────────────────────────────────
    const micBtn = container.querySelector('#chat-mic-btn');
    const SpeechRec = window.SpeechRecognition || window.webkitSpeechRecognition;
    if (!SpeechRec) {
      if (micBtn) micBtn.style.display = 'none';
    } else {
      const rec = new SpeechRec();
      rec.lang = 'es-ES';
      rec.interimResults = true;
      rec.maxAlternatives = 1;
      let _listening = false;

      rec.onstart = () => {
        _listening = true;
        if (micBtn) {
          micBtn.style.color = 'var(--accent-red, #ff4444)';
          micBtn.style.borderColor = 'var(--accent-red, #ff4444)';
          micBtn.title = 'Escuchando… clic para detener';
        }
      };
      rec.onend = () => {
        _listening = false;
        if (micBtn) {
          micBtn.style.color = '';
          micBtn.style.borderColor = '';
          micBtn.title = 'Hablar (micrófono)';
        }
      };
      rec.onresult = (e) => {
        const transcript = Array.from(e.results).map(r => r[0].transcript).join('');
        if (inputEl) inputEl.value = transcript;
        if (e.results[e.results.length - 1].isFinal && transcript.trim()) {
          _sendMessage();
        }
      };
      rec.onerror = (e) => {
        _listening = false;
        if (micBtn) { micBtn.style.color = ''; micBtn.style.borderColor = ''; }
        const msg = e.error === 'not-allowed'
          ? 'Permiso de micrófono denegado'
          : 'Error de micrófono: ' + e.error;
        window.AtlasToast?.show(msg, 'error');
      };
      micBtn?.addEventListener('click', () => {
        if (_listening) { rec.stop(); return; }
        rec.start();
      });
    }
    // ─────────────────────────────────────────────────────────────────────────

    // Session timer
    const sessionTimer = setInterval(() => {
      const secs = Math.floor((Date.now() - sessionStart) / 1000);
      const m = Math.floor(secs / 60), s = secs % 60;
      if (sessionEl) sessionEl.textContent = `${String(m).padStart(2,'0')}:${String(s).padStart(2,'0')}`;
    }, 1000);
    container._sessionTimer = sessionTimer;

    // Load active model from health
    fetch('/health').then(r => r.json()).then(d => {
      const ver = d.checks?.version || d.version || 'ATLAS Brain';
      if (modelEl) modelEl.textContent = ver;
    }).catch(() => {});

    // Quick prompts
    container.querySelector('#chat-quick-btns')?.addEventListener('click', (e) => {
      const btn = e.target.closest('[data-qi]');
      if (!btn) return;
      const qi = parseInt(btn.dataset.qi, 10);
      if (!isNaN(qi) && QUICK_PROMPTS[qi]) {
        if (inputEl) inputEl.value = QUICK_PROMPTS[qi].text;
        _sendMessage();
      }
    });

    // Clear chat
    container.querySelector('#chat-clear-btn')?.addEventListener('click', () => {
      if (messagesEl) messagesEl.innerHTML = `
        <div class="chat-msg-system">
          <div class="chat-bubble system">
            <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="3 6 5 6 21 6"/><path d="M19 6v14a2 2 0 01-2 2H7a2 2 0 01-2-2V6"/></svg>
            Conversación limpiada.
          </div>
        </div>`;
    });

    // Send on Ctrl+Enter
    inputEl?.addEventListener('keydown', (e) => {
      if (e.key === 'Enter' && e.ctrlKey) {
        e.preventDefault();
        _sendMessage();
      }
    });

    sendBtn?.addEventListener('click', _sendMessage);

    async function _sendMessage() {
      const text = (inputEl?.value || '').trim();
      if (!text) return;
      if (inputEl) inputEl.value = '';

      // Append user bubble
      _appendBubble('user', text);

      // Thinking indicator
      const thinkId = 'think-' + Date.now();
      _appendThinking(thinkId);

      msgCount++;
      if (countEl) countEl.textContent = msgCount;
      if (sendBtn) sendBtn.disabled = true;

      const t0 = performance.now();
      try {
        const r = await fetch('/brain/process', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ text: text, context: 'chat_v4' }),
        });
        const d = await r.json().catch(() => ({}));
        const ms = Math.round(performance.now() - t0);

        // Remove thinking
        document.getElementById(thinkId)?.remove();

        if (!r.ok || d.ok === false) {
          throw new Error(d.error || d.message || `HTTP ${r.status}`);
        }

        const reply = d?.data?.response || d?.data?.output || d?.response || d?.output
          || d?.data?.result?.response || d?.data?.text || d?.data
          || 'Sin respuesta del cerebro.';

        const replyStr = typeof reply === 'string' ? reply : JSON.stringify(reply, null, 2);
        _appendBubble('atlas', replyStr, ms);

        totalMs += ms;
        if (avgMsEl) avgMsEl.textContent = `${Math.round(totalMs / msgCount)} ms`;

        // Update model from response
        const model = d?.data?.model || d?.model;
        if (model && modelEl) modelEl.textContent = model;

      } catch (e) {
        document.getElementById(thinkId)?.remove();
        _appendError(e.message);
        window.AtlasToast?.show(e.message, 'error');
      } finally {
        if (sendBtn) sendBtn.disabled = false;
        if (inputEl) inputEl.focus();
      }
    }

    function _appendBubble(role, text, ms) {
      const div = document.createElement('div');
      div.className = `chat-msg chat-msg-${role === 'user' ? 'user' : 'atlas'}`;
      div.innerHTML = `
        <div>
          <div class="chat-bubble ${role === 'user' ? 'user' : 'atlas'}">${_esc(text)}</div>
          <div class="chat-meta ${role === 'user' ? 'right' : ''}">
            ${role === 'user' ? 'Tú' : 'ATLAS Brain'} · ${_ts()}${ms !== undefined ? ` · ${ms}ms` : ''}
          </div>
        </div>`;
      messagesEl.appendChild(div);
      _scrollToBottom();
    }

    function _appendThinking(id) {
      const div = document.createElement('div');
      div.className = 'chat-msg chat-msg-atlas';
      div.id = id;
      div.innerHTML = `
        <div>
          <div class="chat-bubble thinking">
            ● Procesando...
          </div>
        </div>`;
      messagesEl.appendChild(div);
      _scrollToBottom();
    }

    function _appendError(msg) {
      const div = document.createElement('div');
      div.className = 'chat-msg chat-msg-atlas';
      div.innerHTML = `
        <div>
          <div class="chat-bubble error-bubble">⚠ ${_esc(msg)}</div>
          <div class="chat-meta">Error · ${_ts()}</div>
        </div>`;
      messagesEl.appendChild(div);
      _scrollToBottom();
    }

    function _scrollToBottom() {
      if (windowEl) windowEl.scrollTop = windowEl.scrollHeight;
    }

    // Auto-enviar si viene de consulta de voz desde el landing
    const pendingVoice = sessionStorage.getItem('atlas-voice-query');
    if (pendingVoice) {
      sessionStorage.removeItem('atlas-voice-query');
      if (inputEl) inputEl.value = pendingVoice;
      setTimeout(() => _sendMessage(), 350);
    } else {
      if (inputEl) inputEl.focus();
    }
  },

  destroy() {
    // clearInterval handled via container._sessionTimer — cleared on container teardown
  },
};

window.AtlasModuleChat = { id: 'chat' };
