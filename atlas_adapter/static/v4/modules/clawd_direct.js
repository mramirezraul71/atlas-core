function _esc(s) {
  const d = document.createElement('span');
  d.textContent = s ?? '';
  return d.innerHTML;
}

function _pickReply(data) {
  if (!data || typeof data !== 'object') return '';
  return String(data.reply || data.response || data.text || data.message || '').trim();
}

export default {
  id: 'clawd-direct',
  label: 'Clawd Directo',
  icon: 'message-square',
  category: 'intelligence',

  render(container) {
    container.innerHTML = `
      <div class="module-view" style="height:100%;display:flex;flex-direction:column">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Dashboard
          </button>
          <h2>Clawd Directo</h2>
          <div style="margin-left:auto;display:flex;gap:8px;align-items:center">
            <a class="action-btn" href="/workspace" target="_blank" rel="noopener">Workspace</a>
          </div>
        </div>
        <div class="module-body" style="display:flex;flex-direction:column;gap:12px;height:100%">
          <div class="stat-row">
            <div class="stat-card"><div class="stat-card-label">Bridge estable</div><div class="stat-card-value" id="cd-stable">--</div></div>
            <div class="stat-card"><div class="stat-card-label">Repo access</div><div class="stat-card-value" id="cd-access">--</div></div>
            <div class="stat-card"><div class="stat-card-label">Provider</div><div class="stat-card-value" id="cd-provider">--</div></div>
          </div>
          <div style="display:flex;gap:8px;flex-wrap:wrap">
            <input id="cd-token" placeholder="X-Atlas-Core (opcional)" style="min-width:280px;flex:1;padding:8px;border:1px solid var(--border-subtle);border-radius:8px;background:var(--surface-1);color:var(--text-primary)">
            <button class="action-btn" id="cd-refresh">Refrescar estado</button>
          </div>
          <div id="cd-log" style="flex:1;overflow:auto;background:var(--surface-0);border:1px solid var(--border-subtle);border-radius:10px;padding:12px;font-size:12px"></div>
          <div style="display:flex;gap:8px">
            <textarea id="cd-msg" rows="3" placeholder="Escribe instrucción directa a ClawdBOT..." style="flex:1;padding:10px;border:1px solid var(--border-subtle);border-radius:8px;background:var(--surface-1);color:var(--text-primary)"></textarea>
            <button class="action-btn primary" id="cd-send">Enviar</button>
          </div>
          <div style="display:flex;gap:8px;align-items:center">
            <input id="cd-cmd" placeholder="Acción bridge (ej: git status)" style="flex:1;padding:8px;border:1px solid var(--border-subtle);border-radius:8px;background:var(--surface-1);color:var(--text-primary)">
            <select id="cd-repo" style="padding:8px;border:1px solid var(--border-subtle);border-radius:8px;background:var(--surface-1);color:var(--text-primary)">
              <option value="panaderia">panaderia</option>
              <option value="vision">vision</option>
            </select>
            <button class="action-btn" id="cd-run">Ejecutar acción</button>
          </div>
        </div>
      </div>
    `;

    const elStable = container.querySelector('#cd-stable');
    const elAccess = container.querySelector('#cd-access');
    const elProvider = container.querySelector('#cd-provider');
    const elToken = container.querySelector('#cd-token');
    const elLog = container.querySelector('#cd-log');
    const elMsg = container.querySelector('#cd-msg');
    const elCmd = container.querySelector('#cd-cmd');
    const elRepo = container.querySelector('#cd-repo');
    const tokenKey = 'atlas-clawd-token';
    elToken.value = localStorage.getItem(tokenKey) || '';

    function logLine(obj) {
      const txt = typeof obj === 'string' ? obj : JSON.stringify(obj, null, 2);
      elLog.textContent += `${new Date().toLocaleTimeString()} ${txt}\n\n`;
      elLog.scrollTop = elLog.scrollHeight;
    }

    function header() {
      const v = (elToken.value || '').trim();
      if (v) localStorage.setItem(tokenKey, v);
      return v ? { 'Content-Type': 'application/json', 'X-Atlas-Core': v } : { 'Content-Type': 'application/json' };
    }

    async function refreshStatus() {
      try {
        const r = await fetch('/api/clawd/bridge/status?run_snapshot=true', { headers: header() });
        const d = await r.json();
        const stable = Boolean(d?.stable);
        const auth = Boolean(d?.repo_access?.authorized);
        elStable.textContent = stable ? 'SI' : 'NO';
        elAccess.textContent = auth ? 'AUTORIZADO' : 'SIN TOKEN';
        logLine({ status: d });
      } catch (e) {
        logLine({ error: String(e) });
      }
    }

    container.querySelector('#cd-refresh')?.addEventListener('click', refreshStatus);

    container.querySelector('#cd-send')?.addEventListener('click', async () => {
      const message = (elMsg.value || '').trim();
      if (!message) return;
      logLine({ you: message });
      try {
        const r = await fetch('/api/comms/atlas/message', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ user_id: 'owner', channel: 'workspace', message, context: { source: 'v4-clawd-direct' } }),
        });
        const d = await r.json();
        const reply = _pickReply(d) || '(sin respuesta)';
        elProvider.textContent = _esc(d.provider || 'unknown');
        logLine({ clawd: reply, raw: d });
      } catch (e) {
        logLine({ error: String(e) });
      }
    });

    container.querySelector('#cd-run')?.addEventListener('click', async () => {
      const command = (elCmd.value || '').trim();
      if (!command) return;
      const payload = {
        action_id: `ui-${Date.now()}`,
        target_repo: elRepo.value,
        command,
        run_snapshot: true,
        timeout_sec: 180,
      };
      logLine({ action: payload });
      try {
        const r = await fetch('/api/clawd/bridge/action', {
          method: 'POST',
          headers: header(),
          body: JSON.stringify(payload),
        });
        const d = await r.json();
        logLine({ result: d });
      } catch (e) {
        logLine({ error: String(e) });
      }
    });

    refreshStatus();
    return () => {};
  },
};

window.AtlasModuleClawdDirect = { id: 'clawd-direct' };
