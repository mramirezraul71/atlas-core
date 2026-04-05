function _esc(s) {
  const d = document.createElement('span');
  d.textContent = s ?? '';
  return d.innerHTML;
}

export default {
  id: 'codex-supervisor',
  label: 'Codex Supervisor',
  icon: 'cpu',
  category: 'control',

  render(container) {
    container.innerHTML = `
      <div class="module-view" style="height:100%;display:flex;flex-direction:column">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Dashboard
          </button>
          <h2>Codex Supervisor</h2>
          <div style="margin-left:auto;display:flex;gap:8px;align-items:center">
            <a class="action-btn" href="/nexus" target="_blank" rel="noopener">Abrir Panel NEXUS</a>
          </div>
        </div>
        <div class="module-body" style="display:flex;flex-direction:column;gap:12px;height:100%">
          <div class="stat-row">
            <div class="stat-card"><div class="stat-card-label">Daemon</div><div class="stat-card-value" id="cs-daemon">--</div></div>
            <div class="stat-card"><div class="stat-card-label">Directivas</div><div class="stat-card-value" id="cs-count">--</div></div>
            <div class="stat-card"><div class="stat-card-label">Thread</div><div class="stat-card-value" id="cs-thread">--</div></div>
          </div>
          <div style="display:flex;gap:8px;flex-wrap:wrap">
            <button class="action-btn" id="cs-refresh">Refrescar</button>
            <button class="action-btn" id="cs-open-workspace" title="Abre el panel canónico de NEXUS">Ir a NEXUS</button>
            <button class="action-btn" id="cs-link-claude">Vincular suscripción Claude</button>
            <a class="action-btn" href="https://console.anthropic.com/" target="_blank" rel="noopener">Anthropic Console</a>
            <a class="action-btn" href="#/config">AI Config</a>
          </div>
          <div id="cs-claude-box" style="display:none;background:var(--surface-0);border:1px solid var(--border-subtle);border-radius:10px;padding:10px">
            <div style="display:flex;gap:8px;align-items:center;flex-wrap:wrap">
              <div style="font-size:12px;color:var(--text-muted)">Estado Anthropic:</div>
              <div id="cs-claude-status" style="font-size:12px;font-weight:600">--</div>
            </div>
            <div style="display:flex;gap:8px;margin-top:8px;flex-wrap:wrap">
              <input id="cs-claude-key" type="password" placeholder="Pega aquí tu ANTHROPIC_API_KEY" style="min-width:300px;flex:1;padding:8px;border:1px solid var(--border-subtle);border-radius:8px;background:var(--surface-1);color:var(--text-primary)">
              <button class="action-btn primary" id="cs-claude-save">Guardar credencial</button>
              <button class="action-btn" id="cs-claude-clear">Borrar</button>
            </div>
            <div style="font-size:11px;color:var(--text-muted);margin-top:8px">
              Esta vinculación usa API key de Anthropic para controlar costos por uso dentro de ATLAS.
            </div>
          </div>
          <div style="display:flex;gap:8px">
            <textarea id="cs-objective" rows="3" placeholder="Objetivo para Supervisor (ej: revisa errores del módulo tools y define plan de corrección)" style="flex:1;padding:10px;border:1px solid var(--border-subtle);border-radius:8px;background:var(--surface-1);color:var(--text-primary)"></textarea>
            <button class="action-btn primary" id="cs-advise">Enviar tarea</button>
          </div>
          <div id="cs-log" style="flex:1;overflow:auto;background:var(--surface-0);border:1px solid var(--border-subtle);border-radius:10px;padding:12px;font-size:12px"></div>
          <div style="font-size:12px;color:var(--text-muted)">
            Costos Claude: configura tu clave Anthropic en <a href="#/config" style="color:var(--accent-primary)">AI Config</a>
            o usando <code>POST /api/brain/credentials</code> con <code>provider_id=anthropic</code>.
          </div>
        </div>
      </div>
    `;

    const elDaemon = container.querySelector('#cs-daemon');
    const elCount = container.querySelector('#cs-count');
    const elThread = container.querySelector('#cs-thread');
    const elObjective = container.querySelector('#cs-objective');
    const elLog = container.querySelector('#cs-log');
    const elClaudeBox = container.querySelector('#cs-claude-box');
    const elClaudeStatus = container.querySelector('#cs-claude-status');
    const elClaudeKey = container.querySelector('#cs-claude-key');
    let currentThreadId = '';

    function logLine(obj) {
      const txt = typeof obj === 'string' ? obj : JSON.stringify(obj, null, 2);
      elLog.textContent += `${new Date().toLocaleTimeString()} ${txt}\n\n`;
      elLog.scrollTop = elLog.scrollHeight;
    }

    async function refresh() {
      try {
        const [r1, r2, r3] = await Promise.all([
          fetch('/supervisor/daemon/status'),
          fetch('/supervisor/directives?limit=100'),
          fetch('/api/brain/credentials/status'),
        ]);
        const d1 = await r1.json();
        const d2 = await r2.json();
        const d3 = await r3.json();
        const daemonOk = Boolean(d1?.ok);
        const directives = Array.isArray(d2?.data) ? d2.data : [];
        const anthropicOk = Boolean(d3?.credentials?.anthropic?.configured);
        elDaemon.textContent = daemonOk ? 'ACTIVO' : 'NO DISPONIBLE';
        elCount.textContent = String(directives.length);
        elClaudeStatus.textContent = anthropicOk ? 'VINCULADO' : 'NO VINCULADO';
        logLine({ daemon: d1, directives: directives.slice(0, 20) });
      } catch (e) {
        logLine({ error: String(e) });
      }
    }

    container.querySelector('#cs-refresh')?.addEventListener('click', refresh);
    container.querySelector('#cs-open-workspace')?.addEventListener('click', () => window.open('/nexus', '_blank'));
    container.querySelector('#cs-link-claude')?.addEventListener('click', () => {
      elClaudeBox.style.display = elClaudeBox.style.display === 'none' ? 'block' : 'none';
    });
    container.querySelector('#cs-claude-save')?.addEventListener('click', async () => {
      const apiKey = (elClaudeKey.value || '').trim();
      if (!apiKey) return;
      try {
        const r = await fetch('/api/brain/credentials', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ provider_id: 'anthropic', api_key: apiKey }),
        });
        const d = await r.json();
        logLine({ claude_link: d });
        elClaudeKey.value = '';
        await refresh();
      } catch (e) {
        logLine({ error: String(e) });
      }
    });
    container.querySelector('#cs-claude-clear')?.addEventListener('click', async () => {
      try {
        const r = await fetch('/api/brain/credentials', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ provider_id: 'anthropic', api_key: '' }),
        });
        const d = await r.json();
        logLine({ claude_unlink: d });
        await refresh();
      } catch (e) {
        logLine({ error: String(e) });
      }
    });

    container.querySelector('#cs-advise')?.addEventListener('click', async () => {
      const objective = (elObjective.value || '').trim();
      if (!objective) return;
      try {
        const r = await fetch('/supervisor/advise', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            objective,
            user_id: 'owner',
            thread_id: currentThreadId || undefined,
            context: { source: 'v4-codex-supervisor' },
          }),
        });
        const d = await r.json();
        currentThreadId = String(d.thread_id || currentThreadId || '');
        elThread.textContent = currentThreadId ? _esc(currentThreadId.slice(0, 12)) : '--';
        logLine({ objective, result: d?.result || d });
      } catch (e) {
        logLine({ error: String(e) });
      }
    });

    refresh();
    return () => {};
  },
};

window.AtlasModuleCodexSupervisor = { id: 'codex-supervisor' };
