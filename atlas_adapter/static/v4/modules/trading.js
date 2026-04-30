/**
 * ATLAS v4.2 — Trading Module
 * Panel de análisis de trading vía Grok/xAI proxy.
 */
function _esc(s) { const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML; }

const QUICK_QUERIES = [
  'Analiza BTC/USD en las últimas 24h',
  'Señales de trading para ETH hoy',
  'Resumen de mercados cripto actuales',
  'Análisis de riesgo para posición larga en BTC',
];

export default {
  id: 'trading',
  label: 'Trading',
  icon: 'trending-up',
  category: 'tools',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Inicio
          </button>
          <h2>Trading AI</h2>
          <div style="margin-left:auto">
            <span class="chip blue">Powered by Grok</span>
          </div>
        </div>
        <div class="module-body">

          <!-- Stats row -->
          <div class="stat-row" style="margin-bottom:20px">
            <div class="stat-card hero">
              <div class="stat-card-label">Motor de Análisis</div>
              <div class="stat-card-value accent" style="font-size:20px">Grok / xAI</div>
              <div class="stat-card-sub">Análisis de mercados en tiempo real</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Consultas realizadas</div>
              <div class="stat-card-value" id="t-count">0</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Última consulta</div>
              <div class="stat-card-value" id="t-last" style="font-size:13px">--</div>
            </div>
          </div>

          <!-- Quick actions -->
          <div class="section-title">Consultas Rápidas</div>
          <div class="action-bar" style="flex-wrap:wrap">
            ${QUICK_QUERIES.map((q, i) => `
              <button class="action-btn" style="font-size:11px" data-q="${_esc(q)}" id="tq-${i}">${_esc(q)}</button>
            `).join('')}
          </div>

          <!-- Input -->
          <div class="section-title" style="margin-top:16px">Consulta Personalizada</div>
          <div style="display:flex;flex-direction:column;gap:8px">
            <textarea id="t-query" class="feedback-box" style="min-height:80px"
              placeholder="Pregunta al AI de trading... Ej: ¿Cuál es tu análisis de BTC/USD?"></textarea>
            <div style="display:flex;gap:8px">
              <button class="action-btn primary" id="t-send">
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><line x1="22" y1="2" x2="11" y2="13"/><polygon points="22 2 15 22 11 13 2 9 22 2"/></svg>
                Analizar
              </button>
              <button class="action-btn" id="t-clear">Limpiar</button>
            </div>
          </div>

          <!-- Response -->
          <div class="section-title" style="margin-top:20px">
            Análisis
            <span class="count" id="t-model"></span>
          </div>
          <div id="t-response" class="codebox" style="min-height:120px;white-space:pre-wrap;word-break:break-word;font-family:var(--font);font-size:13px;line-height:1.6">
            <span style="color:var(--text-muted)">El análisis aparecerá aquí...</span>
          </div>

        </div>
      </div>
    `;

    let queryCount = 0;

    // Quick query buttons
    QUICK_QUERIES.forEach((q, i) => {
      container.querySelector(`#tq-${i}`)?.addEventListener('click', () => {
        const ta = container.querySelector('#t-query');
        if (ta) ta.value = q;
        _send(container, q, () => queryCount++, () => {
          const el = container.querySelector('#t-count');
          if (el) el.textContent = ++queryCount;
          const last = container.querySelector('#t-last');
          if (last) last.textContent = new Date().toLocaleTimeString();
        });
      });
    });

    // Send button
    container.querySelector('#t-send')?.addEventListener('click', () => {
      const q = (container.querySelector('#t-query')?.value || '').trim();
      if (!q) return;
      _send(container, q, null, () => {
        const el = container.querySelector('#t-count');
        if (el) el.textContent = ++queryCount;
        const last = container.querySelector('#t-last');
        if (last) last.textContent = new Date().toLocaleTimeString();
      });
    });

    // Clear
    container.querySelector('#t-clear')?.addEventListener('click', () => {
      const ta = container.querySelector('#t-query');
      if (ta) ta.value = '';
      const resp = container.querySelector('#t-response');
      if (resp) resp.innerHTML = `<span style="color:var(--text-muted)">El análisis aparecerá aquí...</span>`;
    });
  },

  destroy() {},
};

async function _send(container, query, onStart, onDone) {
  const resp = container.querySelector('#t-response');
  const model = container.querySelector('#t-model');
  if (resp) resp.innerHTML = `<div style="display:flex;gap:8px;align-items:center"><div class="spinner"></div><span style="color:var(--text-muted)">Consultando Grok...</span></div>`;
  if (onStart) onStart();

  try {
    const r = await fetch('/api/trading/proxy', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ prompt: query, max_tokens: 1024 }),
    });
    const data = await r.json().catch(() => ({}));
    if (!r.ok || data.ok === false) throw new Error(data.error || `HTTP ${r.status}`);

    const text = data?.data?.content
      || data?.data?.text
      || data?.data?.response
      || data?.response
      || data?.content
      || (typeof data?.data === 'string' ? data.data : null)
      || JSON.stringify(data, null, 2);

    const usedModel = data?.data?.model || data?.model || 'grok';
    if (model) model.textContent = usedModel;

    if (resp) {
      resp.style.color = 'var(--text-secondary)';
      resp.textContent = text;
    }
    if (onDone) onDone();
  } catch (e) {
    if (resp) {
      resp.style.color = 'var(--accent-red)';
      resp.textContent = `Error: ${e.message}`;
    }
    window.AtlasToast?.show(e.message, 'error');
  }
}

window.AtlasModuleTrading = { id: 'trading' };
