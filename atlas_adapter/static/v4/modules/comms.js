/**
 * ATLAS v4.2 â€” Communications Module
 * Panel de comunicaciones: estado WAHA/WhatsApp, QR, restart, test.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'comms-module';

function _esc(s) { const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML; }

export default {
  id: 'comms',
  label: 'Comunicaciones',
  icon: 'message-circle',
  category: 'configuration',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Inicio
          </button>
          <h2>Comunicaciones</h2>
          <div style="margin-left:auto"><span class="live-badge">LIVE</span></div>
        </div>
        <div class="module-body">

          <!-- Status -->
          <div class="stat-row" style="margin-bottom:16px">
            <div class="stat-card hero">
              <div class="stat-card-label">Canal Principal</div>
              <div class="stat-card-value" id="c-channel">--</div>
              <div class="stat-card-sub" id="c-phone"></div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Estado WAHA</div>
              <div class="stat-card-value" id="c-state">--</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Mensajes enviados</div>
              <div class="stat-card-value accent" id="c-sent">--</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Cola offline</div>
              <div class="stat-card-value" id="c-offline">--</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">IA p95</div>
              <div class="stat-card-value" id="c-p95">--</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">IA p99</div>
              <div class="stat-card-value" id="c-p99">--</div>
            </div>
          </div>

          <!-- Actions -->
          <div class="section-title">Acciones</div>
          <div class="action-bar">
            <button class="action-btn primary" id="btn-qr">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="3" y="3" width="7" height="7"/><rect x="14" y="3" width="7" height="7"/><rect x="3" y="14" width="7" height="7"/><path d="M14 14h3v3M17 14v.01M14 17v.01M17 17v.01"/></svg>
              Ver QR WhatsApp
            </button>
            <button class="action-btn" id="btn-restart">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="23 4 23 10 17 10"/><path d="M20.49 15a9 9 0 11-2.12-9.36L23 10"/></svg>
              Reiniciar WAHA
            </button>
            <button class="action-btn" id="btn-test">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M9 11l3 3L22 4"/><path d="M21 12v7a2 2 0 01-2 2H5a2 2 0 01-2-2V5a2 2 0 012-2h11"/></svg>
              Test de comms
            </button>
          </div>

          <!-- Action output -->
          <div id="comms-action" style="margin-top:12px"></div>

          <!-- Raw status -->
          <div class="section-title" style="margin-top:20px">DiagnÃ³stico</div>
          <pre class="codebox" id="comms-raw" style="max-height:180px;overflow:auto">Cargando...</pre>

          <div class="section-title" style="margin-top:16px">Historial ATLAS (movil)</div>
          <div id="c-history" style="display:flex;flex-direction:column;gap:8px">
            <div class="codebox" style="font-size:11px">Cargando historial...</div>
          </div>
        </div>
      </div>
    `;

    container.querySelector('#btn-qr')?.addEventListener('click',      () => _qr(container));
    container.querySelector('#btn-restart')?.addEventListener('click',  () => _restart(container));
    container.querySelector('#btn-test')?.addEventListener('click',     () => _test(container));

    _refresh(container);
    poll(POLL_ID, '/ans/comms/status', 6000, () => _refresh(container));
  },

  destroy() { stop(POLL_ID); },
};

async function _refresh(container) {
  try {
    const [ansResp, atlasResp, histResp] = await Promise.all([
      fetch('/ans/comms/status'),
      fetch('/api/comms/atlas/status'),
      fetch('/api/comms/atlas/history?limit=12'),
    ]);
    const ansData = await ansResp.json().catch(() => ({}));
    const atlasData = await atlasResp.json().catch(() => ({}));
    const histData = await histResp.json().catch(() => ({}));
    _renderStatus(container, ansData, atlasData, histData);
    _renderHistory(container, histData?.items || []);
  } catch (e) {
    const el = container.querySelector('#comms-raw');
    if (el) el.textContent = `Error: ${e.message}`;
  }
}

function _renderStatus(container, data, atlasData = {}, histData = {}) {
  const p = data?.data ?? data;
  const connected = p?.connected === true || p?.status === 'connected' || p?.state === 'connected';
  const txt = (id, v) => { const e = container.querySelector(id); if (e) e.textContent = v; };
  txt('#c-channel', p?.channel || p?.provider || 'WhatsApp');
  txt('#c-phone',   p?.phone || p?.number || '');
  txt('#c-state',   p?.state || p?.status || (connected ? 'Conectado' : 'Desconectado'));
  const stEl = container.querySelector('#c-state');
  if (stEl) stEl.style.color = connected ? 'var(--accent-green)' : 'var(--accent-orange)';
  txt('#c-sent', p?.messages_sent ?? p?.sent ?? '--');
  txt('#c-offline', atlasData?.queue_pending ?? '--');
  const metrics = histData?.metrics || {};
  txt('#c-p95', typeof metrics?.p95_ms === 'number' ? `${metrics.p95_ms}ms` : '--');
  txt('#c-p99', typeof metrics?.p99_ms === 'number' ? `${metrics.p99_ms}ms` : '--');
  const qEl = container.querySelector('#c-offline');
  if (qEl && typeof atlasData?.queue_pending === 'number') {
    qEl.style.color = atlasData.queue_pending > 0 ? 'var(--accent-orange)' : 'var(--accent-green)';
  }
  const p95El = container.querySelector('#c-p95');
  if (p95El && typeof metrics?.p95_ms === 'number') {
    p95El.style.color = metrics.p95_ms > 2500 ? 'var(--accent-orange)' : 'var(--accent-green)';
  }
  const p99El = container.querySelector('#c-p99');
  if (p99El && typeof metrics?.p99_ms === 'number') {
    p99El.style.color = metrics.p99_ms > 3500 ? 'var(--accent-orange)' : 'var(--accent-green)';
  }
  const raw = container.querySelector('#comms-raw');
  if (raw) {
    raw.textContent = JSON.stringify({
      ans: p,
      atlas_bridge: atlasData,
      history_metrics: metrics,
    }, null, 2);
  }
}

function _renderHistory(container, items) {
  const el = container.querySelector('#c-history');
  if (!el) return;
  const rows = Array.isArray(items) ? items : [];
  if (!rows.length) {
    el.innerHTML = `<div class="codebox" style="font-size:11px">Sin interacciones registradas.</div>`;
    return;
  }
  el.innerHTML = rows.slice(0, 12).map((it) => {
    const when = it?.created_at ? new Date(it.created_at).toLocaleTimeString() : '--';
    const user = _esc(it?.user_id || 'guest');
    const contact = _esc(it?.contact_name || '');
    const req = _esc(it?.request || it?.request_summary || '');
    const rsp = _esc(it?.response || it?.response_summary || '');
    const urgency = _esc(it?.urgency || 'normal');
    const provider = _esc(it?.provider || '--');
    const latency = typeof it?.latency_ms === 'number' ? `${it.latency_ms}ms` : '--';
    const off = it?.offline_mode ? '<span class="chip orange">offline</span>' : '<span class="chip green">online</span>';
    return `<div class="codebox" style="font-size:11px;line-height:1.45">
      <div style="display:flex;justify-content:space-between;gap:8px;margin-bottom:4px">
        <strong>${user}</strong>
        <span>${when} Â· ${urgency}</span>
      </div>
      <div style="opacity:.86"><strong>Perfil:</strong> ${contact || '--'} · <strong>IA:</strong> ${provider} · <strong>Latencia:</strong> ${latency}</div>
      <div><strong>Cliente:</strong> ${req}</div>
      <div style="margin-top:2px"><strong>ATLAS:</strong> ${rsp}</div>
      <div style="margin-top:4px">${off}</div>
    </div>`;
  }).join('');
}

function _setAction(container, html) {
  const el = container.querySelector('#comms-action');
  if (el) el.innerHTML = html || '';
}

async function _qr(container) {
  _setAction(container, '<div style="display:flex;gap:8px;align-items:center"><div class="spinner"></div><span style="font-size:12px;color:var(--text-muted)">Obteniendo QR...</span></div>');
  try {
    const r = await fetch('/ans/comms/waha/qr');
    const data = await r.json().catch(() => ({}));
    if (!r.ok || data.ok === false) throw new Error(data.error || `HTTP ${r.status}`);
    const qr = (data.data && (data.data.qr || data.data.url)) || data.qr || data.url || '';
    if (qr) {
      _setAction(container, `<div style="display:flex;gap:16px;align-items:flex-start;flex-wrap:wrap">
        <img alt="WAHA QR" src="${_esc(qr)}" style="width:200px;height:200px;border-radius:12px;border:1px solid var(--border);background:white;object-fit:contain">
        <div style="flex:1;min-width:200px">
          <div class="stat-card-label" style="margin-bottom:6px">Escanea con WhatsApp</div>
          <div class="codebox" style="word-break:break-all;font-size:10px">${_esc(qr.slice(0, 200))}${qr.length > 200 ? '...' : ''}</div>
        </div>
      </div>`);
    } else {
      _setAction(container, `<pre class="codebox">${_esc(JSON.stringify(data, null, 2))}</pre>`);
    }
  } catch (e) {
    _setAction(container, `<div style="color:var(--accent-red);font-size:12px">Error: ${_esc(e.message)}</div>`);
  }
}

async function _restart(container) {
  _setAction(container, '<div style="display:flex;gap:8px;align-items:center"><div class="spinner"></div><span style="font-size:12px;color:var(--text-muted)">Reiniciando WAHA...</span></div>');
  try {
    const r = await fetch('/ans/comms/waha/restart', { method: 'POST' });
    const data = await r.json().catch(() => ({}));
    if (!r.ok || data.ok === false) throw new Error(data.error || `HTTP ${r.status}`);
    window.AtlasToast?.show('WAHA reiniciado', 'success');
    _setAction(container, `<div class="chip green">WAHA reiniciado correctamente</div>`);
    setTimeout(() => _refresh(container), 2000);
  } catch (e) {
    window.AtlasToast?.show(e.message, 'error');
    _setAction(container, `<div style="color:var(--accent-red);font-size:12px">Error: ${_esc(e.message)}</div>`);
  }
}

async function _test(container) {
  _setAction(container, '<div style="display:flex;gap:8px;align-items:center"><div class="spinner"></div><span style="font-size:12px;color:var(--text-muted)">Ejecutando test...</span></div>');
  try {
    const r = await fetch('/ans/comms/test', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' });
    const data = await r.json().catch(() => ({}));
    if (!r.ok || data.ok === false) throw new Error(data.error || `HTTP ${r.status}`);
    window.AtlasToast?.show('Test OK', 'success');
    _setAction(container, `<pre class="codebox">${_esc(JSON.stringify(data, null, 2))}</pre>`);
  } catch (e) {
    window.AtlasToast?.show(e.message, 'error');
    _setAction(container, `<div style="color:var(--accent-red);font-size:12px">Error: ${_esc(e.message)}</div>`);
  }
}

window.AtlasModuleComms = { id: 'comms' };

