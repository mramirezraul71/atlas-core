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
          <div class="section-title">Vincular WhatsApp</div>
          <div class="action-bar" style="flex-wrap:wrap;gap:8px">
            <button class="action-btn" id="btn-pairing">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="5" y="2" width="14" height="20" rx="2" ry="2"/><line x1="12" y1="18" x2="12.01" y2="18"/></svg>
              Código de emparejamiento
            </button>
            <button class="action-btn" id="btn-qr">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="3" y="3" width="7" height="7"/><rect x="14" y="3" width="7" height="7"/><rect x="3" y="14" width="7" height="7"/><path d="M14 14h3v3M17 14v.01M14 17v.01M17 17v.01"/></svg>
              Ver QR
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

          <!-- Meta WhatsApp Business API wizard -->
          <div class="section-title" style="margin-top:20px">
            Meta WhatsApp Business API
            <span class="chip" id="meta-status-chip" style="margin-left:8px;font-size:10px">--</span>
          </div>
          <div id="meta-wizard" style="margin-top:8px"></div>

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

    container.querySelector('#btn-pairing')?.addEventListener('click',  () => _pairingCode(container));
    container.querySelector('#btn-qr')?.addEventListener('click',      () => _qr(container));
    container.querySelector('#btn-restart')?.addEventListener('click',  () => _restart(container));
    container.querySelector('#btn-test')?.addEventListener('click',     () => _test(container));

    _refresh(container);
    _metaWizard(container);
    poll(POLL_ID, '/api/comms/whatsapp/status', 8000, () => _refresh(container));
  },

  destroy() { stop(POLL_ID); },
};

async function _refresh(container) {
  try {
    const [waResp, atlasResp, histResp] = await Promise.all([
      fetch('/api/comms/whatsapp/status'),
      fetch('/api/comms/atlas/status'),
      fetch('/api/comms/atlas/history?limit=12'),
    ]);
    const ansData = await waResp.json().catch(() => ({}));
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
  const connected = p?.ready === true || p?.provider_status?.authenticated === true ||
    p?.connected === true || p?.status === 'connected' || p?.state === 'connected';
  const txt = (id, v) => { const e = container.querySelector(id); if (e) e.textContent = v; };
  txt('#c-channel', p?.channel || p?.provider || 'WhatsApp');
  txt('#c-phone',   p?.to_number || p?.phone || p?.number || '');
  const wStatus = p?.provider_status?.status || p?.state || p?.status || (connected ? 'Conectado' : 'Desconectado');
  txt('#c-state', wStatus);
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

async function _pairingCode(container) {
  // Mostrar input para número de teléfono
  _setAction(container, `
    <div style="display:flex;flex-direction:column;gap:10px;max-width:420px">
      <div class="stat-card-label">Código de emparejamiento — sin QR</div>
      <div style="font-size:12px;color:var(--text-secondary)">
        Introduce el número de teléfono asociado a tu cuenta WhatsApp.<br>
        Luego en WhatsApp → <strong>Dispositivos vinculados → Vincular con número de teléfono</strong>.
      </div>
      <div style="display:flex;gap:8px;align-items:center">
        <input id="pairing-phone" type="tel" placeholder="+34612345678"
          style="flex:1;background:var(--bg-elevated);border:1px solid var(--border);border-radius:6px;
                 padding:7px 10px;color:var(--text-primary);font-size:13px;outline:none" />
        <button class="action-btn primary" id="btn-pairing-confirm">Obtener código</button>
      </div>
    </div>
  `);
  container.querySelector('#btn-pairing-confirm')?.addEventListener('click', async () => {
    const phone = (container.querySelector('#pairing-phone')?.value || '').trim();
    if (!phone) { window.AtlasToast?.show('Introduce un número de teléfono', 'error'); return; }
    _setAction(container, '<div style="display:flex;gap:8px;align-items:center"><div class="spinner"></div><span style="font-size:12px;color:var(--text-muted)">Solicitando código...</span></div>');
    try {
      const r = await fetch('/api/comms/whatsapp/pairing-code', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ phone }),
      });
      const data = await r.json().catch(() => ({}));
      if (data.ok === false) throw new Error(data.error || `HTTP ${r.status}`);
      const code = data.code || '--';
      _setAction(container, `
        <div style="display:flex;flex-direction:column;gap:10px;max-width:420px">
          <div class="stat-card-label" style="color:var(--accent-green)">Código de emparejamiento listo</div>
          <div style="font-size:38px;font-weight:700;letter-spacing:6px;font-family:var(--font-mono);
                      text-align:center;padding:16px;background:var(--bg-elevated);border-radius:8px;
                      border:1px solid var(--accent-green);color:var(--accent-green)">
            ${_esc(code)}
          </div>
          <div style="font-size:12px;color:var(--text-secondary);line-height:1.5">
            En tu WhatsApp → <strong>Ajustes → Dispositivos vinculados → Vincular un dispositivo</strong><br>
            Pulsa <strong>"Vincular con número de teléfono"</strong> e introduce este código.<br>
            <span style="color:var(--accent-orange)">El código expira en ~60 segundos.</span>
          </div>
        </div>
      `);
    } catch (e) {
      _setAction(container, `<div style="color:var(--accent-red);font-size:12px">Error: ${_esc(e.message)}</div>`);
    }
  });
}

async function _qr(container) {
  _setAction(container, '<div style="display:flex;gap:8px;align-items:center"><div class="spinner"></div><span style="font-size:12px;color:var(--text-muted)">Obteniendo QR...</span></div>');
  try {
    const r = await fetch('/api/comms/whatsapp/qr');
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
    const r = await fetch('/api/comms/whatsapp/start-session', { method: 'POST' });
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
    const r = await fetch('/api/comms/whatsapp/test', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' });
    const data = await r.json().catch(() => ({}));
    if (!r.ok || data.ok === false) throw new Error(data.error || `HTTP ${r.status}`);
    window.AtlasToast?.show('Test OK', 'success');
    _setAction(container, `<pre class="codebox">${_esc(JSON.stringify(data, null, 2))}</pre>`);
  } catch (e) {
    window.AtlasToast?.show(e.message, 'error');
    _setAction(container, `<div style="color:var(--accent-red);font-size:12px">Error: ${_esc(e.message)}</div>`);
  }
}

function _addProgressLine(el, msg, isError = false) {
  if (!el) return;
  const line = document.createElement('div');
  line.style.cssText = `color:${isError ? 'var(--accent-red)' : 'var(--text-secondary)'};padding:1px 0`;
  line.textContent = msg;
  el.appendChild(line);
  el.scrollTop = el.scrollHeight;
}

let _autosetupPollTimer = null;
function _pollAutosetup(container, progress, btn) {
  if (_autosetupPollTimer) clearInterval(_autosetupPollTimer);
  let lastLogCount = 0;
  _autosetupPollTimer = setInterval(async () => {
    try {
      const r = await fetch('/api/comms/meta-whatsapp/autosetup/status');
      const d = await r.json().catch(() => ({}));
      // Añadir nuevos logs
      const logs = d.logs || [];
      for (let i = lastLogCount; i < logs.length; i++) {
        _addProgressLine(progress, `[${logs[i].ts}] ${logs[i].msg}`);
      }
      lastLogCount = logs.length;
      // Detectar fin
      if (d.finished || (!d.running && d.exit_code !== null && d.exit_code !== undefined)) {
        clearInterval(_autosetupPollTimer);
        _autosetupPollTimer = null;
        if (d.success || d.meta_token_saved || d.phone_id_saved) {
          _addProgressLine(progress, '[Atlas] ✓ Configuración completada. Recargando wizard...');
          window.AtlasToast?.show('Meta WhatsApp configurado por Atlas', 'success');
          setTimeout(() => _metaWizard(container), 1000);
        } else {
          _addProgressLine(progress, '[Atlas] Proceso terminado. Revisa logs y completa manualmente si falta token.', false);
          window.AtlasToast?.show('Proceso terminado — revisa el wizard', 'warn');
        }
        if (btn) {
          btn.disabled = false;
          btn.innerHTML = '<svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polygon points="13 2 3 14 12 14 11 22 21 10 12 10 13 2"/></svg> Que lo haga Atlas (autónomo)';
        }
      }
    } catch (e) {
      _addProgressLine(progress, `[poll error] ${e.message}`, true);
    }
  }, 2000);
}

async function _metaWizard(container) {
  const el = container.querySelector('#meta-wizard');
  const chip = container.querySelector('#meta-status-chip');
  if (!el) return;

  let info;
  try {
    const r = await fetch('/api/comms/meta-whatsapp/setup-info');
    info = await r.json().catch(() => ({}));
  } catch {
    el.innerHTML = `<div style="font-size:12px;color:var(--text-muted)">No disponible</div>`;
    return;
  }

  const ready = info.provider_ready;
  if (chip) {
    chip.textContent = ready ? 'ACTIVO' : 'Pendiente configuración';
    chip.style.background = ready ? 'var(--accent-green)' : 'var(--accent-orange)';
    chip.style.color = '#000';
  }

  const stepIcon = (done) => done
    ? `<span style="color:var(--accent-green);font-weight:700">✓</span>`
    : `<span style="color:var(--accent-orange)">○</span>`;

  el.innerHTML = `
    <div style="display:flex;flex-direction:column;gap:10px;max-width:560px">
      <div style="font-size:12px;color:var(--text-secondary);line-height:1.6">
        Canal de producción gratuito (1000 conversaciones/mes). Atlas configura el webhook automáticamente.
      </div>

      <!-- Steps -->
      <div style="display:flex;flex-direction:column;gap:6px">
        <div style="display:flex;gap:8px;align-items:center;font-size:12px">
          ${stepIcon(true)} <strong>Webhook URL:</strong>
          <code style="background:var(--bg-elevated);padding:2px 6px;border-radius:4px;font-size:11px;word-break:break-all">${_esc(info.webhook_url || '')}</code>
        </div>
        <div style="display:flex;gap:8px;align-items:center;font-size:12px">
          ${stepIcon(true)} <strong>Verify Token:</strong>
          <code style="background:var(--bg-elevated);padding:2px 6px;border-radius:4px;font-size:11px">${_esc(info.verify_token || '')}</code>
        </div>
        <div style="display:flex;gap:8px;align-items:center;font-size:12px">
          ${stepIcon(info.meta_token_set)} <strong>Meta Token:</strong>
          <span style="color:var(--text-secondary)">${info.meta_token_set ? 'Configurado ✓' : 'Pendiente — pega tu EAAxxxxx abajo'}</span>
        </div>
        <div style="display:flex;gap:8px;align-items:center;font-size:12px">
          ${stepIcon(info.phone_number_id_set)} <strong>Phone Number ID:</strong>
          <span style="color:var(--text-secondary)">${info.phone_number_id_set ? 'Configurado ✓' : 'Pendiente'}</span>
        </div>
      </div>

      <!-- Autosetup button -->
      <div style="display:flex;gap:8px;align-items:center;flex-wrap:wrap">
        <button class="action-btn primary" id="btn-meta-autosetup" style="font-size:12px;padding:7px 14px">
          <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polygon points="13 2 3 14 12 14 11 22 21 10 12 10 13 2"/></svg>
          Que lo haga Atlas (autónomo)
        </button>
        <button class="action-btn" id="btn-meta-browser" style="font-size:11px">
          <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="10"/><path d="M2 12h20M12 2a15.3 15.3 0 010 20"/></svg>
          Solo abrir navegador
        </button>
      </div>

      <!-- Progress live -->
      <div id="meta-autosetup-progress" style="display:none;flex-direction:column;gap:4px;padding:10px;background:var(--bg-elevated);border-radius:8px;border:1px solid var(--border);max-height:200px;overflow-y:auto;font-size:11px;font-family:var(--font-mono)"></div>

      <!-- Credentials form -->
      ${!ready ? `
      <div style="display:flex;flex-direction:column;gap:6px;padding:12px;background:var(--bg-elevated);border-radius:8px;border:1px solid var(--border)">
        <div style="font-size:11px;color:var(--text-secondary);margin-bottom:4px">
          Tras crear tu app Meta, copia los datos aquí y Atlas los guarda en .env automáticamente:
        </div>
        <input id="meta-token-input" type="text" placeholder="Meta Token (EAAxxxxxx)"
          style="background:var(--bg);border:1px solid var(--border);border-radius:6px;padding:7px 10px;color:var(--text-primary);font-size:12px;outline:none" />
        <input id="meta-phoneid-input" type="text" placeholder="Phone Number ID (ej: 123456789012345)"
          style="background:var(--bg);border:1px solid var(--border);border-radius:6px;padding:7px 10px;color:var(--text-primary);font-size:12px;outline:none" />
        <input id="meta-to-input" type="tel" placeholder="Número para notificaciones Atlas (ej: +34612345678)"
          style="background:var(--bg);border:1px solid var(--border);border-radius:6px;padding:7px 10px;color:var(--text-primary);font-size:12px;outline:none" />
        <button class="action-btn primary" id="btn-meta-save" style="margin-top:4px">
          Guardar y activar proveedor Meta
        </button>
        <div id="meta-save-result"></div>
      </div>` : `
      <div class="chip green" style="width:fit-content">Meta WhatsApp Business API activo</div>`}
    </div>
  `;

  container.querySelector('#btn-meta-autosetup')?.addEventListener('click', async () => {
    const btn = container.querySelector('#btn-meta-autosetup');
    const progress = container.querySelector('#meta-autosetup-progress');
    if (btn) { btn.disabled = true; btn.textContent = '⟳ Iniciando agente...'; }
    if (progress) { progress.style.display = 'flex'; progress.innerHTML = ''; }
    try {
      const r = await fetch('/api/comms/meta-whatsapp/autosetup', { method: 'POST' });
      const d = await r.json().catch(() => ({}));
      if (!d.ok) throw new Error(d.error || 'Error al lanzar agente');
      _addProgressLine(progress, `[Atlas] Agente iniciado (PID ${d.pid})`);
      _addProgressLine(progress, '[Atlas] Chromium se abrirá en tu pantalla');
      _addProgressLine(progress, '[Atlas] Inicia sesión en Meta cuando aparezca el navegador...');
      if (btn) btn.textContent = '⟳ Agente corriendo...';
      _pollAutosetup(container, progress, btn);
    } catch (e) {
      if (progress) _addProgressLine(progress, `[ERROR] ${e.message}`, true);
      if (btn) { btn.disabled = false; btn.innerHTML = '<svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polygon points="13 2 3 14 12 14 11 22 21 10 12 10 13 2"/></svg> Que lo haga Atlas (autónomo)'; }
    }
  });

  container.querySelector('#btn-meta-browser')?.addEventListener('click', async () => {
    const btn = container.querySelector('#btn-meta-browser');
    if (btn) { btn.disabled = true; btn.textContent = '⟳ Abriendo...'; }
    try {
      const r = await fetch('/api/comms/meta-whatsapp/open-browser', { method: 'POST' });
      const d = await r.json().catch(() => ({}));
      if (d.ok) {
        window.AtlasToast?.show('Navegador abierto en Meta Developers', 'success');
      } else if (d.manual_url) {
        window.AtlasToast?.show('Playwright no activo — abre la URL manualmente', 'warn');
        window.open(d.manual_url, '_blank');
      } else {
        window.AtlasToast?.show(d.error || 'Error', 'error');
      }
    } finally {
      if (btn) {
        btn.disabled = false;
        btn.innerHTML = '<svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="10"/><path d="M2 12h20M12 2a15.3 15.3 0 010 20M12 2a15.3 15.3 0 000 20"/></svg> Abrir Meta Developers (Atlas)';
      }
    }
  });

  container.querySelector('#btn-meta-save')?.addEventListener('click', async () => {
    const token = (container.querySelector('#meta-token-input')?.value || '').trim();
    const phoneId = (container.querySelector('#meta-phoneid-input')?.value || '').trim();
    const to = (container.querySelector('#meta-to-input')?.value || '').trim();
    const resEl = container.querySelector('#meta-save-result');
    if (!token || !phoneId) {
      if (resEl) resEl.innerHTML = `<div style="color:var(--accent-red);font-size:12px">Token y Phone Number ID son requeridos</div>`;
      return;
    }
    if (resEl) resEl.innerHTML = '<div class="spinner" style="width:16px;height:16px"></div>';
    try {
      const r = await fetch('/api/comms/meta-whatsapp/save-credentials', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ meta_token: token, phone_number_id: phoneId, whatsapp_to: to }),
      });
      const d = await r.json().catch(() => ({}));
      if (d.ok) {
        window.AtlasToast?.show('Meta WhatsApp activado', 'success');
        setTimeout(() => _metaWizard(container), 500);
      } else {
        if (resEl) resEl.innerHTML = `<div style="color:var(--accent-red);font-size:12px">Error: ${_esc(d.error || 'unknown')}</div>`;
      }
    } catch (e) {
      if (resEl) resEl.innerHTML = `<div style="color:var(--accent-red);font-size:12px">${_esc(e.message)}</div>`;
    }
  });
}

window.AtlasModuleComms = { id: 'comms' };

