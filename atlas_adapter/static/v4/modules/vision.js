/**
 * ATLAS v4.2 — Vision Module
 * Control de cámaras, captura de snapshots y stream en tiempo real.
 * Endpoints: /vision/cameras, /vision/capture/{id}, /vision/scan
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'vision-module';

function _esc(s) { const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML; }

export default {
  id: 'vision',
  label: 'Visión',
  icon: 'eye',
  category: 'monitoring',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" id="vis-back">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Dashboard
          </button>
          <h2>Visión Artificial</h2>
          <div style="margin-left:auto;display:flex;gap:8px;align-items:center">
            <span id="vis-stream-badge" style="display:none" class="live-badge" style="background:var(--accent-red)">● STREAM</span>
            <span class="live-badge">LIVE</span>
          </div>
        </div>

        <div class="module-body">

          <!-- KPIs -->
          <div class="stat-row" style="margin-bottom:20px">
            <div class="stat-card hero">
              <div class="stat-card-label">Cámaras detectadas</div>
              <div class="stat-card-value accent" id="vis-cam-count">--</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Cámara seleccionada</div>
              <div class="stat-card-value" id="vis-selected" style="font-size:14px">Ninguna</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Estado stream</div>
              <div class="stat-card-value" id="vis-stream-state" style="font-size:14px">Detenido</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Última captura</div>
              <div class="stat-card-value" id="vis-last-ts" style="font-size:12px">--</div>
            </div>
          </div>

          <!-- Acciones -->
          <div class="action-bar" style="padding-top:0;margin-bottom:16px;flex-wrap:wrap">
            <button class="action-btn" id="btn-vis-scan">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="23 4 23 10 17 10"/><path d="M20.49 15a9 9 0 11-2.12-9.36L23 10"/></svg>
              Escanear cámaras
            </button>
            <button class="action-btn primary" id="btn-vis-capture" disabled>
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="3"/><path d="M20.94 11A8.994 8.994 0 1112.06 3"/></svg>
              Capturar snapshot
            </button>
            <button class="action-btn success" id="btn-vis-stream" disabled>
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polygon points="23 7 16 12 23 17 23 7"/><rect x="1" y="5" width="15" height="14" rx="2" ry="2"/></svg>
              Iniciar stream
            </button>
            <button class="action-btn danger" id="btn-vis-stop" disabled>
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="3" y="3" width="18" height="18" rx="2"/></svg>
              Detener
            </button>
            <span id="vis-msg" style="font-size:11px;color:var(--text-muted);align-self:center"></span>
          </div>

          <!-- Lista de cámaras -->
          <div class="section-title">Cámaras Disponibles</div>
          <div id="vis-cameras-list" style="margin-bottom:20px">
            <div class="empty-state">
              <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5"><path d="M23 19a2 2 0 01-2 2H3a2 2 0 01-2-2V8a2 2 0 012-2h4l2-3h6l2 3h4a2 2 0 012 2z"/><circle cx="12" cy="13" r="4"/></svg>
              <div class="empty-title">Sin cámaras</div>
              <div class="empty-sub">Haz clic en "Escanear cámaras" para detectarlas</div>
            </div>
          </div>

          <!-- Viewport de imagen/stream -->
          <div class="section-title">Vista en Vivo</div>
          <div id="vis-viewport" style="background:var(--surface-0);border:1px solid var(--border-subtle);border-radius:12px;overflow:hidden;margin-bottom:20px;min-height:240px;display:flex;align-items:center;justify-content:center;position:relative">
            <div style="text-align:center;color:var(--text-muted)">
              <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1"><path d="M23 19a2 2 0 01-2 2H3a2 2 0 01-2-2V8a2 2 0 012-2h4l2-3h6l2 3h4a2 2 0 012 2z"/><circle cx="12" cy="13" r="4"/></svg>
              <div style="margin-top:8px;font-size:13px">Selecciona una cámara y captura</div>
            </div>
          </div>

          <!-- Info de cámara activa -->
          <div id="vis-cam-info" style="display:none">
            <div class="section-title">Información de Cámara</div>
            <pre class="codebox" id="vis-cam-raw" style="max-height:180px;overflow-y:auto;font-size:11px"></pre>
          </div>

        </div>
      </div>
    `;

    let selectedCamId = null;
    let streamInterval = null;

    container.querySelector('#vis-back')?.addEventListener('click', () => { location.hash = '/'; });
    container.querySelector('#btn-vis-scan')?.addEventListener('click', () => _scanCameras(container));

    container.querySelector('#btn-vis-capture')?.addEventListener('click', () => {
      if (selectedCamId) _captureSnapshot(selectedCamId, container);
    });

    container.querySelector('#btn-vis-stream')?.addEventListener('click', () => {
      if (selectedCamId) {
        const btn  = container.querySelector('#btn-vis-stream');
        const stop = container.querySelector('#btn-vis-stop');
        const badge = container.querySelector('#vis-stream-badge');
        const stateEl = container.querySelector('#vis-stream-state');
        streamInterval = setInterval(() => _captureSnapshot(selectedCamId, container, true), 1500);
        if (btn)   btn.disabled = true;
        if (stop)  stop.disabled = false;
        if (badge) badge.style.display = '';
        if (stateEl) { stateEl.textContent = 'Streaming'; stateEl.style.color = 'var(--accent-red)'; }
        container._streamInterval = streamInterval;
      }
    });

    container.querySelector('#btn-vis-stop')?.addEventListener('click', () => {
      clearInterval(container._streamInterval);
      const streamBtn = container.querySelector('#btn-vis-stream');
      const stopBtn   = container.querySelector('#btn-vis-stop');
      const badge     = container.querySelector('#vis-stream-badge');
      const stateEl   = container.querySelector('#vis-stream-state');
      if (streamBtn)   streamBtn.disabled = !selectedCamId;
      if (stopBtn)     stopBtn.disabled = true;
      if (badge)       badge.style.display = 'none';
      if (stateEl)     { stateEl.textContent = 'Detenido'; stateEl.style.color = ''; }
    });

    // Carga inicial
    _loadCameras(container);
    poll(POLL_ID, '/vision/cameras', 30000, (d) => { if (d) _renderCameras(d, container); });

    // Cleanup: detener stream al salir
    container._selectedCamIdRef = (id) => { selectedCamId = id; };
  },

  destroy() {
    stop(POLL_ID);
    // stream cleanup handled by btn-vis-stop or hash change
  },
};

/* ─── Cargar cámaras ─────────────────────────────────────────────────── */

async function _loadCameras(container) {
  try {
    const r = await fetch('/vision/cameras');
    const d = await r.json().catch(() => null);
    _renderCameras(d, container);
  } catch {}
}

function _renderCameras(data, container) {
  const el = container.querySelector('#vis-cameras-list');
  const countEl = container.querySelector('#vis-cam-count');
  if (!el) return;

  const p = data?.data ?? data;
  const cameras = Array.isArray(p) ? p : (p?.cameras || p?.items || []);
  if (countEl) countEl.textContent = cameras.length;

  if (!cameras.length) {
    el.innerHTML = `<div class="empty-state">
      <div class="empty-title">Sin cámaras detectadas</div>
      <div class="empty-sub">Escanea la red o conecta una cámara</div>
    </div>`;
    return;
  }

  el.innerHTML = `<div style="display:grid;grid-template-columns:repeat(auto-fill,minmax(200px,1fr));gap:10px">
    ${cameras.map((cam, i) => {
      const id   = cam.id || cam.cam_id || String(i);
      const name = cam.name || cam.label || `Cámara ${i + 1}`;
      const type = cam.type || cam.source || 'camera';
      const ok   = cam.active !== false && cam.available !== false;
      return `<div class="provider-card ${ok ? 'active' : 'down'}" data-cam-id="${_esc(id)}" style="cursor:pointer">
        <div style="font-size:28px;margin-bottom:6px">📷</div>
        <div class="provider-name">${_esc(name)}</div>
        <div class="provider-role">${_esc(type)}</div>
        ${cam.resolution ? `<div style="font-size:10px;color:var(--text-muted);margin-top:4px">${_esc(cam.resolution)}</div>` : ''}
        <div class="provider-status" style="margin-top:8px">
          <div class="provider-dot ${ok ? 'ok' : 'down'}"></div>
          ${ok ? 'Disponible' : 'No disponible'}
        </div>
        <div style="font-size:10px;color:var(--text-muted);margin-top:8px;text-align:center">Clic para seleccionar →</div>
      </div>`;
    }).join('')}
  </div>`;

  el.querySelectorAll('[data-cam-id]').forEach(card => {
    card.addEventListener('click', () => _selectCamera(card.dataset.camId, card, container));
  });
}

function _selectCamera(camId, cardEl, container) {
  // Deselect all
  container.querySelectorAll('[data-cam-id]').forEach(c => c.style.outline = '');
  // Select current
  cardEl.style.outline = '2px solid var(--accent-primary)';

  const selectedEl = container.querySelector('#vis-selected');
  if (selectedEl) selectedEl.textContent = `Cámara: ${camId}`;

  if (container._selectedCamIdRef) container._selectedCamIdRef(camId);
  // Enable buttons
  const captureBtn = container.querySelector('#btn-vis-capture');
  const streamBtn  = container.querySelector('#btn-vis-stream');
  if (captureBtn) captureBtn.disabled = false;
  if (streamBtn) streamBtn.disabled = false;

  const msg = container.querySelector('#vis-msg');
  if (msg) msg.textContent = `Seleccionada: ${camId}`;
}

/* ─── Escanear cámaras ───────────────────────────────────────────────── */

async function _scanCameras(container) {
  const btn = container.querySelector('#btn-vis-scan');
  const msg = container.querySelector('#vis-msg');
  if (btn) btn.disabled = true;
  if (msg) msg.textContent = 'Escaneando...';
  try {
    const r = await fetch('/vision/scan', { method: 'POST' });
    const d = await r.json().catch(() => ({}));
    if (!r.ok || d.ok === false) throw new Error(d.error || `HTTP ${r.status}`);
    window.AtlasToast?.show('Escaneo completado', 'success');
    if (msg) msg.textContent = '✓ Escaneo completado';
    await _loadCameras(container);
  } catch (e) {
    window.AtlasToast?.show(e.message, 'error');
    if (msg) msg.textContent = `Error: ${e.message}`;
  } finally {
    if (btn) btn.disabled = false;
    setTimeout(() => { const m = container.querySelector('#vis-msg'); if (m) m.textContent = ''; }, 4000);
  }
}

/* ─── Captura de snapshot ────────────────────────────────────────────── */

async function _captureSnapshot(camId, container, silent = false) {
  const vp  = container.querySelector('#vis-viewport');
  const msg = container.querySelector('#vis-msg');
  const ts  = container.querySelector('#vis-last-ts');
  if (!silent && msg) msg.textContent = 'Capturando...';

  try {
    const r = await fetch(`/vision/capture/${encodeURIComponent(camId)}`);
    const d = await r.json().catch(() => ({}));
    if (!r.ok || d.ok === false) throw new Error(d.error || `HTTP ${r.status}`);

    const img = d?.data?.image || d?.data?.frame || d?.image || d?.frame || '';
    const raw = d?.data ?? d ?? {};

    if (img && vp) {
      const src = img.startsWith('data:') ? img : `data:image/jpeg;base64,${img}`;
      vp.innerHTML = `<img src="${src}" style="max-width:100%;max-height:480px;object-fit:contain;display:block;margin:auto">`;
    }

    if (ts) ts.textContent = new Date().toLocaleTimeString();
    if (!silent && msg) msg.textContent = '';

    // Show cam info
    const infoEl = container.querySelector('#vis-cam-info');
    const rawEl  = container.querySelector('#vis-cam-raw');
    if (infoEl) infoEl.style.display = '';
    if (rawEl) rawEl.textContent = JSON.stringify(raw, null, 2);
  } catch (e) {
    if (!silent) {
      window.AtlasToast?.show(e.message, 'error');
      if (msg) msg.textContent = `Error: ${e.message}`;
    }
    if (vp) vp.innerHTML = `<div style="text-align:center;color:var(--accent-red);padding:20px">
      <div style="font-size:24px">⚠</div>
      <div style="margin-top:8px;font-size:13px">Error: ${_esc(e.message)}</div>
    </div>`;
  }
}

window.AtlasModuleVision = { id: 'vision' };
