/**
 * ATLAS v4 — Communications Module
 * WhatsApp/Telegram comms status, QR and test.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'comms-module';

export default {
  id: 'comms',
  label: 'Communications',
  icon: 'message-circle',
  category: 'configuration',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Home
          </button>
          <h2>Communications</h2>
        </div>
        <div class="module-body">
          <div class="config-section">
            <div class="config-section-title">Status</div>
            <pre class="codebox" id="comms-status">Loading...</pre>
          </div>

          <div class="config-section">
            <div class="config-section-title">Actions</div>
            <div style="display:flex;gap:10px;flex-wrap:wrap">
              <button class="chip-btn" id="btn-qr">Get WhatsApp QR</button>
              <button class="chip-btn" id="btn-restart">Restart WAHA</button>
              <button class="chip-btn" id="btn-test">Run comms test</button>
            </div>
            <div style="margin-top:10px" id="comms-action"></div>
          </div>
        </div>
      </div>
    `;

    container.querySelector('#btn-qr')?.addEventListener('click', () => _qr(container));
    container.querySelector('#btn-restart')?.addEventListener('click', () => _restart(container));
    container.querySelector('#btn-test')?.addEventListener('click', () => _test(container));

    _refresh(container);
    poll(POLL_ID, '/ans/comms/status', 6000, (data) => { if (data) _renderStatus(container, data); });
  },

  destroy() { stop(POLL_ID); },
};

async function _refresh(container) {
  try {
    const res = await fetch('/ans/comms/status');
    const data = await res.json();
    _renderStatus(container, data);
  } catch (e) {
    const el = container.querySelector('#comms-status');
    if (el) el.textContent = `Error: ${e.message}`;
  }
}

function _renderStatus(container, data) {
  const el = container.querySelector('#comms-status');
  if (!el) return;
  el.textContent = JSON.stringify(data, null, 2);
}

function _setAction(container, html) {
  const el = container.querySelector('#comms-action');
  if (el) el.innerHTML = html || '';
}

async function _qr(container) {
  _setAction(container, '<div class="spinner"></div> fetching QR...');
  try {
    const res = await fetch('/ans/comms/waha/qr');
    const data = await res.json().catch(() => ({}));
    if (!res.ok || data.ok === false) throw new Error(data.error || `HTTP ${res.status}`);
    // Try multiple known shapes: {data:{qr}} or {qr} or {data:{url}}
    const qr = (data.data && (data.data.qr || data.data.url)) || data.qr || data.url || '';
    if (!qr) {
      _setAction(container, `<pre class="codebox">${_esc(JSON.stringify(data, null, 2))}</pre>`);
      return;
    }
    _setAction(container, `<div style="display:flex;gap:16px;align-items:flex-start;flex-wrap:wrap">
      <img alt="WAHA QR" src="${qr}" style="width:220px;height:220px;border-radius:12px;border:1px solid var(--border);background:white">
      <pre class="codebox" style="flex:1;min-width:260px">${_esc(qr)}</pre>
    </div>`);
  } catch (e) {
    _setAction(container, `<pre class="codebox">Error: ${_esc(e.message)}</pre>`);
  }
}

async function _restart(container) {
  _setAction(container, '<div class="spinner"></div> restarting...');
  try {
    const res = await fetch('/ans/comms/waha/restart', { method: 'POST' });
    const data = await res.json().catch(() => ({}));
    if (!res.ok || data.ok === false) throw new Error(data.error || `HTTP ${res.status}`);
    window.AtlasToast?.show('WAHA restart: OK', 'success');
    _setAction(container, `<pre class="codebox">${_esc(JSON.stringify(data, null, 2))}</pre>`);
  } catch (e) {
    window.AtlasToast?.show(`WAHA restart failed: ${e.message}`, 'error');
    _setAction(container, `<pre class="codebox">Error: ${_esc(e.message)}</pre>`);
  }
}

async function _test(container) {
  _setAction(container, '<div class="spinner"></div> testing...');
  try {
    const res = await fetch('/ans/comms/test', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({}) });
    const data = await res.json().catch(() => ({}));
    if (!res.ok || data.ok === false) throw new Error(data.error || `HTTP ${res.status}`);
    window.AtlasToast?.show('Comms test: OK', 'success');
    _setAction(container, `<pre class="codebox">${_esc(JSON.stringify(data, null, 2))}</pre>`);
  } catch (e) {
    window.AtlasToast?.show(`Comms test failed: ${e.message}`, 'error');
    _setAction(container, `<pre class="codebox">Error: ${_esc(e.message)}</pre>`);
  }
}

function _esc(s) { const d = document.createElement('span'); d.textContent = s || ''; return d.innerHTML; }

window.AtlasModuleComms = { id: 'comms' };
