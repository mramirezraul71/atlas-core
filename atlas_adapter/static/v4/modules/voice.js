/**
 * ATLAS v4 — Voice Module
 * Control TTS/voice assistant endpoints.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'voice-module';

export default {
  id: 'voice',
  label: 'Voice',
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
          <h2>Voice</h2>
        </div>
        <div class="module-body">
          <div class="config-section">
            <div class="config-section-title">Status</div>
            <pre class="codebox" id="voice-status">Loading...</pre>
          </div>

          <div class="config-section">
            <div class="config-section-title">Actions</div>
            <div style="display:flex;gap:10px;flex-wrap:wrap">
              <button class="chip-btn" id="btn-voice-start">Start</button>
              <button class="chip-btn" id="btn-voice-stop">Stop</button>
            </div>
            <div style="margin-top:12px;display:flex;gap:10px;flex-wrap:wrap;align-items:center">
              <input id="voice-text" class="search-bar" style="max-width:520px;margin:0" placeholder="Texto para hablar...">
              <button class="chip-btn" id="btn-voice-speak">Speak</button>
            </div>
            <div style="margin-top:10px" id="voice-action"></div>
          </div>
        </div>
      </div>
    `;

    container.querySelector('#btn-voice-start')?.addEventListener('click', () => _post('/ans/voice/start'));
    container.querySelector('#btn-voice-stop')?.addEventListener('click', () => _post('/ans/voice/stop'));
    container.querySelector('#btn-voice-speak')?.addEventListener('click', () => {
      const t = (container.querySelector('#voice-text')?.value || '').trim();
      if (!t) return;
      _speak(t);
    });

    _refresh(container);
    poll(POLL_ID, '/ans/voice/status', 6000, (data) => { if (data) _renderStatus(container, data); });
  },

  destroy() { stop(POLL_ID); },
};

async function _refresh(container) {
  try {
    const res = await fetch('/ans/voice/status');
    const data = await res.json();
    _renderStatus(container, data);
  } catch (e) {
    const el = container.querySelector('#voice-status');
    if (el) el.textContent = `Error: ${e.message}`;
  }
}

function _renderStatus(container, data) {
  const el = container.querySelector('#voice-status');
  if (!el) return;
  el.textContent = JSON.stringify(data, null, 2);
}

async function _post(url) {
  try {
    const res = await fetch(url, { method: 'POST' });
    const data = await res.json().catch(() => ({}));
    if (!res.ok || data.ok === false) throw new Error(data.error || `HTTP ${res.status}`);
    window.AtlasToast?.show('OK', 'success');
  } catch (e) {
    window.AtlasToast?.show(`Error: ${e.message}`, 'error');
  }
}

async function _speak(text) {
  try {
    const url = '/ans/voice/speak?text=' + encodeURIComponent(text);
    const res = await fetch(url, { method: 'POST' });
    const data = await res.json().catch(() => ({}));
    if (!res.ok || data.ok === false) throw new Error(data.error || `HTTP ${res.status}`);
    window.AtlasToast?.show('Speak queued', 'success');
  } catch (e) {
    window.AtlasToast?.show(`Speak error: ${e.message}`, 'error');
  }
}

window.AtlasModuleVoice = { id: 'voice' };

