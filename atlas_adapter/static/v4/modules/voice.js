/**
 * ATLAS v4.2 — Voice Module
 * Panel de control TTS/voz con estado, inicio/parada y speak.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'voice-module';

function _esc(s) { const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML; }

export default {
  id: 'voice',
  label: 'Voz',
  icon: 'mic',
  category: 'configuration',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Inicio
          </button>
          <h2>Control de Voz</h2>
        </div>
        <div class="module-body">

          <!-- Status row -->
          <div class="stat-row" style="margin-bottom:16px">
            <div class="stat-card hero">
              <div class="stat-card-label">Estado del Motor TTS</div>
              <div class="stat-card-value" id="v-state">--</div>
              <div class="stat-card-sub" id="v-engine"></div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Cola de habla</div>
              <div class="stat-card-value accent" id="v-queue">--</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Idioma</div>
              <div class="stat-card-value" id="v-lang" style="font-size:18px">--</div>
            </div>
          </div>

          <!-- Actions -->
          <div class="section-title">Control del Motor</div>
          <div class="action-bar">
            <button class="action-btn success" id="btn-v-start">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polygon points="5 3 19 12 5 21 5 3"/></svg>
              Iniciar
            </button>
            <button class="action-btn danger" id="btn-v-stop">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="3" y="3" width="18" height="18" rx="2"/></svg>
              Detener
            </button>
          </div>

          <!-- Speak -->
          <div class="section-title" style="margin-top:20px">Hablar</div>
          <div style="display:flex;gap:10px;align-items:flex-end;flex-wrap:wrap">
            <div style="flex:1;min-width:260px">
              <input id="voice-text" class="feedback-box" style="min-height:0;height:42px;resize:none"
                placeholder="Escribe algo para que ATLAS lo diga...">
            </div>
            <button class="action-btn primary" id="btn-v-speak">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polygon points="11 5 6 9 2 9 2 15 6 15 11 19 11 5"/><path d="M19.07 4.93a10 10 0 010 14.14M15.54 8.46a5 5 0 010 7.07"/></svg>
              Hablar
            </button>
          </div>
          <div id="voice-fb" style="font-size:11px;color:var(--text-muted);margin-top:8px"></div>

          <!-- Raw status -->
          <div class="section-title" style="margin-top:20px">Diagnóstico</div>
          <pre class="codebox" id="voice-raw" style="max-height:180px;overflow:auto">Cargando...</pre>
        </div>
      </div>
    `;

    container.querySelector('#btn-v-start')?.addEventListener('click', () => _post('/ans/voice/start', 'Iniciar', container));
    container.querySelector('#btn-v-stop')?.addEventListener('click',  () => _post('/ans/voice/stop',  'Detener', container));
    container.querySelector('#btn-v-speak')?.addEventListener('click', () => {
      const t = (container.querySelector('#voice-text')?.value || '').trim();
      if (!t) return;
      _speak(t, container);
    });

    _refresh(container);
    poll(POLL_ID, '/ans/voice/status', 6000, (data) => { if (data) _renderStatus(container, data); });
  },

  destroy() { stop(POLL_ID); },
};

async function _refresh(container) {
  try {
    const r = await fetch('/ans/voice/status');
    const data = await r.json();
    _renderStatus(container, data);
  } catch (e) {
    const el = container.querySelector('#voice-raw');
    if (el) el.textContent = `Error: ${e.message}`;
  }
}

function _renderStatus(container, data) {
  const p = data?.data ?? data;
  const running = p?.running === true || p?.status === 'running' || p?.active === true;
  const txt = (id, v) => { const e = container.querySelector(id); if (e) e.textContent = v; };
  txt('#v-state', running ? 'Activo' : 'Inactivo');
  const stEl = container.querySelector('#v-state');
  if (stEl) stEl.style.color = running ? 'var(--accent-green)' : 'var(--text-muted)';
  txt('#v-engine', p?.engine || p?.backend || '');
  txt('#v-queue', p?.queue_size ?? p?.queue ?? '--');
  txt('#v-lang', p?.lang || p?.language || '--');
  const raw = container.querySelector('#voice-raw');
  if (raw) raw.textContent = JSON.stringify(p, null, 2);
}

async function _post(url, label, container) {
  const fb = container.querySelector('#voice-fb');
  if (fb) fb.textContent = `${label}...`;
  try {
    const r = await fetch(url, { method: 'POST' });
    const data = await r.json().catch(() => ({}));
    if (!r.ok || data.ok === false) throw new Error(data.error || `HTTP ${r.status}`);
    window.AtlasToast?.show(`${label}: OK`, 'success');
    if (fb) fb.textContent = `${label}: OK`;
    setTimeout(() => _refresh(container), 1000);
  } catch (e) {
    window.AtlasToast?.show(e.message, 'error');
    if (fb) fb.textContent = `Error: ${e.message}`;
  }
}

async function _speak(text, container) {
  const fb = container.querySelector('#voice-fb');
  if (fb) fb.textContent = 'Enviando...';
  try {
    const url = '/ans/voice/speak?text=' + encodeURIComponent(text);
    const r = await fetch(url, { method: 'POST' });
    const data = await r.json().catch(() => ({}));
    if (!r.ok || data.ok === false) throw new Error(data.error || `HTTP ${r.status}`);
    window.AtlasToast?.show('Habla en cola', 'success');
    if (fb) fb.textContent = 'Habla encolada correctamente';
    const inp = container.querySelector('#voice-text');
    if (inp) inp.value = '';
  } catch (e) {
    window.AtlasToast?.show(e.message, 'error');
    if (fb) fb.textContent = `Error: ${e.message}`;
  }
}

window.AtlasModuleVoice = { id: 'voice' };
