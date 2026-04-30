import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'live-diagnostic-module';

function _esc(s) {
  const d = document.createElement('span');
  d.textContent = s ?? '';
  return d.innerHTML;
}

function _txt(container, sel, val) {
  const el = container.querySelector(sel);
  if (el) el.textContent = val ?? '--';
}

export default {
  id: 'live-diagnostic',
  label: 'Diagnostico Live',
  icon: 'activity',
  category: 'monitoring',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" onclick="location.hash='/'">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Dashboard
          </button>
          <h2>Diagnostico Live</h2>
          <div style="margin-left:auto"><span class="live-badge">LIVE</span></div>
        </div>
        <div class="module-body">
          <div class="stat-row">
            <div class="stat-card"><div class="stat-card-label">PUSH</div><div class="stat-card-value" id="ld-push">--</div><div class="stat-card-sub" id="ld-pid"></div></div>
            <div class="stat-card"><div class="stat-card-label">AutoRevive</div><div class="stat-card-value" id="ld-ar">--</div><div class="stat-card-sub" id="ld-ar-sub"></div></div>
            <div class="stat-card"><div class="stat-card-label">Tools Job</div><div class="stat-card-value" id="ld-job">--</div><div class="stat-card-sub" id="ld-job-sub"></div></div>
          </div>
          <div class="section-title">Raw</div>
          <pre id="ld-raw" class="codebox" style="max-height:360px;overflow:auto"></pre>
        </div>
      </div>
    `;

    const renderData = (payload) => {
      const d = payload?.data || payload || {};
      const push = d.push || {};
      const ar = d.autorevive || {};
      const job = d.tools?.last_job || null;

      _txt(container, '#ld-push', push.alive ? 'ONLINE' : 'OFFLINE');
      _txt(container, '#ld-pid', `pid: ${push.pid ?? '--'} | score: ${push.score ?? '--'}`);
      _txt(container, '#ld-ar', ar.last_ok_at ? 'ACTIVO' : 'SIN SEÑAL');
      _txt(container, '#ld-ar-sub', `restarts: ${ar.restarts ?? '--'} | fail_streak: ${ar.fail_streak ?? '--'}`);
      _txt(container, '#ld-job', job ? String(job.status || 'unknown').toUpperCase() : 'N/A');
      _txt(container, '#ld-job-sub', job ? `${job.tool || 'all'} | ${job.job_id || '--'}` : 'sin jobs');

      const rawEl = container.querySelector('#ld-raw');
      if (rawEl) rawEl.innerHTML = _esc(JSON.stringify(payload, null, 2));
    };

    const fetchOnce = async () => {
      try {
        const r = await fetch('/api/diagnostic/live');
        const p = await r.json();
        renderData(p);
      } catch (e) {
        const rawEl = container.querySelector('#ld-raw');
        if (rawEl) rawEl.textContent = `error: ${String(e)}`;
      }
    };

    fetchOnce();
    poll(POLL_ID, '/api/diagnostic/live', 5000, (data) => {
      if (data) renderData(data);
    });
  },

  destroy() {
    stop(POLL_ID);
  },
};

