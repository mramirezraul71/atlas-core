/**
 * ATLAS v4.3 - Bety Eventos
 * Flujo simple para captura diaria en Excel + respaldo server-side.
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'bety-eventos-status';

function _esc(s) {
  const span = document.createElement('span');
  span.textContent = s ?? '';
  return span.innerHTML;
}

function _fmtNum(v) {
  const n = Number(v || 0);
  if (!Number.isFinite(n)) return '0';
  return n.toLocaleString('es-ES', { maximumFractionDigits: 2 });
}

function _fmtDate(ts) {
  if (!ts) return '--';
  try {
    return new Date(ts).toLocaleString('es-ES');
  } catch {
    return String(ts);
  }
}

function _setText(root, sel, value) {
  const el = root.querySelector(sel);
  if (el) el.textContent = value;
}

async function _fetchJson(url, opts) {
  const r = await fetch(url, opts);
  const j = await r.json().catch(() => ({}));
  if (!r.ok || j?.ok === false) {
    throw new Error(j?.error || j?.detail || `HTTP ${r.status}`);
  }
  return j?.data ?? j;
}

function _renderUploads(root, items) {
  const tbody = root.querySelector('#bety-uploads-body');
  if (!tbody) return;
  const rows = Array.isArray(items) ? [...items].reverse() : [];
  if (!rows.length) {
    tbody.innerHTML = `
      <tr>
        <td colspan="5" style="text-align:center;color:var(--text-muted);padding:14px">
          Aun no hay cargas diarias.
        </td>
      </tr>
    `;
    return;
  }
  tbody.innerHTML = rows.map((it) => {
    const t = it?.totals || {};
    const ent = t?.entrada_almacen?.importe_total ?? 0;
    const ven = t?.ventas?.importe_total ?? 0;
    return `
      <tr>
        <td>${_esc(_fmtDate(it?.ts))}</td>
        <td style="max-width:240px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap">${_esc(it?.file_name || '--')}</td>
        <td>$${_esc(_fmtNum(ent))}</td>
        <td>$${_esc(_fmtNum(ven))}</td>
        <td style="font-family:monospace;font-size:11px;max-width:150px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap">${_esc(it?.sha256 || '--')}</td>
      </tr>
    `;
  }).join('');
}

async function _loadStatus(root) {
  const status = await _fetchJson('/api/panaderia/bety/status');
  const summary = status?.latest_summary || {};
  const totals = summary?.totals || {};
  const ventas = totals?.ventas || {};
  const entrada = totals?.entrada_almacen || {};

  _setText(root, '#bety-last-upload', _fmtDate(summary?.file?.uploaded_at));
  _setText(root, '#bety-ventas-total', `$${_fmtNum(ventas?.importe_total)}`);
  _setText(root, '#bety-entrada-total', `$${_fmtNum(entrada?.importe_total)}`);
  _setText(root, '#bety-count-uploads', String((status?.uploads || []).length));
  _setText(root, '#bety-storage-root', status?.storage_root || '--');
  _renderUploads(root, status?.uploads || []);
}

export default {
  id: 'bety-eventos',
  label: 'Bety Eventos',
  icon: 'file-spreadsheet',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" id="bety-back-home">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <path d="M19 12H5M12 19l-7-7 7-7"></path>
            </svg>
            Inicio
          </button>
          <h2>Bety Eventos (Panaderia)</h2>
          <div style="margin-left:auto"><span class="live-badge">LIVE</span></div>
        </div>

        <div class="module-body" style="display:flex;flex-direction:column;gap:14px">
          <div class="stat-row">
            <div class="stat-card">
              <div class="stat-card-label">Ultima carga</div>
              <div class="stat-card-value accent" id="bety-last-upload">--</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Entrada almacen</div>
              <div class="stat-card-value" id="bety-entrada-total">$0</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Ventas acumuladas</div>
              <div class="stat-card-value" id="bety-ventas-total">$0</div>
            </div>
            <div class="stat-card">
              <div class="stat-card-label">Versiones guardadas</div>
              <div class="stat-card-value" id="bety-count-uploads">0</div>
            </div>
          </div>

          <div class="section-card" style="padding:14px;border:1px solid var(--border-subtle);border-radius:12px;background:var(--surface-1)">
            <div class="section-title">1) Descargar plantilla de Bety</div>
            <div style="font-size:12px;color:var(--text-muted);margin:4px 0 10px">
              Archivo separado de la app, para llenar cada dia desde laptop.
            </div>
            <div style="display:flex;gap:8px;flex-wrap:wrap">
              <button class="action-btn primary" id="bety-download-template">Descargar plantilla</button>
              <a class="action-btn" href="/api/panaderia/bety/download/latest" target="_blank" rel="noopener">Descargar ultimo archivo subido</a>
            </div>
          </div>

          <div class="section-card" style="padding:14px;border:1px solid var(--border-subtle);border-radius:12px;background:var(--surface-1)">
            <div class="section-title">2) Subir archivo diario</div>
            <div style="font-size:12px;color:var(--text-muted);margin:4px 0 10px">
              Al subir, ATLAS guarda respaldo automatico y consolida informacion primaria.
            </div>
            <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center">
              <input id="bety-file-input" type="file" accept=".xlsx" />
              <button class="action-btn success" id="bety-upload-btn">Subir a ATLAS</button>
            </div>
            <div id="bety-upload-msg" style="margin-top:8px;font-size:12px;color:var(--text-muted)">Esperando archivo...</div>
          </div>

          <div class="section-card" style="padding:14px;border:1px solid var(--border-subtle);border-radius:12px;background:var(--surface-1)">
            <div class="section-title">3) Respaldo y trazabilidad</div>
            <div style="font-size:12px;color:var(--text-muted);margin:4px 0 10px">
              Cada carga queda versionada con hash y copia de seguridad.
            </div>
            <div style="font-size:12px;color:var(--text-muted)">
              Carpeta de almacenamiento: <code id="bety-storage-root">--</code>
            </div>
          </div>

          <div class="section-title">Historial de cargas</div>
          <div style="overflow:auto;border:1px solid var(--border-subtle);border-radius:12px">
            <table style="width:100%;border-collapse:collapse;font-size:12px">
              <thead>
                <tr style="background:var(--surface-2)">
                  <th style="text-align:left;padding:10px">Fecha</th>
                  <th style="text-align:left;padding:10px">Archivo</th>
                  <th style="text-align:left;padding:10px">Entrada</th>
                  <th style="text-align:left;padding:10px">Ventas</th>
                  <th style="text-align:left;padding:10px">Hash</th>
                </tr>
              </thead>
              <tbody id="bety-uploads-body"></tbody>
            </table>
          </div>
        </div>
      </div>
    `;

    container.querySelector('#bety-back-home')?.addEventListener('click', () => {
      window.location.hash = '/';
    });

    container.querySelector('#bety-download-template')?.addEventListener('click', async () => {
      try {
        const data = await _fetchJson('/api/panaderia/bety/template');
        window.open(data.download_path || data.download_url, '_blank', 'noopener');
        window.AtlasToast?.show('Plantilla lista para descarga.', 'success');
      } catch (err) {
        window.AtlasToast?.show(`No se pudo descargar: ${err.message}`, 'error');
      }
    });

    container.querySelector('#bety-upload-btn')?.addEventListener('click', async () => {
      const msg = container.querySelector('#bety-upload-msg');
      const input = container.querySelector('#bety-file-input');
      const file = input?.files?.[0];
      if (!file) {
        if (msg) msg.textContent = 'Selecciona primero un archivo .xlsx';
        return;
      }
      const fd = new FormData();
      fd.append('file', file);
      try {
        if (msg) msg.textContent = 'Subiendo y consolidando en ATLAS...';
        await _fetchJson('/api/panaderia/bety/upload', { method: 'POST', body: fd });
        if (msg) msg.textContent = 'Carga OK. Respaldo generado correctamente.';
        window.AtlasToast?.show('Archivo cargado y respaldado.', 'success');
        await _loadStatus(container);
      } catch (err) {
        if (msg) msg.textContent = `Error: ${err.message}`;
        window.AtlasToast?.show(`Error de carga: ${err.message}`, 'error');
      }
    });

    _loadStatus(container).catch((e) => {
      window.AtlasToast?.show(`Bety Eventos sin datos iniciales: ${e.message}`, 'warn');
    });

    poll(POLL_ID, '/api/panaderia/bety/status', 30000, (payload) => {
      try {
        const data = payload?.data || payload || {};
        const synthetic = { uploads: data.uploads || [], latest_summary: data.latest_summary || {}, storage_root: data.storage_root || '--' };
        const summary = synthetic.latest_summary;
        const totals = summary.totals || {};
        _setText(container, '#bety-last-upload', _fmtDate(summary?.file?.uploaded_at));
        _setText(container, '#bety-ventas-total', `$${_fmtNum(totals?.ventas?.importe_total)}`);
        _setText(container, '#bety-entrada-total', `$${_fmtNum(totals?.entrada_almacen?.importe_total)}`);
        _setText(container, '#bety-count-uploads', String((synthetic.uploads || []).length));
        _setText(container, '#bety-storage-root', synthetic.storage_root || '--');
        _renderUploads(container, synthetic.uploads || []);
      } catch {
        // best effort polling
      }
    });
  },

  destroy() {
    stop(POLL_ID);
  },
};
