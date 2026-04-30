/**
 * ATLAS v4.2 — Tutorías Module
 * Sistema completo de especialistas, visitas, informes y recomendaciones.
 * Rutas: /tutorias/* (router incluido en atlas_http_api.py)
 */
import { poll, stop } from '../lib/polling.js';

const POLL_ID = 'tutorias-module';

function _esc(s) { const d = document.createElement('span'); d.textContent = s ?? ''; return d.innerHTML; }
function _time(ts) { if (!ts) return '--'; try { return new Date(ts).toLocaleString('es'); } catch { return String(ts); } }

export default {
  id: 'tutorias',
  label: 'Tutorías',
  icon: 'book-open',
  category: 'monitoring',

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" id="tut-back">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Dashboard
          </button>
          <h2>Tutorías y Especialistas</h2>
          <div style="margin-left:auto"><span class="live-badge">LIVE</span></div>
        </div>

        <div class="module-body">

          <!-- KPIs -->
          <div class="stat-row" style="margin-bottom:20px" id="tut-kpis">
            <div style="padding:12px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
          </div>

          <!-- Tabs -->
          <div class="action-bar" style="padding-top:0;margin-bottom:0;border-bottom:1px solid var(--border-subtle);gap:0">
            <button class="action-btn tut-tab active" data-tab="especialistas" style="border-radius:8px 8px 0 0;border-bottom:none">Especialistas</button>
            <button class="action-btn tut-tab" data-tab="visitas" style="border-radius:8px 8px 0 0;border-bottom:none">Visitas</button>
            <button class="action-btn tut-tab" data-tab="recomendaciones" style="border-radius:8px 8px 0 0;border-bottom:none">Recomendaciones</button>
            <button class="action-btn tut-tab" data-tab="registrar" style="border-radius:8px 8px 0 0;border-bottom:none">+ Registrar</button>
          </div>

          <!-- Tab: Especialistas -->
          <div id="tab-especialistas" class="tut-panel" style="padding-top:16px">
            <div class="action-bar" style="padding-top:0;margin-bottom:12px">
              <button class="action-btn" id="btn-tut-refresh-esp">
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="23 4 23 10 17 10"/><path d="M20.49 15a9 9 0 11-2.12-9.36L23 10"/></svg>
                Actualizar
              </button>
            </div>
            <div id="tut-especialistas-list">
              <div style="padding:20px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
            </div>
          </div>

          <!-- Tab: Visitas -->
          <div id="tab-visitas" class="tut-panel" style="display:none;padding-top:16px">
            <div class="action-bar" style="padding-top:0;margin-bottom:12px">
              <button class="action-btn" id="btn-tut-refresh-vis">
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="23 4 23 10 17 10"/><path d="M20.49 15a9 9 0 11-2.12-9.36L23 10"/></svg>
                Actualizar
              </button>
            </div>
            <div id="tut-visitas-list">
              <div style="padding:20px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
            </div>
          </div>

          <!-- Tab: Recomendaciones -->
          <div id="tab-recomendaciones" class="tut-panel" style="display:none;padding-top:16px">
            <div class="action-bar" style="padding-top:0;margin-bottom:12px">
              <button class="action-btn" id="btn-tut-refresh-rec">
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="23 4 23 10 17 10"/><path d="M20.49 15a9 9 0 11-2.12-9.36L23 10"/></svg>
                Actualizar
              </button>
              <select id="tut-rec-filtro" style="padding:6px 10px;background:var(--surface-1);border:1px solid var(--border);border-radius:6px;color:var(--text-primary);font-size:12px">
                <option value="">Todas</option>
                <option value="PENDIENTE">Pendientes</option>
                <option value="EN_PROCESO">En proceso</option>
                <option value="IMPLEMENTADA">Implementadas</option>
              </select>
            </div>
            <div id="tut-recomendaciones-list">
              <div style="padding:20px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
            </div>
          </div>

          <!-- Tab: Registrar nuevo especialista -->
          <div id="tab-registrar" class="tut-panel" style="display:none;padding-top:16px">
            <div class="section-title">Registrar Nuevo Especialista</div>
            <div style="display:grid;grid-template-columns:repeat(auto-fill,minmax(220px,1fr));gap:12px;margin-bottom:16px">
              <div>
                <label style="font-size:11px;color:var(--text-muted);display:block;margin-bottom:4px">Nombre</label>
                <input id="esp-nombre" type="text" placeholder="Nombre completo" class="feedback-box" style="min-height:0;height:40px">
              </div>
              <div>
                <label style="font-size:11px;color:var(--text-muted);display:block;margin-bottom:4px">Rol</label>
                <input id="esp-rol" type="text" placeholder="Médico, Profesor, Entrenador..." class="feedback-box" style="min-height:0;height:40px">
              </div>
              <div>
                <label style="font-size:11px;color:var(--text-muted);display:block;margin-bottom:4px">Especialidad</label>
                <input id="esp-especialidad" type="text" placeholder="Área de especialización" class="feedback-box" style="min-height:0;height:40px">
              </div>
            </div>
            <div class="action-bar" style="padding-top:0">
              <button class="action-btn primary" id="btn-tut-registrar">
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><line x1="12" y1="5" x2="12" y2="19"/><line x1="5" y1="12" x2="19" y2="12"/></svg>
                Registrar Especialista
              </button>
              <span id="tut-reg-msg" style="font-size:11px;color:var(--text-muted);align-self:center"></span>
            </div>

            <div class="section-title" style="margin-top:24px">Iniciar Nueva Visita</div>
            <div style="display:grid;grid-template-columns:repeat(auto-fill,minmax(220px,1fr));gap:12px;margin-bottom:16px">
              <div>
                <label style="font-size:11px;color:var(--text-muted);display:block;margin-bottom:4px">Especialista</label>
                <select id="vis-especialista" style="width:100%;padding:7px 10px;background:var(--surface-1);border:1px solid var(--border);border-radius:6px;color:var(--text-primary);font-size:13px">
                  <option value="">Seleccionar especialista...</option>
                </select>
              </div>
              <div>
                <label style="font-size:11px;color:var(--text-muted);display:block;margin-bottom:4px">Propósito</label>
                <input id="vis-proposito" type="text" placeholder="Objetivo de la visita" class="feedback-box" style="min-height:0;height:40px">
              </div>
            </div>
            <div class="action-bar" style="padding-top:0">
              <button class="action-btn success" id="btn-tut-iniciar">
                <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polygon points="5 3 19 12 5 21 5 3"/></svg>
                Iniciar Visita
              </button>
              <span id="tut-vis-msg" style="font-size:11px;color:var(--text-muted);align-self:center"></span>
            </div>
          </div>

        </div>
      </div>
    `;

    // Navegación
    container.querySelector('#tut-back')?.addEventListener('click', () => { location.hash = '/'; });

    // Tabs
    container.querySelectorAll('.tut-tab').forEach(btn => {
      btn.addEventListener('click', () => {
        container.querySelectorAll('.tut-tab').forEach(b => b.classList.remove('active', 'primary'));
        container.querySelectorAll('.tut-panel').forEach(p => p.style.display = 'none');
        btn.classList.add('active');
        const tab = container.querySelector(`#tab-${btn.dataset.tab}`);
        if (tab) tab.style.display = '';
        if (btn.dataset.tab === 'visitas') _loadVisitas(container);
        if (btn.dataset.tab === 'recomendaciones') _loadRecomendaciones(container);
        if (btn.dataset.tab === 'registrar') _loadEspecialistasSelect(container);
      });
    });

    // Botones
    container.querySelector('#btn-tut-refresh-esp')?.addEventListener('click',  () => _loadEspecialistas(container));
    container.querySelector('#btn-tut-refresh-vis')?.addEventListener('click',  () => _loadVisitas(container));
    container.querySelector('#btn-tut-refresh-rec')?.addEventListener('click',  () => _loadRecomendaciones(container));
    container.querySelector('#tut-rec-filtro')?.addEventListener('change',      () => _loadRecomendaciones(container));
    container.querySelector('#btn-tut-registrar')?.addEventListener('click',    () => _registrarEspecialista(container));
    container.querySelector('#btn-tut-iniciar')?.addEventListener('click',      () => _iniciarVisita(container));

    // Carga inicial
    _loadStats(container);
    _loadEspecialistas(container);

    poll(POLL_ID, '/tutorias/estadisticas', 30000, (d) => { if (d) _renderStats(container, d); });
  },

  destroy() { stop(POLL_ID); },
};

/* ─── Stats / KPIs ───────────────────────────────────────────────────── */

async function _loadStats(container) {
  try {
    const r = await fetch('/tutorias/estadisticas');
    const d = await r.json().catch(() => null);
    if (d) _renderStats(container, d);
  } catch {}
}

function _renderStats(container, data) {
  const p = data?.data ?? data ?? {};
  const row = container.querySelector('#tut-kpis');
  if (!row) return;
  row.innerHTML = `
    <div class="stat-card hero">
      <div class="stat-card-label">Especialistas</div>
      <div class="stat-card-value accent">${_esc(String(p.total_especialistas ?? p.especialistas ?? '--'))}</div>
    </div>
    <div class="stat-card">
      <div class="stat-card-label">Visitas realizadas</div>
      <div class="stat-card-value">${_esc(String(p.total_visitas ?? p.visitas ?? '--'))}</div>
    </div>
    <div class="stat-card">
      <div class="stat-card-label">Informes firmados</div>
      <div class="stat-card-value green">${_esc(String(p.total_informes ?? p.informes ?? '--'))}</div>
    </div>
    <div class="stat-card">
      <div class="stat-card-label">Puntuación media</div>
      <div class="stat-card-value" style="font-size:20px">${p.puntuacion_promedio ? `${Number(p.puntuacion_promedio).toFixed(1)}/5.0` : '--'}</div>
    </div>
    <div class="stat-card">
      <div class="stat-card-label">Recomendaciones pendientes</div>
      <div class="stat-card-value orange">${_esc(String(p.recomendaciones_pendientes ?? '--'))}</div>
    </div>
  `;
}

/* ─── Especialistas ──────────────────────────────────────────────────── */

async function _loadEspecialistas(container) {
  const el = container.querySelector('#tut-especialistas-list');
  if (!el) return;
  try {
    const r = await fetch('/tutorias/especialistas');
    const d = await r.json().catch(() => null);
    const p = d?.data ?? d;
    const items = Array.isArray(p) ? p : (p?.items || p?.especialistas || []);

    if (!items.length) {
      el.innerHTML = `<div class="empty-state">
        <div class="empty-title">Sin especialistas registrados</div>
        <div class="empty-sub">Ve a la pestaña "+ Registrar" para añadir el primero</div>
      </div>`;
      return;
    }

    el.innerHTML = `<div style="display:grid;grid-template-columns:repeat(auto-fill,minmax(240px,1fr));gap:10px">
      ${items.map(e => {
        const puntuacion = e.puntuacion_promedio ? `${Number(e.puntuacion_promedio).toFixed(1)}/5` : '--';
        return `<div class="provider-card active" style="cursor:default">
          <div style="font-size:24px;margin-bottom:6px">🎓</div>
          <div class="provider-name">${_esc(e.nombre || e.name || '--')}</div>
          <div class="provider-role">${_esc(e.rol || e.role || '')}</div>
          <div style="font-size:11px;color:var(--text-muted);margin-top:4px">${_esc(e.especialidad || '')}</div>
          <div class="provider-status" style="margin-top:8px;justify-content:space-between">
            <span>⭐ ${puntuacion}</span>
            <span>${e.total_visitas ?? 0} visitas</span>
          </div>
        </div>`;
      }).join('')}
    </div>`;
  } catch (e) {
    if (el) el.innerHTML = `<div class="empty-state"><div class="empty-title" style="color:var(--accent-red)">Error: ${_esc(e.message)}</div></div>`;
  }
}

async function _loadEspecialistasSelect(container) {
  const sel = container.querySelector('#vis-especialista');
  if (!sel || sel.options.length > 1) return; // Already loaded
  try {
    const r = await fetch('/tutorias/especialistas');
    const d = await r.json().catch(() => null);
    const p = d?.data ?? d;
    const items = Array.isArray(p) ? p : (p?.items || p?.especialistas || []);
    items.forEach(e => {
      const opt = document.createElement('option');
      opt.value = e.id || e.especialista_id || '';
      opt.textContent = `${e.nombre || '--'} (${e.rol || ''})`;
      sel.appendChild(opt);
    });
  } catch {}
}

/* ─── Visitas ────────────────────────────────────────────────────────── */

async function _loadVisitas(container) {
  const el = container.querySelector('#tut-visitas-list');
  if (!el) return;
  try {
    const r = await fetch('/tutorias/visitas?limite=20');
    const d = await r.json().catch(() => null);
    const p = d?.data ?? d;
    const items = Array.isArray(p) ? p : (p?.items || p?.visitas || []);

    if (!items.length) {
      el.innerHTML = `<div class="empty-state"><div class="empty-title">Sin visitas registradas</div></div>`;
      return;
    }

    el.innerHTML = `<div style="display:flex;flex-direction:column;gap:10px">
      ${items.map(v => {
        const estado = v.estado || v.status || 'desconocido';
        const cls = estado === 'COMPLETADA' || estado === 'completed' ? 'green' : estado === 'EN_CURSO' ? 'accent' : '';
        const score = v.puntuacion ? `⭐ ${v.puntuacion}/5` : '';
        return `<div style="background:var(--surface-1);border:1px solid var(--border-subtle);border-radius:10px;padding:14px">
          <div style="display:flex;justify-content:space-between;align-items:flex-start;margin-bottom:6px">
            <div>
              <div style="font-weight:600;font-size:14px">${_esc(v.especialista_nombre || v.especialista || 'Especialista')}</div>
              <div style="font-size:11px;color:var(--text-muted)">${_esc(_time(v.fecha_inicio || v.ts))}</div>
            </div>
            <div style="display:flex;gap:6px;align-items:center">
              ${score ? `<span style="font-size:12px;color:var(--accent-orange)">${score}</span>` : ''}
              <span class="chip ${cls}">${_esc(estado)}</span>
            </div>
          </div>
          ${v.proposito || v.objetivo ? `<div style="font-size:12px;color:var(--text-secondary);margin-top:4px">${_esc(v.proposito || v.objetivo)}</div>` : ''}
          ${v.hallazgos ? `<div style="font-size:11px;color:var(--text-muted);margin-top:6px;border-top:1px solid var(--border-subtle);padding-top:6px">${_esc(v.hallazgos.slice(0, 150))}${v.hallazgos.length > 150 ? '...' : ''}</div>` : ''}
          ${estado === 'EN_CURSO' ? `
            <div class="action-bar" style="padding-top:8px;margin-top:8px;border-top:1px solid var(--border-subtle)">
              <button class="action-btn primary" data-finalizar="${_esc(String(v.id || v.visita_id || ''))}">
                Finalizar visita
              </button>
            </div>` : ''}
        </div>`;
      }).join('')}
    </div>`;

    // Finalizar visita buttons
    el.querySelectorAll('[data-finalizar]').forEach(btn => {
      btn.addEventListener('click', () => _finalizarVisita(btn.dataset.finalizar, container));
    });
  } catch (e) {
    if (el) el.innerHTML = `<div class="empty-state"><div class="empty-title" style="color:var(--accent-red)">Error: ${_esc(e.message)}</div></div>`;
  }
}

/* ─── Recomendaciones ────────────────────────────────────────────────── */

async function _loadRecomendaciones(container) {
  const el = container.querySelector('#tut-recomendaciones-list');
  if (!el) return;
  const filtro = container.querySelector('#tut-rec-filtro')?.value || '';
  try {
    const url = `/tutorias/recomendaciones${filtro ? `?estado=${encodeURIComponent(filtro)}&limite=30` : '?limite=30'}`;
    const r = await fetch(url);
    const d = await r.json().catch(() => null);
    const p = d?.data ?? d;
    const items = Array.isArray(p) ? p : (p?.items || p?.recomendaciones || []);

    if (!items.length) {
      el.innerHTML = `<div class="empty-state"><div class="empty-title">Sin recomendaciones</div></div>`;
      return;
    }

    el.innerHTML = `<div style="display:flex;flex-direction:column;gap:8px">
      ${items.map(rec => {
        const id = rec.id || rec.recomendacion_id || '';
        const estado = rec.estado || rec.status || 'PENDIENTE';
        const cls = estado === 'IMPLEMENTADA' ? 'green' : estado === 'EN_PROCESO' ? 'accent' : 'orange';
        return `<div style="background:var(--surface-1);border:1px solid var(--border-subtle);border-radius:10px;padding:12px">
          <div style="display:flex;justify-content:space-between;align-items:flex-start;gap:8px">
            <div style="flex:1">
              <div style="font-weight:600;font-size:13px">${_esc(rec.titulo || rec.title || rec.descripcion?.slice(0,60) || '--')}</div>
              ${rec.descripcion ? `<div style="font-size:11px;color:var(--text-secondary);margin-top:3px">${_esc(rec.descripcion.slice(0, 120))}${rec.descripcion.length > 120 ? '...' : ''}</div>` : ''}
              <div style="font-size:10px;color:var(--text-muted);margin-top:4px">${_esc(_time(rec.fecha_creacion || rec.ts))}</div>
            </div>
            <span class="chip ${cls}" style="white-space:nowrap">${_esc(estado)}</span>
          </div>
          ${estado !== 'IMPLEMENTADA' ? `
          <div style="display:flex;gap:6px;margin-top:8px;flex-wrap:wrap">
            ${estado === 'PENDIENTE' ? `<button class="action-btn" style="font-size:11px;padding:4px 8px" data-rec-id="${_esc(String(id))}" data-rec-action="EN_PROCESO">→ En proceso</button>` : ''}
            <button class="action-btn success" style="font-size:11px;padding:4px 8px" data-rec-id="${_esc(String(id))}" data-rec-action="IMPLEMENTADA">✓ Implementada</button>
          </div>` : ''}
        </div>`;
      }).join('')}
    </div>`;

    el.querySelectorAll('[data-rec-id]').forEach(btn => {
      btn.addEventListener('click', () => _updateRecomendacion(btn.dataset.recId, btn.dataset.recAction, container));
    });
  } catch (e) {
    if (el) el.innerHTML = `<div class="empty-state"><div class="empty-title" style="color:var(--accent-red)">Error: ${_esc(e.message)}</div></div>`;
  }
}

/* ─── Acciones ───────────────────────────────────────────────────────── */

async function _registrarEspecialista(container) {
  const nombre = container.querySelector('#esp-nombre')?.value?.trim();
  const rol    = container.querySelector('#esp-rol')?.value?.trim();
  const esp    = container.querySelector('#esp-especialidad')?.value?.trim();
  const msg    = container.querySelector('#tut-reg-msg');
  if (!nombre) { if (msg) msg.textContent = 'El nombre es requerido'; return; }

  if (msg) msg.textContent = 'Registrando...';
  try {
    const r = await fetch('/tutorias/especialistas', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ nombre, rol, especialidad: esp }),
    });
    const d = await r.json().catch(() => ({}));
    if (!r.ok || d.ok === false) throw new Error(d.error || d.detail || `HTTP ${r.status}`);
    window.AtlasToast?.show(`Especialista "${nombre}" registrado`, 'success');
    if (msg) msg.textContent = '✓ Especialista registrado';
    container.querySelector('#esp-nombre').value = '';
    container.querySelector('#esp-rol').value = '';
    container.querySelector('#esp-especialidad').value = '';
    _loadStats(container);
  } catch (e) {
    window.AtlasToast?.show(e.message, 'error');
    if (msg) msg.textContent = `Error: ${e.message}`;
  }
  setTimeout(() => { const m = container.querySelector('#tut-reg-msg'); if (m) m.textContent = ''; }, 5000);
}

async function _iniciarVisita(container) {
  const espId   = container.querySelector('#vis-especialista')?.value;
  const prop    = container.querySelector('#vis-proposito')?.value?.trim();
  const msg     = container.querySelector('#tut-vis-msg');
  if (!espId) { if (msg) msg.textContent = 'Selecciona un especialista'; return; }

  if (msg) msg.textContent = 'Iniciando visita...';
  try {
    const r = await fetch('/tutorias/visitas', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ especialista_id: espId, proposito: prop || 'Visita general' }),
    });
    const d = await r.json().catch(() => ({}));
    if (!r.ok || d.ok === false) throw new Error(d.error || d.detail || `HTTP ${r.status}`);
    window.AtlasToast?.show('Visita iniciada', 'success');
    if (msg) msg.textContent = '✓ Visita iniciada';
    _loadStats(container);
  } catch (e) {
    window.AtlasToast?.show(e.message, 'error');
    if (msg) msg.textContent = `Error: ${e.message}`;
  }
  setTimeout(() => { const m = container.querySelector('#tut-vis-msg'); if (m) m.textContent = ''; }, 5000);
}

async function _finalizarVisita(visitaId, container) {
  if (!confirm('¿Finalizar esta visita?')) return;
  try {
    const r = await fetch(`/tutorias/visitas/${encodeURIComponent(visitaId)}/finalizar`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ hallazgos: 'Visita completada', puntuacion: 5 }),
    });
    const d = await r.json().catch(() => ({}));
    if (!r.ok || d.ok === false) throw new Error(d.error || d.detail || `HTTP ${r.status}`);
    window.AtlasToast?.show('Visita finalizada', 'success');
    _loadVisitas(container);
    _loadStats(container);
  } catch (e) {
    window.AtlasToast?.show(e.message, 'error');
  }
}

async function _updateRecomendacion(id, estado, container) {
  try {
    const r = await fetch(`/tutorias/recomendaciones/${encodeURIComponent(id)}/estado?estado=${encodeURIComponent(estado)}`, {
      method: 'PUT',
    });
    const d = await r.json().catch(() => ({}));
    if (!r.ok || d.ok === false) throw new Error(d.error || d.detail || `HTTP ${r.status}`);
    window.AtlasToast?.show(`Recomendación → ${estado}`, 'success');
    _loadRecomendaciones(container);
    _loadStats(container);
  } catch (e) {
    window.AtlasToast?.show(e.message, 'error');
  }
}

window.AtlasModuleTutorias = { id: 'tutorias' };
