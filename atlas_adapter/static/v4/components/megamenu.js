/**
 * ATLAS v4.2 — Mega Menu
 * Menú lateral con categorías, sub-items y acceso directo a filtros.
 * Estructura reagrupada: Núcleo / Actividad / Control / Integraciones / Sistema
 */
import { get, set, on } from '../lib/state.js';
import { navigate } from '../lib/router.js';

// Items con soporte de `children` para sub-navegación
const MENU_ITEMS = [
  {
    category: 'Núcleo',
    items: [
      { id: 'health',   label: 'Salud del sistema',      desc: 'Score, checks y recursos del sistema',     icon: 'activity', route: '/health' },
      { id: 'modules',  label: 'Módulos del Cuerpo', desc: 'Estado de los 16 subsistemas de Atlas',    icon: 'grid',     route: '/modules' },
      { id: 'api',      label: 'Explorador API',        desc: 'Explorar y probar endpoints en vivo',      icon: 'code',     route: '/api_explorer' },
    ]
  },
  {
    category: 'Actividad',
    items: [
      {
        id: 'bitacora', label: 'Bitácora ANS', desc: 'Registro de eventos del sistema', icon: 'file-text', route: '/bitacora',
        children: [
          { label: 'Todos los eventos',   route: '/bitacora' },
          { label: 'Solo errores',        route: '/bitacora?f=error' },
          { label: 'Advertencias',        route: '/bitacora?f=warn' },
          { label: 'Informativos',        route: '/bitacora?f=info' },
        ]
      },
      { id: 'audit',    label: 'Rastro de auditoria',   desc: 'Historial de auditoria del sistema',    icon: 'shield',      route: '/audit' },
      { id: 'events',   label: 'Bus de eventos',     desc: 'Eventos del bus del kernel en vivo',    icon: 'zap',         route: '/events' },
      { id: 'healing',  label: 'Auto-reparacion',  desc: 'Historial de auto-reparacion ANS',      icon: 'activity',    route: '/healing' },
    ]
  },
  {
    category: 'Control & Autonomía',
    items: [
      { id: 'autonomy',  label: 'Autonomia',      desc: 'Daemon ANS, tareas y modo de gobierno', icon: 'cpu',     route: '/autonomy' },
      { id: 'approvals', label: 'Aprobaciones',  desc: 'Flujos de aprobación pendientes',       icon: 'shield',  route: '/approvals' },
      { id: 'learning',  label: 'Aprendizaje',   desc: 'Motor de aprendizaje y patrones',       icon: 'trending-up', route: '/learning' },
    ]
  },
  {
    category: 'Inteligencia',
    items: [
      { id: 'chat',      label: 'Chat Cerebro',      desc: 'Conversación directa con el cerebro',    icon: 'message-square', route: '/chat' },
      { id: 'clawd-direct', label: 'ATLAS Directo',  desc: 'Canal directo con ATLAS + bridge operativo',    icon: 'message-square', route: '/clawd-direct' },
      { id: 'codex-supervisor', label: 'Codex Supervisor', desc: 'Supervisor para asignar tareas y seguimiento', icon: 'cpu', route: '/codex-supervisor' },
      { id: 'cognitive', label: 'Sistema Nervioso',  desc: 'Nodos cerebrales, goals y memoria',      icon: 'brain',          route: '/cognitive' },
      { id: 'tutorias',  label: 'Tutorías',          desc: 'Especialistas, visitas y recomendaciones', icon: 'users',         route: '/tutorias' },
    ]
  },
  {
    category: 'Memoria & Datos',
    items: [
      { id: 'memory',  label: 'Memoria Cognitiva', desc: 'Lifelog, World Model y Autobiográfica', icon: 'brain',    route: '/memory' },
      { id: 'config',  label: 'Modelos AI',         desc: 'Proveedores, modelos y configuración',  icon: 'sliders', route: '/config' },
    ]
  },
  {
    category: 'Sensorial',
    items: [
      { id: 'vision',  label: 'Visión Artificial', desc: 'Cámaras, snapshots y stream en vivo',   icon: 'eye',            route: '/vision' },
      { id: 'comms',   label: 'Comunicaciones',    desc: 'WhatsApp/WAHA, webhooks',               icon: 'message-circle', route: '/comms' },
      { id: 'voice',   label: 'Voz / TTS',         desc: 'Motor de voz y síntesis',               icon: 'mic',            route: '/voice' },
    ]
  },
  {
    category: 'Integraciones',
    items: [
      { id: 'trading',      label: 'IA de trading',        desc: 'Analisis de mercados con Grok',                    icon: 'trending-up', route: '/trading',      badge: 'Beta' },
      { id: 'atlas-quant',  label: 'Atlas Code-Quant',  desc: 'Bot de trading algoritmico con IA — puerto 8792', icon: 'trending-up', route: '/atlas-quant',  badge: 'Nuevo'  },
      { id: 'atlas-quant-scanner', label: 'Escaner Quant', desc: 'Barrido permanente de oportunidades explicables', icon: 'trending-up', route: '/atlas-quant', badge: 'Live' },
      { id: 'access-control', label: 'Control de Acceso', desc: 'Usuarios, sesiones y permisos del sistema',      icon: 'lock',        route: '/access-control' },
    ]
  },
  {
    category: 'Brazos',
    items: [
      { id: 'rauli-vision',    label: 'Panel de Rauli Vision',    desc: 'Acceso directo al brazo sensorial', icon: 'eye',     route: '/apps/vision' },
      { id: 'rauli-panaderia', label: 'Panel de Rauli Panaderia', desc: 'Acceso directo al brazo operativo', icon: 'package', route: '/apps/panaderia' },
      { id: 'mis-apps',        label: 'Mis Apps',                  desc: 'Panel completo de brazos ATLAS',    icon: 'layers',  route: '/apps' },
    ]
  },
  {
    category: 'Sistema',
    items: [
      { id: 'tools-menu', label: 'Menu de herramientas',        desc: 'Inventario, salud y mejoras de herramientas', icon: 'grid', route: '/tools-menu' },
      { id: 'software-center', label: 'Centro de software', desc: 'Software instalado, drivers y ciclo de mantenimiento', icon: 'package', route: '/software-center' },
      { id: 'update', label: 'Actualizar ATLAS',   desc: 'Comprobar y aplicar actualizaciones', icon: 'zap', route: '/__update' },
    ]
  },
];

const ICONS = {
  brain: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M12 2a7 7 0 017 7c0 3-2 5-4 6v3h-6v-3c-2-1-4-3-4-6a7 7 0 017-7z"/></svg>',
  terminal: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="4 17 10 11 4 5"/><line x1="12" y1="19" x2="20" y2="19"/></svg>',
  code: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="16 18 22 12 16 6"/><polyline points="8 6 2 12 8 18"/></svg>',
  layout: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="3" y="3" width="18" height="18" rx="2"/><line x1="3" y1="9" x2="21" y2="9"/><line x1="9" y1="9" x2="9" y2="21"/></svg>',
  activity: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M22 12h-4l-3 9L9 3l-3 9H2"/></svg>',
  'file-text': '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M14 2H6a2 2 0 00-2 2v16a2 2 0 002 2h12a2 2 0 002-2V8z"/><path d="M14 2v6h6M16 13H8M16 17H8M10 9H8"/></svg>',
  monitor: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="2" y="3" width="20" height="14" rx="2"/><path d="M8 21h8M12 17v4"/></svg>',
  zap: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polygon points="13 2 3 14 12 14 11 22 21 10 12 10 13 2"/></svg>',
  shield: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M12 22s8-4 8-10V5l-8-3-8 3v7c0 6 8 10 8 10z"/></svg>',
  cpu: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="4" y="4" width="16" height="16" rx="2"/><rect x="9" y="9" width="6" height="6"/><path d="M9 1v3M15 1v3M9 20v3M15 20v3M20 9h3M20 14h3M1 9h3M1 14h3"/></svg>',
  sliders: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><line x1="4" y1="21" x2="4" y2="14"/><line x1="4" y1="10" x2="4" y2="3"/><line x1="12" y1="21" x2="12" y2="12"/><line x1="12" y1="8" x2="12" y2="3"/><line x1="20" y1="21" x2="20" y2="16"/><line x1="20" y1="12" x2="20" y2="3"/><line x1="1" y1="14" x2="7" y2="14"/><line x1="9" y1="8" x2="15" y2="8"/><line x1="17" y1="16" x2="23" y2="16"/></svg>',
  'message-circle': '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M21 11.5a8.38 8.38 0 01-.9 3.8 8.5 8.5 0 01-7.6 4.7 8.38 8.38 0 01-3.8-.9L3 21l1.9-5.7a8.38 8.38 0 01-.9-3.8 8.5 8.5 0 014.7-7.6 8.38 8.38 0 013.8-.9h.5a8.48 8.48 0 018 8v.5z"/></svg>',
  'trending-up': '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="22 7 13.5 15.5 8.5 10.5 2 17"/><polyline points="16 7 22 7 22 13"/></svg>',
  mic: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M12 1a3 3 0 00-3 3v8a3 3 0 006 0V4a3 3 0 00-3-3z"/><path d="M19 10v2a7 7 0 01-14 0v-2M12 19v4M8 23h8"/></svg>',
  chevron: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="9 18 15 12 9 6"/></svg>',
  grid: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="3" y="3" width="7" height="7"/><rect x="14" y="3" width="7" height="7"/><rect x="14" y="14" width="7" height="7"/><rect x="3" y="14" width="7" height="7"/></svg>',
  eye: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z"/><circle cx="12" cy="12" r="3"/></svg>',
  'message-square': '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M21 15a2 2 0 01-2 2H7l-4 4V5a2 2 0 012-2h14a2 2 0 012 2z"/></svg>',
  users: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M17 21v-2a4 4 0 00-4-4H5a4 4 0 00-4 4v2"/><circle cx="9" cy="7" r="4"/><path d="M23 21v-2a4 4 0 00-3-3.87M16 3.13a4 4 0 010 7.75"/></svg>',
  layers: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polygon points="12 2 2 7 12 12 22 7 12 2"/><polyline points="2 17 12 22 22 17"/><polyline points="2 12 12 17 22 12"/></svg>',
  package: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><line x1="16.5" y1="9.4" x2="7.5" y2="4.21"/><path d="M21 16V8a2 2 0 00-1-1.73l-7-4a2 2 0 00-2 0l-7 4A2 2 0 002 8v8a2 2 0 001 1.73l7 4a2 2 0 002 0l7-4A2 2 0 0021 16z"/><polyline points="3.27 6.96 12 12.01 20.73 6.96"/><line x1="12" y1="22.08" x2="12" y2="12"/></svg>',
};

let _overlay = null;
let _menu = null;

export function mount(parent) {
  _overlay = document.createElement('div');
  _overlay.className = 'megamenu-overlay';
  _overlay.addEventListener('click', close);
  parent.appendChild(_overlay);

  _menu = document.createElement('nav');
  _menu.className = 'megamenu';
  _menu.innerHTML = _buildHTML();
  parent.appendChild(_menu);

  _attachEvents();

  on('menuOpen', isOpen => {
    if (isOpen) { _overlay.classList.add('open'); _menu.classList.add('open'); }
    else         { _overlay.classList.remove('open'); _menu.classList.remove('open'); }
  });

  // Marca el item activo en cada cambio de ruta
  on('route', _updateActive);
  _updateActive(get('route') || '/');
}

function _buildHTML() {
  return MENU_ITEMS.map(cat => {
    const items = cat.items.map(it => {
      const hasChildren = it.children?.length > 0;
      return `
        <div class="megamenu-item" data-id="${it.id}"
          ${it.route ? `data-route="${it.route}"` : ''}
          ${it.href  ? `data-href="${it.href}"`   : ''}
          ${hasChildren ? 'data-has-children="1"' : ''}>
          <div class="megamenu-item-icon">${ICONS[it.icon] || ''}</div>
          <div class="megamenu-item-text">
            <div class="label">${it.label}</div>
            <div class="desc">${it.desc}</div>
          </div>
          ${it.badge ? `<span class="megamenu-item-badge">${it.badge}</span>` : ''}
          ${hasChildren ? `<svg class="megamenu-item-chevron" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="9 18 15 12 9 6"/></svg>` : ''}
        </div>
        ${hasChildren ? `
          <div class="megamenu-subitems" id="sub-${it.id}">
            ${it.children.map(c => `
              <div class="megamenu-subitem" data-route="${c.route}">
                ${c.label}
              </div>
            `).join('')}
          </div>
        ` : ''}
      `;
    }).join('');

    return `
      <div class="megamenu-category">
        <div class="megamenu-category-label">${cat.category}</div>
        ${items}
      </div>
    `;
  }).join('<div class="megamenu-divider"></div>');
}

function _attachEvents() {
  if (!_menu) return;

  // Items principales
  _menu.querySelectorAll('.megamenu-item').forEach(item => {
    item.addEventListener('click', () => {
      const route    = item.dataset.route;
      const href     = item.dataset.href;
      const hasChild = item.dataset.hasChildren === '1';

      if (hasChild) {
        // Toggle sub-items
        const id  = item.dataset.id;
        const sub = document.getElementById(`sub-${id}`);
        const expanded = item.classList.toggle('expanded');
        if (sub) sub.classList.toggle('open', expanded);
        return; // No navegar al hacer expand
      }

      close();
      if (href)   window.open(href, '_blank');
      else if (route) navigate(route);
    });
  });

  // Sub-items
  _menu.querySelectorAll('.megamenu-subitem').forEach(sub => {
    sub.addEventListener('click', (e) => {
      e.stopPropagation();
      const route = sub.dataset.route;
      close();
      if (route) navigate(route);
    });
  });
}

function _updateActive(currentRoute) {
  if (!_menu) return;
  const clean = (currentRoute || '/').split('?')[0];

  _menu.querySelectorAll('.megamenu-item').forEach(item => {
    const r = item.dataset.route;
    // Highlight /modules también cuando se navega a sub-rutas de body-module
    const parentMatch = r === '/modules' && clean.startsWith('/body-module/');
    const match = r && (r === clean || r === `/${clean.replace(/^\//, '')}` || parentMatch);
    item.classList.toggle('active', !!match);
  });

  _menu.querySelectorAll('.megamenu-subitem').forEach(sub => {
    // Activo si la ruta base coincide (ignora params)
    const r = (sub.dataset.route || '').split('?')[0];
    sub.classList.toggle('active', r === clean);
  });

  // Auto-expand el bloque padre si hay un sub-item activo
  _menu.querySelectorAll('.megamenu-subitems').forEach(block => {
    const hasActive = block.querySelector('.megamenu-subitem.active');
    if (hasActive) {
      block.classList.add('open');
      const parentItem = block.previousElementSibling;
      if (parentItem?.classList.contains('megamenu-item')) {
        parentItem.classList.add('expanded');
      }
    }
  });
}

export function open()   { set('menuOpen', true); }
export function close()  { set('menuOpen', false); }
export function toggle() { set('menuOpen', !get('menuOpen')); }

export function addItem(category, item) {
  const cat = MENU_ITEMS.find(c => c.category === category);
  if (cat) cat.items.push(item);
  else MENU_ITEMS.push({ category, items: [item] });
  if (_menu) {
    _menu.innerHTML = _buildHTML();
    _attachEvents();
  }
}

window.AtlasMegaMenu = { mount, open, close, toggle, addItem };
