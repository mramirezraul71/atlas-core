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
      { id: 'health',      label: 'System Health',  desc: 'Score, checks y recursos del sistema', icon: 'activity',  route: '/health' },
      { id: 'api',         label: 'API Explorer',   desc: 'Explorar y probar endpoints en vivo',   icon: 'code',      route: '/api_explorer' },
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
      { id: 'audit',    label: 'Audit Trail',   desc: 'Historial de auditoría del sistema',    icon: 'shield',      route: '/audit' },
      { id: 'events',   label: 'Event Bus',     desc: 'Eventos del bus del kernel en vivo',    icon: 'zap',         route: '/events' },
      { id: 'healing',  label: 'Auto-Healing',  desc: 'Historial de auto-reparación ANS',      icon: 'activity',    route: '/healing' },
    ]
  },
  {
    category: 'Control & Autonomía',
    items: [
      { id: 'autonomy',  label: 'Autonomy',      desc: 'Daemon ANS, tareas y modo de gobierno', icon: 'cpu',     route: '/autonomy' },
      { id: 'approvals', label: 'Aprobaciones',  desc: 'Flujos de aprobación pendientes',       icon: 'shield',  route: '/approvals' },
      { id: 'learning',  label: 'Aprendizaje',   desc: 'Motor de aprendizaje y patrones',       icon: 'trending-up', route: '/learning' },
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
    category: 'Integraciones',
    items: [
      { id: 'comms',   label: 'Comunicaciones', desc: 'WhatsApp/WAHA, webhooks',       icon: 'message-circle', route: '/comms' },
      { id: 'voice',   label: 'Voz / TTS',      desc: 'Motor de voz y síntesis',       icon: 'mic',            route: '/voice' },
      { id: 'trading', label: 'Trading AI',     desc: 'Análisis de mercados con Grok', icon: 'trending-up',    route: '/trading', badge: 'Beta' },
    ]
  },
  {
    category: 'Sistema',
    items: [
      { id: 'ui-v3',  label: 'Dashboard v3.8',    desc: 'Panel operacional completo legado', icon: 'layout', href: '/v3' },
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
    const match = r && (r === clean || r === `/${clean.replace(/^\//, '')}`);
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
