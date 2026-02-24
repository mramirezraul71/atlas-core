/**
 * ATLAS v4 — Mega Menu
 * Slide-in navigation organized by category. Extensible via manifest.
 */
import { get, set, on } from '../lib/state.js';
import { navigate } from '../lib/router.js';

const MENU_ITEMS = [
  {
    category: 'AI & Tools',
    items: [
      { id: 'assistant', label: 'AI Assistant', desc: 'Chat with Atlas agent', icon: 'brain', route: '/ask' },
      { id: 'workspace', label: 'Workspace', desc: 'Agent IDE interface', icon: 'terminal', href: '/workspace' },
      { id: 'interpreter', label: 'Interpreter', desc: 'Code execution engine', icon: 'code', route: '/module/interpreter' },
    ]
  },
  {
    category: 'Legacy',
    items: [
      { id: 'ui-v3', label: 'UI v3 (legacy)', desc: 'Previous dashboard (/ui)', icon: 'layout', href: '/ui' },
    ]
  },
  {
    category: 'Monitoring',
    items: [
      { id: 'health', label: 'System Health', desc: 'CPU, memory, services', icon: 'activity', route: '/module/health' },
      { id: 'bitacora', label: 'Bitacora', desc: 'Activity log', icon: 'file-text', route: '/module/bitacora' },
      { id: 'monitor', label: 'Monitor', desc: 'Real-time operations', icon: 'monitor', route: '/module/monitor' },
      { id: 'nervous', label: 'Nervous System', desc: 'Neural state tracking', icon: 'zap', route: '/module/nervous' },
      { id: 'audit', label: 'Audit Tail', desc: 'Recent audit lines', icon: 'file-text', route: '/module/audit' },
      { id: 'events', label: 'Event Bus', desc: 'Kernel event history', icon: 'zap', route: '/module/events' },
      { id: 'healing', label: 'Healing', desc: 'Recent healing actions', icon: 'activity', route: '/module/healing' },
      { id: 'memory', label: 'Cognitive Memory', desc: 'Memory subsystems status', icon: 'brain', route: '/module/memory' },
    ]
  },
  {
    category: 'Control',
    items: [
      { id: 'governance', label: 'Governance', desc: 'Policy & autonomy', icon: 'shield', route: '/module/autonomy' },
      { id: 'autonomy', label: 'Autonomy', desc: 'Daemon, tasks, governance status', icon: 'cpu', route: '/module/autonomy' },
      { id: 'approvals', label: 'Approvals', desc: 'Pending approvals', icon: 'shield', route: '/module/approvals' },
    ]
  },
  {
    category: 'Configuration',
    items: [
      { id: 'config', label: 'AI Models', desc: 'Provider & model config', icon: 'sliders', route: '/module/config' },
      { id: 'comms', label: 'Communications', desc: 'WhatsApp, Telegram', icon: 'message-circle', route: '/module/comms' },
      { id: 'voice', label: 'Voice', desc: 'Voice assistant / TTS', icon: 'message-circle', route: '/module/voice' },
    ]
  },
  {
    category: 'Tools',
    items: [
      { id: 'api', label: 'API Explorer', desc: 'Browse/call OpenAPI endpoints', icon: 'code', route: '/module/api' },
    ]
  },
  {
    category: 'Apps',
    items: [
      { id: 'trading', label: 'Trading', desc: 'Market analysis (coming soon)', icon: 'trending-up', route: '/module/trading', badge: 'Soon' },
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

  _menu.querySelectorAll('.megamenu-item').forEach(item => {
    item.addEventListener('click', () => {
      const route = item.dataset.route;
      const href = item.dataset.href;
      close();
      if (href) window.open(href, '_blank');
      else if (route) navigate(route);
    });
  });

  on('menuOpen', open => {
    if (open) { _overlay.classList.add('open'); _menu.classList.add('open'); }
    else { _overlay.classList.remove('open'); _menu.classList.remove('open'); }
  });
}

function _buildHTML() {
  return MENU_ITEMS.map(cat => `
    <div class="megamenu-category">
      <div class="megamenu-category-label">${cat.category}</div>
      ${cat.items.map(it => `
        <div class="megamenu-item" data-id="${it.id}" ${it.route ? `data-route="${it.route}"` : ''} ${it.href ? `data-href="${it.href}"` : ''}>
          <div class="megamenu-item-icon">${ICONS[it.icon] || ''}</div>
          <div class="megamenu-item-text">
            <div class="label">${it.label}</div>
            <div class="desc">${it.desc}</div>
          </div>
          ${it.badge ? `<span class="megamenu-item-badge">${it.badge}</span>` : ''}
        </div>
      `).join('')}
    </div>
  `).join('<div class="megamenu-divider"></div>');
}

export function open() { set('menuOpen', true); }
export function close() { set('menuOpen', false); }
export function toggle() { set('menuOpen', !get('menuOpen')); }

export function addItem(category, item) {
  const cat = MENU_ITEMS.find(c => c.category === category);
  if (cat) cat.items.push(item);
  else MENU_ITEMS.push({ category, items: [item] });
  if (_menu) _menu.innerHTML = _buildHTML();
}

window.AtlasMegaMenu = { mount, open, close, toggle, addItem };
