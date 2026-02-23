/**
 * ATLAS v4 — Hash Router
 * Simple hash-based routing with history support.
 */
import { set, get, on } from './state.js';

const _routes = new Map();
let _currentCleanup = null;

export function register(pattern, handler) {
  _routes.set(pattern, handler);
}

export function navigate(path) {
  window.location.hash = path;
}

export function currentRoute() {
  return window.location.hash.slice(1) || '/';
}

function _matchRoute(hash) {
  const path = hash || '/';

  if (_routes.has(path)) return { handler: _routes.get(path), params: {} };

  for (const [pattern, handler] of _routes) {
    if (!pattern.includes(':')) continue;
    const patternParts = pattern.split('/');
    const pathParts = path.split('/');
    if (patternParts.length !== pathParts.length) continue;

    const params = {};
    let match = true;
    for (let i = 0; i < patternParts.length; i++) {
      if (patternParts[i].startsWith(':')) {
        params[patternParts[i].slice(1)] = pathParts[i];
      } else if (patternParts[i] !== pathParts[i]) {
        match = false; break;
      }
    }
    if (match) return { handler, params };
  }

  return null;
}

function _handleRoute() {
  const hash = window.location.hash.slice(1) || '/';
  set('route', hash);

  if (_currentCleanup) {
    try { _currentCleanup(); } catch {}
    _currentCleanup = null;
  }

  const match = _matchRoute(hash);
  if (match) {
    const cleanup = match.handler(match.params);
    if (typeof cleanup === 'function') _currentCleanup = cleanup;
  }
}

export function init() {
  window.addEventListener('hashchange', _handleRoute);
  _handleRoute();
}

window.AtlasRouter = { register, navigate, currentRoute, init };
