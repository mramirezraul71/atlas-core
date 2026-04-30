/**
 * ATLAS v4 — Progress Tracker Component
 */
let _el = null;
let _fillEl = null;

export function mount(container) {
  _el = document.createElement('div');
  _el.className = 'assistant-progress';
  _el.style.display = 'none';
  _el.innerHTML = `
    <div class="progress-track">
      <div class="progress-fill" style="width: 0%"></div>
    </div>
  `;
  _fillEl = _el.querySelector('.progress-fill');
  container.appendChild(_el);
}

export function update(pct) {
  if (!_el) return;
  _el.style.display = 'block';
  _fillEl.style.width = Math.min(100, Math.max(0, pct)) + '%';
}

export function hide() {
  if (_el) _el.style.display = 'none';
}

export function reset() {
  if (_fillEl) _fillEl.style.width = '0%';
  hide();
}

window.AtlasProgress = { mount, update, hide, reset };
