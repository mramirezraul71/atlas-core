/**
 * ATLAS v4 — Smart Polling Manager
 * Auto-pauses when tab is hidden, exponential backoff on errors.
 */
const _polls = new Map();

export function poll(id, url, intervalMs, callback) {
  if (_polls.has(id)) stop(id);

  const state = {
    url, intervalMs, callback,
    timer: null,
    paused: false,
    errorCount: 0,
    tick: null,
  };

  async function tick() {
    if (state.paused || document.hidden) return;
    try {
      const res = await fetch(url);
      const data = await res.json();
      state.errorCount = 0;
      callback(data, null);
    } catch (e) {
      state.errorCount++;
      callback(null, e);
    }
    const backoff = state.errorCount > 0
      ? Math.min(intervalMs * Math.pow(2, state.errorCount), 60000)
      : intervalMs;
    state.timer = setTimeout(tick, backoff);
  }

  state.tick = tick;
  state.timer = setTimeout(tick, 100);
  _polls.set(id, state);

  return () => stop(id);
}

export function stop(id) {
  const s = _polls.get(id);
  if (s) {
    clearTimeout(s.timer);
    _polls.delete(id);
  }
}

export function stopAll() {
  for (const id of _polls.keys()) stop(id);
}

export function pause(id) {
  const s = _polls.get(id);
  if (s) s.paused = true;
}

export function resume(id) {
  const s = _polls.get(id);
  if (s) { s.paused = false; }
}

document.addEventListener('visibilitychange', () => {
  const hidden = document.hidden;
  for (const [, state] of _polls) {
    clearTimeout(state.timer);
    if (hidden) {
      state.timer = null;
    } else if (!state.paused) {
      // Resume the real polling loop after tab returns to foreground.
      state.timer = setTimeout(state.tick, 200);
    }
  }
});

window.AtlasPolling = { poll, stop, stopAll, pause, resume };
