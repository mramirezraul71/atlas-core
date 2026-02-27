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
    if (hidden) {
      clearTimeout(state.timer);
    } else if (!state.paused) {
      state.timer = setTimeout(async () => {
        try {
          const res = await fetch(state.url);
          const data = await res.json();
          state.callback(data, null);
        } catch (e) { state.callback(null, e); }
        state.timer = setTimeout(() => {}, state.intervalMs);
      }, 200);
    }
  }
});

window.AtlasPolling = { poll, stop, stopAll, pause, resume };
