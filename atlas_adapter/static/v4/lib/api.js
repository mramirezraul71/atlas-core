/**
 * ATLAS v4 — API Client
 * Centralized fetch wrapper with retry, timeout, and error normalization.
 */
const BASE = '';

async function request(path, opts = {}) {
  const { method = 'GET', body, timeout = 15000, retries = 1, headers = {} } = opts;
  const ctrl = new AbortController();
  const timer = setTimeout(() => ctrl.abort(), timeout);

  const fetchOpts = {
    method,
    signal: ctrl.signal,
    headers: { 'Content-Type': 'application/json', ...headers },
  };
  if (body) fetchOpts.body = typeof body === 'string' ? body : JSON.stringify(body);

  let lastErr;
  for (let i = 0; i <= retries; i++) {
    try {
      const res = await fetch(BASE + path, fetchOpts);
      clearTimeout(timer);
      if (!res.ok) {
        const text = await res.text().catch(() => '');
        throw new Error(`HTTP ${res.status}: ${text.slice(0, 200)}`);
      }
      const ct = res.headers.get('content-type') || '';
      return ct.includes('json') ? res.json() : res.text();
    } catch (e) {
      lastErr = e;
      if (e.name === 'AbortError') throw new Error(`Timeout after ${timeout}ms: ${path}`);
      if (i < retries) await new Promise(r => setTimeout(r, 500 * (i + 1)));
    }
  }
  throw lastErr;
}

export function get(path, opts) { return request(path, { ...opts, method: 'GET' }); }
export function post(path, body, opts) { return request(path, { ...opts, method: 'POST', body }); }
export function put(path, body, opts) { return request(path, { ...opts, method: 'PUT', body }); }
export function del(path, opts) { return request(path, { ...opts, method: 'DELETE' }); }

/**
 * SSE streaming — returns { stream, abort }
 * stream is an async iterable of parsed JSON events.
 */
export function sse(path, body) {
  const ctrl = new AbortController();
  const fetchPromise = fetch(BASE + path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
    signal: ctrl.signal,
  });

  async function* iterate() {
    const res = await fetchPromise;
    if (!res.ok) {
      const text = await res.text().catch(() => '');
      throw new Error(`HTTP ${res.status}: ${text.slice(0, 200)}`);
    }
    const reader = res.body.getReader();
    const decoder = new TextDecoder();
    let buffer = '';

    while (true) {
      const { done, value } = await reader.read();
      if (done) break;
      buffer += decoder.decode(value, { stream: true });

      // Parse SSE events: blocks separated by blank line.
      // Each block may contain: "event: <type>" and one or more "data: <payload>" lines.
      while (true) {
        const sep = buffer.indexOf('\n\n');
        if (sep < 0) break;
        const block = buffer.slice(0, sep);
        buffer = buffer.slice(sep + 2);

        const lines = block.split('\n').map(l => l.replace(/\r$/, ''));
        let eventType = 'message';
        const dataLines = [];

        for (const raw of lines) {
          const line = raw.trimEnd();
          if (!line) continue;
          if (line.startsWith('event:')) {
            eventType = line.slice(6).trim() || 'message';
            continue;
          }
          if (line.startsWith('data:')) {
            dataLines.push(line.slice(5).trimStart());
          }
        }

        if (dataLines.length === 0) continue;
        const dataText = dataLines.join('\n');
        if (dataText === '[DONE]') return;

        let payload = dataText;
        try { payload = JSON.parse(dataText); } catch {}

        if (payload && typeof payload === 'object' && !Array.isArray(payload)) {
          // Spread payload into the event object so callers can access fields directly.
          yield { type: eventType, ...payload };
        } else {
          yield { type: eventType, data: payload };
        }
      }
    }
  }

  return { stream: iterate(), abort: () => ctrl.abort() };
}

window.AtlasAPI = { get, post, put, del, sse };
