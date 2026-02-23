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
    const reader = res.body.getReader();
    const decoder = new TextDecoder();
    let buffer = '';

    while (true) {
      const { done, value } = await reader.read();
      if (done) break;
      buffer += decoder.decode(value, { stream: true });

      const lines = buffer.split('\n');
      buffer = lines.pop() || '';

      for (const line of lines) {
        const trimmed = line.trim();
        if (!trimmed || !trimmed.startsWith('data: ')) continue;
        const json = trimmed.slice(6);
        if (json === '[DONE]') return;
        try { yield JSON.parse(json); } catch {}
      }
    }
  }

  return { stream: iterate(), abort: () => ctrl.abort() };
}

window.AtlasAPI = { get, post, put, del, sse };
