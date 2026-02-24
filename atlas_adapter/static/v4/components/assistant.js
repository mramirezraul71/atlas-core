/**
 * ATLAS v4 — AI Assistant (Perplexity-style)
 * Conversational SSE streaming with tool-call visualization, thinking dialog,
 * citations, and progress tracking.
 */
import { get, set, on } from '../lib/state.js';
import { sse } from '../lib/api.js';
import { navigate } from '../lib/router.js';

const SVG_SEND = '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M22 2L11 13"/><path d="M22 2l-7 20-4-9-9-4 20-7z"/></svg>';
const SVG_STOP = '<svg viewBox="0 0 24 24" fill="currentColor"><rect x="6" y="6" width="12" height="12" rx="2"/></svg>';

let _thread = [];
let _container = null;
let _threadEl = null;
let _inputEl = null;
let _sendBtn = null;
let _progressEl = null;
let _progressFill = null;
let _currentAbort = null;
let _currentMsgEl = null;
let _history = [];

const _THREAD_KEY = 'atlas-v4-thread-agent';
function _getThreadId() { return localStorage.getItem(_THREAD_KEY) || ''; }
function _setThreadId(tid) { if (tid) localStorage.setItem(_THREAD_KEY, String(tid)); }

export function render(container, params = {}) {
  _container = container;
  container.innerHTML = `
    <div class="assistant">
      <div class="assistant-thread" id="ast-thread"></div>
      <div class="assistant-progress" id="ast-progress" style="display:none">
        <div class="progress-track"><div class="progress-fill" id="ast-progress-fill" style="width:0%"></div></div>
      </div>
      <div class="assistant-input-area">
        <div class="assistant-input-wrap">
          <textarea class="assistant-input" id="ast-input" rows="1"
                    placeholder="Pregunta algo a Atlas..."></textarea>
          <button class="assistant-send" id="ast-send">${SVG_SEND}</button>
        </div>
      </div>
    </div>
  `;

  _threadEl = container.querySelector('#ast-thread');
  _inputEl = container.querySelector('#ast-input');
  _sendBtn = container.querySelector('#ast-send');
  _progressEl = container.querySelector('#ast-progress');
  _progressFill = container.querySelector('#ast-progress-fill');

  _inputEl.addEventListener('keydown', e => {
    if (e.key === 'Enter' && !e.shiftKey) { e.preventDefault(); _submit(); }
  });
  _inputEl.addEventListener('input', () => {
    _inputEl.style.height = 'auto';
    _inputEl.style.height = Math.min(_inputEl.scrollHeight, 200) + 'px';
    window.AtlasCompanion?.onUserType();
  });
  _sendBtn.addEventListener('click', () => {
    if (get('assistantBusy')) _abort();
    else _submit();
  });

  if (params.q) {
    _inputEl.value = params.q;
    setTimeout(() => _submit(), 100);
  } else {
    _inputEl.focus();
  }

  _renderThread();

  return () => { _abort(); };
}

function _submit() {
  const text = _inputEl.value.trim();
  if (!text || get('assistantBusy')) return;

  _thread.push({ role: 'user', content: text });
  _history.push({ role: 'user', content: text });
  _inputEl.value = '';
  _inputEl.style.height = 'auto';
  _renderThread();

  set('assistantBusy', true);
  _sendBtn.innerHTML = SVG_STOP;

  _streamAgent(text);
}

async function _streamAgent(message) {
  _progressEl.style.display = 'block';
  _progressFill.style.width = '0%';

  const assistantMsg = { role: 'assistant', content: '', thinking: [], tools: [], meta: {} };
  _thread.push(assistantMsg);
  _renderThread();

  const thread_id = _getThreadId();
  const { stream, abort } = sse('/agent/chat', {
    message,
    history: _history.slice(-20),
    thread_id: thread_id || null,
    user_id: 'owner',
  });
  _currentAbort = abort;

  try {
    for await (const evt of stream) {
      // Persist thread id whenever it appears (thread event or injected into done/thinking).
      if (evt && evt.thread_id) _setThreadId(evt.thread_id);
      switch (evt.type) {
        case 'thread':
          // Pure thread-id event; UI already persisted above.
          break;

        case 'thinking':
          assistantMsg.thinking.push(evt.text || evt.content || '');
          window.AtlasCompanion?.onAgentThink();
          break;

        case 'tool_call':
          assistantMsg.tools.push({
            name: evt.tool || evt.name,
            input: evt.input,
            status: 'running',
            result: null,
          });
          if (evt.progress_pct) _progressFill.style.width = evt.progress_pct + '%';
          break;

        case 'tool_progress':
          if (assistantMsg.tools.length > 0) {
            const last = assistantMsg.tools[assistantMsg.tools.length - 1];
            if (!last._output) last._output = '';
            last._output += (evt.line || evt.data || '') + '\n';
          }
          break;

        case 'tool_result': {
          const t = assistantMsg.tools.find(t => t.status === 'running');
          if (t) {
            t.status = (evt.error || evt.is_error) ? 'error' : 'done';
            t.result = evt.result || evt.output || '';
          }
          if (evt.progress_pct) _progressFill.style.width = evt.progress_pct + '%';
          break;
        }

        case 'text':
          assistantMsg.content += evt.text || evt.content || '';
          window.AtlasCompanion?.onAgentSpeak();
          break;

        case 'done':
          if (evt.text) assistantMsg.content = evt.text;
          if (evt.model) assistantMsg.meta.model = evt.model;
          if (evt.provider) assistantMsg.meta.provider = evt.provider;
          if (evt.complexity) assistantMsg.meta.complexity = evt.complexity;
          if (evt.iterations) assistantMsg.meta.iterations = evt.iterations;
          _progressFill.style.width = '100%';
          break;

        case 'error':
          assistantMsg.content += `\n\n**Error:** ${evt.error || evt.message || 'Unknown error'}`;
          break;

        case 'close':
          // Server signals stream end.
          break;
      }
      _updateCurrentMsg(assistantMsg);
    }
  } catch (e) {
    if (e.name !== 'AbortError') {
      assistantMsg.content += `\n\n**Connection error:** ${e.message}`;
      _updateCurrentMsg(assistantMsg);
    }
  }

  _history.push({ role: 'assistant', content: assistantMsg.content });

  const hasError = assistantMsg.tools.some(t => t.status === 'error');
  window.AtlasCompanion?.onAgentDone(!hasError && assistantMsg.content.length > 0);

  set('assistantBusy', false);
  _sendBtn.innerHTML = SVG_SEND;
  setTimeout(() => { _progressEl.style.display = 'none'; }, 1500);
}

function _abort() {
  if (_currentAbort) { _currentAbort(); _currentAbort = null; }
  set('assistantBusy', false);
  _sendBtn.innerHTML = SVG_SEND;
  _progressEl.style.display = 'none';
}

function _renderThread() {
  if (!_threadEl) return;
  _threadEl.innerHTML = _thread.map((msg, i) => {
    if (msg.role === 'user') return _renderUserMsg(msg);
    return _renderAssistantMsg(msg, i);
  }).join('');
  _scrollBottom();
}

function _updateCurrentMsg(msg) {
  const idx = _thread.indexOf(msg);
  if (idx < 0) return;
  const els = _threadEl.querySelectorAll('.msg');
  if (els[idx]) {
    els[idx].outerHTML = _renderAssistantMsg(msg, idx);
  }
  _scrollBottom();
}

function _renderUserMsg(msg) {
  return `<div class="msg msg-user"><div class="msg-bubble">${_escHTML(msg.content)}</div></div>`;
}

function _renderAssistantMsg(msg, idx) {
  let html = '<div class="msg msg-atlas">';
  html += '<div class="msg-atlas-avatar">A</div>';
  html += '<div class="msg-content">';

  if (msg.thinking.length > 0) {
    const thinkText = msg.thinking.join('\n');
    html += `<div class="msg-thinking" onclick="this.classList.toggle('collapsed')">
      <div class="msg-thinking-indicator"><div class="spinner"></div></div>
      <div class="msg-thinking-body">${_escHTML(thinkText)}</div>
    </div>`;
  }

  for (const tool of (msg.tools || [])) {
    const statusClass = tool.status;
    const statusText = tool.status === 'running' ? 'executing...' : tool.status;
    html += `<div class="msg-tool">
      <span class="msg-tool-icon"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polygon points="13 2 3 14 12 14 11 22 21 10 12 10 13 2"/></svg></span>
      <span class="msg-tool-name">${_escHTML(tool.name)}</span>
      <span class="msg-tool-status ${statusClass}">${statusText}</span>
    </div>`;
  }

  if (msg.content) {
    html += `<div class="msg-bubble">${_formatContent(msg.content)}</div>`;
  }

  if (msg.meta && Object.keys(msg.meta).length > 0) {
    html += '<div class="msg-meta">';
    if (msg.meta.model) html += `<span class="meta-chip">${_escHTML(msg.meta.model)}</span>`;
    if (msg.meta.provider) html += `<span class="meta-chip">${_escHTML(msg.meta.provider)}</span>`;
    if (msg.meta.complexity) html += `<span class="meta-chip">${_escHTML(msg.meta.complexity)}</span>`;
    if (msg.meta.iterations) html += `<span class="meta-chip">${msg.meta.iterations} iterations</span>`;
    html += '</div>';
  }

  html += '</div></div>';
  return html;
}

function _formatContent(text) {
  let s = _escHTML(text);
  s = s.replace(/```(\w*)\n([\s\S]*?)```/g, '<pre><code>$2</code></pre>');
  s = s.replace(/`([^`]+)`/g, '<code>$1</code>');
  s = s.replace(/\*\*(.+?)\*\*/g, '<strong>$1</strong>');
  s = s.replace(/\n/g, '<br>');
  return `<p>${s}</p>`;
}

function _escHTML(s) {
  const d = document.createElement('div');
  d.textContent = s || '';
  return d.innerHTML;
}

function _scrollBottom() {
  if (_threadEl) _threadEl.scrollTop = _threadEl.scrollHeight;
}

export function clear() {
  _thread = [];
  _history = [];
  if (_threadEl) _threadEl.innerHTML = '';
}

window.AtlasAssistant = { render, clear };
