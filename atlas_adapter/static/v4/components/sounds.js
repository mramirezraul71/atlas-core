/**
 * ATLAS v4 — Synthetic Sound Engine
 * Web Audio API procedural sounds — no external files needed.
 */
let _ctx = null;
let _enabled = true;
let _volume = 0.15;

function _ensure() {
  if (!_ctx) {
    _ctx = new (window.AudioContext || window.webkitAudioContext)();
  }
  if (_ctx.state === 'suspended') _ctx.resume();
  return _ctx;
}

function _gain(v) {
  const ctx = _ensure();
  const g = ctx.createGain();
  g.gain.value = v * _volume;
  g.connect(ctx.destination);
  return g;
}

export function click() {
  if (!_enabled) return;
  const ctx = _ensure();
  const o = ctx.createOscillator();
  const g = _gain(0.3);
  o.type = 'sine';
  o.frequency.setValueAtTime(800, ctx.currentTime);
  o.frequency.exponentialRampToValueAtTime(1200, ctx.currentTime + 0.03);
  g.gain.exponentialRampToValueAtTime(0.001, ctx.currentTime + 0.08);
  o.connect(g);
  o.start(ctx.currentTime);
  o.stop(ctx.currentTime + 0.08);
}

export function navigate() {
  if (!_enabled) return;
  const ctx = _ensure();
  const o = ctx.createOscillator();
  const g = _gain(0.2);
  o.type = 'sine';
  o.frequency.setValueAtTime(400, ctx.currentTime);
  o.frequency.exponentialRampToValueAtTime(800, ctx.currentTime + 0.15);
  g.gain.exponentialRampToValueAtTime(0.001, ctx.currentTime + 0.2);
  o.connect(g);
  o.start(ctx.currentTime);
  o.stop(ctx.currentTime + 0.2);
}

export function typing() {
  if (!_enabled) return;
  const ctx = _ensure();
  const o = ctx.createOscillator();
  const g = _gain(0.08);
  o.type = 'square';
  o.frequency.value = 600 + Math.random() * 200;
  g.gain.exponentialRampToValueAtTime(0.001, ctx.currentTime + 0.025);
  o.connect(g);
  o.start(ctx.currentTime);
  o.stop(ctx.currentTime + 0.025);
}

export function thinking() {
  if (!_enabled) return;
  const ctx = _ensure();
  const o1 = ctx.createOscillator();
  const o2 = ctx.createOscillator();
  const g = _gain(0.12);
  o1.type = 'sine';
  o2.type = 'sine';
  o1.frequency.value = 220;
  o2.frequency.value = 224;
  o1.connect(g);
  o2.connect(g);
  g.gain.exponentialRampToValueAtTime(0.001, ctx.currentTime + 0.6);
  o1.start(ctx.currentTime);
  o2.start(ctx.currentTime);
  o1.stop(ctx.currentTime + 0.6);
  o2.stop(ctx.currentTime + 0.6);
}

export function success() {
  if (!_enabled) return;
  const ctx = _ensure();
  const notes = [523, 659, 784];
  notes.forEach((freq, i) => {
    const o = ctx.createOscillator();
    const g = _gain(0.2);
    o.type = 'sine';
    o.frequency.value = freq;
    g.gain.setValueAtTime(0.2 * _volume, ctx.currentTime + i * 0.12);
    g.gain.exponentialRampToValueAtTime(0.001, ctx.currentTime + i * 0.12 + 0.2);
    o.connect(g);
    o.start(ctx.currentTime + i * 0.12);
    o.stop(ctx.currentTime + i * 0.12 + 0.2);
  });
}

export function error() {
  if (!_enabled) return;
  const ctx = _ensure();
  const o = ctx.createOscillator();
  const g = _gain(0.2);
  o.type = 'sawtooth';
  o.frequency.setValueAtTime(300, ctx.currentTime);
  o.frequency.exponentialRampToValueAtTime(150, ctx.currentTime + 0.3);
  g.gain.exponentialRampToValueAtTime(0.001, ctx.currentTime + 0.3);
  o.connect(g);
  o.start(ctx.currentTime);
  o.stop(ctx.currentTime + 0.3);
}

export function greeting() {
  if (!_enabled) return;
  const ctx = _ensure();
  const melody = [392, 440, 523, 659, 784];
  melody.forEach((freq, i) => {
    const o = ctx.createOscillator();
    const g = _gain(0.15);
    o.type = 'sine';
    o.frequency.value = freq;
    g.gain.setValueAtTime(0.15 * _volume, ctx.currentTime + i * 0.1);
    g.gain.exponentialRampToValueAtTime(0.001, ctx.currentTime + i * 0.1 + 0.15);
    o.connect(g);
    o.start(ctx.currentTime + i * 0.1);
    o.stop(ctx.currentTime + i * 0.1 + 0.15);
  });
}

export function hover() {
  if (!_enabled) return;
  const ctx = _ensure();
  const o = ctx.createOscillator();
  const g = _gain(0.06);
  o.type = 'sine';
  o.frequency.value = 1000;
  g.gain.exponentialRampToValueAtTime(0.001, ctx.currentTime + 0.04);
  o.connect(g);
  o.start(ctx.currentTime);
  o.stop(ctx.currentTime + 0.04);
}

export function toggle(enabled) { _enabled = enabled; }
export function setVolume(v) { _volume = Math.max(0, Math.min(1, v)); }
export function isEnabled() { return _enabled; }

window.AtlasSounds = { click, navigate, typing, thinking, success, error, greeting, hover, toggle, setVolume, isEnabled };
