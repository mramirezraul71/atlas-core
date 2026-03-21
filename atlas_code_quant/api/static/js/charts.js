/* ================================================================
   Atlas Code-Quant — Charts Module
   TradingView Lightweight Charts + Chart.js wrappers
   ================================================================ */

// ── Paleta ────────────────────────────────────────────────────────
const COLORS = {
  accent:  '#00d4aa',
  red:     '#f56565',
  green:   '#48bb78',
  yellow:  '#f6c90e',
  text:    '#8892a4',
  grid:    '#1f2433',
  bg:      '#13161d',
  border:  '#1f2433',
};

// Chart.js defaults
Chart.defaults.color          = COLORS.text;
Chart.defaults.borderColor    = COLORS.grid;
Chart.defaults.backgroundColor = 'transparent';
Chart.defaults.font.family    = "'JetBrains Mono','Fira Code','Consolas',monospace";
Chart.defaults.font.size      = 11;

// ── Lightweight Charts (TradingView) ──────────────────────────────
function makeLWChart(containerId, opts = {}) {
  const container = document.getElementById(containerId);
  if (!container) return null;
  container.innerHTML = '';

  const chart = LightweightCharts.createChart(container, {
    width:  container.clientWidth  || 400,
    height: container.clientHeight || 180,
    layout: {
      background: { color: COLORS.bg },
      textColor:  COLORS.text,
    },
    grid: {
      vertLines:  { color: COLORS.grid },
      horzLines:  { color: COLORS.grid },
    },
    crosshair: { mode: LightweightCharts.CrosshairMode.Normal },
    rightPriceScale: {
      borderColor: COLORS.border,
      textColor:   COLORS.text,
    },
    timeScale: {
      borderColor:      COLORS.border,
      timeVisible:      true,
      secondsVisible:   false,
    },
    ...opts,
  });

  // Auto-resize
  const ro = new ResizeObserver(() => {
    chart.applyOptions({ width: container.clientWidth, height: container.clientHeight });
  });
  ro.observe(container);

  return chart;
}

// ── Equity curve ──────────────────────────────────────────────────
function renderEquityChart(containerId, data = []) {
  // data: [{time: unix_sec, value: number}, ...]
  const chart  = makeLWChart(containerId);
  if (!chart) return;
  const series = chart.addAreaSeries({
    lineColor:          COLORS.accent,
    topColor:           'rgba(0,212,170,0.18)',
    bottomColor:        'rgba(0,212,170,0.00)',
    lineWidth:          2,
    crosshairMarkerVisible: true,
    priceFormat: { type: 'price', precision: 2, minMove: 0.01 },
  });
  if (data.length) series.setData(data);
  return { chart, series };
}

// ── Drawdown chart ─────────────────────────────────────────────────
function renderDrawdownChart(containerId, data = []) {
  const chart  = makeLWChart(containerId);
  if (!chart) return;
  const series = chart.addAreaSeries({
    lineColor:    COLORS.red,
    topColor:     'rgba(245,101,101,0.15)',
    bottomColor:  'rgba(245,101,101,0.00)',
    lineWidth:    2,
    priceFormat:  { type: 'percent' },
    invertFilledArea: true,
  });
  if (data.length) series.setData(data);
  return { chart, series };
}

// ── PnL Distribution (Chart.js bar) ──────────────────────────────
function renderPnlDistChart(canvasId, pnls = []) {
  const ctx = document.getElementById(canvasId);
  if (!ctx) return;

  // Build histogram bins
  if (!pnls.length) return;
  const min = Math.min(...pnls), max = Math.max(...pnls);
  const bins = 20;
  const width = (max - min) / bins || 1;
  const counts = new Array(bins).fill(0);
  const labels = [];
  for (let i = 0; i < bins; i++) labels.push((min + i * width).toFixed(1));
  pnls.forEach(v => {
    const idx = Math.min(Math.floor((v - min) / width), bins - 1);
    counts[idx]++;
  });

  const colors = counts.map((_, i) => {
    const center = min + (i + 0.5) * width;
    return center >= 0 ? 'rgba(72,187,120,0.7)' : 'rgba(245,101,101,0.7)';
  });

  if (ctx._chartInstance) ctx._chartInstance.destroy();
  ctx._chartInstance = new Chart(ctx, {
    type: 'bar',
    data: { labels, datasets: [{ data: counts, backgroundColor: colors, borderWidth: 0 }] },
    options: {
      responsive: true, maintainAspectRatio: false,
      plugins: { legend: { display: false } },
      scales: {
        x: { grid: { color: COLORS.grid }, ticks: { maxTicksLimit: 8 } },
        y: { grid: { color: COLORS.grid } },
      },
    },
  });
}

// ── Monte Carlo fan chart ─────────────────────────────────────────
function renderMonteCarloChart(canvasId, mcData = null) {
  const ctx = document.getElementById(canvasId);
  if (!ctx) return;

  if (!mcData) {
    // Placeholder
    if (ctx._chartInstance) ctx._chartInstance.destroy();
    ctx._chartInstance = new Chart(ctx, {
      type: 'line',
      data: { labels: [], datasets: [] },
      options: {
        responsive: true, maintainAspectRatio: false,
        plugins: { legend: { display: false },
          title: { display: true, text: 'Sin datos — ejecuta un backtest', color: COLORS.text } },
      },
    });
    return;
  }

  const { median, p05, p95, labels } = mcData;
  if (ctx._chartInstance) ctx._chartInstance.destroy();
  ctx._chartInstance = new Chart(ctx, {
    type: 'line',
    data: {
      labels,
      datasets: [
        { label: 'P95', data: p95, borderColor: 'rgba(0,212,170,0.3)', borderWidth: 1, fill: false, pointRadius: 0 },
        { label: 'Mediana', data: median, borderColor: COLORS.accent, borderWidth: 2, fill: false, pointRadius: 0 },
        { label: 'P05', data: p05, borderColor: 'rgba(245,101,101,0.3)', borderWidth: 1, fill: false, pointRadius: 0 },
      ]
    },
    options: {
      responsive: true, maintainAspectRatio: false,
      plugins: { legend: { display: true, labels: { boxWidth: 10 } } },
      scales: {
        x: { grid: { color: COLORS.grid }, ticks: { maxTicksLimit: 6 } },
        y: { grid: { color: COLORS.grid } },
      },
    },
  });
}

// ── Win/Loss streak bar ───────────────────────────────────────────
function renderStreakChart(canvasId, trades = []) {
  const ctx = document.getElementById(canvasId);
  if (!ctx) return;
  const values = trades.map(t => t.pnl || 0);
  const colors = values.map(v => v >= 0 ? 'rgba(72,187,120,0.75)' : 'rgba(245,101,101,0.75)');

  if (ctx._chartInstance) ctx._chartInstance.destroy();
  ctx._chartInstance = new Chart(ctx, {
    type: 'bar',
    data: {
      labels: values.map((_, i) => `#${i + 1}`),
      datasets: [{ data: values, backgroundColor: colors, borderWidth: 0 }]
    },
    options: {
      responsive: true, maintainAspectRatio: false,
      plugins: { legend: { display: false } },
      scales: {
        x: { grid: { display: false }, ticks: { maxTicksLimit: 12 } },
        y: { grid: { color: COLORS.grid } },
      },
    },
  });
}

// ── Journal cumulative PnL line ───────────────────────────────────
function renderJournalPnlChart(canvasId, entries = []) {
  const ctx = document.getElementById(canvasId);
  if (!ctx) return;
  let cum = 0;
  const labels = [], data = [];
  entries.forEach(e => {
    cum += e.pnl || 0;
    labels.push(e.closed_at ? e.closed_at.slice(0, 10) : '');
    data.push(parseFloat(cum.toFixed(2)));
  });

  if (ctx._chartInstance) ctx._chartInstance.destroy();
  ctx._chartInstance = new Chart(ctx, {
    type: 'line',
    data: {
      labels,
      datasets: [{
        data,
        borderColor: COLORS.accent,
        backgroundColor: 'rgba(0,212,170,0.07)',
        fill: true,
        borderWidth: 2,
        pointRadius: 0,
        tension: 0.3,
      }]
    },
    options: {
      responsive: true, maintainAspectRatio: false,
      plugins: { legend: { display: false } },
      scales: {
        x: { grid: { color: COLORS.grid }, ticks: { maxTicksLimit: 8 } },
        y: { grid: { color: COLORS.grid } },
      },
    },
  });
}

// ── Journal by strategy bar ───────────────────────────────────────
function renderJournalStratChart(canvasId, entries = []) {
  const ctx = document.getElementById(canvasId);
  if (!ctx) return;
  const byStrat = {};
  entries.forEach(e => {
    const k = e.strategy || 'unknown';
    byStrat[k] = (byStrat[k] || 0) + (e.pnl || 0);
  });
  const labels = Object.keys(byStrat);
  const values = labels.map(k => parseFloat(byStrat[k].toFixed(2)));

  if (ctx._chartInstance) ctx._chartInstance.destroy();
  ctx._chartInstance = new Chart(ctx, {
    type: 'bar',
    data: {
      labels,
      datasets: [{
        data: values,
        backgroundColor: values.map(v => v >= 0 ? 'rgba(72,187,120,0.7)' : 'rgba(245,101,101,0.7)'),
        borderWidth: 0,
      }]
    },
    options: {
      responsive: true, maintainAspectRatio: false,
      indexAxis: 'y',
      plugins: { legend: { display: false } },
      scales: {
        x: { grid: { color: COLORS.grid } },
        y: { grid: { display: false } },
      },
    },
  });
}

// ── Visual feature radar ──────────────────────────────────────────
function renderVisualFeaturesChart(canvasId, features = []) {
  const ctx = document.getElementById(canvasId);
  if (!ctx) return;
  const labels = features.map((_, i) => `f${i}`);

  if (ctx._chartInstance) ctx._chartInstance.destroy();
  ctx._chartInstance = new Chart(ctx, {
    type: 'bar',
    data: {
      labels,
      datasets: [{
        data: features,
        backgroundColor: 'rgba(0,212,170,0.5)',
        borderColor: COLORS.accent,
        borderWidth: 1,
      }]
    },
    options: {
      responsive: true, maintainAspectRatio: false,
      plugins: { legend: { display: false } },
      scales: {
        x: { grid: { display: false }, ticks: { font: { size: 9 } } },
        y: { grid: { color: COLORS.grid }, min: 0, max: 1 },
      },
    },
  });
}

window.QuantCharts = {
  renderEquityChart,
  renderDrawdownChart,
  renderPnlDistChart,
  renderMonteCarloChart,
  renderStreakChart,
  renderJournalPnlChart,
  renderJournalStratChart,
  renderVisualFeaturesChart,
};
