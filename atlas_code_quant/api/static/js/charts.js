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

// CDN opcional (jsdelivr): si falla la red, el dashboard sigue cargando KPIs vía REST/WS.
const CHART_JS = typeof Chart !== 'undefined' ? Chart : null;
const LW = typeof LightweightCharts !== 'undefined' ? LightweightCharts : null;

if (CHART_JS) {
  CHART_JS.defaults.color = COLORS.text;
  CHART_JS.defaults.borderColor = COLORS.grid;
  CHART_JS.defaults.backgroundColor = 'transparent';
  CHART_JS.defaults.font.family = "'JetBrains Mono','Fira Code','Consolas',monospace";
  CHART_JS.defaults.font.size = 11;
}

// ── Lightweight Charts (TradingView) ──────────────────────────────
function makeLWChart(containerId, opts = {}) {
  if (!LW) return null;
  const container = document.getElementById(containerId);
  if (!container) return null;
  container.innerHTML = '';

  const chart = LW.createChart(container, {
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
    crosshair: { mode: LW.CrosshairMode.Normal },
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

function normalizeSeriesTime(rawTime) {
  if (rawTime == null) return null;
  if (typeof rawTime === 'string') {
    const parsed = Date.parse(rawTime);
    if (Number.isFinite(parsed)) {
      return Math.floor(parsed / 1000);
    }
  }
  const numeric = Number(rawTime);
  if (!Number.isFinite(numeric)) return null;
  if (numeric > 9999999999) {
    return Math.floor(numeric / 1000);
  }
  return Math.floor(numeric);
}

function normalizeSeriesData(data = []) {
  if (!Array.isArray(data)) return [];
  const byTime = new Map();
  data.forEach((point) => {
    const time = normalizeSeriesTime(point?.time);
    const value = Number(point?.value);
    if (time == null || !Number.isFinite(value)) return;
    byTime.set(time, { time, value });
  });
  return Array.from(byTime.values()).sort((a, b) => a.time - b.time);
}

function normalizeNumericArray(values = []) {
  if (!Array.isArray(values)) return [];
  return values
    .map((value) => Number(value))
    .filter((value) => Number.isFinite(value));
}

function safeSetSeriesData(series, data = []) {
  if (!series) return [];
  const normalized = normalizeSeriesData(data);
  series.setData(normalized);
  return normalized;
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
  safeSetSeriesData(series, data);
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
  safeSetSeriesData(series, data);
  return { chart, series };
}

// ── PnL Distribution (Chart.js bar) ──────────────────────────────
function renderPnlDistChart(canvasId, pnls = []) {
  const ctx = document.getElementById(canvasId);
  if (!ctx) return;
  if (!CHART_JS) return;

  // Build histogram bins
  const normalizedPnls = normalizeNumericArray(pnls);
  if (!normalizedPnls.length) return;
  const min = Math.min(...normalizedPnls), max = Math.max(...normalizedPnls);
  const bins = 20;
  const width = (max - min) / bins || 1;
  const counts = new Array(bins).fill(0);
  const labels = [];
  for (let i = 0; i < bins; i++) labels.push((min + i * width).toFixed(1));
  normalizedPnls.forEach(v => {
    const idx = Math.min(Math.floor((v - min) / width), bins - 1);
    counts[idx]++;
  });

  const colors = counts.map((_, i) => {
    const center = min + (i + 0.5) * width;
    return center >= 0 ? 'rgba(72,187,120,0.7)' : 'rgba(245,101,101,0.7)';
  });

  if (ctx._chartInstance) ctx._chartInstance.destroy();
  ctx._chartInstance = new CHART_JS(ctx, {
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
  if (!CHART_JS) return;

  if (!mcData) {
    // Placeholder
    if (ctx._chartInstance) ctx._chartInstance.destroy();
    ctx._chartInstance = new CHART_JS(ctx, {
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
  ctx._chartInstance = new CHART_JS(ctx, {
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
  if (!CHART_JS) return;
  const values = trades.map(t => t.pnl || 0);
  const colors = values.map(v => v >= 0 ? 'rgba(72,187,120,0.75)' : 'rgba(245,101,101,0.75)');

  if (ctx._chartInstance) ctx._chartInstance.destroy();
  ctx._chartInstance = new CHART_JS(ctx, {
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
  if (!CHART_JS) return;
  let cum = 0;
  const labels = [], data = [];
  entries.forEach(e => {
    cum += e.pnl || 0;
    labels.push(e.closed_at ? e.closed_at.slice(0, 10) : '');
    data.push(parseFloat(cum.toFixed(2)));
  });

  if (ctx._chartInstance) ctx._chartInstance.destroy();
  ctx._chartInstance = new CHART_JS(ctx, {
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
  if (!CHART_JS) return;
  const byStrat = {};
  entries.forEach(e => {
    const k = e.strategy || 'unknown';
    byStrat[k] = (byStrat[k] || 0) + (e.pnl || 0);
  });
  const labels = Object.keys(byStrat);
  const values = labels.map(k => parseFloat(byStrat[k].toFixed(2)));

  if (ctx._chartInstance) ctx._chartInstance.destroy();
  ctx._chartInstance = new CHART_JS(ctx, {
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
  if (!CHART_JS) return;
  const labels = features.map((_, i) => `f${i}`);

  if (ctx._chartInstance) ctx._chartInstance.destroy();
  ctx._chartInstance = new CHART_JS(ctx, {
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

// ── Performance Heatmap (día × hora) ─────────────────────────────
function renderHeatmapChart(canvasId, heatmapData = []) {
  const ctx = document.getElementById(canvasId);
  if (!ctx) return;
  if (!CHART_JS) return;

  const days   = ['Lun', 'Mar', 'Mié', 'Jue', 'Vie', 'Sáb', 'Dom'];
  const hours  = Array.from({length: 24}, (_, i) => `${i}h`);
  const matrix = Array.from({length: 7}, () => new Array(24).fill(null));

  heatmapData.forEach(pt => {
    const d = ((pt.weekday || 0) + 6) % 7; // 0=Mon
    const h = pt.hour || 0;
    if (matrix[d][h] === null) matrix[d][h] = 0;
    matrix[d][h] += pt.success_rate_pct || 0;
  });

  const datasets = days.map((day, di) => ({
    label: day,
    data: hours.map((_, hi) => ({
      x: hi,
      y: di,
      v: matrix[di][hi],
    })),
    backgroundColor: (ctx) => {
      const v = ctx.raw?.v;
      if (v === null || v === undefined) return 'rgba(255,255,255,0.03)';
      const norm = Math.max(0, Math.min(1, v / 100));
      return norm >= 0.5
        ? `rgba(72,187,120,${0.1 + norm * 0.7})`
        : `rgba(245,101,101,${0.1 + (1 - norm) * 0.7})`;
    },
    borderWidth: 1,
    borderColor: '#1f2433',
    borderRadius: 2,
    width: ({chart}) => (chart.chartArea?.width || 400) / 24 - 2,
    height: ({chart}) => (chart.chartArea?.height || 200) / 7 - 2,
  }));

  if (ctx._chartInstance) ctx._chartInstance.destroy();
  ctx._chartInstance = new CHART_JS(ctx, {
    type: 'bubble',
    data: { datasets },
    options: {
      responsive: true, maintainAspectRatio: false,
      plugins: {
        legend: { display: false },
        tooltip: {
          callbacks: {
            label: (item) => {
              const v = item.raw.v;
              return v !== null ? `${days[item.raw.y]} ${hours[item.raw.x]}: ${v?.toFixed(1)}% win` : 'Sin datos';
            }
          }
        }
      },
      scales: {
        x: {
          type: 'linear', min: -0.5, max: 23.5,
          ticks: { stepSize: 1, callback: v => `${v}h`, maxTicksLimit: 12 },
          grid: { color: COLORS.grid },
        },
        y: {
          type: 'linear', min: -0.5, max: 6.5,
          ticks: { stepSize: 1, callback: v => days[v] || '' },
          grid: { color: COLORS.grid },
          reverse: false,
        },
      },
    },
  });
}

// ── Rolling Sharpe chart ──────────────────────────────────────────
function renderRollingSharpeChart(canvasId, equityCurve = []) {
  const ctx = document.getElementById(canvasId);
  if (!ctx) return;
  if (!CHART_JS) return;

  // Compute rolling log-returns and rolling Sharpe (20-period)
  const WINDOW = 20;
  if (equityCurve.length < WINDOW + 1) return;

  const values = equityCurve.map(p => p.value);
  const labels = equityCurve.map(p => {
    const d = new Date(p.time * 1000);
    return `${d.getMonth()+1}/${d.getDate()}`;
  });

  const logRets = [];
  for (let i = 1; i < values.length; i++) {
    logRets.push(values[i] > 0 && values[i-1] > 0 ? Math.log(values[i] / values[i-1]) : 0);
  }

  const sharpeRolling = [];
  const sharpeLabels = [];
  for (let i = WINDOW; i <= logRets.length; i++) {
    const slice = logRets.slice(i - WINDOW, i);
    const mean = slice.reduce((a,b) => a+b, 0) / WINDOW;
    const std  = Math.sqrt(slice.reduce((s, x) => s + (x - mean) ** 2, 0) / (WINDOW - 1));
    sharpeRolling.push(std > 1e-9 ? parseFloat((mean / std * Math.sqrt(252)).toFixed(3)) : 0);
    sharpeLabels.push(labels[i] || '');
  }

  if (ctx._chartInstance) ctx._chartInstance.destroy();
  ctx._chartInstance = new CHART_JS(ctx, {
    type: 'line',
    data: {
      labels: sharpeLabels,
      datasets: [{
        label: 'Sharpe Rolling 20',
        data: sharpeRolling,
        borderColor: COLORS.yellow,
        backgroundColor: 'rgba(246,201,14,0.07)',
        fill: true, borderWidth: 2, pointRadius: 0, tension: 0.3,
      }]
    },
    options: {
      responsive: true, maintainAspectRatio: false,
      plugins: {
        legend: { display: false },
        annotation: {
          annotations: {
            zeroline: { type: 'line', yMin: 0, yMax: 0, borderColor: COLORS.text, borderWidth: 1, borderDash: [4,4] },
            sharpe1:  { type: 'line', yMin: 1, yMax: 1, borderColor: COLORS.green, borderWidth: 1, borderDash: [4,4] },
          }
        }
      },
      scales: {
        x: { grid: { color: COLORS.grid }, ticks: { maxTicksLimit: 8 } },
        y: { grid: { color: COLORS.grid } },
      },
    },
  });
}

window.QuantCharts = {
  normalizeSeriesData,
  normalizeNumericArray,
  safeSetSeriesData,
  renderEquityChart,
  renderDrawdownChart,
  renderPnlDistChart,
  renderMonteCarloChart,
  renderStreakChart,
  renderJournalPnlChart,
  renderJournalStratChart,
  renderVisualFeaturesChart,
  renderHeatmapChart,
  renderRollingSharpeChart,
};
