/* ================================================================
   Atlas Code-Quant — Advanced Analytics Charts Module
   6 professional charts + robot status widget
   Dependencies: Chart.js 4.x (already loaded), QuantAPI, QuantCharts
   ================================================================ */

const AtlasCharts = (() => {
  'use strict';

  // ── Design tokens (v4.2) ────────────────────────────────────────
  const C = {
    bg:       '#0d0f14',
    panel:    '#13161d',
    grid:     'rgba(255,255,255,0.05)',
    text:     '#e2e8f0',
    textSec:  '#94a3b8',
    textMut:  '#4a5568',
    cyan:     '#06b6d4',
    cyanDim:  'rgba(6,182,212,0.15)',
    rose:     '#f43f5e',
    roseDim:  'rgba(244,63,94,0.15)',
    green:    '#48bb78',
    greenDim: 'rgba(72,187,120,0.12)',
    red:      '#f56565',
    redDim:   'rgba(245,101,101,0.12)',
    yellow:   '#f6c90e',
    accent:   '#00d4aa',
    border:   '#1f2433',
  };

  const METHOD_COLORS = [
    '#06b6d4', '#f59e0b', '#8b5cf6', '#ec4899', '#10b981',
    '#f43f5e', '#3b82f6', '#84cc16', '#14b8a6', '#e879f9',
  ];

  const CHART_JS = typeof Chart !== 'undefined' ? Chart : null;

  // ── State ───────────────────────────────────────────────────────
  let _cache = { chartData: null, icSummary: null, lastFetch: 0 };
  const _instances = {};
  let _pollTimer = null;
  let _visible = false;

  // ── Helpers ─────────────────────────────────────────────────────
  function _destroy(key) {
    if (_instances[key]) {
      _instances[key].destroy();
      delete _instances[key];
    }
  }

  function _chartOpts(overrides = {}) {
    return {
      responsive: true,
      maintainAspectRatio: false,
      animation: { duration: 300 },
      plugins: {
        legend: { display: false },
        ...overrides.plugins,
      },
      scales: {
        x: {
          grid: { color: C.grid, drawBorder: false },
          ticks: { color: C.textSec, font: { size: 10, family: "'JetBrains Mono', monospace" }, maxTicksLimit: 8 },
          ...overrides.xScale,
        },
        y: {
          grid: { color: C.grid, drawBorder: false },
          ticks: { color: C.textSec, font: { size: 10, family: "'JetBrains Mono', monospace" } },
          ...overrides.yScale,
        },
        ...overrides.scales,
      },
      ...overrides.root,
    };
  }

  function _ts(label, el) {
    if (!el) return;
    el.textContent = `Actualizado: ${new Date().toLocaleTimeString()}`;
  }

  // ── Data fetching ───────────────────────────────────────────────
  async function _fetchData(force = false) {
    const now = Date.now();
    if (!force && _cache.chartData && now - _cache.lastFetch < 25000) return _cache;
    try {
      const scope = window.QUANT_ACCOUNT_SCOPE || 'paper';
      const [chartResp, icResp] = await Promise.all([
        QuantAPI.journalChartData(500, scope).catch(() => null),
        QuantAPI.learningICSummary().catch(() => null),
      ]);
      _cache.chartData = chartResp?.ok ? chartResp.data : null;
      _cache.icSummary = icResp?.ok ? icResp.data : null;
      _cache.lastFetch = now;
    } catch (_) {}
    return _cache;
  }

  // ── 1. EQUITY CURVE ─────────────────────────────────────────────
  function renderEquityCurve(containerId, trades) {
    const canvas = document.getElementById(containerId);
    if (!canvas || !trades?.length) return;
    _destroy(containerId);

    const labels = trades.map(t => (t.exit_time || '').slice(5, 10) || `#${t.n}`);
    const equity = trades.map(t => t.equity);
    const dd = trades.map(t => t.drawdown_pct);

    // Find peak and max drawdown annotations
    let maxDdIdx = 0, maxEqIdx = 0;
    trades.forEach((t, i) => {
      if (t.drawdown_pct < trades[maxDdIdx].drawdown_pct) maxDdIdx = i;
      if (t.equity > trades[maxEqIdx].equity) maxEqIdx = i;
    });

    _instances[containerId] = new CHART_JS(canvas, {
      type: 'line',
      data: {
        labels,
        datasets: [
          {
            label: 'Equity',
            data: equity,
            borderColor: C.cyan,
            backgroundColor: C.cyanDim,
            fill: true,
            borderWidth: 2,
            pointRadius: 0,
            pointHitRadius: 6,
            tension: 0.25,
            order: 1,
          },
          {
            label: 'Drawdown %',
            data: dd,
            borderColor: C.rose,
            backgroundColor: C.roseDim,
            fill: true,
            borderWidth: 1.5,
            pointRadius: 0,
            tension: 0.25,
            yAxisID: 'yDD',
            order: 2,
          },
        ],
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: { duration: 300 },
        interaction: { mode: 'index', intersect: false },
        plugins: {
          legend: { display: true, labels: { color: C.textSec, boxWidth: 10, font: { size: 10 } } },
          tooltip: {
            backgroundColor: '#1a1d27',
            titleColor: C.text,
            bodyColor: C.textSec,
            borderColor: C.border,
            borderWidth: 1,
            callbacks: {
              afterBody: (items) => {
                const i = items[0]?.dataIndex;
                if (i == null || !trades[i]) return '';
                const t = trades[i];
                return `Trades: ${t.n}  |  DD: ${t.drawdown_pct}%`;
              },
            },
          },
        },
        scales: {
          x: { grid: { color: C.grid, drawBorder: false }, ticks: { color: C.textSec, font: { size: 10 }, maxTicksLimit: 10 } },
          y: { grid: { color: C.grid, drawBorder: false }, ticks: { color: C.cyan, font: { size: 10, family: "'JetBrains Mono', monospace" } }, position: 'left' },
          yDD: { grid: { display: false }, ticks: { color: C.rose, font: { size: 10 }, callback: v => `${v}%` }, position: 'right', reverse: true },
        },
      },
    });
  }

  // ── 2. LEARNING CURVE (IC rolling) ──────────────────────────────
  function renderLearningCurve(containerId, icData) {
    const canvas = document.getElementById(containerId);
    if (!canvas) return;
    _destroy(containerId);

    // icData expected: { methods: { method_name: { ic, n_signals, ... }, ... }, per_method: {...} }
    // If no data, show placeholder
    const methods = icData?.by_method || icData?.methods || icData?.per_method || {};
    const methodNames = Object.keys(methods);

    if (!methodNames.length) {
      _instances[containerId] = new CHART_JS(canvas, {
        type: 'line',
        data: { labels: ['0'], datasets: [{ data: [0], borderColor: C.textMut, borderDash: [4, 4], pointRadius: 0 }] },
        options: _chartOpts({
          plugins: {
            legend: { display: false },
            title: { display: true, text: 'IC Tracker sin datos — se llena con trades cerrados', color: C.textSec, font: { size: 11 } },
          },
        }),
      });
      return;
    }

    // Build datasets: one line per method with IC value
    const datasets = methodNames.map((name, idx) => {
      const m = methods[name];
      const ic = typeof m === 'object' ? (m.ic ?? m.spearman_ic ?? 0) : (m || 0);
      const n = typeof m === 'object' ? (m.n_signals ?? m.sample ?? 1) : 1;
      return {
        label: name,
        data: [parseFloat(ic) || 0],
        borderColor: METHOD_COLORS[idx % METHOD_COLORS.length],
        backgroundColor: 'transparent',
        borderWidth: 2,
        pointRadius: 4,
        pointBackgroundColor: METHOD_COLORS[idx % METHOD_COLORS.length],
      };
    });

    _instances[containerId] = new CHART_JS(canvas, {
      type: 'bar',
      data: { labels: methodNames, datasets: [{
        data: methodNames.map(name => {
          const m = methods[name];
          return parseFloat(typeof m === 'object' ? (m.ic ?? m.spearman_ic ?? 0) : (m || 0)) || 0;
        }),
        backgroundColor: methodNames.map((_, i) => METHOD_COLORS[i % METHOD_COLORS.length] + '99'),
        borderColor: methodNames.map((_, i) => METHOD_COLORS[i % METHOD_COLORS.length]),
        borderWidth: 1,
      }] },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: { duration: 300 },
        indexAxis: 'y',
        plugins: {
          legend: { display: false },
          tooltip: {
            backgroundColor: '#1a1d27',
            titleColor: C.text,
            bodyColor: C.textSec,
            callbacks: {
              label: (item) => {
                const name = methodNames[item.dataIndex];
                const m = methods[name];
                const n = typeof m === 'object' ? (m.n_signals ?? m.sample ?? '--') : '--';
                return `IC: ${item.formattedValue}  |  n=${n}`;
              },
            },
          },
          annotation: {
            annotations: {
              threshold: {
                type: 'line',
                xMin: 0.05, xMax: 0.05,
                borderColor: C.yellow,
                borderWidth: 1,
                borderDash: [4, 4],
                label: { display: true, content: 'IC=0.05', color: C.yellow, font: { size: 9 }, position: 'start' },
              },
            },
          },
        },
        scales: {
          x: { grid: { color: C.grid, drawBorder: false }, ticks: { color: C.textSec, font: { size: 10 } }, title: { display: true, text: 'IC (Spearman)', color: C.textMut, font: { size: 10 } } },
          y: { grid: { display: false }, ticks: { color: C.text, font: { size: 10 } } },
        },
      },
    });
  }

  // ── 3. R-MULTIPLE DISTRIBUTION ──────────────────────────────────
  function renderRMultiple(containerId, trades) {
    const canvas = document.getElementById(containerId);
    if (!canvas || !trades?.length) return;
    _destroy(containerId);

    const rValues = trades.map(t => t.r_multiple || 0).filter(r => r !== 0);
    if (!rValues.length) return;

    // Build histogram bins from -5R to +5R
    const BINS = 21;
    const MIN_R = -5, MAX_R = 5;
    const binW = (MAX_R - MIN_R) / BINS;
    const counts = new Array(BINS).fill(0);
    const labels = [];
    for (let i = 0; i < BINS; i++) {
      const lo = MIN_R + i * binW;
      labels.push(`${lo.toFixed(1)}R`);
    }

    rValues.forEach(r => {
      const clamped = Math.max(MIN_R, Math.min(MAX_R - 0.001, r));
      const idx = Math.floor((clamped - MIN_R) / binW);
      counts[Math.min(idx, BINS - 1)]++;
    });

    const expectancy = rValues.reduce((s, v) => s + v, 0) / rValues.length;

    const colors = counts.map((_, i) => {
      const center = MIN_R + (i + 0.5) * binW;
      if (center < -1) return 'rgba(244,63,94,0.8)';
      if (center < 0)  return 'rgba(244,63,94,0.5)';
      if (center < 1)  return 'rgba(72,187,120,0.5)';
      return 'rgba(72,187,120,0.8)';
    });

    _instances[containerId] = new CHART_JS(canvas, {
      type: 'bar',
      data: {
        labels,
        datasets: [{ data: counts, backgroundColor: colors, borderWidth: 0, borderRadius: 2 }],
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: { duration: 300 },
        plugins: {
          legend: { display: false },
          tooltip: { backgroundColor: '#1a1d27', titleColor: C.text, bodyColor: C.textSec },
          annotation: {
            annotations: {
              expectancy: {
                type: 'line',
                xMin: ((expectancy - MIN_R) / binW) - 0.5,
                xMax: ((expectancy - MIN_R) / binW) - 0.5,
                borderColor: C.yellow,
                borderWidth: 2,
                borderDash: [6, 3],
                label: {
                  display: true,
                  content: `Expectancy: ${expectancy.toFixed(2)}R`,
                  color: C.yellow,
                  backgroundColor: 'rgba(0,0,0,0.7)',
                  font: { size: 11, weight: 'bold', family: "'JetBrains Mono', monospace" },
                  position: 'start',
                },
              },
            },
          },
        },
        scales: {
          x: { grid: { display: false }, ticks: { color: C.textSec, font: { size: 9 }, maxTicksLimit: 11 } },
          y: { grid: { color: C.grid, drawBorder: false }, ticks: { color: C.textSec, font: { size: 10 } } },
        },
      },
    });
  }

  // ── 4. IC HEATMAP ───────────────────────────────────────────────
  function renderICHeatmap(containerId, icData) {
    const canvas = document.getElementById(containerId);
    if (!canvas) return;
    _destroy(containerId);

    const methods = icData?.by_method || icData?.methods || icData?.per_method || {};
    const methodNames = Object.keys(methods);

    if (!methodNames.length) {
      _instances[containerId] = new CHART_JS(canvas, {
        type: 'bar',
        data: { labels: [], datasets: [] },
        options: _chartOpts({
          plugins: { title: { display: true, text: 'Sin datos de IC', color: C.textSec, font: { size: 11 } } },
        }),
      });
      return;
    }

    // Single-period heatmap: methods x current IC
    const periods = ['Actual'];
    const dataPoints = [];
    methodNames.forEach((name, yi) => {
      const m = methods[name];
      const ic = parseFloat(typeof m === 'object' ? (m.ic ?? m.spearman_ic ?? 0) : (m || 0)) || 0;
      const n = typeof m === 'object' ? (m.n_signals ?? m.sample ?? 0) : 0;
      const wr = typeof m === 'object' ? (m.win_rate ?? 0) : 0;
      dataPoints.push({ x: 0, y: yi, v: ic, n, wr, name });
    });

    function icColor(v) {
      if (v == null) return 'rgba(255,255,255,0.03)';
      if (v >= 0.05) return `rgba(72,187,120,${0.3 + Math.min(v, 0.3) * 2})`;
      if (v >= 0.02) return `rgba(246,201,14,${0.3 + v * 5})`;
      if (v > -0.02) return 'rgba(255,255,255,0.06)';
      return `rgba(244,63,94,${0.3 + Math.min(Math.abs(v), 0.3) * 2})`;
    }

    _instances[containerId] = new CHART_JS(canvas, {
      type: 'bubble',
      data: {
        datasets: [{
          data: dataPoints.map(p => ({ x: p.x, y: p.y, r: 18, _v: p.v, _n: p.n, _wr: p.wr, _name: p.name })),
          backgroundColor: dataPoints.map(p => icColor(p.v)),
          borderColor: C.border,
          borderWidth: 1,
        }],
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: { duration: 300 },
        plugins: {
          legend: { display: false },
          tooltip: {
            backgroundColor: '#1a1d27',
            titleColor: C.text,
            bodyColor: C.textSec,
            callbacks: {
              title: (items) => items[0]?.raw?._name || '',
              label: (item) => {
                const d = item.raw;
                return [`IC: ${(d._v || 0).toFixed(4)}`, `Trades: ${d._n}`, `Win Rate: ${((d._wr || 0) * 100).toFixed(1)}%`];
              },
            },
          },
        },
        scales: {
          x: { display: false, min: -0.5, max: 0.5 },
          y: {
            type: 'linear',
            min: -0.5,
            max: methodNames.length - 0.5,
            ticks: { stepSize: 1, callback: v => methodNames[v] || '', color: C.text, font: { size: 10 } },
            grid: { color: C.grid, drawBorder: false },
          },
        },
      },
    });
  }

  // ── 5. DAILY P&L CALENDAR ──────────────────────────────────────
  function renderPnlCalendar(containerId, calendar) {
    const el = document.getElementById(containerId);
    if (!el) return;

    if (!calendar?.length) {
      el.innerHTML = '<div style="color:#94a3b8;text-align:center;padding:24px">Sin trades cerrados para generar calendario</div>';
      return;
    }

    // GitHub-contribution-style grid
    const maxAbs = Math.max(...calendar.map(d => Math.abs(d.pnl)), 1);
    const weeks = {};
    calendar.forEach(d => {
      const dt = new Date(d.date + 'T12:00:00');
      const weekStart = new Date(dt);
      weekStart.setDate(dt.getDate() - dt.getDay());
      const wk = weekStart.toISOString().slice(0, 10);
      if (!weeks[wk]) weeks[wk] = new Array(7).fill(null);
      weeks[wk][dt.getDay()] = d;
    });

    const weekKeys = Object.keys(weeks).sort();
    const dayLabels = ['D', 'L', 'M', 'X', 'J', 'V', 'S'];
    const cellSize = 18;
    const gap = 3;

    let html = '<div style="display:flex;gap:2px;overflow-x:auto;padding:4px">';
    // Day labels column
    html += '<div style="display:flex;flex-direction:column;gap:' + gap + 'px;margin-right:4px;padding-top:' + (cellSize + gap) + 'px">';
    dayLabels.forEach((l, i) => {
      if (i % 2 === 1) html += `<div style="width:14px;height:${cellSize}px;font-size:9px;color:${C.textMut};display:flex;align-items:center">${l}</div>`;
      else html += `<div style="width:14px;height:${cellSize}px"></div>`;
    });
    html += '</div>';

    weekKeys.forEach(wk => {
      const month = new Date(wk + 'T12:00:00').toLocaleDateString('es', { month: 'short' });
      html += '<div style="display:flex;flex-direction:column;gap:' + gap + 'px">';
      html += `<div style="height:${cellSize}px;font-size:8px;color:${C.textMut};text-align:center;line-height:${cellSize}px">${month}</div>`;
      for (let d = 0; d < 7; d++) {
        const day = weeks[wk][d];
        let bg = 'rgba(255,255,255,0.03)';
        let title = 'Sin trades';
        if (day) {
          const intensity = Math.min(Math.abs(day.pnl) / maxAbs, 1);
          const alpha = 0.2 + intensity * 0.65;
          bg = day.pnl >= 0
            ? `rgba(72,187,120,${alpha})`
            : `rgba(244,63,94,${alpha})`;
          title = `${day.date}: $${day.pnl.toFixed(2)} | ${day.trades} trades | ${day.dominant_strategy}`;
        }
        html += `<div style="width:${cellSize}px;height:${cellSize}px;background:${bg};border-radius:3px;cursor:default" title="${title}"></div>`;
      }
      html += '</div>';
    });
    html += '</div>';

    // Legend
    html += '<div style="display:flex;gap:12px;justify-content:center;margin-top:8px;font-size:10px;color:' + C.textSec + '">';
    html += '<span style="display:flex;align-items:center;gap:4px"><span style="width:10px;height:10px;background:rgba(244,63,94,0.6);border-radius:2px"></span> Loss</span>';
    html += '<span style="display:flex;align-items:center;gap:4px"><span style="width:10px;height:10px;background:rgba(255,255,255,0.05);border-radius:2px"></span> Sin trades</span>';
    html += '<span style="display:flex;align-items:center;gap:4px"><span style="width:10px;height:10px;background:rgba(72,187,120,0.6);border-radius:2px"></span> Win</span>';
    html += '</div>';

    el.innerHTML = html;
  }

  // ── 6. POLICY EVOLUTION ─────────────────────────────────────────
  function renderPolicyEvolution(containerId, icData) {
    const canvas = document.getElementById(containerId);
    if (!canvas) return;
    _destroy(containerId);

    const methods = icData?.by_method || icData?.methods || icData?.per_method || {};
    const methodNames = Object.keys(methods);

    if (!methodNames.length) {
      _instances[containerId] = new CHART_JS(canvas, {
        type: 'bar',
        data: { labels: [], datasets: [] },
        options: _chartOpts({
          plugins: { title: { display: true, text: 'Sin datos de policy', color: C.textSec, font: { size: 11 } } },
        }),
      });
      return;
    }

    const sizeMultipliers = methodNames.map(name => {
      const m = methods[name];
      return typeof m === 'object' ? (m.size_multiplier ?? m.weight ?? 1.0) : 1.0;
    });
    const scoreThresholds = methodNames.map(name => {
      const m = methods[name];
      return typeof m === 'object' ? (m.score_threshold ?? m.min_score ?? 65) : 65;
    });
    const icValues = methodNames.map(name => {
      const m = methods[name];
      return parseFloat(typeof m === 'object' ? (m.ic ?? m.spearman_ic ?? 0) : 0) || 0;
    });

    const barColors = icValues.map(ic =>
      ic >= 0.05 ? C.green + 'cc' :
      ic >= 0 ? C.yellow + '99' :
      C.rose + 'cc'
    );

    _instances[containerId] = new CHART_JS(canvas, {
      type: 'bar',
      data: {
        labels: methodNames,
        datasets: [
          {
            label: 'Size Multiplier',
            data: sizeMultipliers,
            backgroundColor: barColors,
            borderWidth: 0,
            borderRadius: 3,
            yAxisID: 'yLeft',
            order: 2,
          },
          {
            label: 'Score Threshold',
            data: scoreThresholds,
            type: 'line',
            borderColor: C.cyan,
            backgroundColor: 'transparent',
            borderWidth: 2,
            pointRadius: 4,
            pointBackgroundColor: C.cyan,
            yAxisID: 'yRight',
            order: 1,
          },
        ],
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: { duration: 300 },
        plugins: {
          legend: { display: true, labels: { color: C.textSec, boxWidth: 10, font: { size: 10 } } },
          tooltip: {
            backgroundColor: '#1a1d27',
            titleColor: C.text,
            bodyColor: C.textSec,
          },
        },
        scales: {
          x: { grid: { display: false }, ticks: { color: C.text, font: { size: 10 } } },
          yLeft: {
            position: 'left',
            grid: { color: C.grid, drawBorder: false },
            ticks: { color: C.green, font: { size: 10 } },
            min: 0, max: 1.5,
            title: { display: true, text: 'Size Mult.', color: C.textMut, font: { size: 9 } },
          },
          yRight: {
            position: 'right',
            grid: { display: false },
            ticks: { color: C.cyan, font: { size: 10 } },
            min: 0, max: 100,
            title: { display: true, text: 'Score Thr.', color: C.textMut, font: { size: 9 } },
          },
        },
      },
    });
  }

  // ── Robot Status Widget ─────────────────────────────────────────
  function renderRobotStatus(containerId, chartData, icData) {
    const el = document.getElementById(containerId);
    if (!el) return;

    const trades = chartData?.trades || [];
    const today = new Date().toISOString().slice(0, 10);
    const todayTrades = trades.filter(t => (t.exit_time || '').startsWith(today));
    const todayPnl = todayTrades.reduce((s, t) => s + (t.pnl || 0), 0);
    const totalTrades = trades.length;
    const lastEquity = trades.length ? trades[trades.length - 1].equity : 0;

    // IC average
    const methods = icData?.by_method || icData?.methods || icData?.per_method || {};
    const icVals = Object.values(methods).map(m =>
      parseFloat(typeof m === 'object' ? (m.ic ?? m.spearman_ic ?? 0) : (m || 0)) || 0
    );
    const avgIc = icVals.length ? icVals.reduce((s, v) => s + v, 0) / icVals.length : 0;

    // Health color
    let healthColor = C.green;
    let pulseClass = 'pulse-active';
    if (totalTrades === 0) { healthColor = C.yellow; pulseClass = 'pulse-idle'; }
    if (avgIc < -0.02) { healthColor = C.rose; pulseClass = 'pulse-warn'; }

    const eqChangeStr = lastEquity !== 0
      ? `${todayPnl >= 0 ? '+' : ''}$${todayPnl.toFixed(2)}`
      : '--';

    el.innerHTML = `
      <div class="robot-widget" style="display:flex;align-items:center;gap:16px;padding:12px 16px;background:${C.panel};border-radius:8px;border:1px solid ${C.border}">
        <div class="robot-icon" style="position:relative;width:40px;height:40px;flex-shrink:0">
          <svg viewBox="0 0 40 40" width="40" height="40">
            <rect x="8" y="12" width="24" height="20" rx="4" fill="${healthColor}" opacity="0.2" stroke="${healthColor}" stroke-width="1.5"/>
            <circle cx="16" cy="22" r="2.5" fill="${healthColor}"/>
            <circle cx="24" cy="22" r="2.5" fill="${healthColor}"/>
            <rect x="14" y="27" width="12" height="2" rx="1" fill="${healthColor}" opacity="0.6"/>
            <line x1="20" y1="6" x2="20" y2="12" stroke="${healthColor}" stroke-width="1.5"/>
            <circle cx="20" cy="5" r="2" fill="${healthColor}"/>
          </svg>
          <style>
            @keyframes robotPulse { 0%,100%{opacity:0.6} 50%{opacity:1} }
            .robot-icon svg { animation: robotPulse 2s ease-in-out infinite; }
          </style>
        </div>
        <div style="display:flex;flex-direction:column;gap:2px;flex:1;min-width:0">
          <div style="display:flex;gap:16px;font-size:11px;color:${C.textSec}">
            <span>IC: <strong style="color:${avgIc >= 0.05 ? C.green : avgIc >= 0 ? C.text : C.rose}">${avgIc.toFixed(3)}</strong></span>
            <span>Hoy: <strong style="color:${todayPnl >= 0 ? C.green : C.rose}">${eqChangeStr}</strong></span>
            <span>Trades: <strong style="color:${C.text}">${todayTrades.length} hoy / ${totalTrades} total</strong></span>
          </div>
          <div style="font-size:10px;color:${C.textMut}">Equity acumulado: $${lastEquity.toFixed(2)}</div>
        </div>
      </div>`;
  }

  // ── Render all charts ───────────────────────────────────────────
  async function _renderAll() {
    const { chartData, icSummary } = await _fetchData();
    const trades = chartData?.trades || [];
    const calendar = chartData?.calendar || [];

    renderRobotStatus('ac-robot-status', chartData, icSummary);

    if (!CHART_JS) {
      _ts('update', document.getElementById('ac-last-update'));
      return;
    }

    renderEquityCurve('ac-equity-curve', trades);
    renderLearningCurve('ac-learning-curve', icSummary);
    renderRMultiple('ac-r-distribution', trades);
    renderICHeatmap('ac-ic-heatmap', icSummary);
    renderPnlCalendar('ac-pnl-calendar', calendar);
    renderPolicyEvolution('ac-policy-evolution', icSummary);

    _ts('update', document.getElementById('ac-last-update'));
  }

  // ── Polling ─────────────────────────────────────────────────────
  function _startPolling() {
    if (_pollTimer) return;
    _pollTimer = setInterval(() => {
      if (_visible) _renderAll();
    }, 30000);
  }

  function _stopPolling() {
    if (_pollTimer) { clearInterval(_pollTimer); _pollTimer = null; }
  }

  // ── Public API ──────────────────────────────────────────────────
  async function init() {
    _visible = true;
    await _renderAll();
    _startPolling();
  }

  async function refresh() {
    _cache.lastFetch = 0;
    await _renderAll();
  }

  function show() { _visible = true; _startPolling(); _renderAll(); }
  function hide() { _visible = false; _stopPolling(); }

  function getChartById(id) { return _instances[id] || null; }

  return { init, refresh, show, hide, getChartById };
})();

window.AtlasCharts = AtlasCharts;
