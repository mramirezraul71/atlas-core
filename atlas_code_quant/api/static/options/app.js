/* ================================================================
   OptionStrat UI â€” Atlas Code-Quant  v1.0.0
   Vanilla JS SPA. Communicates with /options/* API endpoints.
   ================================================================ */

'use strict';

// â”€â”€ API base (same-origin) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
const API = '';  // relative URLs

// â”€â”€ State â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
const State = {
  legs: [],            // array of leg objects
  payoffChart: null,
  portPnlChart: null,
  portBiasChart: null,
  templates: [],
  selectedTemplate: null,
  currentStrategy: null,   // last preview result
  previewBuilt: false,
  portfolioPollHandle: null,
  loadingPortfolio: false,
  optionstratConfig: { mirror_only: false },
  mirrorStatus: null,
};

// â”€â”€ Helpers â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
function toast(msg, type = 'info', ms = 3500) {
  const c = document.getElementById('toast-container');
  const d = document.createElement('div');
  d.className = `toast ${type}`;
  d.textContent = msg;
  c.appendChild(d);
  setTimeout(() => d.remove(), ms);
}

function fmt(n, digits = 2) {
  if (n == null || isNaN(n)) return '--';
  return Number(n).toFixed(digits);
}
function fmtSign(n, digits = 2) {
  if (n == null || isNaN(n)) return '--';
  const s = Number(n).toFixed(digits);
  return n > 0 ? '+' + s : s;
}
function fmtMoney(n) {
  if (n == null || isNaN(n)) return '--';
  if (Math.abs(n) >= 1e6) return (n/1e6).toFixed(2) + 'M';
  if (Math.abs(n) >= 1e3) return (n/1e3).toFixed(1) + 'k';
  return fmtSign(n, 2);
}

async function apiPost(path, body) {
  const r = await fetch(API + path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  const j = await r.json();
  if (!r.ok || !j.ok) throw new Error(j.detail || j.error || 'Error en API');
  return j.data;
}
async function apiGet(path) {
  const r = await fetch(API + path);
  const j = await r.json();
  if (!r.ok || !j.ok) throw new Error(j.detail || j.error || 'Error en API');
  return j.data;
}
async function apiDelete(path) {
  const r = await fetch(API + path, { method: 'DELETE' });
  const j = await r.json();
  if (!r.ok || !j.ok) throw new Error(j.detail || j.error || 'Error en API');
  return j.data;
}

// â”€â”€ Nav â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
function initNav() {
  document.querySelectorAll('.nav-item[data-view]').forEach(el => {
    el.addEventListener('click', () => {
      document.querySelectorAll('.nav-item').forEach(x => x.classList.remove('active'));
      document.querySelectorAll('.view').forEach(x => x.classList.remove('active'));
      el.classList.add('active');
      document.getElementById('view-' + el.dataset.view).classList.add('active');
      if (el.dataset.view === 'templates') loadTemplates();
      if (el.dataset.view === 'portfolio') loadPortfolio();
    });
  });

  // Clock
  setInterval(() => {
    const el = document.getElementById('sidebar-time');
    if (el) el.textContent = new Date().toLocaleTimeString('es-ES');
  }, 1000);

  // API health
  checkApiStatus();
  setInterval(checkApiStatus, 30000);
  State.portfolioPollHandle = setInterval(() => {
    const isPortfolioActive = document.getElementById('view-portfolio')?.classList.contains('active');
    if (isPortfolioActive) loadPortfolio();
  }, 10000);
}

async function checkApiStatus() {
  try {
    const status = await apiGet('/options/portfolio/status');
    const el = document.getElementById('api-status');
    el.className = 'badge badge-open';
    el.textContent = 'API OK';
    renderBrokerMirrorBadge(status);
    if (typeof status?.mirror_only === 'boolean') {
      State.optionstratConfig = { ...State.optionstratConfig, mirror_only: status.mirror_only };
      applyOptionstratModeUI();
    }
  } catch {
    const el = document.getElementById('api-status');
    el.className = 'badge badge-closed';
    el.textContent = 'API --';
    renderBrokerMirrorBadge(null);
  }
}

function renderBrokerMirrorBadge(status) {
  const badge = document.getElementById('mirror-status');
  if (!badge) return;
  State.mirrorStatus = status || null;
  if (!status || !status.mirror_only) {
    badge.className = 'badge badge-closed';
    badge.textContent = 'BROKER MIRROR: OFF';
    badge.title = 'Mirror mode desactivado';
    return;
  }

  const lastSyncRaw = status.last_sync_at;
  const lastSyncTs = lastSyncRaw ? Date.parse(lastSyncRaw) : NaN;
  const now = Date.now();
  const ageSec = Number.isFinite(lastSyncTs) ? Math.max(0, Math.floor((now - lastSyncTs) / 1000)) : Infinity;
  const isFresh = ageSec <= 45;
  const isStale = ageSec > 45 && ageSec <= 180;
  const stats = status.last_sync_stats || {};
  const scope = status.tradier_scope || State.optionstratConfig?.tradier_scope || '--';
  const tooltip = [
    `Scope: ${scope}`,
    `Open: ${status.n_open ?? '--'} | Closed: ${status.n_closed ?? '--'} | Total: ${status.n_total ?? '--'}`,
    `Last sync: ${lastSyncRaw || '--'}`,
    `Delta ciclo: +${stats.added || 0} / ~${stats.updated || 0} / -${stats.closed || 0}`,
    `Activas broker: ${stats.active ?? '--'}`,
  ].join('\n');
  badge.title = tooltip;

  if (isFresh) {
    badge.className = 'badge badge-open';
    badge.textContent = `BROKER MIRROR: ON · ${ageSec}s`;
    return;
  }
  if (isStale) {
    badge.className = 'badge badge-warn';
    badge.textContent = `BROKER MIRROR: STALE · ${ageSec}s`;
    return;
  }
  badge.className = 'badge badge-closed';
  badge.textContent = 'BROKER MIRROR: OFFLINE';
}

async function loadOptionstratConfig() {
  try {
    const cfg = await apiGet('/options/config');
    State.optionstratConfig = cfg || { mirror_only: false, tradier_scope: '--' };
    applyOptionstratModeUI();
    renderBrokerMirrorBadge(State.mirrorStatus);
  } catch {
    State.optionstratConfig = { mirror_only: false, tradier_scope: '--' };
    applyOptionstratModeUI();
    renderBrokerMirrorBadge(State.mirrorStatus);
  }
}

function applyOptionstratModeUI() {
  const mirrorOnly = !!State.optionstratConfig?.mirror_only;
  const addBtn = document.getElementById('btn-add-to-portfolio');
  if (addBtn) {
    addBtn.disabled = mirrorOnly;
    addBtn.title = mirrorOnly ? 'Modo espejo broker: alta manual bloqueada' : '';
    addBtn.textContent = mirrorOnly ? '⛔ Solo espejo broker' : '＋ Añadir a cartera';
  }
}

// â”€â”€ Legs Builder â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
let _legCounter = 0;

function makeLegId() { return 'leg_' + (++_legCounter); }

function defaultExpiry(dte = 30) {
  const d = new Date();
  d.setDate(d.getDate() + dte);
  return d.toISOString().split('T')[0];
}

function addOptionLeg(optType, side = 'long') {
  const spot = parseFloat(document.getElementById('strat-spot').value) || 500;
  State.legs.push({
    id: makeLegId(),
    leg_type: 'option',
    option_type: optType,  // 'call' | 'put'
    side,
    strike: spot,
    expiry: defaultExpiry(30),
    iv: 0.25,
    premium: optType === 'call' ? 5.0 : 3.0,
    quantity: 1,
    multiplier: 100,
    underlying_symbol: document.getElementById('strat-underlying').value || 'SPY',
    underlying_price: spot,
  });
  renderLegs();
}

function addLinearLeg() {
  const spot = parseFloat(document.getElementById('strat-spot').value) || 500;
  State.legs.push({
    id: makeLegId(),
    leg_type: 'linear',
    symbol: document.getElementById('strat-underlying').value || 'SPY',
    asset_class: 'stock',
    side: 'long',
    quantity: 100,
    entry_price: spot,
    multiplier: 1,
  });
  renderLegs();
}

function removeLeg(id) {
  State.legs = State.legs.filter(l => l.id !== id);
  renderLegs();
}

function renderLegs() {
  const list = document.getElementById('legs-list');
  const empty = document.getElementById('legs-empty');
  if (!State.legs.length) {
    list.innerHTML = '';
    list.appendChild(empty);
    empty.style.display = '';
    return;
  }
  empty.style.display = 'none';
  list.innerHTML = State.legs.map(leg => renderLegCard(leg)).join('');
  // Bind remove buttons
  list.querySelectorAll('[data-remove]').forEach(btn => {
    btn.addEventListener('click', () => removeLeg(btn.dataset.remove));
  });
  // Bind input changes
  list.querySelectorAll('[data-field]').forEach(inp => {
    inp.addEventListener('change', e => {
      const legId = inp.dataset.legId;
      const field = inp.dataset.field;
      const leg = State.legs.find(l => l.id === legId);
      if (!leg) return;
      const v = inp.type === 'number' ? parseFloat(inp.value) : inp.value;
      leg[field] = isNaN(v) ? inp.value : v;
    });
  });
}

function renderLegCard(leg) {
  if (leg.leg_type === 'option') {
    const cls = `${leg.side}-${leg.option_type}`;
    const badge = `<span class="leg-type-badge ${leg.option_type}">${leg.side.toUpperCase()} ${leg.option_type.toUpperCase()}</span>`;
    return `
    <div class="leg-card ${cls}">
      <div>
        ${badge}
        <div class="leg-label" style="margin-top:4px">Strike ($)</div>
        <input class="input-sm" type="number" step="0.5" value="${leg.strike}"
          data-leg-id="${leg.id}" data-field="strike" />
      </div>
      <div>
        <div class="leg-label">Expiry</div>
        <input class="input-sm" type="date" value="${leg.expiry}"
          data-leg-id="${leg.id}" data-field="expiry" />
      </div>
      <div>
        <div class="leg-label">Premium ($)</div>
        <input class="input-sm" type="number" step="0.01" value="${leg.premium}"
          data-leg-id="${leg.id}" data-field="premium" />
      </div>
      <div>
        <div class="leg-label">IV (0-1)</div>
        <input class="input-sm" type="number" step="0.01" value="${leg.iv}"
          data-leg-id="${leg.id}" data-field="iv" />
        <div class="leg-label" style="margin-top:4px">Qty</div>
        <input class="input-sm" type="number" value="${leg.quantity}"
          data-leg-id="${leg.id}" data-field="quantity" style="margin-top:2px" />
      </div>
      <button class="leg-remove" data-remove="${leg.id}" title="Eliminar leg">âœ•</button>
    </div>`;
  } else {
    // linear
    return `
    <div class="leg-card linear">
      <div>
        <span class="leg-type-badge lin">${leg.side.toUpperCase()} STOCK</span>
        <div class="leg-label" style="margin-top:4px">SÃ­mbolo</div>
        <input class="input-sm" value="${leg.symbol}"
          data-leg-id="${leg.id}" data-field="symbol" />
      </div>
      <div>
        <div class="leg-label">Precio entrada ($)</div>
        <input class="input-sm" type="number" step="0.01" value="${leg.entry_price}"
          data-leg-id="${leg.id}" data-field="entry_price" />
      </div>
      <div>
        <div class="leg-label">Cantidad</div>
        <input class="input-sm" type="number" value="${leg.quantity}"
          data-leg-id="${leg.id}" data-field="quantity" />
      </div>
      <div>
        <div class="leg-label">Lado</div>
        <select class="select-sm" data-leg-id="${leg.id}" data-field="side">
          <option ${leg.side==='long'?'selected':''}>long</option>
          <option ${leg.side==='short'?'selected':''}>short</option>
        </select>
      </div>
      <button class="leg-remove" data-remove="${leg.id}" title="Eliminar leg">âœ•</button>
    </div>`;
  }
}

// â”€â”€ Build strategy payload from State â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
function buildStrategyPayload() {
  const name = document.getElementById('strat-name').value || 'Mi Estrategia';
  const underlying = document.getElementById('strat-underlying').value || 'SPY';
  const spot = parseFloat(document.getElementById('strat-spot').value) || 500;
  const rate = (parseFloat(document.getElementById('strat-rate').value) || 5) / 100;

  const legs = State.legs.map(leg => {
    if (leg.leg_type === 'option') {
      return {
        leg_type: 'option',
        underlying_symbol: leg.underlying_symbol || underlying,
        expiry: leg.expiry,
        strike: leg.strike,
        option_type: leg.option_type,
        side: leg.side,
        quantity: leg.quantity,
        premium: leg.premium,
        multiplier: leg.multiplier || 100,
        iv: leg.iv,
        underlying_price: spot,
      };
    } else {
      return {
        leg_type: 'linear',
        symbol: leg.symbol || underlying,
        asset_class: leg.asset_class || 'stock',
        side: leg.side,
        quantity: leg.quantity,
        entry_price: leg.entry_price,
        multiplier: leg.multiplier || 1,
      };
    }
  });

  return {
    strategy: { name, underlying, legs, metadata: {} },
    market: {
      underlying_price: spot,
      risk_free_rate: rate,
    },
    n_points: 300,
    spot_range_pct: 0.35,
  };
}

// â”€â”€ Preview â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
async function previewStrategy() {
  if (!State.legs.length) { toast('AÃ±ade al menos un leg', 'err'); return; }
  const btn = document.getElementById('btn-preview');
  btn.disabled = true;
  btn.innerHTML = '<span class="spinner"></span>';
  try {
    const payload = buildStrategyPayload();
    const data = await apiPost('/options/strategies/preview', payload);
    State.currentStrategy = data;
    State.previewBuilt = true;
    renderPreviewResults(data);
    toast('Estrategia calculada en ' + (data.ms || '?') + ' ms', 'ok');
  } catch(e) {
    toast('Error: ' + e.message, 'err');
  } finally {
    btn.disabled = false;
    btn.innerHTML = 'â–¶ Calcular';
  }
}

function renderPreviewResults(data) {
  // Risk bar
  const risk = data.risk_summary;
  document.getElementById('risk-bar').style.display = '';
  const mp = risk.max_profit;
  const ml = risk.max_loss;
  document.getElementById('risk-max-profit').textContent =
    mp === null || mp === undefined ? 'Ilimitado' : ('$' + fmt(mp));
  document.getElementById('risk-max-loss').textContent =
    ml === null || ml === undefined ? 'Ilimitado' : ('$' + fmt(ml));
  document.getElementById('risk-premium').textContent = '$' + fmt(risk.net_premium);

  // Greeks
  const g = data.greeks;
  document.getElementById('greeks-row').style.display = '';
  document.getElementById('g-delta').textContent = fmt(g.delta, 3);
  document.getElementById('g-gamma').textContent = fmt(g.gamma, 4);
  document.getElementById('g-theta').textContent = fmtSign(g.theta, 2);
  document.getElementById('g-vega').textContent = fmt(g.vega, 2);
  document.getElementById('g-rho').textContent = fmt(g.rho, 2);
  colorizeGreek('g-theta', g.theta);
  colorizeGreek('g-delta', g.delta, false);

  // Breakevens
  const bkList = document.getElementById('breakeven-list');
  bkList.innerHTML = '';
  if (data.breakevens && data.breakevens.length) {
    data.breakevens.forEach(b => {
      const chip = document.createElement('span');
      chip.className = 'breakeven-chip';
      chip.textContent = '$' + fmt(b);
      bkList.appendChild(chip);
    });
  } else {
    bkList.innerHTML = '<span class="empty-chip">Sin breakeven detectado</span>';
  }

  // Chart
  document.getElementById('chart-placeholder').style.display = 'none';
  renderPayoffChart(data.s_grid, data.payoff_expiry, data.pnl_today);
}

function colorizeGreek(id, val, negBad = true) {
  const el = document.getElementById(id);
  if (!el) return;
  if (negBad) {
    el.classList.toggle('text-green', val > 0);
    el.classList.toggle('text-red', val < 0);
  }
}

// â”€â”€ Payoff Chart â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
function renderPayoffChart(sGrid, payoffExpiry, pnlToday) {
  const ctx = document.getElementById('chart-payoff').getContext('2d');
  if (State.payoffChart) State.payoffChart.destroy();

  const zeroLine = sGrid.map(() => 0);

  State.payoffChart = new Chart(ctx, {
    type: 'line',
    data: {
      labels: sGrid.map(s => '$' + s.toFixed(0)),
      datasets: [
        {
          label: 'Vencimiento',
          data: payoffExpiry,
          borderColor: '#00d4aa',
          borderWidth: 2,
          pointRadius: 0,
          fill: false,
          tension: 0.1,
        },
        {
          label: 'T+0 (hoy)',
          data: pnlToday,
          borderColor: '#63b3ed',
          borderWidth: 1.5,
          borderDash: [5, 3],
          pointRadius: 0,
          fill: false,
          tension: 0.2,
        },
        {
          label: 'Zero',
          data: zeroLine,
          borderColor: 'rgba(74,85,104,0.5)',
          borderWidth: 1,
          borderDash: [2, 2],
          pointRadius: 0,
          fill: false,
        },
      ],
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      interaction: { mode: 'index', intersect: false },
      plugins: {
        legend: { display: false },
        tooltip: {
          backgroundColor: '#191d26',
          borderColor: '#2a3148',
          borderWidth: 1,
          titleColor: '#8892a4',
          bodyColor: '#e2e8f0',
          callbacks: {
            label: ctx => {
              if (ctx.dataset.label === 'Zero') return null;
              return ctx.dataset.label + ': $' + ctx.parsed.y.toFixed(2);
            },
          },
        },
      },
      scales: {
        x: {
          ticks: { color: '#4a5568', font: { size: 10 }, maxTicksLimit: 10 },
          grid: { color: '#1f2433' },
        },
        y: {
          ticks: { color: '#4a5568', font: { size: 10 },
            callback: v => '$' + v.toFixed(0) },
          grid: { color: '#1f2433' },
        },
      },
    },
  });
}

// â”€â”€ Clear builder â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
function clearBuilder() {
  State.legs = [];
  State.currentStrategy = null;
  State.previewBuilt = false;
  renderLegs();
  document.getElementById('risk-bar').style.display = 'none';
  document.getElementById('greeks-row').style.display = 'none';
  document.getElementById('chart-placeholder').style.display = '';
  document.getElementById('breakeven-list').innerHTML = '<span class="empty-chip">--</span>';
  if (State.payoffChart) { State.payoffChart.destroy(); State.payoffChart = null; }
}

// â”€â”€ Add to portfolio â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
async function addToPortfolio() {
  if (State.optionstratConfig?.mirror_only) {
    toast('Modo espejo broker activo: alta manual deshabilitada', 'err');
    return;
  }
  if (!State.previewBuilt || !State.legs.length) {
    toast('Calcula la estrategia primero', 'err'); return;
  }
  try {
    const p = buildStrategyPayload();
    const payload = {
      strategy: p.strategy,
      market: p.market,
      quantity: 1,
      notes: '',
      tags: [],
    };
    const data = await apiPost('/options/portfolio/add', payload);
    toast('Estrategia "' + data.name + '" aÃ±adida a cartera', 'ok');
  } catch(e) {
    toast('Error: ' + e.message, 'err');
  }
}

// â”€â”€ Templates â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
async function loadTemplates() {
  const grid = document.getElementById('template-grid');
  grid.innerHTML = '<div class="text-muted label-sm">Cargandoâ€¦</div>';
  try {
    const data = await apiGet('/options/templates');
    State.templates = data;
    renderTemplateGrid(data);
  } catch(e) {
    grid.innerHTML = '<div class="text-muted label-sm">Error: ' + e.message + '</div>';
  }
}

function renderTemplateGrid(templates) {
  const biasFilter = document.getElementById('tpl-filter-bias').value;
  const catFilter  = document.getElementById('tpl-filter-cat').value;
  const grid = document.getElementById('template-grid');

  const filtered = templates.filter(t =>
    (!biasFilter || t.bias === biasFilter) &&
    (!catFilter  || t.category === catFilter)
  );

  if (!filtered.length) {
    grid.innerHTML = '<div class="text-muted label-sm">Sin resultados</div>';
    return;
  }

  grid.innerHTML = filtered.map(t => `
    <div class="template-card" data-tpl="${t.name}" title="${t.name}">
      <div class="tpl-name">${t.name.replace(/_/g, ' ')}</div>
      <div class="tpl-cat">${t.category}</div>
      <div class="tpl-bias ${t.bias}">${t.bias}</div>
    </div>`).join('');

  grid.querySelectorAll('.template-card').forEach(card => {
    card.addEventListener('click', () => selectTemplate(card.dataset.tpl));
  });
}

function selectTemplate(name) {
  document.querySelectorAll('.template-card').forEach(c =>
    c.classList.toggle('selected', c.dataset.tpl === name));
  State.selectedTemplate = name;

  const panel = document.getElementById('tpl-build-panel');
  panel.style.display = '';
  document.getElementById('tpl-build-title').textContent = 'Construir: ' + name.replace(/_/g, ' ');
  panel.scrollIntoView({ behavior: 'smooth' });
}

async function buildFromTemplate() {
  if (!State.selectedTemplate) { toast('Selecciona una plantilla', 'err'); return; }
  const btn = document.getElementById('btn-tpl-build');
  btn.disabled = true;
  try {
    const spot = parseFloat(document.getElementById('tpl-spot').value) || 500;
    const strike  = parseFloat(document.getElementById('tpl-strike').value) || spot;
    const strike2 = parseFloat(document.getElementById('tpl-strike2').value) || spot + 10;
    const strike3 = parseFloat(document.getElementById('tpl-strike3').value) || spot - 10;
    const strike4 = parseFloat(document.getElementById('tpl-strike4').value) || spot + 20;

    const payload = {
      template_name: State.selectedTemplate,
      symbol: document.getElementById('tpl-symbol').value || 'SPY',
      spot,
      dte: parseInt(document.getElementById('tpl-dte').value) || 30,
      iv:  parseFloat(document.getElementById('tpl-iv').value) || 0.25,
      quantity: parseInt(document.getElementById('tpl-qty').value) || 1,
      strikes: {
        strike: strike,
        strike_long: strike,
        strike_short: strike2,
        strike_call: strike,
        strike_put: strike3,
        lower_call: strike,
        upper_call: strike2,
        lower_put: strike3,
        upper_put: strike4,
        strike_atm: spot,
        strike_otm: strike2,
      },
    };

    const data = await apiPost('/options/templates/build', payload);
    // Load strategy into builder
    await loadStrategyIntoBuilder(data, spot);
    toast('Plantilla cargada en el Constructor', 'ok');
    // Switch to builder view
    document.querySelector('.nav-item[data-view="builder"]').click();
  } catch(e) {
    toast('Error: ' + e.message, 'err');
  } finally {
    btn.disabled = false;
  }
}

async function loadStrategyIntoBuilder(strat, spot) {
  document.getElementById('strat-name').value = strat.name || 'Mi Estrategia';
  document.getElementById('strat-underlying').value = strat.underlying || 'SPY';
  document.getElementById('strat-spot').value = spot;
  State.legs = [];
  _legCounter = 0;
  for (const leg of (strat.legs || [])) {
    State.legs.push({
      id: makeLegId(),
      ...leg,
    });
  }
  renderLegs();
  State.previewBuilt = false;
  document.getElementById('risk-bar').style.display = 'none';
  document.getElementById('greeks-row').style.display = 'none';
  document.getElementById('chart-placeholder').style.display = '';
  if (State.payoffChart) { State.payoffChart.destroy(); State.payoffChart = null; }
}

// â”€â”€ Scenario heatmap â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
async function runScenario() {
  if (!State.legs.length) { toast('Construye una estrategia primero', 'err'); return; }
  const btn = document.getElementById('btn-run-scenario');
  btn.disabled = true;
  btn.innerHTML = '<span class="spinner"></span>';

  try {
    const p = buildStrategyPayload();
    const dteRaw = document.getElementById('sc-dte-vals').value;
    const dteValues = dteRaw.split(',').map(x => parseInt(x.trim())).filter(Boolean);
    const payload = {
      strategy: p.strategy,
      spot_range: [
        parseFloat(document.getElementById('sc-spot-min').value) || 430,
        parseFloat(document.getElementById('sc-spot-max').value) || 570,
      ],
      dte_values: dteValues,
      iv_multiplier: parseFloat(document.getElementById('sc-iv-mult').value) || 1.0,
      risk_free_rate: p.market.risk_free_rate,
      n_spots: parseInt(document.getElementById('sc-n-spots').value) || 15,
    };
    const data = await apiPost('/options/scenario', payload);
    renderHeatmap(data);
    toast('Heatmap calculado', 'ok');
  } catch(e) {
    toast('Error: ' + e.message, 'err');
  } finally {
    btn.disabled = false;
    btn.innerHTML = 'â–¶ Calcular Heatmap';
  }
}

function renderHeatmap(data) {
  const container = document.getElementById('heatmap-container');
  const { s_values, dte_values, pnl_matrix } = data;

  // Find min/max for color scale
  let minPnl = Infinity, maxPnl = -Infinity;
  pnl_matrix.forEach(row => row.forEach(v => {
    if (v < minPnl) minPnl = v;
    if (v > maxPnl) maxPnl = v;
  }));

  function pnlColor(v) {
    if (v >= 0) {
      const t = maxPnl > 0 ? Math.min(1, v / maxPnl) : 0;
      const g = Math.round(26 + t * (187 - 26));
      const rr = Math.round(26 + (1-t) * (0));
      return `rgb(${rr},${g},${Math.round(26 + t * 0)})`;
    } else {
      const t = minPnl < 0 ? Math.min(1, v / minPnl) : 0;
      const r = Math.round(245 * t + 30 * (1-t));
      return `rgb(${r},${Math.round(26*(1-t))},${Math.round(26*(1-t))})`;
    }
  }

  let html = '<table class="heatmap-tbl"><thead><tr><th>Spotâ†“ / DTEâ†’</th>';
  dte_values.forEach(d => { html += `<th>${d}d</th>`; });
  html += '</tr></thead><tbody>';

  s_values.forEach((spot, i) => {
    html += `<tr><th>$${spot.toFixed(0)}</th>`;
    pnl_matrix[i].forEach(v => {
      const bg = pnlColor(v);
      const textClr = Math.abs(v) > (maxPnl - minPnl) * 0.3 ? '#e2e8f0' : '#8892a4';
      html += `<td style="background:${bg};color:${textClr};min-width:56px;border-radius:2px">
        ${v >= 0 ? '+' : ''}${v.toFixed(0)}
      </td>`;
    });
    html += '</tr>';
  });

  html += '</tbody></table>';
  container.innerHTML = html;
}

// â”€â”€ Portfolio â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
async function loadPortfolio() {
  if (State.loadingPortfolio) return;
  State.loadingPortfolio = true;
  const btn = document.getElementById('btn-port-refresh');
  btn.innerHTML = '<span class="spinner"></span>';
  try {
    const marketPrices = parseMarketPrices();
    const data = await apiPost('/options/portfolio/summary', {
      market_prices: marketPrices,
      risk_free_rate: 0.05,
    });
    renderPortfolio(data);
  } catch(e) {
    toast('Error portfolio: ' + e.message, 'err');
  } finally {
    btn.innerHTML = 'â†º Actualizar';
    State.loadingPortfolio = false;
  }
}

function parseMarketPrices() {
  const raw = document.getElementById('port-spot-input').value || '';
  const result = {};
  raw.split(',').forEach(pair => {
    const [sym, price] = pair.split(':');
    if (sym && price) result[sym.trim().toUpperCase()] = parseFloat(price.trim());
  });
  return result;
}

function renderPortfolio(data) {
  if (typeof data.mirror_only === 'boolean') {
    State.optionstratConfig = { ...State.optionstratConfig, mirror_only: data.mirror_only };
    applyOptionstratModeUI();
  }
  renderPortfolioSync(data.sync);
  // Total PnL
  const pnlEl = document.getElementById('pg-total-pnl');
  pnlEl.textContent = '$' + fmtSign(data.total_pnl);
  pnlEl.className = 'greek-value ' + (data.total_pnl >= 0 ? 'text-green' : 'text-red');

  // Greeks totals
  const gt = data.greeks?.total || {};
  document.getElementById('pg-delta').textContent = fmt(gt.delta, 3);
  document.getElementById('pg-gamma').textContent = fmt(gt.gamma, 4);
  const thetaEl = document.getElementById('pg-theta');
  thetaEl.textContent = fmtSign(gt.theta, 2);
  thetaEl.className = 'greek-value theta ' + (gt.theta < 0 ? 'text-red' : 'text-green');
  document.getElementById('pg-vega').textContent = fmt(gt.vega, 2);

  // Risk flags
  const flagsBox = document.getElementById('risk-flags-box');
  const flagsList = document.getElementById('risk-flags-list');
  if (data.risk_flags && data.risk_flags.length) {
    flagsBox.style.display = '';
    flagsList.innerHTML = data.risk_flags.map(f =>
      `<div style="color:var(--yellow);font-size:12px;font-family:var(--font-mono);margin:2px 0">âš  ${f}</div>`
    ).join('');
  } else {
    flagsBox.style.display = 'none';
  }

  // Strategies table
  const tbody = document.getElementById('portfolio-body');
  if (!data.entries || !data.entries.length) {
    tbody.innerHTML = '<tr class="empty-row"><td colspan="10">Sin estrategias abiertas</td></tr>';
  } else {
    tbody.innerHTML = data.entries.map(e => {
      const pnl = e.current_pnl;
      const pnlCls = pnl >= 0 ? 'td-pos' : 'td-neg';
      const mp = e.max_profit != null ? '$' + fmt(e.max_profit) : 'âˆž';
      const ml = e.max_loss != null ? '$' + fmt(e.max_loss) : '-âˆž';
      const opened = e.opened_at ? new Date(e.opened_at).toLocaleDateString('es-ES') : '--';
      return `<tr>
        <td>${e.name}</td>
        <td>${e.underlying || '--'}</td>
        <td><span class="tpl-bias ${e.bias || 'neutral'}">${e.bias || '--'}</span></td>
        <td class="${pnlCls}">${'$' + fmtSign(pnl)}</td>
        <td class="mono">${fmt(e.greeks?.delta, 3)}</td>
        <td class="mono ${(e.greeks?.theta||0) < 0 ? 'td-neg' : 'td-pos'}">${fmtSign(e.greeks?.theta, 2)}</td>
        <td class="td-pos">${mp}</td>
        <td class="td-neg">${ml}</td>
        <td class="td-muted">${opened}</td>
        <td><button class="btn-danger" data-close="${e.name}" ${State.optionstratConfig?.mirror_only ? 'disabled title="Modo espejo broker"' : ''}>âœ• Cerrar</button></td>
      </tr>`;
    }).join('');
    tbody.querySelectorAll('[data-close]').forEach(btn => {
      btn.addEventListener('click', () => closeStrategy(btn.dataset.close));
    });
  }

  // Charts
  renderPortfolioCharts(data);
  renderPortfolioHistory(data.history || []);
}

function renderPortfolioSync(sync) {
  const badge = document.getElementById('port-sync-badge');
  if (!badge) return;
  if (!sync) {
    badge.className = 'badge badge-closed';
    badge.textContent = 'SYNC --';
    return;
  }
  const changed = (sync.added || 0) + (sync.updated || 0) + (sync.closed || 0);
  badge.className = changed > 0 ? 'badge badge-open' : 'badge badge-closed';
  badge.textContent = `SYNC +${sync.added || 0}/~${sync.updated || 0}/-${sync.closed || 0}`;
}

function renderPortfolioHistory(historyRows) {
  const tbody = document.getElementById('portfolio-history-body');
  if (!tbody) return;
  if (!historyRows.length) {
    tbody.innerHTML = '<tr class="empty-row"><td colspan="5">Sin historial de cierres</td></tr>';
    return;
  }
  tbody.innerHTML = historyRows.map(row => {
    const opened = row.opened_at ? new Date(row.opened_at).toLocaleString('es-ES') : '--';
    const closed = row.closed_at ? new Date(row.closed_at).toLocaleString('es-ES') : '--';
    return `<tr>
      <td>${row.name || '--'}</td>
      <td>${row.underlying || '--'}</td>
      <td>${row.source || 'manual'}</td>
      <td class="td-muted">${opened}</td>
      <td class="td-muted">${closed}</td>
    </tr>`;
  }).join('');
}

async function closeStrategy(name) {
  if (State.optionstratConfig?.mirror_only) {
    toast('Modo espejo broker activo: cierre manual deshabilitado', 'err');
    return;
  }
  if (!confirm(`Â¿Cerrar estrategia "${name}"?`)) return;
  try {
    await apiDelete('/options/portfolio/' + encodeURIComponent(name));
    toast('Estrategia cerrada', 'ok');
    loadPortfolio();
  } catch(e) {
    toast('Error: ' + e.message, 'err');
  }
}

function renderPortfolioCharts(data) {
  // PnL by underlying
  const pnlCtx = document.getElementById('chart-port-pnl').getContext('2d');
  if (State.portPnlChart) State.portPnlChart.destroy();
  const underlying = Object.keys(data.by_underlying || {});
  const pnlVals = underlying.map(k => data.by_underlying[k]);
  State.portPnlChart = new Chart(pnlCtx, {
    type: 'bar',
    data: {
      labels: underlying.length ? underlying : ['--'],
      datasets: [{
        data: pnlVals.length ? pnlVals : [0],
        backgroundColor: pnlVals.map(v => v >= 0 ? 'rgba(72,187,120,0.6)' : 'rgba(245,101,101,0.6)'),
        borderRadius: 4,
      }],
    },
    options: {
      responsive: true, maintainAspectRatio: false,
      plugins: { legend: { display: false }, tooltip: {
        callbacks: { label: c => '$' + c.parsed.y.toFixed(2) }
      }},
      scales: {
        x: { ticks: { color: '#4a5568', font: { size: 10 } }, grid: { display: false } },
        y: { ticks: { color: '#4a5568', font: { size: 10 },
          callback: v => '$' + v.toFixed(0) }, grid: { color: '#1f2433' } },
      },
    },
  });

  // Bias donut
  const biasCtx = document.getElementById('chart-port-bias').getContext('2d');
  if (State.portBiasChart) State.portBiasChart.destroy();
  const biasKeys = Object.keys(data.by_bias || {});
  const biasVals = biasKeys.map(k => data.by_bias[k]);
  const biasColors = {
    bullish: '#48bb78', bearish: '#f56565', neutral: '#8892a4',
    very_bullish: '#00d4aa', very_bearish: '#fc8181', unknown: '#4a5568',
    neutral_bullish: '#9ae6b4', neutral_bearish: '#feb2b2',
  };
  State.portBiasChart = new Chart(biasCtx, {
    type: 'doughnut',
    data: {
      labels: biasKeys.length ? biasKeys : ['Sin datos'],
      datasets: [{
        data: biasVals.length ? biasVals : [1],
        backgroundColor: biasKeys.length
          ? biasKeys.map(k => biasColors[k] || '#4a5568')
          : ['#1f2433'],
        borderWidth: 0,
      }],
    },
    options: {
      responsive: true, maintainAspectRatio: false,
      plugins: {
        legend: { labels: { color: '#8892a4', font: { size: 10 }, boxWidth: 10 } },
      },
      cutout: '65%',
    },
  });
}

// â”€â”€ Wire up all events â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
function initEvents() {
  // Builder
  document.getElementById('btn-add-call').addEventListener('click', () => addOptionLeg('call'));
  document.getElementById('btn-add-put').addEventListener('click', () => addOptionLeg('put'));
  document.getElementById('btn-add-stock').addEventListener('click', addLinearLeg);
  document.getElementById('btn-preview').addEventListener('click', previewStrategy);
  document.getElementById('btn-clear-builder').addEventListener('click', clearBuilder);
  document.getElementById('btn-add-to-portfolio').addEventListener('click', addToPortfolio);

  // Templates
  document.getElementById('tpl-refresh').addEventListener('click', () => {
    renderTemplateGrid(State.templates);
    loadTemplates();
  });
  document.getElementById('tpl-filter-bias').addEventListener('change', () =>
    renderTemplateGrid(State.templates));
  document.getElementById('tpl-filter-cat').addEventListener('change', () =>
    renderTemplateGrid(State.templates));
  document.getElementById('btn-tpl-build').addEventListener('click', buildFromTemplate);
  document.getElementById('btn-tpl-cancel').addEventListener('click', () => {
    document.getElementById('tpl-build-panel').style.display = 'none';
    State.selectedTemplate = null;
    document.querySelectorAll('.template-card').forEach(c => c.classList.remove('selected'));
  });

  // Scenario
  document.getElementById('btn-run-scenario').addEventListener('click', runScenario);

  // Portfolio
  document.getElementById('btn-port-refresh').addEventListener('click', loadPortfolio);
}

// â”€â”€ Bootstrap â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
document.addEventListener('DOMContentLoaded', () => {
  initNav();
  initEvents();
  loadOptionstratConfig();
  // Seed a default leg to show UI
  addOptionLeg('call', 'long');
  addOptionLeg('put', 'long');
  renderLegs();
  toast('OptionStrat cargado', 'info', 2000);
});

