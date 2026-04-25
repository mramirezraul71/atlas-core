(function () {
  const state = {
    activeTab: "principal",
    symbol: "SPY",
    timer: null,
    lastSummary: null,
    stream: null,
    streamConnected: false,
    lastHeartbeatMs: 0,
    lastEventAt: null,
    staleTimer: null,
  };

  const endpoints = {
    summary: (symbol) => `/api/radar/dashboard/summary?symbol=${encodeURIComponent(symbol)}`,
    providers: "/api/radar/diagnostics/providers",
    decisionsRecent: "/api/radar/decisions/recent?limit=40",
    decisionsStats: "/api/radar/decisions/stats",
    dealer: (symbol) => `/api/radar/dealer/${encodeURIComponent(symbol)}`,
    fast: (symbol) => `/api/radar/diagnostics/fast/${encodeURIComponent(symbol)}`,
    structural: (symbol) => `/api/radar/diagnostics/structural/${encodeURIComponent(symbol)}`,
    political: (symbol) => `/api/radar/political/${encodeURIComponent(symbol)}`,
    stream: "/api/radar/stream",
  };

  const el = {
    refreshBtn: document.getElementById("refreshBtn"),
    symbolInput: document.getElementById("symbolInput"),
    autoRefreshToggle: document.getElementById("autoRefreshToggle"),
    refreshIntervalSelect: document.getElementById("refreshIntervalSelect"),
    ultimaActualizacion: document.getElementById("ultimaActualizacion"),
    latenciaActual: document.getElementById("latenciaActual"),
    estadoGlobal: document.getElementById("estadoGlobal"),
    estadoFresh: document.getElementById("estadoFresh"),
    commandInput: document.getElementById("comandoGlobal"),
    commandHint: document.getElementById("comandoHint"),
    resumenEjecutivo: document.getElementById("resumenEjecutivo"),
    alertasRapidas: document.getElementById("alertasRapidas"),
    kpiClassification: document.getElementById("kpiClassification"),
    kpiFastPressure: document.getElementById("kpiFastPressure"),
    kpiStructural: document.getElementById("kpiStructural"),
    kpiAlignment: document.getElementById("kpiAlignment"),
    kpiDivergence: document.getElementById("kpiDivergence"),
    kpiConflict: document.getElementById("kpiConflict"),
    tablaSignals: document.getElementById("tablaSignals"),
    tablaProviders: document.getElementById("tablaProviders"),
    dealerFastSummary: document.getElementById("dealerFastSummary"),
    structuralSummary: document.getElementById("structuralSummary"),
    tablaDecisiones: document.getElementById("tablaDecisiones"),
    providersMetrics: document.getElementById("providersMetrics"),
    tablaProvidersFull: document.getElementById("tablaProvidersFull"),
    symbolNarrative: document.getElementById("symbolNarrative"),
    symbolCards: document.getElementById("symbolCards"),
    liveMode: document.getElementById("liveMode"),
    liveLastEvent: document.getElementById("liveLastEvent"),
    liveHeartbeat: document.getElementById("liveHeartbeat"),
  };

  function formatNumber(value, digits = 2) {
    if (value === null || value === undefined || value === "") return "-";
    const num = Number(value);
    if (!Number.isFinite(num)) return String(value);
    return num.toFixed(digits);
  }

  function formatPct(value) {
    if (value === null || value === undefined) return "-";
    const num = Number(value);
    if (!Number.isFinite(num)) return "-";
    return `${(num * 100).toFixed(1)}%`;
  }

  function formatTs(ts) {
    if (!ts) return "-";
    const d = new Date(ts);
    if (Number.isNaN(d.valueOf())) return String(ts);
    return d.toLocaleString("es-ES", { hour12: false });
  }

  function nowLabel() {
    return new Date().toLocaleString("es-ES", { hour12: false });
  }

  function badgeHtml(kind, text) {
    return `<span class="badge badge-${kind}">${text}</span>`;
  }

  function decisionBadge(decision) {
    const map = {
      accepted: ["accepted", "Aceptado"],
      rejected: ["rejected", "Rechazado"],
      caution: ["caution", "Precaución"],
      bypassed: ["neutral", "Bypassed"],
    };
    const [kind, text] = map[decision] || ["neutral", decision || "N/D"];
    return badgeHtml(kind, text);
  }

  function readinessBadge(isReady, stale) {
    if (stale) return badgeHtml("stale", "Obsoleto");
    return isReady ? badgeHtml("accepted", "Activo") : badgeHtml("caution", "Degradado");
  }

  function clearNode(node) {
    while (node.firstChild) node.removeChild(node.firstChild);
  }

  function renderKvs(target, pairs) {
    clearNode(target);
    pairs.forEach(([key, value]) => {
      const box = document.createElement("div");
      box.className = "kv-item";
      box.innerHTML = `<div class="kv-key">${key}</div><div class="kv-val">${value ?? "-"}</div>`;
      target.appendChild(box);
    });
  }

  function setFreshness(timestamp) {
    if (!timestamp) {
      el.estadoFresh.className = "badge badge-neutral";
      el.estadoFresh.textContent = "Sin timestamp";
      return;
    }
    const ageMs = Date.now() - new Date(timestamp).valueOf();
    if (!Number.isFinite(ageMs)) {
      el.estadoFresh.className = "badge badge-neutral";
      el.estadoFresh.textContent = "Timestamp inválido";
      return;
    }
    if (ageMs > 120000) {
      el.estadoFresh.className = "badge badge-stale";
      el.estadoFresh.textContent = "Datos obsoletos";
    } else if (ageMs > 45000) {
      el.estadoFresh.className = "badge badge-caution";
      el.estadoFresh.textContent = "Datos envejecidos";
    } else {
      el.estadoFresh.className = "badge badge-accepted";
      el.estadoFresh.textContent = "Datos frescos";
    }
  }

  async function fetchJson(url) {
    const started = performance.now();
    const res = await fetch(url, { headers: { Accept: "application/json" } });
    const elapsed = performance.now() - started;
    if (!res.ok) {
      throw new Error(`${res.status} ${res.statusText}`);
    }
    return { data: await res.json(), elapsed };
  }

  async function refreshAll() {
    const symbol = (el.symbolInput.value || "SPY").trim().toUpperCase();
    state.symbol = symbol;
    const startedLabel = nowLabel();
    el.refreshBtn.disabled = true;
    el.refreshBtn.textContent = "Actualizando...";
    try {
      const [summaryRes, providersRes, decisionsRes, statsRes, dealerRes, fastRes, structuralRes, politicalRes] =
        await Promise.all([
          fetchJson(endpoints.summary(symbol)),
          fetchJson(endpoints.providers),
          fetchJson(endpoints.decisionsRecent),
          fetchJson(endpoints.decisionsStats),
          fetchJson(endpoints.dealer(symbol)),
          fetchJson(endpoints.fast(symbol)),
          fetchJson(endpoints.structural(symbol)),
          fetchJson(endpoints.political(symbol)),
        ]);

      const latency = Math.max(
        summaryRes.elapsed,
        providersRes.elapsed,
        decisionsRes.elapsed,
        dealerRes.elapsed,
        fastRes.elapsed,
        structuralRes.elapsed
      );
      el.latenciaActual.textContent = `${latency.toFixed(0)} ms`;
      el.ultimaActualizacion.textContent = startedLabel;

      state.lastSummary = summaryRes.data;
      renderPrincipal(summaryRes.data, providersRes.data, decisionsRes.data, dealerRes.data, fastRes.data, structuralRes.data);
      renderDecisions(decisionsRes.data);
      renderProviders(providersRes.data, statsRes.data);
      renderSymbol(symbol, summaryRes.data, dealerRes.data, fastRes.data, structuralRes.data, politicalRes.data);
      if (!state.streamConnected) {
        setLiveStatus("Polling activo");
      }
    } catch (error) {
      el.latenciaActual.textContent = "-";
      el.ultimaActualizacion.textContent = startedLabel;
      el.estadoGlobal.className = "badge badge-rejected";
      el.estadoGlobal.textContent = "Error de actualización";
      el.estadoFresh.className = "badge badge-stale";
      el.estadoFresh.textContent = String(error.message || error);
      el.alertasRapidas.innerHTML = `<div class="alert-item">No fue posible actualizar: ${String(
        error.message || error
      )}</div>`;
    } finally {
      el.refreshBtn.disabled = false;
      el.refreshBtn.textContent = "Actualizar";
    }
  }

  function renderPrincipal(summary, providers, decisions, dealer, fast, structural) {
    const radar = summary?.radar ?? {};
    const signal = radar?.signal ?? {};
    const meta = signal?.meta ?? {};
    const classification = meta?.snapshot_classification || "unknown";

    el.estadoGlobal.className =
      classification === "fully_operable"
        ? "badge badge-accepted"
        : classification === "non_operable"
        ? "badge badge-rejected"
        : "badge badge-caution";
    el.estadoGlobal.textContent = classification.replaceAll("_", " ");

    setFreshness(signal?.timestamp);

    el.kpiClassification.textContent = classification;
    el.kpiFastPressure.textContent = formatNumber(meta?.fast_pressure_score ?? meta?.fast_composite_pressure);
    el.kpiStructural.textContent = formatNumber(meta?.structural_confidence_score ?? meta?.structural_composite_confidence);
    el.kpiAlignment.textContent = formatNumber(meta?.fast_structural_alignment);
    el.kpiDivergence.textContent = formatNumber(meta?.fast_structural_divergence_score);
    el.kpiConflict.textContent = String(meta?.horizon_conflict ?? "-");

    el.resumenEjecutivo.textContent = buildExecutiveSummary(signal, meta, summary?.decision_gate?.latest);
    renderAlerts(meta, summary?.decision_gate?.latest);

    const recentSignals = summary?.decision_gate?.recent || decisions?.recent || [];
    el.tablaSignals.innerHTML = recentSignals
      .slice(0, 20)
      .map((item) => {
        const evaluation = item?.evaluation || item;
        return `
          <tr>
            <td class="metric">${evaluation?.symbol || "-"}</td>
            <td>${evaluation?.timeframe || "-"}</td>
            <td>${evaluation?.snapshot_classification || "-"}</td>
            <td>${evaluation?.bias || "-"}</td>
            <td class="score">${formatNumber(evaluation?.fast_pressure_score)} / ${formatNumber(
          evaluation?.structural_confidence_score
        )}</td>
            <td>${decisionBadge(evaluation?.decision)}</td>
            <td class="metric">${formatTs(evaluation?.timestamp)}</td>
          </tr>
        `;
      })
      .join("");

    const providerList = providers?.providers || providers?.status || [];
    el.tablaProviders.innerHTML = providerList
      .slice(0, 16)
      .map((provider) => {
        const ready = Boolean(provider?.is_ready ?? provider?.ready);
        const stale = Boolean(provider?.stale_indicator ?? provider?.is_stale);
        return `
          <tr>
            <td>${provider?.provider || provider?.name || "-"}</td>
            <td>${readinessBadge(ready, stale)}</td>
            <td class="metric">${formatNumber(provider?.latency_ms ?? provider?.current_latency_ms, 0)} ms</td>
            <td class="metric">${formatNumber(provider?.p95_latency_ms, 0)} ms</td>
            <td>${provider?.active_fallback_indicator ? badgeHtml("caution", "Activo") : badgeHtml("neutral", "No")}</td>
            <td>${provider?.circuit_open_indicator ? badgeHtml("rejected", "Abierto") : badgeHtml("accepted", "Cerrado")}</td>
            <td class="metric">${formatPct(provider?.availability_ratio)}</td>
            <td>${provider?.last_error_type || "-"}</td>
          </tr>
        `;
      })
      .join("");

    renderKvs(el.dealerFastSummary, [
      ["Gamma flip", dealer?.gamma_flip_level ?? "-"],
      ["Dealer skew", formatNumber(dealer?.dealer_skew_score)],
      ["Call wall", dealer?.call_wall ?? "-"],
      ["Put wall", dealer?.put_wall ?? "-"],
      ["Fast pressure", formatNumber(fast?.fast_pressure_score)],
      ["Riesgo fast", formatNumber(fast?.fast_risk_score)],
    ]);

    renderKvs(el.structuralSummary, [
      ["Confianza structural", formatNumber(structural?.structural_confidence_score)],
      ["Bullish structural", formatNumber(structural?.structural_bullish_score)],
      ["Bearish structural", formatNumber(structural?.structural_bearish_score)],
      ["Alineación cross-horizon", String(meta?.cross_horizon_alignment ?? "-")],
      ["Clasificación", classification],
      ["Timestamp señal", formatTs(signal?.timestamp)],
    ]);
  }

  function renderAlerts(meta, latestDecision) {
    const alerts = [];
    if (meta?.horizon_conflict) {
      alerts.push("Conflicto fast vs structural activo.");
    }
    if ((meta?.fast_structural_divergence_score ?? 0) > 0.6) {
      alerts.push("Divergencia de horizonte elevada.");
    }
    if (latestDecision?.decision === "rejected") {
      alerts.push(`Gate rechazó última señal: ${latestDecision?.reason || "sin razón detallada"}.`);
    }
    if (alerts.length === 0) {
      alerts.push("Sin alertas críticas. Operación en régimen normal.");
    }
    el.alertasRapidas.innerHTML = alerts.map((text) => `<div class="alert-item">${text}</div>`).join("");
  }

  function buildExecutiveSummary(signal, meta, latestDecision) {
    const bias = signal?.bias || "neutral";
    const classification = meta?.snapshot_classification || "desconocido";
    const fast = formatNumber(meta?.fast_pressure_score ?? meta?.fast_composite_pressure);
    const structural = formatNumber(meta?.structural_confidence_score ?? meta?.structural_composite_confidence);
    const gate = latestDecision?.decision ? `Gate: ${latestDecision.decision}` : "Gate sin decisión reciente";
    return `Estado ${classification}. Sesgo ${bias}. Fast pressure ${fast} y confianza estructural ${structural}. ${gate}.`;
  }

  function renderDecisions(decisions) {
    const items = decisions?.recent || [];
    el.tablaDecisiones.innerHTML = items
      .map((entry) => {
        const ev = entry?.evaluation || entry;
        return `
          <tr>
            <td class="metric">${ev?.symbol || "-"}</td>
            <td>${ev?.timeframe || "-"}</td>
            <td>${ev?.snapshot_classification || "-"}</td>
            <td class="score">${formatNumber(ev?.fast_pressure_score)}</td>
            <td class="score">${formatNumber(ev?.structural_confidence_score)}</td>
            <td>${decisionBadge(ev?.decision)}</td>
            <td>${ev?.reason || "-"}</td>
            <td class="metric">${formatTs(ev?.timestamp)}</td>
          </tr>
        `;
      })
      .join("");
  }

  function renderProviders(providers, stats) {
    const list = providers?.providers || providers?.status || [];
    const total = list.length;
    const ready = list.filter((item) => Boolean(item?.is_ready ?? item?.ready)).length;
    const degraded = list.filter((item) => !Boolean(item?.is_ready ?? item?.ready)).length;
    const fallback = list.filter((item) => Boolean(item?.active_fallback_indicator)).length;

    el.providersMetrics.innerHTML = [
      kpiCard("Providers totales", String(total)),
      kpiCard("Activos", String(ready)),
      kpiCard("Degradados", String(degraded)),
      kpiCard("Fallback activo", String(fallback)),
    ].join("");

    el.tablaProvidersFull.innerHTML = list
      .map((provider) => {
        const readyBadge = readinessBadge(Boolean(provider?.is_ready ?? provider?.ready), provider?.stale_indicator);
        const circuit = provider?.circuit_open_indicator ? badgeHtml("rejected", "Abierto") : badgeHtml("accepted", "Cerrado");
        return `
          <tr>
            <td>${provider?.provider || provider?.name || "-"}</td>
            <td>${readyBadge}</td>
            <td>${provider?.active_fallback_indicator ? badgeHtml("caution", "Sí") : badgeHtml("neutral", "No")}</td>
            <td>${circuit}</td>
            <td class="metric">${formatNumber(provider?.latency_ms ?? provider?.current_latency_ms, 0)} ms</td>
            <td class="metric">${formatNumber(provider?.p95_latency_ms, 0)} ms</td>
            <td class="metric">${provider?.consecutive_errors ?? 0}</td>
            <td class="metric">${formatPct(provider?.availability_ratio)}</td>
            <td>${provider?.last_error_type || "-"}</td>
          </tr>
        `;
      })
      .join("");

    if (stats?.stats) {
      const s = stats.stats;
      el.providersMetrics.innerHTML += [
        kpiCard("Decisiones accepted", String(s?.by_decision?.accepted ?? 0)),
        kpiCard("Decisiones rejected", String(s?.by_decision?.rejected ?? 0)),
        kpiCard("Promedio fast", formatNumber(s?.avg_fast_pressure_score)),
        kpiCard("Promedio structural", formatNumber(s?.avg_structural_confidence_score)),
      ].join("");
    }
  }

  function kpiCard(label, value) {
    return `<article class="kpi"><div class="kpi-label">${label}</div><div class="kpi-value metric">${value}</div></article>`;
  }

  function renderSymbol(symbol, summary, dealer, fast, structural, political) {
    const latestDecision = summary?.decision_gate?.latest;
    el.symbolNarrative.textContent = buildSymbolNarrative(symbol, dealer, fast, structural, political, latestDecision);

    const cards = [
      ["Símbolo", symbol],
      ["Dealer acceleration", formatNumber(dealer?.acceleration_zone_score)],
      ["Fast pressure", formatNumber(fast?.fast_pressure_score)],
      ["Fast directional bias", formatNumber(fast?.fast_directional_bias_score)],
      ["Structural confidence", formatNumber(structural?.structural_confidence_score)],
      ["Political aggregate", formatNumber(political?.aggregate_signal_score)],
      ["Última decisión gate", latestDecision?.decision || "-"],
      ["Timestamp dealer", formatTs(dealer?.timestamp)],
    ];
    el.symbolCards.innerHTML = cards.map(([k, v]) => kpiCard(k, String(v))).join("");
  }

  function buildSymbolNarrative(symbol, dealer, fast, structural, political, latestDecision) {
    return `${symbol}: dealer skew ${formatNumber(dealer?.dealer_skew_score)}, presión fast ${formatNumber(
      fast?.fast_pressure_score
    )}, confianza structural ${formatNumber(structural?.structural_confidence_score)} y señal política ${formatNumber(
      political?.aggregate_signal_score
    )}. Decisión más reciente del gate: ${latestDecision?.decision || "no disponible"}.`;
  }

  function setupTabs() {
    const tabButtons = Array.from(document.querySelectorAll(".tab"));
    const panels = {
      principal: document.getElementById("tab-principal"),
      decisiones: document.getElementById("tab-decisiones"),
      providers: document.getElementById("tab-providers"),
      simbolo: document.getElementById("tab-simbolo"),
    };
    tabButtons.forEach((btn) => {
      btn.addEventListener("click", () => {
        state.activeTab = btn.dataset.tab;
        tabButtons.forEach((node) => node.classList.remove("active"));
        btn.classList.add("active");
        Object.entries(panels).forEach(([name, panel]) => {
          panel.classList.toggle("active", name === state.activeTab);
        });
      });
    });
  }

  function executeCommand(raw) {
    const cmd = raw.trim().toLowerCase();
    if (!cmd) return;
    if (cmd.includes("providers degradados")) {
      document.querySelector('[data-tab="providers"]').click();
      filterProviderRows(true);
      return;
    }
    if (cmd.includes("señales rechazadas") || cmd.includes("senales rechazadas")) {
      document.querySelector('[data-tab="decisiones"]').click();
      filterDecisionRows("rejected");
      return;
    }
    if (cmd.startsWith("abrir dealer de ")) {
      const symbol = raw.slice("abrir dealer de ".length).trim().toUpperCase();
      if (symbol) {
        el.symbolInput.value = symbol;
        document.querySelector('[data-tab="simbolo"]').click();
        refreshAll();
      }
      return;
    }
    if (cmd.includes("actualizar ahora")) {
      refreshAll();
      return;
    }
    el.commandHint.textContent = "Comando no reconocido. Ejemplos: providers degradados | señales rechazadas | abrir dealer de SPY";
  }

  function filterProviderRows(onlyDegraded) {
    const rows = Array.from(el.tablaProvidersFull.querySelectorAll("tr"));
    rows.forEach((row) => {
      const text = row.textContent.toLowerCase();
      if (!onlyDegraded) {
        row.style.display = "";
        return;
      }
      row.style.display = text.includes("degradado") || text.includes("obsoleto") ? "" : "none";
    });
  }

  function filterDecisionRows(targetDecision) {
    const rows = Array.from(el.tablaDecisiones.querySelectorAll("tr"));
    rows.forEach((row) => {
      const text = row.textContent.toLowerCase();
      row.style.display = text.includes(targetDecision) || text.includes("rechazado") ? "" : "none";
    });
  }

  function setupAutoRefresh() {
    if (state.timer) {
      clearInterval(state.timer);
      state.timer = null;
    }
    if (!el.autoRefreshToggle.checked) {
      setLiveStatus(state.streamConnected ? "Streaming activo" : "Sin señal");
      return;
    }
    const interval = Number(el.refreshIntervalSelect.value || "10000");
    state.timer = setInterval(refreshAll, interval);
    if (!state.streamConnected) {
      setLiveStatus("Polling activo");
    }
  }

  function setLiveStatus(text) {
    el.liveMode.textContent = text;
  }

  function markLastEvent(label) {
    const ts = nowLabel();
    state.lastEventAt = ts;
    el.liveLastEvent.textContent = `${label} @ ${ts}`;
  }

  function markHeartbeat() {
    state.lastHeartbeatMs = Date.now();
    el.liveHeartbeat.textContent = nowLabel();
  }

  function ensureStreamHealthWatchdog() {
    if (state.staleTimer) {
      clearInterval(state.staleTimer);
    }
    state.staleTimer = setInterval(() => {
      if (!state.streamConnected || !state.lastHeartbeatMs) return;
      const age = Date.now() - state.lastHeartbeatMs;
      if (age > 25000) {
        setLiveStatus("Conexión degradada");
        teardownStream();
        setupAutoRefresh();
      }
    }, 5000);
  }

  function handleStreamEvent(type, payload) {
    if (type === "heartbeat") {
      markHeartbeat();
      return;
    }
    markLastEvent(type);
    if (type === "snapshot") {
      state.symbol = payload?.symbol || state.symbol;
      refreshAll();
      return;
    }
    if (type === "decision") {
      refreshAll();
      return;
    }
    if (type === "provider_health" || type === "degradation" || type === "alert") {
      const msg = payload?.message || `${type} recibido`;
      el.alertasRapidas.innerHTML = `<div class="alert-item">${msg}</div>${el.alertasRapidas.innerHTML}`;
      refreshAll();
    }
  }

  function teardownStream() {
    if (state.stream) {
      state.stream.close();
      state.stream = null;
    }
    state.streamConnected = false;
  }

  function setupStreamingWithFallback() {
    if (typeof EventSource === "undefined") {
      setLiveStatus("Polling activo");
      setupAutoRefresh();
      return;
    }
    teardownStream();
    try {
      const source = new EventSource(endpoints.stream);
      state.stream = source;
      source.onopen = () => {
        state.streamConnected = true;
        setLiveStatus("Streaming activo");
        markHeartbeat();
        if (state.timer) {
          clearInterval(state.timer);
          state.timer = null;
        }
      };
      source.onmessage = (event) => {
        try {
          const parsed = JSON.parse(event.data);
          handleStreamEvent(parsed?.type || "unknown", parsed?.payload || {});
        } catch (_error) {
          markLastEvent("evento-inválido");
        }
      };
      const typedEvents = ["snapshot", "decision", "provider_health", "degradation", "alert", "heartbeat"];
      typedEvents.forEach((name) => {
        source.addEventListener(name, (event) => {
          try {
            const parsed = JSON.parse(event.data);
            handleStreamEvent(name, parsed?.payload || {});
          } catch (_error) {
            markLastEvent(`${name}-inválido`);
          }
        });
      });
      source.onerror = () => {
        teardownStream();
        setLiveStatus("Conexión degradada");
        setupAutoRefresh();
      };
      ensureStreamHealthWatchdog();
    } catch (_error) {
      teardownStream();
      setLiveStatus("Polling activo");
      setupAutoRefresh();
    }
  }

  function setupInteractions() {
    el.refreshBtn.addEventListener("click", refreshAll);
    el.symbolInput.addEventListener("change", refreshAll);
    el.autoRefreshToggle.addEventListener("change", setupAutoRefresh);
    el.refreshIntervalSelect.addEventListener("change", setupAutoRefresh);

    el.commandInput.addEventListener("keydown", (event) => {
      if (event.key === "Enter") {
        executeCommand(el.commandInput.value);
      }
    });

    document.addEventListener("keydown", (event) => {
      if (event.ctrlKey && event.key.toLowerCase() === "k") {
        event.preventDefault();
        el.commandInput.focus();
      }
    });
  }

  setupTabs();
  setupInteractions();
  setupStreamingWithFallback();
  if (!state.streamConnected) {
    setupAutoRefresh();
  }
  refreshAll();
})();
