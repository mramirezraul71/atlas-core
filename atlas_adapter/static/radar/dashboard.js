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
    cameraHealth: "/api/radar/sensors/camera/health",
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
    cameraStatusSummary: document.getElementById("cameraStatusSummary"),
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

  /** Clasificaciones del API (snake_case) → etiqueta en español para la UI. */
  function clasificacionEs(code) {
    const c = String(code || "").trim().toLowerCase();
    const map = {
      fully_operable: "Totalmente operable",
      operable_with_degradation: "Operable con degradación",
      structural_only: "Solo contexto estructural",
      fast_only: "Solo contexto táctico (fast)",
      non_operable: "No operable",
      demonstration_without_engine: "Modo demostración (sin motor radar)",
      modo_demostracion_sin_motor: "Modo demostración (sin motor radar)",
      unknown: "Desconocido",
      desconocido: "Desconocido",
    };
    if (map[c]) return map[c];
    if (!c) return "Desconocido";
    return String(code).replaceAll("_", " ");
  }

  function sesgoEs(code) {
    const s = String(code || "").trim().toLowerCase();
    const map = {
      neutral: "neutral",
      bullish: "alcista",
      bearish: "bajista",
      long: "largo",
      short: "corto",
    };
    return map[s] || (code ? String(code) : "—");
  }

  function decisionEtiqueta(code) {
    const d = String(code || "").trim().toLowerCase();
    const map = {
      accepted: "aceptada",
      rejected: "rechazada",
      caution: "precaución",
      bypassed: "derivada",
    };
    return map[d] || (code ? String(code) : "sin dato");
  }

  function formatSiNo(v) {
    if (v === true || v === "true") return "Sí";
    if (v === false || v === "false") return "No";
    if (v === null || v === undefined || v === "") return "—";
    return String(v);
  }

  function ultimoErrorEs(raw) {
    const r = String(raw || "").trim();
    if (!r) return "—";
    if (r === "stub_backend" || r.includes("stub")) return "Modo demostración (sin motor)";
    if (r === "modo_demostracion_sin_motor") return "Modo demostración (sin motor)";
    return r.replaceAll("_", " ");
  }

  function tipoEventoStreamEs(type) {
    const t = String(type || "").toLowerCase();
    const map = {
      snapshot: "instantánea",
      decision: "decisión",
      provider_health: "salud de proveedores",
      degradation: "degradación",
      alert: "alerta",
      heartbeat: "latido",
      unknown: "desconocido",
    };
    return map[t] || t.replaceAll("_", " ");
  }

  function badgeHtml(kind, text) {
    return `<span class="badge badge-${kind}">${text}</span>`;
  }

  function decisionBadge(decision) {
    const map = {
      accepted: ["accepted", "Aceptado"],
      rejected: ["rejected", "Rechazado"],
      caution: ["caution", "Precaución"],
      bypassed: ["neutral", "Derivada"],
    };
    const [kind, text] = map[decision] || ["neutral", decision || "N/D"];
    return badgeHtml(kind, text);
  }

  function readinessBadge(isReady, stale, hint) {
    if (hint === "modo_demostracion") {
      return badgeHtml("neutral", "Modo demostración");
    }
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
      el.estadoFresh.textContent = "Sin marca de tiempo";
      return;
    }
    const ageMs = Date.now() - new Date(timestamp).valueOf();
    if (!Number.isFinite(ageMs)) {
      el.estadoFresh.className = "badge badge-neutral";
      el.estadoFresh.textContent = "Marca de tiempo inválida";
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
    const res = await fetch(url, {
      headers: { Accept: "application/json" },
      cache: "no-store",
    });
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
      const [summaryRes, providersRes, decisionsRes, statsRes, dealerRes, fastRes, structuralRes, politicalRes, cameraRes] =
        await Promise.all([
          fetchJson(endpoints.summary(symbol)),
          fetchJson(endpoints.providers),
          fetchJson(endpoints.decisionsRecent),
          fetchJson(endpoints.decisionsStats),
          fetchJson(endpoints.dealer(symbol)),
          fetchJson(endpoints.fast(symbol)),
          fetchJson(endpoints.structural(symbol)),
          fetchJson(endpoints.political(symbol)),
          fetchJson(endpoints.cameraHealth),
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
      renderPrincipal(summaryRes.data, providersRes.data, decisionsRes.data, dealerRes.data, fastRes.data, structuralRes.data, cameraRes.data);
      renderDecisions(decisionsRes.data);
      renderProviders(providersRes.data, statsRes.data);
      renderSymbol(symbol, summaryRes.data, dealerRes.data, fastRes.data, structuralRes.data, politicalRes.data);
      if (!state.streamConnected) {
        setLiveStatus("Encuesta periódica");
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

  function renderPrincipal(summary, providers, decisions, dealer, fast, structural, cameraHealth) {
    const radar = summary?.radar ?? {};
    const signal = radar?.signal ?? {};
    const meta = signal?.meta ?? {};
    const classification = meta?.snapshot_classification || "unknown";

    if (classification === "fully_operable") {
      el.estadoGlobal.className = "badge badge-accepted";
    } else if (classification === "non_operable") {
      el.estadoGlobal.className = "badge badge-rejected";
    } else if (
      classification === "demonstration_without_engine" ||
      classification === "modo_demostracion_sin_motor"
    ) {
      el.estadoGlobal.className = "badge badge-neutral";
    } else {
      el.estadoGlobal.className = "badge badge-caution";
    }
    el.estadoGlobal.textContent = clasificacionEs(classification);

    setFreshness(signal?.timestamp);

    el.kpiClassification.textContent = clasificacionEs(classification);
    el.kpiFastPressure.textContent = formatNumber(meta?.fast_pressure_score ?? meta?.fast_composite_pressure);
    el.kpiStructural.textContent = formatNumber(meta?.structural_confidence_score ?? meta?.structural_composite_confidence);
    el.kpiAlignment.textContent = formatNumber(meta?.fast_structural_alignment);
    el.kpiDivergence.textContent = formatNumber(meta?.fast_structural_divergence_score);
    el.kpiConflict.textContent = formatSiNo(meta?.horizon_conflict);

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
            <td>${clasificacionEs(evaluation?.snapshot_classification)}</td>
            <td>${sesgoEs(evaluation?.bias)}</td>
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
            <td>${readinessBadge(ready, stale, provider?.ui_status_hint)}</td>
            <td class="metric">${formatNumber(provider?.latency_ms ?? provider?.current_latency_ms, 0)} ms</td>
            <td class="metric">${formatNumber(provider?.p95_latency_ms, 0)} ms</td>
            <td>${provider?.active_fallback_indicator ? badgeHtml("caution", "Activo") : badgeHtml("neutral", "No")}</td>
            <td>${provider?.circuit_open_indicator ? badgeHtml("rejected", "Abierto") : badgeHtml("accepted", "Cerrado")}</td>
            <td class="metric">${formatPct(provider?.availability_ratio)}</td>
            <td>${ultimoErrorEs(provider?.last_error_type)}</td>
          </tr>
        `;
      })
      .join("");

    renderKvs(el.dealerFastSummary, [
      ["Nivel gamma (flip)", dealer?.gamma_flip_level ?? "-"],
      ["Sesgo dealer", formatNumber(dealer?.dealer_skew_score)],
      ["Muro de calls", dealer?.call_wall ?? "-"],
      ["Muro de puts", dealer?.put_wall ?? "-"],
      ["Presión táctica (fast)", formatNumber(fast?.fast_pressure_score)],
      ["Riesgo táctico (fast)", formatNumber(fast?.fast_risk_score)],
    ]);

    renderKvs(el.structuralSummary, [
      ["Confianza estructural", formatNumber(structural?.structural_confidence_score)],
      ["Componente alcista", formatNumber(structural?.structural_bullish_score)],
      ["Componente bajista", formatNumber(structural?.structural_bearish_score)],
      ["Alineación multi‑marco", String(meta?.cross_horizon_alignment ?? "-")],
      ["Clasificación", clasificacionEs(classification)],
      ["Marca de tiempo de la señal", formatTs(signal?.timestamp)],
    ]);

    const cam = cameraHealth?.camera || summary?.camera_context || {};
    renderKvs(el.cameraStatusSummary, [
      ["Proveedor de cámara", cam?.provider || "—"],
      ["Estado", cam?.status || "—"],
      ["Listo", formatSiNo(cam?.provider_ready)],
      ["Última captura", formatTs(cam?.last_capture)],
      ["Presencia", formatNumber(cam?.presence_score)],
      ["Actividad", formatNumber(cam?.activity_level)],
    ]);
  }

  function renderAlerts(meta, latestDecision) {
    const alerts = [];
    if (meta?.horizon_conflict) {
      alerts.push("Conflicto entre señal táctica (fast) y estructural.");
    }
    if ((meta?.fast_structural_divergence_score ?? 0) > 0.6) {
      alerts.push("Divergencia de horizonte elevada.");
    }
    if (latestDecision?.decision === "rejected") {
      alerts.push(`La compuerta rechazó la última señal: ${latestDecision?.reason || "sin motivo detallado"}.`);
    }
    if (alerts.length === 0) {
      alerts.push("Sin alertas críticas. Operación en régimen normal.");
    }
    el.alertasRapidas.innerHTML = alerts.map((text) => `<div class="alert-item">${text}</div>`).join("");
  }

  function buildExecutiveSummary(signal, meta, latestDecision) {
    const bias = sesgoEs(signal?.bias || "neutral");
    const classification = clasificacionEs(meta?.snapshot_classification || "desconocido");
    const fast = formatNumber(meta?.fast_pressure_score ?? meta?.fast_composite_pressure);
    const structural = formatNumber(meta?.structural_confidence_score ?? meta?.structural_composite_confidence);
    const gate = latestDecision?.decision
      ? `Compuerta: decisión ${decisionEtiqueta(latestDecision.decision)}.`
      : "Compuerta: sin decisión reciente.";
    return `Estado: ${classification}. Sesgo: ${bias}. Presión táctica (fast) ${fast} y confianza estructural ${structural}. ${gate}`;
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
            <td>${clasificacionEs(ev?.snapshot_classification)}</td>
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
      kpiCard("Proveedores totales", String(total)),
      kpiCard("Activos", String(ready)),
      kpiCard("Degradados", String(degraded)),
      kpiCard("Respaldo activo", String(fallback)),
    ].join("");

    el.tablaProvidersFull.innerHTML = list
      .map((provider) => {
        const readyBadge = readinessBadge(
          Boolean(provider?.is_ready ?? provider?.ready),
          provider?.stale_indicator,
          provider?.ui_status_hint
        );
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
            <td>${ultimoErrorEs(provider?.last_error_type)}</td>
          </tr>
        `;
      })
      .join("");

    if (stats?.stats) {
      const s = stats.stats;
      el.providersMetrics.innerHTML += [
        kpiCard("Decisiones aceptadas", String(s?.by_decision?.accepted ?? 0)),
        kpiCard("Decisiones rechazadas", String(s?.by_decision?.rejected ?? 0)),
        kpiCard("Promedio presión táctica (fast)", formatNumber(s?.avg_fast_pressure_score)),
        kpiCard("Promedio confianza estructural", formatNumber(s?.avg_structural_confidence_score)),
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
      ["Zona de aceleración dealer", formatNumber(dealer?.acceleration_zone_score)],
      ["Presión táctica (fast)", formatNumber(fast?.fast_pressure_score)],
      ["Sesgo direccional (fast)", formatNumber(fast?.fast_directional_bias_score)],
      ["Confianza estructural", formatNumber(structural?.structural_confidence_score)],
      ["Agregado político", formatNumber(political?.aggregate_signal_score)],
      ["Última decisión (compuerta)", latestDecision?.decision ? decisionEtiqueta(latestDecision.decision) : "—"],
      ["Marca de tiempo dealer", formatTs(dealer?.timestamp)],
    ];
    el.symbolCards.innerHTML = cards.map(([k, v]) => kpiCard(k, String(v))).join("");
  }

  function buildSymbolNarrative(symbol, dealer, fast, structural, political, latestDecision) {
    return `${symbol}: sesgo dealer ${formatNumber(dealer?.dealer_skew_score)}, presión táctica (fast) ${formatNumber(
      fast?.fast_pressure_score
    )}, confianza estructural ${formatNumber(structural?.structural_confidence_score)} y agregado político ${formatNumber(
      political?.aggregate_signal_score
    )}. Última decisión de la compuerta: ${latestDecision?.decision ? decisionEtiqueta(latestDecision.decision) : "no disponible"}.`;
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
      setLiveStatus(state.streamConnected ? "Transmisión en vivo (SSE)" : "Sin señal");
      return;
    }
    const interval = Number(el.refreshIntervalSelect.value || "10000");
    state.timer = setInterval(refreshAll, interval);
    if (!state.streamConnected) {
      setLiveStatus("Encuesta periódica");
    }
  }

  function setLiveStatus(text) {
    el.liveMode.textContent = text;
  }

  function markLastEvent(label) {
    const ts = nowLabel();
    state.lastEventAt = ts;
    el.liveLastEvent.textContent = `${label} · ${ts}`;
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
    markLastEvent(tipoEventoStreamEs(type));
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
      const msg = payload?.message || `${tipoEventoStreamEs(type)} recibido`;
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
      setLiveStatus("Encuesta periódica");
      setupAutoRefresh();
      return;
    }
    teardownStream();
    try {
      const source = new EventSource(endpoints.stream);
      state.stream = source;
      source.onopen = () => {
        state.streamConnected = true;
        setLiveStatus("Transmisión en vivo (SSE)");
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
            markLastEvent(`${tipoEventoStreamEs(name)} (payload inválido)`);
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
      setLiveStatus("Encuesta periódica");
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
