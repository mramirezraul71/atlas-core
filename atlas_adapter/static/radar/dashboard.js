(function () {
  /** Sin latido SSE en este intervalo → desconectado / reconexión. */
  const SSE_HEARTBEAT_DEAD_MS = 42000;
  /** Sin snapshot útil (merge o refresh) → stale. */
  const SNAPSHOT_USEFUL_STALE_MS = 48000;
  const SESSION_SYMBOL_KEY = "atlas_radar_active_symbol";
  const SEARCH_DEBOUNCE_MS = 420;
  const SEARCH_MIN_CHARS = 1;
  const DEFAULT_SYMBOL_FALLBACK = "SPY";

  const state = {
    activeTab: "principal",
    symbol: "",
    timer: null,
    lastSummary: null,
    stream: null,
    streamConnected: false,
    lastHeartbeatMs: 0,
    lastEventAt: null,
    staleTimer: null,
    sseReconnectTimer: null,
    sseBackoffMs: 3000,
    cameraPollTimer: null,
    liveQualityTicker: null,
    lastSnapshotUsefulAt: 0,
    lastQuantReachable: true,
    lastEnvelopeSeq: 0,
    lastDegradedSig: "",
    symbolSuggestIndex: -1,
    symbolSuggestOpen: false,
    symbolSearchDebounce: null,
    symbolSearchSeq: 0,
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
    cameraStatus: "/api/radar/camera/status",
    stream: () => {
      const s = getActiveSymbol();
      return `/api/radar/stream?symbol=${encodeURIComponent(s)}`;
    },
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
    cameraStateBadge: document.getElementById("cameraStateBadge"),
    tablaDecisiones: document.getElementById("tablaDecisiones"),
    providersMetrics: document.getElementById("providersMetrics"),
    tablaProvidersFull: document.getElementById("tablaProvidersFull"),
    symbolNarrative: document.getElementById("symbolNarrative"),
    symbolCards: document.getElementById("symbolCards"),
    liveMode: document.getElementById("liveMode"),
    liveLastEvent: document.getElementById("liveLastEvent"),
    liveHeartbeat: document.getElementById("liveHeartbeat"),
    sseLiveQuality: document.getElementById("sseLiveQuality"),
    sseSinceSnapshot: document.getElementById("sseSinceSnapshot"),
    sseSinceHeartbeat: document.getElementById("sseSinceHeartbeat"),
    sseReconnecting: document.getElementById("sseReconnecting"),
    symbolSuggestList: document.getElementById("symbolSuggestList"),
    symbolSearchMessage: document.getElementById("symbolSearchMessage"),
    symbolActivePill: document.getElementById("symbolActivePill"),
    symbolSearchCombo: document.getElementById("symbolSearchCombo"),
  };

  const searchResponseCache = new Map();

  /** sessionStorage opcional: nunca lanza; devuelve cadena vacía si no hay API o acceso denegado. */
  function safeGetSessionValue(key) {
    try {
      if (typeof sessionStorage === "undefined" || sessionStorage === null) return "";
      const v = sessionStorage.getItem(String(key || ""));
      return v == null ? "" : String(v);
    } catch (_e) {
      return "";
    }
  }

  /** Devuelve true si se guardó; false si storage no disponible o error (sandbox, cuota, etc.). */
  function safeSetSessionValue(key, value) {
    try {
      if (typeof sessionStorage === "undefined" || sessionStorage === null) return false;
      sessionStorage.setItem(String(key || ""), String(value ?? ""));
      return true;
    } catch (_e) {
      return false;
    }
  }

  function safeGetSessionSymbol() {
    const raw = safeGetSessionValue(SESSION_SYMBOL_KEY).trim().toUpperCase();
    return raw && /^[A-Z0-9.-]{1,12}$/i.test(raw) ? raw : "";
  }

  function persistActiveSymbol(sym) {
    const s = String(sym || "").trim().toUpperCase();
    if (!s) return;
    safeSetSessionValue(SESSION_SYMBOL_KEY, s);
  }

  /**
   * Fuente de verdad del ticker activo: **state.symbol** (memoria JS).
   * Mientras el usuario edita en el combobox (foco + texto distinto al comprometido), se usa el input.
   * sessionStorage solo rehidrata al arranque vía init; aquí es fallback si memoria/input vacíos.
   */
  function isSymbolTypingDraft() {
    if (document.activeElement !== el.symbolInput) return false;
    const inputRaw = String(el.symbolInput?.value || "").trim().toUpperCase();
    if (!inputRaw) return false;
    const fromState = String(state.symbol || "").trim().toUpperCase();
    if (!fromState) return true;
    return inputRaw !== fromState;
  }

  function getActiveSymbol() {
    if (isSymbolTypingDraft()) {
      const draft = String(el.symbolInput?.value || "").trim().toUpperCase();
      if (draft) return draft;
    }
    const fromState = String(state.symbol || "").trim().toUpperCase();
    if (fromState && /^[A-Z0-9.-]{1,12}$/i.test(fromState)) return fromState;
    const fromInput = String(el.symbolInput?.value || "").trim().toUpperCase();
    if (fromInput) return fromInput;
    const persisted = safeGetSessionSymbol();
    if (persisted) return persisted;
    return DEFAULT_SYMBOL_FALLBACK;
  }

  function updateSymbolPill() {
    const s = getActiveSymbol();
    el.symbolActivePill.textContent = s ? `Activo: ${s}` : "";
  }

  function setSymbolSearchMessage(text) {
    if (!text) {
      el.symbolSearchMessage.textContent = "";
      el.symbolSearchMessage.classList.add("hidden");
      return;
    }
    el.symbolSearchMessage.textContent = text;
    el.symbolSearchMessage.classList.remove("hidden");
  }

  function closeSymbolSuggest() {
    state.symbolSuggestOpen = false;
    state.symbolSuggestIndex = -1;
    el.symbolSuggestList.classList.add("hidden");
    el.symbolSuggestList.innerHTML = "";
    el.symbolInput.setAttribute("aria-expanded", "false");
    el.symbolInput.removeAttribute("aria-activedescendant");
  }

  function setSuggestActiveIndex(newIdx) {
    const items = Array.from(el.symbolSuggestList.querySelectorAll('[role="option"]'));
    if (!items.length) {
      el.symbolInput.removeAttribute("aria-activedescendant");
      return;
    }
    const len = items.length;
    const idx = ((newIdx % len) + len) % len;
    state.symbolSuggestIndex = idx;
    items.forEach((node, i) => {
      const sel = i === idx;
      node.classList.toggle("is-active", sel);
      node.setAttribute("aria-selected", sel ? "true" : "false");
    });
    const active = items[idx];
    if (active?.id) {
      el.symbolInput.setAttribute("aria-activedescendant", active.id);
    } else {
      el.symbolInput.removeAttribute("aria-activedescendant");
    }
    active?.scrollIntoView({ block: "nearest" });
  }

  function applySelectedSymbol(sym, opts) {
    const s = String(sym || "").trim().toUpperCase();
    if (!s) return;
    const o = opts || {};
    el.symbolInput.value = s;
    state.symbol = s;
    persistActiveSymbol(s);
    updateSymbolPill();
    closeSymbolSuggest();
    setSymbolSearchMessage("");
    if (!o.skipDataRefresh) {
      refreshAll();
      setupStreamingWithFallback();
    }
  }

  async function fetchSymbolSearchRemote(q) {
    const key = `${q.toLowerCase()}|25`;
    if (searchResponseCache.has(key)) {
      return searchResponseCache.get(key);
    }
    const url = `/api/radar/symbols/search?q=${encodeURIComponent(q)}&limit=25`;
    const res = await fetch(url, { headers: { Accept: "application/json" }, cache: "no-store" });
    if (!res.ok) {
      searchResponseCache.delete(key);
      return {
        ok: false,
        source: "unavailable",
        message: `Búsqueda HTTP ${res.status}. Reinicia PUSH si acabas de desplegar la ruta /api/radar/symbols/search.`,
        matches: [],
        truncated: false,
      };
    }
    let data;
    try {
      data = await res.json();
    } catch (_parseErr) {
      searchResponseCache.delete(key);
      return {
        ok: false,
        source: "unavailable",
        message: "Respuesta de búsqueda no válida (JSON).",
        matches: [],
        truncated: false,
      };
    }
    if (data && data.ok && data.source === "quant") {
      searchResponseCache.set(key, data);
      if (searchResponseCache.size > 40) {
        const firstKey = searchResponseCache.keys().next().value;
        searchResponseCache.delete(firstKey);
      }
    } else {
      searchResponseCache.delete(key);
    }
    return data;
  }

  function renderSymbolSuggest(matches, activeIndex) {
    el.symbolSuggestList.innerHTML = "";
    if (!matches || !matches.length) return;
    matches.forEach((m, idx) => {
      const li = document.createElement("li");
      const optId = `symbol-suggest-opt-${idx}`;
      li.id = optId;
      li.className = "symbol-suggest-item";
      li.setAttribute("role", "option");
      li.setAttribute("aria-selected", idx === activeIndex ? "true" : "false");
      if (idx === activeIndex) li.classList.add("is-active");
      li.dataset.symbol = m.symbol || "";
      const opt = m.is_optionable ? "opciones" : "sin opciones";
      const ex = m.exchange || "—";
      const nm = m.name ? String(m.name).slice(0, 72) : "—";
      li.innerHTML = `<span class="symbol-suggest-ticker">${m.symbol || ""}</span><span class="symbol-suggest-meta">${nm} · ${ex} · ${opt}</span>`;
      let picked = false;
      const pick = (ev) => {
        ev.preventDefault();
        if (picked) return;
        picked = true;
        applySelectedSymbol(li.dataset.symbol);
      };
      li.addEventListener("mousedown", pick);
      li.addEventListener("pointerdown", pick);
      el.symbolSuggestList.appendChild(li);
    });
    const start = Math.max(0, Math.min(activeIndex, matches.length - 1));
    setSuggestActiveIndex(start);
  }

  async function runSymbolSearchQuery(raw) {
    const q = String(raw || "").trim();
    const seq = ++state.symbolSearchSeq;
    if (q.length < SEARCH_MIN_CHARS) {
      closeSymbolSuggest();
      setSymbolSearchMessage("");
      return;
    }
    try {
      const data = await fetchSymbolSearchRemote(q);
      if (seq !== state.symbolSearchSeq) return;
      if (!data.ok || data.source === "unavailable") {
        closeSymbolSuggest();
        setSymbolSearchMessage(
          data.message || "Búsqueda no disponible: motor Quant no accesible. Puedes escribir un ticker y pulsar Actualizar."
        );
        return;
      }
      const matches = Array.isArray(data.matches) ? data.matches : [];
      const hint = data.hint ? String(data.hint) : "";
      if (hint && matches.length === 0) {
        setSymbolSearchMessage(hint);
        closeSymbolSuggest();
        return;
      }
      setSymbolSearchMessage(hint && matches.length ? hint : "");
      state.symbolSuggestIndex = matches.length ? 0 : -1;
      state.symbolSuggestOpen = matches.length > 0;
      if (matches.length) {
        el.symbolSuggestList.classList.remove("hidden");
        el.symbolInput.setAttribute("aria-expanded", "true");
        renderSymbolSuggest(matches, state.symbolSuggestIndex);
      } else {
        closeSymbolSuggest();
        setSymbolSearchMessage("Sin resultados para esa búsqueda.");
      }
    } catch (_err) {
      if (seq !== state.symbolSearchSeq) return;
      closeSymbolSuggest();
      setSymbolSearchMessage("Error de red al buscar símbolos.");
    }
  }

  function scheduleSymbolSearch() {
    if (state.symbolSearchDebounce) {
      clearTimeout(state.symbolSearchDebounce);
    }
    state.symbolSearchDebounce = setTimeout(() => {
      state.symbolSearchDebounce = null;
      const q = el.symbolInput.value.trim();
      runSymbolSearchQuery(q);
    }, SEARCH_DEBOUNCE_MS);
  }

  function moveSuggestHighlight(delta) {
    const items = el.symbolSuggestList.querySelectorAll('[role="option"]');
    if (!state.symbolSuggestOpen || !items.length) return;
    const len = items.length;
    let next = state.symbolSuggestIndex + delta;
    if (next < 0) next = len - 1;
    if (next >= len) next = 0;
    setSuggestActiveIndex(next);
  }

  function confirmSuggestSelection() {
    const items = Array.from(el.symbolSuggestList.querySelectorAll('[role="option"]'));
    if (!state.symbolSuggestOpen || !items.length) return false;
    const idx = Math.max(0, Math.min(state.symbolSuggestIndex, items.length - 1));
    const sym = items[idx].dataset.symbol;
    if (sym) {
      applySelectedSymbol(sym);
      return true;
    }
    return false;
  }

  function setupSymbolSearch() {
    el.symbolInput.addEventListener("input", () => {
      scheduleSymbolSearch();
    });
    el.symbolInput.addEventListener("keydown", (event) => {
      if (event.key === "Tab") {
        closeSymbolSuggest();
        return;
      }
      if (event.key === "ArrowDown") {
        if (state.symbolSuggestOpen) {
          event.preventDefault();
          moveSuggestHighlight(1);
        }
        return;
      }
      if (event.key === "ArrowUp") {
        if (state.symbolSuggestOpen) {
          event.preventDefault();
          moveSuggestHighlight(-1);
        }
        return;
      }
      if (event.key === "Enter") {
        if (state.symbolSuggestOpen && confirmSuggestSelection()) {
          event.preventDefault();
          return;
        }
        const typed = el.symbolInput.value.trim();
        if (typed) {
          applySelectedSymbol(typed, { skipDataRefresh: false });
          event.preventDefault();
        }
        return;
      }
      if (event.key === "Escape") {
        closeSymbolSuggest();
        return;
      }
    });
    el.symbolInput.addEventListener("focus", () => {
      const q = el.symbolInput.value.trim();
      if (q.length >= SEARCH_MIN_CHARS) {
        scheduleSymbolSearch();
      }
    });
    document.addEventListener(
      "pointerdown",
      (event) => {
        if (!state.symbolSuggestOpen) return;
        const root = el.symbolSearchCombo;
        if (!root) return;
        const t = event.target;
        if (root.contains(t)) return;
        closeSymbolSuggest();
      },
      true
    );
  }

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
      snapshot: "instantánea (legado)",
      snapshot_update: "instantánea (datos)",
      decision: "decisión",
      provider_health: "salud de proveedores",
      provider_state_changed: "proveedores",
      degraded_state_changed: "degradación operativa",
      degradation: "degradación",
      alert: "alerta",
      heartbeat: "latido",
      camera_state_changed: "cámara",
      unknown: "desconocido",
    };
    return map[t] || t.replaceAll("_", " ");
  }

  function cameraStateBadgeKind(state) {
    const s = String(state || "").toLowerCase();
    if (s === "ready") return "badge-accepted";
    if (s === "degraded") return "badge-caution";
    if (s === "disabled" || s === "not_configured") return "badge-neutral";
    return "badge-rejected";
  }

  function cameraStateEs(state) {
    const s = String(state || "").toLowerCase();
    const map = {
      ready: "Listo",
      unavailable: "No disponible",
      degraded: "Degradado",
      disabled: "Desactivado",
      not_configured: "Sin configurar",
    };
    return map[s] || (state ? String(state).replaceAll("_", " ") : "—");
  }

  function renderCameraPanel(cameraHealth, summary) {
    const cam = cameraHealth?.camera || summary?.camera_context || {};
    const st = String(cam.state || "").toLowerCase();
    if (el.cameraStateBadge) {
      el.cameraStateBadge.className = `badge ${cameraStateBadgeKind(cam.state)} camera-state-badge`;
      el.cameraStateBadge.textContent = cameraStateEs(cam.state);
    }
    const actNote = cam.activity_note != null ? cam.activity_note : cam.activity;
    const actDisplay =
      actNote != null && actNote !== ""
        ? String(actNote)
        : formatNumber(cam.activity_level);
    const pnp = Array.isArray(cam.pnp_hints) && cam.pnp_hints.length ? cam.pnp_hints.join("; ") : "—";
    const rows = [
      ["Proveedor", cam.provider || "—"],
      ["Resumen", cam.status || cameraStateEs(cam.state)],
      ["Modo físico (Quant)", cam.mode_detected || "—"],
      ["Backend captura", cam.backend || "—"],
      ["Índice USB", cam.device_index != null && cam.device_index !== "" ? String(cam.device_index) : "—"],
      ["PnP (hints)", pnp],
      ["Notas", cam.notes || "—"],
      ["Disponibilidad", cam.availability_pct != null ? `${Number(cam.availability_pct).toFixed(1)} %` : "—"],
      ["Última captura", formatTs(cam.last_capture_ts || cam.last_capture)],
      ["Presencia operador", formatSiNo(cam.presence)],
      ["Actividad / contexto", actDisplay],
    ];
    const deg = cam.degradation_reason && String(cam.degradation_reason).toLowerCase() !== "quant_unreachable";
    if (deg) {
      rows.push(["Motivo degradación", String(cam.degradation_reason)]);
    }
    renderKvs(el.cameraStatusSummary, rows);
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

  let _sseRefreshDebounce = null;
  function scheduleRefreshAllFromSse() {
    if (_sseRefreshDebounce) clearTimeout(_sseRefreshDebounce);
    _sseRefreshDebounce = setTimeout(() => {
      _sseRefreshDebounce = null;
      refreshAll();
    }, 200);
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
    const symbol = getActiveSymbol();
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
          fetchJson(endpoints.cameraStatus),
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
      state.lastQuantReachable = summaryRes.data?.transport?.stub !== true;
      state.lastSnapshotUsefulAt = Date.now();
      renderPrincipal(summaryRes.data, providersRes.data, decisionsRes.data, dealerRes.data, fastRes.data, structuralRes.data, {
        camera: cameraRes.data?.camera || {},
      });
      renderDecisions(decisionsRes.data);
      renderProviders(providersRes.data, statsRes.data);
      renderSymbol(symbol, summaryRes.data, dealerRes.data, fastRes.data, structuralRes.data, politicalRes.data);
      persistActiveSymbol(symbol);
      updateSymbolPill();
      updateSseConnectionUi();
      if (!state.streamConnected) {
        setLiveStatus("Encuesta periódica");
      }
    } catch (error) {
      el.latenciaActual.textContent = "-";
      el.ultimaActualizacion.textContent = startedLabel;
      el.estadoGlobal.className = "badge badge-rejected";
      el.estadoGlobal.textContent = state.lastSummary ? "Actualización parcial" : "Error de actualización";
      el.estadoFresh.className = "badge badge-stale";
      el.estadoFresh.textContent = String(error.message || error);
      const keep = state.lastSummary
        ? "Se conserva el último snapshot válido en pantalla."
        : "No hay snapshot previo para mostrar.";
      el.alertasRapidas.innerHTML = `<div class="alert-item">${keep} Detalle: ${String(error.message || error)}</div>`;
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

    renderCameraPanel(cameraHealth, summary);
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
        applySelectedSymbol(symbol, { skipDataRefresh: true });
        document.querySelector('[data-tab="simbolo"]').click();
        refreshAll();
        setupStreamingWithFallback();
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

  const CAMERA_POLL_MS = 30000;

  function setupCameraPolling() {
    if (state.cameraPollTimer) {
      clearInterval(state.cameraPollTimer);
      state.cameraPollTimer = null;
    }
    state.cameraPollTimer = setInterval(async () => {
      try {
        const res = await fetchJson(endpoints.cameraStatus);
        renderCameraPanel({ camera: res.data?.camera || {} }, state.lastSummary);
      } catch (_err) {
        /* silencioso: el estado global ya refleja fallos de red en «Actualizar» */
      }
    }, CAMERA_POLL_MS);
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

  function formatAgeSeconds(ms) {
    if (!ms || ms < 0 || !Number.isFinite(ms)) return "-";
    const s = Math.floor(ms / 1000);
    if (s < 60) return `${s}s`;
    const m = Math.floor(s / 60);
    return `${m}m ${s % 60}s`;
  }

  function setSseReconnecting(on) {
    if (el.sseReconnecting) {
      el.sseReconnecting.classList.toggle("hidden", !on);
    }
  }

  function updateSseConnectionUi() {
    const sym = getActiveSymbol();
    const hbMs = state.lastHeartbeatMs ? Date.now() - state.lastHeartbeatMs : null;
    const snapMs = state.lastSnapshotUsefulAt ? Date.now() - state.lastSnapshotUsefulAt : null;
    if (el.sseSinceHeartbeat) {
      el.sseSinceHeartbeat.textContent = hbMs == null ? "-" : formatAgeSeconds(hbMs);
    }
    if (el.sseSinceSnapshot) {
      el.sseSinceSnapshot.textContent = snapMs == null ? "-" : formatAgeSeconds(snapMs);
    }
    let q = "fresh";
    if (!state.streamConnected) {
      q = "disconnected";
    } else if (hbMs != null && hbMs > SSE_HEARTBEAT_DEAD_MS) {
      q = "disconnected";
    } else if (!state.lastQuantReachable) {
      q = "degraded";
    } else if (snapMs != null && snapMs > SNAPSHOT_USEFUL_STALE_MS) {
      q = "stale";
    }
    if (el.sseLiveQuality) {
      const labels = {
        fresh: ["badge-accepted", "SSE: fresco"],
        stale: ["badge-stale", "SSE: datos envejecidos"],
        degraded: ["badge-caution", "SSE: Quant stub / degradado"],
        disconnected: ["badge-rejected", "SSE: desconectado"],
      };
      const [cls, text] = labels[q] || labels.fresh;
      el.sseLiveQuality.className = `badge ${cls}`;
      el.sseLiveQuality.textContent = text;
    }
    const needle = sym ? `symbol=${encodeURIComponent(sym)}` : "";
    if (
      state.streamConnected &&
      needle &&
      !isSymbolTypingDraft() &&
      state.stream &&
      state.stream.url &&
      !String(state.stream.url).includes(needle)
    ) {
      teardownStream();
      setupStreamingWithFallback();
    }
  }

  function startLiveQualityTicker() {
    if (state.liveQualityTicker) clearInterval(state.liveQualityTicker);
    state.liveQualityTicker = setInterval(updateSseConnectionUi, 1000);
  }

  function handleRadarEnvelope(env) {
    if (!env || typeof env !== "object") return;
    if (env.symbol && env.symbol !== getActiveSymbol()) return;
    if (typeof env.sequence === "number") {
      state.lastEnvelopeSeq = env.sequence;
    }
    const t = env.type;
    const d = env.data || {};
    if (t === "heartbeat") {
      if (typeof d.quant_reachable === "boolean") {
        state.lastQuantReachable = d.quant_reachable;
      }
      markHeartbeat();
      markLastEvent(tipoEventoStreamEs("heartbeat"));
      updateSseConnectionUi();
      return;
    }
    markLastEvent(tipoEventoStreamEs(t));
    if (t === "snapshot_update") {
      if (d.summary) {
        state.lastSummary = d.summary;
        state.lastSnapshotUsefulAt = Date.now();
      }
      scheduleRefreshAllFromSse();
      return;
    }
    if (t === "camera_state_changed") {
      const c = d.camera || {};
      renderCameraPanel({ camera: c }, state.lastSummary);
      return;
    }
    if (t === "provider_state_changed") {
      const body = d.providers_payload;
      if (body) {
        fetchJson(endpoints.decisionsStats)
          .then((statsRes) => {
            renderProviders(body, statsRes.data);
          })
          .catch(() => {});
      }
      return;
    }
    if (t === "degraded_state_changed") {
      const sig = `${d.snapshot_classification}|${d.quant_reachable}|${d.transport_stub}`;
      if (sig !== state.lastDegradedSig) {
        state.lastDegradedSig = sig;
        const msg = d.operational_degraded
          ? `Degradación: ${String(d.snapshot_classification || "—")} · Quant ${d.quant_reachable ? "OK" : "no accesible"}`
          : "Estado operativo estable";
        el.alertasRapidas.innerHTML = `<div class="alert-item">${msg}</div>${el.alertasRapidas.innerHTML}`;
      }
      return;
    }
  }

  function routeSseEvent(eventName, raw) {
    let env;
    try {
      env = JSON.parse(raw);
    } catch (_e) {
      markLastEvent("SSE JSON inválido");
      return;
    }
    if (env && typeof env.sequence === "number" && env.type && env.timestamp && env.symbol) {
      handleRadarEnvelope(env);
      return;
    }
    if (eventName === "heartbeat" && env?.payload) {
      markHeartbeat();
      markLastEvent(tipoEventoStreamEs("heartbeat"));
      return;
    }
    if (eventName === "snapshot" && env?.payload) {
      markLastEvent(tipoEventoStreamEs("snapshot"));
      scheduleRefreshAllFromSse();
      return;
    }
    if (eventName === "camera_state_changed") {
      const c = env?.camera || env?.data?.camera || {};
      renderCameraPanel({ camera: c }, state.lastSummary);
      return;
    }
    if (eventName === "decision") {
      refreshAll();
      return;
    }
    if (eventName === "provider_health" || eventName === "degradation" || eventName === "alert") {
      const msg = env?.payload?.message || env?.message || tipoEventoStreamEs(eventName);
      el.alertasRapidas.innerHTML = `<div class="alert-item">${msg}</div>${el.alertasRapidas.innerHTML}`;
      refreshAll();
    }
  }

  function ensureStreamHealthWatchdog() {
    if (state.staleTimer) {
      clearInterval(state.staleTimer);
    }
    state.staleTimer = setInterval(() => {
      if (!state.streamConnected || !state.lastHeartbeatMs) return;
      const age = Date.now() - state.lastHeartbeatMs;
      if (age > SSE_HEARTBEAT_DEAD_MS) {
        setLiveStatus("SSE sin latido — reconectando");
        teardownStream();
        setupAutoRefresh();
        setSseReconnecting(true);
      }
    }, 5000);
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
    if (state.sseReconnectTimer) {
      clearTimeout(state.sseReconnectTimer);
      state.sseReconnectTimer = null;
    }
    teardownStream();
    try {
      const url = endpoints.stream();
      const source = new EventSource(url);
      state.stream = source;
      const typedEvents = [
        "heartbeat",
        "snapshot_update",
        "snapshot",
        "camera_state_changed",
        "provider_state_changed",
        "degraded_state_changed",
        "decision",
        "provider_health",
        "degradation",
        "alert",
      ];
      typedEvents.forEach((name) => {
        source.addEventListener(name, (event) => {
          try {
            routeSseEvent(name, event.data);
          } catch (_error) {
            markLastEvent(`${tipoEventoStreamEs(name)} (error)`);
          }
        });
      });
      source.onopen = () => {
        state.streamConnected = true;
        state.sseBackoffMs = 3000;
        setSseReconnecting(false);
        setLiveStatus("Transmisión en vivo (SSE)");
        markHeartbeat();
        state.lastSnapshotUsefulAt = Date.now();
        if (state.timer) {
          clearInterval(state.timer);
          state.timer = null;
        }
        startLiveQualityTicker();
        updateSseConnectionUi();
      };
      source.onerror = () => {
        teardownStream();
        setLiveStatus("SSE desconectado");
        setupAutoRefresh();
        updateSseConnectionUi();
        setSseReconnecting(true);
        if (state.sseReconnectTimer) {
          clearTimeout(state.sseReconnectTimer);
        }
        const delay = state.sseBackoffMs;
        state.sseBackoffMs = Math.min(state.sseBackoffMs * 2, 60000);
        state.sseReconnectTimer = setTimeout(() => {
          state.sseReconnectTimer = null;
          setupStreamingWithFallback();
        }, delay);
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
  setupSymbolSearch();
  const _initial = (() => {
    const s = safeGetSessionSymbol();
    return s || DEFAULT_SYMBOL_FALLBACK;
  })();
  /* Memoria JS = fuente de verdad; input y storage alineados al mismo valor inicial. */
  state.symbol = _initial;
  el.symbolInput.value = _initial;
  updateSymbolPill();
  startLiveQualityTicker();
  setupStreamingWithFallback();
  if (!state.streamConnected) {
    setupAutoRefresh();
  }
  setupCameraPolling();
  refreshAll();
})();
