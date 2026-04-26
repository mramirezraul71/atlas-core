/**
 * ATLAS v4 - AI Configuration Module
 * - Uses real model inventory from local Ollama and workspace model catalog
 * - Adds "automatic local" mode and keeps it as default
 * - Saves compatible payload to /config/ai
 */
import { poll, stop } from "../lib/polling.js";

const POLL_ID = "config-module";
const REALTIME_POLL_ID = "config-module:realtime";
const REALTIME_INTERVAL_MS = 12000;
const ACTIVITY_HISTORY_LIMIT = 8;
const AUTO_LOCAL_MODE = "auto_local";
const AUTO_LOCAL_MODEL = "auto_local";
const WORKING_PHASES = new Set(["starting", "running", "planning", "executing"]);

const MODE_OPTIONS = [
  { value: AUTO_LOCAL_MODE, label: "Automatica local (default)" },
  { value: "auto", label: "Auto multi proveedor" },
  { value: "manual", label: "Manual" },
];

const PROVIDER_OPTIONS = [
  { value: "ollama", label: "Ollama (Local)" },
  { value: "openai", label: "OpenAI" },
  { value: "anthropic", label: "Anthropic" },
  { value: "gemini", label: "Gemini" },
  { value: "deepseek", label: "DeepSeek" },
  { value: "xai", label: "xAI" },
  { value: "groq", label: "Groq" },
  { value: "bedrock", label: "AWS Bedrock" },
  { value: "perplexity", label: "Perplexity" },
  { value: "mistral", label: "Mistral" },
];

const FALLBACK_LOCAL_MODELS = [
  "llama3.1:8b",
  "qwen2.5-coder:7b",
  "deepseek-coder-v2:16b",
  "qwen3-coder:30b",
  "qwen3-vl:30b",
  "llama3.2-vision:11b",
];

const CODE_MODEL_RE = /(coder|codellama|code|deepseek-coder)/i;
const REASONING_MODEL_RE = /(reason|r1|qwen3|llama3\.1|deepseek)/i;
const EMBEDDING_MODEL_RE = /(embed|embedding|nomic-embed)/i;

function _esc(text) {
  const span = document.createElement("span");
  span.textContent = text ?? "";
  return span.innerHTML;
}

function _normalizeMode(mode) {
  const m = String(mode || "").trim().toLowerCase();
  if (
    !m ||
    m === "hybrid" ||
    m === "auto-local" ||
    m === "local_auto" ||
    m === "automatic_local"
  ) {
    return AUTO_LOCAL_MODE;
  }
  if (m === "auto" || m === "manual" || m === AUTO_LOCAL_MODE) {
    return m;
  }
  return AUTO_LOCAL_MODE;
}

function _toUnique(values) {
  const seen = new Set();
  const out = [];
  for (const raw of values || []) {
    const value = String(raw || "").trim();
    if (!value || seen.has(value)) continue;
    seen.add(value);
    out.push(value);
  }
  return out;
}

function _extractWorkspaceModels(raw) {
  const data = raw?.data ?? raw ?? {};
  const rows = Array.isArray(data?.models) ? data.models : [];
  return _toUnique(
    rows.map((row) => {
      if (typeof row === "string") return row;
      const id = String(row?.id || "").trim();
      if (!id) return "";
      return id.includes("/") ? id.split("/").slice(1).join("/") : id;
    }),
  );
}

function _extractWorkspaceProviderCounts(raw) {
  const data = raw?.data ?? raw ?? {};
  const rows = Array.isArray(data?.models) ? data.models : [];
  const counts = {};

  for (const row of rows) {
    const id = typeof row === "string" ? row : String(row?.id || "").trim();
    if (!id) continue;
    const provider = id.includes("/")
      ? String(id.split("/")[0] || "").trim().toLowerCase() || "ollama"
      : "ollama";
    counts[provider] = (counts[provider] || 0) + 1;
  }

  return counts;
}

function _extractOllamaModels(raw) {
  const data = raw?.data ?? raw ?? {};
  const rows = Array.isArray(data?.models)
    ? data.models
    : Array.isArray(data?.local_models)
      ? data.local_models
      : [];
  return _toUnique(
    rows.map((row) => {
      if (typeof row === "string") return row;
      return row?.full_name || row?.name || row?.model || row?.id || "";
    }),
  );
}

function _buildCatalog(workspaceModels, ollamaModels) {
  const merged = _toUnique([
    ...ollamaModels,
    ...workspaceModels,
    ...FALLBACK_LOCAL_MODELS,
  ]).filter((name) => !EMBEDDING_MODEL_RE.test(name));

  const brainModels = merged;
  const codeModels = _toUnique(merged.filter((name) => CODE_MODEL_RE.test(name)));
  const reasoningModels = _toUnique(
    merged.filter((name) => REASONING_MODEL_RE.test(name)),
  );

  return {
    brainModels: brainModels.length ? brainModels : FALLBACK_LOCAL_MODELS,
    codeModels: codeModels.length ? codeModels : FALLBACK_LOCAL_MODELS,
    reasoningModels: reasoningModels.length ? reasoningModels : FALLBACK_LOCAL_MODELS,
  };
}

function _optionsFromModels(models, withAutoLocal = false) {
  const opts = [];
  if (withAutoLocal) {
    opts.push({
      value: AUTO_LOCAL_MODEL,
      label: "Automatica local (usa enrutamiento local inteligente)",
    });
  }
  for (const model of models || []) {
    opts.push({ value: model, label: model });
  }
  return opts;
}

function _ensureOption(options, currentValue) {
  const current = String(currentValue || "").trim();
  if (!current) return options;
  if (options.some((opt) => String(opt.value) === current)) {
    return options;
  }
  return [...options, { value: current, label: current }];
}

function _renderSelect(id, options, current) {
  const normalizedOptions = _ensureOption(options || [], current);
  const currentValue = String(current || "").trim();
  return `
    <select id="${id}" style="width:100%;padding:7px 10px;background:var(--surface-1);border:1px solid var(--border);border-radius:6px;color:var(--text-primary);font-size:13px">
      ${normalizedOptions
        .map((opt) => {
          const value = String(opt.value ?? "");
          const selected = value === currentValue ? "selected" : "";
          return `<option value="${_esc(value)}" ${selected}>${_esc(opt.label || value)}</option>`;
        })
        .join("")}
    </select>`;
}

function _renderInput(id, placeholder, current, type = "text") {
  return `
    <input id="${id}" type="${type}" value="${_esc(String(current ?? ""))}" placeholder="${_esc(placeholder)}"
      style="width:100%;padding:7px 10px;background:var(--surface-1);border:1px solid var(--border);border-radius:6px;color:var(--text-primary);font-size:13px;box-sizing:border-box">`;
}

function _renderTextarea(id, placeholder, current, rows = 4) {
  return `
    <textarea id="${id}" rows="${rows}" placeholder="${_esc(placeholder)}"
      style="width:100%;padding:7px 10px;background:var(--surface-1);border:1px solid var(--border);border-radius:6px;color:var(--text-primary);font-size:13px;box-sizing:border-box;resize:vertical;font-family:inherit">${_esc(String(current ?? ""))}</textarea>`;
}

function _toBoolean(value, defaultValue = false) {
  if (typeof value === "boolean") return value;
  if (value === null || value === undefined || value === "") return defaultValue;
  const normalized = String(value).trim().toLowerCase();
  if (["1", "true", "yes", "si", "on"].includes(normalized)) return true;
  if (["0", "false", "no", "off"].includes(normalized)) return false;
  return defaultValue;
}

function _renderBooleanSelect(id, current) {
  const boolValue = _toBoolean(current, false);
  return _renderSelect(
    id,
    [
      { value: "true", label: "Activo" },
      { value: "false", label: "Inactivo" },
    ],
    String(boolValue),
  );
}

function _unwrapData(raw) {
  if (!raw || typeof raw !== "object") return {};
  return raw?.data ?? raw;
}

function _safeNum(value, fallback = 0) {
  const num = Number(value);
  return Number.isFinite(num) ? num : fallback;
}

function _formatInt(value, fallback = "--") {
  const num = Number(value);
  if (!Number.isFinite(num)) return fallback;
  return Math.round(num).toLocaleString();
}

function _formatDecimal(value, digits = 1, fallback = "--") {
  const num = Number(value);
  if (!Number.isFinite(num)) return fallback;
  return num.toFixed(digits);
}

function _formatPercent(value, digits = 1, fallback = "--") {
  const num = Number(value);
  if (!Number.isFinite(num)) return fallback;
  return `${num.toFixed(digits)}%`;
}

function _formatMoney(value, fallback = "--") {
  const num = Number(value);
  if (!Number.isFinite(num)) return fallback;
  return `$${num.toFixed(2)}`;
}

function _parseTsMs(ts) {
  if (!ts) return 0;
  if (typeof ts === "number" && Number.isFinite(ts)) {
    return ts > 1e12 ? ts : ts * 1000;
  }
  const parsed = Date.parse(String(ts));
  return Number.isFinite(parsed) ? parsed : 0;
}

function _formatTime(ts) {
  const ms = _parseTsMs(ts);
  if (!ms) return "--:--";
  return new Date(ms).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" });
}

function _modeHelpText(mode) {
  if (mode === AUTO_LOCAL_MODE) {
    return "Default: usa modelos locales de Ollama y escala por complejidad/especialidad.";
  }
  if (mode === "auto") {
    return "Auto global: permite usar proveedores externos segun disponibilidad.";
  }
  return "Manual: fija proveedor/modelo exacto para todas las solicitudes.";
}

export default {
  id: "config",
  label: "AI Configuration",
  icon: "sliders",
  category: "configuration",

  render(container) {
    container.innerHTML = `
      <div class="module-view">
        <div class="module-header">
          <button class="back-btn" id="cfg-back">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 12H5M12 19l-7-7 7-7"/></svg>
            Dashboard
          </button>
          <h2>Configuracion AI</h2>
        </div>

        <div class="module-body">
          <div class="section-title">Estado del Sistema</div>
          <div class="stat-row" style="margin-bottom:20px" id="cfg-health-row">
            <div style="padding:12px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
          </div>

          <div class="section-title">Modelos AI
            <span id="cfg-save-ts" style="font-size:10px;color:var(--text-muted);font-weight:400;margin-left:8px"></span>
          </div>
          <div style="display:grid;grid-template-columns:repeat(auto-fill,minmax(260px,1fr));gap:12px;margin-bottom:20px" id="cfg-form-grid">
            <div style="padding:12px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
          </div>

          <div class="action-bar" style="margin-bottom:20px">
            <button class="action-btn primary" id="btn-cfg-save">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 21H5a2 2 0 01-2-2V5a2 2 0 012-2h11l5 5v11a2 2 0 01-2 2z"/><polyline points="17 21 17 13 7 13 7 21"/><polyline points="7 3 7 8 15 8"/></svg>
              Guardar Configuracion
            </button>
            <button class="action-btn" id="btn-cfg-test">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M9 11l3 3L22 4"/><path d="M21 12v7a2 2 0 01-2 2H5a2 2 0 01-2-2V5a2 2 0 012-2h11"/></svg>
              Test conexion
            </button>
            <button class="action-btn" id="btn-cfg-load">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="23 4 23 10 17 10"/><path d="M20.49 15a9 9 0 11-2.12-9.36L23 10"/></svg>
              Recargar
            </button>
            <span id="cfg-msg" style="font-size:11px;color:var(--text-muted);align-self:center"></span>
          </div>

          <div class="section-title">Proveedores AI</div>
          <div id="cfg-providers" style="margin-bottom:20px">
            <div style="padding:12px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
          </div>

          <div class="section-title">Observabilidad IA</div>
          <div id="cfg-observability" style="margin-bottom:20px">
            <div style="padding:12px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
          </div>

          <div class="section-title">Actividad en tiempo real</div>
          <div id="cfg-activity" style="margin-bottom:20px">
            <div style="padding:12px;text-align:center"><div class="spinner" style="margin:0 auto"></div></div>
          </div>

          <div class="section-title">Modelos Ollama (Local)</div>
          <div class="action-bar" style="margin-bottom:12px">
            <button class="action-btn" id="btn-ollama-list">
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="23 4 23 10 17 10"/><path d="M20.49 15a9 9 0 11-2.12-9.36L23 10"/></svg>
              Actualizar lista
            </button>
          </div>
          <div id="cfg-ollama">
            <div style="font-size:12px;color:var(--text-muted);padding:8px 0">Haz clic en "Actualizar lista" para cargar los modelos Ollama disponibles.</div>
          </div>
        </div>
      </div>
    `;

    container.querySelector("#cfg-back")?.addEventListener("click", () => {
      location.hash = "/";
    });
    container
      .querySelector("#btn-cfg-save")
      ?.addEventListener("click", () => _saveConfig(container));
    container
      .querySelector("#btn-cfg-test")
      ?.addEventListener("click", () => _testConn(container));
    container
      .querySelector("#btn-cfg-load")
      ?.addEventListener("click", () => _loadAll(container));
    container
      .querySelector("#btn-ollama-list")
      ?.addEventListener("click", () => _loadAll(container));

    _loadAll(container);
    poll(REALTIME_POLL_ID, null, REALTIME_INTERVAL_MS, () =>
      _refreshRealtime(container),
    );
  },

  destroy() {
    stop(POLL_ID);
    stop(REALTIME_POLL_ID);
  },
  badge() {
    return null;
  },
};

async function _fetchConfigData() {
  return _fetchConfigDataWithOptions({ includeAutonomy: false });
}

async function _fetchConfigDataWithOptions(options = {}) {
  const includeAutonomy = Boolean(options?.includeAutonomy);
  const requests = {
    healthRes: fetch("/health").then((r) => r.json()).catch(() => null),
    healthDeepRes: fetch("/health/deep").then((r) => r.json()).catch(() => null),
    aiRes: fetch("/config/ai").then((r) => r.json()).catch(() => null),
    workspaceRes: fetch("/api/workspace/interpreter/models")
      .then((r) => r.json())
      .catch(() => null),
    ollamaRes: fetch("/config/ai/ollama-models").then((r) => r.json()).catch(() => null),
    brainStateRes: fetch("/api/brain/state").then((r) => r.json()).catch(() => null),
    aiStatusRes: fetch("/ai/status").then((r) => r.json()).catch(() => null),
    agentModelsRes: fetch("/agent/models").then((r) => r.json()).catch(() => null),
    observabilityRes: fetch("/api/observability/metrics")
      .then((r) => r.json())
      .catch(() => null),
    metricsRes: fetch("/metrics").then((r) => r.json()).catch(() => null),
  };

  if (includeAutonomy) {
    requests.autonomyRes = fetch("/api/autonomy/status")
      .then((r) => r.json())
      .catch(() => null);
  }

  const entries = await Promise.all(
    Object.entries(requests).map(async ([key, req]) => [key, await req]),
  );
  const payload = Object.fromEntries(entries);
  if (!includeAutonomy) payload.autonomyRes = null;
  return payload;
}

function _applyConfigData(
  container,
  payload,
  refreshForm = true,
  forceFormRender = false,
) {
  const {
    healthRes,
    healthDeepRes,
    autonomyRes,
    aiRes,
    workspaceRes,
    ollamaRes,
    brainStateRes,
    aiStatusRes,
    agentModelsRes,
    observabilityRes,
    metricsRes,
  } = payload || {};
  const workspaceModels = _extractWorkspaceModels(workspaceRes);
  const ollamaModels = _extractOllamaModels(ollamaRes);
  const catalog = _buildCatalog(workspaceModels, ollamaModels);
  container._cfgCatalog = catalog;
  const aiPayload = aiRes?.data ?? aiRes ?? {};
  container._cfgLastAiPayload = aiPayload;
  const aiSignature = JSON.stringify(aiPayload);

  const normalizedHealth = _normalizeHealthPayload(
    container,
    healthRes,
    healthDeepRes,
    autonomyRes,
  );
  if (normalizedHealth) _renderHealth(container, normalizedHealth);
  const shouldRenderForm =
    refreshForm &&
    (forceFormRender ||
      !container.querySelector("#cfg-ai-mode") ||
      container._cfgAppliedAiSignature !== aiSignature);
  if (shouldRenderForm) {
    _renderAIForm(container, aiPayload, catalog);
    container._cfgAppliedAiSignature = aiSignature;
  }
  _renderProviders(
    container,
    brainStateRes,
    workspaceRes,
    aiStatusRes,
    agentModelsRes,
  );
  _renderObservability(container, autonomyRes, aiStatusRes, observabilityRes, metricsRes);
  _renderLiveActivity(container, autonomyRes, aiStatusRes, observabilityRes, agentModelsRes);
  _renderOllamaTable(container, ollamaModels);
}

async function _loadAll(container) {
  const payload = await _fetchConfigDataWithOptions({ includeAutonomy: true });
  _applyConfigData(container, payload, true, true);
}

function _isConfigFormBusy(container) {
  const form = container.querySelector("#cfg-form-grid");
  const active = document.activeElement;
  const msg = String(container.querySelector("#cfg-msg")?.textContent || "");
  if (/Guardando|Probando/i.test(msg)) return true;
  return Boolean(form && active && form.contains(active));
}

async function _refreshRealtime(container) {
  const payload = await _fetchConfigDataWithOptions({ includeAutonomy: true });
  const refreshForm = !_isConfigFormBusy(container);
  _applyConfigData(container, payload, refreshForm);
}

function _normalizeHealthPayload(container, healthRes, healthDeepRes, autonomyRes) {
  const base = { ...(healthRes?.data ?? healthRes ?? {}) };
  const deep = healthDeepRes?.data ?? healthDeepRes ?? {};
  const autonomy = autonomyRes?.data ?? autonomyRes ?? {};

  const scoreBase = Number(base?.score);
  const scoreDeep = Number(deep?.score);
  if (Number.isFinite(scoreBase)) {
    base.score = scoreBase;
  } else if (Number.isFinite(scoreDeep)) {
    base.score = scoreDeep;
  }

  const uptimeRaw = base?.uptime_human || base?.uptime;
  if (!uptimeRaw) {
    const uptimeHours = Number(autonomy?.kpis?.uptime_hours);
    if (Number.isFinite(uptimeHours)) {
      container._cfgUptimeBaseHours = uptimeHours;
      container._cfgUptimeBaseTs = Date.now();
    }
    const cachedHours = Number(container?._cfgUptimeBaseHours);
    if (Number.isFinite(cachedHours)) {
      const baseTs = Number(container?._cfgUptimeBaseTs) || Date.now();
      const elapsedHours = Math.max(0, (Date.now() - baseTs) / 3600000);
      base.uptime_human = _formatUptimeHuman(cachedHours + elapsedHours);
    }
  }

  return base;
}

function _formatUptimeHuman(hoursValue) {
  const totalMinutes = Math.max(0, Math.round((Number(hoursValue) || 0) * 60));
  const days = Math.floor(totalMinutes / 1440);
  const afterDays = totalMinutes % 1440;
  const hours = Math.floor(afterDays / 60);
  const minutes = afterDays % 60;

  if (days > 0) return `${days}d ${hours}h ${minutes}m`;
  if (hours > 0) return `${hours}h ${minutes}m`;
  return `${minutes}m`;
}

function _renderHealth(container, data) {
  const payload = data?.data ?? data ?? {};
  const ok = payload.ok ?? true;
  const row = container.querySelector("#cfg-health-row");
  if (!row) return;
  row.innerHTML = `
    <div class="stat-card">
      <div class="stat-card-label">Version</div>
      <div class="stat-card-value accent" style="font-size:18px">${_esc(payload.version || payload.checks?.version || "--")}</div>
    </div>
    <div class="stat-card">
      <div class="stat-card-label">Estado</div>
      <div class="stat-card-value ${ok ? "green" : "orange"}" style="font-size:18px">${ok ? "Saludable" : "Degradado"}</div>
    </div>
    <div class="stat-card">
      <div class="stat-card-label">Puntuacion</div>
      <div class="stat-card-value" style="font-size:18px">${_esc(String(payload.score ?? "--"))}</div>
    </div>
    <div class="stat-card">
      <div class="stat-card-label">Tiempo activo</div>
      <div class="stat-card-value" style="font-size:14px">${_esc(payload.uptime_human || payload.uptime || "--")}</div>
    </div>
    <div class="stat-card">
      <div class="stat-card-label">PID</div>
      <div class="stat-card-value" style="font-size:16px">${_esc(String(payload.pid ?? "--"))}</div>
    </div>
  `;
}

function _renderAIForm(container, payload, catalog) {
  const grid = container.querySelector("#cfg-form-grid");
  if (!grid) return;

  const mode = _normalizeMode(payload?.mode);
  const provider = String(payload?.provider || "ollama").toLowerCase();

  const rawModel = String(payload?.model || payload?.brain_model || "").trim();
  const brainModel =
    mode === AUTO_LOCAL_MODE
      ? AUTO_LOCAL_MODEL
      : rawModel || catalog.brainModels[0] || "llama3.1:8b";

  const codeModel = String(
    payload?.specialists?.code?.model ||
      payload?.code_model ||
      catalog.codeModels[0] ||
      "qwen2.5-coder:7b",
  );
  const reasoningModel = String(
    payload?.specialists?.analysis?.model ||
      payload?.reasoning_model ||
      catalog.reasoningModels[0] ||
      "qwen3-coder:30b",
  );
  const topP = _safeNum(payload?.top_p, 0.9);
  const topK = _safeNum(payload?.top_k, 40);
  const maxTokens = _safeNum(payload?.max_tokens, 2048);
  const repeatPenalty = _safeNum(payload?.repeat_penalty, 1.1);
  const systemPrompt = String(
    payload?.system_prompt ||
      "Eres ATLAS, sistema autonomo de gestion inteligente. Responde con soluciones tecnicas concretas.",
  );
  const ollamaUrl = String(payload?.ollama_url || "http://localhost:11434");
  const logLevel = String(payload?.log_level || "info").toLowerCase();

  grid.innerHTML = `
    <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px">
      <div style="font-size:11px;color:var(--text-muted);margin-bottom:6px;font-weight:600;text-transform:uppercase;letter-spacing:.5px">Modo AI</div>
      ${_renderSelect("cfg-ai-mode", MODE_OPTIONS, mode)}
      <div style="margin-top:8px;font-size:11px;color:var(--text-muted)" id="cfg-mode-help">${_esc(_modeHelpText(mode))}</div>
    </div>
    <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px">
      <div style="font-size:11px;color:var(--text-muted);margin-bottom:6px;font-weight:600;text-transform:uppercase;letter-spacing:.5px">Proveedor principal</div>
      ${_renderSelect("cfg-provider", PROVIDER_OPTIONS, provider)}
    </div>
    <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px">
      <div style="font-size:11px;color:var(--text-muted);margin-bottom:6px;font-weight:600;text-transform:uppercase;letter-spacing:.5px">Modelo principal</div>
      ${_renderSelect("cfg-brain-model", _optionsFromModels(catalog.brainModels, true), brainModel)}
    </div>
    <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px">
      <div style="font-size:11px;color:var(--text-muted);margin-bottom:6px;font-weight:600;text-transform:uppercase;letter-spacing:.5px">Modelo codigo</div>
      ${_renderSelect("cfg-code-model", _optionsFromModels(catalog.codeModels, false), codeModel)}
    </div>
    <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px">
      <div style="font-size:11px;color:var(--text-muted);margin-bottom:6px;font-weight:600;text-transform:uppercase;letter-spacing:.5px">Modelo razonamiento</div>
      ${_renderSelect(
        "cfg-reasoning-model",
        _optionsFromModels(catalog.reasoningModels, false),
        reasoningModel,
      )}
    </div>
    <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px">
      <div style="font-size:11px;color:var(--text-muted);margin-bottom:6px;font-weight:600;text-transform:uppercase;letter-spacing:.5px">Ventana contexto</div>
      <input id="cfg-ctx-window" type="number" min="1024" max="128000" step="1024" value="${payload?.context_window || 8192}"
        style="width:100%;padding:7px 10px;background:var(--surface-0);border:1px solid var(--border);border-radius:6px;color:var(--text-primary);font-size:13px;box-sizing:border-box">
    </div>
    <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px">
      <div style="font-size:11px;color:var(--text-muted);margin-bottom:6px;font-weight:600;text-transform:uppercase;letter-spacing:.5px">Temperatura</div>
      <input id="cfg-temperature" type="number" min="0" max="2" step="0.05" value="${payload?.temperature ?? 0.2}"
        style="width:100%;padding:7px 10px;background:var(--surface-0);border:1px solid var(--border);border-radius:6px;color:var(--text-primary);font-size:13px;box-sizing:border-box">
    </div>
    <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px">
      <div style="font-size:11px;color:var(--text-muted);margin-bottom:6px;font-weight:600;text-transform:uppercase;letter-spacing:.5px">Top P</div>
      <input id="cfg-top-p" type="number" min="0" max="1" step="0.01" value="${topP}"
        style="width:100%;padding:7px 10px;background:var(--surface-0);border:1px solid var(--border);border-radius:6px;color:var(--text-primary);font-size:13px;box-sizing:border-box">
    </div>
    <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px">
      <div style="font-size:11px;color:var(--text-muted);margin-bottom:6px;font-weight:600;text-transform:uppercase;letter-spacing:.5px">Top K</div>
      <input id="cfg-top-k" type="number" min="0" max="200" step="1" value="${Math.round(topK)}"
        style="width:100%;padding:7px 10px;background:var(--surface-0);border:1px solid var(--border);border-radius:6px;color:var(--text-primary);font-size:13px;box-sizing:border-box">
    </div>
    <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px">
      <div style="font-size:11px;color:var(--text-muted);margin-bottom:6px;font-weight:600;text-transform:uppercase;letter-spacing:.5px">Max tokens</div>
      <input id="cfg-max-tokens" type="number" min="256" max="32768" step="64" value="${Math.round(maxTokens)}"
        style="width:100%;padding:7px 10px;background:var(--surface-0);border:1px solid var(--border);border-radius:6px;color:var(--text-primary);font-size:13px;box-sizing:border-box">
    </div>
    <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px">
      <div style="font-size:11px;color:var(--text-muted);margin-bottom:6px;font-weight:600;text-transform:uppercase;letter-spacing:.5px">Repeat penalty</div>
      <input id="cfg-repeat-penalty" type="number" min="0.8" max="2" step="0.05" value="${repeatPenalty}"
        style="width:100%;padding:7px 10px;background:var(--surface-0);border:1px solid var(--border);border-radius:6px;color:var(--text-primary);font-size:13px;box-sizing:border-box">
    </div>
    <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px">
      <div style="font-size:11px;color:var(--text-muted);margin-bottom:6px;font-weight:600;text-transform:uppercase;letter-spacing:.5px">Memoria contexto</div>
      ${_renderSelect(
        "cfg-ctx-type",
        [
          { value: "short", label: "Corto plazo" },
          { value: "long", label: "Largo plazo" },
        ],
        payload?.memory_context || "long",
      )}
    </div>
    <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px">
      <div style="font-size:11px;color:var(--text-muted);margin-bottom:6px;font-weight:600;text-transform:uppercase;letter-spacing:.5px">Nivel log</div>
      ${_renderSelect(
        "cfg-log-level",
        [
          { value: "debug", label: "Debug" },
          { value: "info", label: "Info" },
          { value: "warning", label: "Warning" },
          { value: "error", label: "Error" },
        ],
        logLevel,
      )}
    </div>
    <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px">
      <div style="font-size:11px;color:var(--text-muted);margin-bottom:6px;font-weight:600;text-transform:uppercase;letter-spacing:.5px">Respuesta streaming</div>
      ${_renderBooleanSelect("cfg-stream-response", payload?.stream_response)}
    </div>
    <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px">
      <div style="font-size:11px;color:var(--text-muted);margin-bottom:6px;font-weight:600;text-transform:uppercase;letter-spacing:.5px">Guardar historial</div>
      ${_renderBooleanSelect("cfg-save-history", payload?.save_history)}
    </div>
    <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px;grid-column:1 / -1">
      <div style="font-size:11px;color:var(--text-muted);margin-bottom:6px;font-weight:600;text-transform:uppercase;letter-spacing:.5px">URL Ollama</div>
      ${_renderInput("cfg-ollama-url", "http://localhost:11434", ollamaUrl)}
    </div>
    <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px;grid-column:1 / -1">
      <div style="font-size:11px;color:var(--text-muted);margin-bottom:6px;font-weight:600;text-transform:uppercase;letter-spacing:.5px">System prompt</div>
      ${_renderTextarea("cfg-system-prompt", "Instrucciones base del orquestador", systemPrompt, 4)}
    </div>
  `;

  const modeEl = container.querySelector("#cfg-ai-mode");
  modeEl?.addEventListener("change", () => _applyModeBehavior(container));
  _applyModeBehavior(container);
}

function _applyModeBehavior(container) {
  const mode = _normalizeMode(container.querySelector("#cfg-ai-mode")?.value);
  const providerEl = container.querySelector("#cfg-provider");
  const modelEl = container.querySelector("#cfg-brain-model");
  const helpEl = container.querySelector("#cfg-mode-help");

  if (helpEl) helpEl.textContent = _modeHelpText(mode);
  if (!providerEl || !modelEl) return;

  if (mode === AUTO_LOCAL_MODE) {
    providerEl.value = "ollama";
    modelEl.value = AUTO_LOCAL_MODEL;
    providerEl.disabled = true;
  } else {
    providerEl.disabled = false;
    if (modelEl.value === AUTO_LOCAL_MODEL) {
      const fallback = container?._cfgCatalog?.brainModels?.[0] || "llama3.1:8b";
      modelEl.value = fallback;
    }
  }
}

function _extractAiProviderMap(aiStatusRaw) {
  const aiStatus = _unwrapData(aiStatusRaw);
  const rows = Array.isArray(aiStatus?.providers) ? aiStatus.providers : [];
  const map = {};
  for (const row of rows) {
    const id = String(row?.id || "").trim().toLowerCase();
    if (!id) continue;
    map[id] = {
      available: typeof row?.available === "boolean" ? row.available : null,
      isFree: typeof row?.is_free === "boolean" ? row.is_free : null,
      name: String(row?.name || id),
    };
  }
  return map;
}

function _extractAgentRuntimeByProvider(agentModelsRaw) {
  const rows = _unwrapData(agentModelsRaw);
  const list = Array.isArray(rows) ? rows : [];
  const out = {};

  for (const row of list) {
    const providerId = String(row?.provider || "").trim().toLowerCase();
    if (!providerId) continue;
    if (!out[providerId]) {
      out[providerId] = {
        total: 0,
        availableCount: 0,
        availabilityKnown: false,
        warn: false,
        message: "",
      };
    }
    const bucket = out[providerId];
    bucket.total += 1;
    if (typeof row?.available === "boolean") {
      bucket.availabilityKnown = true;
      if (row.available) bucket.availableCount += 1;
    }
    const runtimeState = String(row?.runtime_state || "").trim().toLowerCase();
    if (runtimeState === "warn" || runtimeState === "err" || runtimeState === "error") {
      bucket.warn = true;
    }
    const msg = String(row?.status_message || row?.runtime_error || "").trim();
    if (msg && !bucket.message) bucket.message = msg;
  }

  return out;
}

function _renderProviders(
  container,
  brainStateRaw,
  workspaceRaw,
  aiStatusRaw,
  agentModelsRaw,
) {
  const el = container.querySelector("#cfg-providers");
  if (!el) return;

  const brain = brainStateRaw?.data ?? brainStateRaw ?? {};
  const credentials = brain?.credentials || {};
  const providerOptions = Array.isArray(brain?.provider_options)
    ? brain.provider_options
    : [];
  const countByProvider = _extractWorkspaceProviderCounts(workspaceRaw);
  const aiProviderMap = _extractAiProviderMap(aiStatusRaw);
  const runtimeByProvider = _extractAgentRuntimeByProvider(agentModelsRaw);

  const providers = new Set(["ollama"]);
  for (const p of providerOptions) providers.add(String(p?.id || "").trim());
  for (const key of Object.keys(credentials || {})) providers.add(String(key || ""));
  for (const key of Object.keys(countByProvider || {})) providers.add(String(key || ""));
  for (const key of Object.keys(aiProviderMap || {})) providers.add(String(key || ""));
  for (const key of Object.keys(runtimeByProvider || {})) providers.add(String(key || ""));

  const ordered = Array.from(providers)
    .filter(Boolean)
    .sort((a, b) => (a === "ollama" ? -1 : b === "ollama" ? 1 : a.localeCompare(b)));

  if (!ordered.length) {
    el.innerHTML = `<div class="empty-state"><div class="empty-title">Sin datos de proveedores</div></div>`;
    return;
  }

  el.innerHTML = `<div class="provider-grid">
    ${ordered
      .map((providerId) => {
        const modelCount = countByProvider[providerId] || 0;
        const cred = credentials[providerId] || {};
        const aiMeta = aiProviderMap[providerId] || {};
        const runtimeMeta = runtimeByProvider[providerId] || {};
        const configured =
          providerId === "ollama" ||
          Boolean(cred?.configured) ||
          modelCount > 0 ||
          (runtimeMeta?.total || 0) > 0;

        const runtimeAvailable =
          runtimeMeta?.availabilityKnown === true
            ? runtimeMeta.availableCount > 0
            : null;
        const aiAvailable =
          typeof aiMeta?.available === "boolean" ? aiMeta.available : null;
        const operational = runtimeAvailable !== null ? runtimeAvailable : aiAvailable;

        const down = operational === false;
        const warn = !down && Boolean(runtimeMeta?.warn);
        const statusClass = down ? "down" : "active";
        const dotClass = down ? "down" : "ok";
        const modelRuntimeText =
          runtimeMeta?.total > 0
            ? `${runtimeMeta.availableCount || 0}/${runtimeMeta.total} runtime`
            : "sin runtime";
        const billingTag =
          aiMeta?.isFree === true
            ? "Gratis"
            : aiMeta?.isFree === false
              ? "Pago/API"
              : "N/A";

        let statusLabel = "Activo";
        if (down) statusLabel = "No disponible";
        else if (!configured) statusLabel = "Sin credenciales";
        else if (warn) statusLabel = "Activo con alertas";

        const runtimeMsg = String(runtimeMeta?.message || "").slice(0, 120);
        return `<div class="provider-card ${statusClass}">
          <div class="provider-name">${_esc(providerId)}</div>
          <div class="provider-role">${modelCount} modelos workspace | ${_esc(modelRuntimeText)}</div>
          <div class="provider-role">Tipo: ${_esc(billingTag)}</div>
          <div class="provider-status">
            <div class="provider-dot ${dotClass}"></div>
            ${_esc(statusLabel)}
          </div>
          ${runtimeMsg ? `<div style="margin-top:6px;font-size:11px;color:var(--text-muted)">${_esc(runtimeMsg)}</div>` : ""}
        </div>`;
      })
      .join("")}
  </div>`;
}

function _renderObservability(
  container,
  autonomyRaw,
  aiStatusRaw,
  observabilityRaw,
  metricsRaw,
) {
  const el = container.querySelector("#cfg-observability");
  if (!el) return;

  const autonomy = _unwrapData(autonomyRaw);
  const kpis = autonomy?.kpis || {};
  const aiStatus = _unwrapData(aiStatusRaw);
  const observability = _unwrapData(observabilityRaw);
  const metrics = _unwrapData(metricsRaw);
  const routeToModel =
    aiStatus?.route_to_model && typeof aiStatus.route_to_model === "object"
      ? aiStatus.route_to_model
      : {};
  const telemetry =
    aiStatus?.telemetry && typeof aiStatus.telemetry === "object"
      ? aiStatus.telemetry
      : {};

  const totalRequests = _safeNum(
    observability?.total_requests,
    _safeNum(metrics?.total_requests, 0),
  );
  const activeRequests = _safeNum(
    observability?.active_requests,
    _safeNum(metrics?.active_requests, 0),
  );
  const memoryMb = _safeNum(
    observability?.memory_mb,
    _safeNum(metrics?.memory_mb, 0),
  );
  const healthScore = _safeNum(
    observability?.health_score,
    _safeNum(autonomy?.level, 0),
  );
  const successRate = _safeNum(kpis?.success_rate_24h, _safeNum(kpis?.success_rate, 0));
  const queue = _safeNum(kpis?.pending_queue_count, 0);
  const incidents = _safeNum(kpis?.incidents_resolved, 0);
  const mttr = _safeNum(kpis?.mttr_minutes, 0);
  const spent = _safeNum(aiStatus?.spent_today_usd, 0);
  const budget = _safeNum(aiStatus?.budget_daily_usd, 0);
  const maxTaskCost = _safeNum(aiStatus?.max_cost_per_task_usd, 0);
  const externalApisLabel =
    typeof aiStatus?.external_apis_allowed === "boolean"
      ? aiStatus.external_apis_allowed
        ? "Permitidas"
        : "Bloqueadas"
      : "Sin datos";
  const paidApisLabel =
    typeof aiStatus?.paid_api_allowed === "boolean"
      ? aiStatus.paid_api_allowed
        ? "Permitidas"
        : "Bloqueadas"
      : "Sin datos";

  const routeRows = Object.entries(routeToModel).slice(0, 8);
  const telemetryRows = Object.entries(telemetry)
    .map(([id, row]) => ({
      id,
      requests: _safeNum(row?.requests, 0),
      successRate: _safeNum(row?.success_rate, 0) * 100,
      errorRate: _safeNum(row?.error_rate, 0) * 100,
      latency: _safeNum(row?.latency_avg_ms, 0),
      cost: _safeNum(row?.cost_estimate_usd, 0),
    }))
    .sort((a, b) => b.requests - a.requests)
    .slice(0, 4);

  el.innerHTML = `
    <div class="stat-row" style="margin-bottom:12px">
      <div class="stat-card">
        <div class="stat-card-label">Solicitudes</div>
        <div class="stat-card-value" style="font-size:18px">${_esc(_formatInt(totalRequests))}</div>
      </div>
      <div class="stat-card">
        <div class="stat-card-label">Solicitudes activas</div>
        <div class="stat-card-value ${activeRequests > 0 ? "accent" : ""}" style="font-size:18px">${_esc(_formatInt(activeRequests))}</div>
      </div>
      <div class="stat-card">
        <div class="stat-card-label">Memoria proceso</div>
        <div class="stat-card-value" style="font-size:18px">${_esc(`${_formatDecimal(memoryMb, 1, "0.0")} MB`)}</div>
      </div>
      <div class="stat-card">
        <div class="stat-card-label">Salud observabilidad</div>
        <div class="stat-card-value ${healthScore >= 80 ? "green" : healthScore >= 50 ? "accent" : "orange"}" style="font-size:18px">${_esc(_formatDecimal(healthScore, 1, "--"))}</div>
      </div>
      <div class="stat-card">
        <div class="stat-card-label">Exito 24h</div>
        <div class="stat-card-value" style="font-size:18px">${_esc(_formatPercent(successRate, 1, "--"))}</div>
      </div>
      <div class="stat-card">
        <div class="stat-card-label">Cola operativa</div>
        <div class="stat-card-value" style="font-size:18px">${_esc(_formatInt(queue))}</div>
      </div>
      <div class="stat-card">
        <div class="stat-card-label">Incidentes resueltos</div>
        <div class="stat-card-value green" style="font-size:18px">${_esc(_formatInt(incidents))}</div>
      </div>
      <div class="stat-card">
        <div class="stat-card-label">MTTR</div>
        <div class="stat-card-value" style="font-size:18px">${_esc(`${_formatDecimal(mttr, 1, "0.0")} min`)}</div>
      </div>
    </div>

    <div style="display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:12px">
      <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:12px">
        <div style="font-size:11px;color:var(--text-muted);font-weight:600;text-transform:uppercase;letter-spacing:.5px;margin-bottom:8px">Presupuesto y politicas</div>
        <div style="font-size:12px;color:var(--text-primary);margin-bottom:6px">Gastado hoy: <strong>${_esc(_formatMoney(spent, "$0.00"))}</strong></div>
        <div style="font-size:12px;color:var(--text-primary);margin-bottom:6px">Presupuesto diario: <strong>${_esc(_formatMoney(budget, "$0.00"))}</strong></div>
        <div style="font-size:12px;color:var(--text-primary);margin-bottom:6px">Max costo por tarea: <strong>${_esc(_formatMoney(maxTaskCost, "$0.00"))}</strong></div>
        <div style="font-size:11px;color:var(--text-muted)">
          APIs externas: ${_esc(externalApisLabel)} | APIs de pago: ${_esc(paidApisLabel)}
        </div>
      </div>

      <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:12px">
        <div style="font-size:11px;color:var(--text-muted);font-weight:600;text-transform:uppercase;letter-spacing:.5px;margin-bottom:8px">Ruteo automatico por especialidad</div>
        ${
          routeRows.length
            ? routeRows
                .map(
                  ([route, model]) =>
                    `<div style="display:flex;justify-content:space-between;gap:8px;padding:5px 0;border-bottom:1px solid var(--border-subtle)">
                      <span style="font-size:12px;color:var(--text-muted)">${_esc(route)}</span>
                      <span style="font-size:11px;color:var(--text-primary);font-family:var(--font-mono);text-align:right">${_esc(String(model || "--"))}</span>
                    </div>`,
                )
                .join("")
            : `<div style="font-size:12px;color:var(--text-muted)">Sin rutas registradas.</div>`
        }
      </div>

      <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:12px">
        <div style="font-size:11px;color:var(--text-muted);font-weight:600;text-transform:uppercase;letter-spacing:.5px;margin-bottom:8px">Telemetria reciente por modelo</div>
        ${
          telemetryRows.length
            ? telemetryRows
                .map(
                  (row) =>
                    `<div style="padding:7px 0;border-bottom:1px solid var(--border-subtle)">
                      <div style="font-size:11px;color:var(--text-primary);font-family:var(--font-mono)">${_esc(row.id)}</div>
                      <div style="font-size:11px;color:var(--text-muted)">
                        req: ${_esc(_formatInt(row.requests, "0"))} | exito: ${_esc(_formatPercent(row.successRate, 1, "0.0%"))} | error: ${_esc(_formatPercent(row.errorRate, 1, "0.0%"))}
                      </div>
                      <div style="font-size:11px;color:var(--text-muted)">
                        lat avg: ${_esc(`${_formatDecimal(row.latency, 1, "0.0")} ms`)} | costo: ${_esc(_formatMoney(row.cost, "$0.00"))}
                      </div>
                    </div>`,
                )
                .join("")
            : `<div style="font-size:12px;color:var(--text-muted)">Aun no hay telemetria de inferencias.</div>`
        }
      </div>
    </div>
  `;
}

function _mergeActivityHistory(container, timelineRows) {
  const incoming = Array.isArray(timelineRows) ? timelineRows : [];
  const existing = Array.isArray(container?._cfgActivityHistory)
    ? container._cfgActivityHistory
    : [];
  const merged = [];
  const seen = new Set();

  for (const row of [...incoming, ...existing]) {
    const event = String(row?.event || row?.message || "").trim();
    if (!event) continue;
    const ts = row?.ts || row?.timestamp || "";
    const key = `${ts}|${event}`;
    if (seen.has(key)) continue;
    seen.add(key);
    merged.push({
      ts,
      event,
      kind: String(row?.kind || "info"),
      result: String(row?.result || "n/a"),
    });
  }

  merged.sort((a, b) => _parseTsMs(b.ts) - _parseTsMs(a.ts));
  const trimmed = merged.slice(0, ACTIVITY_HISTORY_LIMIT);
  container._cfgActivityHistory = trimmed;
  return trimmed;
}

function _renderLiveActivity(
  container,
  autonomyRaw,
  aiStatusRaw,
  observabilityRaw,
  agentModelsRaw,
) {
  const el = container.querySelector("#cfg-activity");
  if (!el) return;

  const autonomy = _unwrapData(autonomyRaw);
  const kpis = autonomy?.kpis || {};
  const manager = autonomy?.manager || {};
  const daemon = manager?.daemon || {};
  const latest = manager?.latest || {};
  const timelineRows = Array.isArray(autonomy?.timeline) ? autonomy.timeline : [];
  const historyRows = _mergeActivityHistory(container, timelineRows);

  const observability = _unwrapData(observabilityRaw);
  const activeRequests = _safeNum(observability?.active_requests, 0);
  const inProgress = _safeNum(kpis?.tasks_in_progress, 0);
  const queue = _safeNum(kpis?.pending_queue_count, 0);
  const autonomyLevel = _safeNum(autonomy?.level, 0);

  const currentPhase = String(
    daemon?.current_phase || latest?.current_phase || "idle",
  ).toLowerCase();
  const currentAction = String(
    daemon?.current_action || latest?.current_action || "",
  ).trim();
  const currentSummary = String(latest?.summary || "").trim();
  const currentTask =
    currentAction ||
    currentSummary ||
    historyRows[0]?.event ||
    "Monitoreo activo del sistema";

  const aiStatus = _unwrapData(aiStatusRaw);
  const providersRows = Array.isArray(_unwrapData(agentModelsRaw))
    ? _unwrapData(agentModelsRaw)
    : [];
  const providersTotal = new Set(
    providersRows
      .map((row) => String(row?.provider || "").trim().toLowerCase())
      .filter(Boolean),
  ).size;
  const providersUp = new Set(
    providersRows
      .filter((row) => row?.available)
      .map((row) => String(row?.provider || "").trim().toLowerCase())
      .filter(Boolean),
  ).size;

  const workingNow =
    WORKING_PHASES.has(currentPhase) || inProgress > 0 || activeRequests > 0;
  const statusText = workingNow ? "Trabajando" : "En espera";
  const statusTone = workingNow ? "green" : "accent";
  const nextRun = String(daemon?.next_run_at || "").trim();
  const nextRunLabel = nextRun ? _formatTime(nextRun) : "--:--";

  el.innerHTML = `
    <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:14px;margin-bottom:12px">
      <div style="display:flex;align-items:center;justify-content:space-between;gap:12px;flex-wrap:wrap">
        <div>
          <div style="font-size:11px;color:var(--text-muted);font-weight:600;text-transform:uppercase;letter-spacing:.5px">Estado operativo en vivo</div>
          <div style="font-size:17px;color:var(--text-primary);margin-top:4px">${_esc(currentTask)}</div>
        </div>
        <div style="display:flex;align-items:center;gap:8px">
          <div class="provider-dot ok"></div>
          <div class="stat-card-value ${statusTone}" style="font-size:16px">${_esc(statusText)}</div>
        </div>
      </div>

      <div style="display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:10px;margin-top:12px">
        <div style="background:var(--surface-0);border:1px solid var(--border-subtle);border-radius:8px;padding:10px">
          <div style="font-size:10px;color:var(--text-muted);text-transform:uppercase">Fase actual</div>
          <div style="font-size:13px;color:var(--text-primary)">${_esc(currentPhase || "idle")}</div>
        </div>
        <div style="background:var(--surface-0);border:1px solid var(--border-subtle);border-radius:8px;padding:10px">
          <div style="font-size:10px;color:var(--text-muted);text-transform:uppercase">Solicitudes activas</div>
          <div style="font-size:13px;color:var(--text-primary)">${_esc(_formatInt(activeRequests, "0"))}</div>
        </div>
        <div style="background:var(--surface-0);border:1px solid var(--border-subtle);border-radius:8px;padding:10px">
          <div style="font-size:10px;color:var(--text-muted);text-transform:uppercase">Tareas en progreso</div>
          <div style="font-size:13px;color:var(--text-primary)">${_esc(_formatInt(inProgress, "0"))}</div>
        </div>
        <div style="background:var(--surface-0);border:1px solid var(--border-subtle);border-radius:8px;padding:10px">
          <div style="font-size:10px;color:var(--text-muted);text-transform:uppercase">Cola total</div>
          <div style="font-size:13px;color:var(--text-primary)">${_esc(_formatInt(queue, "0"))}</div>
        </div>
        <div style="background:var(--surface-0);border:1px solid var(--border-subtle);border-radius:8px;padding:10px">
          <div style="font-size:10px;color:var(--text-muted);text-transform:uppercase">Proveedores activos</div>
          <div style="font-size:13px;color:var(--text-primary)">${_esc(`${providersUp}/${providersTotal || 0}`)}</div>
        </div>
        <div style="background:var(--surface-0);border:1px solid var(--border-subtle);border-radius:8px;padding:10px">
          <div style="font-size:10px;color:var(--text-muted);text-transform:uppercase">Nivel autonomia</div>
          <div style="font-size:13px;color:var(--text-primary)">${_esc(_formatDecimal(autonomyLevel, 1, "--"))}</div>
        </div>
      </div>

      <div style="margin-top:10px;font-size:11px;color:var(--text-muted)">
        Proxima corrida daemon: ${_esc(nextRunLabel)} | Router IA: ${aiStatus?.ok ? "activo" : "sin datos"} | Actualizado: ${_esc(_formatTime(Date.now()))}
      </div>
    </div>

    <div style="background:var(--surface-1);border:1px solid var(--border);border-radius:10px;padding:12px">
      <div style="font-size:11px;color:var(--text-muted);font-weight:600;text-transform:uppercase;letter-spacing:.5px;margin-bottom:8px">Bitacora reciente de actividad</div>
      ${
        historyRows.length
          ? historyRows
              .map(
                (row) =>
                  `<div style="padding:8px 0;border-bottom:1px solid var(--border-subtle)">
                    <div style="font-size:11px;color:var(--text-muted)">${_esc(_formatTime(row.ts))} | ${_esc(row.kind)} | ${_esc(row.result)}</div>
                    <div style="font-size:12px;color:var(--text-primary);margin-top:2px">${_esc(row.event)}</div>
                  </div>`,
              )
              .join("")
          : `<div style="font-size:12px;color:var(--text-muted)">Sin eventos recientes en timeline.</div>`
      }
    </div>
  `;
}

function _renderOllamaTable(container, models) {
  const el = container.querySelector("#cfg-ollama");
  if (!el) return;
  if (!models.length) {
    el.innerHTML =
      '<div class="empty-state"><div class="empty-title">Sin modelos Ollama locales</div><div class="empty-sub">Verifica Ollama y vuelve a refrescar.</div></div>';
    return;
  }
  el.innerHTML = `
    <div style="overflow-x:auto;border-radius:8px;border:1px solid var(--border-subtle)">
      <table style="width:100%;border-collapse:collapse;font-size:12px">
        <thead><tr style="background:var(--surface-1)">
          <th style="text-align:left;padding:8px 12px;color:var(--text-muted);font-size:10px;text-transform:uppercase;border-bottom:1px solid var(--border-subtle)">Modelo</th>
          <th style="text-align:left;padding:8px 12px;color:var(--text-muted);font-size:10px;text-transform:uppercase;border-bottom:1px solid var(--border-subtle)">Estado</th>
        </tr></thead>
        <tbody>
          ${models
            .map((name, index) => {
              return `<tr style="${index % 2 ? "background:var(--surface-0)" : ""}">
                <td style="padding:7px 12px;border-bottom:1px solid var(--border-subtle);color:var(--text-primary);font-family:var(--font-mono);font-size:11px">${_esc(name)}</td>
                <td style="padding:7px 12px;border-bottom:1px solid var(--border-subtle)"><span class="chip green">disponible</span></td>
              </tr>`;
            })
            .join("")}
        </tbody>
      </table>
    </div>`;
}

async function _saveConfig(container) {
  const msg = container.querySelector("#cfg-msg");
  const btnSave = container.querySelector("#btn-cfg-save");
  if (btnSave) btnSave.disabled = true;
  if (msg) msg.textContent = "Guardando...";

  const get = (selector) => container.querySelector(selector)?.value || "";
  const parseFloatSafe = (value, fallback) => {
    const n = Number.parseFloat(String(value ?? "").trim());
    return Number.isFinite(n) ? n : fallback;
  };
  const parseIntSafe = (value, fallback) => {
    const n = Number.parseInt(String(value ?? "").trim(), 10);
    return Number.isFinite(n) ? n : fallback;
  };
  const mode = _normalizeMode(get("#cfg-ai-mode"));
  let provider = String(get("#cfg-provider") || "ollama").toLowerCase();
  let model = String(get("#cfg-brain-model") || "").trim();

  if (mode === AUTO_LOCAL_MODE) {
    provider = "ollama";
    model = AUTO_LOCAL_MODEL;
  }

  const codeModel = String(get("#cfg-code-model") || model || "qwen2.5-coder:7b");
  const reasoningModel = String(
    get("#cfg-reasoning-model") || model || "qwen3-coder:30b",
  );
  const chatModel =
    model === AUTO_LOCAL_MODEL || !model ? "llama3.1:8b" : model;

  const payload = {
    mode,
    provider,
    model,
    brain_model: model,
    code_model: codeModel,
    reasoning_model: reasoningModel,
    temperature: parseFloatSafe(get("#cfg-temperature"), 0.2),
    top_p: parseFloatSafe(get("#cfg-top-p"), 0.9),
    top_k: parseIntSafe(get("#cfg-top-k"), 40),
    max_tokens: parseIntSafe(get("#cfg-max-tokens"), 2048),
    repeat_penalty: parseFloatSafe(get("#cfg-repeat-penalty"), 1.1),
    context_window: parseIntSafe(get("#cfg-ctx-window"), 8192),
    memory_context: get("#cfg-ctx-type") || "long",
    system_prompt: String(get("#cfg-system-prompt") || "").trim(),
    ollama_url: String(get("#cfg-ollama-url") || "http://localhost:11434").trim(),
    stream_response: _toBoolean(get("#cfg-stream-response"), false),
    save_history: _toBoolean(get("#cfg-save-history"), true),
    log_level: String(get("#cfg-log-level") || "info").toLowerCase(),
    specialists: {
      code: { enabled: true, model: codeModel },
      vision: { enabled: true, model: "qwen3-vl:30b" },
      chat: { enabled: true, model: chatModel },
      analysis: { enabled: true, model: reasoningModel },
      creative: { enabled: false, model: chatModel },
    },
  };

  try {
    const response = await fetch("/config/ai", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const body = await response.json().catch(() => ({}));
    if (!response.ok || body?.ok === false) {
      throw new Error(body?.error || `HTTP ${response.status}`);
    }
    window.AtlasToast?.show("Configuracion guardada", "success");
    if (msg) msg.textContent = "OK Guardado";
    const ts = container.querySelector("#cfg-save-ts");
    if (ts) ts.textContent = `Guardado a las ${new Date().toLocaleTimeString()}`;
  } catch (error) {
    window.AtlasToast?.show(error.message, "error");
    if (msg) msg.textContent = `Error: ${error.message}`;
  } finally {
    if (btnSave) btnSave.disabled = false;
    setTimeout(() => {
      const m = container.querySelector("#cfg-msg");
      if (m) m.textContent = "";
    }, 4000);
  }
}

async function _testConn(container) {
  const msg = container.querySelector("#cfg-msg");
  const btn = container.querySelector("#btn-cfg-test");
  if (btn) btn.disabled = true;
  if (msg) msg.textContent = "Probando conexion...";
  try {
    const response = await fetch("/config/ai/test", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        model: container.querySelector("#cfg-brain-model")?.value || "",
      }),
    });
    const body = await response.json().catch(() => ({}));
    if (!response.ok || body?.ok === false) {
      throw new Error(body?.error || `HTTP ${response.status}`);
    }
    const latency = body?.data?.latency_ms || body?.latency_ms || "";
    window.AtlasToast?.show(
      `Conexion OK${latency ? ` (${latency}ms)` : ""}`,
      "success",
    );
    if (msg) msg.textContent = `OK Conexion${latency ? ` ${latency}ms` : ""}`;
  } catch (error) {
    window.AtlasToast?.show(`Test fallido: ${error.message}`, "error");
    if (msg) msg.textContent = `Error: ${error.message}`;
  } finally {
    if (btn) btn.disabled = false;
    setTimeout(() => {
      const m = container.querySelector("#cfg-msg");
      if (m) m.textContent = "";
    }, 6000);
  }
}

window.AtlasModuleConfig = { id: "config" };
