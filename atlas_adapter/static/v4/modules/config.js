/**
 * ATLAS v4 - AI Configuration Module
 * - Uses real model inventory from local Ollama and workspace model catalog
 * - Adds "automatic local" mode and keeps it as default
 * - Saves compatible payload to /config/ai
 */
import { poll, stop } from "../lib/polling.js";

const POLL_ID = "config-module";
const AUTO_LOCAL_MODE = "auto_local";
const AUTO_LOCAL_MODEL = "auto_local";

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
    poll(POLL_ID, "/health", 12000, (data) => {
      if (data) _renderHealth(container, data);
    });
  },

  destroy() {
    stop(POLL_ID);
  },
  badge() {
    return null;
  },
};

async function _loadAll(container) {
  const [healthRes, aiRes, workspaceRes, ollamaRes, brainStateRes] =
    await Promise.all([
      fetch("/health").then((r) => r.json()).catch(() => null),
      fetch("/config/ai").then((r) => r.json()).catch(() => null),
      fetch("/api/workspace/interpreter/models")
        .then((r) => r.json())
        .catch(() => null),
      fetch("/config/ai/ollama-models").then((r) => r.json()).catch(() => null),
      fetch("/api/brain/state").then((r) => r.json()).catch(() => null),
    ]);

  const workspaceModels = _extractWorkspaceModels(workspaceRes);
  const ollamaModels = _extractOllamaModels(ollamaRes);
  const catalog = _buildCatalog(workspaceModels, ollamaModels);
  container._cfgCatalog = catalog;
  container._cfgLastAiPayload = aiRes?.data ?? aiRes ?? {};

  if (healthRes) _renderHealth(container, healthRes);
  _renderAIForm(container, container._cfgLastAiPayload, catalog);
  _renderProviders(container, brainStateRes, workspaceRes);
  _renderOllamaTable(container, ollamaModels);
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
      <div class="stat-card-label">Score</div>
      <div class="stat-card-value" style="font-size:18px">${_esc(String(payload.score ?? "--"))}</div>
    </div>
    <div class="stat-card">
      <div class="stat-card-label">Uptime</div>
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

function _renderProviders(container, brainStateRaw, workspaceRaw) {
  const el = container.querySelector("#cfg-providers");
  if (!el) return;

  const brain = brainStateRaw?.data ?? brainStateRaw ?? {};
  const credentials = brain?.credentials || {};
  const providerOptions = Array.isArray(brain?.provider_options)
    ? brain.provider_options
    : [];
  const workspaceModels = _extractWorkspaceModels(workspaceRaw);

  const countByProvider = {};
  for (const id of workspaceModels) {
    const provider = id.includes("/") ? id.split("/")[0] : "ollama";
    countByProvider[provider] = (countByProvider[provider] || 0) + 1;
  }

  const providers = new Set(["ollama"]);
  for (const p of providerOptions) providers.add(String(p?.id || "").trim());
  for (const key of Object.keys(credentials || {})) providers.add(String(key || ""));
  for (const key of Object.keys(countByProvider || {})) providers.add(String(key || ""));

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
        const cred = credentials[providerId] || {};
        const configured =
          providerId === "ollama" ? true : Boolean(cred?.configured);
        const modelCount = countByProvider[providerId] || 0;
        const statusClass = configured ? "active" : "down";
        const dotClass = configured ? "ok" : "down";
        return `<div class="provider-card ${statusClass}">
          <div class="provider-name">${_esc(providerId)}</div>
          <div class="provider-role">${modelCount} modelos detectados</div>
          <div class="provider-status">
            <div class="provider-dot ${dotClass}"></div>
            ${configured ? "Activo" : "Sin credenciales"}
          </div>
        </div>`;
      })
      .join("")}
  </div>`;
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
    temperature: parseFloat(get("#cfg-temperature") || "0.2"),
    context_window: parseInt(get("#cfg-ctx-window") || "8192", 10),
    memory_context: get("#cfg-ctx-type") || "long",
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
