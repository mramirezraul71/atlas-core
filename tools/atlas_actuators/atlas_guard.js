const fs = require("fs");
const path = require("path");
const crypto = require("crypto");
const { repoRoot, audit } = require("./atlas_logger");

const stateDir = path.join(repoRoot, "state", "atlas_actuators");
const statePath = path.join(stateDir, "loop_guard.json");
const rootEnvPath = path.join(repoRoot, ".env");
const configEnvPath = path.join(repoRoot, "config", "atlas.env");
const configEnvExamplePath = path.join(repoRoot, "config", "atlas.env.example");

function getEnvFromFile(envPath, key) {
  if (!fs.existsSync(envPath)) return "";
  const lines = fs.readFileSync(envPath, "utf8").split(/\r?\n/);
  for (const line of lines) {
    if (!line || line.trim().startsWith("#")) continue;
    const idx = line.indexOf("=");
    if (idx < 0) continue;
    const k = line.slice(0, idx).trim();
    if (k !== key) continue;
    return line.slice(idx + 1).trim().replace(/^['"]|['"]$/g, "");
  }
  return "";
}

function getConfigValue(key, fallback = "") {
  return (
    process.env[key] ||
    getEnvFromFile(rootEnvPath, key) ||
    getEnvFromFile(configEnvPath, key) ||
    getEnvFromFile(configEnvExamplePath, key) ||
    fallback
  );
}

function normalizeMode(value) {
  const mode = String(value || "").trim().toLowerCase();
  if (mode === "grow") return "growth";
  if (mode === "growth") return "growth";
  if (mode === "governed") return "governed";
  return "governed";
}

function isTruthy(value) {
  const v = String(value || "").trim().toLowerCase();
  return v === "1" || v === "true" || v === "yes" || v === "on";
}

function isPlaceholderSecret(value) {
  const v = String(value || "").trim().toUpperCase();
  return !v || v.startsWith("CHANGE_ME") || v === "YOUR_TOKEN_HERE";
}

function getGovernanceContext() {
  const mode = normalizeMode(getConfigValue("GOVERNANCE_MODE", "governed"));
  const bind = isTruthy(getConfigValue("ATLAS_ACTUATORS_BIND_GOVERNANCE", "true"));
  const allowGoverned = isTruthy(getConfigValue("ATLAS_ACTUATOR_ALLOW_GOVERNED", "false"));
  const forceFullAutonomy = getConfigValue("ATLAS_ACTUATOR_FULL_AUTONOMY", "");
  const fullAutonomy = forceFullAutonomy
    ? isTruthy(forceFullAutonomy)
    : mode === "growth";
  return {
    mode,
    bind_governance: bind,
    full_autonomy: fullAutonomy,
    allow_governed: allowGoverned
  };
}

function readCoreToken() {
  const direct = getConfigValue("ATLAS_CENTRAL_CORE", "");
  if (!isPlaceholderSecret(direct)) return direct;
  const governanceSecret = getConfigValue("APPROVALS_CHAIN_SECRET", "");
  if (!isPlaceholderSecret(governanceSecret)) return governanceSecret;
  return "";
}

function authorize(coreTokenArg, options = {}) {
  const governance = getGovernanceContext();
  const allowGoverned = Boolean(options.allowGoverned || governance.allow_governed);
  if (governance.bind_governance && governance.mode !== "growth" && !allowGoverned) {
    throw new Error(
      `Actuators blocked by governance mode=${governance.mode}. Switch to growth/grow or use --allow-governed.`
    );
  }

  const expected = readCoreToken();
  if (!expected) {
    if (governance.full_autonomy) {
      return {
        ok: true,
        governance,
        auth_mode: "governance_full_autonomy_no_token"
      };
    }
    throw new Error("ATLAS_CENTRAL_CORE/APPROVALS_CHAIN_SECRET missing or placeholder.");
  }
  let provided = coreTokenArg || process.env.ATLAS_CENTRAL_CORE;
  if (!provided && governance.full_autonomy) {
    provided = expected;
  }
  if (!provided) {
    throw new Error("Missing --core-token and env token not available.");
  }
  if (provided !== expected) {
    throw new Error("Invalid ATLAS_CENTRAL_CORE token.");
  }
  return {
    ok: true,
    governance,
    auth_mode: "token"
  };
}

function loadState() {
  if (!fs.existsSync(statePath)) {
    return { events: [] };
  }
  try {
    return JSON.parse(fs.readFileSync(statePath, "utf8"));
  } catch {
    return { events: [] };
  }
}

function saveState(state) {
  fs.mkdirSync(stateDir, { recursive: true });
  fs.writeFileSync(statePath, JSON.stringify(state, null, 2), "utf8");
}

function createActionKey(input) {
  return crypto.createHash("sha1").update(JSON.stringify(input)).digest("hex");
}

function enforceLoopGuard(actionKey, options = {}) {
  const cooldownSeconds = Number(options.cooldownSeconds || 8);
  const maxEvents = Number(options.maxEvents || 40);
  const windowSeconds = Number(options.windowSeconds || 300);
  const now = Date.now();
  const state = loadState();
  const windowStart = now - windowSeconds * 1000;

  state.events = (state.events || []).filter((e) => Number(e.ts) >= windowStart);

  const perKey = state.events.filter((e) => e.key === actionKey);
  const last = perKey[perKey.length - 1];
  if (last && now - Number(last.ts) < cooldownSeconds * 1000) {
    const deltaMs = now - Number(last.ts);
    const msg = `Loop guard cooldown: wait ${cooldownSeconds}s (delta=${deltaMs}ms).`;
    audit("ACTUATOR_LOOP_BLOCK", {
      action_key: actionKey,
      reason: "cooldown",
      delta_ms: deltaMs
    });
    throw new Error(msg);
  }

  if (perKey.length >= maxEvents) {
    audit("ACTUATOR_LOOP_BLOCK", {
      action_key: actionKey,
      reason: "max_events",
      max_events: maxEvents,
      window_seconds: windowSeconds
    });
    throw new Error(`Loop guard max_events reached (${maxEvents}/${windowSeconds}s).`);
  }

  state.events.push({ key: actionKey, ts: now });
  saveState(state);
}

module.exports = {
  authorize,
  createActionKey,
  enforceLoopGuard,
  readCoreToken,
  getGovernanceContext
};
