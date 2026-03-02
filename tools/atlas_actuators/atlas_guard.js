const fs = require("fs");
const path = require("path");
const crypto = require("crypto");
const { repoRoot, audit } = require("./atlas_logger");

const stateDir = path.join(repoRoot, "state", "atlas_actuators");
const statePath = path.join(stateDir, "loop_guard.json");
const rootEnvPath = path.join(repoRoot, ".env");

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

function readCoreToken() {
  return process.env.ATLAS_CENTRAL_CORE || getEnvFromFile(rootEnvPath, "ATLAS_CENTRAL_CORE");
}

function authorize(coreTokenArg) {
  const expected = readCoreToken();
  if (!expected) {
    throw new Error("ATLAS_CENTRAL_CORE is missing (env or .env).");
  }
  const provided = coreTokenArg || process.env.ATLAS_CENTRAL_CORE;
  if (!provided) {
    throw new Error("Missing --core-token and env token not available.");
  }
  if (provided !== expected) {
    throw new Error("Invalid ATLAS_CENTRAL_CORE token.");
  }
  return true;
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
  readCoreToken
};
