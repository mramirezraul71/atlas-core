const fs = require("fs");
const path = require("path");

const repoRoot = path.resolve(__dirname, "..", "..");
const logsDir = path.join(repoRoot, "logs");
const snapshotLogPath = path.join(logsDir, "snapshot_safe_diagnostic.log");
const actuatorLogPath = path.join(logsDir, "atlas_actuators.log");

function ensureLogDir() {
  fs.mkdirSync(logsDir, { recursive: true });
}

function nowIso() {
  return new Date().toISOString();
}

function flatValue(value) {
  if (value === null || value === undefined) return "";
  const text = typeof value === "string" ? value : JSON.stringify(value);
  return text.replace(/\s+/g, " ").trim();
}

function formatEvent(event, payload = {}) {
  const fields = Object.entries(payload)
    .map(([k, v]) => `${k}=${flatValue(v)}`)
    .join(" ");
  return `${event}${fields ? " " + fields : ""}`;
}

function writeSnapshotLine(line) {
  ensureLogDir();
  fs.appendFileSync(snapshotLogPath, `${nowIso()} ${line}\n`, "utf8");
}

function writeActuatorLine(line) {
  ensureLogDir();
  fs.appendFileSync(actuatorLogPath, `${nowIso()} ${line}\n`, "utf8");
}

function audit(event, payload = {}) {
  const line = formatEvent(event, payload);
  writeActuatorLine(line);
  writeSnapshotLine(`ATLAS_ACTUATOR ${line}`);
}

module.exports = {
  audit,
  repoRoot,
  snapshotLogPath
};
