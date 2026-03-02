const fs = require("fs");
const path = require("path");
const { spawnSync } = require("child_process");
const { audit, repoRoot } = require("./atlas_logger");
const { readCoreToken } = require("./atlas_guard");

function checkDependency(name) {
  try {
    require.resolve(name);
    return true;
  } catch {
    return false;
  }
}

function checkTaskOk() {
  const ps = "Get-ScheduledTask -TaskName 'ATLAS_SnapshotSafe' | Out-Null; if ($?) { 'ok' } else { 'missing' }";
  const proc = spawnSync("powershell", ["-NoProfile", "-Command", ps], { encoding: "utf8" });
  if (proc.status !== 0) {
    return { ok: false, detail: (proc.stderr || "").trim() || "Get-ScheduledTask failed" };
  }
  const out = (proc.stdout || "").trim().toLowerCase();
  return {
    ok: out === "ok",
    detail: out === "ok" ? "ATLAS_SnapshotSafe found" : "ATLAS_SnapshotSafe missing"
  };
}

function main() {
  const lastRunPath = path.join(repoRoot, "state", "atlas_actuators", "last_action_run.json");
  const result = {
    ok: true,
    ts: new Date().toISOString(),
    checks: {
      token: Boolean(readCoreToken()),
      playwright: checkDependency("playwright"),
      puppeteer: checkDependency("puppeteer"),
      last_action_run_exists: fs.existsSync(lastRunPath),
      task_ok: false,
      task_detail: ""
    }
  };

  const task = checkTaskOk();
  result.checks.task_ok = task.ok;
  result.checks.task_detail = task.detail;

  if (!result.checks.token || !result.checks.playwright || !result.checks.puppeteer || !result.checks.task_ok) {
    result.ok = false;
  }

  const event = result.ok ? "ACTUATOR_HEALTH_OK" : "ACTUATOR_HEALTH_WARN";
  audit(event, {
    token: result.checks.token ? 1 : 0,
    playwright: result.checks.playwright ? 1 : 0,
    puppeteer: result.checks.puppeteer ? 1 : 0,
    task_ok: result.checks.task_ok ? 1 : 0
  });

  process.stdout.write(`${JSON.stringify(result)}\n`);
  if (!result.ok) process.exitCode = 1;
}

main();
