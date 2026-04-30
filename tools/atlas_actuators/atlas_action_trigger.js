const fs = require("fs");
const path = require("path");
const crypto = require("crypto");
const { chromium } = require("playwright");
const puppeteer = require("puppeteer");
const { audit, repoRoot } = require("./atlas_logger");
const {
  authorize,
  createActionKey,
  enforceLoopGuard
} = require("./atlas_guard");

const ALLOWED_ACTIONS = new Set([
  "navigate",
  "click",
  "type",
  "press",
  "wait",
  "waitForSelector",
  "screenshot"
]);

function parseArgs(argv) {
  const result = {};
  for (let i = 0; i < argv.length; i += 1) {
    const raw = argv[i];
    if (!raw.startsWith("--")) continue;
    const key = raw.slice(2);
    const next = argv[i + 1];
    if (!next || next.startsWith("--")) {
      result[key] = true;
    } else {
      result[key] = next;
      i += 1;
    }
  }
  return result;
}

function toBoolean(value, fallback = false) {
  if (value === undefined || value === null) return fallback;
  return String(value).toLowerCase() === "true";
}

function parseActions(args) {
  if (args.actions) {
    return JSON.parse(String(args.actions));
  }
  if (args["actions-file"]) {
    const filePath = path.isAbsolute(args["actions-file"])
      ? args["actions-file"]
      : path.resolve(repoRoot, args["actions-file"]);
    return JSON.parse(fs.readFileSync(filePath, "utf8"));
  }
  throw new Error("Missing --actions JSON or --actions-file path.");
}

function normalizeActions(rawActions, maxActions) {
  if (!Array.isArray(rawActions)) throw new Error("Actions payload must be an array.");
  if (!rawActions.length) throw new Error("At least one action is required.");
  if (rawActions.length > maxActions) {
    throw new Error(`Action limit exceeded (${rawActions.length}/${maxActions}).`);
  }
  for (const [idx, action] of rawActions.entries()) {
    if (!action || typeof action !== "object") {
      throw new Error(`Invalid action at index ${idx}.`);
    }
    const type = String(action.type || "");
    if (!ALLOWED_ACTIONS.has(type)) {
      throw new Error(`Action '${type}' is not allowed.`);
    }
  }
  return rawActions;
}

async function executeAction(page, action, index, defaultTimeoutMs) {
  const type = String(action.type);
  if (type === "navigate") {
    await page.goto(String(action.url || ""), {
      waitUntil: "domcontentloaded",
      timeout: Number(action.timeout || defaultTimeoutMs)
    });
    return;
  }
  if (type === "click") {
    await page.click(String(action.selector), { timeout: Number(action.timeout || defaultTimeoutMs) });
    return;
  }
  if (type === "type") {
    await page.fill(String(action.selector), String(action.text || ""), {
      timeout: Number(action.timeout || defaultTimeoutMs)
    });
    return;
  }
  if (type === "press") {
    await page.keyboard.press(String(action.key || "Enter"));
    return;
  }
  if (type === "wait") {
    await page.waitForTimeout(Number(action.ms || 500));
    return;
  }
  if (type === "waitForSelector") {
    await page.waitForSelector(String(action.selector), {
      timeout: Number(action.timeout || defaultTimeoutMs)
    });
    return;
  }
  if (type === "screenshot") {
    const outDir = path.join(repoRoot, "snapshots", "atlas_actuators");
    fs.mkdirSync(outDir, { recursive: true });
    const shotPath = action.path
      ? (path.isAbsolute(action.path) ? action.path : path.resolve(repoRoot, action.path))
      : path.join(outDir, `action_step_${index + 1}_${Date.now()}.png`);
    await page.screenshot({ path: shotPath, fullPage: true });
  }
}

async function runPlaywright(target, actions, timeoutMs, dryRun) {
  const browser = await chromium.launch({ headless: true });
  try {
    const page = await browser.newPage();
    await page.goto(target, { waitUntil: "domcontentloaded", timeout: timeoutMs });
    if (dryRun) return;
    for (const [i, action] of actions.entries()) {
      audit("ACTUATOR_ACTION_STEP", { engine: "playwright", index: i + 1, type: action.type });
      await executeAction(page, action, i, timeoutMs);
    }
  } finally {
    await browser.close();
  }
}

async function runPuppeteer(target, actions, timeoutMs, dryRun) {
  const browser = await puppeteer.launch({ headless: "new" });
  try {
    const page = await browser.newPage();
    await page.goto(target, { waitUntil: "domcontentloaded", timeout: timeoutMs });
    if (dryRun) return;
    for (const [i, action] of actions.entries()) {
      audit("ACTUATOR_ACTION_STEP", { engine: "puppeteer", index: i + 1, type: action.type });
      await executeAction(page, action, i, timeoutMs);
    }
  } finally {
    await browser.close();
  }
}

function persistLastRun(payload) {
  const stateDir = path.join(repoRoot, "state", "atlas_actuators");
  fs.mkdirSync(stateDir, { recursive: true });
  const outPath = path.join(stateDir, "last_action_run.json");
  fs.writeFileSync(outPath, JSON.stringify(payload, null, 2), "utf8");
}

async function main() {
  const args = parseArgs(process.argv.slice(2));
  const coreToken = args["core-token"] ? String(args["core-token"]) : "";
  const target = String(args.target || "");
  const engine = String(args.engine || "playwright").toLowerCase();
  const timeoutMs = Number(args.timeout || 15000);
  const maxActions = Number(args["max-actions"] || 20);
  const allowGoverned = toBoolean(args["allow-governed"], false);
  if (!target) throw new Error("Missing --target URL.");
  if (engine !== "playwright" && engine !== "puppeteer") {
    throw new Error("Engine must be 'playwright' or 'puppeteer'.");
  }

  try {
    const auth = authorize(coreToken, { allowGoverned });
    const dryRun =
      args["dry-run"] !== undefined
        ? toBoolean(args["dry-run"], false)
        : !auth.governance.full_autonomy;
    if (!auth.governance.full_autonomy && !dryRun) {
      throw new Error(
        `Full autonomy disabled in governance mode=${auth.governance.mode}; use --dry-run true or switch to growth.`
      );
    }
    const actions = normalizeActions(parseActions(args), maxActions);
    const actionHash = crypto.createHash("sha1").update(JSON.stringify(actions)).digest("hex");
    const actionKey = createActionKey({ target, engine, actionHash });
    enforceLoopGuard(actionKey, {
      cooldownSeconds: Number(args["cooldown-seconds"] || 8),
      maxEvents: Number(args["guard-max-events"] || 40),
      windowSeconds: Number(args["guard-window-seconds"] || 300)
    });

    audit("ACTUATOR_ACTION_START", {
      engine,
      target,
      actions: actions.length,
      dry_run: dryRun ? 1 : 0,
      governance_mode: auth.governance.mode,
      full_autonomy: auth.governance.full_autonomy ? 1 : 0
    });

    if (engine === "playwright") {
      await runPlaywright(target, actions, timeoutMs, dryRun);
    } else {
      await runPuppeteer(target, actions, timeoutMs, dryRun);
    }

    const result = {
      ok: true,
      engine,
      target,
      actions: actions.length,
      dry_run: dryRun,
      ts: new Date().toISOString()
    };
    persistLastRun(result);
    audit("ACTUATOR_ACTION_OK", {
      engine,
      target,
      actions: actions.length,
      dry_run: dryRun ? 1 : 0,
      governance_mode: auth.governance.mode,
      full_autonomy: auth.governance.full_autonomy ? 1 : 0
    });
    process.stdout.write(
      `${JSON.stringify({
        ...result,
        governance_mode: auth.governance.mode,
        full_autonomy: auth.governance.full_autonomy
      })}\n`
    );
  } catch (error) {
    const message = error.message || String(error);
    persistLastRun({
      ok: false,
      error: message,
      target,
      engine,
      ts: new Date().toISOString()
    });
    audit("ACTUATOR_ACTION_ERR", { engine, target, error: message });
    process.stdout.write(`${JSON.stringify({ ok: false, error: message })}\n`);
    process.exitCode = 1;
  }
}

main();
