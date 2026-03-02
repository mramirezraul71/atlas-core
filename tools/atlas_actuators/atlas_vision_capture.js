const fs = require("fs");
const path = require("path");
const { spawnSync } = require("child_process");
const { chromium } = require("playwright");
const puppeteer = require("puppeteer");
const { audit, repoRoot } = require("./atlas_logger");
const { authorize } = require("./atlas_guard");

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

function stamp() {
  return new Date().toISOString().replace(/[:.]/g, "-");
}

function ensureOutputPath(outputArg) {
  const dir = path.join(repoRoot, "snapshots", "atlas_actuators");
  fs.mkdirSync(dir, { recursive: true });
  if (outputArg) {
    return path.isAbsolute(outputArg) ? outputArg : path.resolve(repoRoot, outputArg);
  }
  return path.join(dir, `vision_capture_${stamp()}.png`);
}

async function captureWithPlaywright(target, outputPath, timeoutMs, headless) {
  const browser = await chromium.launch({ headless });
  try {
    const page = await browser.newPage();
    if (target) {
      await page.goto(target, { waitUntil: "domcontentloaded", timeout: timeoutMs });
    }
    await page.screenshot({ path: outputPath, fullPage: true });
  } finally {
    await browser.close();
  }
}

async function captureWithPuppeteer(target, outputPath, timeoutMs, headless, cdp) {
  let browser = null;
  let connectedExternal = false;
  try {
    if (cdp) {
      browser = await puppeteer.connect({ browserWSEndpoint: cdp });
      connectedExternal = true;
    } else {
      browser = await puppeteer.launch({ headless: headless ? "new" : false });
    }
    const page = await browser.newPage();
    if (target) {
      await page.goto(target, { waitUntil: "domcontentloaded", timeout: timeoutMs });
    }
    await page.screenshot({ path: outputPath, fullPage: true });
  } finally {
    if (browser) {
      if (connectedExternal) {
        await browser.disconnect();
      } else {
        await browser.close();
      }
    }
  }
}

function captureDesktop(outputPath) {
  const psScript = [
    "Add-Type -AssemblyName System.Windows.Forms",
    "Add-Type -AssemblyName System.Drawing",
    "$bounds = [System.Windows.Forms.Screen]::PrimaryScreen.Bounds",
    "$bitmap = New-Object System.Drawing.Bitmap $bounds.Width, $bounds.Height",
    "$graphics = [System.Drawing.Graphics]::FromImage($bitmap)",
    "$graphics.CopyFromScreen($bounds.Location, [System.Drawing.Point]::Empty, $bounds.Size)",
    `$bitmap.Save('${outputPath.replace(/'/g, "''")}', [System.Drawing.Imaging.ImageFormat]::Png)`,
    "$graphics.Dispose()",
    "$bitmap.Dispose()"
  ].join("; ");
  const proc = spawnSync(
    "powershell",
    ["-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", psScript],
    { encoding: "utf8" }
  );
  if (proc.status !== 0) {
    throw new Error(proc.stderr || "Desktop capture failed.");
  }
}

async function main() {
  const args = parseArgs(process.argv.slice(2));
  const mode = String(args.mode || "playwright").toLowerCase();
  const target = args.target ? String(args.target) : "";
  const outputPath = ensureOutputPath(args.output ? String(args.output) : "");
  const timeoutMs = Number(args.timeout || 15000);
  const headless = String(args.headless || "true").toLowerCase() !== "false";
  const coreToken = args["core-token"] ? String(args["core-token"]) : "";
  const cdp = args.cdp ? String(args.cdp) : "";

  try {
    authorize(coreToken);
    audit("ACTUATOR_VISION_START", { mode, target: target || "desktop", output: outputPath });

    if (mode === "playwright") {
      await captureWithPlaywright(target, outputPath, timeoutMs, headless);
    } else if (mode === "puppeteer") {
      await captureWithPuppeteer(target, outputPath, timeoutMs, headless, cdp);
    } else if (mode === "desktop") {
      captureDesktop(outputPath);
    } else {
      throw new Error(`Unsupported mode: ${mode}`);
    }

    audit("ACTUATOR_VISION_OK", { mode, output: outputPath });
    process.stdout.write(
      `${JSON.stringify({ ok: true, mode, output: outputPath, ts: new Date().toISOString() })}\n`
    );
  } catch (error) {
    audit("ACTUATOR_VISION_ERR", { mode, error: error.message || String(error) });
    process.stdout.write(
      `${JSON.stringify({ ok: false, mode, error: error.message || String(error) })}\n`
    );
    process.exitCode = 1;
  }
}

main();
