const { chromium } = require('playwright');

const URL = process.env.ATLAS_QUANT_UI_URL || 'http://127.0.0.1:8791/ui#/atlas-quant';

async function click(page, selector, name, { optional = false, timeout = 8000 } = {}) {
  const locator = page.locator(selector).first();
  const exists = (await locator.count()) > 0;
  if (!exists) {
    if (optional) return { name, ok: true, skipped: 'no_encontrado' };
    throw new Error(`No se encontro ${name}: ${selector}`);
  }
  await locator.evaluate((el) => el.scrollIntoView({ block: 'center' }));
  await locator.click({ timeout });
  await page.waitForTimeout(900);
  return { name, ok: true };
}

async function main() {
  const browser = await chromium.launch({ headless: true });
  const page = await browser.newPage({ viewport: { width: 1600, height: 1400 } });
  const consoleErrors = [];
  page.on('console', (msg) => {
    if (msg.type() === 'error') consoleErrors.push(msg.text());
  });

  try {
    await page.goto(URL, { waitUntil: 'domcontentloaded', timeout: 30000 });
    await page.waitForSelector('#aq-btn-scanner-save', { timeout: 15000 });

    const results = [];
    results.push(await click(page, '#aq-btn-scanner-save', 'guardar_scanner'));
    results.push(await click(page, '#aq-btn-scanner-run', 'ciclo_ahora'));
    results.push(await click(page, '[data-quant-view-btn="selector"]', 'vista_selector'));
    results.push(await click(page, '#aq-btn-selector-best', 'selector_desde_mejor'));
    results.push(await click(page, '[data-quant-view-btn="operacion"]', 'vista_operacion'));
    results.push(await click(page, '#aq-btn-op-refresh', 'actualizar_operacion'));
    results.push(await click(page, '#aq-btn-op-save', 'guardar_operacion'));
    results.push(await click(page, '#aq-btn-op-evaluate', 'probar_ciclo'));
    results.push(await click(page, '#aq-btn-calib-start', 'calibracion_inicio'));
    results.push(await click(page, '[data-quant-view-btn="diario"]', 'vista_diario'));
    results.push(await click(page, '#aq-btn-journal-sync', 'sync_diario'));
    results.push(await click(page, '[data-quant-view-btn="ejecucion"]', 'vista_ejecucion'));

    const snapshot = {
      ws: await page.locator('text=/WS en vivo|REST de respaldo/').first().innerText().catch(() => ''),
      opResult: await page.locator('#aq-op-result').innerText().catch(() => ''),
      calibState: await page.locator('#aq-calib-state').innerText().catch(() => ''),
    };

    console.log(JSON.stringify({ ok: true, url: page.url(), results, snapshot, consoleErrors }, null, 2));
    await browser.close();
  } catch (error) {
    console.error(JSON.stringify({ ok: false, error: String(error), consoleErrors }, null, 2));
    await browser.close();
    process.exit(1);
  }
}

main();
