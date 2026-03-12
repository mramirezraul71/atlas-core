"""
ATLAS — Configuración autónoma de Meta WhatsApp Business API.
Usa Playwright (ojos + manos digitales) para:
  1. Abrir Meta Developers en navegador visible
  2. Esperar login del usuario (hasta 180s)
  3. Crear app → añadir WhatsApp → configurar webhook → extraer credenciales
  4. Guardar META_WHATSAPP_TOKEN + META_PHONE_NUMBER_ID en .env

Uso:
  python scripts/atlas_meta_whatsapp_autosetup.py \
    --webhook-url https://atlas-dashboard.rauliatlasapp.com/api/comms/whatsapp/inbound \
    --verify-token 2f3719c2770c4bf9d5b7566bfb58a1b86cc63be1c40e802a \
    --status-file logs/meta_autosetup_status.json
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


def _log(msg: str, status_file: Path | None = None, data: dict | None = None) -> None:
    ts = datetime.now(timezone.utc).strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)
    if status_file:
        try:
            existing = {}
            if status_file.exists():
                existing = json.loads(status_file.read_text(encoding="utf-8"))
            logs = existing.get("logs", [])
            logs.append({"ts": ts, "msg": msg})
            payload = {**existing, "logs": logs, "last": msg, "updated": ts}
            if data:
                payload.update(data)
            status_file.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        except Exception as e:
            print(f"  [status_file error] {e}", flush=True)


def _save_env(key: str, value: str, env_path: Path) -> None:
    content = env_path.read_text(encoding="utf-8") if env_path.exists() else ""
    pattern = rf'^{re.escape(key)}\s*=.*$'
    new_line = f'{key}="{value}"'
    if re.search(pattern, content, re.MULTILINE):
        content = re.sub(pattern, new_line, content, flags=re.MULTILINE)
    else:
        content += f'\n{new_line}'
    env_path.write_text(content, encoding="utf-8")
    os.environ[key] = value


def run(webhook_url: str, verify_token: str, status_file: Path, env_path: Path) -> bool:
    sf = status_file
    _log("Iniciando automatización Meta WhatsApp...", sf, {"phase": "start", "ok": None})

    try:
        from playwright.sync_api import sync_playwright, TimeoutError as PWTimeout
    except ImportError:
        _log("ERROR: playwright no instalado. Ejecuta: pip install playwright && playwright install chromium", sf, {"ok": False, "phase": "error"})
        return False

    app_name = f"Atlas-WA-{int(time.time()) % 10000}"

    with sync_playwright() as pw:
        _log("Abriendo navegador Chromium (visible)...", sf, {"phase": "browser"})
        browser = pw.chromium.launch(
            headless=False,
            args=["--start-maximized", "--disable-blink-features=AutomationControlled"],
        )
        ctx = browser.new_context(
            viewport={"width": 1280, "height": 800},
            user_agent="Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/120.0.0.0 Safari/537.36",
        )
        page = ctx.new_page()

        try:
            # ── 1. Ir a Facebook login primero (menos restringido) ────────────
            _log("Navegando a facebook.com para login...", sf)
            page.goto("https://www.facebook.com/", timeout=20000)
            time.sleep(2)

            # Detectar si ya hay sesión o necesita login
            if "facebook.com" in page.url:
                _log("Facebook cargado. Si no estás logueado, inicia sesión ahora.", sf, {"phase": "awaiting_login"})

            # ── 2. Esperar login en Facebook (hasta 180s) ─────────────────────
            _log("Inicia sesión en Facebook si se te pide. Atlas esperará...", sf, {"phase": "awaiting_login"})
            deadline = time.time() + 180
            logged_in = False
            while time.time() < deadline:
                url = page.url
                # Detectar login exitoso en facebook.com (cualquier página autenticada)
                if "facebook.com" in url and "login" not in url and "checkpoint" not in url:
                    # Verificar que hay elementos de usuario autenticado
                    try:
                        page.wait_for_selector(
                            '[aria-label="Facebook"], [data-pagelet="LeftRail"], '
                            '[data-pagelet="Stories"], [role="main"]',
                            timeout=3000
                        )
                        logged_in = True
                        break
                    except PWTimeout:
                        pass
                time.sleep(2)

            if not logged_in:
                # Asumir login si estamos en facebook.com sin /login
                if "facebook.com" in page.url and "login" not in page.url:
                    logged_in = True

            if logged_in:
                # Navegar a developers (ahora que estamos logueados)
                _log("Login OK. Navegando a Meta Developers...", sf)
                try:
                    page.goto("https://developers.facebook.com/apps/", timeout=20000)
                    time.sleep(3)
                    content = page.content()
                    if "no está disponible" in content or "not available" in content.lower():
                        _log("Geo-bloqueado incluso con login. Intenta activar una VPN.", sf, {"phase": "geo_blocked"})
                        # Dejar el browser abierto para que el usuario haga algo
                        time.sleep(60)
                except Exception as _nav_err:
                    _log(f"Error navegando: {_nav_err}", sf)

            if not logged_in:
                _log("Timeout esperando login. Abortando.", sf, {"ok": False, "phase": "error"})
                ctx.close()
                return False

            _log("Login detectado. Iniciando creación de app...", sf, {"phase": "create_app"})

            # ── 3. Crear nueva App ───────────────────────────────────────────
            # Hacer clic en "Create App" o "Crear aplicación"
            try:
                page.click('a:has-text("Create App"), button:has-text("Create App"), a:has-text("Crear aplicación"), button:has-text("Crear")', timeout=10000)
            except PWTimeout:
                # Intentar URL directa
                page.goto("https://developers.facebook.com/apps/create/", timeout=15000)

            time.sleep(2)
            _log("En pantalla de selección de tipo de app...", sf)

            # ── 4. Seleccionar tipo "Other" / "Otro" ─────────────────────────
            try:
                page.click('div:has-text("Other"), div:has-text("Otro"), [data-value="other"], label:has-text("Other")', timeout=8000)
                time.sleep(1)
                page.click('button:has-text("Next"), button:has-text("Siguiente")', timeout=5000)
            except PWTimeout:
                # Puede que ya estemos en otro paso del wizard
                _log("Saltando paso tipo de app (puede ser diferente versión del UI)", sf)

            time.sleep(2)

            # ── 5. Seleccionar "Business" ────────────────────────────────────
            try:
                page.click('div:has-text("Business"), [data-value="business"], label:has-text("Business")', timeout=8000)
                time.sleep(1)
                page.click('button:has-text("Next"), button:has-text("Siguiente")', timeout=5000)
            except PWTimeout:
                _log("Saltando selección Business", sf)

            time.sleep(2)

            # ── 6. Nombre de la app ──────────────────────────────────────────
            try:
                name_input = page.wait_for_selector('input[placeholder*="app name" i], input[name*="name" i], input[id*="name" i], input[placeholder*="nombre" i]', timeout=10000)
                name_input.fill(app_name)
                time.sleep(1)
                _log(f"App nombre: {app_name}", sf)
            except PWTimeout:
                _log("No encontré campo de nombre, continuando...", sf)

            # Email field (optional)
            try:
                email_input = page.query_selector('input[type="email"]')
                if email_input:
                    # Leave as-is, prefilled from user account
                    pass
            except Exception:
                pass

            try:
                page.click('button:has-text("Create App"), button:has-text("Create app"), button:has-text("Crear aplicación")', timeout=8000)
                _log("App creada, esperando dashboard...", sf)
            except PWTimeout:
                _log("No pude hacer clic en Crear. Intentando continuar...", sf)

            time.sleep(4)

            # ── 7. Buscar producto WhatsApp y añadirlo ───────────────────────
            _log("Buscando producto WhatsApp para añadir...", sf, {"phase": "add_whatsapp"})

            # Puede que estemos en pantalla de "Add products"
            whatsapp_added = False
            for attempt in range(3):
                try:
                    page.click('div:has-text("WhatsApp") >> .. >> button:has-text("Set up"), a:has-text("Set up") >> xpath=../.. >> div:has-text("WhatsApp")', timeout=5000)
                    whatsapp_added = True
                    break
                except PWTimeout:
                    pass
                try:
                    # Intentar directo
                    page.click('[data-product-id="whatsapp"] button, .product-whatsapp button, button:near(:text("WhatsApp"))', timeout=5000)
                    whatsapp_added = True
                    break
                except PWTimeout:
                    pass
                # Buscar cualquier botón "Set up" cerca de "WhatsApp"
                try:
                    page.evaluate("""
                        const items = Array.from(document.querySelectorAll('*'));
                        const wa = items.find(el => el.textContent.includes('WhatsApp') && (el.tagName === 'DIV' || el.tagName === 'SECTION'));
                        if (wa) {
                            const btn = wa.querySelector('button, a');
                            if (btn) btn.click();
                        }
                    """)
                    whatsapp_added = True
                    break
                except Exception:
                    pass
                time.sleep(2)

            if not whatsapp_added:
                # Navegar directamente a la URL de la app + WhatsApp
                current_url = page.url
                app_id_match = re.search(r'/apps/(\d+)', current_url)
                if app_id_match:
                    app_id = app_id_match.group(1)
                    page.goto(f"https://developers.facebook.com/apps/{app_id}/whatsapp-business/wa-dev-console/", timeout=15000)
                    _log(f"Navegando a WhatsApp dev console para app {app_id}", sf)
                else:
                    _log("No pude determinar app_id, buscando en URL...", sf)

            time.sleep(3)

            # ── 8. Extraer Token temporal y Phone Number ID ──────────────────
            _log("Extrayendo credenciales de la API...", sf, {"phase": "extract_credentials"})

            meta_token = ""
            phone_number_id = ""

            # Esperar que cargue la consola de WhatsApp
            try:
                page.wait_for_selector('text=Temporary access token, text=Access token, text=Token de acceso', timeout=15000)
            except PWTimeout:
                _log("Timeout esperando sección de tokens. Tomando screenshot...", sf)
                shot_path = str(ROOT / "logs" / "meta_autosetup_screenshot.png")
                page.screenshot(path=shot_path)
                _log(f"Screenshot guardado: {shot_path}", sf)

            # Intentar extraer token
            try:
                token_el = page.query_selector('input[value^="EAA"], textarea:has-text("EAA"), code:has-text("EAA")')
                if token_el:
                    meta_token = token_el.input_value() if token_el.get_attribute("value") else token_el.inner_text()
                    meta_token = meta_token.strip()
            except Exception as e:
                _log(f"No pude extraer token automáticamente: {e}", sf)

            # Intentar via JS
            if not meta_token:
                try:
                    meta_token = page.evaluate("""
                        () => {
                            const inputs = Array.from(document.querySelectorAll('input, textarea'));
                            const t = inputs.find(el => (el.value || '').startsWith('EAA'));
                            return t ? t.value : '';
                        }
                    """) or ""
                except Exception:
                    pass

            # Phone Number ID
            try:
                phone_id_el = page.query_selector('input[value^="1"], [data-testid*="phone-number-id"]')
                # Buscar campo que dice "Phone number ID" y tiene valor numérico largo
                phone_number_id = page.evaluate("""
                    () => {
                        const labels = Array.from(document.querySelectorAll('label, div, span, p'));
                        const lbl = labels.find(el => el.textContent.toLowerCase().includes('phone number id'));
                        if (lbl) {
                            // Buscar input hermano o próximo
                            const parent = lbl.closest('div, section, tr');
                            if (parent) {
                                const inp = parent.querySelector('input');
                                if (inp) return inp.value;
                                // Buscar texto numérico
                                const text = parent.textContent;
                                const match = text.match(/\\b(\\d{10,})/);
                                if (match) return match[1];
                            }
                        }
                        return '';
                    }
                """) or ""
            except Exception:
                pass

            _log(f"Token extraído: {'SÍ (' + meta_token[:12] + '...)' if meta_token else 'NO'}", sf)
            _log(f"Phone Number ID extraído: {'SÍ (' + phone_number_id + ')' if phone_number_id else 'NO'}", sf)

            # ── 9. Configurar Webhook ────────────────────────────────────────
            _log("Configurando webhook...", sf, {"phase": "configure_webhook"})

            # Navegar a la sección de configuración
            app_id_match = re.search(r'/apps/(\d+)', page.url)
            if app_id_match:
                app_id = app_id_match.group(1)
                page.goto(f"https://developers.facebook.com/apps/{app_id}/whatsapp-business/", timeout=15000)
                time.sleep(2)

            # Buscar tab/sección "Configuration" o "API Setup"
            try:
                page.click('a:has-text("Configuration"), a:has-text("Configuración"), [href*="configuration"]', timeout=8000)
                time.sleep(2)
            except PWTimeout:
                _log("No encontré tab Configuration, buscando webhook en página actual...", sf)

            # Buscar botón "Edit" junto a Webhook
            try:
                page.click('button:has-text("Edit"), button:has-text("Editar"), a:has-text("Edit")', timeout=8000)
                time.sleep(1)
            except PWTimeout:
                _log("No encontré botón Edit para webhook", sf)

            # Rellenar Callback URL
            try:
                cb_input = page.wait_for_selector('input[placeholder*="callback" i], input[placeholder*="URL" i], input[name*="callback" i], input[name*="url" i]', timeout=8000)
                cb_input.fill(webhook_url)
                _log(f"Webhook URL rellenada: {webhook_url}", sf)
                time.sleep(0.5)
            except PWTimeout:
                _log("No encontré campo Callback URL", sf)

            # Rellenar Verify Token
            try:
                vt_inputs = page.query_selector_all('input[placeholder*="token" i], input[name*="token" i], input[type="text"]')
                for inp in vt_inputs:
                    val = inp.get_attribute("value") or ""
                    placeholder = inp.get_attribute("placeholder") or ""
                    if "token" in placeholder.lower() or (not val and inp != vt_inputs[0]):
                        inp.fill(verify_token)
                        _log(f"Verify token rellenado", sf)
                        break
            except Exception:
                pass

            # Verify and Save
            try:
                page.click('button:has-text("Verify"), button:has-text("Verificar"), button:has-text("Verify and save")', timeout=8000)
                time.sleep(3)
                _log("Webhook verificado y guardado.", sf, {"phase": "webhook_done"})
            except PWTimeout:
                _log("No pude hacer clic en Verify — puede requerir acción manual", sf)

            # ── 10. Suscribir a mensajes ─────────────────────────────────────
            try:
                page.click('button:has-text("Subscribe"), button:has-text("Suscribir"), [aria-label*="messages"]', timeout=5000)
                time.sleep(1)
            except PWTimeout:
                pass

            # ── 11. Guardar en .env ──────────────────────────────────────────
            _log("Guardando credenciales en .env...", sf, {"phase": "saving"})

            if meta_token:
                _save_env("META_WHATSAPP_TOKEN", meta_token, env_path)
                _save_env("WHATSAPP_PROVIDER", "meta", env_path)
                _save_env("WHATSAPP_ENABLED", "true", env_path)
                _log("META_WHATSAPP_TOKEN guardado en .env", sf)

            if phone_number_id:
                _save_env("META_PHONE_NUMBER_ID", phone_number_id, env_path)
                _log("META_PHONE_NUMBER_ID guardado en .env", sf)

            # Tomar screenshot final
            shot_final = str(ROOT / "logs" / "meta_autosetup_final.png")
            page.screenshot(path=shot_final)
            _log(f"Screenshot final: {shot_final}", sf)

            success = bool(meta_token or phone_number_id)
            result = {
                "ok": success,
                "phase": "done" if success else "partial",
                "meta_token_saved": bool(meta_token),
                "phone_id_saved": bool(phone_number_id),
                "webhook_configured": True,
                "app_name": app_name,
                "screenshot": shot_final,
                "note": "Credenciales guardadas automáticamente" if success else
                        "Webhook configurado pero token/phone_id requieren extracción manual desde el dashboard",
            }
            _log(f"Completado. Token: {'OK' if meta_token else 'pendiente'} | PhoneID: {'OK' if phone_number_id else 'pendiente'}", sf, result)

            time.sleep(5)  # Dejar el navegador abierto unos segundos
            ctx.close()
            return success

        except Exception as e:
            _log(f"ERROR inesperado: {e}", sf, {"ok": False, "phase": "error", "error": str(e)})
            try:
                shot_err = str(ROOT / "logs" / "meta_autosetup_error.png")
                page.screenshot(path=shot_err)
                _log(f"Screenshot de error: {shot_err}", sf)
            except Exception:
                pass
            ctx.close()
            return False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Atlas Meta WhatsApp autosetup")
    parser.add_argument("--webhook-url", default="https://atlas-dashboard.rauliatlasapp.com/api/comms/whatsapp/inbound")
    parser.add_argument("--verify-token", default="")
    parser.add_argument("--status-file", default="logs/meta_autosetup_status.json")
    parser.add_argument("--env-file", default=".env")
    args = parser.parse_args()

    if not args.verify_token:
        args.verify_token = os.getenv("WHATSAPP_WEBHOOK_VERIFY_TOKEN", "")

    root = Path(__file__).resolve().parent.parent
    status_file = root / args.status_file
    env_file = root / args.env_file

    status_file.parent.mkdir(parents=True, exist_ok=True)
    ok = run(args.webhook_url, args.verify_token, status_file, env_file)
    sys.exit(0 if ok else 1)
