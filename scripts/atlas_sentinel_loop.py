"""
ATLAS Sentinel Loop

Bucle de autochequeo continuo para ATLAS_PUSH:
- atlas_visual_audit(): inspecciona salud, pantalla y cámaras (incluye Insta360 si está registrada).
- atlas_self_heal(): aplica acciones correctivas automáticas seguras.
- atlas_heartbeat(): publica estado operativo en log y opcionalmente por voz.

Uso rápido:
  .venv\\Scripts\\python.exe scripts\\atlas_sentinel_loop.py --once
  .venv\\Scripts\\python.exe scripts\\atlas_sentinel_loop.py --interval-sec 90 --heartbeat-sec 30
"""
from __future__ import annotations

import argparse
import atexit
import json
import os
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import requests


ROOT = Path(__file__).resolve().parents[1]
LOG_DIR = ROOT / "logs" / "sentinel"
LOG_DIR.mkdir(parents=True, exist_ok=True)
HTTP_SESSION = requests.Session()
HTTP_SESSION.trust_env = False
_LOCK_HANDLE = None
_MUTEX_HANDLE = None


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _safe_print(msg: str) -> None:
    try:
        print(msg, flush=True)
    except UnicodeEncodeError:
        enc = (getattr(sys.stdout, "encoding", None) or "utf-8")
        print(msg.encode("utf-8", errors="replace").decode(enc, errors="replace"), flush=True)


def _to_json_bytes(payload: Dict[str, Any]) -> bytes:
    return json.dumps(payload, ensure_ascii=False).encode("utf-8")


def _acquire_single_instance_lock(lock_path: Path) -> bool:
    """Evita múltiples instancias en paralelo del loop Sentinel."""
    global _LOCK_HANDLE, _MUTEX_HANDLE
    if os.name == "nt":
        try:
            import ctypes

            kernel32 = ctypes.windll.kernel32  # type: ignore[attr-defined]
            mutex_name = "Global\\ATLAS_PUSH_SENTINEL_MUTEX"
            handle = kernel32.CreateMutexW(None, False, mutex_name)
            if not handle:
                return False
            last_error = kernel32.GetLastError()
            # ERROR_ALREADY_EXISTS = 183
            if int(last_error) == 183:
                kernel32.CloseHandle(handle)
                return False
            _MUTEX_HANDLE = handle

            def _release_mutex() -> None:
                global _MUTEX_HANDLE
                if _MUTEX_HANDLE:
                    try:
                        kernel32.CloseHandle(_MUTEX_HANDLE)
                    except Exception:
                        pass
                    _MUTEX_HANDLE = None

            atexit.register(_release_mutex)
            return True
        except Exception:
            pass

    lock_path.parent.mkdir(parents=True, exist_ok=True)
    fh = open(lock_path, "a+b")
    try:
        if os.name == "nt":
            import msvcrt

            fh.seek(0)
            msvcrt.locking(fh.fileno(), msvcrt.LK_NBLCK, 1)
        else:
            import fcntl

            fcntl.flock(fh.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except Exception:
        try:
            fh.close()
        except Exception:
            pass
        return False

    _LOCK_HANDLE = fh

    def _release() -> None:
        global _LOCK_HANDLE
        if _LOCK_HANDLE is None:
            return
        try:
            if os.name == "nt":
                import msvcrt

                _LOCK_HANDLE.seek(0)
                msvcrt.locking(_LOCK_HANDLE.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                import fcntl

                fcntl.flock(_LOCK_HANDLE.fileno(), fcntl.LOCK_UN)
        except Exception:
            pass
        try:
            _LOCK_HANDLE.close()
        except Exception:
            pass
        _LOCK_HANDLE = None

    atexit.register(_release)
    return True


def _http_json(
    method: str,
    url: str,
    *,
    payload: Optional[Dict[str, Any]] = None,
    timeout: float = 10.0,
) -> Tuple[bool, int, Dict[str, Any], str]:
    try:
        resp = HTTP_SESSION.request(
            method=method.upper(),
            url=url,
            json=payload,
            timeout=(3.5, float(timeout)),
            headers={"Accept": "application/json"},
        )
        raw = resp.text or ""
        try:
            body = resp.json() if raw else {}
        except Exception:
            body = {}
        if 200 <= int(resp.status_code) < 300:
            return True, int(resp.status_code), body, ""
        return False, int(resp.status_code), body, f"http_{resp.status_code}"
    except Exception as e:
        return False, 0, {}, f"{type(e).__name__}: {e}"


def _http_bytes(url: str, timeout: float = 10.0) -> Tuple[bool, int, bytes, str]:
    try:
        resp = HTTP_SESSION.get(url, timeout=(3.5, float(timeout)))
        if 200 <= int(resp.status_code) < 300:
            return True, int(resp.status_code), resp.content or b"", ""
        return False, int(resp.status_code), b"", f"http_{resp.status_code}"
    except Exception as e:
        return False, 0, b"", f"{type(e).__name__}: {e}"


@dataclass
class SentinelIssue:
    issue_id: str
    severity: str  # info|warning|critical
    message: str
    source: str
    details: Dict[str, Any] = field(default_factory=dict)


@dataclass
class SentinelAudit:
    ts: str
    ok: bool
    issues: List[SentinelIssue] = field(default_factory=list)
    stats: Dict[str, Any] = field(default_factory=dict)
    evidence: Dict[str, Any] = field(default_factory=dict)


class AtlasSentinel:
    def __init__(
        self,
        *,
        atlas_base: str,
        robot_base: str,
        interval_sec: int,
        heartbeat_sec: int,
        speak_alerts: bool,
        auto_heal: bool,
        camera_prefer_keyword: str,
        camera_prefer_id: str,
        camera_prefer_index: int,
        dry_run: bool,
    ) -> None:
        self.atlas_base = atlas_base.rstrip("/")
        self.robot_base = robot_base.rstrip("/")
        self.interval_sec = max(15, int(interval_sec))
        self.heartbeat_sec = max(10, int(heartbeat_sec))
        self.speak_alerts = bool(speak_alerts)
        self.auto_heal = bool(auto_heal)
        self.camera_prefer_keyword = (camera_prefer_keyword or "insta").strip().lower()
        self.camera_prefer_id = (camera_prefer_id or "").strip()
        self.camera_prefer_index = max(0, int(camera_prefer_index))
        self.dry_run = bool(dry_run)

        self.stop_event = threading.Event()
        self._last_hb_ts = 0.0
        self._last_audit: Optional[SentinelAudit] = None
        self._heal_cooldowns: Dict[str, float] = {}
        self._push_fail_streak = 0
        self._camera_fail_streak = 0
        self._detected_cam_index_cache: Optional[int] = None
        self._detected_cam_model_cache: str = ""
        self._detected_cam_cache_ts: float = 0.0
        self._jsonl_path = LOG_DIR / f"sentinel_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"

    # ----------------- Logging / comms -----------------
    def _append_log(self, kind: str, payload: Dict[str, Any]) -> None:
        row = {
            "ts": _now_iso(),
            "kind": kind,
            "payload": payload,
        }
        with self._jsonl_path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")

    def _speak(self, text: str) -> None:
        if not self.speak_alerts:
            return
        if self.dry_run:
            self._append_log("speak_dry_run", {"text": text})
            return
        _http_json(
            "POST",
            f"{self.atlas_base}/api/comms/speak",
            payload={"text": text},
            timeout=8.0,
        )

    def _emit_ops(self, message: str, level: str = "info", source: str = "sentinel") -> None:
        if self.dry_run:
            self._append_log(
                "ops_dry_run",
                {"message": message, "level": level, "source": source},
            )
            return
        _http_json(
            "POST",
            f"{self.atlas_base}/ans/evolution-log",
            payload={"message": message, "ok": level != "error", "source": source},
            timeout=8.0,
        )

    # ----------------- Physical actions -----------------
    def _safe_mouse_probe(self) -> Dict[str, Any]:
        """Movimiento mínimo no destructivo para validar manos."""
        try:
            import pyautogui

            pyautogui.FAILSAFE = True
            x0, y0 = pyautogui.position()
            # Patrón corto para no interferir con el usuario
            pyautogui.moveRel(8, 0, duration=0.06)
            pyautogui.moveRel(-8, 8, duration=0.06)
            pyautogui.moveRel(0, -8, duration=0.06)
            x1, y1 = pyautogui.position()
            return {
                "ok": True,
                "before": [int(x0), int(y0)],
                "after": [int(x1), int(y1)],
            }
        except Exception as e:
            return {"ok": False, "error": f"{type(e).__name__}: {e}"}

    # ----------------- Visual audit -----------------
    def _capture_screen(self) -> Dict[str, Any]:
        ok, status, body, err = _http_json(
            "POST",
            f"{self.atlas_base}/screen/capture",
            payload={"format": "png"},
            timeout=12.0,
        )
        if not ok or status < 200 or status >= 300 or not body.get("ok"):
            return {"ok": False, "error": err or body.get("error") or "screen_capture_failed"}
        data = body.get("data") if isinstance(body.get("data"), dict) else {}
        return {
            "ok": True,
            "path": data.get("path"),
        }

    def _analyze_screen_image_cv(self, image_path: str) -> Dict[str, Any]:
        """Heurística CV local para detectar pantalla negra o congelada."""
        try:
            import cv2
            import numpy as np

            img = cv2.imread(image_path)
            if img is None:
                return {"ok": False, "error": "cv2_read_failed"}
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            mean_val = float(gray.mean())
            lap_var = float(cv2.Laplacian(gray, cv2.CV_64F).var())

            flags: List[str] = []
            if mean_val < 12.0:
                flags.append("screen_too_dark")
            if lap_var < 8.0:
                flags.append("screen_low_detail")

            # Detector de barras negras laterales exageradas (overflow/viewport raro)
            h, w = gray.shape[:2]
            left_band = gray[:, : max(1, int(w * 0.04))]
            right_band = gray[:, max(0, int(w * 0.96)) :]
            edge_dark_ratio = float(
                ((left_band < 10).sum() + (right_band < 10).sum())
                / max(1, left_band.size + right_band.size)
            )
            if edge_dark_ratio > 0.92 and w > 500:
                flags.append("possible_layout_overflow_or_bars")

            return {
                "ok": True,
                "brightness_mean": round(mean_val, 2),
                "laplacian_var": round(lap_var, 2),
                "edge_dark_ratio": round(edge_dark_ratio, 4),
                "flags": flags,
            }
        except Exception as e:
            return {"ok": False, "error": f"{type(e).__name__}: {e}"}

    def _screen_status(self) -> Dict[str, Any]:
        ok, _, body, err = _http_json("GET", f"{self.atlas_base}/screen/status", timeout=8.0)
        if not ok or not body:
            return {"ok": False, "error": err or "screen_status_failed"}
        return {"ok": bool(body.get("ok")), "data": body.get("data", {}), "error": body.get("error")}

    def _camera_probe(self) -> Dict[str, Any]:
        """
        Revisa cámara preferida (Insta360 si aparece por keyword),
        luego fallback a snapshot proxy local.
        """
        out: Dict[str, Any] = {
            "ok": False,
            "source": "",
            "camera_id": "",
            "bytes": 0,
            "error": "",
        }

        local_index = self._resolve_preferred_local_camera_index()

        # 1) UBIQ cameras registry
        ok, _, cams_body, err = _http_json(
            "GET",
            f"{self.atlas_base}/api/vision/ubiq/cameras?limit=200",
            timeout=10.0,
        )
        cam_candidates: List[Dict[str, Any]] = []
        if ok and cams_body.get("ok"):
            data = cams_body.get("data") if isinstance(cams_body.get("data"), dict) else {}
            cams = data.get("cameras") if isinstance(data.get("cameras"), list) else []
            cam_candidates = [c for c in cams if isinstance(c, dict)]

        preferred: Optional[Dict[str, Any]] = None
        if self.camera_prefer_id:
            for c in cam_candidates:
                if str(c.get("id") or "").strip().lower() == self.camera_prefer_id.lower():
                    preferred = c
                    break
        for c in cam_candidates:
            if preferred is not None:
                break
            s = " ".join(
                [
                    str(c.get("id") or ""),
                    str(c.get("model") or ""),
                    str(c.get("source") or ""),
                    str(c.get("url") or ""),
                ]
            ).lower()
            if self.camera_prefer_keyword and self.camera_prefer_keyword in s:
                preferred = c
                break
        if preferred is None and cam_candidates:
            preferred = cam_candidates[0]

        if preferred:
            cam_id = str(preferred.get("id") or "").strip()
            ok2, _, snap_body, err2 = _http_json(
                "GET",
                f"{self.atlas_base}/api/vision/ubiq/snapshot/{cam_id}",
                timeout=12.0,
            )
            if ok2 and snap_body.get("ok"):
                data = snap_body.get("data") if isinstance(snap_body.get("data"), dict) else {}
                b64 = str(data.get("image_base64") or "")
                out.update(
                    {
                        "ok": bool(b64),
                        "source": "ubiq",
                        "camera_id": cam_id,
                        "bytes": int(len(b64) * 0.75) if b64 else 0,
                        "error": "" if b64 else "empty_snapshot",
                    }
                )
                if out["ok"]:
                    return out
            else:
                out["error"] = err2 or snap_body.get("error") or "ubiq_snapshot_failed"

        # 2) Fallback: proxy robot snapshot
        ok3, status3, content3, err3 = _http_bytes(
            f"{self.atlas_base}/cuerpo/vision/snapshot?index={int(local_index)}&enhance=auto&jpeg_quality=80",
            timeout=10.0,
        )
        if ok3 and status3 == 200 and len(content3) > 2048:
            out.update(
                {
                    "ok": True,
                    "source": "cuerpo_snapshot",
                    "camera_id": f"index:{int(local_index)}",
                    "bytes": len(content3),
                    "error": "",
                    "camera_model": self._detected_cam_model_cache or "",
                }
            )
            return out

        out.update(
            {
                "ok": False,
                "source": out.get("source") or "none",
                "camera_id": out.get("camera_id") or "",
                "bytes": int(out.get("bytes") or 0),
                "error": out.get("error") or err3 or err or "camera_probe_failed",
            }
        )
        return out

    def _resolve_preferred_local_camera_index(self) -> int:
        """
        Prioridad:
        1) --camera-index explícito
        2) cache de detección (válido por 10 min)
        3) /api/camera/detect buscando keyword (insta)
        4) índice 0
        """
        if self.camera_prefer_index > 0:
            return int(self.camera_prefer_index)

        now = time.time()
        if self._detected_cam_index_cache is not None and (now - self._detected_cam_cache_ts) < 600:
            return int(self._detected_cam_index_cache)

        ok, _, body, _ = _http_json("GET", f"{self.robot_base}/api/camera/detect", timeout=25.0)
        if ok and isinstance(body.get("cameras"), list):
            cams = [c for c in body.get("cameras", []) if isinstance(c, dict)]
            preferred: Optional[Dict[str, Any]] = None
            for c in cams:
                s = " ".join(
                    [
                        str(c.get("model") or ""),
                        str(c.get("driver") or ""),
                        str(c.get("vid_pid") or ""),
                    ]
                ).lower()
                if self.camera_prefer_keyword and self.camera_prefer_keyword in s:
                    preferred = c
                    break
            if preferred is None and cams:
                preferred = cams[0]
            if preferred is not None:
                try:
                    idx = int(preferred.get("index", 0) or 0)
                except Exception:
                    idx = 0
                self._detected_cam_index_cache = idx
                self._detected_cam_model_cache = str(preferred.get("model") or "")
                self._detected_cam_cache_ts = now
                return idx
        return 0

    def atlas_visual_audit(self) -> SentinelAudit:
        issues: List[SentinelIssue] = []
        evidence: Dict[str, Any] = {}
        stats: Dict[str, Any] = {}

        # A) Health base
        ok_h, _, health, err_h = _http_json("GET", f"{self.atlas_base}/health", timeout=8.0)
        if not ok_h or not health.get("ok"):
            self._push_fail_streak += 1
            sev = "critical" if self._push_fail_streak >= 2 else "warning"
            issues.append(
                SentinelIssue(
                    issue_id="push_health",
                    severity=sev,
                    message=f"PUSH health no OK: {err_h or health.get('error') or 'unknown'}",
                    source="health",
                    details={"health": health, "streak": self._push_fail_streak},
                )
            )
        else:
            self._push_fail_streak = 0
        stats["health_ok"] = bool(ok_h and health.get("ok"))
        stats["health_score"] = health.get("score")
        stats["push_fail_streak"] = self._push_fail_streak

        # B) Selfcheck
        ok_sc, _, selfcheck, err_sc = _http_json(
            "GET", f"{self.atlas_base}/support/selfcheck", timeout=12.0
        )
        sc_data = selfcheck.get("data") if isinstance(selfcheck.get("data"), dict) else {}
        problems = sc_data.get("problems") if isinstance(sc_data.get("problems"), list) else []
        stats["selfcheck_problem_count"] = len(problems)
        if not ok_sc:
            issues.append(
                SentinelIssue(
                    issue_id="selfcheck_unreachable",
                    severity="warning",
                    message=f"/support/selfcheck no disponible: {err_sc}",
                    source="selfcheck",
                )
            )
        else:
            for p in problems:
                if not isinstance(p, dict):
                    continue
                sev = str(p.get("severity") or "warning").lower()
                issue_sev = "critical" if sev == "critical" else "warning"
                issues.append(
                    SentinelIssue(
                        issue_id=f"selfcheck:{p.get('id','unknown')}",
                        severity=issue_sev,
                        message=str(p.get("message") or p.get("suggestion") or "selfcheck problem"),
                        source="selfcheck",
                        details=p,
                    )
                )

        # C) Screen module + capture + CV heuristics
        screen_st = self._screen_status()
        evidence["screen_status"] = screen_st
        if not screen_st.get("ok"):
            issues.append(
                SentinelIssue(
                    issue_id="screen_status_fail",
                    severity="warning",
                    message=f"/screen/status error: {screen_st.get('error')}",
                    source="screen",
                    details=screen_st,
                )
            )
        else:
            data = screen_st.get("data") if isinstance(screen_st.get("data"), dict) else {}
            if not data.get("enabled"):
                issues.append(
                    SentinelIssue(
                        issue_id="screen_disabled",
                        severity="warning",
                        message="Módulo screen deshabilitado o sin dependencias.",
                        source="screen",
                        details=data,
                    )
                )

            cap = self._capture_screen()
            evidence["screen_capture"] = cap
            if cap.get("ok") and cap.get("path"):
                cvr = self._analyze_screen_image_cv(str(cap["path"]))
                evidence["screen_cv"] = cvr
                if cvr.get("ok"):
                    flags = cvr.get("flags") if isinstance(cvr.get("flags"), list) else []
                    for flag in flags:
                        sev = "critical" if flag == "screen_too_dark" else "warning"
                        issues.append(
                            SentinelIssue(
                                issue_id=f"screen_cv:{flag}",
                                severity=sev,
                                message=f"Anomalía visual detectada: {flag}",
                                source="screen_cv",
                                details=cvr,
                            )
                        )
            else:
                issues.append(
                    SentinelIssue(
                        issue_id="screen_capture_fail",
                        severity="warning",
                        message=f"Fallo capturando pantalla: {cap.get('error')}",
                        source="screen",
                    )
                )

        # D) Camera probe (Insta360 preferida)
        cam = self._camera_probe()
        evidence["camera_probe"] = cam
        stats["camera_source"] = cam.get("source")
        stats["camera_bytes"] = cam.get("bytes")
        if not cam.get("ok"):
            self._camera_fail_streak += 1
            cam_sev = "critical" if self._camera_fail_streak >= 2 else "warning"
            issues.append(
                SentinelIssue(
                    issue_id="camera_probe_fail",
                    severity=cam_sev,
                    message=f"Camera snapshot no disponible: {cam.get('error')}",
                    source="camera",
                    details={**cam, "streak": self._camera_fail_streak},
                )
            )
        else:
            self._camera_fail_streak = 0
        stats["camera_fail_streak"] = self._camera_fail_streak

        # E) Non-destructive hands probe
        mouse = self._safe_mouse_probe()
        evidence["mouse_probe"] = mouse
        if not mouse.get("ok"):
            issues.append(
                SentinelIssue(
                    issue_id="hands_mouse_fail",
                    severity="warning",
                    message=f"Probe de mouse falló: {mouse.get('error')}",
                    source="hands",
                    details=mouse,
                )
            )

        ok = not any(i.severity == "critical" for i in issues)
        return SentinelAudit(
            ts=_now_iso(),
            ok=ok,
            issues=issues,
            stats=stats,
            evidence=evidence,
        )

    # ----------------- Self-heal -----------------
    def _in_cooldown(self, key: str, sec: int) -> bool:
        last = self._heal_cooldowns.get(key, 0.0)
        return (time.time() - last) < float(sec)

    def _mark_heal(self, key: str) -> None:
        self._heal_cooldowns[key] = time.time()

    def _run_powershell_file(self, script_path: Path, args: Optional[List[str]] = None) -> Dict[str, Any]:
        if self.dry_run:
            return {"ok": True, "dry_run": True, "script": str(script_path), "args": args or []}
        cmd = [
            "powershell",
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-WindowStyle",
            "Hidden",
            "-File",
            str(script_path),
        ] + (args or [])
        try:
            p = subprocess.run(
                cmd,
                cwd=str(ROOT),
                capture_output=True,
                text=True,
                timeout=180,
            )
            return {
                "ok": p.returncode == 0,
                "returncode": int(p.returncode),
                "stdout": (p.stdout or "")[:600],
                "stderr": (p.stderr or "")[:600],
            }
        except Exception as e:
            return {"ok": False, "error": f"{type(e).__name__}: {e}"}

    def atlas_self_heal(self, audit: SentinelAudit) -> Dict[str, Any]:
        actions: List[Dict[str, Any]] = []
        issue_ids = [i.issue_id for i in audit.issues]

        # 1) PUSH caído -> reinicio programado
        push_critical = any(i.issue_id == "push_health" and i.severity == "critical" for i in audit.issues)
        if push_critical and not self._in_cooldown("restart_push", 180):
            script = ROOT / "scripts" / "restart_push_from_api.ps1"
            if script.is_file():
                r = self._run_powershell_file(script, args=["-DelaySeconds", "1", "-HealthTimeoutSec", "120"])
                actions.append({"action": "restart_push", "result": r})
                self._mark_heal("restart_push")

        # 2) Problemas críticos de cámara -> autorepair de cámaras
        camera_critical = any(i.issue_id == "camera_probe_fail" and i.severity == "critical" for i in audit.issues)
        if camera_critical and not self._in_cooldown("camera_autorepair", 300):
            py = ROOT / ".venv" / "Scripts" / "python.exe"
            script = ROOT / "scripts" / "atlas_camera_autorepair.py"
            if py.is_file() and script.is_file():
                if self.dry_run:
                    r = {
                        "ok": True,
                        "dry_run": True,
                        "cmd": [str(py), str(script)],
                    }
                else:
                    try:
                        p = subprocess.run(
                            [str(py), str(script)],
                            cwd=str(ROOT),
                            capture_output=True,
                            text=True,
                            timeout=240,
                        )
                        r = {
                            "ok": p.returncode == 0,
                            "returncode": int(p.returncode),
                            "stdout": (p.stdout or "")[:800],
                            "stderr": (p.stderr or "")[:800],
                        }
                    except Exception as e:
                        r = {"ok": False, "error": f"{type(e).__name__}: {e}"}
                actions.append({"action": "camera_autorepair", "result": r})
                self._mark_heal("camera_autorepair")

        # 3) Inconsistencias no críticas -> ciclo ANS safe
        warning_count = sum(1 for i in audit.issues if i.severity == "warning")
        if warning_count > 0 and not self._in_cooldown("ans_run_now", 120):
            if self.dry_run:
                r = {"ok": True, "dry_run": True}
            else:
                ok, sc, body, err = _http_json(
                    "POST",
                    f"{self.atlas_base}/ans/run-now",
                    payload={"mode": "safe"},
                    timeout=30.0,
                )
                r = {"ok": ok and sc == 200 and body.get("ok", False), "status": sc, "body": body, "error": err}
            actions.append({"action": "ans_run_now", "result": r})
            self._mark_heal("ans_run_now")

        # 4) screen disabled -> intento de activar dependencias opcionales vía heal
        if "screen_disabled" in issue_ids and not self._in_cooldown("screen_deps_heal", 900):
            if self.dry_run:
                r = {"ok": True, "dry_run": True}
            else:
                ok, sc, body, err = _http_json(
                    "POST",
                    f"{self.atlas_base}/ans/run-now",
                    payload={"mode": "safe"},
                    timeout=30.0,
                )
                r = {"ok": ok and sc == 200, "status": sc, "body": body, "error": err}
            actions.append({"action": "screen_deps_heal", "result": r})
            self._mark_heal("screen_deps_heal")

        healed_ok = all((a.get("result") or {}).get("ok", False) for a in actions) if actions else True
        return {
            "ok": healed_ok,
            "actions": actions,
            "count": len(actions),
        }

    # ----------------- Heartbeat -----------------
    def atlas_heartbeat(self, audit: Optional[SentinelAudit], heal: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        crit = 0
        warn = 0
        if audit:
            crit = sum(1 for i in audit.issues if i.severity == "critical")
            warn = sum(1 for i in audit.issues if i.severity == "warning")
        health = "OK" if crit == 0 else "DEGRADED"

        msg = (
            f"[Sentinel] estado={health} crit={crit} warn={warn} "
            f"camera={audit.stats.get('camera_source') if audit else 'n/a'} "
            f"heals={heal.get('count') if heal else 0}"
        )
        _safe_print(msg)
        self._append_log(
            "heartbeat",
            {
                "health": health,
                "critical": crit,
                "warning": warn,
                "heals": heal.get("count") if heal else 0,
                "camera_source": audit.stats.get("camera_source") if audit else None,
                "camera_bytes": audit.stats.get("camera_bytes") if audit else None,
            },
        )

        if crit == 0 and warn == 0:
            # Mensaje solicitado por usuario/prompt
            self._emit_ops("Sistema al 100% de capacidad operativa.", level="info", source="sentinel")
        elif crit > 0:
            alert_txt = "Alerta Sentinel: problema crítico detectado. Requiere atención."
            self._emit_ops(alert_txt, level="error", source="sentinel")
            self._speak(alert_txt)
        return {"ok": True, "message": msg}

    # ----------------- Loop -----------------
    def run_once(self) -> int:
        _safe_print("Iniciando escaneo de módulos...")
        audit = self.atlas_visual_audit()
        self._last_audit = audit
        self._append_log(
            "audit",
            {
                "ok": audit.ok,
                "stats": audit.stats,
                "issues": [i.__dict__ for i in audit.issues],
                "evidence": audit.evidence,
            },
        )

        for i in audit.issues:
            lvl = i.severity.upper()
            _safe_print(f"[{lvl}] {i.issue_id} :: {i.message}")

        heal_result = {"ok": True, "actions": [], "count": 0}
        if self.auto_heal:
            _safe_print("Error visual/operativo detectado, aplicando parche..." if audit.issues else "Sin errores críticos, validando estabilidad...")
            heal_result = self.atlas_self_heal(audit)
            self._append_log("self_heal", heal_result)

        self.atlas_heartbeat(audit, heal_result)
        self._last_hb_ts = time.time()
        return 0 if audit.ok else 2

    def run_loop(self, *, max_cycles: int = 0) -> int:
        _safe_print("Protocolo Sentinel iniciado. ATLAS está vigilando su propia estructura.")
        self._append_log(
            "startup",
            {
                "atlas_base": self.atlas_base,
                "robot_base": self.robot_base,
                "interval_sec": self.interval_sec,
                "heartbeat_sec": self.heartbeat_sec,
                "auto_heal": self.auto_heal,
                "speak_alerts": self.speak_alerts,
                "camera_prefer_keyword": self.camera_prefer_keyword,
                "camera_prefer_id": self.camera_prefer_id,
                "camera_prefer_index": self.camera_prefer_index,
                "dry_run": self.dry_run,
            },
        )

        cycles = 0
        last_code = 0
        while not self.stop_event.is_set():
            cycles += 1
            last_code = self.run_once()

            if max_cycles > 0 and cycles >= max_cycles:
                break

            sleep_left = self.interval_sec
            while sleep_left > 0 and not self.stop_event.is_set():
                time.sleep(1.0)
                sleep_left -= 1
                now = time.time()
                if (now - self._last_hb_ts) >= self.heartbeat_sec:
                    self.atlas_heartbeat(self._last_audit, {"count": 0, "ok": True})
                    self._last_hb_ts = now

        self._append_log("shutdown", {"cycles": cycles, "last_code": last_code})
        return last_code


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="ATLAS Sentinel - visual audit + self-heal + heartbeat loop")
    p.add_argument("--atlas-base", default=os.getenv("ATLAS_PUSH_BASE", "http://127.0.0.1:8791"))
    p.add_argument("--robot-base", default=os.getenv("NEXUS_ROBOT_API_URL", "http://127.0.0.1:8002"))
    p.add_argument("--interval-sec", type=int, default=int(os.getenv("ATLAS_SENTINEL_INTERVAL_SEC", "90")))
    p.add_argument("--heartbeat-sec", type=int, default=int(os.getenv("ATLAS_SENTINEL_HEARTBEAT_SEC", "30")))
    p.add_argument("--camera-prefer-keyword", default=os.getenv("ATLAS_SENTINEL_CAMERA_KEYWORD", "insta"))
    p.add_argument("--camera-id", default=os.getenv("ATLAS_SENTINEL_CAMERA_ID", ""))
    p.add_argument("--camera-index", type=int, default=int(os.getenv("ATLAS_SENTINEL_CAMERA_INDEX", "0")))
    p.add_argument("--speak-alerts", action="store_true")
    p.add_argument("--no-auto-heal", action="store_true")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--once", action="store_true")
    p.add_argument("--max-cycles", type=int, default=0)
    return p


def main() -> int:
    try:
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

    lock_path = LOG_DIR / "atlas_sentinel.lock"
    if not _acquire_single_instance_lock(lock_path):
        _safe_print("Sentinel ya está en ejecución (lock activo).")
        return 0

    args = build_parser().parse_args()
    sentinel = AtlasSentinel(
        atlas_base=args.atlas_base,
        robot_base=args.robot_base,
        interval_sec=args.interval_sec,
        heartbeat_sec=args.heartbeat_sec,
        speak_alerts=bool(args.speak_alerts),
        auto_heal=not bool(args.no_auto_heal),
        camera_prefer_keyword=str(args.camera_prefer_keyword or "insta"),
        camera_prefer_id=str(args.camera_id or ""),
        camera_prefer_index=int(args.camera_index or 0),
        dry_run=bool(args.dry_run),
    )

    try:
        if args.once:
            _safe_print("Protocolo Sentinel iniciado. ATLAS está vigilando su propia estructura.")
            return sentinel.run_once()
        return sentinel.run_loop(max_cycles=int(args.max_cycles or 0))
    except KeyboardInterrupt:
        _safe_print("Sentinel detenido por usuario.")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
