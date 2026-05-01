"""Pipeline de ingest para la biblioteca académica de ATLAS-Quant.

Funciones:
  1. generate_fichas()       — genera fichas JSON desde el catálogo en memoria
  2. ingest_open_access()    — intenta descargar PDFs open-access a knowledge/raw/
  3. ingest_status()         — devuelve estado del pipeline (fichas generadas, papers descargados)

Los PDFs se guardan en knowledge/raw/<id>.pdf
Las fichas JSON se guardan en knowledge/fichas/<id>.json

El ingest es SOLO para papers open-access (NBER, SSRN, Meb Faber, etc.).
No descarga contenido protegido por copyright.
"""
from __future__ import annotations

import json
import logging
import os
import time
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

# Directorio base del módulo knowledge
_KNOWLEDGE_DIR = Path(__file__).resolve().parent
_FICHAS_DIR = _KNOWLEDGE_DIR / "fichas"
_RAW_DIR = _KNOWLEDGE_DIR / "raw"

# Papers conocidos con acceso libre — URL directa a PDF o abstract
_OPEN_ACCESS_URLS: dict[str, str] = {
    "fama_1965_rw": "https://www.jstor.org/stable/4469865",
    "moskowitz_ooi_pedersen_2012": "https://papers.ssrn.com/sol3/papers.cfm?abstract_id=1878282",
    "jegadeesh_titman_1993": "https://papers.ssrn.com/sol3/papers.cfm?abstract_id=227234",
    "lo_mackinlay_1988_vr": "https://papers.ssrn.com/sol3/papers.cfm?abstract_id=1128673",
    "asness_moskowitz_pedersen_2013": "https://papers.ssrn.com/sol3/papers.cfm?abstract_id=2174501",
    "faber_2007_tactical": "https://papers.ssrn.com/sol3/papers.cfm?abstract_id=962461",
    "daniel_hirshleifer_subrahmanyam_1998": "https://papers.ssrn.com/sol3/papers.cfm?abstract_id=278645",
    "barberis_thaler_2003": "https://papers.ssrn.com/sol3/papers.cfm?abstract_id=278748",
    "kaminski_lo_2014_stoploss": "https://papers.ssrn.com/sol3/papers.cfm?abstract_id=2127657",
    "almgren_chriss_2001_execution": "https://papers.ssrn.com/sol3/papers.cfm?abstract_id=1086983",
    "lo_mamaysky_wang_2000": "https://papers.ssrn.com/sol3/papers.cfm?abstract_id=225210",
    "jegadeesh_1990_reversal": "https://papers.ssrn.com/sol3/papers.cfm?abstract_id=227250",
    "debondt_thaler_1985": "https://papers.ssrn.com/sol3/papers.cfm?abstract_id=244174",
    "lo_mackinlay_1990_overreactions": "https://papers.ssrn.com/sol3/papers.cfm?abstract_id=227241",
    "brinson_hood_beebower_1986": "https://doi.org/10.2469/faj.v42.n4.39",
    "lopez_de_prado_2018_afml": "https://www.wiley.com/en-us/Advances+in+Financial+Machine+Learning-p-9781119482086",
    "perold_1988_is": "https://doi.org/10.2469/faj.v44.n1.23",
    "black_scholes_1973": "https://doi.org/10.1086/260062",
    "ohara_1995_mmt": "",
    "grinold_kahn_2000_apm": "",
    "tsay_2010_fts": "",
    "van_tharp_1999_riskmanagement": "",
    "natenberg_1994_options_volatility": "",
    "prechter_frost_1978_elliott": "",
    "poser_2003_elliott_effectiveness": "",
    "almgren_chriss_2001_execution": "https://papers.ssrn.com/sol3/papers.cfm?abstract_id=1086983",
}


def generate_fichas(sources: list[dict[str, Any]] | None = None) -> dict[str, Any]:
    """Genera fichas JSON para todas las fuentes del catálogo.

    Escribe un archivo JSON por fuente en knowledge/fichas/<id>.json
    Retorna resumen de la operación.
    """
    from atlas_code_quant.knowledge.sources_catalog import SOURCES as _CATALOG

    if sources is None:
        sources = _CATALOG

    _FICHAS_DIR.mkdir(parents=True, exist_ok=True)

    created = []
    updated = []
    errors = []

    for src in sources:
        sid = src.get("id")
        if not sid:
            errors.append({"error": "source without id", "source": str(src)[:80]})
            continue
        fpath = _FICHAS_DIR / f"{sid}.json"
        # Enriquecer la ficha con URL open-access si está en el mapa
        ficha = dict(src)
        if not ficha.get("url") and sid in _OPEN_ACCESS_URLS:
            ficha["url"] = _OPEN_ACCESS_URLS[sid]
        ficha["_generated_at"] = _iso_now()
        existed = fpath.exists()
        try:
            with open(fpath, "w", encoding="utf-8") as f:
                json.dump(ficha, f, ensure_ascii=False, indent=2)
            (updated if existed else created).append(sid)
        except OSError as e:
            errors.append({"id": sid, "error": str(e)})

    return {
        "fichas_dir": str(_FICHAS_DIR),
        "created": len(created),
        "updated": len(updated),
        "errors": len(errors),
        "created_ids": created,
        "updated_ids": updated,
        "error_details": errors,
    }


def ingest_open_access(
    source_ids: list[str] | None = None,
    timeout_sec: float = 10.0,
) -> dict[str, Any]:
    """Intenta descargar páginas de abstract/landing de papers open-access.

    No descarga PDFs completos — guarda la página de abstract como .html
    en knowledge/raw/<id>.html (respeta copyright: solo metadatos públicos).

    Args:
        source_ids: IDs específicos a intentar. Si None, intenta todos.
        timeout_sec: Timeout HTTP por request.

    Returns:
        Resumen: {downloaded, skipped, failed, details}
    """
    try:
        import urllib.request
        import urllib.error
    except ImportError:
        return {"error": "urllib not available"}

    _RAW_DIR.mkdir(parents=True, exist_ok=True)

    if source_ids is None:
        source_ids = [sid for sid, url in _OPEN_ACCESS_URLS.items() if url]

    downloaded = []
    skipped = []
    failed = []

    for sid in source_ids:
        url = _OPEN_ACCESS_URLS.get(sid, "")
        if not url:
            skipped.append({"id": sid, "reason": "no open-access URL"})
            continue

        out_path = _RAW_DIR / f"{sid}.html"
        if out_path.exists():
            skipped.append({"id": sid, "reason": "already downloaded"})
            continue

        try:
            req = urllib.request.Request(
                url,
                headers={
                    "User-Agent": (
                        "ATLAS-Quant Academic Ingest Bot/1.0 "
                        "(research reference retrieval; contact: atlas@internal)"
                    )
                },
            )
            with urllib.request.urlopen(req, timeout=timeout_sec) as resp:
                content = resp.read()
            with open(out_path, "wb") as f:
                f.write(content)
            downloaded.append({"id": sid, "url": url, "bytes": len(content)})
            time.sleep(1.0)  # Rate limit: 1 request/sec
        except Exception as e:
            failed.append({"id": sid, "url": url, "error": str(e)[:120]})
            logger.warning("[ingest] Failed %s: %s", sid, e)

    return {
        "raw_dir": str(_RAW_DIR),
        "downloaded": len(downloaded),
        "skipped": len(skipped),
        "failed": len(failed),
        "download_details": downloaded,
        "skip_details": skipped,
        "fail_details": failed,
    }


def ingest_status() -> dict[str, Any]:
    """Retorna estado actual del pipeline de ingest."""
    from atlas_code_quant.knowledge.sources_catalog import SOURCES as _CATALOG

    total_sources = len(_CATALOG)
    fichas_count = len(list(_FICHAS_DIR.glob("*.json"))) if _FICHAS_DIR.exists() else 0
    raw_count = len(list(_RAW_DIR.glob("*"))) if _RAW_DIR.exists() else 0

    open_access_ids = [sid for sid, url in _OPEN_ACCESS_URLS.items() if url]

    return {
        "total_sources": total_sources,
        "fichas_generated": fichas_count,
        "fichas_pending": total_sources - fichas_count,
        "fichas_dir": str(_FICHAS_DIR),
        "raw_papers_downloaded": raw_count,
        "raw_dir": str(_RAW_DIR),
        "open_access_candidates": len(open_access_ids),
        "open_access_ids": open_access_ids,
        "fichas_complete": fichas_count >= total_sources,
    }


def _iso_now() -> str:
    from datetime import datetime, timezone
    return datetime.now(timezone.utc).isoformat()
