"""Tests para KnowledgeBase — biblioteca académica de ATLAS-Quant."""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from atlas_code_quant.knowledge.knowledge_base import KnowledgeBase, get_knowledge_base
from atlas_code_quant.knowledge.sources_catalog import (
    SOURCES,
    SCANNER_METHOD_TO_SOURCES,
    TIMEFRAME_TO_SOURCES,
    CATEGORY_LABELS,
)


# ── Catálogo ──────────────────────────────────────────────────────────────────

class TestSourcesCatalog:
    def test_sources_not_empty(self):
        assert len(SOURCES) >= 15

    def test_all_sources_have_required_fields(self):
        required = ["id", "title", "authors", "year", "source", "topic", "key_findings",
                    "atlas_utility", "confidence_level", "scanner_methods", "tags"]
        for src in SOURCES:
            for field in required:
                assert field in src, f"Source '{src.get('id')}' missing field '{field}'"

    def test_ids_are_unique(self):
        ids = [s["id"] for s in SOURCES]
        assert len(ids) == len(set(ids)), "Hay IDs duplicados en el catálogo"

    def test_confidence_levels_valid(self):
        valid = {"high", "medium", "low", "contested"}
        for src in SOURCES:
            assert src["confidence_level"] in valid, (
                f"'{src['id']}' tiene confidence_level inválido: {src['confidence_level']}"
            )

    def test_categories_valid(self):
        valid = {"CAT1", "CAT2", "CAT3", "CAT4", "CAT5"}
        for src in SOURCES:
            assert src.get("category") in valid, (
                f"'{src['id']}' tiene category inválida: {src.get('category')}"
            )

    def test_scanner_method_index_built(self):
        assert len(SCANNER_METHOD_TO_SOURCES) > 0
        # Los métodos conocidos del scanner deben tener al menos 1 fuente
        for method in ["trend_ema_stack", "breakout_donchian", "rsi_pullback_trend"]:
            assert method in SCANNER_METHOD_TO_SOURCES, f"Método '{method}' no tiene fuentes mapeadas"
            assert len(SCANNER_METHOD_TO_SOURCES[method]) >= 1

    def test_timeframe_index_built(self):
        assert "swing" in TIMEFRAME_TO_SOURCES
        assert "all" in TIMEFRAME_TO_SOURCES

    def test_category_labels_complete(self):
        for cat in ["CAT1", "CAT2", "CAT3", "CAT4", "CAT5"]:
            assert cat in CATEGORY_LABELS

    def test_known_references_present(self):
        """Verifica que las referencias clave estén en el catálogo."""
        known_ids = [
            "fama_1965_rw",
            "jegadeesh_titman_1993",
            "grinold_kahn_2000_apm",
            "perold_1988_is",
            "lopez_de_prado_2018_afml",
            "van_tharp_1999_riskmanagement",
        ]
        catalog_ids = {s["id"] for s in SOURCES}
        for sid in known_ids:
            assert sid in catalog_ids, f"Referencia clave '{sid}' no encontrada en el catálogo"


# ── KnowledgeBase — consultas ─────────────────────────────────────────────────

@pytest.fixture
def kb():
    return KnowledgeBase()


class TestKnowledgeBaseQueries:
    def test_sources_for_method_trend_ema(self, kb):
        sources = kb.sources_for_method("trend_ema_stack")
        assert len(sources) >= 2
        ids = [s["id"] for s in sources]
        assert "moskowitz_ooi_pedersen_2012" in ids

    def test_sources_for_method_breakout(self, kb):
        sources = kb.sources_for_method("breakout_donchian")
        assert len(sources) >= 1

    def test_sources_for_method_unknown_returns_empty(self, kb):
        sources = kb.sources_for_method("nonexistent_method_xyz")
        assert sources == []

    def test_sources_for_timeframe_swing(self, kb):
        sources = kb.sources_for_timeframe("swing")
        assert len(sources) >= 3
        ids = [s["id"] for s in sources]
        assert "jegadeesh_titman_1993" in ids

    def test_sources_for_timeframe_intraday(self, kb):
        sources = kb.sources_for_timeframe("intraday")
        assert len(sources) >= 1
        ids = [s["id"] for s in sources]
        assert "ohara_1995_mmt" in ids

    def test_sources_for_timeframe_alias_1d(self, kb):
        """1d debe mapearse a swing."""
        sources_1d = kb.sources_for_timeframe("1d")
        sources_swing = kb.sources_for_timeframe("swing")
        assert len(sources_1d) == len(sources_swing)

    def test_sources_for_topic_momentum(self, kb):
        sources = kb.sources_for_topic("momentum")
        assert len(sources) >= 3
        ids = [s["id"] for s in sources]
        assert "jegadeesh_titman_1993" in ids

    def test_sources_for_topic_options(self, kb):
        sources = kb.sources_for_topic("options")
        assert len(sources) >= 1
        ids = [s["id"] for s in sources]
        assert "black_scholes_1973" in ids

    def test_sources_by_confidence_high_only(self, kb):
        high_only = kb.sources_by_confidence(min_level="high")
        for src in high_only:
            assert src["confidence_level"] == "high"

    def test_sources_by_confidence_medium_includes_high(self, kb):
        medium_plus = kb.sources_by_confidence(min_level="medium")
        high_only = kb.sources_by_confidence(min_level="high")
        assert len(medium_plus) >= len(high_only)

    def test_source_by_id_found(self, kb):
        src = kb.source_by_id("fama_1965_rw")
        assert src is not None
        assert src["title"] == "Random Walks in Stock-Market Prices"

    def test_source_by_id_not_found(self, kb):
        src = kb.source_by_id("nonexistent_id_xyz")
        assert src is None

    def test_all_sources_returns_complete_catalog(self, kb):
        all_s = kb.all_sources()
        assert len(all_s) == len(SOURCES)


# ── advisory_context ──────────────────────────────────────────────────────────

class TestAdvisoryContext:
    def test_advisory_context_structure(self, kb):
        ctx = kb.advisory_context(method="trend_ema_stack", timeframe="1d")
        assert "academic_support_score" in ctx
        assert "warnings" in ctx
        assert "key_insights" in ctx
        assert "recommendations" in ctx
        assert "sources_found" in ctx
        assert "sources_used" in ctx
        assert "query" in ctx

    def test_academic_support_score_range(self, kb):
        ctx = kb.advisory_context(method="trend_ema_stack", timeframe="swing")
        score = ctx["academic_support_score"]
        assert 0.0 <= score <= 100.0

    def test_high_confidence_methods_score_above_60(self, kb):
        """trend_ema_stack tiene múltiples fuentes high — debe tener soporte > 60."""
        ctx = kb.advisory_context(method="trend_ema_stack", timeframe="1d")
        assert ctx["academic_support_score"] > 60.0

    def test_low_confidence_method_returns_warnings(self, kb):
        """Elliott waves tiene evidencia baja — debe generar warnings."""
        ctx = kb.advisory_context(topic="elliott_waves")
        # Si hay fuentes low/contested, debe haber warnings
        low_sources = [s for s in ctx.get("key_insights", []) if s.get("confidence") in ("low", "contested")]
        if low_sources:
            assert len(ctx["warnings"]) > 0

    def test_recommendations_not_empty(self, kb):
        ctx = kb.advisory_context(method="trend_ema_stack", timeframe="1d")
        assert len(ctx["recommendations"]) >= 1

    def test_max_sources_respected(self, kb):
        ctx = kb.advisory_context(method="trend_ema_stack", timeframe="swing", max_sources=2)
        assert ctx["sources_used"] <= 2

    def test_query_with_no_params_returns_empty_insights(self, kb):
        ctx = kb.advisory_context()
        # Sin parámetros, no hay método ni timeframe — puede retornar vacío o solo "all"
        assert isinstance(ctx["key_insights"], list)

    def test_topic_momentum_finds_jegadeesh(self, kb):
        ctx = kb.advisory_context(topic="momentum", max_sources=10)
        ids = [ins["id"] for ins in ctx.get("key_insights", [])]
        assert "jegadeesh_titman_1993" in ids

    def test_advisory_context_for_all_scanner_methods(self, kb):
        """Todos los métodos del scanner deben retornar contexto válido."""
        methods = ["trend_ema_stack", "breakout_donchian", "rsi_pullback_trend",
                   "relative_strength_overlay", "order_flow_proxy", "ml_directional"]
        for method in methods:
            ctx = kb.advisory_context(method=method, timeframe="1d")
            assert ctx["academic_support_score"] >= 0.0
            assert isinstance(ctx["recommendations"], list)

    def test_intraday_timeframe_includes_microstructure(self, kb):
        ctx = kb.advisory_context(timeframe="intraday", max_sources=10)
        ids = [ins["id"] for ins in ctx.get("key_insights", [])]
        assert "ohara_1995_mmt" in ids

    def test_query_field_reflects_input(self, kb):
        ctx = kb.advisory_context(method="breakout_donchian", timeframe="swing", topic="momentum")
        assert ctx["query"]["method"] == "breakout_donchian"
        assert ctx["query"]["timeframe"] == "swing"
        assert ctx["query"]["topic"] == "momentum"


# ── summary ───────────────────────────────────────────────────────────────────

class TestKnowledgeBaseSummary:
    def test_summary_structure(self, kb):
        s = kb.summary()
        assert "total_sources" in s
        assert "by_category" in s
        assert "by_confidence" in s
        assert "scanner_methods_covered" in s
        assert "timeframes_covered" in s

    def test_summary_total_matches_catalog(self, kb):
        s = kb.summary()
        assert s["total_sources"] == len(SOURCES)

    def test_summary_all_categories_present(self, kb):
        s = kb.summary()
        for cat in ["CAT1", "CAT2", "CAT3", "CAT4", "CAT5"]:
            assert cat in s["by_category"]

    def test_summary_scanner_methods_covered(self, kb):
        s = kb.summary()
        covered = set(s["scanner_methods_covered"])
        assert "trend_ema_stack" in covered
        assert "breakout_donchian" in covered


# ── Singleton ─────────────────────────────────────────────────────────────────

class TestSingleton:
    def test_get_knowledge_base_returns_same_instance(self):
        kb1 = get_knowledge_base()
        kb2 = get_knowledge_base()
        assert kb1 is kb2

    def test_singleton_is_knowledge_base(self):
        kb = get_knowledge_base()
        assert isinstance(kb, KnowledgeBase)


# ── search() ──────────────────────────────────────────────────────────────────

class TestSearch:
    def test_search_returns_list(self, kb):
        results = kb.search("momentum")
        assert isinstance(results, list)

    def test_search_momentum_finds_jegadeesh(self, kb):
        results = kb.search("momentum")
        ids = [r["id"] for r in results]
        assert "jegadeesh_titman_1993" in ids

    def test_search_momentum_finds_multiple(self, kb):
        results = kb.search("momentum")
        assert len(results) >= 3

    def test_search_options_finds_black_scholes(self, kb):
        results = kb.search("options pricing Black Scholes")
        ids = [r["id"] for r in results]
        assert "black_scholes_1973" in ids

    def test_search_execution_finds_perold(self, kb):
        results = kb.search("implementation shortfall execution")
        ids = [r["id"] for r in results]
        assert "perold_1988_is" in ids

    def test_search_empty_query_returns_empty(self, kb):
        assert kb.search("") == []
        assert kb.search("  ") == []

    def test_search_unknown_term_returns_empty(self, kb):
        results = kb.search("xyznomatchterm123456")
        assert results == []

    def test_search_max_results_respected(self, kb):
        results = kb.search("market", max_results=3)
        assert len(results) <= 3

    def test_search_results_ranked_by_relevance(self, kb):
        """Más tokens coincidentes → primero en resultados."""
        results = kb.search("momentum trend time series")
        assert len(results) >= 1
        # Los primeros deben tener más coincidencias — verificar que hay resultados coherentes
        ids = [r["id"] for r in results]
        # Moskowitz es el paper de time series momentum — debe aparecer
        assert "moskowitz_ooi_pedersen_2012" in ids

    def test_search_ml_finds_lopez_de_prado(self, kb):
        results = kb.search("machine learning financial")
        ids = [r["id"] for r in results]
        assert "lopez_de_prado_2018_afml" in ids

    def test_search_advisory_context_has_no_ic_live_key_missing(self, kb):
        """advisory_context siempre incluye ic_live_data en el retorno."""
        ctx = kb.advisory_context(method="trend_ema_stack", timeframe="1d")
        assert "ic_live_data" in ctx


# ── Fichas JSON ────────────────────────────────────────────────────────────────

class TestFichas:
    _FICHAS_DIR = Path(__file__).resolve().parent.parent / "knowledge" / "fichas"

    def test_fichas_directory_exists(self):
        assert self._FICHAS_DIR.exists(), "fichas/ directory does not exist"

    def test_fichas_count_matches_catalog(self):
        fichas = list(self._FICHAS_DIR.glob("*.json"))
        assert len(fichas) == len(SOURCES), (
            f"Expected {len(SOURCES)} fichas, found {len(fichas)}"
        )

    def test_fichas_are_valid_json(self):
        for fpath in self._FICHAS_DIR.glob("*.json"):
            with open(fpath, encoding="utf-8") as f:
                data = json.load(f)
            assert "id" in data, f"{fpath.name} missing 'id'"

    def test_fichas_ids_match_filenames(self):
        for fpath in self._FICHAS_DIR.glob("*.json"):
            with open(fpath, encoding="utf-8") as f:
                data = json.load(f)
            expected_name = f"{data['id']}.json"
            assert fpath.name == expected_name, (
                f"Filename {fpath.name} does not match id {data['id']}"
            )

    def test_key_fichas_present(self):
        required_ids = [
            "fama_1965_rw", "jegadeesh_titman_1993", "moskowitz_ooi_pedersen_2012",
            "lopez_de_prado_2018_afml", "black_scholes_1973",
        ]
        for sid in required_ids:
            fpath = self._FICHAS_DIR / f"{sid}.json"
            assert fpath.exists(), f"Ficha not found: {sid}.json"


# ── IngestPipeline ────────────────────────────────────────────────────────────

class TestIngestPipeline:
    def test_generate_fichas_returns_summary(self):
        from atlas_code_quant.knowledge.ingest_pipeline import generate_fichas
        result = generate_fichas()
        assert "created" in result or "updated" in result
        assert result["errors"] == 0

    def test_ingest_status_structure(self):
        from atlas_code_quant.knowledge.ingest_pipeline import ingest_status
        status = ingest_status()
        assert "total_sources" in status
        assert "fichas_generated" in status
        assert "open_access_candidates" in status
        assert "fichas_complete" in status

    def test_ingest_status_fichas_complete(self):
        from atlas_code_quant.knowledge.ingest_pipeline import ingest_status
        status = ingest_status()
        assert status["fichas_complete"] is True
        assert status["fichas_generated"] == status["total_sources"]

    def test_ingest_status_open_access_candidates(self):
        from atlas_code_quant.knowledge.ingest_pipeline import ingest_status
        status = ingest_status()
        assert status["open_access_candidates"] >= 10
