"""Tests memoria semántica: add_experience, recall_similar, stats.

Para que pasen (no skip): PyTorch>=2.4, sentence-transformers>=3.0, datasets>=2.14, huggingface-hub>=0.19.
  pip install --upgrade "torch>=2.4.0" "sentence-transformers>=3.0.0" "datasets>=2.14.0" "huggingface-hub>=0.19.0"
"""
import os
import tempfile
import pytest

try:
    from modules.humanoid.memory_engine.semantic_memory import SemanticMemory
    _tmp = tempfile.mkdtemp()
    _path = os.path.join(_tmp, "probe.sqlite")
    try:
        _sm = SemanticMemory(db_path=_path)
        SKIP_TESTS = False
        skip_reason = ""
    except (ImportError, NameError) as e:
        SKIP_TESTS = True
        skip_reason = str(e)
    finally:
        for name in ("probe.sqlite", "embeddings.faiss"):
            p = os.path.join(_tmp, name)
            if os.path.exists(p):
                try:
                    os.unlink(p)
                except Exception:
                    pass
        try:
            os.rmdir(_tmp)
        except Exception:
            pass
except ImportError as e:
    SKIP_TESTS = True
    skip_reason = str(e)


@pytest.fixture
def semantic_memory_db():
    tmpdir = tempfile.mkdtemp()
    path = os.path.join(tmpdir, "semantic_memory.sqlite")
    try:
        yield path
    finally:
        for name in ("semantic_memory.sqlite", "embeddings.faiss"):
            p = os.path.join(tmpdir, name)
            if os.path.exists(p):
                try:
                    os.unlink(p)
                except Exception:
                    pass
        try:
            os.rmdir(tmpdir)
        except Exception:
            pass


@pytest.mark.skipif(SKIP_TESTS, reason=f"Dependencies missing: {skip_reason}")
def test_semantic_memory_add_and_recall(semantic_memory_db):
    from modules.humanoid.memory_engine.semantic_memory import SemanticMemory
    sm = SemanticMemory(db_path=semantic_memory_db)
    id1 = sm.add_experience("cocinar pasta con tomate", context="cocina", tags=["plan"])
    id2 = sm.add_experience("preparar espagueti a la boloñesa", context="cocina", tags=["plan"])
    assert id1 >= 1
    assert id2 >= 1
    results = sm.recall_similar("cocinar pasta", top_k=3, min_similarity=0.3)
    assert len(results) >= 1
    assert any("pasta" in (r.get("description") or "") or "espagueti" in (r.get("description") or "") for r in results)


@pytest.mark.skipif(SKIP_TESTS, reason=f"Dependencies missing: {skip_reason}")
def test_semantic_memory_stats(semantic_memory_db):
    from modules.humanoid.memory_engine.semantic_memory import SemanticMemory
    sm = SemanticMemory(db_path=semantic_memory_db)
    sm.add_experience("reparar bug en API", outcome="fixed", tags=["plan"])
    stats = sm.get_statistics()
    assert stats["total_experiences"] >= 1
    assert stats["embedding_dimension"] == 384
    assert "storage_size_mb" in stats


@pytest.mark.skipif(SKIP_TESTS, reason=f"Dependencies missing: {skip_reason}")
def test_semantic_memory_empty_recall(semantic_memory_db):
    from modules.humanoid.memory_engine.semantic_memory import SemanticMemory
    sm = SemanticMemory(db_path=semantic_memory_db)
    results = sm.recall_similar("cualquier query", top_k=5)
    assert results == []
