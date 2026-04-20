## Summary

- Documents the canonical `atlas_core` layout and its relationship with the `atlas_code_quant.atlas_core` shim (`atlas_core/CANONICAL.md`).
- Adds a manifest (`atlas_core/MANIFEST.json`) plus a verification script (`scripts/verify_atlas_core_manifest.py`) to ensure core modules import cleanly.
- Adds a focused test suite (`test_atlas_core_canonical_manifest.py`) as a lightweight reproducibility/consistency check; no trading logic changes.

## Testing

- `python atlas_code_quant/scripts/verify_atlas_core_manifest.py` → exit 0.
- `python -m pytest atlas_code_quant/tests/test_atlas_core_canonical_manifest.py -q` → 7 passed.

## Risks

- Manifest drift if the `atlas_core` tree evolves; mitigated by the verify script and potential CI integration.
- Versioning information (if any) can become stale unless tied into the release process; this PR keeps it minimal and documentation-focused.

## Follow-ups

- Optionally add CI jobs to run the manifest verify script on main/release branches.
- Explore Docker/compose or similar for a full-stack smoke environment based on the documented core.
- Extend the canonical docs with more detailed architectural diagrams if needed.
