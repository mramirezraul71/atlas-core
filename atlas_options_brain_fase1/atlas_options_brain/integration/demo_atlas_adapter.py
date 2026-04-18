"""
Demo AtlasOptionsClient (paper, sin live).

Ejecutar desde la raíz del repo padre del paquete:
  python -m atlas_options_brain.integration.demo_atlas_adapter
"""
from __future__ import annotations

from datetime import date

from atlas_options_brain.integration.atlas_adapter import AtlasOptionsClient
from atlas_options_brain.tests.test_atlas_adapter import FakeProvider


def main() -> None:
    as_of = date(2025, 5, 1)
    exp = date(2025, 6, 20)
    prov = FakeProvider([exp], spot=520.0)
    client = AtlasOptionsClient(prov, as_of=as_of)

    strat = client.build_strategy_from_chain(
        "SPY",
        "bull_put",
        params={"short_delta": 0.25, "width": 10},
    )
    pos = client.open_paper_position(strat, atlas_strategy_type="bull_put")
    print("--- Atlas view (paper) ---")
    print(client.atlas_view_row(pos))

    client.mark_all_positions()
    prov.bump_quotes(0.03)
    client.mark_all_positions()
    prov.bump_quotes(-0.02)
    client.mark_all_positions()

    print("Tras 3 marks grabados:")
    for p in client.list_open_positions():
        print(client.atlas_view_row(p))
        if p.last_snapshot and p.last_snapshot.match_info:
            kinds = ",".join(m.match_kind.value for m in p.last_snapshot.match_info)
            print(f"  last_match_kinds=[{kinds}]")


if __name__ == "__main__":
    main()
