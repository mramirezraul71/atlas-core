"""
Atlas Options Brain – Demo Rápida (sin API key, usa yfinance)
=============================================================
Ejecutar:  python demo.py
Requiere:  pip install yfinance
"""
from datetime import date
from atlas_options_brain import (
    YFinanceProvider, IronCondor, BullPutSpread,
    BearCallSpread, CoveredCall,
)


def main():
    print("Conectando a yfinance (gratuito, solo dev)...")
    provider = YFinanceProvider()

    symbol = "SPY"
    spot   = provider.get_quote(symbol)
    exps   = provider.get_expirations(symbol)
    print(f"SPY spot: ${spot:.2f}  |  Próximos vencimientos: {exps[:5]}")

    # Elige el vencimiento ~30-45 DTE
    exp = exps[2] if len(exps) > 2 else exps[0]
    print(f"\nUsando vencimiento: {exp}")

    chain = provider.get_chain(symbol, exp)
    print(f"Cadena cargada: {len(chain.contracts)} contratos")

    # ── Iron Condor δ0.16 width 5 ──────────────────────────────────────────
    try:
        ic = IronCondor.from_chain(chain, wing_delta=0.16, wing_width=5)
        print("\n" + ic.summary())
    except Exception as e:
        print(f"Iron Condor: {e}")

    # ── Bull Put Spread δ0.25 width 5 ──────────────────────────────────────
    try:
        bps = BullPutSpread.from_chain(chain, short_delta=0.25, width=5)
        print("\n" + bps.summary())
    except Exception as e:
        print(f"Bull Put Spread: {e}")

    # ── Covered Call δ0.25 ─────────────────────────────────────────────────
    try:
        cc = CoveredCall.from_chain(chain, short_delta=0.25)
        print("\n" + cc.summary())
    except Exception as e:
        print(f"Covered Call: {e}")


if __name__ == "__main__":
    main()
