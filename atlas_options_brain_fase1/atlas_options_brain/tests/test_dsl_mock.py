"""Test unitario con datos mock – NO requiere API key."""
import sys, os, math
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, ROOT)

from datetime import date
from atlas_options_brain.models.option_contract import (
    OptionContract, OptionsChain, OptionType, Greeks
)
from atlas_options_brain.dsl.strategy import (
    IronCondor, BullPutSpread, BearCallSpread,
    BullCallSpread, BearPutSpread, CoveredCall,
)


def bs_approx_price(spot, strike, days=45, iv=0.20, r=0.05, is_call=True):
    """Aproximación simple Black-Scholes para precios realistas en el mock."""
    T = days / 365.0
    if T <= 0 or iv <= 0:
        return max(0.01, spot - strike if is_call else strike - spot)
    d1 = (math.log(spot / strike) + (r + 0.5 * iv**2) * T) / (iv * math.sqrt(T))
    d2 = d1 - iv * math.sqrt(T)
    # Approx normal CDF
    def N(x): return 0.5 * (1 + math.erf(x / math.sqrt(2)))
    if is_call:
        price = spot * N(d1) - strike * math.exp(-r * T) * N(d2)
    else:
        price = strike * math.exp(-r * T) * N(-d2) - spot * N(-d1)
    return max(0.01, round(price, 2))


def make_chain(symbol="SPY", spot=520.0, exp=date(2025, 6, 20)) -> OptionsChain:
    contracts = []
    strikes = [spot + (i - 15) * 5 for i in range(31)]
    for strike in strikes:
        distance = (strike - spot) / spot
        call_delta = max(0.01, min(0.99, 0.50 - distance * 2.5))
        put_delta  = max(0.01, min(0.99, 0.50 + distance * 2.5))
        for ot, delta in [
            (OptionType.CALL, call_delta),
            (OptionType.PUT,  put_delta),
        ]:
            mid    = bs_approx_price(spot, strike, is_call=(ot == OptionType.CALL))
            spread = max(0.02, mid * 0.05)
            contracts.append(OptionContract(
                symbol=symbol, option_type=ot, strike=float(strike),
                expiration=exp, last_price=mid,
                bid=round(mid - spread, 2), ask=round(mid + spread, 2),
                volume=500, open_interest=1000,
                greeks=Greeks(delta=delta, gamma=0.02, theta=-0.05, vega=0.15, iv=0.22),
            ))
    return OptionsChain(symbol=symbol, expiration=exp, spot_price=spot, contracts=contracts)


if __name__ == "__main__":
    chain = make_chain()
    print(f"Chain: {chain.symbol} spot={chain.spot_price} contracts={len(chain.contracts)}")

    ic = IronCondor.from_chain(chain, wing_delta=0.20, wing_width=10)
    print(ic.summary())
    assert ic.net_premium > 0,   f"IC debe tener crédito neto, got {ic.net_premium}"
    assert ic.max_loss < 0,      "IC max_loss debe ser negativo"
    assert len(ic.breakevens) == 2

    bps = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
    print(bps.summary())
    assert bps.net_premium > 0

    bcs = BearCallSpread.from_chain(chain, short_delta=0.25, width=10)
    print(bcs.summary())
    assert bcs.net_premium > 0

    bulls = BullCallSpread.from_chain(chain, long_delta=0.50, width=10)
    print(bulls.summary())
    assert bulls.max_profit > 0

    bears = BearPutSpread.from_chain(chain, long_delta=0.50, width=10)
    print(bears.summary())
    assert bears.max_profit > 0

    cc = CoveredCall.from_chain(chain, short_delta=0.25, stock_basis=510.0)
    print(cc.summary())
    assert cc.net_premium > 0

    print("\n✅  Todos los tests pasaron correctamente.")
