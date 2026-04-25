from __future__ import annotations

from atlas_scanner.features.gamma import (
    GammaRegime,
    StrikeGamma,
    classify_gamma_regime,
    find_call_wall,
    find_gamma_flip,
    find_put_wall,
)


def test_find_call_and_put_wall_simple_case() -> None:
    strikes = (
        StrikeGamma(strike=3900.0, call_gamma=120.0, put_gamma=-80.0),
        StrikeGamma(strike=3950.0, call_gamma=300.0, put_gamma=-120.0),
        StrikeGamma(strike=4000.0, call_gamma=180.0, put_gamma=-250.0),
    )
    assert find_call_wall(strikes) == 3950.0
    assert find_put_wall(strikes) == 4000.0


def test_find_call_and_put_wall_empty_or_zero_cases() -> None:
    assert find_call_wall(()) is None
    assert find_put_wall(()) is None

    all_zero = (
        StrikeGamma(strike=3900.0, call_gamma=0.0, put_gamma=0.0),
        StrikeGamma(strike=3950.0, call_gamma=0.0, put_gamma=0.0),
    )
    assert find_call_wall(all_zero) is None
    assert find_put_wall(all_zero) is None


def test_find_gamma_flip_with_sign_change() -> None:
    strikes = (
        StrikeGamma(strike=3900.0, call_gamma=4.0, put_gamma=1.0),   # +5
        StrikeGamma(strike=3950.0, call_gamma=3.0, put_gamma=1.0),   # +4
        StrikeGamma(strike=4000.0, call_gamma=-2.0, put_gamma=-4.0), # -6
    )
    gamma_flip = find_gamma_flip(strikes)
    assert gamma_flip is not None
    assert 3950.0 < gamma_flip < 4000.0


def test_find_gamma_flip_without_sign_change_or_short_input() -> None:
    positive = (
        StrikeGamma(strike=3900.0, call_gamma=2.0, put_gamma=1.0),
        StrikeGamma(strike=3950.0, call_gamma=3.0, put_gamma=1.0),
    )
    negative = (
        StrikeGamma(strike=3900.0, call_gamma=-2.0, put_gamma=-1.0),
        StrikeGamma(strike=3950.0, call_gamma=-3.0, put_gamma=-1.0),
    )
    assert find_gamma_flip(positive) is None
    assert find_gamma_flip(negative) is None
    assert find_gamma_flip(()) is None
    assert find_gamma_flip((StrikeGamma(strike=3900.0, call_gamma=1.0, put_gamma=-0.5),)) is None


def test_classify_gamma_regime_default_threshold() -> None:
    assert classify_gamma_regime(10.0, neutral_threshold=0.0) == GammaRegime.POSITIVE
    assert classify_gamma_regime(-10.0, neutral_threshold=0.0) == GammaRegime.NEGATIVE
    assert classify_gamma_regime(0.0, neutral_threshold=0.0) == GammaRegime.NEUTRAL


def test_classify_gamma_regime_with_neutral_band() -> None:
    assert classify_gamma_regime(0.5, neutral_threshold=1.0) == GammaRegime.NEUTRAL
    assert classify_gamma_regime(-0.5, neutral_threshold=1.0) == GammaRegime.NEUTRAL
    assert classify_gamma_regime(1.5, neutral_threshold=1.0) == GammaRegime.POSITIVE
    assert classify_gamma_regime(-1.5, neutral_threshold=1.0) == GammaRegime.NEGATIVE

