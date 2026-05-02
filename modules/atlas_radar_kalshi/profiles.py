"""Perfiles operativos de radar (gating + riesgo)."""

from __future__ import annotations

# Sin RADAR_PROFILE en entorno: paper con reglas cercanas a aggressive pero gating
# tolerante a libros finos (REST/demo/prod) para que haya flujo de órdenes virtuales.
RADAR_DEFAULT_PROFILE: str = "paper_aggressive"

RADAR_PROFILE_PRESETS: dict[str, dict[str, dict[str, float | int]]] = {
    # Calibración en paper (feed REST/Kalshi público, libros 1-cara): gating
    # permisivo; riesgo acotado pero sin cortar por DD pequeño como aggressive.
    "paper_calib": {
        "gate": {
            "edge_net_min": 0.012,
            "confidence_min": 0.32,
            "spread_max_ticks": 10,
            "min_depth_yes": 1,
            "min_depth_no": 1,
            "max_quote_age_ms": 60000,
            "max_latency_ms": 2000,
            "cooldown_seconds": 15,
        },
        "risk": {
            "kelly_fraction": 0.12,
            "max_position_pct": 0.02,
            "max_market_exposure_pct": 0.12,
            "max_total_exposure_pct": 0.45,
            "daily_dd_limit_pct": 0.10,
            "weekly_dd_limit_pct": 0.18,
            "max_consecutive_losses": 8,
            "max_open_positions": 12,
            "max_orders_per_minute": 18,
        },
    },
    # Perfil robusto para feed público/paper.
    "paper_safe": {
        "gate": {
            "edge_net_min": 0.02,
            "confidence_min": 0.45,
            "spread_max_ticks": 6,
            "min_depth_yes": 10,
            "min_depth_no": 10,
            "max_quote_age_ms": 15000,
            "max_latency_ms": 1500,
            "cooldown_seconds": 45,
        },
        "risk": {
            "kelly_fraction": 0.15,
            "max_position_pct": 0.02,
            "max_market_exposure_pct": 0.10,
            "max_total_exposure_pct": 0.30,
            "daily_dd_limit_pct": 0.02,
            "weekly_dd_limit_pct": 0.05,
            "max_consecutive_losses": 3,
            "max_open_positions": 5,
            "max_orders_per_minute": 12,
        },
    },
    # Perfil recomendado para operación continua.
    "balanced": {
        "gate": {
            "edge_net_min": 0.035,
            "confidence_min": 0.60,
            "spread_max_ticks": 3,
            "min_depth_yes": 1000,
            "min_depth_no": 1000,
            "max_quote_age_ms": 2000,
            "max_latency_ms": 1500,
            "cooldown_seconds": 300,
        },
        "risk": {
            "kelly_fraction": 0.25,
            "max_position_pct": 0.025,
            "max_market_exposure_pct": 0.15,
            "max_total_exposure_pct": 0.55,
            "daily_dd_limit_pct": 0.03,
            "weekly_dd_limit_pct": 0.07,
            "max_consecutive_losses": 5,
            "max_open_positions": 10,
            "max_orders_per_minute": 10,
        },
    },
    # Perfil agresivo: solo con track-record validado.
    "aggressive": {
        "gate": {
            "edge_net_min": 0.025,
            "confidence_min": 0.55,
            "spread_max_ticks": 4,
            "min_depth_yes": 500,
            "min_depth_no": 500,
            "max_quote_age_ms": 1000,
            "max_latency_ms": 1200,
            "cooldown_seconds": 120,
        },
        "risk": {
            "kelly_fraction": 0.30,
            "max_position_pct": 0.04,
            "max_market_exposure_pct": 0.20,
            "max_total_exposure_pct": 0.65,
            "daily_dd_limit_pct": 0.05,
            "weekly_dd_limit_pct": 0.10,
            "max_consecutive_losses": 7,
            "max_open_positions": 15,
            "max_orders_per_minute": 20,
        },
    },
    # Paper + datos reales (demo o prod) + intención agresiva: riesgo alto pero
    # gating compatible con profundidad baja / colas REST (evita “solo decisiones”).
    "paper_aggressive": {
        "gate": {
            "edge_net_min": 0.0135,
            "confidence_min": 0.40,
            "spread_max_ticks": 8,
            "min_depth_yes": 8,
            "min_depth_no": 8,
            "max_quote_age_ms": 25000,
            "max_latency_ms": 1800,
            "cooldown_seconds": 28,
        },
        "risk": {
            "kelly_fraction": 0.28,
            "max_position_pct": 0.038,
            "max_market_exposure_pct": 0.18,
            "max_total_exposure_pct": 0.62,
            "daily_dd_limit_pct": 0.05,
            "weekly_dd_limit_pct": 0.10,
            "max_consecutive_losses": 7,
            "max_open_positions": 14,
            "max_orders_per_minute": 22,
        },
    },
}

