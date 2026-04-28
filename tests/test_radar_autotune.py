from pathlib import Path

from modules.atlas_radar_kalshi.autotune import AutoTuneConfig, AutoTuneController


def test_autotune_reacts_to_losses(tmp_path: Path) -> None:
    ctrl = AutoTuneController(
        log_dir=tmp_path,
        cfg=AutoTuneConfig(mode="assisted", enabled=True, degrade_loss_streak=3),
    )
    runtime = {
        "edge_threshold": 0.03,
        "edge_net_min": 0.02,
        "kelly_fraction": 0.30,
        "confidence_min": 0.55,
        "max_open_positions": 15,
    }
    metrics = {
        "performance": {"current_drawdown_cents": 2400, "expectancy_cents": -300},
        "risk": {"consecutive_losses": 4},
        "actionable_pct": 0.12,
    }
    patch = ctrl.propose(runtime, metrics)
    assert "loss_reaction" in patch.reason
    assert patch.kelly_fraction is not None
    assert patch.kelly_fraction < runtime["kelly_fraction"]
    assert patch.edge_net_min is not None
    assert patch.edge_net_min > runtime["edge_net_min"]


def test_autotune_post_window_degradation_triggers_false_keep(tmp_path: Path) -> None:
    ctrl = AutoTuneController(log_dir=tmp_path, cfg=AutoTuneConfig(mode="auto", enabled=True))
    runtime = {
        "edge_threshold": 0.03,
        "edge_net_min": 0.02,
        "kelly_fraction": 0.25,
        "confidence_min": 0.55,
        "max_open_positions": 10,
    }
    before = {
        "performance": {"current_drawdown_cents": 1000, "expectancy_cents": 120},
        "risk": {"consecutive_losses": 1},
        "actionable_pct": 0.25,
    }
    patch = ctrl.propose(runtime, before)
    ctrl.mark_applied(patch, runtime, before)
    after = {
        "performance": {"current_drawdown_cents": 2200, "expectancy_cents": -90},
        "risk": {"consecutive_losses": 3},
        "actionable_pct": 0.20,
    }
    keep = ctrl.evaluate_post_window(after)
    assert keep is False
