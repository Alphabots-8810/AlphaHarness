"""Tests for the auto-tuner: it must converge to better gains from bad seeds.

In-process against the flywheel plant model (fast, deterministic). The NT path is
exercised separately by demo_autotune.py against sim_plant.
"""
from alphaharness.autotune import autotune, plant_evaluator, shooter_cost

BOUNDS = {"kP": (0.5, 30.0), "kD": (0.0, 1.5)}
STEPS = {"kP": 4.0, "kD": 0.2}


def _run(seed, noise=0.0, budget=30):
    ev = plant_evaluator(target=60.0, noise=noise)
    seed_cost = shooter_cost(ev(seed))
    res = autotune(ev, seed=seed, bounds=BOUNDS, steps=STEPS, budget=budget)
    return ev, seed_cost, res


def test_converges_from_sluggish_seed():
    # kP=3 -> big steady-state error, slow; tuner should raise kP + add kD
    ev, seed_cost, res = _run({"kP": 3.0, "kD": 0.0})
    assert res["best_cost"] < 0.7 * seed_cost           # meaningfully better
    assert res["best_gains"]["kP"] > 8                  # moved toward the bowl
    assert res["best_metrics"]["overshoot_pct"] < 12    # not by overshooting


def test_calms_aggressive_seed():
    # kP=28, no damping -> oscillatory; tuner should back kP off / add kD
    ev, seed_cost, res = _run({"kP": 28.0, "kD": 0.0})
    assert res["best_cost"] < seed_cost
    assert res["best_gains"]["kP"] < 28.0


def test_improves_under_noise():
    # the optimizer must not be fooled by a noisy/quantized capture
    ev, seed_cost, res = _run({"kP": 3.0, "kD": 0.0}, noise=0.3, budget=30)
    assert res["best_cost"] < seed_cost
    assert res["best_gains"]["kP"] > 6


def test_history_is_recorded():
    _, _, res = _run({"kP": 5.0, "kD": 0.0}, budget=12)
    assert len(res["history"]) == res["evals"] <= 12
    assert all("cost" in h and "gains" in h for h in res["history"])


def test_cost_penalizes_overshoot():
    low = shooter_cost({"rise_time_s": 0.1, "overshoot_pct": 2.0, "steady_state_error_pct": 0.0})
    high = shooter_cost({"rise_time_s": 0.1, "overshoot_pct": 40.0, "steady_state_error_pct": 0.0})
    assert high > low
    # runaway capture gets a big finite cost, not inf/crash
    assert shooter_cost({"rise_time_s": None, "overshoot_pct": 400.0,
                         "steady_state_error_pct": 0.0}) > 40
