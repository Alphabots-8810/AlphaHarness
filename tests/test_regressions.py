"""Regression tests for bugs found in the 2026-06-07 adversarial audit.

Each test pins a confirmed bug so it can't silently come back. (The server-side
async / M2 setpoint-zeroing fixes are integration-level and covered by the live
MCP checks in tests.e2e_mcp / tests.demo_autotune.)
"""
import numpy as np

from alphaharness.metrics import compute_step_response_metrics, _damping_from_peak
from alphaharness.autotune import shooter_cost
from alphaharness.sim_robot import analytic_ground_truth


def _first_order(n=120, tgt=60.0):
    t = np.linspace(0, 2, n)
    y = tgt * (1 - np.exp(-t / 0.2))
    return t, y


# M3 — empty (non-None) current array must degrade to None, not crash np.max
def test_M3_empty_current_no_crash():
    t, y = _first_order()
    m = compute_step_response_metrics(t, y, 0.0, 60.0, y0=0.0,
                                      i_cur=np.array([]), current_limit=60.0)
    assert m["peak_current"] is None and m["saturated"] is None


# L3 — overshoot >= 100% must not yield a negative zeta labeled 'underdamped'
def test_L3_no_negative_zeta_above_100pct():
    for ov in (100.0, 120.0, 200.0):
        r = _damping_from_peak(ov, 0.2)
        assert r["damping_ratio"] is None and r["regime"] == "unstable"
    assert _damping_from_peak(50.0, 0.2)["damping_ratio"] is not None   # still works below 100


# L4 — a NaN sample must not produce a NaN peak paired with a bogus 0% overshoot
def test_L4_nan_sample_filtered():
    t, y = _first_order()
    y = y.copy(); y[60] = np.nan
    m = compute_step_response_metrics(t, y, 0.0, 60.0, y0=0.0)
    assert np.isfinite(m["peak_value"]) and np.isfinite(m["steady_state_value"])


# L5 — settle-band keys must be injective for fractional-percent bands
def test_L5_settle_keys_distinct():
    t, y = _first_order()
    ks = [k for k in compute_step_response_metrics(t, y, 0.0, 60.0, y0=0.0,
                                                   settle_bands=(0.02, 0.025))
          if k.startswith("settle_time")]
    assert set(ks) == {"settle_time_2pct", "settle_time_2.5pct"}


# L6 — saturation_fraction is over the POST-step window when t_cur is given
def test_L6_saturation_fraction_post_step():
    t = np.linspace(0, 4, 200); ts = 1.0
    y = np.where(t < ts, 0.0, 60.0); i = np.where(t < ts, 0.0, 60.0)
    m = compute_step_response_metrics(t, y, ts, 60.0, t_cur=t, i_cur=i, current_limit=60.0)
    assert m["saturation_fraction"] > 0.95            # not the ~0.75 whole-capture dilution


# M1 — cost must be monotone across the 150% overshoot boundary (no downward cliff)
def test_M1_cost_monotone_across_150():
    c149 = shooter_cost({"overshoot_pct": 149, "rise_time_s": 0.5})
    c151 = shooter_cost({"overshoot_pct": 151, "rise_time_s": 0.5})
    c500 = shooter_cost({"overshoot_pct": 500, "rise_time_s": 0.5})
    assert c151 >= c149 and c500 > c151


# L7 — a real rise_time of 0.0 is best-case, not the 2.5 'missing' fallback
def test_L7_rise_zero_is_zero():
    assert shooter_cost({"rise_time_s": 0.0, "overshoot_pct": 0.0, "steady_state_error_pct": 0.0}) == 0.0
    assert shooter_cost({"rise_time_s": None, "overshoot_pct": 0.0, "steady_state_error_pct": 0.0}) == 2.5


# L8 — zeta=0 must not raise ZeroDivisionError
def test_L8_zeta_zero_no_crash():
    gt = analytic_ground_truth(0.0, 18.0, 60.0)
    assert gt["regime"] == "undamped/marginal"
    assert gt["settle_2pct_s"] is None
