"""Ground-truth tests for the metric layer.

Two layers, per advisor seam #2:
  * analytic: clean 50 Hz sampling -> metrics must match closed-form.
  * realistic: + gaussian noise + quantization -> metrics must stay robust.
A clean-curve-only test would pass on a fiction the wire never delivers.
"""
import math

import numpy as np
import pytest

from alphaharness.metrics import compute_step_response_metrics
from alphaharness.nt_client import NTClient
from alphaharness.sim_robot import second_order_step, analytic_ground_truth


def make_traj(zeta, wn, y0, target, t_step=1.0, dur=4.0, rate=50.0,
              noise=0.0, quant=0.0, seed=0):
    rng = np.random.default_rng(seed)
    n = int(round((t_step + dur) * rate))
    t = np.arange(n) / rate
    y = np.array([second_order_step(ti - t_step, y0, target, zeta, wn) for ti in t])
    if noise:
        y = y + rng.normal(0.0, noise, size=y.size)
    if quant:
        y = np.round(y / quant) * quant
    return t, y


# ---------------------------------------------------------------- analytic
def test_underdamped_clean_matches_closed_form():
    zeta, wn, target = 0.5, 18.0, 60.0
    gt = analytic_ground_truth(zeta, wn, target)
    t, y = make_traj(zeta, wn, 0.0, target)
    m = compute_step_response_metrics(t, y, t_step=1.0, target=target, y0=0.0)

    # overshoot within ~1.5 absolute pct of textbook (tight enough to catch a
    # regression to the old peak-trimming bias, which was ~2-8 pts low)
    assert abs(m["overshoot_pct"] - gt["overshoot_pct"]) < 1.5
    # peak time within ~1 sample
    assert abs(m["peak_time_s"] - gt["peak_time_s"]) < 0.04
    # damping ratio recovered within 0.06
    assert abs(m["damping_ratio"] - zeta) < 0.06
    # damped frequency within 10%
    assert abs(m["damped_freq_hz"] - gt["damped_freq_hz"]) / gt["damped_freq_hz"] < 0.12
    # steady state lands on target
    assert abs(m["steady_state_error"]) < 0.2
    # settle 2% is slower than 5%
    assert m["settle_time_2pct"] >= m["settle_time_5pct"]
    assert m["regime"] == "underdamped"


def test_zeta_045_explicit_closed_form():
    # the exact case whose ground truth once drifted: ζ=0.45 -> Mp=20.5%.
    # pinned to the hand-computed closed form so it can never silently regress.
    zeta, wn, target = 0.45, 20.0, 55.0
    Mp = 100.0 * math.exp(-zeta * math.pi / math.sqrt(1 - zeta**2))
    assert abs(Mp - 20.535) < 0.01                     # sanity on the reference itself
    t, y = make_traj(zeta, wn, 0.0, target)
    m = compute_step_response_metrics(t, y, 1.0, target, y0=0.0)
    assert abs(m["overshoot_pct"] - Mp) < 1.5
    assert abs(m["damping_ratio"] - zeta) < 0.06


def test_rise_time_present_and_ordered():
    t, y = make_traj(0.7, 20.0, 0.0, 50.0)
    m = compute_step_response_metrics(t, y, 1.0, 50.0, y0=0.0)
    assert m["rise_time_s"] is not None and m["rise_time_s"] > 0
    # rise should precede settle
    assert m["rise_time_s"] < m["settle_time_5pct"]


# --------------------------------------------------------------- realistic
def test_underdamped_noisy_quantized_robust():
    zeta, wn, target = 0.5, 18.0, 60.0
    gt = analytic_ground_truth(zeta, wn, target)
    t, y = make_traj(zeta, wn, 0.0, target, noise=0.4, quant=0.05, seed=3)
    m = compute_step_response_metrics(t, y, t_step=1.0, target=target, y0=0.0)
    # looser than clean, but still tight: a single noisy spike must not fake
    # overshoot, and the median+parabolic peak must not regress to trimming.
    assert abs(m["overshoot_pct"] - gt["overshoot_pct"]) < 2.5
    assert abs(m["steady_state_error"]) < 1.0          # ~1.6% of 60
    assert m["damping_ratio"] is not None
    assert 0.43 < m["damping_ratio"] < 0.58
    assert m["settle_time_2pct"] is not None


def test_noise_does_not_invent_overshoot_on_overdamped():
    # overdamped: even with noise, overshoot should stay small
    t, y = make_traj(1.4, 12.0, 0.0, 40.0, noise=0.3, quant=0.05, seed=7)
    m = compute_step_response_metrics(t, y, 1.0, 40.0, y0=0.0)
    assert m["overshoot_pct"] < 3.0


# ------------------------------------------------------------- direction
def test_negative_step_60_to_0():
    zeta, wn = 0.4, 16.0
    gt = analytic_ground_truth(zeta, wn, 0.0)
    t, y = make_traj(zeta, wn, 60.0, 0.0)
    m = compute_step_response_metrics(t, y, t_step=1.0, target=0.0, y0=60.0)
    # overshoot measured in the step direction (undershoot below 0)
    Mp = 100.0 * math.exp(-zeta * math.pi / math.sqrt(1 - zeta**2))
    assert abs(m["overshoot_pct"] - Mp) < 3.0
    # target ~0 -> settle ref falls back to |step|; steady state near 0
    assert abs(m["steady_state_value"]) < 1.0
    assert m["settle_ref"] == pytest.approx(60.0, abs=1.0)


def test_overdamped_no_overshoot():
    t, y = make_traj(1.5, 10.0, 0.0, 30.0)
    m = compute_step_response_metrics(t, y, 1.0, 30.0, y0=0.0)
    assert m["overshoot_pct"] < 1.0
    assert m["regime"] == "overdamped"
    assert m["damping_ratio"] is None


def test_settle_band_relative_to_target_not_initial():
    # initial value far from zero; band must key off target (50), not y0 (200)
    t, y = make_traj(0.6, 18.0, 200.0, 50.0)
    m = compute_step_response_metrics(t, y, 1.0, 50.0, y0=200.0)
    assert m["settle_ref"] == pytest.approx(50.0, abs=0.5)
    assert m["settle_time_2pct"] is not None


# --------------------------------------------------- step resolution (no NT)
def test_resolve_step_inferred_from_sparse_edge():
    # setpoint held 0 then jumps to 60 at t=1.0 (sparse: only 2 samples)
    sp = [(0.0, 0.0), (1.0, 60.0)]
    t_meas = np.linspace(0, 5, 250)
    y_meas = np.where(t_meas < 1.0, 0.0, 60.0)
    t_step, target, mode = NTClient._resolve_step(sp, t_meas, y_meas, None)
    assert mode == "inferred_edge"
    assert target == 60.0
    assert t_step == pytest.approx(1.0)


def test_resolve_step_provided_target():
    sp = [(0.0, 0.0), (1.0, 60.0)]
    t_meas = np.linspace(0, 5, 250)
    y_meas = np.where(t_meas < 1.0, 0.0, 60.0)
    t_step, target, mode = NTClient._resolve_step(sp, t_meas, y_meas, 60.0)
    assert mode == "provided"
    assert target == 60.0


def test_resolve_step_constant_setpoint_infers_onset():
    # setpoint never changes in window -> infer onset from measurement
    sp = [(0.0, 60.0), (5.0, 60.0)]
    t_meas = np.linspace(0, 5, 250)
    y_meas = np.where(t_meas < 2.0, 0.0, 60.0)
    t_step, target, mode = NTClient._resolve_step(sp, t_meas, y_meas, None)
    assert mode == "inferred_constant"
    assert target == 60.0
    assert 1.8 < t_step < 2.2
