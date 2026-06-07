"""Ground-truth tests for the offline WPILOG arm (scope b).

Writes a synthetic .wpilog (known 2nd-order step), reads it back through the same
metric layer the live path uses, and checks the numbers match closed form — the
file analogue of e2e_sim.py.
"""
import math
import tempfile

import pytest

from alphaharness.sim_log import write_step_log
from alphaharness.wpilog_reader import analyze_step, list_entries

MEAS = "/AdvantageKit/RealOutputs/Shooter/MeasuredRPS"
SETP = "/Tuning/SHooterRPS"
CUR = "/AdvantageKit/RealOutputs/Shooter/StatorCurrent"


def _make(zeta, wn, target, **kw):
    path = tempfile.mktemp(suffix=".wpilog")
    gt = write_step_log(path, zeta=zeta, wn=wn, target=target, **kw)
    return path, gt


def test_wpilog_roundtrip_matches_ground_truth():
    path, gt = _make(0.45, 20.0, 55.0, noise=0.35, quant=0.05, seed=3)
    m = analyze_step(path, MEAS, SETP, current_key=CUR, current_limit=60.0)
    # overshoot within ~2 pts of closed form, over a noisy/quantized log
    assert abs(m["overshoot_pct"] - gt["overshoot_pct"]) < 2.0
    assert abs(m["damping_ratio"] - gt["zeta"]) < 0.08
    assert abs(m["steady_state_error"]) < 1.0
    assert m["_step_source"] == "inferred_edge"     # target inferred from sparse /Tuning edge
    assert m["saturated"] is True                   # current proxy saturates near the step


def test_wpilog_list_entries():
    path, _ = _make(0.5, 18.0, 60.0)
    names = {e["name"] for e in list_entries(path)}
    assert MEAS in names and SETP in names
    assert any(n.startswith("/GroundTruth/") for n in names)


def test_wpilog_explicit_target_overdamped():
    path, gt = _make(1.3, 12.0, 40.0, noise=0.2)
    m = analyze_step(path, MEAS, target=40.0)        # no setpoint key -> provided target
    assert m["_step_source"] == "provided"
    assert m["overshoot_pct"] < 3.0
    assert m["regime"] == "overdamped"


def test_wpilog_hood_guard():
    path, _ = _make(0.5, 18.0, 60.0)
    with pytest.raises(ValueError, match="profile"):
        analyze_step(path, "/AdvantageKit/RealOutputs/Hood/Angle",
                     "/Tuning/HoodAngle")
