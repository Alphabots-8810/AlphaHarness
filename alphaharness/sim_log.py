"""AlphaHarness — synthetic .wpilog generator (ground-truth substrate for scope b).

Writes a .wpilog with the SAME topology a real AdvantageKit log has — a sparse
on-change setpoint plus a dense ~50 Hz measurement — driven by a known 2nd-order
step, plus its closed-form ground truth under /GroundTruth/*. Lets the offline
arm be validated against truth, exactly like sim_robot.py does for the live arm.

    python -m alphaharness.sim_log --zeta 0.5 --wn 18 --target 60 --out /tmp/shot.wpilog
"""
from __future__ import annotations

import argparse

import numpy as np
from wpiutil import DataLogWriter
from wpiutil.log import DoubleLogEntry

from .sim_robot import second_order_step, analytic_ground_truth

# nonzero base time: WPILog treats timestamp 0 as "use current wall clock"
_BASE_US = 1_000_000


def write_step_log(path: str, *, zeta: float = 0.5, wn: float = 18.0,
                   target: float = 60.0, warmup: float = 1.0, duration: float = 4.0,
                   rate: float = 50.0, noise: float = 0.3, quant: float = 0.05,
                   seed: int = 0,
                   setpoint_key: str = "/Tuning/SHooterRPS",
                   measurement_key: str = "/AdvantageKit/RealOutputs/Shooter/MeasuredRPS",
                   current_key: str = "/AdvantageKit/RealOutputs/Shooter/StatorCurrent",
                   current_limit: float = 60.0) -> dict:
    """Write the synthetic log; return the analytic ground truth dict."""
    rng = np.random.default_rng(seed)
    gt = analytic_ground_truth(zeta, wn, target)

    w = DataLogWriter(path)
    sp = DoubleLogEntry(w, setpoint_key)
    meas = DoubleLogEntry(w, measurement_key)
    cur = DoubleLogEntry(w, current_key)
    gt_entries = {k: DoubleLogEntry(w, f"/GroundTruth/{k}")
                  for k, v in gt.items() if isinstance(v, (int, float))}

    # ground truth (retained, at base time)
    for k, e in gt_entries.items():
        e.append(float(gt[k]), _BASE_US)

    # sparse setpoint: 0 at base, step to target at warmup
    t_step_us = _BASE_US + int(warmup * 1e6)
    sp.append(0.0, _BASE_US)
    sp.append(float(target), t_step_us)

    # dense measurement + current
    dt = 1.0 / rate
    n = int((warmup + duration) * rate)
    kP = 7.0
    for i in range(n):
        t_rel = i * dt
        ts_us = _BASE_US + int(t_rel * 1e6)
        tau = t_rel - warmup
        a, b = (0.0, target) if tau >= 0 else (0.0, 0.0)
        y = second_order_step(tau, a, b, zeta, wn) if tau >= 0 else 0.0
        y_n = y + rng.normal(0.0, noise)
        if quant > 0:
            y_n = round(y_n / quant) * quant
        meas.append(float(y_n), ts_us)
        cur.append(float(min(abs(kP * (b - y)), current_limit)), ts_us)

    w.flush()
    try:
        w.stop()
    except Exception:
        pass
    del w
    return gt


def main():
    ap = argparse.ArgumentParser(description="AlphaHarness synthetic .wpilog generator")
    ap.add_argument("--zeta", type=float, default=0.5)
    ap.add_argument("--wn", type=float, default=18.0)
    ap.add_argument("--target", type=float, default=60.0)
    ap.add_argument("--noise", type=float, default=0.3)
    ap.add_argument("--quant", type=float, default=0.05)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="/tmp/alphaharness_shot.wpilog")
    args = ap.parse_args()
    gt = write_step_log(args.out, zeta=args.zeta, wn=args.wn, target=args.target,
                        noise=args.noise, quant=args.quant, seed=args.seed)
    print(f"wrote {args.out}")
    print("ground truth:", {k: round(v, 4) for k, v in gt.items() if isinstance(v, (int, float))})


if __name__ == "__main__":
    main()
