"""AlphaHarness — synthetic NT4 robot (ground-truth substrate).

8810's ShooterIOSim has no physics model, so `./gradlew simulateJava` won't
produce a realistic step response. This module is the stand-in: a standalone
NT4 *server* that publishes a controllable 2nd-order step response whose
overshoot / settle / damping are known in CLOSED FORM. That lets us validate
the whole NT4 -> capture -> metrics pipeline against ground truth, with zero
robot and zero physics-sim setup.

CRITICAL (advisor seam #1): it publishes the *same shape and rate the real
robot does* —
  * setpoint  -> sparse, on-change edge   (/Tuning/SHooterRPS)
  * measured  -> dense ~50 Hz trajectory  (/AdvantageKit/RealOutputs/Shooter/MeasuredRPS)
so green tests exercise the exact ingestion path production uses, not a fiction.

Run:
    python -m alphaharness.sim_robot --zeta 0.5 --wn 18 --target 60 --noise 0.4
Then point AlphaHarness at 127.0.0.1 and capture_step_response.
"""
from __future__ import annotations

import argparse
import math
import time

import numpy as np
import ntcore


def second_order_step(tau: float, a: float, b: float, zeta: float, wn: float) -> float:
    """Value at time `tau` after a step a->b of an underdamped 2nd-order plant.

    y(0)=a, y(inf)=b. For zeta>=1 returns the (over/critically) damped form.
    """
    if tau <= 0:
        return a
    if wn <= 0:
        return a          # degenerate (no dynamics); avoids div-by-zero in the overdamped branch
    span = b - a
    if zeta < 1.0:
        wd = wn * math.sqrt(1 - zeta**2)
        phi = math.acos(zeta)
        env = math.exp(-zeta * wn * tau) / math.sqrt(1 - zeta**2)
        return b - span * env * math.sin(wd * tau + phi)
    elif abs(zeta - 1.0) < 1e-6:
        return b - span * (1 + wn * tau) * math.exp(-wn * tau)
    else:
        r = math.sqrt(zeta**2 - 1)
        s1 = -wn * (zeta - r)
        s2 = -wn * (zeta + r)
        c1 = s2 / (s2 - s1)
        c2 = -s1 / (s2 - s1)
        return b - span * (c1 * math.exp(s1 * tau) + c2 * math.exp(s2 * tau))


def analytic_ground_truth(zeta: float, wn: float, step: float) -> dict:
    """Textbook closed-form metrics for the underdamped step (for comparison)."""
    gt = {"zeta": zeta, "wn_rad_s": wn}
    if zeta <= 1e-9 or wn <= 0:
        # undamped / marginal: settle = 4/(zeta*wn) -> division by zero; report cleanly
        gt["overshoot_pct"] = 100.0
        gt["regime"] = "undamped/marginal"
        gt["settle_2pct_s"] = None
        gt["settle_5pct_s"] = None
    elif zeta < 1.0:
        wd = wn * math.sqrt(1 - zeta**2)
        Mp = math.exp(-zeta * math.pi / math.sqrt(1 - zeta**2))
        gt["overshoot_pct"] = 100.0 * Mp
        gt["peak_time_s"] = math.pi / wd
        gt["damped_freq_hz"] = wd / (2 * math.pi)
        gt["natural_freq_hz"] = wn / (2 * math.pi)
        gt["settle_2pct_s"] = 4.0 / (zeta * wn)   # standard approximation
        gt["settle_5pct_s"] = 3.0 / (zeta * wn)
    else:
        gt["overshoot_pct"] = 0.0
        gt["regime"] = "overdamped/critical"
    return gt


def main():
    ap = argparse.ArgumentParser(description="AlphaHarness synthetic NT4 robot")
    ap.add_argument("--zeta", type=float, default=0.5, help="damping ratio")
    ap.add_argument("--wn", type=float, default=18.0, help="natural freq (rad/s)")
    ap.add_argument("--target", type=float, default=60.0, help="step target (RPS)")
    ap.add_argument("--period", type=float, default=6.0, help="seconds between toggles")
    ap.add_argument("--once", action="store_true",
                    help="deterministic single step 0->target after --warmup, then hold "
                         "(for end-to-end tests); no further toggles")
    ap.add_argument("--warmup", type=float, default=2.0,
                    help="seconds to hold 0 before the single step in --once mode")
    ap.add_argument("--rate", type=float, default=50.0, help="measurement publish Hz")
    ap.add_argument("--noise", type=float, default=0.3, help="gaussian noise std (units)")
    ap.add_argument("--quant", type=float, default=0.05, help="quantization step (units)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--setpoint-key", default="/Tuning/SHooterRPS")
    ap.add_argument("--measurement-key",
                    default="/AdvantageKit/RealOutputs/Shooter/MeasuredRPS")
    ap.add_argument("--current-key",
                    default="/AdvantageKit/RealOutputs/Shooter/StatorCurrent")
    ap.add_argument("--current-limit", type=float, default=60.0,
                    help="stator current limit (A) the synthetic effort saturates at")
    ap.add_argument("--dense-setpoint", action="store_true",
                    help="ALSO publish a dense setpoint output "
                         "(/AdvantageKit/RealOutputs/Shooter/SetpointRPS) to exercise "
                         "the 'target provided' ingestion path")
    ap.add_argument("--dense-setpoint-key",
                    default="/AdvantageKit/RealOutputs/Shooter/SetpointRPS")
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)

    gt = analytic_ground_truth(args.zeta, args.wn, args.target)
    print("=" * 64)
    print("AlphaHarness synthetic robot — NT4 server on 127.0.0.1:5810")
    print(f"  zeta={args.zeta}  wn={args.wn} rad/s  target={args.target}")
    print(f"  setpoint (sparse) : {args.setpoint_key}")
    print(f"  measured (dense)  : {args.measurement_key}  @ {args.rate} Hz")
    if args.dense_setpoint:
        print(f"  setpoint (dense)  : {args.dense_setpoint_key}")
    print("  GROUND TRUTH (closed form):")
    for k, v in gt.items():
        print(f"    {k:18s} = {v:.4f}" if isinstance(v, float) else f"    {k:18s} = {v}")
    print("  toggling 0 <-> target every "
          f"{args.period}s. Ctrl-C to stop.")
    print("=" * 64)

    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startServer()
    time.sleep(0.3)  # let the server bind

    sp_pub = inst.getDoubleTopic(args.setpoint_key).publish()
    meas_pub = inst.getDoubleTopic(args.measurement_key).publish()
    cur_pub = inst.getDoubleTopic(args.current_key).publish()
    dense_sp_pub = (inst.getDoubleTopic(args.dense_setpoint_key).publish()
                    if args.dense_setpoint else None)

    # publish our OWN ground truth on NT so the e2e test compares against the
    # actual sim params (structurally impossible to drift / hardcode the wrong ref)
    gt_pubs = {}
    for k, v in gt.items():
        if isinstance(v, (int, float)):
            p = inst.getDoubleTopic(f"/GroundTruth/{k}").publish()
            p.set(float(v))
            gt_pubs[k] = p

    # step state
    a, b = 0.0, 0.0                  # from-value, to-value of the active step
    start = time.monotonic()
    t_step = start
    next_toggle = start + (args.warmup if args.once else args.period)
    stepped_once = False
    sp_pub.set(b)                    # initial sparse publish
    if dense_sp_pub:
        dense_sp_pub.set(b)

    dt = 1.0 / args.rate
    kP = 7.0                          # rough effort model for the current signal
    try:
        while True:
            now = time.monotonic()

            # sparse setpoint edge on toggle
            do_step = (now >= next_toggle) and not (args.once and stepped_once)
            if do_step:
                a = b
                b = args.target if b == 0.0 else 0.0
                t_step = now
                next_toggle = float("inf") if args.once else now + args.period
                stepped_once = True
                sp_pub.set(b)                       # <-- sparse, on-change only
                if dense_sp_pub:
                    dense_sp_pub.set(b)
                print(f"[{now - start:.2f}] STEP {a:.0f} -> {b:.0f}")

            # dense measurement: 2nd-order response + noise + quantization
            tau = now - t_step
            y = second_order_step(tau, a, b, args.zeta, args.wn)
            y_noisy = y + rng.normal(0.0, args.noise)
            if args.quant > 0:
                y_noisy = round(y_noisy / args.quant) * args.quant
            meas_pub.set(y_noisy)

            # dense setpoint (only if enabled) republished each loop = real-robot
            # behaviour IF they add Logger.recordOutput for the setpoint
            if dense_sp_pub:
                dense_sp_pub.set(b)

            # crude stator-current proxy: effort ~ kP*error, saturated at limit
            err = b - y
            cur = min(abs(kP * err), args.current_limit)
            cur_pub.set(cur)

            inst.flush()
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nstopping synthetic robot.")
    finally:
        inst.stopServer()


if __name__ == "__main__":
    main()
