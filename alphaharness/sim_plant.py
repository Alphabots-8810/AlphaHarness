"""AlphaHarness — closed-loop flywheel NT4 server (the auto-tune demo target).

Wraps FlywheelPlant behind NT: reads the live gains AlphaHarness writes to
/Tuning/Shooter/* and the setpoint at /Tuning/SHooterRPS, runs the PIDF + plant
each loop, and publishes the response. Because the response DEPENDS on the gains,
the auto-tuner can actually optimize against it — unlike sim_robot.py (fixed play).

    python -m alphaharness.sim_plant          # then run autotune against 127.0.0.1
"""
from __future__ import annotations

import argparse
import time

import numpy as np
import ntcore

from .plant import FlywheelPlant

GAINS = ["kP", "kI", "kD", "kS", "kV"]
SEED = {"kP": 3.0, "kI": 0.0, "kD": 0.0, "kS": 0.0, "kV": 0.0}   # deliberately sluggish


def main():
    ap = argparse.ArgumentParser(description="AlphaHarness closed-loop flywheel NT server")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--noise", type=float, default=0.25)
    ap.add_argument("--quant", type=float, default=0.05)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--meas-key", default="/AdvantageKit/RealOutputs/Shooter/MeasuredRPS")
    ap.add_argument("--cur-key", default="/AdvantageKit/RealOutputs/Shooter/StatorCurrent")
    ap.add_argument("--sp-key", default="/Tuning/SHooterRPS")
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    plant = FlywheelPlant()

    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startServer()
    time.sleep(0.3)

    meas_pub = inst.getDoubleTopic(args.meas_key).publish()
    cur_pub = inst.getDoubleTopic(args.cur_key).publish()
    sp_pub = inst.getDoubleTopic(args.sp_key).publish()
    sp_pub.set(0.0)

    # publish seed gains so the topics exist, and subscribe to read client overrides
    gain_pubs, gain_subs = {}, {}
    for g in GAINS:
        key = f"/Tuning/Shooter/{g}"
        gain_pubs[g] = inst.getDoubleTopic(key).publish()
        gain_pubs[g].set(SEED[g])
        gain_subs[g] = inst.getDoubleTopic(key).subscribe(SEED[g])
    sp_sub = inst.getDoubleTopic(args.sp_key).subscribe(0.0)

    print("AlphaHarness closed-loop flywheel — NT4 server on 127.0.0.1:5810")
    print(f"  reads gains /Tuning/Shooter/* + setpoint {args.sp_key}; publishes {args.meas_key}")
    print("  seed gains (sluggish):", SEED, "— run autotune to improve. Ctrl-C to stop.")

    dt = 1.0 / args.rate
    try:
        while True:
            gains = {g: gain_subs[g].get() for g in GAINS}
            sp = sp_sub.get()
            w, ia = plant.step(dt, sp, gains["kP"], gains["kI"], gains["kD"],
                               gains["kS"], gains["kV"])
            wn = w + rng.normal(0.0, args.noise)
            if args.quant > 0:
                wn = round(wn / args.quant) * args.quant
            meas_pub.set(float(wn))
            cur_pub.set(float(ia))
            sp_pub.set(float(sp))     # republish setpoint dense (so it shows in AdvantageScope)
            inst.flush()
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nstopping flywheel server.")
    finally:
        inst.stopServer()


if __name__ == "__main__":
    main()
