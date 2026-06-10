"""Live auto-tune demo: AlphaHarness tunes a closed-loop flywheel over NT.

Launch `python -m alphaharness.sim_plant` first, then this. Every evaluation is a
real NT round-trip: write kP/kD -> command a step -> capture the response -> score.
"""
import sys
import time

from alphaharness.autotune import autotune, shooter_cost
from alphaharness.nt_client import NTClient

MEAS = "/AdvantageKit/RealOutputs/Shooter/MeasuredRPS"
SP = "/Tuning/ShooterRPS"
CUR = "/AdvantageKit/RealOutputs/Shooter/StatorCurrent"
TARGET = 60.0


def main():
    c = NTClient("AlphaHarness-autotune")
    for attempt in range(6):
        try:
            print("connected:", c.connect(server="127.0.0.1", timeout=10.0)); break
        except Exception as e:
            print(f"  retry {attempt+1}: {e}"); time.sleep(3)
    else:
        print("RESULT: FAIL ❌ (no sim_plant)"); sys.exit(1)
    time.sleep(0.5)

    def evaluate(g):
        c.set_gain("/Tuning/Shooter/kP", g["kP"])
        c.set_gain("/Tuning/Shooter/kD", g["kD"])
        return c.command_step_and_capture(MEAS, SP, TARGET, duration_s=2.0, current_key=CUR)

    seed = {"kP": 3.0, "kD": 0.0}
    print("\nmeasuring seed (sluggish)...")
    seed_m = evaluate(seed)
    seed_cost = shooter_cost(seed_m)
    print(f"SEED kP={seed['kP']} kD={seed['kD']}: OS={seed_m['overshoot_pct']:.1f}% "
          f"rise={seed_m['rise_time_s']:.3f} sse%={seed_m['steady_state_error_pct']:.2f} cost={seed_cost:.3f}")

    print("\nAUTOTUNING live over NT (perturb → measure → score → set_gain → repeat)...")
    res = autotune(evaluate, seed=seed, bounds={"kP": (0.5, 30.0), "kD": (0.0, 1.5)},
                   steps={"kP": 4.0, "kD": 0.2}, budget=22, log=print)
    bg, bm = res["best_gains"], res["best_metrics"]
    c.set_gain("/Tuning/Shooter/kP", bg["kP"])
    c.set_gain("/Tuning/Shooter/kD", bg["kD"])
    c.disconnect()

    print(f"\n=== RESULT ({res['evals']} live evaluations over NT) ===")
    print(f"  seed:  kP={seed['kP']:.1f} kD={seed['kD']:.2f}  cost={seed_cost:.3f}  "
          f"(OS {seed_m['overshoot_pct']:.1f}%, sse {seed_m['steady_state_error_pct']:.1f}%)")
    print(f"  tuned: kP={bg['kP']:.1f} kD={bg['kD']:.2f}  cost={res['best_cost']:.3f}  "
          f"(OS {bm['overshoot_pct']:.1f}%, sse {bm['steady_state_error_pct']:.1f}%)")
    ok = res["best_cost"] < seed_cost
    print("\nRESULT:", "PASS ✅ — AlphaHarness auto-tuned a live closed loop over NT"
          if ok else "FAIL ❌")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
