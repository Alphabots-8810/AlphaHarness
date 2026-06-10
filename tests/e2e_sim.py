"""End-to-end: real NT4 wire from synthetic robot -> NTClient -> metrics.

Validates the WHOLE pipeline against ground truth, not just the math.

Ground truth is READ FROM NT (/GroundTruth/*, published by sim_robot) — NOT
hardcoded here — so the comparison can never drift from the sim's actual params.
Launch the sim with --once and ANY params; this test adapts.
"""
import sys

from alphaharness.nt_client import NTClient

MEAS = "/AdvantageKit/RealOutputs/Shooter/MeasuredRPS"
SETP = "/Tuning/ShooterRPS"
CUR = "/AdvantageKit/RealOutputs/Shooter/StatorCurrent"


def main():
    c = NTClient("AlphaHarness-e2e")
    print("connecting to 127.0.0.1 ...")
    info = c.connect(server="127.0.0.1", timeout=8.0)
    print("  ", info)

    # capture FIRST (don't burn the warmup window) — the step must fall inside it
    print("capturing step (5s, hi-rate)...")
    m = c.capture_step_response(MEAS, SETP, duration_s=5.0, current_key=CUR,
                                current_limit=60.0)

    # ground truth is retained on NT — read it AFTER the capture (no hardcoded ref)
    gt = {k: c.snapshot(f"/GroundTruth/{k}", settle=0.3)
          for k in ("overshoot_pct", "peak_time_s", "damped_freq_hz",
                    "natural_freq_hz", "settle_2pct_s", "zeta")}
    print("  ground truth (from NT):",
          {k: round(v, 3) for k, v in gt.items()})
    c.disconnect()

    print("\n--- MEASURED vs GROUND TRUTH (read live off NT) ---")
    # NOTE: damping_ratio / damped_freq are DERIVED from overshoot+peak_time via the
    # 2nd-order formula, so them matching is NOT an independent check — it's overshoot
    # restated. The real independent measurements are overshoot, peak_time, settle, sse.
    rows = [
        ("overshoot_pct", m["overshoot_pct"], gt["overshoot_pct"], 2.0),
        ("peak_time_s", m["peak_time_s"], gt["peak_time_s"], 0.05),
        ("damped_freq_hz*", m["damped_freq_hz"], gt["damped_freq_hz"], 0.6),
        ("damping_ratio*", m["damping_ratio"], gt["zeta"], 0.10),
        ("settle_time_2pct", m["settle_time_2pct"], gt["settle_2pct_s"], 0.5),
    ]
    ok = True
    for name, got, truth, tol in rows:
        if got is None:
            print(f"  {name:18s} got=None  truth={truth:.3f}  FAIL")
            ok = False
            continue
        d = abs(got - truth)
        flag = "ok" if d <= tol else "FAIL"
        if d > tol:
            ok = False
        print(f"  {name:18s} got={got:8.3f}  truth={truth:7.3f}  |d|={d:6.3f} (tol {tol})  {flag}")

    print(f"\n  step_source={m['_step_source']}  sample_rate_hz={m['sample_rate_hz']:.1f}"
          f"  n={m['n_samples']}  sse={m['steady_state_error']:.3f}"
          f"  peak_current={m['peak_current']:.1f}  saturated={m['saturated']}")
    print("\nRESULT:", "PASS ✅" if ok else "FAIL ❌")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
