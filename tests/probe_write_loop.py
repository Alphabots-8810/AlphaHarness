"""Demonstrate the scope-a WRITE loop against the real 8810 robot code in sim.

AlphaHarness writes /Tuning/Shooter/kP and confirms the ROBOT consumed it by reading
AdvantageKit's recorded mirror of that input (the robot republishes what it actually
read). Proves AlphaHarness -> NT -> robot-reads-it.

HONEST BOUNDARY: in SIM the wired IO is ShooterIOSim (no TalonFX), so the final
getConfigurator().apply() is a no-op — the Phoenix6 apply itself can only be exercised
on real hardware (REAL mode = ShooterIOPheonix6). This proves the write+consume half.
"""
import sys
import time

from alphaharness.nt_client import NTClient

GAIN = "/Tuning/Shooter/kP"
NEW = 9.0


def main():
    c = NTClient("AlphaHarness-write")
    for attempt in range(6):
        try:
            print("connected:", c.connect(server="127.0.0.1", timeout=10.0))
            break
        except Exception as e:
            print(f"  retry {attempt+1}: {e}"); time.sleep(3)
    else:
        print("RESULT: FAIL ❌ (no sim)"); sys.exit(1)
    time.sleep(1.0)

    before = c.snapshot(GAIN, settle=0.5)
    print(f"\nbefore: {GAIN} = {before}  (robot default should be 7.0)")

    # find AdvantageKit's recorded mirror of this tunable input
    mirrors = [s["name"] for s in c.list_signals(prefix="/AdvantageKit")
               if "Shooter/kP" in s["name"] or s["name"].endswith("Tuning/Shooter/kP")]
    print("AdvantageKit mirror candidates:", mirrors)

    print(f"\nAlphaHarness set_gain({GAIN}, {NEW}) ...")
    print("  ->", c.set_gain(GAIN, NEW))
    time.sleep(1.0)   # let the robot read it + AdvantageKit record it

    after = c.snapshot(GAIN, settle=0.5)
    print(f"after:  {GAIN} = {after}")
    mirror_vals = {m: c.snapshot(m, settle=0.4) for m in mirrors}
    print("mirror after:", mirror_vals)
    c.disconnect()

    ok = abs(after - NEW) < 1e-6
    consumed = any(abs(v - NEW) < 1e-6 for v in mirror_vals.values())
    print(f"\nwrite landed on NT: {ok}   robot consumed (mirror==new): {consumed}")
    if ok and consumed:
        print("RESULT: PASS ✅ — AlphaHarness wrote a gain and the real robot consumed it")
    elif ok:
        print("RESULT: PARTIAL ⚠️ — write landed on NT, but no AdvantageKit mirror confirmed "
              "consumption (mirror path may differ; the topic is robot-owned so the write reached it)")
    else:
        print("RESULT: FAIL ❌")
        sys.exit(1)


if __name__ == "__main__":
    main()
