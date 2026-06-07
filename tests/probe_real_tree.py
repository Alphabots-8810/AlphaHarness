"""Probe AlphaHarness against the REAL 8810 robot code running in sim.

Proves the harness discovers the actual AdvantageKit NT topology (real key names,
real /Tuning structure) — not just the synthetic sim_robot. Run after launching
`./gradlew simulateJava` on the AlphaHarness branch (which adds the scope-a shim's
/Tuning/Shooter/* tunables + the /AdvantageKit/RealOutputs/Shooter/SetpointRPS output).
"""
import sys
import time

from alphaharness.nt_client import NTClient


def main():
    c = NTClient("AlphaHarness-probe")
    # retry connect: sim boot timing is uncertain
    for attempt in range(6):
        try:
            info = c.connect(server="127.0.0.1", timeout=10.0)
            print("connected:", info)
            break
        except Exception as e:
            print(f"  connect attempt {attempt+1} failed: {e}")
            time.sleep(3)
    else:
        print("RESULT: FAIL ❌ (could not connect to sim)")
        sys.exit(1)

    time.sleep(1.0)  # let more announcements land
    shooter = c.list_signals(prefix="/AdvantageKit/RealOutputs/Shooter")
    tuning = c.list_signals(prefix="/Tuning/Shooter")
    print(f"\n/AdvantageKit/RealOutputs/Shooter/* ({len(shooter)}):")
    for s in shooter:
        print("   ", s["name"], f"({s['type']})")
    print(f"\n/Tuning/Shooter/* ({len(tuning)}):")
    for s in tuning:
        print("   ", s["name"], f"({s['type']})")

    sp = c.read_signal("/AdvantageKit/RealOutputs/Shooter/SetpointRPS", 0.6)
    print("\nSetpointRPS read:", sp)
    c.disconnect()

    names = {s["name"] for s in shooter} | {s["name"] for s in tuning}
    need = {
        "/AdvantageKit/RealOutputs/Shooter/SetpointRPS",     # the shim's new dense setpoint output
        "/Tuning/Shooter/kP", "/Tuning/Shooter/kD",          # the shim's gain tunables
    }
    missing = need - names
    if missing:
        print("\nRESULT: FAIL ❌ missing:", missing)
        sys.exit(1)
    print("\nRESULT: PASS ✅ — AlphaHarness discovered the real robot's NT tree + scope-a shim topics")


if __name__ == "__main__":
    main()
