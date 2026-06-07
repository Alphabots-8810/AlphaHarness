# AlphaHarness 🅰️

**Read-only NT4 telemetry + step-response metrics for FRC, exposed to Claude over MCP.**

The third in Alphabots' `Alpha*` line: **AlphaSim** simulates the shot offline;
**AlphaHarness** watches the real robot online and measures how its loops respond —
the foundation for an agent that tunes them.

> **Scope:** **(c)** read-only metric core + **(b)** offline `.wpilog` arm + **(a)** a
> human-gated `set_gain` write. `set_gain` only takes effect when the robot firmware has
> `tuningMode=true` AND a human has enabled it (an LLM can't enable a robot; never on FMS).
> Validated against 8810's real robot code in sim (see [Validated on the real robot](#validated-on-the-real-robot)).

---

## The one architectural idea

**Claude does not read from AdvantageScope.** AdvantageScope is a read-only viewer
with no API. It and AlphaHarness are *siblings*: both connect to the same **NT4
server** (the roboRIO) and read the same stream. AlphaHarness's connection is just
the bidirectional one — NT4 is the only writable channel, which is what makes
closed-loop tuning possible later.

```
[robot code]              [NT4 = roboRIO/sim]          [AlphaHarness]        [Claude]
ShooterIO ---AdvantageKit--> /AdvantageKit/... (RO) --sub--> pyntcore client --MCP--> agent
 LoggedTunableNumber  <------ /Tuning/<key>  (RW)  <--(scope a, not v0)--   set_gain
```

The genuinely new piece — and the gap in every existing FRC NT/log tool — is the
**metric layer**: `capture_step_response` returns ~10 scalars (rise, overshoot %,
settle, steady-state error, damping ratio, peak current, saturation) instead of a
50 Hz waveform, so an LLM reasons over numbers, not a flood of samples.

---

## Quick start (no robot needed)

```bash
cd ~/Projects/AlphaHarness
source .venv/bin/activate

# 1) start the synthetic robot — a real NT4 server with a KNOWN 2nd-order response
alphaharness-sim --once --zeta 0.5 --wn 18 --target 60 --warmup 3 --noise 0.3

# 2) (in another shell) point the harness at it and measure
python -m tests.e2e_sim        # prints measured metrics vs closed-form ground truth
```

`alphaharness-sim` publishes the **same topology the real robot does** — a sparse,
on-change setpoint edge (`/Tuning/SHooterRPS`) plus a dense ~50 Hz measurement
(`/AdvantageKit/RealOutputs/Shooter/MeasuredRPS`) — so what passes here exercises the
exact ingestion path production uses.

### Offline arm (scope b — no robot, no live connection)

The same metric layer also reads post-match `.wpilog` files (which 8810 already writes):

```bash
alphaharness-simlog --zeta 0.45 --wn 20 --target 55 --out /tmp/shot.wpilog   # synth a log w/ ground truth
python -m pytest tests/test_wpilog.py -q                                      # validate the offline path
```

Then ask Claude: *"analyze the shooter step in `/path/to/match.wpilog`."* (MCP tools
`list_wpilog_signals` / `analyze_wpilog_step`).

## Wire it into Claude Code

`.mcp.json` is already in this repo. From the project dir, Claude Code picks up the
`alphaharness` MCP server automatically (run `/mcp` to confirm). Then ask Claude:

> *"Connect to 127.0.0.1, list the shooter signals, capture a step response on the
> shooter and tell me if kD looks low."*

Tools (9): `connect` · `status` · `list_signals` · `read_signal` · `capture_step_response`
· `set_gain` · `autotune_shooter` · `list_wpilog_signals` · `analyze_wpilog_step`.

## Point it at the real robot / WPILib sim

| Target | Command / call |
|---|---|
| Synthetic robot | `connect(server="127.0.0.1")` |
| `./gradlew simulateJava` | `connect(server="127.0.0.1")` |
| Real roboRIO | `connect(team=8810)` |

For the real robot, start the capture, then have a human command the step (e.g. set
`/Tuning/SHooterRPS` from AdvantageScope Tuning Mode). AlphaHarness sees the sparse
edge and the dense response. **Only one writer to `/Tuning` at a time** — don't let a
human and the harness fight over the same key.

---

## Grounding in 8810's code (`~/Downloads/8810_work/2026_8810_main`)

**Already there:** AdvantageKit logging (`LoggedRobot` + `NT4Publisher` + `WPILOGWriter`,
so telemetry is live on NT today), the 6328 `LoggedTunableNumber` wrapper with its
`/Tuning` prefix, Phoenix6 closed-loop config, a `SysIdRoutine` on the drive.

**Caveats this v0 already respects:**
- `ShooterIOSim` has **no physics model** → `simulateJava` won't produce a real step
  response. Hence `sim_robot.py` as a ground-truth substrate.
- The commanded setpoint is **not currently a logged output** → the harness infers the
  target from the sparse `/Tuning` edge (`_step_source="inferred_edge"`). One benign
  `Logger.recordOutput("Shooter/SetpointRPS", …)` line would let it read the target
  directly (`--dense-setpoint` on the sim mimics this).
- The shooter (`VelocityTorqueCurrentFOC`) and hood (`MotionMagicTorqueCurrentFOC`) are
  **torque-current domain**; a voltage-domain SysId FF won't drop in cleanly. Relevant
  when scope (a) seeds gains — flagged, not yet wired.
- The hood is **MotionMagic profile-following, not a step** → `capture_step_response`
  refuses hood/MotionMagic keys unless `allow_profile=True`.

---

## Tests

```bash
python -m pytest tests/ -v          # 20 unit tests: metrics + step-resolution + wpilog + autotune
python -m tests.e2e_sim             # full NT4 wire vs analytic ground truth (needs sim --once)
python -m tests.e2e_mcp             # MCP transport end-to-end (needs sim --period 4)
python -m tests.demo_autotune       # live auto-tune over NT (needs alphaharness-plant running)
```

Tests run on clean **and** noisy/quantized signals — a clean-curve-only test would pass
on a fiction the wire never delivers. `e2e_sim` reads the sim's ground truth **off NT**
(`/GroundTruth/*`), so the comparison can't drift from the actual sim params.

### What's validated — and the honest limits

- **The substrate is a perfect 2nd-order system.** The tests prove the metric layer
  recovers 2nd-order parameters from 2nd-order data (overshoot within ~1 pt of closed
  form, ζ within ~0.02). They say **nothing** about a real shooter, which is *not*
  2nd-order (feedforward-dominant, quantized encoder velocity, game-piece disturbance,
  motor nonlinearity). A synthetic 2nd-order sim can never catch that by construction.
- **Trust the model-free metrics on real data:** `overshoot_pct`, `rise_time`,
  `settle_time_*`, `steady_state_error`, `peak_current`, `saturated`.
- **`damping_ratio` / `damped_freq_hz` / `natural_freq_hz` are DERIVED**, not measured —
  algebra on overshoot + peak-time via the 2nd-order formula. ζ matching truth in the
  e2e is overshoot *restated*, **not** an independent third check. Treat them as a
  heuristic shape descriptor; they may be physically meaningless off a 2nd-order plant.
- **`capture_step_response` assumes a clean step:** mechanism near idle, exactly one
  setpoint change in the window. A spinning shooter or a double-tapped tunable will
  mis-resolve the step (first edge wins; `y0` inferred from pre-step samples).

---

## Roadmap

- **(c) — v0:** read-only NT-MCP + metric layer. ✅
- **(b) — offline WPILOG arm:** the SAME metric layer + step-resolution pointed at
  post-match `.wpilog` (8810 already writes them) via `wpiutil.log.DataLogReader`. ✅
  Tools `list_wpilog_signals` / `analyze_wpilog_step`; no robot, no live connection.
- **(a) — closed-loop, harness + robot halves built & demoed:**
  - Harness side: `set_gain` writes `/Tuning/*` (human-gated, see above). ✅
  - Robot side: the `LoggedTunableNumber → ifChanged → getConfigurator().apply()` re-config
    shim + a `tuningMode` flag on 8810's `AlphaHarness` git branch. ✅ (see below)
  - Demoed: AlphaHarness wrote `/Tuning/Shooter/kP` and the real robot **consumed it**
    (AdvantageKit mirror `/AdvantageKit/NetworkInputs/Tuning/Shooter/kP` updated). ✅
  - **The autonomous optimize loop** (`autotune_shooter`): perturb → measure → score →
    `set_gain` → repeat, coordinate pattern search over (kP, kD). ✅ Demoed live over NT
    against a closed-loop flywheel — tuned a sluggish kP=3 (14% SSE) down to kP=13/kD=0.2
    (0% overshoot, 5% SSE), cost −57%, in 18 NT evaluations. (`python -m tests.demo_autotune`)
  - **Not yet:** the Phoenix6 `apply()` itself only runs in REAL mode (sim uses
    `ShooterIOSim`, no TalonFX), and the autotuner is validated on a synthetic flywheel /
    the plant model — not a real shooter. Sim-first via Maple-Sim before any real-hardware
    closed loop; replay can't re-tune a feedback gain (it changes future inputs).

## The auto-tuner

```bash
alphaharness-plant            # a closed-loop flywheel NT server whose response depends on the gains
python -m tests.demo_autotune # AlphaHarness tunes it live over NT
```

`autotune.py` is a derivative-free coordinate pattern search minimizing
`cost = rise + 0.012·overshoot² + 0.05·|SSE%|` (always-defined metrics, so the surface stays
smooth even when settle-to-band is undefined). The evaluator is pluggable: the **same**
optimizer runs in-process against `plant.py` (fast tests) or over NT via `set_gain` +
`command_step_and_capture` (live). Bounded gains + an eval budget keep it out of unstable
territory; on real hardware it's human-gated (tuningMode + Test-mode enable, never FMS).

## Validated on the real robot

Beyond the synthetic substrate, AlphaHarness is validated against **8810's actual robot
code** (`~/Downloads/8810_work/2026_8810_main`, branch `AlphaHarness`) running in
`./gradlew simulateJava`:

- **Discovery:** connected and saw the real **367-topic** AdvantageKit tree, including the
  scope-a shim's `/AdvantageKit/RealOutputs/Shooter/SetpointRPS` + `/Tuning/Shooter/{kP..kV}`.
  (`python -m tests.probe_real_tree`)
- **Write→consume:** `set_gain("/Tuning/Shooter/kP", 9.0)` → robot read it → AdvantageKit
  recorded `9.0` at the mirror. (`python -m tests.probe_write_loop`)

The robot-side shim is 4 small edits (Constants `tuningMode`, an IO `setShooterPID` hook,
its Phoenix6 impl, and the gated `ifChanged` re-apply in `ShooterSubsystem.periodic`), all
dead when `tuningMode=false`. Review the diff: `git -C ~/Downloads/8810_work/2026_8810_main show AlphaHarness`.

## Safety (matters from scope a onward)

`isFMSAttached()` → refuse all tuning. Real robot is **always human-enabled, agent-advised**
(an LLM can't enable a robot). Full autonomy lives only in sim. Soft limits + stator-current
limits + motor-safety heartbeat are set in IO, not tuning logic, so they survive a hung loop.
