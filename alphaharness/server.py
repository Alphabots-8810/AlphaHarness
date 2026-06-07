"""AlphaHarness — NT4 + WPILOG MCP server for FRC telemetry & PID tuning.

Exposes the robot to Claude as MCP tools. Scope:
  (c) read-only metrics  : connect, status, list_signals, read_signal, capture_step_response
  (a) write / closed loop : set_gain, autotune_shooter
                            (human-gated: robot tuningMode + Test-mode enable, never FMS)
  (b) offline .wpilog     : list_wpilog_signals, analyze_wpilog_step

Long-running tools (NT sleeps, file IO, the tune loop) are `async def` and offload the
blocking work to a worker thread via anyio.to_thread, so a single tool call can't freeze
the MCP event loop (mcp 1.27.x runs sync tool bodies on the loop thread).

Run standalone (stdio):  python -m alphaharness.server
Or register in Claude Code via .mcp.json (see README).
"""
from __future__ import annotations

import anyio
from mcp.server.fastmcp import FastMCP

from .nt_client import NTClient
from . import wpilog_reader

mcp = FastMCP("AlphaHarness")
_client = NTClient()


async def _off(fn):
    """Run a blocking callable in a worker thread (keeps the MCP loop responsive)."""
    return await anyio.to_thread.run_sync(fn)


@mcp.tool()
async def connect(server: str = "127.0.0.1", team: int | None = None,
                  timeout: float = 6.0) -> dict:
    """Connect to a robot/sim NT4 server.

    Use server="127.0.0.1" for `./gradlew simulateJava` or the synthetic robot.
    Use team=8810 for the real roboRIO. (team overrides server when given.)
    """
    if team is not None:
        return await _off(lambda: _client.connect(team=team, timeout=timeout))
    return await _off(lambda: _client.connect(server=server, timeout=timeout))


@mcp.tool()
def status() -> dict:
    """Report whether AlphaHarness is connected and how many topics it sees."""
    return {"connected": _client.connected(),
            "target": _client.target_desc,
            "topics_known": len(_client.inst.getTopicInfo()) if _client.target_desc else 0}


@mcp.tool()
def list_signals(prefix: str = "", limit: int = 200) -> dict:
    """List NT topics (optionally filtered by name prefix), with their types.

    Tip: prefix='/AdvantageKit/RealOutputs/Shooter' to find shooter signals,
    or '/Tuning' to find the writable tunables.
    """
    rows = _client.list_signals(prefix=prefix, limit=limit)
    return {"count": len(rows), "signals": rows}


@mcp.tool()
async def read_signal(key: str, window_s: float = 1.0) -> dict:
    """Sample one double signal for `window_s` seconds and return summary stats."""
    return await _off(lambda: _client.read_signal(key, window_s=window_s))


@mcp.tool()
async def capture_step_response(measurement_key: str,
                                setpoint_key: str | None = None,
                                duration_s: float = 4.0,
                                current_key: str | None = None,
                                target: float | None = None,
                                current_limit: float | None = None,
                                allow_profile: bool = False) -> dict:
    """Capture a velocity/position STEP and return scalar tuning metrics.

    Start this, then command the step (change the setpoint) within the window —
    AlphaHarness sees the sparse setpoint edge and the dense measurement
    trajectory, and returns: rise_time, overshoot_pct, settle_time (2%/5%),
    steady_state_error, damping_ratio, damped_freq, peak_current, saturated.

    Trust which fields:
      * MODEL-FREE (valid on any plant): overshoot_pct, rise_time, settle_time_*,
        steady_state_error, peak_current, saturated.
      * 2ND-ORDER-MODEL ESTIMATES (derived from overshoot+peak_time; a heuristic
        shape descriptor, NOT independent measurements, and may be meaningless if
        the mechanism isn't ~2nd-order): damping_ratio, damped_freq_hz, natural_freq_hz.

    Assumes a clean step: the mechanism is near idle and exactly ONE setpoint
    change happens in the window. A shooter already spinning, or a double-tap on
    the tunable, will mis-resolve the step (first edge wins; y0 from pre-step samples).

    measurement_key : dense ~50 Hz output, e.g. /AdvantageKit/RealOutputs/Shooter/MeasuredRPS
    setpoint_key    : the setpoint source (sparse /Tuning edge or a dense setpoint output).
                      Omit only if you pass `target` explicitly.
    target          : pass to force the target (else inferred from the setpoint).
    current_key     : optional stator-current signal for peak/saturation.

    Refuses hood / MotionMagic keys (profile-following != step) unless
    allow_profile=True.
    """
    return await _off(lambda: _client.capture_step_response(
        measurement_key=measurement_key, setpoint_key=setpoint_key,
        duration_s=duration_s, current_key=current_key, target=target,
        current_limit=current_limit, allow_profile=allow_profile))


# ----------------------------------------------------------------- write (scope a)
@mcp.tool()
def set_gain(key: str, value: float) -> dict:
    """Write a control gain over NT (scope-a). RESTRICTED to /Tuning/* keys only.

    SAFETY — read before use: this only takes effect when the robot firmware has
    tuningMode=true AND a HUMAN has enabled it in Test mode (an LLM cannot enable a
    robot). Do NOT use on an FMS-attached robot. Last-writer-wins per topic, so don't
    write a key a human is editing in AdvantageScope at the same time. Pair with
    capture_step_response to close the tune loop: read response -> propose gain -> set_gain -> re-measure.
    """
    return _client.set_gain(key, value)


def _run_autotune_shooter(target_rps, budget, seed_kP, seed_kD,
                          measurement_key, setpoint_key, current_key) -> dict:
    """Blocking body of autotune_shooter (run off-thread). Always leaves the robot on a
    known-good gain set with the setpoint zeroed, even if a capture fails mid-run."""
    from .autotune import autotune as _autotune

    def evaluate(g):
        _client.set_gain("/Tuning/Shooter/kP", g["kP"])
        _client.set_gain("/Tuning/Shooter/kD", g["kD"])
        return _client.command_step_and_capture(
            measurement_key, setpoint_key, target_rps, duration_s=2.2, current_key=current_key)

    res = None
    try:
        res = _autotune(evaluate, seed={"kP": seed_kP, "kD": seed_kD},
                        bounds={"kP": (0.5, 30.0), "kD": (0.0, 1.5)},
                        steps={"kP": 4.0, "kD": 0.2}, budget=budget)
        bm = res["best_metrics"]
        return {"best_gains": res["best_gains"], "best_cost": res["best_cost"], "evals": res["evals"],
                "best_metrics": {k: bm.get(k) for k in
                                 ("overshoot_pct", "rise_time_s", "settle_time_2pct", "steady_state_error_pct")},
                "history": res["history"]}
    finally:
        # Never strand the robot on aggressive trial gains while it's driving toward target:
        # restore the best (or seed) gains and zero the setpoint, on success OR error.
        good = res["best_gains"] if res else {"kP": seed_kP, "kD": seed_kD}
        try:
            _client.set_gain("/Tuning/Shooter/kP", good["kP"])
            _client.set_gain("/Tuning/Shooter/kD", good["kD"])
            _client.set_gain(setpoint_key, 0.0)
        except Exception:
            pass


@mcp.tool()
async def autotune_shooter(target_rps: float = 60.0, budget: int = 24,
                           seed_kP: float = 3.0, seed_kD: float = 0.0,
                           measurement_key: str = "/AdvantageKit/RealOutputs/Shooter/MeasuredRPS",
                           setpoint_key: str = "/Tuning/SHooterRPS",
                           current_key: str = "/AdvantageKit/RealOutputs/Shooter/StatorCurrent") -> dict:
    """Autonomously tune the shooter kP/kD: perturb → measure → score → set_gain → repeat.

    The closed loop AlphaHarness was built for. Requires connect() first and the robot in
    tuningMode (sim, or a human-enabled Test-mode real robot — never FMS). Always leaves the
    robot on the best gains it found with the setpoint zeroed (even if a capture fails), and
    returns the gains, cost, and per-evaluation history.
    """
    return await _off(lambda: _run_autotune_shooter(
        target_rps, budget, seed_kP, seed_kD, measurement_key, setpoint_key, current_key))


# ----------------------------------------------------------------- offline (scope b)
@mcp.tool()
async def list_wpilog_signals(path: str, prefix: str = "") -> dict:
    """List entries (name + type) in a post-match .wpilog file (offline, no robot)."""
    rows = await _off(lambda: [e for e in wpilog_reader.list_entries(path)
                               if e["name"].startswith(prefix)])
    return {"count": len(rows), "path": path, "signals": rows}


@mcp.tool()
async def analyze_wpilog_step(path: str, measurement_key: str,
                              setpoint_key: str | None = None, target: float | None = None,
                              current_key: str | None = None, current_limit: float | None = None,
                              allow_profile: bool = False) -> dict:
    """Analyze a step response stored in a .wpilog file — same metrics as the live
    tool, but offline on a log 8810 already writes. No robot, no live connection.

    Same field-trust rules and clean-step assumption as capture_step_response.
    """
    return await _off(lambda: wpilog_reader.analyze_step(
        path, measurement_key, setpoint_key=setpoint_key, target=target,
        current_key=current_key, current_limit=current_limit, allow_profile=allow_profile))


def main():
    mcp.run()


if __name__ == "__main__":
    main()
