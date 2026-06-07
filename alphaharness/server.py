"""AlphaHarness — read-only NT4 MCP server.

Exposes the robot's live telemetry to Claude as MCP tools. v0 is READ-ONLY:
Claude can watch and measure, but cannot write gains (that's scope (a)).

Tools:
    connect(server|team)            -- attach to a robot / sim / synthetic NT4 server
    status()                        -- connection + topic count
    list_signals(prefix)            -- discover NT topics
    read_signal(key, window_s)      -- quick stats on one signal
    capture_step_response(...)      -- the core: scalar step-response metrics

Run standalone (stdio):  python -m alphaharness.server
Or register in Claude Code via .mcp.json (see README).
"""
from __future__ import annotations

from mcp.server.fastmcp import FastMCP

from .nt_client import NTClient
from . import wpilog_reader

mcp = FastMCP("AlphaHarness")
_client = NTClient()


@mcp.tool()
def connect(server: str = "127.0.0.1", team: int | None = None,
            timeout: float = 6.0) -> dict:
    """Connect to a robot/sim NT4 server.

    Use server="127.0.0.1" for `./gradlew simulateJava` or the synthetic robot.
    Use team=8810 for the real roboRIO. (team overrides server when given.)
    """
    if team is not None:
        return _client.connect(team=team, timeout=timeout)
    return _client.connect(server=server, timeout=timeout)


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
def read_signal(key: str, window_s: float = 1.0) -> dict:
    """Sample one double signal for `window_s` seconds and return summary stats."""
    return _client.read_signal(key, window_s=window_s)


@mcp.tool()
def capture_step_response(measurement_key: str,
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
    return _client.capture_step_response(
        measurement_key=measurement_key, setpoint_key=setpoint_key,
        duration_s=duration_s, current_key=current_key, target=target,
        current_limit=current_limit, allow_profile=allow_profile)


# ----------------------------------------------------------------- offline (scope b)
@mcp.tool()
def list_wpilog_signals(path: str, prefix: str = "") -> dict:
    """List entries (name + type) in a post-match .wpilog file (offline, no robot)."""
    rows = [e for e in wpilog_reader.list_entries(path) if e["name"].startswith(prefix)]
    return {"count": len(rows), "path": path, "signals": rows}


@mcp.tool()
def analyze_wpilog_step(path: str, measurement_key: str,
                        setpoint_key: str | None = None, target: float | None = None,
                        current_key: str | None = None, current_limit: float | None = None,
                        allow_profile: bool = False) -> dict:
    """Analyze a step response stored in a .wpilog file — same metrics as the live
    tool, but offline on a log 8810 already writes. No robot, no live connection.

    Same field-trust rules and clean-step assumption as capture_step_response.
    """
    return wpilog_reader.analyze_step(
        path, measurement_key, setpoint_key=setpoint_key, target=target,
        current_key=current_key, current_limit=current_limit, allow_profile=allow_profile)


def main():
    mcp.run()


if __name__ == "__main__":
    main()
