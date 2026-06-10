"""End-to-end through the MCP transport (what Claude actually drives).

Spawns the AlphaHarness MCP server over stdio, does the real MCP handshake,
lists tools, then connect + list_signals + capture_step_response against a
synthetic robot that must already be running on 127.0.0.1 (periodic mode).
Asserts the tool layer is well-formed end-to-end (structure, not exact values —
ground-truth precision is covered by e2e_sim.py).
"""
import asyncio
import json
import sys

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client


def _payload(result):
    """Extract the JSON dict a FastMCP tool returned."""
    # FastMCP returns structured content; fall back to text content.
    if getattr(result, "structuredContent", None):
        sc = result.structuredContent
        return sc.get("result", sc)
    return json.loads(result.content[0].text)


async def main():
    params = StdioServerParameters(command=sys.executable,
                                   args=["-m", "alphaharness.server"])
    async with stdio_client(params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()

            tools = await session.list_tools()
            names = sorted(t.name for t in tools.tools)
            print("tools:", names)
            assert {"connect", "status", "list_signals", "read_signal",
                    "capture_step_response"} <= set(names), names

            r = _payload(await session.call_tool("connect", {"server": "127.0.0.1"}))
            print("connect:", r)
            assert r["connected"] is True

            r = _payload(await session.call_tool(
                "list_signals", {"prefix": "/AdvantageKit"}))
            print("list_signals count:", r["count"])
            assert r["count"] >= 1

            r = _payload(await session.call_tool("capture_step_response", {
                "measurement_key": "/AdvantageKit/RealOutputs/Shooter/MeasuredRPS",
                "setpoint_key": "/Tuning/ShooterRPS",
                "duration_s": 5.0,
                "current_key": "/AdvantageKit/RealOutputs/Shooter/StatorCurrent",
                "current_limit": 60.0,
            }))
            print("capture keys:", sorted(k for k in r if not k.startswith("_")))
            for need in ("overshoot_pct", "settle_time_2pct", "steady_state_error",
                         "damping_ratio", "peak_current"):
                assert need in r, need
            print(f"  overshoot={r['overshoot_pct']:.1f}%  sse={r['steady_state_error']:.2f}"
                  f"  source={r['_step_source']}  n={r['n_samples']}")

            # the hood guard must fire — FastMCP surfaces a raised ValueError as an
            # isError result (not a client exception), so check the result.
            res = await session.call_tool("capture_step_response", {
                "measurement_key": "/AdvantageKit/RealOutputs/Hood/Angle",
                "setpoint_key": "/Tuning/HoodAngle", "duration_s": 0.2})
            err_text = (res.content[0].text if res.content else "").lower()
            guard_ok = bool(res.isError) and ("profile" in err_text or "motionmagic" in err_text)
            print("hood guard fired:", guard_ok, "->", err_text[:60])
            assert guard_ok

    print("\nMCP E2E: PASS ✅")


if __name__ == "__main__":
    asyncio.run(main())
