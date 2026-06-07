"""AlphaHarness — NT4 telemetry + step-response metrics for FRC, over MCP.

Core (scope c): watch the robot, measure step responses, reason about gains (read-only).
Offline (scope b): the same metric layer over post-match .wpilog files.
Write (scope a): set_gain() writes /Tuning/* gains — but only takes effect when the
robot firmware has tuningMode=true AND a human has enabled it (an LLM can't enable a
robot, and never on FMS). The closed loop is human-gated by design.
"""
from .metrics import compute_step_response_metrics
from .nt_client import NTClient

__all__ = ["compute_step_response_metrics", "NTClient"]
__version__ = "0.1.0"
