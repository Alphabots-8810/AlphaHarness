"""AlphaHarness — read-only NT4 telemetry + step-response metrics for FRC.

v0 scope (c): watch the robot, measure step responses, reason about gains.
Write-back / closed-loop auto-tune (scope a) is intentionally NOT here.
"""
from .metrics import compute_step_response_metrics
from .nt_client import NTClient

__all__ = ["compute_step_response_metrics", "NTClient"]
__version__ = "0.1.0"
