"""AlphaHarness — read-only NT4 client + the capture seam.

Connects as its own NT4 client (the same stream AdvantageScope reads), discovers
topics, and captures time-aligned trajectories for the metric layer. READ-ONLY:
there is no gain write-back here — that's scope (a), deliberately out of v0.

Capture honours advisor seam #1:
  * measurement (dense) and setpoint (sparse edge) are subscribed with
    sendAll=True + periodic<=0.02 during the window, so 50 Hz transients aren't
    aliased by the default 10 Hz / latest-only delivery.
  * cross-topic alignment uses NT4 *server* timestamps (one clock).
  * the step (t_step, target) is *resolved* here — provided (a dense setpoint
    output) or inferred (the sparse /Tuning edge) — so metrics.py stays agnostic.
"""
from __future__ import annotations

import time

import numpy as np
import ntcore

from .metrics import compute_step_response_metrics

_HOOD_HINT = ("hood", "motionmagic", "motion_magic")


def _looks_like_profile(*keys: str) -> bool:
    blob = " ".join(k.lower() for k in keys if k)
    return any(h in blob for h in _HOOD_HINT)


class NTClient:
    def __init__(self, identity: str = "AlphaHarness"):
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.identity = identity
        self._multi = None
        self.target_desc = None

    # ------------------------------------------------------------------ conn
    def connect(self, server: str | None = None, team: int | None = None,
                port: int | None = None, timeout: float = 6.0) -> dict:
        """Connect to a robot/sim NT4 server.

        server="127.0.0.1" for `simulateJava` or the synthetic robot;
        team=8810 for the real roboRIO (mDNS roborio-8810-frc.local).
        """
        self.inst.startClient4(self.identity)
        p = port or ntcore.NetworkTableInstance.kDefaultPort4
        if team is not None:
            self.inst.setServerTeam(team, p)
            self.target_desc = f"team {team}:{p}"
        else:
            self.inst.setServer(server or "127.0.0.1", p)
            self.target_desc = f"{server or '127.0.0.1'}:{p}"

        # discover all topics (topics-only = metadata, no values streamed)
        self._multi = ntcore.MultiSubscriber(
            self.inst, [""], ntcore.PubSubOptions(topicsOnly=True))

        # NT4 connect is async: wait for the link, THEN for announcements
        deadline = time.time() + timeout
        while time.time() < deadline and not self.inst.isConnected():
            time.sleep(0.05)
        if not self.inst.isConnected():
            raise ConnectionError(
                f"NT4 connect to {self.target_desc} timed out after {timeout}s "
                f"(is the robot/sim/synthetic running?)")
        time.sleep(0.4)  # let topic announcements arrive before list/capture
        return {"connected": True, "target": self.target_desc,
                "topics_known": len(self.inst.getTopicInfo())}

    def disconnect(self):
        if self._multi is not None:
            self._multi.close()
            self._multi = None
        self.inst.stopClient()

    def connected(self) -> bool:
        return self.inst.isConnected()

    # ------------------------------------------------------------------ list
    def list_signals(self, prefix: str = "", limit: int | None = None) -> list[dict]:
        infos = self.inst.getTopicInfo()
        rows = [{"name": ti.name, "type": ti.type_str}
                for ti in infos if ti.name.startswith(prefix)]
        rows.sort(key=lambda r: r["name"])
        return rows[:limit] if limit else rows

    def snapshot(self, key: str, settle: float = 0.3, default: float = float("nan")) -> float:
        """Return the current value of a double topic (retained value)."""
        sub = self.inst.getDoubleTopic(key).subscribe(default)
        time.sleep(settle)
        v = sub.get()
        sub.close()
        return float(v)

    # ------------------------------------------------------------------ read
    def read_signal(self, key: str, window_s: float = 1.0) -> dict:
        opts = ntcore.PubSubOptions(
            sendAll=True, periodic=0.02, keepDuplicates=True,
            pollStorage=int(window_s * 200) + 500)
        sub = self.inst.getDoubleTopic(key).subscribe(float("nan"), opts)
        sub.readQueue()                       # drain stale
        time.sleep(window_s)
        q = sub.readQueue()
        sub.close()
        vals = np.array([v.value for v in q], dtype=float)
        if vals.size == 0:
            return {"key": key, "samples": 0,
                    "note": "no samples (wrong key? not a double? not publishing?)"}
        ts = np.array([(v.serverTime or v.time) for v in q], dtype=float) / 1e6
        rate = float((vals.size - 1) / (ts[-1] - ts[0])) if ts[-1] > ts[0] else None
        return {"key": key, "samples": int(vals.size), "rate_hz": rate,
                "last": float(vals[-1]), "mean": float(vals.mean()),
                "min": float(vals.min()), "max": float(vals.max()),
                "std": float(vals.std())}

    # --------------------------------------------------------------- capture
    def _capture_raw(self, keys: list[str], duration_s: float, hi_rate: bool):
        opts = ntcore.PubSubOptions(
            sendAll=True, periodic=0.02 if hi_rate else 0.1, keepDuplicates=True,
            pollStorage=int(duration_s * 200) + 500)   # keep the WHOLE window, not last-N
        subs = {k: self.inst.getDoubleTopic(k).subscribe(float("nan"), opts)
                for k in keys}
        for s in subs.values():
            s.readQueue()                     # drain stale
        time.sleep(duration_s)
        out = {}
        for k, s in subs.items():
            q = s.readQueue()
            out[k] = [((v.serverTime or v.time) / 1e6, v.value) for v in q]
            s.close()
        return out

    @staticmethod
    def _resolve_step(sp_samples, t_meas, y_meas, target):
        """Return (t_step, target, mode). sp_samples = [(t, value), ...]."""
        # 1) explicit target provided -> just locate the step instant
        if target is not None:
            t_step = NTClient._edge_time(sp_samples)
            if t_step is None:
                t_step = NTClient._onset_from_measurement(t_meas, y_meas)
            return float(t_step), float(target), "provided"

        # 2) infer from a sparse setpoint edge
        if sp_samples:
            edge_t = NTClient._edge_time(sp_samples)
            if edge_t is not None:
                # target = setpoint value at/after the edge
                tgt = next(v for t, v in sp_samples if t >= edge_t)
                return float(edge_t), float(tgt), "inferred_edge"
            # no edge in window: setpoint held constant -> use it as target,
            # infer the instant from the measurement's own onset
            const_target = sp_samples[-1][1]
            t_step = NTClient._onset_from_measurement(t_meas, y_meas)
            return float(t_step), float(const_target), "inferred_constant"

        raise ValueError("no setpoint samples and no explicit target: cannot "
                         "resolve the step. Pass target=, or subscribe a setpoint key.")

    @staticmethod
    def _edge_time(sp_samples):
        """First time the setpoint value changes (the sparse edge)."""
        if not sp_samples:
            return None
        first = sp_samples[0][1]
        for t, v in sp_samples:
            if abs(v - first) > 1e-9:
                return t
        return None

    @staticmethod
    def _onset_from_measurement(t_meas, y_meas):
        """Fallback: detect step onset from when the measurement starts moving."""
        y = np.asarray(y_meas, float)
        t = np.asarray(t_meas, float)
        y0 = np.median(y[: max(3, y.size // 20)])
        rng = max(np.ptp(y), 1e-6)
        moved = np.where(np.abs(y - y0) > 0.1 * rng)[0]
        return float(t[moved[0]]) if moved.size else float(t[0])

    def capture_step_response(self, measurement_key: str, setpoint_key: str | None,
                              duration_s: float = 4.0, current_key: str | None = None,
                              target: float | None = None, current_limit: float | None = None,
                              hi_rate: bool = True, allow_profile: bool = False) -> dict:
        """Capture a velocity/position STEP and return scalar metrics.

        Guard: refuses hood / MotionMagic keys unless allow_profile=True — that's
        profile-following, not a step (use a profile-error metric, not this).
        """
        if not allow_profile and _looks_like_profile(measurement_key, setpoint_key or ""):
            raise ValueError(
                f"'{measurement_key}' looks like a MotionMagic / hood signal. "
                "That's profile-FOLLOWING, not a step response — scoring it like a "
                "flywheel is the classic mistake. Pass allow_profile=True only if "
                "you really mean to, or use the (not-yet-built) profile-error mode.")

        keys = [measurement_key]
        if setpoint_key:
            keys.append(setpoint_key)
        if current_key:
            keys.append(current_key)
        raw = self._capture_raw(keys, duration_s, hi_rate)

        meas = raw[measurement_key]
        if len(meas) < 3:
            raise ValueError(
                f"only {len(meas)} samples on '{measurement_key}' — wrong key, not "
                "a double, or nothing publishing. Check list_signals().")
        t_meas = np.array([t for t, _ in meas], float)
        y_meas = np.array([v for _, v in meas], float)

        sp = raw.get(setpoint_key, []) if setpoint_key else []
        t_step, resolved_target, mode = self._resolve_step(sp, t_meas, y_meas, target)

        t_cur = i_cur = None
        if current_key and raw.get(current_key):
            cur = raw[current_key]
            t_cur = np.array([t for t, _ in cur], float)
            i_cur = np.array([v for _, v in cur], float)

        m = compute_step_response_metrics(
            t_meas, y_meas, t_step, resolved_target,
            t_cur=t_cur, i_cur=i_cur, current_limit=current_limit)
        m["_step_source"] = mode
        m["_measurement_key"] = measurement_key
        m["_setpoint_key"] = setpoint_key
        return m
