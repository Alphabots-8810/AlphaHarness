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
        self._gain_pubs = {}      # key -> publisher (kept alive so written values persist)

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
        for p in self._gain_pubs.values():
            p.close()
        self._gain_pubs = {}
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

    # ------------------------------------------------------------------ write
    def set_gain(self, key: str, value: float) -> dict:
        """Write a gain over NT (scope-a capability). RESTRICTED to /Tuning/* only.

        SAFETY: this only takes effect when the robot has Constants.tuningMode=true AND is
        enabled by a HUMAN in Test mode. An LLM cannot enable a robot. Never call this on a robot
        that is FMS-attached. Last-writer-wins on the topic — don't fight a human editing /Tuning
        in AdvantageScope at the same time.
        """
        if not key.startswith("/Tuning/"):
            raise ValueError(
                f"AlphaHarness only writes under /Tuning/* (the gain channel); refusing '{key}'. "
                "Writing arbitrary robot state is out of scope.")
        pub = self._gain_pubs.get(key)
        if pub is None:
            pub = self.inst.getDoubleTopic(key).publish()
            self._gain_pubs[key] = pub
        pub.set(float(value))
        self.inst.flush()
        return {"key": key, "value": float(value), "note": "takes effect only if robot tuningMode=true + human-enabled"}

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
            if t_step is None:                         # target known but no edge/movement
                t_step = float(np.asarray(t_meas, float)[0])
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
            if t_step is None:
                # no edge AND the measurement never moved -> no real step (idle window).
                # Refuse rather than fabricate confident metrics from noise.
                raise ValueError(
                    "no setpoint edge in the window and the measurement never moved — "
                    "no step to analyze (idle window / step missed). Start the capture "
                    "before commanding the step, or pass target= explicitly.")
            return float(t_step), float(const_target), "inferred_constant"

        raise ValueError("no setpoint samples and no explicit target: cannot "
                         "resolve the step. Pass target=, or subscribe a setpoint key.")

    def command_step_and_capture(self, measurement_key: str, setpoint_key: str, target: float,
                                 duration_s: float = 2.2, current_key: str | None = None,
                                 current_limit: float | None = None, pre_zero_s: float = 0.7,
                                 hi_rate: bool = True) -> dict:
        """Active capture for closed-loop tuning: AlphaHarness ITSELF commands the step.

        Zeroes the setpoint and lets it settle, then subscribes, commands the step, and
        captures the response — so the optimizer controls perturbation timing. target is
        provided; t_step comes from the measurement onset (pre-zeroed to ~0).
        """
        self.set_gain(setpoint_key, 0.0)
        time.sleep(pre_zero_s)
        opts = ntcore.PubSubOptions(sendAll=True, periodic=0.02 if hi_rate else 0.1,
                                    keepDuplicates=True, pollStorage=int(duration_s * 200) + 500)
        msub = self.inst.getDoubleTopic(measurement_key).subscribe(float("nan"), opts)
        csub = (self.inst.getDoubleTopic(current_key).subscribe(float("nan"), opts)
                if current_key else None)
        msub.readQueue()
        if csub:
            csub.readQueue()
        self.set_gain(setpoint_key, float(target))     # the step (AlphaHarness-commanded edge)
        time.sleep(duration_s)
        mq = msub.readQueue()
        msub.close()
        t_meas = np.array([(v.serverTime or v.time) / 1e6 for v in mq], float)
        y_meas = np.array([v.value for v in mq], float)
        t_cur = i_cur = None
        if csub:
            cq = csub.readQueue()
            csub.close()
            t_cur = np.array([(v.serverTime or v.time) / 1e6 for v in cq], float)
            i_cur = np.array([v.value for v in cq], float)
        if t_meas.size < 3:
            raise ValueError(f"only {t_meas.size} samples on '{measurement_key}'")
        t_step = self._onset_from_measurement(t_meas, y_meas)
        if t_step is None:                             # pre-zeroed then stepped, so it should move
            t_step = float(t_meas[0])
        m = compute_step_response_metrics(t_meas, y_meas, t_step, float(target),
                                          t_cur=t_cur, i_cur=i_cur, current_limit=current_limit)
        m["_step_source"] = "active_command"
        return m

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
        """Onset time of the step, or None if the measurement never significantly moved.

        Noise is estimated robustly as median |consecutive diff| (immune to the rising edge
        itself — std of the first samples would over-estimate when the step starts at t0).
        A window whose whole excursion (ptp) is within ~10x that noise is treated as pure
        noise / no step, so an idle trace can't be mistaken for a step.
        """
        y = np.asarray(y_meas, float)
        t = np.asarray(t_meas, float)
        n0 = max(3, y.size // 20)
        y0 = float(np.median(y[:n0]))
        dif = np.abs(np.diff(y))
        noise = float(np.median(dif)) if dif.size else 0.0
        ptp = float(np.ptp(y))
        if ptp <= 10.0 * noise:                 # no real excursion beyond noise -> not a step
            return None
        thresh = max(0.1 * max(ptp, 1e-6), 6.0 * noise)
        moved = np.where(np.abs(y - y0) > thresh)[0]
        return float(t[moved[0]]) if moved.size else None

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
