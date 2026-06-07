"""AlphaHarness — offline WPILOG arm (scope b).

The SAME metric layer + step-resolution as the live NT path, pointed at post-match
`.wpilog` files (8810 already writes them via WPILOGWriter). No robot, no live
connection, no safety surface — read a log and tell the user what the gains did.

Reuses `metrics.compute_step_response_metrics` and `NTClient._resolve_step` so the
offline and live arms can never diverge in how they score a step.
"""
from __future__ import annotations

import numpy as np
from wpiutil.log import DataLogReader

from .metrics import compute_step_response_metrics
from .nt_client import NTClient, _looks_like_profile

_NUMERIC = {"double", "float", "int64", "boolean"}


def _get_value(record, typ: str) -> float:
    if typ == "double":
        return record.getDouble()
    if typ == "float":
        return record.getFloat()
    if typ == "int64":
        return float(record.getInteger())
    if typ == "boolean":
        return 1.0 if record.getBoolean() else 0.0
    raise ValueError(f"non-numeric type {typ}")


def list_entries(path: str) -> list[dict]:
    """List all entries (name + type) in a .wpilog."""
    entries = {}
    for rec in DataLogReader(path):
        if rec.isStart():
            sd = rec.getStartData()
            entries[sd.entry] = {"name": sd.name, "type": sd.type}
    return sorted(entries.values(), key=lambda r: r["name"])


def extract_signals(path: str, keys) -> dict:
    """Single forward pass: return {key: (t_seconds[], values[])} for numeric keys."""
    want = set(keys)
    id_meta = {}                     # entry_id -> (name, type)
    out = {k: ([], []) for k in keys}
    for rec in DataLogReader(path):
        if rec.isStart():
            sd = rec.getStartData()
            if sd.name in want:
                id_meta[sd.entry] = (sd.name, sd.type)
        elif rec.isFinish() or rec.isControl():
            continue
        else:
            meta = id_meta.get(rec.getEntry())
            if meta is None:
                continue
            name, typ = meta
            if typ not in _NUMERIC:
                continue
            try:
                val = _get_value(rec, typ)
            except Exception:
                continue
            out[name][0].append(rec.getTimestamp() / 1e6)
            out[name][1].append(val)
    return {k: (np.asarray(ts, float), np.asarray(vs, float)) for k, (ts, vs) in out.items()}


def analyze_step(path: str, measurement_key: str, setpoint_key: str | None = None,
                 target: float | None = None, current_key: str | None = None,
                 current_limit: float | None = None, allow_profile: bool = False) -> dict:
    """Analyze a step response stored in a .wpilog. Same contract as the live tool.

    Refuses hood / MotionMagic keys (profile-following, not a step) unless
    allow_profile=True.
    """
    if not allow_profile and _looks_like_profile(measurement_key, setpoint_key or ""):
        raise ValueError(
            f"'{measurement_key}' looks like a hood / MotionMagic signal — that's "
            "profile-following, not a step response. Pass allow_profile=True only if "
            "you really mean to.")

    keys = [measurement_key]
    if setpoint_key:
        keys.append(setpoint_key)
    if current_key:
        keys.append(current_key)
    sig = extract_signals(path, keys)

    t_meas, y_meas = sig[measurement_key]
    if t_meas.size < 3:
        raise ValueError(
            f"only {t_meas.size} samples on '{measurement_key}' in {path} — wrong "
            "key or not a numeric entry. Use list_entries() to see what's logged.")

    sp_samples = []
    if setpoint_key and sig[setpoint_key][0].size:
        sp_samples = [(float(t), float(v)) for t, v in zip(*sig[setpoint_key])]
    t_step, resolved_target, mode = NTClient._resolve_step(sp_samples, t_meas, y_meas, target)

    t_cur = i_cur = None
    if current_key and sig.get(current_key) and sig[current_key][0].size:
        t_cur, i_cur = sig[current_key]

    m = compute_step_response_metrics(
        t_meas, y_meas, t_step, resolved_target,
        t_cur=t_cur, i_cur=i_cur, current_limit=current_limit)
    m["_step_source"] = mode
    m["_measurement_key"] = measurement_key
    m["_source"] = f"wpilog:{path}"
    return m
