"""AlphaHarness — step-response metrics (pure, numpy-only).

Given a *dense* measurement trajectory and a *sparse* step event, compute the
scalar control-tuning metrics an LLM agent reasons over. No NT, no I/O here.

CONTRACT (locked 2026-06-07 — advisor seam #1):
    measurement = dense trajectory -> (t_meas[s], y_meas[])     # ~50 Hz, noisy, quantized
    step        = sparse event     -> (t_step[s], target)       # the setpoint edge

This mirrors the real NT topology: AdvantageKit outputs republish ~every robot
loop (dense), while the /Tuning setpoint updates on-change (a sparse edge).
`metrics.py` is agnostic to whether `target` was *provided* (a dense setpoint
output) or *inferred* (from the sparse /Tuning edge) — the caller resolves that.

Design decisions:
  * Settle bands are relative to the final TARGET (guarded when |target|~0,
    where we fall back to |step_size|). Never relative to the initial value.
  * Direction-agnostic: 0->60 and 60->0 are handled identically via step sign.
  * Robust to wire reality: peak/overshoot/settle run on a lightly smoothed
    copy so a single noisy sample can't fake an overshoot or delay settling;
    steady-state error is computed on RAW tail samples.
  * Only valid for a *step* response (shooter velocity). NOT for MotionMagic
    profile-following (the hood) — that needs a separate profile-error metric.
"""
from __future__ import annotations

import numpy as np

EPS = 1e-9


def _smooth(y: np.ndarray, w: int) -> np.ndarray:
    """Edge-preserving median filter (spike-reject WITHOUT trimming the peak).

    A mean moving-average attenuates a narrow overshoot hump — systematically
    *under*-reporting overshoot, the one error direction that's unsafe (an agent
    would think the loop is better-damped than it is). A median filter rejects
    single-sample spikes while preserving the peak height. Edge-padded so the
    endpoints stay honest (a zero-padded mean made the tail look "never settled").
    w is forced odd for symmetry.
    """
    if w <= 1 or y.size < w:
        return y
    if w % 2 == 0:
        w += 1
    pad = w // 2
    yp = np.pad(y, pad, mode="edge")
    win = np.lib.stride_tricks.sliding_window_view(yp, w)   # (n, w)
    return np.median(win, axis=1)


def _parabolic_peak(t: np.ndarray, y: np.ndarray, idx: int):
    """Sub-sample peak (value, time) via 3-point parabolic fit around idx.

    Removes the sample-rate dependence of taking the raw max: the true peak of
    a smooth hump rarely lands exactly on a sample.
    """
    if idx <= 0 or idx >= y.size - 1:
        return float(y[idx]), float(t[idx])
    ym, y0, yp = y[idx - 1], y[idx], y[idx + 1]
    denom = ym - 2 * y0 + yp
    if abs(denom) < EPS:
        return float(y0), float(t[idx])
    p = 0.5 * (ym - yp) / denom                 # in [-0.5, 0.5]
    peak_val = y0 - 0.25 * (ym - yp) * p
    dt_local = t[idx + 1] - t[idx]
    return float(peak_val), float(t[idx] + p * dt_local)


_MIN_OVERSHOOT_PCT = 1.0   # below this we call it over/critically damped


def _damping_from_peak(overshoot_pct: float, peak_time: float) -> dict:
    """Derive zeta / damped / natural freq from the (already accurate) primary peak.

    For a 2nd-order step the overshoot and the peak time pin the whole system:
        Mp = exp(-zeta*pi/sqrt(1-zeta^2))   ->   zeta = -ln(Mp)/sqrt(pi^2+ln^2(Mp))
        t_peak = pi/wd                       ->   wd = pi/t_peak,  wn = wd/sqrt(1-zeta^2)
    This couples damping to the median+parabolic peak estimate (robust to noise),
    instead of log-decrement on noisy secondary peaks which a median filter's
    plateaus make unreliable.

    HONESTY LABELS (read before trusting these for tuning):
    * DERIVED, NOT MEASURED. zeta/wd/wn are algebra on overshoot_pct + peak_time,
      not independent observations. If overshoot matches truth, zeta matches *by
      construction* — do NOT count overshoot and zeta as two separate confirmations.
    * 2ND-ORDER-MODEL ESTIMATES. The formulas assume a 2nd-order plant. A real FRC
      shooter is feedforward-dominant and not 2nd-order, so zeta/wn may be
      physically meaningless there. The MODEL-FREE metrics (overshoot %, settle,
      SSE, peak current) stay valid regardless; treat zeta/wn as a heuristic shape
      descriptor, not ground truth.
    """
    out = {"damping_ratio": None, "damped_freq_hz": None,
           "natural_freq_hz": None, "regime": "overdamped"}
    if overshoot_pct < _MIN_OVERSHOOT_PCT or peak_time <= EPS:
        return out

    Mp = overshoot_pct / 100.0
    lnMp = np.log(Mp)
    zeta = float(-lnMp / np.sqrt(np.pi**2 + lnMp**2))
    wd = np.pi / peak_time
    out["regime"] = "underdamped"
    out["damping_ratio"] = zeta
    out["damped_freq_hz"] = float(wd / (2 * np.pi))
    if zeta < 1:
        wn = wd / np.sqrt(1 - zeta**2)
        out["natural_freq_hz"] = float(wn / (2 * np.pi))
    return out


def compute_step_response_metrics(
    t_meas,
    y_meas,
    t_step: float,
    target: float,
    *,
    y0: float | None = None,
    t_cur=None,
    i_cur=None,
    current_limit: float | None = None,
    settle_bands=(0.02, 0.05),
    smooth_window: int = 5,
) -> dict:
    """Compute step-response metrics for a velocity/position step.

    Parameters
    ----------
    t_meas, y_meas : array-like
        Dense measurement trajectory in seconds and engineering units.
    t_step : float
        Time (same clock as t_meas) the setpoint stepped to `target`.
    target : float
        New setpoint value after the step (provided or inferred upstream).
    y0 : float, optional
        Pre-step steady value. If None, inferred from samples with t < t_step
        (falling back to the first post-step sample).
    t_cur, i_cur : array-like, optional
        Stator/supply current trajectory for peak-current + saturation flag.
    current_limit : float, optional
        If given, `saturated` is True when |i| reaches this limit.
    settle_bands : tuple
        Fractional settle bands relative to |target| (guarded to |step| if ~0).
    smooth_window : int
        Moving-average window (samples) used for peak/settle robustness.

    Returns
    -------
    dict  (all JSON-serializable scalars; None where undefined)
    """
    t = np.asarray(t_meas, dtype=float)
    y = np.asarray(y_meas, dtype=float)
    if t.size != y.size:
        raise ValueError(f"t_meas/y_meas length mismatch: {t.size} vs {y.size}")
    if t.size < 3:
        raise ValueError("need >=3 measurement samples")

    order = np.argsort(t)
    t, y = t[order], y[order]

    pre = t < t_step
    post = ~pre
    if post.sum() < 3:
        raise ValueError("need >=3 post-step samples; widen the capture window "
                         "or check t_step")

    if y0 is None:
        y0 = float(np.median(y[pre])) if pre.sum() >= 1 else float(y[post][0])

    tp_arr = t[post] - t_step          # time since step
    yp = y[post]
    yp_s = _smooth(yp, smooth_window)      # wider median: rise + settle
    yp_pk = _smooth(yp, 3)                 # narrow median: peak (less trim, spike-reject)

    step_size = float(target - y0)
    step_sign = 1.0 if step_size >= 0 else -1.0
    abs_step = abs(step_size)

    # --- settle reference: relative to final target, guarded near zero ---
    ref = abs(target) if abs(target) > EPS else abs_step
    ref = ref if ref > EPS else 1.0

    # --- normalized response r: 0 at y0, 1 at target (direction-agnostic) ---
    if abs_step > EPS:
        r = (yp_s - y0) / step_size
    else:
        r = np.zeros_like(yp_s)

    # --- rise time 10% -> 90% ---
    rise_time = None
    if abs_step > EPS:
        def _cross(frac):
            hit = np.where(r >= frac)[0]
            return tp_arr[hit[0]] if hit.size else None
        t10, t90 = _cross(0.10), _cross(0.90)
        if t10 is not None and t90 is not None and t90 >= t10:
            rise_time = float(t90 - t10)

    # --- peak / overshoot (median-smoothed + parabolic, direction-aware) ---
    # search the response in the step direction so 60->0 undershoot is a "peak"
    if abs_step > EPS:
        signed = (yp_pk - y0) * step_sign
        peak_idx = int(np.argmax(signed))
        peak_value, peak_time = _parabolic_peak(tp_arr, yp_pk, peak_idx)
        # overshoot relative to the STEP magnitude, measured past target
        overshoot_pct = float(max(0.0, (peak_value - target) * step_sign) / abs_step * 100.0)
    else:
        peak_idx = 0
        peak_value = float(yp_s[0])
        peak_time = float(tp_arr[0])
        overshoot_pct = 0.0

    # --- settle time per band (smoothed, relative to target) ---
    settle = {}
    for band in settle_bands:
        tol = band * ref
        outside = np.abs(yp_s - target) > tol
        if outside.any():
            last_out = np.where(outside)[0][-1]
            settle_t = float(tp_arr[last_out + 1]) if last_out + 1 < tp_arr.size else None
        else:
            settle_t = float(tp_arr[0])
        settle[f"settle_time_{int(band*100)}pct"] = settle_t

    # --- steady-state error on RAW tail (last 20% of post window, >=5 samples) ---
    n_tail = max(5, int(0.2 * yp.size))
    tail = yp[-n_tail:]
    ss_value = float(np.mean(tail))
    ss_error = float(ss_value - target)
    ss_error_pct = float(ss_error / ref * 100.0)
    ss_std = float(np.std(tail))

    # --- oscillation / damping (derived from the accurate primary peak) ---
    damping = _damping_from_peak(overshoot_pct, peak_time)

    # --- current / saturation (optional) ---
    cur = {"peak_current": None, "saturated": None, "saturation_fraction": None}
    if i_cur is not None:
        ic = np.asarray(i_cur, dtype=float)
        cur["peak_current"] = float(np.max(np.abs(ic)))
        if current_limit is not None:
            sat = np.abs(ic) >= (current_limit - EPS)
            cur["saturated"] = bool(sat.any())
            cur["saturation_fraction"] = float(sat.mean())

    dt = np.diff(t)
    sample_rate = float(1.0 / np.median(dt)) if dt.size and np.median(dt) > EPS else None

    result = {
        "target": float(target),
        "y0": float(y0),
        "step_size": step_size,
        "rise_time_s": rise_time,
        "peak_value": peak_value,
        "peak_time_s": peak_time,
        "overshoot_pct": overshoot_pct,
        "steady_state_value": ss_value,
        "steady_state_error": ss_error,
        "steady_state_error_pct": ss_error_pct,
        "steady_state_std": ss_std,
        "settle_ref": float(ref),
        **settle,
        **damping,
        **cur,
        "n_samples": int(yp.size),
        "sample_rate_hz": sample_rate,
        "capture_duration_s": float(tp_arr[-1]),
    }
    return result
