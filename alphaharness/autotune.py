"""AlphaHarness — autonomous PID auto-tuner (scope a capstone).

The optimize loop: perturb (command a step) -> measure (step-response metrics) ->
score (cost) -> propose new gains -> apply -> repeat, until it converges.

Derivative-free coordinate pattern search (Hooke-Jeeves style) over (kP, kD):
robust to the noisy, occasionally-undefined metrics a real capture returns, and
transparent (you can read exactly why each move was taken). The evaluator is
pluggable — the SAME optimizer runs in-process against the plant model (fast tests)
or over NT against a live robot/sim (set_gain -> capture).

Bounds + a max-eval budget keep it from wandering into unstable gains. On real
hardware this is human-gated (tuningMode + Test-mode enable); see server.set_gain.
"""
from __future__ import annotations


def shooter_cost(m: dict) -> float:
    """Scalar cost for a shooter velocity step. Lower = better.

    rise rewards speed (-> higher kP); overshoot^2 penalizes ring (-> kD / lower kP);
    |SSE| penalizes not reaching target (-> higher kP). Uses always-defined metrics so
    the surface stays smooth even when settle-to-band is undefined (SSE present).
    """
    rise = m.get("rise_time_s") or 2.5
    overshoot = m.get("overshoot_pct") or 0.0
    sse = abs(m.get("steady_state_error_pct") or 0.0)
    # stability guard: a wild capture (huge overshoot / divergence) gets a big finite cost
    if overshoot > 150:
        return 50.0 + overshoot / 100.0
    return rise + 0.012 * overshoot * overshoot + 0.05 * sse


def _clip(v, lo, hi):
    return max(lo, min(hi, v))


def autotune(evaluate, *, seed, bounds, steps, cost=shooter_cost,
             budget: int = 28, shrink: float = 0.5, min_step_frac: float = 0.05,
             log=lambda s: None) -> dict:
    """Coordinate pattern search.

    evaluate(gains: dict) -> metrics dict   (gains carries the full set; we move kP/kD)
    seed    : {"kP":..,"kD":..,...} starting gains
    bounds  : {"kP":(lo,hi), "kD":(lo,hi)}
    steps   : {"kP":init_step, "kD":init_step}
    Returns best gains/metrics/cost + the full evaluation history.
    """
    dims = list(steps.keys())
    x = dict(seed)
    step = dict(steps)
    history = []

    def ev(g):
        m = evaluate(g)
        c = cost(m)
        history.append({"gains": {d: g[d] for d in dims}, "cost": c,
                        "overshoot_pct": m.get("overshoot_pct"),
                        "rise_time_s": m.get("rise_time_s"),
                        "settle_time_2pct": m.get("settle_time_2pct"),
                        "sse_pct": m.get("steady_state_error_pct")})
        return c, m

    best_cost, best_m = ev(x)
    log(f"seed {_fmt(x, dims)} -> cost {best_cost:.3f}")

    while len(history) < budget and any(
            step[d] > min_step_frac * (bounds[d][1] - bounds[d][0]) for d in dims):
        improved = False
        for d in dims:
            for sign in (+1, -1):
                if len(history) >= budget:
                    break
                trial = dict(x)
                trial[d] = _clip(x[d] + sign * step[d], *bounds[d])
                if trial[d] == x[d]:
                    continue
                c, m = ev(trial)
                if c < best_cost - 1e-6:
                    best_cost, best_m, x = c, m, trial
                    improved = True
                    log(f"  move {d}{'+' if sign>0 else '-'} -> {_fmt(x, dims)} cost {c:.3f}")
                    break
            if improved:
                break
        if not improved:
            for d in dims:
                step[d] *= shrink
            log(f"  no improve, shrink steps -> {_fmt(step, dims)}")

    return {"best_gains": {d: x[d] for d in dims}, "best_cost": best_cost,
            "best_metrics": best_m, "evals": len(history), "history": history}


def _fmt(g, dims):
    return "{" + ", ".join(f"{d}={g[d]:.3f}" for d in dims) + "}"


# --------------------------------------------------------- in-process evaluator
def plant_evaluator(target: float = 60.0, noise: float = 0.0, **plant_kw):
    """Fast evaluator against the flywheel plant model (no NT) — for tests/demos."""
    from .plant import simulate, FlywheelPlant
    from .metrics import compute_step_response_metrics

    def evaluate(gains: dict) -> dict:
        p = FlywheelPlant(**plant_kw) if plant_kw else FlywheelPlant()
        t, w, c, ts = simulate(gains, target, plant=p, duration=2.5, noise=noise)
        return compute_step_response_metrics(t, w, ts, target, y0=0.0, t_cur=t, i_cur=c)

    return evaluate
