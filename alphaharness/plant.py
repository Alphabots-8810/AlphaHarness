"""AlphaHarness — closed-loop flywheel plant model (pure, numpy-only).

Unlike sim_robot.py (which PLAYS a fixed 2nd-order response), this is a real
closed-loop plant: a flywheel with electrical/actuation lag, driven by a velocity
PIDF controller whose gains are inputs. The step response therefore DEPENDS on the
gains — which is what makes an auto-tuner meaningful (write kP -> response changes).

Plant (2-state):  i' = (u - i)/tau_e         # actuation/electrical lag
                  w' = (kt*i - b*w)/J         # flywheel inertia + friction
Control (velocity PIDF, Phoenix6-ish, FF on setpoint):
                  u = kS*sign(sp) + kV*sp + kP*e + kI*∫e + kD*de/dt,  clamped to +-u_max

High kP + the lag => overshoot/oscillation; low kP (pure feedback, kV=0) => sluggish
with steady-state error. So there is a real settle-vs-overshoot-vs-SSE tradeoff to tune.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class FlywheelPlant:
    # Calibrated (kV=0, robot-ish) so the (kP, kD) cost surface is a clean bowl with an
    # interior optimum near kP~12, kD~0.1: low kP -> sluggish + steady-state error,
    # high kP -> overshoot/oscillation, kD damps. omega_max = kt*u_max/b = 120 >> 60.
    J: float = 0.12
    b: float = 1.5
    kt: float = 3.0
    tau_e: float = 0.05
    u_max: float = 60.0
    substeps: int = 12
    # state
    omega: float = 0.0
    i: float = 0.0
    integ: float = 0.0
    e_prev: float = 0.0

    def reset(self, omega: float = 0.0):
        self.omega = omega
        self.i = 0.0
        self.integ = 0.0
        self.e_prev = 0.0

    def step(self, dt: float, sp: float, kP: float, kI: float = 0.0,
             kD: float = 0.0, kS: float = 0.0, kV: float = 0.0):
        """Advance one control period; returns (omega, abs_current)."""
        e = sp - self.omega
        self.integ += e * dt
        self.integ = float(np.clip(self.integ, -self.u_max, self.u_max))  # anti-windup
        deriv = (e - self.e_prev) / dt
        self.e_prev = e
        u = kS * np.sign(sp) + kV * sp + kP * e + kI * self.integ + kD * deriv
        u = float(np.clip(u, -self.u_max, self.u_max))
        h = dt / self.substeps
        for _ in range(self.substeps):
            self.i += h * (u - self.i) / self.tau_e
            self.omega += h * (self.kt * self.i - self.b * self.omega) / self.J
        return self.omega, abs(self.i)


def simulate(gains: dict, target: float, *, warmup: float = 0.6, duration: float = 2.0,
             rate: float = 50.0, noise: float = 0.0, quant: float = 0.0, seed: int = 0,
             plant: FlywheelPlant | None = None):
    """Run a full 0->target step. Returns (t[], omega[], current[], t_step)."""
    rng = np.random.default_rng(seed)
    p = plant or FlywheelPlant()
    p.reset(0.0)
    dt = 1.0 / rate
    n = int((warmup + duration) * rate)
    t = np.arange(n) * dt
    t_step = warmup
    omega = np.empty(n)
    cur = np.empty(n)
    for k in range(n):
        sp = target if t[k] >= t_step else 0.0
        w, ia = p.step(dt, sp, gains.get("kP", 0.0), gains.get("kI", 0.0),
                       gains.get("kD", 0.0), gains.get("kS", 0.0), gains.get("kV", 0.0))
        wn = w + (rng.normal(0.0, noise) if noise else 0.0)
        if quant:
            wn = round(wn / quant) * quant
        omega[k] = wn
        cur[k] = ia
    return t, omega, cur, t_step
