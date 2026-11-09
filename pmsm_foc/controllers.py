"""Discrete PI controllers for the current and speed loops."""
from .transforms import sat


class PI:
    """PI with output clamp and conditional-integration anti-windup."""

    def __init__(self, kp: float, ki: float, Ts: float, limit: float):
        self.kp, self.ki, self.Ts, self.limit = kp, ki, Ts, limit
        self.integ = 0.0

    def reset(self, value: float = 0.0):
        self.integ = value

    def update(self, error: float, ff: float = 0.0) -> float:
        u_unsat = self.kp * error + self.integ + ff
        u = sat(u_unsat, self.limit)
        # integrate only if not pushing further into saturation
        if u == u_unsat or (u_unsat > self.limit and error < 0) or (u_unsat < -self.limit and error > 0):
            self.integ += self.ki * error * self.Ts
        return u
