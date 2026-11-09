"""Clarke/Park transforms (amplitude-invariant) and small helpers."""
import math


def park(a: float, b: float, theta: float):
    """alpha-beta -> d-q at electrical angle theta."""
    c, s = math.cos(theta), math.sin(theta)
    return c * a + s * b, -s * a + c * b


def inv_park(d: float, q: float, theta: float):
    """d-q -> alpha-beta at electrical angle theta."""
    c, s = math.cos(theta), math.sin(theta)
    return c * d - s * q, s * d + c * q


def wrap(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def sat(x: float, limit: float) -> float:
    return max(-limit, min(limit, x))
