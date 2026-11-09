"""Surface-mount PMSM plant, simulated in the true rotor d-q frame.

State x = [id, iq, wm, theta_e]
    did/dt  = (vd - Rs id + we Ls iq) / Ls
    diq/dt  = (vq - Rs iq - we Ls id - we psi_f) / Ls
    dwm/dt  = (Te - B wm - TL) / J,   Te = 1.5 p psi_f iq   (Ld = Lq)
    dthe/dt = p wm
The inverter is modelled by its average value: the commanded alpha-beta
voltage (after limiting) is applied for the whole control period.
"""
from .transforms import park, inv_park
from .params import MotorParams


class PMSM:
    def __init__(self, m: MotorParams):
        self.m = m
        self.id = 0.0
        self.iq = 0.0
        self.wm = 0.0
        self.theta = 0.0   # true electrical angle (unwrapped)

    def _deriv(self, x, va, vb, TL):
        m = self.m
        id_, iq, wm, th = x
        vd, vq = park(va, vb, th)
        we = m.p * wm
        did = (vd - m.Rs * id_ + we * m.Ls * iq) / m.Ls
        diq = (vq - m.Rs * iq - we * m.Ls * id_ - we * m.psi_f) / m.Ls
        Te = 1.5 * m.p * m.psi_f * iq
        dwm = (Te - m.B * wm - TL) / m.J
        return (did, diq, dwm, m.p * wm)

    def step(self, va: float, vb: float, TL: float, h: float):
        """Advance one RK4 step of length h with constant voltage and load."""
        x = (self.id, self.iq, self.wm, self.theta)
        k1 = self._deriv(x, va, vb, TL)
        k2 = self._deriv(tuple(xi + 0.5 * h * ki for xi, ki in zip(x, k1)), va, vb, TL)
        k3 = self._deriv(tuple(xi + 0.5 * h * ki for xi, ki in zip(x, k2)), va, vb, TL)
        k4 = self._deriv(tuple(xi + h * ki for xi, ki in zip(x, k3)), va, vb, TL)
        self.id, self.iq, self.wm, self.theta = (
            xi + h / 6.0 * (a + 2 * b + 2 * c + d) for xi, a, b, c, d in zip(x, k1, k2, k3, k4)
        )

    @property
    def torque(self) -> float:
        return 1.5 * self.m.p * self.m.psi_f * self.iq

    def i_alpha_beta(self):
        return inv_park(self.id, self.iq, self.theta)
