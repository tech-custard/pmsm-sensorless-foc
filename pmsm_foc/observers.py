"""Rotor speed and position estimators.

Both estimators use only the measured stator currents (alpha-beta) and the
commanded stator voltages, never the rotor position sensor.

SMOEstimator
    Sliding-mode current observer in the stationary frame. The switching term
    z = k * sat(i_hat - i) is driven to equal the back-EMF; a first-order
    low-pass filter extracts it, and a PLL locks onto its angle. The PLL is
    normalised by the EMF magnitude, so its loop gain does not change with speed.

MRASEstimator
    Model-reference adaptive system in the estimated rotor frame. The motor is
    the reference model; an adjustable current model runs at the estimated
    speed. With i'_d = i_d + psi_f/Ls, the Popov-stable adaptation law is
        e = i'_d * i'_q,hat - i'_q * i'_d,hat
        w_hat = Kp e + Ki * integral(e)
    and the rotor angle is the integral of w_hat.
"""
import math
from .transforms import park, wrap


class SMOEstimator:
    name = "SMO"

    def __init__(self, motor, smo, Ts):
        self.m, self.c, self.Ts = motor, smo, Ts
        self.ia_hat = self.ib_hat = 0.0
        self.ea = self.eb = 0.0          # filtered back-EMF estimate
        self.theta_pll = 0.0
        self.w_pll = 0.0                  # electrical speed estimate [rad/s]
        self.integ = 0.0
        wn = 2 * math.pi * smo.pll_bw_hz
        self.kp_pll = 2 * smo.pll_zeta * wn
        self.ki_pll = wn * wn

    def _sw(self, err):
        return self.c.k * max(-1.0, min(1.0, err / self.c.phi))

    def update(self, va, vb, ia, ib):
        m, c, Ts = self.m, self.c, self.Ts
        za = self._sw(self.ia_hat - ia)
        zb = self._sw(self.ib_hat - ib)
        self.ia_hat += Ts / m.Ls * (va - m.Rs * self.ia_hat - za)
        self.ib_hat += Ts / m.Ls * (vb - m.Rs * self.ib_hat - zb)
        a = c.lpf_wc * Ts
        self.ea += a * (za - self.ea)
        self.eb += a * (zb - self.eb)
        # PLL: e_alpha = -psi w sin(th), e_beta = psi w cos(th)
        mag = max(math.hypot(self.ea, self.eb), 1e-3)
        err = (-self.ea * math.cos(self.theta_pll) - self.eb * math.sin(self.theta_pll)) / mag
        self.integ += self.ki_pll * err * Ts
        self.w_pll = self.kp_pll * err + self.integ
        self.theta_pll = wrap(self.theta_pll + self.w_pll * Ts)

    @property
    def theta(self):
        # compensate the phase lag of the first-order back-EMF filter
        return wrap(self.theta_pll + math.atan2(self.w_pll, self.c.lpf_wc))

    @property
    def we(self):
        return self.w_pll

    @property
    def we_filtered(self):
        """PLL integrator output: the smooth part of the speed estimate, used for control."""
        return self.integ


class MRASEstimator:
    name = "MRAS"

    def __init__(self, motor, mras, Ts):
        self.m, self.c, self.Ts = motor, mras, Ts
        self.idp_hat = motor.psi_f / motor.Ls   # adjustable model states i'_d, i'_q
        self.iqp_hat = 0.0
        self.w = 0.0
        self.integ = 0.0
        self._theta = 0.0

    def init_state(self, theta, we):
        self._theta, self.w, self.integ = theta, we, we

    def update(self, va, vb, ia, ib):
        m, c, Ts = self.m, self.c, self.Ts
        vd, vq = park(va, vb, self._theta)
        id_, iq = park(ia, ib, self._theta)
        k = m.psi_f / m.Ls
        idp, iqp = id_ + k, iq
        vdp = vd + m.Rs * k
        # adjustable model (forward Euler at the control rate)
        didp = -m.Rs / m.Ls * self.idp_hat + self.w * self.iqp_hat + vdp / m.Ls
        diqp = -self.w * self.idp_hat - m.Rs / m.Ls * self.iqp_hat + vq / m.Ls
        self.idp_hat += Ts * didp
        self.iqp_hat += Ts * diqp
        err = idp * self.iqp_hat - iqp * self.idp_hat
        self.integ += c.ki * err * Ts
        self.w = c.kp * err + self.integ
        self._theta = wrap(self._theta + self.w * Ts)

    @property
    def theta(self):
        return self._theta

    @property
    def we(self):
        return self.w

    @property
    def we_filtered(self):
        """Integral part of the adaptation law: the smooth speed estimate used for control."""
        return self.integ
