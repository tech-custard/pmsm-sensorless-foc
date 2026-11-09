"""Closed-loop simulation: I/f start-up, then sensorless (or sensored) FOC."""
import math
import random
from dataclasses import dataclass
from .params import Config
from .motor import PMSM
from .controllers import PI
from .observers import SMOEstimator, MRASEstimator
from .transforms import park, inv_park, wrap

RPM = 2 * math.pi / 60.0   # rpm -> rad/s


@dataclass
class Profile:
    """Speed reference [rpm] and load torque [N m] as functions of time."""
    speed_rpm: callable
    load_nm: callable
    t_end: float


def make_estimator(kind, cfg):
    if kind == "smo":
        return SMOEstimator(cfg.motor, cfg.smo, cfg.control.Ts)
    if kind == "mras":
        return MRASEstimator(cfg.motor, cfg.mras, cfg.control.Ts)
    if kind == "sensored":
        return None
    raise ValueError(kind)


def simulate(kind: str, profile: Profile, cfg: Config = None, plant_motor=None, log_every: int = 4):
    """Run one experiment.

    kind         'smo', 'mras' or 'sensored' (uses the true angle; baseline only)
    plant_motor  optional MotorParams for the simulated machine when it differs
                 from the controller's parameters (robustness tests)
    Returns a dict of logged lists.
    """
    cfg = cfg or Config()
    m, ctl, inv = cfg.motor, cfg.control, cfg.inverter
    Ts, h = ctl.Ts, ctl.Ts / ctl.substeps
    rng = random.Random(cfg.seed)
    plant = PMSM(plant_motor or m)
    est = make_estimator(kind, cfg)

    wc = 2 * math.pi * ctl.current_bw_hz
    pi_d = PI(m.Ls * wc, m.Rs * wc, Ts, inv.Vmax)
    pi_q = PI(m.Ls * wc, m.Rs * wc, Ts, inv.Vmax)
    ws = 2 * math.pi * ctl.speed_bw_hz
    kp_w = m.J * ws / m.Kt
    pi_w = PI(kp_w, kp_w * ws / 4.0, Ts, ctl.i_max)

    sensorless = kind != "sensored"
    mode = "align" if sensorless else "closed"
    id_hand = 0.0
    theta_if, w_if = 0.0, 0.0
    accel = ctl.if_accel_rpm_s * RPM * m.p           # electrical rad/s^2
    va = vb = 0.0
    log = {k: [] for k in ("t", "w_ref", "w", "w_hat", "theta_err", "id", "iq", "Te", "TL", "mode")}
    n = int(round(profile.t_end / Ts))
    t_switch = None

    for k in range(n):
        t = k * Ts
        w_ref = profile.speed_rpm(t) * RPM               # mechanical rad/s
        TL = profile.load_nm(t)
        ia, ib = plant.i_alpha_beta()
        if cfg.current_noise_std:
            ia += rng.gauss(0, cfg.current_noise_std)
            ib += rng.gauss(0, cfg.current_noise_std)

        if est is not None:
            est.update(va, vb, ia, ib)                   # voltages applied over the last period

        id_ref = 0.0
        if mode == "align":
            # hold a d-axis current at angle 0 so the rotor settles at a known angle
            theta_c, id_ref, iq_ref, w_fb = 0.0, ctl.if_current, 0.0, 0.0
            if t >= ctl.align_time:
                mode = "if"
        elif mode == "if":
            # I/f: current vector of fixed amplitude rotates open-loop; the rotor
            # follows it with a load angle delta where Te = Kt * I * sin(delta)
            w_if = min(w_if + accel * Ts, w_ref * m.p) if w_ref > 0 else 0.0
            theta_if = wrap(theta_if + w_if * Ts)
            theta_c, id_ref, iq_ref, w_fb = theta_if, ctl.if_current, 0.0, w_if / m.p
            if w_if / m.p >= ctl.switch_rpm * RPM:
                mode, t_switch = "closed", t
                # bumpless transfer: keep the same torque-producing current
                delta = wrap(theta_if - est.theta)
                pi_w.reset(ctl.if_current * math.sin(delta))
                id_hand = ctl.if_current * math.cos(delta)
        else:
            if sensorless:
                theta_c, w_fb = est.theta, est.we_filtered / m.p
            else:
                theta_c, w_fb = wrap(plant.theta), plant.wm
            iq_ref = pi_w.update(w_ref - w_fb)
            if t_switch is not None and id_hand:
                # ramp the leftover d-axis current to zero over 50 ms
                id_hand = max(0.0, id_hand - Ts * ctl.if_current / 0.05)
                id_ref = id_hand

        id_m, iq_m = park(ia, ib, theta_c)
        we_c = w_fb * m.p
        vd = pi_d.update(id_ref - id_m, ff=-we_c * m.Ls * iq_m)
        vq = pi_q.update(iq_ref - iq_m, ff=we_c * (m.Ls * id_m + m.psi_f))
        mag = math.hypot(vd, vq)
        if mag > inv.Vmax:
            vd, vq = vd * inv.Vmax / mag, vq * inv.Vmax / mag
        va, vb = inv_park(vd, vq, theta_c)

        for _ in range(ctl.substeps):
            plant.step(va, vb, TL, h)

        if k % log_every == 0:
            log["t"].append(t)
            log["w_ref"].append(w_ref / RPM)
            log["w"].append(plant.wm / RPM)
            log["w_hat"].append((est.we / m.p / RPM) if est else plant.wm / RPM)
            log["theta_err"].append(math.degrees(wrap(est.theta - plant.theta)) if est else 0.0)
            log["id"].append(plant.id)
            log["iq"].append(plant.iq)
            log["Te"].append(plant.torque)
            log["TL"].append(TL)
            log["mode"].append(mode)
    log["t_switch"] = t_switch
    log["kind"] = kind
    return log
