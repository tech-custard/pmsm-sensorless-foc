"""Performance metrics computed from a simulation log."""
import math


def _window(log, t0, t1):
    return [i for i, t in enumerate(log["t"]) if t0 <= t < t1]


def rms(xs):
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else float("nan")


def steady_metrics(log, t0, t1):
    ix = _window(log, t0, t1)
    w_err = [log["w"][i] - log["w_ref"][i] for i in ix]
    est_err = [log["w_hat"][i] - log["w"][i] for i in ix]
    th = [log["theta_err"][i] for i in ix]
    te = [log["Te"][i] for i in ix]
    return {
        "speed_err_mean_rpm": sum(w_err) / len(w_err),
        "speed_err_rms_rpm": rms(w_err),
        "est_speed_err_rms_rpm": rms(est_err),
        "angle_err_mean_deg": sum(th) / len(th),
        "angle_err_rms_deg": rms(th),
        "angle_err_max_deg": max(abs(x) for x in th),
        "torque_ripple_pkpk_mNm": 1000 * (max(te) - min(te)),
    }


def settling_time(log, t_step, target_rpm, band=0.02, t_max=None):
    """Time after t_step until speed stays within +-band of target."""
    ix = [i for i, t in enumerate(log["t"]) if t >= t_step and (t_max is None or t < t_max)]
    tol = band * abs(target_rpm)
    last_out = None
    for i in ix:
        if abs(log["w"][i] - target_rpm) > tol:
            last_out = log["t"][i]
    if last_out is None:
        return 0.0
    return last_out - t_step if last_out < log["t"][ix[-1]] else float("nan")


def load_step_metrics(log, t_step, t_end, ref_rpm):
    ix = _window(log, t_step, t_end)
    dip = ref_rpm - min(log["w"][i] for i in ix)
    th = max(abs(log["theta_err"][i]) for i in ix)
    rec = settling_time(log, t_step, ref_rpm, band=0.01, t_max=t_end)
    return {"speed_dip_rpm": dip, "recovery_time_ms": 1000 * rec, "angle_err_max_deg": th}


def locked(log, t_from):
    """True if the estimator kept |angle error| < 30 deg after t_from."""
    return all(abs(e) < 30 for t, e in zip(log["t"], log["theta_err"]) if t >= t_from)
