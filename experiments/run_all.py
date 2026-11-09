"""Run every experiment, write results/metrics.csv and results/figures/*.png.

Usage:  python experiments/run_all.py
Every number in the README comes from this script.
"""
import csv, os, sys, copy
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pmsm_foc.sim import simulate, Profile
from pmsm_foc.params import Config, MotorParams
from pmsm_foc import metrics as M

ROOT = os.path.join(os.path.dirname(__file__), "..")
FIG = os.path.join(ROOT, "results", "figures")
os.makedirs(FIG, exist_ok=True)
KINDS = ["sensored", "smo", "mras"]
LABEL = {"sensored": "Sensored (reference)", "smo": "SMO + PLL", "mras": "MRAS"}
STYLE = {"sensored": "-", "smo": "--", "mras": ":"}
rows = []


def ramp(t, t0, t1, a, b):
    if t <= t0: return a
    if t >= t1: return b
    return a + (b - a) * (t - t0) / (t1 - t0)


def plot(name, logs, title, xlim=None):
    fig, ax = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    for k, L in logs.items():
        ax[0].plot(L["t"], L["w"], STYLE[k], label=f"{LABEL[k]}: actual")
        if k != "sensored":
            ax[0].plot(L["t"], L["w_hat"], STYLE[k], alpha=0.5, label=f"{LABEL[k]}: estimate")
            ax[1].plot(L["t"], L["theta_err"], STYLE[k], label=LABEL[k])
        ax[2].plot(L["t"], L["iq"], STYLE[k], label=LABEL[k])
    first = next(iter(logs.values()))
    ax[0].plot(first["t"], first["w_ref"], "k", lw=0.8, label="reference")
    ax[0].set_ylabel("Speed [rpm]"); ax[1].set_ylabel("Angle error [deg]"); ax[2].set_ylabel("iq [A]")
    ax[2].set_xlabel("Time [s]"); ax[1].set_ylim(-20, 20)
    for a in ax: a.grid(alpha=0.3); a.legend(fontsize=7, loc="best")
    if xlim: ax[2].set_xlim(*xlim)
    fig.suptitle(title); fig.tight_layout()
    fig.savefig(os.path.join(FIG, name), dpi=130); plt.close(fig)


def record(exp, kind, d, note=""):
    rows.append({"experiment": exp, "estimator": kind, **{k: round(v, 3) if isinstance(v, float) else v for k, v in d.items()}, "note": note})


# E1 speed steps -----------------------------------------------------------
p1 = Profile(lambda t: 1000 if t < 0.6 else (3000 if t < 1.0 else 1500), lambda t: 0.0, 1.4)
logs = {k: simulate(k, p1) for k in KINDS}
for k, L in logs.items():
    d = M.steady_metrics(L, 0.45, 0.6)
    d["settle_1000_to_3000_ms"] = 1000 * M.settling_time(L, 0.6, 3000, t_max=1.0)
    d["settle_3000_to_1500_ms"] = 1000 * M.settling_time(L, 1.0, 1500, t_max=1.4)
    d["locked"] = M.locked(L, 0.25) if k != "sensored" else True
    record("E1_speed_steps", k, d)
plot("E1_speed_steps.png", logs, "E1: I/f start-up, then speed steps 1000 > 3000 > 1500 rpm (no load)")

# E2 load step ---------------------------------------------------------------
p2 = Profile(lambda t: 2000, lambda t: 0.10 if 0.8 <= t < 1.1 else 0.0, 1.3)
logs = {k: simulate(k, p2) for k in KINDS}
for k, L in logs.items():
    d = M.steady_metrics(L, 0.9, 1.1)
    d.update({f"load_{a}": b for a, b in M.load_step_metrics(L, 0.8, 1.1, 2000).items()})
    d["locked"] = M.locked(L, 0.25) if k != "sensored" else True
    record("E2_load_step", k, d, "0.10 N m applied at 0.8 s, removed at 1.1 s")
plot("E2_load_step.png", logs, "E2: 0.10 N m load step at 2000 rpm", xlim=(0.6, 1.3))

# E3 low speed ---------------------------------------------------------------
for target in (300, 150, 50):
    p3 = Profile(lambda t, tg=target: ramp(t, 0.6, 1.2, 1000, tg), lambda t: 0.02, 2.0)
    logs = {k: simulate(k, p3) for k in KINDS}
    for k, L in logs.items():
        d = M.steady_metrics(L, 1.6, 2.0)
        d["locked"] = M.locked(L, 0.25) if k != "sensored" else True
        record(f"E3_low_speed_{target}rpm", k, d, "ramp 1000 rpm > target, 0.02 N m load")
    plot(f"E3_low_speed_{target}rpm.png", logs, f"E3: ramp down to {target} rpm with 0.02 N m load")

# E4 parameter mismatch (plant differs from controller model) ----------------
cases = {"Rs_plus50": dict(Rs=0.36 * 1.5), "Ls_plus20": dict(Ls=0.6e-3 * 1.2), "psi_minus10": dict(psi_f=0.0064 * 0.9)}
for cname, chg in cases.items():
    plant = MotorParams(**chg)
    for target in (2000, 300):
        p4 = Profile(lambda t, tg=target: ramp(t, 0.6, 1.0, 1000, tg), lambda t: 0.05 if t >= 1.3 else 0.0, 1.8)
        logs = {k: simulate(k, p4, plant_motor=plant) for k in ["smo", "mras"]}
        for k, L in logs.items():
            d = M.steady_metrics(L, 1.5, 1.8)
            d["locked"] = M.locked(L, 0.25)
            record(f"E4_{cname}_{target}rpm", k, d, f"plant {chg}, 0.05 N m from 1.3 s")
        if target == 300:
            plot(f"E4_{cname}_300rpm.png", logs, f"E4: plant {cname} at 300 rpm")

# E5 measurement noise --------------------------------------------------------
cfg = Config(); cfg.current_noise_std = 0.02
logs = {k: simulate(k, p2, cfg) for k in KINDS}
for k, L in logs.items():
    d = M.steady_metrics(L, 0.9, 1.1)
    d["locked"] = M.locked(L, 0.25) if k != "sensored" else True
    record("E5_current_noise_20mA", k, d, "Gaussian noise 0.02 A rms on both measured currents, E2 profile")
plot("E5_current_noise.png", logs, "E5: E2 load step with 20 mA rms current-sensor noise", xlim=(0.6, 1.3))

keys = []
for r in rows:
    for k in r:
        if k not in keys: keys.append(k)
with open(os.path.join(ROOT, "results", "metrics.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(rows)
print(f"wrote {len(rows)} rows to results/metrics.csv and figures to results/figures/")
