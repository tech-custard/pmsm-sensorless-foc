"""Quick checks: run with  python -m pytest -q"""
import math, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from pmsm_foc.transforms import park, inv_park
from pmsm_foc.sim import simulate, Profile
from pmsm_foc import metrics as M

PROFILE = Profile(lambda t: 1000.0, lambda t: 0.02 if t > 0.5 else 0.0, 0.7)


def test_park_round_trip():
    for th in (0.0, 0.7, -2.1, 3.0):
        a, b = inv_park(*park(1.3, -0.4, th), th)
        assert abs(a - 1.3) < 1e-12 and abs(b + 0.4) < 1e-12


def test_sensored_tracks_speed():
    L = simulate("sensored", PROFILE)
    s = M.steady_metrics(L, 0.6, 0.7)
    assert abs(s["speed_err_mean_rpm"]) < 5


def test_estimators_lock_at_1000rpm():
    for kind in ("smo", "mras"):
        L = simulate(kind, PROFILE)
        s = M.steady_metrics(L, 0.6, 0.7)
        assert M.locked(L, 0.25), kind
        assert abs(s["angle_err_mean_deg"]) < 5, kind
        assert abs(s["speed_err_mean_rpm"]) < 5, kind
