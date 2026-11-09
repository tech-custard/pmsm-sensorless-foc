"""Rebuild the results table in README.md from results/metrics.csv."""
import csv, os
ROOT = os.path.join(os.path.dirname(__file__), "..")
rows = list(csv.DictReader(open(os.path.join(ROOT, "results", "metrics.csv"))))
NAME = {"sensored": "Sensored", "smo": "SMO + PLL", "mras": "MRAS"}


def f(r, k, nd=1):
    v = r.get(k, "")
    if v in ("", "nan"): return "n/a"
    s = f"{float(v):.{nd}f}"
    return s[1:] if s.startswith("-") and float(s) == 0 else s


lines = ["| Experiment | Estimator | Stayed locked | Mean speed error [rpm] | Mean angle error [deg] | Max angle error in window [deg] | Extra |",
         "| --- | --- | --- | --- | --- | --- | --- |"]
for r in rows:
    if r["estimator"] == "sensored" and not r["experiment"].startswith(("E1", "E2")):
        continue
    extra = ""
    if r["experiment"].startswith("E1"):
        extra = f"settling 1000>3000: {f(r,'settle_1000_to_3000_ms',0)} ms; 3000>1500: {f(r,'settle_3000_to_1500_ms',0)} ms"
    elif r["experiment"].startswith("E2"):
        extra = f"dip {f(r,'load_speed_dip_rpm',0)} rpm, recovery {f(r,'load_recovery_time_ms',0)} ms"
    elif r["experiment"].startswith("E5"):
        extra = f"speed-estimate noise {f(r,'est_speed_err_rms_rpm')} rpm rms"
    locked = "yes" if r["locked"] == "True" else "**no**"
    lines.append(f"| {r['experiment']} | {NAME[r['estimator']]} | {locked} | {f(r,'speed_err_mean_rpm')} | "
                 f"{f(r,'angle_err_mean_deg',2)} | {f(r,'angle_err_max_deg',1)} | {extra} |")
table = "\n".join(lines)
path = os.path.join(ROOT, "README.md")
text = open(path).read()
a, b = text.index("<!-- RESULTS:START -->"), text.index("<!-- RESULTS:END -->")
open(path, "w").write(text[: a + len("<!-- RESULTS:START -->")] + "\n" + table + "\n" + text[b:])
print("README table updated")
