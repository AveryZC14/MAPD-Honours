#!/usr/bin/env python3
"""Part C: guide-path level on IH 10k over the first 500 steps (level 4 from the A runs). Reads series.json."""
import json
from pathlib import Path
D = Path(__file__).resolve().parent
d = json.load(open(D / "series.json")); S = d["series"]; R = {r["name"]: r for r in d["runs"]}
print(f"{'run':<36} {'fin@500':>7} {'long_n':>6} {'long_ms':>7} {'coarse':>6} {'build':>6} {'cells':>6} {'clear1%':>7} {'stuck_max':>9} {'err':>3} {'fallb':>5}")
for src in ["corridor", "lift"]:
    for lvl in [3, 4, 6]:
        name = f"{'A' if lvl == 4 else 'C'}_IH_mp_2p_01_10000_{src}_L{lvl}"
        if name not in S or "step" not in S[name]:  # not run, or still running
            continue
        s, r = S[name], R[name]
        fin = sum(f for st, f in zip(s["step"], s["finished_step"]) if st <= 500)
        lb = [x for x in s.get("long_by_step", []) if x[0] <= 500]
        n = sum(x[1] for x in lb); ms = sum(x[1] * x[2] for x in lb) / max(1, n)
        print(f"{name:<36} {fin:>7} {n:>6} {ms:>7.2f} {r.get('long_coarse_ms', 0):>6.2f} {r.get('long_build_ms', 0):>6.2f}"
              f" {r.get('long_cells_mean', 0):>6.0f} {str(r.get('no_path_clear')):>7} {r.get('stuck_max', 0):>9.0f}"
              f" {r.get('errors')!s:>3} {r.get('fallbacks', 0):>5.0f}")
print("(coarse/build/cells: whole run, so the A rows cover 1,500 steps)")
