# Superseded sweep runs

Runs whose results shouldn't be used, kept for comparison. The rerun has
the same name in `../`. `sweep_summary.csv` keeps both rows; use the last
row per run name.

- `E1_scene_mp_4p_03_80000_s6L6_lift-L4` (2026-10-08 08:35-09:00): an
  `apt-get install` of TeX Live ran 08:47-08:50 during the run (not seen by
  the sweep's check, which only looks for lifelong / bench processes); 1,482
  of 1,500 decisions, 9,266 finished. Rerun 2026-10-09.
