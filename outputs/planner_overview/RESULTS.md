# Planner overview runs: results tables

Overnight 2026-10-01/02, 21 runs, `run.sh` in this folder. All runs: solver 6 (`--scheduleModel 6`), corridor margin 0,
corridor congestion off, `--computeGuidePaths false`, one run at a time. Every run exited 0 with 0 planner errors and
0 hierarchy fallbacks; the memory watchdog never fired. Run notes: `overnight_notes.txt`. Regenerate with
`python3 summarise.py && python3 make_results_md.py`.

**Terms**

- *Long path*: a guide path whose start-goal Manhattan distance is at least 500 cells (delivery legs, mostly).
- *No path*: agents without a guide path at that decision (they fall back to Manhattan moves).
- *Backlog clear*: first decision after which the no-path count stays at or under 1% of agents.
- *Replans*: paths built for the same goal the agent already had (it was pushed too far off its path).
- *Stretch (M)*: path cells / start-goal Manhattan distance. Manhattan is a lower bound, so compare between runs,
  not against 1. The bench tables (part D) use the true shortest distance instead.
- Times are wall-clock ms on this machine (8 cores, 31 GB).

## A. Does it help? 10k agents, 1,500 steps, guide-path level 4, solver 6 level 4

### Outcome

| Run | Deliveries | Tasks opened | Decisions / steps | Errors | Fallbacks | Peak GB | Wall |
|:---|---:|---:|---:|---:|---:|---:|---:|
| orz900d astar | 5,423 | 15,392 | 1,500 / 1,500 | 0 | 0 | 1.7 | 22:13.24 |
| orz900d corridor | 4,756 | 14,710 | 1,500 / 1,500 | 0 | 0 | 1.6 | 22:27.33 |
| orz900d lift | 3,719 | 13,680 | 1,500 / 1,500 | 0 | 0 | 1.7 | 22:28.00 |
| IH astar | 2,464 | 12,361 | 1,004 / 1,500 | 0 | 0 | 4.7 | 18:49.15 |
| IH corridor | 9,213 | 18,986 | 1,500 / 1,500 | 0 | 0 | 5.0 | 23:06.73 |
| IH lift | 7,607 | 17,329 | 1,500 / 1,500 | 0 | 0 | 5.0 | 23:09.77 |
| scene astar | 388 | 10,367 | 820 / 1,500 | 0 | 0 | 16.9 | 20:50.75 |
| scene corridor | 1,405 | 11,348 | 1,165 / 1,500 | 0 | 0 | 18.1 | 24:26.95 |
| scene lift | 1,302 | 11,205 | 1,200 / 1,501 | 0 | 0 | 17.9 | 24:40.73 |

### Agents without a path, and stuck agents

| Run | No path, peak | Backlog clear (decision) | No path, end | Stuck, max | Stuck, end |
|:---|---:|---:|---:|---:|---:|
| orz900d astar | 9,003 | 128 | 0 | 753 | 5 |
| orz900d corridor | 7,193 | 25 | 0 | 230 | 13 |
| orz900d lift | 2,179 | 5 | 0 | 79 | 6 |
| IH astar | 8,930 | - | 6,046 | 4,101 | 3,989 |
| IH corridor | 1,492 | 22 | 0 | 6 | 0 |
| IH lift | 0 | 1 | 0 | 1 | 0 |
| scene astar | 9,587 | - | 8,159 | 6,326 | 6,295 |
| scene corridor | 4,650 | 92 | 5 | 68 | 0 |
| scene lift | 64 | 1 | 0 | 7 | 0 |

No-path count at selected decisions:

| Run | d1 | d2 | d10 | d25 | d50 | d100 | d200 | d500 | d1000 |
|:---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| orz900d astar | 0 | 3,899 | 8,960 | 7,991 | 6,222 | 2,128 | 0 | 0 | 0 |
| orz900d corridor | 0 | 3,559 | 5,954 | 0 | 0 | 0 | 0 | 0 | 0 |
| orz900d lift | 0 | 1,846 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| IH astar | 0 | 299 | 4,535 | 7,839 | 8,930 | 8,483 | 7,797 | 6,663 | 6,053 |
| IH corridor | 0 | 0 | 1,436 | 0 | 0 | 0 | 0 | 0 | 0 |
| IH lift | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| scene astar | 0 | 89 | 2,379 | 6,542 | 9,319 | 9,557 | 9,286 | 8,657 | - |
| scene corridor | 0 | 0 | 1,444 | 3,974 | 4,140 | 0 | 0 | 0 | 2 |
| scene lift | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

### Guide paths built

| Run | Paths | Long paths | ms p50 | ms p90 | ms max | Long ms | Long ms p90 | Long: coarse ms | Long: build ms | Long cells | Long stretch (M) | Replans |
|:---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| orz900d astar | 44,071 | 12,454 | 5.52 | 16.70 | 50 | 13.99 | 19.23 | 0.00 | 0.00 | 1,828 | 2.687 | 32.2% |
| orz900d corridor | 58,177 | 19,475 | 1.10 | 3.51 | 16 | 2.87 | 4.21 | 0.06 | 2.63 | 1,663 | 2.448 | 49.8% |
| orz900d lift | 61,411 | 19,783 | 0.26 | 0.66 | 5 | 0.56 | 0.83 | 0.05 | 0.30 | 2,131 | 3.170 | 55.7% |
| IH astar | 15,577 | 4,034 | 0.01 | 200.02 | 1,205 | 214.81 | 557.47 | 0.00 | 0.00 | 1,260 | 1.171 | 0.0% |
| IH corridor | 38,430 | 15,901 | 0.24 | 3.33 | 59 | 2.43 | 5.05 | 0.26 | 2.05 | 1,218 | 1.055 | 0.7% |
| IH lift | 34,954 | 14,380 | 0.26 | 1.42 | 8 | 1.12 | 1.99 | 0.27 | 0.71 | 1,319 | 1.144 | 0.2% |
| scene astar | 11,933 | 1,807 | 0.02 | 169.37 | 2,281 | 483.35 | 1,126.72 | 0.00 | 0.00 | 3,025 | 1.402 | 0.0% |
| scene corridor | 22,762 | 10,809 | 0.35 | 11.51 | 136 | 7.56 | 14.48 | 0.68 | 6.57 | 2,860 | 1.265 | 0.1% |
| scene lift | 22,507 | 10,693 | 0.31 | 2.96 | 16 | 2.13 | 3.64 | 0.61 | 1.23 | 2,980 | 1.311 | 0.0% |

`astar` has no coarse/build split (one full-map A*), so those columns read 0.

### Deliveries over time (cumulative)

| Run | step 100 | step 250 | step 500 | step 750 | step 1000 | step 1250 | step 1500 |
|:---|---:|---:|---:|---:|---:|---:|---:|
| orz900d astar | 399 | 1,058 | 2,066 | 3,110 | 3,934 | 4,719 | 5,423 |
| orz900d corridor | 416 | 1,016 | 1,771 | 2,509 | 3,207 | 3,972 | 4,756 |
| orz900d lift | 254 | 758 | 1,393 | 2,015 | 2,632 | 3,201 | 3,719 |
| IH astar | 28 | 196 | 532 | 906 | 1,373 | 1,869 | 2,464 |
| IH corridor | 54 | 383 | 1,295 | 2,750 | 4,727 | 6,869 | 9,213 |
| IH lift | 23 | 266 | 1,007 | 2,187 | 3,757 | 5,624 | 7,607 |
| scene astar | 2 | 23 | 88 | 159 | 240 | 320 | 388 |
| scene corridor | 7 | 67 | 270 | 530 | 804 | 1,087 | 1,405 |
| scene lift | 4 | 47 | 239 | 478 | 747 | 1,017 | 1,302 |

### Planner time per decision (means over all decisions)

| Run | Scheduler ms | Scheduler max | Guide paths ms | Frank-Wolfe ms | PIBT ms | PIBT max | PIBT reserve |
|:---|---:|---:|---:|---:|---:|---:|---:|
| orz900d astar | 12 | 47 | 203 | 602 | 44 | 84 | 100 |
| orz900d corridor | 12 | 41 | 60 | 745 | 54 | 116 | 100 |
| orz900d lift | 11 | 40 | 17 | 789 | 54 | 99 | 100 |
| IH astar | 106 | 262 | 929 | 1 | 21 | 37 | 100 |
| IH corridor | 162 | 242 | 32 | 621 | 47 | 84 | 100 |
| IH lift | 151 | 259 | 17 | 647 | 47 | 98 | 100 |
| scene astar | 238 | 1,938 | 1,068 | 1 | 15 | 48 | 100 |
| scene corridor | 599 | 1,924 | 73 | 352 | 51 | 91 | 100 |
| scene lift | 546 | 1,916 | 23 | 433 | 53 | 151 | 100 |

Frank-Wolfe fills whatever time is left before the PIBT reserve, so a high value means spare time, not cost.
The scene corridor/lift runs miss about 300 decisions because the scheduler's time grows to about 1 s per decision
late in the run (solver 6 at level 4; known todo item), not because of the planner.

### Per-path time by start-goal distance

**orz900d**

| Manhattan distance | astar n | astar ms | astar stretch (M) | corridor n | corridor ms | corridor stretch (M) | lift n | lift ms | lift stretch (M) |
|:---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0-50 | 15,352 | 0.04 | 1.042 | 15,242 | 0.02 | 1.008 | 14,450 | 0.02 | 3.439 |
| 50-200 | 3,676 | 3.24 | 2.173 | 4,843 | 0.36 | 1.839 | 5,767 | 0.14 | 2.484 |
| 200-500 | 12,589 | 8.95 | 2.810 | 18,617 | 1.41 | 2.428 | 21,411 | 0.35 | 3.260 |
| 500-1,000 | 11,993 | 13.85 | 2.692 | 18,853 | 2.82 | 2.452 | 19,156 | 0.55 | 3.180 |
| 1,000-2,000 | 461 | 17.62 | 2.541 | 622 | 4.16 | 2.325 | 627 | 0.72 | 2.850 |

**IH**

| Manhattan distance | astar n | astar ms | astar stretch (M) | corridor n | corridor ms | corridor stretch (M) | lift n | lift ms | lift stretch (M) |
|:---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0-50 | 10,011 | 0.21 | 1.075 | 18,542 | 0.08 | 1.004 | 17,055 | 0.09 | 2.543 |
| 50-200 | 438 | 23.21 | 1.867 | 1,253 | 0.24 | 1.133 | 1,091 | 0.23 | 1.500 |
| 200-500 | 1,094 | 49.42 | 1.406 | 2,734 | 0.73 | 1.179 | 2,428 | 0.48 | 1.320 |
| 500-1,000 | 1,953 | 128.17 | 1.211 | 6,575 | 1.49 | 1.078 | 5,974 | 0.77 | 1.175 |
| 1,000-2,000 | 1,940 | 285.68 | 1.136 | 8,621 | 2.96 | 1.041 | 7,745 | 1.31 | 1.125 |
| 2,000-4,000 | 141 | 439.66 | 1.099 | 705 | 4.84 | 1.010 | 661 | 2.12 | 1.092 |

**scene**

| Manhattan distance | astar n | astar ms | astar stretch (M) | corridor n | corridor ms | corridor stretch (M) | lift n | lift ms | lift stretch (M) |
|:---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0-50 | 9,542 | 0.02 | 1.003 | 10,673 | 0.05 | 1.003 | 10,584 | 0.06 | 2.427 |
| 50-200 | 497 | 0.52 | 1.109 | 810 | 0.15 | 1.040 | 787 | 0.17 | 1.398 |
| 200-500 | 87 | 24.88 | 1.634 | 470 | 0.78 | 1.243 | 443 | 0.41 | 1.316 |
| 500-1,000 | 227 | 110.22 | 1.646 | 1,210 | 1.98 | 1.350 | 1,171 | 0.78 | 1.407 |
| 1,000-2,000 | 584 | 255.28 | 1.431 | 3,291 | 4.70 | 1.328 | 3,226 | 1.46 | 1.381 |
| 2,000-4,000 | 884 | 659.19 | 1.345 | 5,559 | 9.50 | 1.230 | 5,520 | 2.59 | 1.273 |
| 4,000-inf | 112 | 1,040.96 | 1.209 | 749 | 14.69 | 1.108 | 776 | 3.68 | 1.144 |

## B. Scaling: lift, guide-path level 4, solver 6 level 8, 500 steps

### IH

| Agents | Deliveries | Decisions / steps | No path, peak | Backlog clear | No path, end | Stuck, max | Stuck, end | Paths | Long ms | Long ms p90 | Peak GB | Wall |
|:---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10k | 997 | 500 / 500 | 0 | 1 | 0 | 2 | 0 | 21,855 | 1.00 | 1.65 | 5.0 | 7:49.96 |
| 20k | 1,852 | 500 / 500 | 9,951 | 18 | 0 | 24 | 1 | 43,101 | 0.96 | 1.57 | 5.7 | 7:28.96 |
| 40k | 3,426 | 498 / 500 | 40,000 | 72 | 0 | 227 | 15 | 59,046 | 0.94 | 1.53 | 6.5 | 6:42.46 |
| 80k | 5,872 | 491 / 500 | 80,000 | - | 64,244 | 6,847 | 6,847 | 17,328 | 0.81 | 1.32 | 8.4 | 3:21.29 |

Where each 1,000 ms step goes (mean ms per decision, decisions 51 onwards):

| Agents | Scheduler | Guide paths | Frank-Wolfe | PIBT used | PIBT max | PIBT reserve | Unused |
|:---|---:|---:|---:|---:|---:|---:|---:|
| 10k | 20 | 10 | 786 | 44 | 60 | 100 | 140 |
| 20k | 30 | 16 | 666 | 94 | 184 | 200 | 194 |
| 40k | 46 | 52 | 408 | 186 | 360 | 400 | 309 |
| 80k | 72 | 22 | 8 | 102 | 135 | 800 | 796 |

Unused = 1,000 minus the four columns: the unused part of the PIBT reserve plus about 80 ms of fixed tolerances.

No-path count at selected decisions:

| Agents | d1 | d10 | d25 | d50 | d100 | d200 | d300 | d400 | d490 |
|:---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10k | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 20k | 9,951 | 3,435 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 40k | 40,000 | 32,253 | 25,051 | 12,343 | 0 | 0 | 0 | 0 | 0 |
| 80k | 80,000 | 79,016 | 78,419 | 77,626 | 76,021 | 72,779 | 69,692 | 66,719 | 64,270 |

### scene

| Agents | Deliveries | Decisions / steps | No path, peak | Backlog clear | No path, end | Stuck, max | Stuck, end | Paths | Long ms | Long ms p90 | Peak GB | Wall |
|:---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10k | 273 | 500 / 500 | 0 | 1 | 0 | 0 | 0 | 20,470 | 2.15 | 3.59 | 17.8 | 9:03.38 |
| 20k | 545 | 500 / 500 | 10,647 | 53 | 0 | 4 | 4 | 40,689 | 2.15 | 3.62 | 19.1 | 8:53.00 |
| 40k | 1,017 | 500 / 500 | 40,000 | 179 | 0 | 1,647 | 28 | 52,497 | 2.10 | 3.56 | 21.5 | 8:08.92 |
| 80k | 1,697 | 498 / 500 | 80,000 | - | 78,430 | 29,803 | 29,803 | 2,090 | 2.06 | 3.53 | 17.2 | 4:51.91 |

Where each 1,000 ms step goes (mean ms per decision, decisions 51 onwards):

| Agents | Scheduler | Guide paths | Frank-Wolfe | PIBT used | PIBT max | PIBT reserve | Unused |
|:---|---:|---:|---:|---:|---:|---:|---:|
| 10k | 30 | 17 | 764 | 52 | 77 | 100 | 136 |
| 20k | 53 | 27 | 629 | 108 | 168 | 200 | 184 |
| 40k | 81 | 144 | 277 | 194 | 268 | 400 | 305 |
| 80k | 120 | 3 | 8 | 79 | 130 | 800 | 790 |

Unused = 1,000 minus the four columns: the unused part of the PIBT reserve plus about 80 ms of fixed tolerances.

No-path count at selected decisions:

| Agents | d1 | d10 | d25 | d50 | d100 | d200 | d300 | d400 | d490 |
|:---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10k | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 20k | 10,647 | 2,493 | 3,268 | 583 | 0 | 0 | 0 | 0 | 0 |
| 40k | 40,000 | 32,221 | 31,558 | 27,533 | 17,379 | 0 | 0 | 0 | 0 |
| 80k | 80,000 | 79,518 | 79,307 | 79,193 | 79,007 | 78,631 | 78,476 | 78,451 | 78,427 |

At 80k the 800 ms PIBT reserve (1 ms per 100 agents) leaves stage 2 almost no time; PIBT itself used at most
about 135 ms there, but with most agents on Manhattan moves (cheaper than following a path). At 40k with every
agent on a path it peaked at 268-360 ms, so a linear guess for 80k with paths is about 550-720 ms.

## C. Guide-path level: IH 10k, solver 6 level 4, first 500 steps

Level 4 rows are the first 500 steps of the part A runs. Cells and coarse/build split are whole-run means.

| Run | Deliveries by step 500 | Long paths | Long ms | Coarse ms | Build ms | Long cells | Backlog clear | Stuck, max |
|:---|---:|---:|---:|---:|---:|---:|---:|---:|
| corridor L3 | 1,297 | 9,293 | 2.40 | 0.76 | 1.52 | 1,232 | 22 | 1 |
| corridor L4 | 1,295 | 9,290 | 2.42 | 0.26 | 2.05 | 1,218 | 22 | 6 |
| corridor L6 | 1,239 | 9,136 | 6.73 | 0.03 | 6.57 | 1,244 | 73 | 37 |
| lift L3 | 1,135 | 9,136 | 1.45 | 0.73 | 0.59 | 1,267 | 1 | 0 |
| lift L4 | 1,007 | 9,030 | 0.98 | 0.27 | 0.71 | 1,319 | 1 | 1 |
| lift L6 | 638 | 8,729 | 0.95 | 0.03 | 0.75 | 1,625 | 1 | 0 |

## D. Path length / shortest, by shortest distance (bench, no planner)

From `../hierarchy_lift_bench/coarse_astar/` (random agent-start to task pairs, 300 on orz900d and IH, 200 on
scene). Shortest = BFS on the fine map. Mean per bin, p90 in brackets. Corridor = margin 0.

**orz900d**

| Shortest distance | Pairs | L1 lift / corridor | L2 lift / corridor | L3 lift / corridor | L4 lift / corridor | L6 lift / corridor |
|:---|---:|---:|---:|---:|---:|---:|
| 0-100 | 14 | 1.06 (1.12) / 1.00 | 1.20 (1.41) / 1.00 | 1.47 (1.74) / 1.00 | 2.10 (3.32) / 1.00 | 4.64 (6.85) / 1.00 |
| 100-300 | 30 | 1.02 (1.04) / 1.00 | 1.07 (1.13) / 1.00 | 1.15 (1.28) / 1.00 | 1.37 (1.66) / 1.00 | 2.33 (3.17) / 1.00 |
| 300-1,000 | 96 | 1.01 (1.02) / 1.00 | 1.05 (1.07) / 1.00 | 1.11 (1.17) / 1.00 | 1.24 (1.35) / 1.00 | 1.67 (1.98) / 1.00 |
| 1,000-2,000 | 109 | 1.01 (1.02) / 1.00 | 1.04 (1.06) / 1.00 | 1.12 (1.14) / 1.00 | 1.26 (1.32) / 1.00 | 1.53 (1.68) / 1.00 |
| 2,000-4,000 | 51 | 1.01 (1.01) / 1.00 | 1.04 (1.04) / 1.00 | 1.10 (1.12) / 1.00 | 1.24 (1.28) / 1.00 | 1.48 (1.52) / 1.00 |

**IH**

| Shortest distance | Pairs | L1 lift / corridor | L2 lift / corridor | L3 lift / corridor | L4 lift / corridor | L6 lift / corridor |
|:---|---:|---:|---:|---:|---:|---:|
| 0-100 | 1 | 1.00 (1.00) / 1.00 | 1.00 (1.00) / 1.00 | 1.00 (1.00) / 1.00 | 1.17 (1.17) / 1.00 | 2.66 (2.66) / 1.34 |
| 100-300 | 12 | 1.01 (1.02) / 1.00 | 1.03 (1.08) / 1.00 | 1.07 (1.15) / 1.00 | 1.16 (1.32) / 1.00 | 1.66 (2.34) / 1.01 |
| 300-1,000 | 100 | 1.00 (1.01) / 1.00 | 1.01 (1.03) / 1.00 | 1.04 (1.07) / 1.00 | 1.10 (1.17) / 1.00 | 1.43 (1.74) / 1.02 |
| 1,000-2,000 | 165 | 1.00 (1.01) / 1.00 | 1.01 (1.02) / 1.00 | 1.03 (1.05) / 1.00 | 1.09 (1.13) / 1.00 | 1.31 (1.48) / 1.02 |
| 2,000-4,000 | 22 | 1.00 (1.00) / 1.00 | 1.01 (1.01) / 1.00 | 1.03 (1.04) / 1.00 | 1.09 (1.14) / 1.00 | 1.28 (1.36) / 1.01 |

**scene**

| Shortest distance | Pairs | L2 lift / corridor | L3 lift / corridor | L4 lift / corridor | L5 lift / corridor | L6 lift / corridor |
|:---|---:|---:|---:|---:|---:|---:|
| 100-300 | 2 | 1.01 (1.01) / 1.00 | 1.09 (1.13) / 1.00 | 1.15 (1.25) / 1.00 | 1.45 (1.72) / 1.00 | 1.83 (2.18) / 1.00 |
| 300-1,000 | 15 | 1.02 (1.03) / 1.00 | 1.04 (1.07) / 1.00 | 1.07 (1.14) / 1.01 | 1.16 (1.27) / 1.01 | 1.32 (1.50) / 1.02 |
| 1,000-2,000 | 43 | 1.01 (1.02) / 1.00 | 1.03 (1.04) / 1.00 | 1.05 (1.08) / 1.00 | 1.11 (1.18) / 1.01 | 1.25 (1.43) / 1.01 |
| 2,000-4,000 | 101 | 1.01 (1.01) / 1.00 | 1.02 (1.04) / 1.00 | 1.05 (1.07) / 1.00 | 1.11 (1.16) / 1.01 | 1.21 (1.34) / 1.01 |
| 4,000-inf | 39 | 1.01 (1.01) / 1.00 | 1.02 (1.04) / 1.01 | 1.05 (1.11) / 1.01 | 1.10 (1.16) / 1.01 | 1.22 (1.30) / 1.02 |

## E. Path length: A* vs corridor vs lift on the same pairs (bench, no planner)

Same bench pairs as part D; every builder gets the same start and goal, so the ratios are paired. A* is the
planner's full-map A* with an empty congestion map, so it finds a shortest path (A* / shortest = 1.000 on every
pair). The planner's own A* runs with congestion and takes longer routes; part A's stretch columns show that, but
those runs have different goals, so they can't be paired like this. Ratios: mean, p90 and max over pairs.

**orz900d**

| Level | Pairs | A* cells | Corridor cells | Lift cells | Corridor / A* | p90 | max | Lift / A* | p90 | max |
|:---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| L1 | 300 | 1,166 | 1,167 | 1,180 | 1.000 | 1.00 | 1.01 | 1.015 | 1.02 | 1.14 |
| L2 | 300 | 1,166 | 1,168 | 1,217 | 1.001 | 1.00 | 1.02 | 1.054 | 1.07 | 1.44 |
| L3 | 300 | 1,166 | 1,168 | 1,296 | 1.002 | 1.00 | 1.04 | 1.131 | 1.19 | 2.33 |
| L4 | 300 | 1,166 | 1,168 | 1,460 | 1.002 | 1.00 | 1.04 | 1.299 | 1.40 | 3.53 |
| L6 | 300 | 1,166 | 1,166 | 1,806 | 1.000 | 1.00 | 1.00 | 1.790 | 2.26 | 9.76 |

**IH**

| Level | Pairs | A* cells | Corridor cells | Lift cells | Corridor / A* | p90 | max | Lift / A* | p90 | max |
|:---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| L1 | 300 | 1,189 | 1,190 | 1,193 | 1.000 | 1.00 | 1.02 | 1.004 | 1.01 | 1.03 |
| L2 | 300 | 1,189 | 1,191 | 1,203 | 1.001 | 1.00 | 1.04 | 1.013 | 1.02 | 1.08 |
| L3 | 300 | 1,189 | 1,192 | 1,226 | 1.002 | 1.01 | 1.06 | 1.034 | 1.06 | 1.15 |
| L4 | 300 | 1,189 | 1,192 | 1,295 | 1.002 | 1.01 | 1.04 | 1.094 | 1.15 | 1.37 |
| L6 | 300 | 1,189 | 1,212 | 1,579 | 1.020 | 1.07 | 1.36 | 1.365 | 1.58 | 2.73 |

**scene**

| Level | Pairs | A* cells | Corridor cells | Lift cells | Corridor / A* | p90 | max | Lift / A* | p90 | max |
|:---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| L2 | 200 | 2,804 | 2,810 | 2,833 | 1.001 | 1.00 | 1.10 | 1.010 | 1.02 | 1.11 |
| L3 | 200 | 2,804 | 2,816 | 2,875 | 1.003 | 1.01 | 1.10 | 1.027 | 1.04 | 1.13 |
| L4 | 200 | 2,804 | 2,822 | 2,941 | 1.005 | 1.01 | 1.10 | 1.051 | 1.08 | 1.25 |
| L5 | 200 | 2,804 | 2,829 | 3,102 | 1.008 | 1.02 | 1.10 | 1.114 | 1.18 | 1.72 |
| L6 | 200 | 2,804 | 2,833 | 3,424 | 1.010 | 1.03 | 1.21 | 1.236 | 1.36 | 2.18 |
