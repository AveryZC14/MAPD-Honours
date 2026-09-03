# Throughput dashboard (Claude Artifact) — scene_mp_4p_03 + IH_mp_2p_01

Added 2026-08-27, extended 2026-08-31 into a **two-dataset dashboard**. Despite
the doc's filename (a holdover from when this only covered `scene_mp_4p_03`),
the page now holds two independent sweeps behind a top-level "Dataset:"
switcher (see "Dataset switcher" below) — everything from the original single
-dataset version still applies *within* whichever dataset is active, it's
just no longer the only thing on the page:

1. **`scene_mp_4p_03`** (~13.9M cells / ~6.3M walkable, 5000 agents) — solver 6
   at every `--flowSolveLevel` 1-8 and the solver 1 baseline, both at 5000
   timesteps; a solver 1 run stretched to 20000 timesteps; and (added
   2026-08-31) flowSolveLevel 1-2 each re-run to a 10000-timestep horizon.
   Source data: `outputs/scene_mp_4p_03_5000ts_sweep/`. The sweep referenced
   in `ai/auto_benchmarking_scene_mp_4p_03.md`.
2. **`IH_mp_2p_01`** (~3.44M cells / ~2.22M walkable, 10000 agents, added
   2026-08-31) — solver 6 at `--flowSolveLevel` 1-6 vs. the solver 1
   baseline, ~7000-timestep horizon. Source data:
   `outputs/IH_mp_2p_01_10000_7000ts_sweep/` (run via
   `scripts/run_benchmarks.py`). A **longer-horizon rerun** of the map in
   `ai/auto_benchmarking_IH_mp_2p_01.md` (that sweep was 500 timesteps) —
   see "Findings" below for why the two disagree so much on solver-6's
   margin.

Mixed horizons *within* a dataset were handled first (2026-08-28, see "Mixed
horizons" under Implementation notes) — the dataset switcher is a separate,
later addition on top of that, not a replacement for it. Not a repo tool in
the `map_reduction_test/visualisation/` sense (no Python, doesn't read the
JSON live) — it's a hand/AI-authored self-contained HTML page published as a
Claude Artifact, with the data baked in as JS literals at publish time.

**Live page:** https://claude.ai/code/artifact/b7877bbb-8313-4351-986b-1ab90907543f
(private to the account that published it; not indexed/discoverable
otherwise).

**Source HTML, checked into this repo:**
`outputs/scene_mp_4p_03_5000ts_sweep/throughput_dashboard.html` — a byte-for-byte
copy of what's currently live at the URL above, kept here specifically so a
future session doesn't need to fetch the live page to get an editable copy
(a session's `/tmp` scratchpad, where this was originally authored, does not
persist across sessions).

## Why this exists

Started as "analyse the in-progress solver 6 sweep" and grew across a
conversation into a fuller throughput/timing investigation once a unit bug
was found (see "Findings baked into the page" below). Kept as a live
artifact rather than a one-off chart because the user wanted to keep coming
back to it and adding to it.

## Dataset switcher

A "Dataset:" segmented control, above the existing "X-axis:" toggle, swaps
which sweep the whole page reads from. Everything else on the page (mode
toggle, zoom, configs-shown, all three charts, the summary table, point
lookup) is unchanged — it now just reads its data through one indirection
layer instead of module-level constants:

```js
const DATASETS = {
  scene_mp_4p_03: { label, title, sub, notesHtml, footerHtml,
                     SERIES, CUM, TIMING, FINALS, MAKESPANS, STATS,
                     BASELINE_FALLBACK_VAR },
  IH_mp_2p_01:    { /* same shape */ },
};
let CUM, TIMING, FINALS, MAKESPANS, STATS, SERIES, BASELINE_FALLBACK_VAR; // let, not const
function loadDataset(key){
  activeDataset = key;
  const d = DATASETS[key];
  CUM = d.CUM; TIMING = d.TIMING; /* ...etc */
  // swap page chrome: pageTitle/pageSub/datasetNotes/pageFooter innerHTML = d.*
}
```

Every render function (`renderChart1/2/3`, `renderTable`, `renderStats`,
`renderLookup*`, `computeSeriesColors`) already re-read these names fresh on
every `renderAll()` call (pre-existing behavior, needed for the mode/zoom/
visibility toggles to work at all) — so making them `let`s reassigned by
`loadDataset()` instead of one fixed set of `const`s was enough to make the
whole rest of the file dataset-agnostic with **no changes to any render
function's body**. The only things that don't get clobbered on switch:

- **Visibility and zoom are tracked per dataset** (`visibleByDataset[key]`,
  `zoomByDataset[key]`), not reset — switching away and back preserves
  whatever you'd shown/hidden or zoomed to in each one.
- **`mode` (makespan/steps) is shared globally**, deliberately — it's a
  display preference, not dataset-bound data, so it stays put across a
  switch.
- Page chrome (`<h1 id="pageTitle">`, `<p id="pageSub">`, `<div
  id="datasetNotes">`, `<footer id="pageFooter">`) is now empty scaffolding
  in the static HTML — `loadDataset()` sets `innerHTML` from the active
  dataset's `title`/`sub`/`notesHtml`/`footerHtml` strings. The "Findings
  baked into the page's own notes" prose for each dataset now lives in its
  `notesHtml` JS string, not as static `<p class="note">` markup — see
  "Extending" below for where to edit dataset-specific narrative text.
- `BASELINE_FALLBACK_VAR` is also per-dataset now (previously one flat
  object) since each dataset has its own set of non-ramp-eligible keys —
  `scene_mp_4p_03`'s `1_10k`/`2_10k` extension keys aren't pure-digit
  (`isLevelKey` is `/^\d+$/`), so they fall to this map (`--cat4`/`--cat5`)
  in the >8-visible palette branch same as `s1`/`s1_20k`, even though
  they're conceptually "level 1/2, just longer" — a simplification, not a
  bug: giving them the ramp's actual level-1/2 hue would need `isLevelKey`
  to parse a level number back out of a suffixed key, not implemented.

## What's on the page

*(Applies within whichever dataset is active — see "Dataset switcher" above.)*

Three charts, a summary table, and a point-lookup tool, all sharing one
**Per real timestep (makespan) / Per decision (steps)** toggle at the top:

1. **Cumulative tasks finished** — one line per config. In makespan mode,
   x = real elapsed simulated timestep; in steps mode, x = the *n*-th
   genuine planner decision (array index into `timeStepMetrics`), so the
   long stretches spent in forced timeout catch-up (see below) collapse
   away and you see what each scheduler did with the decisions it actually
   got to make. Its y-axis auto-rescales to whatever's visible when zoomed
   (see below), so zooming into the early burst fills the chart instead of
   staying squashed against a y-axis sized for the final total.
2. **Wall-clock cost per decision (log scale)** — one dot per logged
   decision, y = that decision's `PlannerTime` in seconds (log-scaled,
   floor 0.5s), with a dashed reference line at the 1s `--planTimeLimit`
   default. Same x-mode toggle. Y-axis stays a fixed absolute log scale
   (0.5s-1h) even when zoomed, so wall-clock cost stays comparable across
   zoom levels.
3. **Summary table** — per config, in column order: tasks finished,
   decisions logged, makespan, avg real timesteps/decision, `tp` per 1k
   steps (real-time throughput), `tp` per decision (unbounded-time
   throughput), cumulative flow/local-node match counts, worst single
   decision's wall-clock cost and where it happened, last real task
   completion, hierarchy build time. The two "@ ts" / "@ #" columns follow
   the toggle. Every `<th>` carries a `title` tooltip, and a collapsed
   "What do these columns mean?" `<details>` block under the table lists
   the same definitions inline — both sourced from one
   `summaryColumnDefs()` function so they can't drift apart.
4. **Point lookup** — numeric input + slider + preset chips; answers "how
   many tasks had each config finished by real timestep X" (makespan mode)
   or "...by decision #N" (steps mode). Uses floor/step semantics (last
   known cumulative value at or before the queried point), not
   interpolation, since task counts only change on a genuine decision.
5. **New tasks finished per step (chart 3)** — the non-cumulative
   companion to chart 1, on a **linear**, dynamically-rescaling y-axis.
   y = `TasksFinishedThisStep` at that decision, derived by differencing
   consecutive `CUM[key]` entries (`deltaSeries(key)`, no separately-
   embedded data needed). Its mark type **depends on the mode toggle**,
   unlike charts 1/2 which look the same in both modes:
   - **Makespan mode: lollipop** (a thin stem from 0 up to the value, dot
     at the top, `nearestPoint3` hover tooltip — no connected line). Real
     decisions land at irregular, config-specific real timesteps, so a
     line here would draw a meaningless slope across whatever gap of real
     time happens to separate two decisions.
   - **Steps mode: connected line** (chart 1's `findY`-interpolation +
     hover-dot + crosshair pattern). Decision index is a shared, regular
     grid across configs here, so a line is meaningful. Once a config's
     own last real decision is within the visible range, one extra point
     is appended immediately after it at value 0 (`lastX + 1e-6`, not
     exactly `lastX`, to avoid a divide-by-zero in `findY3`'s
     interpolation) — so the line visibly drops to 0 right where that
     config's run ends, instead of just stopping mid-air or (worse)
     trailing off as if more decisions might still come. It does *not*
     extend flat at 0 all the way to the chart's right edge — configs with
     very different decision counts (32 for solver 1, 2204+ for level 4)
     end their lines at correspondingly different x positions, which is
     itself informative.
   Same x-mode toggle, zoom, and configs-shown as the other two charts.
6. **Zoom control** — a number input + preset chips + reset button, right
   below the mode toggle, that sets an upper x-bound applied to all three
   charts (not the summary table or point lookup, which always operate on
   the full run). Tracked as a per-mode value (`zoomMax = {makespan: null,
   steps: null}`, `null` = full range) so switching the toggle doesn't
   clobber whichever zoom you'd set in the other mode.
7. **Configs shown (master series toggle)** — one clickable chip per config
   in the former static legend spot, now up top before any chart. Click to
   hide/show; "Show all" resets. Backed by one `visible[key]` map and a
   `visibleSeries()` helper that every render function (`renderChart1/2/3`,
   `renderStats`, `renderTable`, `renderLookupTable`, and `currentXMax`
   itself) filters through — so hiding a config removes it from literally
   everything on the page, including the "full range" denominator the zoom
   presets and axis ticks are computed from, and the makespan-mode x-axis
   max if it was the longest-running visible config (see "Mixed horizons"
   below). Refuses to hide the last visible config (no-ops the click)
   rather than leaving every chart empty. Colors also react to this: with
   10 configs defined now, showing all of them uses the ramp/accent
   fallback scheme, but hiding enough to get to 8 or fewer flips every
   visible config over to a distinct categorical hue (see "Palette" below)
   — so hiding a couple of levels to focus a comparison also gets you
   better color separation for free.

## Data source and how it was built

Each dataset's `CUM`/`TIMING`/`FINALS`/`MAKESPANS`/`STATS` are built by
walking `timeStepMetrics` in the source result JSON(s) the same way,
regardless of which dataset:

- `CUM[key]` = `[[Timestep, cumulativeTasksFinished], ...]` — running sum of
  `TasksFinishedThisStep`.
- `TIMING[key]` = `[[Timestep, PlannerTime_seconds, SchedulerSolveTime_seconds], ...]`
  — both already in seconds in the source JSON (see gotcha below).
- `STATS[key]` = `{worstDecisionSec, worstDecisionTs, worstDecisionIdx,
  lastCompletionTs, lastCompletionIdx, hierarchyBuildTime,
  totalLocalNodeMatchCount, totalFlowMatchCount}` — `worstDecisionSec`/`Ts`/`Idx`
  are the max-`PlannerTime` row; `lastCompletionTs`/`Idx` are the last row
  with `TasksFinishedThisStep > 0`; the rest are top-level fields
  (`schedulerHierarchyBuildTime`, `totalLocalNodeMatchCount`,
  `totalFlowMatchCount`) copied straight through.
- `FINALS[key]` / `MAKESPANS[key]` = `numTaskFinished` / `makespan`, top-level.

This exact reconstruction (max-`PlannerTime` row for `worstDecision*`, last
nonzero-`TasksFinishedThisStep` row for `lastCompletion*`, `round(x, 3)` for
`TIMING`/`worstDecisionSec`, `round(x, 2)` for `hierarchyBuildTime`) was
verified byte-for-byte against `scene_mp_4p_03`'s already-published `"1"`
and `"s1"` `STATS` entries before being reused for the 2026-08-31 additions
below — see `/tmp/.../scratchpad/build_dataset_js.py` from that session if
a future one needs to regenerate rather than re-derive this (that path is
this session's ephemeral `/tmp` scratchpad, not part of the repo — it will
not exist in a later session; treat this paragraph as the spec to
reimplement from, not a path to go read).

### `scene_mp_4p_03` (12 configs)

All in `outputs/scene_mp_4p_03_5000ts_sweep/`:
`scene_mp_4p_03_5000_solver6_level{1..8}.json` (keys `"1"`..`"8"`),
`scene_mp_4p_03_5000_solver1.json` (`"s1"`),
`scene_mp_4p_03_5000_solver1_s20000ts.json` (`"s1_20k"`, 20000-timestep
horizon), and, added 2026-08-31,
`scene_mp_4p_03_5000_solver6_level{1,2}_10000ts.json` (`"1_10k"`/`"2_10k"`,
levels 1-2 re-run to a 10000-timestep horizon — the run that was still
in-flight as of 2026-08-28, now landed). Horizons are mixed (5000/10000/20000
real timesteps depending on config), which is why `currentXMax`/`effectiveXMax`
take `Math.max` of `MAKESPANS[key]` over whatever's currently visible instead
of a hardcoded value (see "Mixed horizons" below) — this machinery already
handled a third distinct `simulationTime` with no changes needed when the
`_10k` configs were added.

### `IH_mp_2p_01` (7 configs, added 2026-08-31)

All in `outputs/IH_mp_2p_01_10000_7000ts_sweep/`:
`IH_mp_2p_01_10000_solver6_level{1..6}.json` (keys `"1"`..`"6"`) and
`IH_mp_2p_01_10000_solver1.json` (`"s1"`) — 10000 agents, ~7000-timestep
horizon, generated via `scripts/run_benchmarks.py --map IH_mp_2p_01
--agents 10000 --solvers 1 6 --levels 1 2 3 4 5 6 -s 7000 ...`. A
longer-horizon rerun of the map/agent-count in
`ai/auto_benchmarking_IH_mp_2p_01.md` (that sweep used 500 timesteps) — see
"Findings" below for what changed.

### Extending an existing dataset vs. adding a new one

`key` is a free-form string per dataset (not required to match across
datasets — `IH_mp_2p_01` reuses `"1"`..`"6"`/`"s1"` independently of
`scene_mp_4p_03`'s own `"1"`..`"8"`, no collision, since each dataset's
`SERIES`/`CUM`/etc. live in their own `DATASETS[key]` object). `SERIES`
(inside each dataset object, not module-level anymore — see "Dataset
switcher" above) is the place that maps a key to its display label; add a
row there plus one entry in each of the five per-dataset data objects to add
another config to an *existing* dataset (see "Extending" below for the new-
dataset case, which additionally needs a `DATASETS` entry, a switcher
button, and its own `title`/`sub`/`notesHtml`/`footerHtml`).

This is a **snapshot**, not a live view — if a source JSON changes (e.g. a
level's run gets redone), the page will not reflect that until someone
regenerates the embedded arrays and republishes.

## Implementation notes

- Single self-contained `.html` — no build step, no external requests
  (Artifacts run under a strict CSP). Charts are hand-rolled SVG via `el(tag,
  attrs)` (`document.createElementNS(SVG_NS, ...)`); tables/buttons/etc. use
  the separate `html(tag, attrs)` helper (`document.createElement`) — see
  the gotcha below for why these are two different functions, not one.
- **Palette — now computed per render, not fixed per config (redesigned
  twice: 2026-08-28 for the level-count problem, then again same day for
  visibility-aware reassignment):**
  1. First pass (levels 5/7/8 landing): a static, one-slot-per-config
     palette broke once 8 flowSolveLevels + solver 1 = 9 series, one past
     the categorical palette's 8-hue cap (the dataviz skill is explicit
     that a 9th series must never be a generated hue). Fix: flowSolveLevel
     is an **ordered magnitude** (coarsening depth), not an unordered
     category, so levels don't need 8 distinct hues — they need one
     **sequential ramp**, light->dark = shallow->deep, per the dataviz
     skill's `references/palette.md` "Sequential hue" table. Ramp steps
     250/300/350/400/450/500/550/600 (`--ramp1`=`#86b6ef` down to
     `--ramp8`=`#184f95`) were chosen as the *intersection* of the skill's
     documented ordinal-safe ranges (light: no lighter than step 250;
     dark: no darker than step 600) — one hex per step works in both
     themes with no separate dark set, and it's exactly 8 steps for
     exactly 8 levels. Baselines got fixed categorical accents instead
     (not "a 9th level" — a different algorithm).
  2. Second pass (this request): with 10 configs now defined, always
     rendering the ramp would waste the categorical palette's better
     per-series distinctiveness whenever the user has hidden enough
     configs to fit within 8 again via the "Configs shown" toggle. Fix:
     `computeSeriesColors()` (called first thing in every `renderAll()`)
     recomputes every config's color from scratch based on
     **how many are currently visible**:
     - **<=8 visible:** compact categorical assignment — walk `SERIES` in
       its fixed canonical order (levels 1-8, then `s1`, then `s1_20k`)
       and hand each *visible* one the next unused slot from `--cat1`
       (`#2a78d6` blue) through `--cat8` (`#e34948` red, the dataviz
       skill's full default 8-hue order). A config's color can therefore
       shift when *other* configs are toggled (compact reassignment, not
       a fixed per-key color) — accepted tradeoff for maximizing
       distinctiveness within whatever's shown.
     - **>8 visible:** falls back to pass 1's scheme — flowSolveLevel keys
       (`isLevelKey`, `/^\d+$/`) get their fixed `--rampN` step; baselines
       get a fixed `--catN` accent (`BASELINE_FALLBACK_VAR`: `s1`->orange
       `--cat2`, `s1_20k`->aqua `--cat3`; any further baseline key not in
       that map falls back to `--cat8` red).
     `SERIES` entries themselves now carry only `key`/`label` — no
     hardcoded `color`/`raw`. Every render function reads colors from the
     `seriesColor` map (`seriesColor[s.key]`, a `"var(--catN)"` /
     `"var(--rampN)"` string) instead of a field on the series object;
     `s.raw` was dropped entirely once this landed — inline `style="background:${...}"`
     and SVG `fill`/`stroke` attributes both already resolved `var()`
     references fine, so the separate raw-hex fallback was redundant, not
     load-bearing.
- The makespan/steps toggle is one `mode` variable; every render function
  (`renderChart1`, `renderChart2`, `renderChart3`, `renderTable`,
  `renderLookup` / `renderLookupTable`) reads it fresh and is safe to call
  at any time — toggling just calls `renderAll()` again, no
  incremental-update logic.
- **Mixed horizons:** `currentXMax(dataset)` in makespan mode used to
  return a hardcoded `5001` — broke the instant a 20000-timestep config
  was added, since it silently truncated that config's chart data to the
  first ~5000 timesteps instead of erroring or adapting. Now returns
  `Math.max(...visibleSeries().map(s => MAKESPANS[s.key]))`, so it grows to
  20001 the moment `s1_20k` is shown and shrinks back to 5000/5001 the
  moment it's hidden — exactly the "just disable it at the top" workflow
  the user described when asking for this. Steps mode was already dynamic
  this way (`max(decisions logged)` across visible series, currently level
  8's 2459) and didn't need to change. `niceTicks(xMax, 5)` (used for every
  chart's x-axis ticks, both modes) was already generic — no hardcoded
  tick list existed to break.
- **Gotcha (hit and fixed once already):** `el(tag, attrs)` builds elements
  via `document.createElementNS(SVG_NS, tag)` — it's for the SVG charts
  only. Building HTML (`<table>`/`<tr>`/`<td>`/`<button>`, etc.) with it
  creates foreign-namespace elements that don't get table layout or
  `innerHTML` parsing, and the summary/lookup tables silently render empty.
  Use the separate `html(tag, attrs)` helper (`document.createElement`) for
  anything that isn't going inside an `<svg>`.
- **Gotcha (hit and fixed once already):** the zoom `<input>`'s own `input`
  event listener re-renders charts by calling them directly
  (`renderChart1(); renderChart2();`) instead of `renderAll()`, for
  responsiveness while typing. When chart 3 was added, that hardcoded list
  wasn't updated, so typing into the zoom box moved charts 1 and 2 but left
  chart 3 stuck at the old range — `renderAll()`-based call sites (the
  reset button, the zoom preset chips, the mode toggle) all picked it up
  fine. If a 4th chart gets added, grep for every literal `renderChart1();
  renderChart2();`-style hardcoded call list (not just `renderAll()`) and
  update those too.

## Findings baked into the page's own notes (context for editing them)

Each dataset's notes now live in its own `notesHtml` JS string (see "Dataset
switcher" above), not shared static markup — edit the right dataset's
`notesHtml` in `DATASETS`, not a page-wide `<p class="note">`. What follows
is `scene_mp_4p_03`'s notes (the original set below, plus one more added
2026-08-31 for the `_10k` extensions); `IH_mp_2p_01`'s are in its own
subsection further down.

### `scene_mp_4p_03`

- **All five configs show an early burst then a long near-flat tail**:
  most task completions land in the first several hundred real timesteps,
  then `FlowMatchCount` collapses toward zero and the rest of the run is a
  trickle of isolated single completions from agents still walking toward
  a far-away already-assigned task.
- **Units correction mid-investigation**: `PlannerTime` / `SchedulerSolveTime`
  in `TimeStepMetric` are `std::chrono::duration<double>` — **seconds**, not
  milliseconds (`src/CompetitionSystem.cpp:275`,`:284-286`). An early pass of
  this analysis misread them as ms and called multi-second stalls "healthy."
  Worth remembering if working with these fields directly (e.g. in
  `visualisation/compute_throughput_metrics.py`, which doesn't currently
  surface either field).
- **Why flowSolveLevel 1 stops dead near the end**: not exhaustion — a
  single decision's background planner thread never returned before the
  simulation's total step budget ran out (`CompetitionSystem.cpp:92`,
  `while (timestep + timeout_timesteps < simulation_time)`), so the run
  just burns the remaining real timesteps waiting and gives up, leaving
  `proposed_actions` at its all-wait reset. The last logged row of *every*
  run that ends this way is not a genuine decision — it reuses the
  previous row's `SchedulerSolveTime`/etc. verbatim (confirmed by exact
  equality) — see "steps recorded" caveat below.
- **Why flowSolveLevel 2 looks "jumpy"**: not random — a genuine
  `SchedulerSolveTime` blowup (400-550 real seconds) recurs roughly every
  ~500 real timesteps, each one forcing a ~500-timestep catch-up freeze,
  followed by a short burst of fast decisions clearing the backlog.
- **flowSolveLevel 3/4 never blow their time budget** (worst decision:
  ~30s / ~3.2s respectively, across the whole run) — they taper off
  asymptotically instead of stalling, and win on real-time throughput
  (`tp_makespan`) despite doing far less work per individual decision.
- **The makespan/steps flip**: under "per decision" (steps) framing,
  the ranking inverts — solver 1 and flowSolveLevel 1-2 batch far more
  agent-task matches into each (expensive) decision and win decisively;
  flowSolveLevel 3-4 only win once wall-clock cost per decision is priced
  in. This is exactly what `tp_steps` vs `tp_makespan` in
  `visualisation/compute_throughput_metrics.py` already distinguishes (see
  "Makespan vs. 'timesteps solved'" in `ai/project_context.md`) — this page
  is effectively an interactive, per-timestep-resolution version of that
  same distinction for one specific sweep.
- Solver 1 gets the fewest decisions (32) and the lowest real-time
  throughput of the five, but the highest tasks-per-decision (~30/decision)
  — consistent with `ai/auto_benchmarking_scene_mp_4p_03.md`'s existing
  finding that solver 6 wins big on real-time throughput on this map.
- **Added 2026-08-31 — the `_10k` extensions confirm the backbone-rebuild
  finding in `ai/todo.md`**: level 1 extended from 5000 to 10000 timesteps
  finishes only **17 more tasks** (1431→1448) across the extra 5000
  timesteps — a single decision at timestep 6423 took **4966s (~83min)**,
  matching `ai/todo.md`'s documented root cause (solver 6's per-timestep
  coarse-flow solve re-solves the entire backbone graph regardless of how
  few agents/tasks need matching, so cost stays backbone-sized even once the
  surplus pool has shrunk to near-nothing late in a run). Level 2 fares
  better (1473→1519, worst decision 581s) but shows the same shape at
  smaller scale.

### `IH_mp_2p_01` (added 2026-08-31)

- **Throughput band is narrow, unlike `scene_mp_4p_03`**: tp/makespan
  ranges only ~0.46-0.55 across all 7 configs (level 3 highest, solver 1 and
  level 1 lowest) — nothing resembling `scene_mp_4p_03`'s multi-x spreads
  between configs.
- **Much narrower solver-6 margin than the original 500-timestep sweep of
  this same map/agent-count** (`ai/auto_benchmarking_IH_mp_2p_01.md`, which
  found solver 6 winning by ~4-7x). This run is ~14x longer (~7000 vs. 500
  real timesteps). **Not yet confirmed**, but worth checking against: this
  could be the same long-run backbone-rebuild-dominates-once-surplus-shrinks
  effect as the `scene_mp_4p_03` finding directly above — a short sweep
  catches solver 6 while surplus agents/tasks are still large and the coarse
  flow solve does real work, while a long one spends more of its run in the
  near-zero-surplus regime where every decision pays the same backbone-sized
  cost regardless of how little matching it actually does. If confirmed,
  this would mean **short-horizon sweeps have been systematically
  overstating solver 6's real-world advantage** wherever a long run pushes
  surplus pools toward zero — worth flagging next to the `ai/todo.md` item
  once someone deliberately tests it (e.g. a short vs. long sweep on the
  same map/config, isolating horizon as the only variable).

## How to edit this in a future session

1. Read the current source: either `outputs/scene_mp_4p_03_5000ts_sweep/throughput_dashboard.html`
   in this repo, or, if it might have diverged (someone edited the live
   page directly, or another session updated it without updating this
   copy), fetch the live version first via the Artifact tool,
   `action: "read"` with the URL above.
2. Edit the local `.html` file directly (Edit tool) — it's plain
   HTML/CSS/JS, no templating.
3. Republish with the Artifact tool: `action: "publish"`,
   `file_path: ".../throughput_dashboard.html"`, `url: "<the URL above>"`.
   Passing `url` is required to update the *same* artifact instead of
   creating a new one.
4. Copy the just-published file back over
   `outputs/scene_mp_4p_03_5000ts_sweep/throughput_dashboard.html` if you
   edited a scratchpad copy instead of this file directly, so the repo copy
   stays in sync, and update this doc if the data/columns/behavior changed
   materially.

### Extending an existing dataset with a new config

Levels 1-4 and 6 were added first (2026-08-27); levels 5/7/8 and the
20000-timestep solver 1 baseline followed on 2026-08-28, in the same batch
that forced both the palette redesign and the mixed-horizon `currentXMax`
fix described above; the `1_10k`/`2_10k` 10000-timestep extensions landed
2026-08-31 in the same session the dataset switcher was added. All eight
`--flowSolveLevel` values this repo's compile-time `kDefaultCoarsenLevels`
supports are now in `scene_mp_4p_03`. To add another config to an *existing*
dataset:

1. Confirm the result JSON exists and is complete (`makespan` present, not
   mid-run) — see `ai/project_context.md`'s makespan/steps-recorded section
   for how to tell a genuinely finished run apart from a stale/in-progress
   file.
2. Pick a new key that won't collide with an existing one *within that
   dataset* (keys don't need to be globally unique across datasets — see
   "Extending an existing dataset vs. adding a new one" above), compute
   `CUM`, `TIMING`, `FINALS`, `MAKESPANS`, `STATS` entries the same way as
   the existing ones (walk `timeStepMetrics`, see field list above) and add
   one `SERIES` entry — `{key, label}` only, no color field
   (`computeSeriesColors()` handles it automatically based on visibility,
   nothing to hand-assign) — all inside that dataset's object in
   `DATASETS`, not at module level.
   - If the new run is itself a flowSolveLevel (`isLevelKey` matches, i.e.
     the key is *only* digits), it'll automatically join the existing
     levels' ramp-mode-fallback path — no `BASELINE_FALLBACK_VAR` entry
     needed.
   - If it's a baseline/different-algorithm run, or a re-run at a different
     horizon (like `s1_20k`, `1_10k`), add an entry to that dataset's own
     `BASELINE_FALLBACK_VAR` (a `--catN` slot not already claimed by another
     baseline in the same dataset) so it has a stable fallback color when >8
     configs in that dataset are visible at once; skip this if you're fine
     with it defaulting to `--cat8` (red) in that case.
3. If a *9th flowSolveLevel* is ever benchmarked in a dataset that already
   uses all 8 ramp steps (a genuinely new depth, not a re-run at a different
   horizon): the ramp has no more dual-mode-safe steps left in its `250-600`
   ordinal range — re-read the dataviz skill's `references/palette.md`
   before picking anything past `--ramp8`, don't just eyeball a hex.
4. Re-run the palette validator (`scripts/validate_palette.js`) on the full
   set of hex values in both light and dark mode before publishing, per the
   dataviz skill (this environment doesn't have `node` installed as of this
   writing — the validator couldn't actually be run for the 2026-08-28 or
   2026-08-31 palette-touching changes; values were instead taken directly
   from the skill's documented, pre-validated tables, or reused unmodified
   from the already-validated `scene_mp_4p_03` set, rather than eyeballed).

### Adding a brand new dataset (a different map/sweep entirely)

Added 2026-08-31 for `IH_mp_2p_01`. Use this instead of the above when the
new data isn't just another config of an existing sweep — different map,
different agent count, something that shouldn't be visually compared
1:1 against what's already there (see "Dataset switcher" above for why).

1. Build the new dataset's `SERIES`/`CUM`/`TIMING`/`FINALS`/`MAKESPANS`/
   `STATS`/`BASELINE_FALLBACK_VAR` the same way as "Extending an existing
   dataset" above, for every config in the new sweep — these are independent
   of every other dataset's keys, so feel free to reuse simple keys like
   `"1"`..`"6"`/`"s1"` again.
2. Add `title` (short, used in `<h1>`), `sub` (the descriptive paragraph
   under the title — map size, what's swept, source dir), `notesHtml` (the
   dataset's own "Findings" prose, as HTML — see the "Findings" section
   above for the current text per dataset) and `footerHtml` strings.
3. Add the whole thing as a new top-level key under `DATASETS = { ... }`.
4. Add a `<button data-dataset="yourKey">label</button>` inside
   `#datasetToggle` in the static HTML. Nothing else needs to change —
   `loadDataset()` already initializes fresh `visibleByDataset`/
   `zoomByDataset` state for any key it hasn't seen before, and every render
   function already reads through the `let CUM/TIMING/...` indirection.
5. Update this doc: a new numbered dataset entry at the top, a new `###`
   subsection under "Data source", a new `###` subsection under "Findings".

## Related docs

- `ai/project_context.md` — "Makespan vs. 'timesteps solved'" section: the
  underlying mechanics this whole page visualizes.
- `ai/auto_benchmarking_scene_mp_4p_03.md` — the sweep methodology and
  earlier (non-5000-timestep) results for the `scene_mp_4p_03` dataset.
- `ai/auto_benchmarking_IH_mp_2p_01.md` — the original 500-timestep sweep of
  the `IH_mp_2p_01` dataset's map/agent-count; the dashboard's version is a
  ~14x-longer rerun (see that dataset's "Findings" above).
- `ai/todo.md` — the solver 6 per-timestep backbone-rebuild-cost finding
  that both the `scene_mp_4p_03` `_10k` extensions and the `IH_mp_2p_01`
  dataset's findings reference.
- `scripts/run_benchmarks.py` — the sweep runner used to generate the
  `IH_mp_2p_01` result JSONs this dataset embeds.
- `visualisation/compute_throughput_metrics.py` — the non-interactive,
  CSV/Markdown-table equivalent of this page's summary table, usable
  head-lessly / across many more result files at once.
