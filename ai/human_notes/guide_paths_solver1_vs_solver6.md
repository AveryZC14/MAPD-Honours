# Guide paths: does solver 1 provide them, does solver 6, and are they useful without traffic-weighted edges

## Does solver 1 provide guide paths?

Yes, conditionally. In `schedule_plan_flow` (`scheduler.cpp:660`), recovering
each agent's task assignment requires "walking" the min-cost-flow solution
node-by-node from the agent to whichever task node absorbed its flow
(`scheduler.cpp:827-861`) — this walk is mandatory, not optional, because
it's literally how the assignment is read out of the flow solver. As a side
effect, that walk produces a concrete path. That path only gets written into
the shared `agent_guide_path` map (`scheduler.cpp:871-872`) if
`use_traffic && env->curr_timestep >= 100` — but computing it costs nothing
extra either way, since the walk already had to happen.

## Does solver 6 ever provide guide paths?

Also yes, conditionally — the same gate:
`use_traffic && env->curr_timestep >= 100` (`scheduler.cpp:991`). But there's
an important asymmetry: for solver 6, that gate controls whether the
*entire* coarse-to-fine "lifting" pipeline (Steps 3-4 of
`compute_reduced_assignment`) runs at all. When the gate is false,
`compute_reduced_assignment` returns right after Step 2 (the cheap top-level
flow solve) and never computes a fine-grained path for anyone. This
early-return was added deliberately as part of the OOM fix (it's real,
unconditional wasted work to compute paths nobody reads) — but the
consequence is that solver 6's guide-path machinery is not just "rarely"
exercised, it is **never** exercised in any benchmark that's been run in this
repo so far, because none of them passed `--useTraffic`. The suspicion that
prompted this question was exactly right, and the reason is structural, not
incidental: every prior run (the memleak-fix verification runs, and the full
15-run `orz900d` sweep) explicitly left traffic off.

## What would be "the benchmark" for solver 6's guide paths?

There isn't one yet, because nothing has ever turned the feature on. To
exercise it at all you need `--useTraffic` *and* to run past timestep 100
(both solvers share that warm-up threshold verbatim). No code change is
required to switch it on — it's a CLI flag away — but worth flagging: the
Step 3-4 lifting code has never been run under real conditions since the
bug-fix pass (bugs 1 and 2 in `ai/claude_memleak_fixes.md` were specifically
about this lifting path misbehaving), and all of that fixing/verification was
done with traffic off. So "turn on `--useTraffic` and see what happens" is
itself the first real test this code will have had post-fix — treat the
first such run as a correctness check (watch for the old symptoms:
fallback-Dijkstra firing constantly, RSS growth) before trusting its numbers,
not just as a throughput comparison.

## Are guide paths useful for the planner without dynamically-weighted edges?

No, and this is the deeper point. Two independent things are gated by the
same `use_traffic` flag:

1. Whether the scheduler's own edge costs reflect congestion
   (`scheduler.cpp:782-797`: cost is a flat `1` per edge when `use_traffic`
   is false, congestion-weighted from `background_flow` when true).
2. Whether the resulting path gets handed to the low-level planner at all.

Meanwhile, the low-level planner (`planner.cpp`) is **never** without a
path — when `agent_guide_path` has no entry for an agent, it falls back to
`update_traj()` (`flow.cpp:162`), which runs its own A* using `lns.flow`, the
planner's *own* independent congestion field built from other agents' live
trajectories (`get_opened_flow`, `planner.cpp:33-60`), completely decoupled
from whatever the scheduler did. And regardless of where an agent's path came
from, `frank_wolfe()` (`flow.cpp:96`) iterates on *all* agents' trajectories
every timestep to reduce predicted congestion. So the scheduler-provided
guide path was never the only source of congestion-awareness — it's only
ever a **seed trajectory** that lets the planner skip one initial A* call for
that agent, before Frank-Wolfe potentially reworks it anyway.

So: with traffic off (every run so far), the scheduler's edges are
unweighted, its "guide path" is just an unweighted shortest path, the
planner's own unweighted BFS heuristic table would find something equally
good, and the fallback `update_traj` A* isn't meaningfully worse — there's no
information in the seed that the planner didn't already have access to. The
entire value proposition of solver-6 guide paths (skip an expensive lift,
hand over a *better-than-naive* congestion-aware seed) only exists once
traffic weighting is switched on. Without it, the feature is inert by
design, not merely untested.

---
*Condensed technical version also lives in `ai/project_context.md` under
"Guide paths: which solvers provide them, and are they doing anything".*
