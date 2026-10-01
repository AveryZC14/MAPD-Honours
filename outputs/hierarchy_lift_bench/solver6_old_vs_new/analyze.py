import gzip, json, statistics as st, sys, os
O = os.path.dirname(os.path.abspath(__file__))
for m in ["IH_mp_2p_01", "orz900d"]:
    for v in ["old", "new"]:
        p = f"{O}/{m}_{v}.json.gz"
        if not os.path.exists(p):
            continue
        d = json.load(gzip.open(p))
        ms = d["timeStepMetrics"]
        solve = [x["SchedulerSolveTime"] * 1000 for x in ms]
        local = [x["SchedulerLocalMatchTime"] * 1000 for x in ms]
        flow = [x["FlowMatchCount"] for x in ms]
        gl = [x["GuidePathLengthSum"] for x in ms]
        plan = [x["PlannerTime"] * 1000 for x in ms]
        busy = [s for s, f in zip(solve, flow) if f > 0]
        per_agent = sum(solve) / max(1, sum(flow))
        print(f"{m:12s} {v}: decisions {len(ms)}, finished {d['numTaskFinished']}, opened {d['numTaskOpened']}, "
              f"errors {d['numPlannerErrors']}/{d['numScheduleErrors']}, timeouts {d.get('numEntryTimeouts')}")
        print(f"   flow-matched total {sum(flow)}, local total {sum(x['LocalNodeMatchCount'] for x in ms)}; "
              f"GuidePathLengthSum total {sum(gl):.0f} ({sum(gl)/max(1,sum(flow)):.0f} per flow-matched agent)")
        print(f"   SchedulerSolveTime ms: total {sum(solve):.0f}, mean {st.mean(solve):.1f}, max {max(solve):.0f}; "
              f"first step {solve[0]:.0f}; steps with flow matches: mean {st.mean(busy) if busy else 0:.1f}; "
              f"per flow-matched agent {per_agent:.1f}")
        print(f"   SchedulerLocalMatchTime ms total {sum(local):.0f}; PlannerTime ms mean {st.mean(plan):.0f}")
