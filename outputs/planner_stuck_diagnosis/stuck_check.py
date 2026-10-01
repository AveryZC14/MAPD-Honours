"""Independent check and pictures of the planner's stuck-agent samples.

For each `stuck sample:` line in a run log, re-derive from the map file alone
whether the agent's position is a Manhattan local minimum toward its goal
(every open neighbour is further from the goal than the current cell), and
draw a crop of the map around it.

usage: stuck_check.py <map file> <log file> <out png> [per_class]
"""
import sys
from collections import Counter, defaultdict
from PIL import Image, ImageDraw

map_path, log_path, out_path = sys.argv[1:4]
per_class = int(sys.argv[4]) if len(sys.argv) > 4 else 6

lines = open(map_path).read().split("\n")
h = int(lines[1].split()[1])
w = int(lines[2].split()[1])
grid = lines[4:4 + h]


def free(r, c):
    return 0 <= r < h and 0 <= c < w and grid[r][c] == "."


samples = []
for line in open(log_path):
    if not line.startswith("stuck sample:"):
        continue
    t = line.split()
    d = dict(zip(t[2::2], t[3::2]))
    loc = [int(x) for x in line.split(" loc ")[1].split()[:2]]
    goal = [int(x) for x in line.split(" goal ")[1].split()[:2]]
    samples.append({"decision": int(d["decision"]), "agent": int(d["agent"]), "class": d["class"],
                    "still": int(d["still"]), "loc": loc, "goal": goal,
                    "has_path": line.strip().endswith("1")})


def manhattan_trap(s):
    (r, c), (gr, gc) = s["loc"], s["goal"]
    here = abs(r - gr) + abs(c - gc)
    nbrs = [(r + dr, c + dc) for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)) if free(r + dr, c + dc)]
    return all(abs(nr - gr) + abs(nc - gc) > here for nr, nc in nbrs)


table = Counter()
for s in samples:
    s["geom_trap"] = manhattan_trap(s)
    table[(s["class"], s["has_path"], s["geom_trap"])] += 1

print(f"{len(samples)} samples")
print(f"{'planner class':20s} {'has_path':9s} {'map says Manhattan trap':24s} count")
for (cls, hp, gt), n in sorted(table.items()):
    print(f"{cls:20s} {str(hp):9s} {str(gt):24s} {n}")

# pictures: first `per_class` samples of each class from the latest decision
by_class = defaultdict(list)
for s in sorted(samples, key=lambda s: -s["decision"]):
    if len(by_class[s["class"]]) < per_class:
        by_class[s["class"]].append(s)
chosen = [s for cls in sorted(by_class) for s in by_class[cls]]
R, CELL = 20, 8
side = (2 * R + 1) * CELL
cols = per_class
rows = max(1, len(by_class))
img = Image.new("RGB", (cols * (side + 10) + 10, rows * (side + 34) + 10), "white")
dr = ImageDraw.Draw(img)
for ci, cls in enumerate(sorted(by_class)):
    for k, s in enumerate(by_class[cls]):
        ox, oy = 10 + k * (side + 10), 10 + ci * (side + 34)
        r0, c0 = s["loc"]
        for i in range(-R, R + 1):
            for j in range(-R, R + 1):
                col = (240, 240, 240) if free(r0 + i, c0 + j) else (40, 40, 40)
                x, y = ox + (j + R) * CELL, oy + (i + R) * CELL
                dr.rectangle([x, y, x + CELL - 1, y + CELL - 1], fill=col)
        cx, cy = ox + R * CELL + CELL // 2, oy + R * CELL + CELL // 2
        gr, gc = s["goal"]
        dist = abs(gr - r0) + abs(gc - c0)
        # arrow toward the goal (clipped to the crop)
        vr, vc = gr - r0, gc - c0
        scale = min(1.0, R / max(1, max(abs(vr), abs(vc))))
        ex, ey = cx + vc * scale * CELL, cy + vr * scale * CELL
        dr.line([cx, cy, ex, ey], fill=(0, 160, 0), width=2)
        dr.ellipse([ex - 4, ey - 4, ex + 4, ey + 4], fill=(0, 160, 0))
        dr.ellipse([cx - 4, cy - 4, cx + 4, cy + 4], fill=(220, 0, 0))
        label = f"{cls} a{s['agent']} still {s['still']}"
        label2 = f"goal {dist} away  path={int(s['has_path'])}  mapTrap={int(s['geom_trap'])}"
        dr.text((ox, oy + side + 2), label, fill="black")
        dr.text((ox, oy + side + 16), label2, fill="black")
img.save(out_path)
print("wrote", out_path)
