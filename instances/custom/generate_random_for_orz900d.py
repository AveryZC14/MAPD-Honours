#!/usr/bin/env python3
#AI GENERATED
"""
Generate agent file, task file and instance JSON for `instances/custom/maps/orz900d.map`.
Creates:
 - instances/custom/agents/random_10000.agents
 - instances/custom/tasks/random_10000.tasks
 - instances/custom/random_10000.json

Usage: python3 instances/custom/generate_random_for_orz900d.py

The script treats any map character that is NOT '@' as traversable.
"""
import os, sys, json, random

random.seed(42)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INST_DIR = os.path.join(ROOT, 'custom')
MAP_PATH = os.path.join(INST_DIR, 'maps', 'orz900d.map')
AGENTS_DIR = os.path.join(INST_DIR, 'agents')
TASKS_DIR = os.path.join(INST_DIR, 'tasks')

AGENTS_FILE = os.path.join(AGENTS_DIR, 'random_10000.agents')
TASKS_FILE = os.path.join(TASKS_DIR, 'random_10000.tasks')
JSON_FILE = os.path.join(INST_DIR, 'random_10000.json')

TEAM_SIZE = 10000
# ensure tasks >= 1.5 * agents (plus a small buffer)
import math
NUM_TASKS = max(10000, int(math.ceil(1.5 * TEAM_SIZE)) + 100)
NUM_TASKS_REVEAL = 1.5
VERSION = "2024 LoRR"

os.makedirs(AGENTS_DIR, exist_ok=True)
os.makedirs(TASKS_DIR, exist_ok=True)

# Read map and collect traversable cell indices (non-'@' treated as free)
if not os.path.exists(MAP_PATH):
    print('Map not found at', MAP_PATH, file=sys.stderr)
    sys.exit(2)

with open(MAP_PATH, 'r', encoding='utf-8') as f:
    lines = [l.rstrip('\n') for l in f]

# Parse header
# Expecting a format like:
# type octile
# height H
# width W
# map
# <H rows>
try:
    height = None
    width = None
    map_start = None
    for i,l in enumerate(lines[:20]):
        if l.startswith('height'):
            height = int(l.split()[1])
        if l.startswith('width'):
            width = int(l.split()[1])
        if l.strip() == 'map':
            map_start = i+1
    if height is None or width is None or map_start is None:
        raise ValueError('Invalid map header')
except Exception as e:
    print('Failed to parse map header:', e, file=sys.stderr)
    sys.exit(3)

map_rows = lines[map_start:map_start+height]
if len(map_rows) < height:
    print('Warning: expected', height, 'map rows but found', len(map_rows), file=sys.stderr)

free_cells = []
for r, row in enumerate(map_rows):
    # pad/truncate to width
    if len(row) < width:
        row = row.ljust(width, '@')
    elif len(row) > width:
        row = row[:width]
    for c,ch in enumerate(row):
        # treat only '.' as traversable (other symbols like 'T','E','S','@' are blocked)
        if ch == '.':
            free_cells.append(r * width + c)

if not free_cells:
    print('No free cells found in map (no char != "@")', file=sys.stderr)
    sys.exit(4)

print('Map dims:', width, 'x', height, '=> total cells', width*height, 'free cells:', len(free_cells))

# Agents: pick unique if possible, otherwise allow sampling with replacement
if len(free_cells) >= TEAM_SIZE:
    agents = random.sample(free_cells, TEAM_SIZE)
else:
    agents = [random.choice(free_cells) for _ in range(TEAM_SIZE)]

# Tasks: generate pickup,drop pairs (pickup != drop)
tasks = []
for i in range(NUM_TASKS):
    p = random.choice(free_cells)
    d = random.choice(free_cells)
    # ensure difference
    if d == p:
        # try a few times
        for _ in range(5):
            d = random.choice(free_cells)
            if d != p:
                break
        else:
            # fallback: pick next index mod
            idx = (free_cells.index(p) + 1) % len(free_cells)
            d = free_cells[idx]
    tasks.append((p, d))

# Write agents file
with open(AGENTS_FILE, 'w', encoding='utf-8') as f:
    f.write('# generated agents for orz900d\n')
    f.write(str(TEAM_SIZE) + '\n')
    for a in agents:
        f.write(str(a) + '\n')

# Write tasks file
with open(TASKS_FILE, 'w', encoding='utf-8') as f:
    f.write('# generated tasks for orz900d\n')
    f.write(str(NUM_TASKS) + '\n')
    for p,d in tasks:
        f.write(f"{p},{d}\n")

# Write instance JSON
instance = {
    'mapFile': 'maps/orz900d.map',
    'agentFile': 'agents/random_10000.agents',
    'teamSize': TEAM_SIZE,
    'taskFile': 'tasks/random_10000.tasks',
    'numTasksReveal': NUM_TASKS_REVEAL,
    'version': VERSION
}
with open(JSON_FILE, 'w', encoding='utf-8') as f:
    json.dump(instance, f, indent=4)

print('Wrote:')
print(' -', AGENTS_FILE)
print(' -', TASKS_FILE)
print(' -', JSON_FILE)
print('Done.')
