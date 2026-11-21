# Sokoban Solver — push-optimal, room by room

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/ardimakefun/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/ardimakefun/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A single-file Node 22 program (`main.ts`, no dependencies) that solves all 5
Sokoban levels of a set in as few pushes as possible — push-optimal on every
level it has been tested on.

## How it works

Every level the generator draws is a hallway with 1–4 rooms off it, and a box
pushed through a door into the hallway can never come back — so each room is
an independent problem and the level's optimum is the sum of its rooms'
optima. The solver exploits exactly that:

1. **Decompose** — cut the dead squares of the hallway row; each connected
   component holding boxes is one room, with its own boxes and goals.
2. **Solve each room push-optimally** with A* over pushes. A state is the
   sorted box squares plus the player's reachable region (named by its lowest
   square). The heuristic is the minimum-cost box→goal matching (Hungarian)
   over grid distances ignoring other boxes — admissible, so the first solved
   state is optimal. Dead squares (reverse-pull reachability from the goals)
   and frozen 2×2 blocks are pruned.
3. **Stitch** — the player walks between rooms (shortest path), and every
   solution is replayed on the original level before it is written out.

Fallbacks, in order, if a room's optimal search ever gives up: whole-level
A*, then a greedy best-first search for any solution, then an empty (but
still valid) answer. All budgets are counted in work (expansions), never in
wall-clock time, so two runs write byte-identical output.

## Results (measured locally with the kit's scorer)

| Set | Score | Notes |
|---|---|---|
| Practice set (kit) | **1,000,000** | all 5 levels in exactly P* pushes (13, 16, 29, 47, 60); ~0.5 s |
| House baseline plain-astar 1.0.0 on the same set | 200,000 | only i001 solved; i002–i005 empty; ~5.7 s |
| 30 freshly drawn sets (150 unseen levels) | **1,000,000 average** | every level in exactly P* pushes; slowest set 0.9 s of the 60 s budget |

Determinism verified: two runs write byte-identical `solution.txt` files.

## Run it

```sh
mkdir -p out
MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

Reads `/input/<id>/level.txt` (or `MAKEFUN_INPUT_DIR`), writes
`/output/<id>/solution.txt` (or `MAKEFUN_OUTPUT_DIR`). Built by an AI agent
for make.fun's "Build the best Sokoban solver, 21 November 2026" bounty.
