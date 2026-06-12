# Python Bindings Gap — Execution Guide

This document describes how to run the work tracked in [`ompl-python-bindings-gap.plan.md`](ompl-python-bindings-gap.plan.md) on this fork.

## Repository

| Item | Value |
|------|-------|
| Local path | `/home/joao/my_repos/forks/ompl` |
| Remote | `git@github.com:joao-pm-santos96/ompl.git` |
| Work branch | `feat/python-bindings-gap` |
| Upstream (optional) | `https://github.com/ompl/ompl` |

## Before You Start

1. **Open this repo as the workspace root** (not the upstream clone under `my_repos/ompl`).
2. **Check out the work branch:**
   ```bash
   cd /home/joao/my_repos/forks/ompl
   git checkout feat/python-bindings-gap
   ```
3. **Initialize submodules** (required for Nanobind):
   ```bash
   git submodule update --init --recursive
   ```
4. **Build and install bindings** (baseline sanity check before adding new code):
   ```bash
   pip install ./py-bindings
   pytest tests/pytests -q
   ```
5. **Read the plan:** [`ompl-python-bindings-gap.plan.md`](ompl-python-bindings-gap.plan.md)

## Branch and PR Strategy

- Do all binding work on **`feat/python-bindings-gap`** — no phase-specific branches.
- Deliver everything in **one PR** to `main` on `joao-pm-santos96/ompl`.
- Open the PR as a **draft** after the Phase 0 commit; mark it ready for review after Phase 7.
- Keep upstream OMPL sync separate: merge/rebase from `ompl/ompl` main only when needed, not mixed into this PR.

## Commit Strategy

Each implementation phase (0–7) becomes **one commit** on the work branch, committed sequentially. Run verification (below) before each commit; the tree must build and tests must pass for that phase's scope.

| Commit | Phase | Scope summary |
|--------|-------|---------------|
| 1 | 0 — Tooling and policy | `list_binding_gaps.py`, GIL docs, planner skeleton |
| 2 | 1 — Base + util | objectives, samplers, state spaces, `GoalLazySamples` fix |
| 3 | 2 — Geometric planners | classic, RRT, PRM, path utilities + smoke tests |
| 4 | 3 — Informed trees | ABITstar → BLITstar (inheritance order) |
| 5 | 4 — Control | ODE solvers, Hy*, LTL stack |
| 6 | 5 — Tools | SelfConfig, experience DBs, XXL, repair helpers |
| 7 | 6 — Multilevel | new submodule scaffold + planners |
| 8 | 7 — CI / docs | expanded pytests, stub verification, doc updates |

Commit message format:

```
feat(pybind): <phase summary>

<optional body: what changed and why>
```

Examples:

- `feat(pybind): add binding gap tooling and gil exclusion docs`
- `feat(pybind): bind base samplers objectives and util helpers`
- `feat(pybind): add multilevel submodule bindings`

## Opening the PR

- **Title:** `feat(pybind): complete missing python bindings`
- **Body:** link to [`ompl-python-bindings-gap.plan.md`](ompl-python-bindings-gap.plan.md), list the eight phase commits, note ~110 new bindings and explicit GIL exclusions from the plan.

## Implementation Order

Implement phases **0 → 7** sequentially on the work branch. Phase 0 deliverables:

1. `scripts/list_binding_gaps.py` — gap audit script
2. GIL exclusion section in `doc/markdown/python.md`
3. Planner binding skeleton under `py-bindings/templates/` (copy of RRT pattern)

Do **not** bind threading-heavy APIs listed in the plan (ParallelPlan, CForest, pRRT, etc.).

## Key Paths

| Purpose | Path |
|---------|------|
| C++ sources | `src/ompl/` |
| Nanobind bindings | `py-bindings/` |
| Binding entry point | `py-bindings/python.cpp` |
| Python tests | `tests/pytests/` |
| Binding contributor docs | `py-bindings/README.md`, `doc/markdown/pybindingsPlanner.md` |
| Plan + checklist | `docs/plans/` |

## Verification After Each Binding

```bash
pip install ./py-bindings
python -c "from ompl import base, geometric, control, util, tools; print('import ok')"
pytest tests/pytests/test_geo_planners.py -q   # or relevant test module
```

For new submodules (Phase 6 multilevel), also verify `.pyi` stubs under the build tree and `from ompl import multilevel`.

## Current Status

Verified on WSL (2026-06-12): `pip install ./py-bindings` succeeds, import smoke test passes (including `multilevel`), gap audit reports zero unintentional gaps (`python scripts/list_binding_gaps.py --fail-on-gap`), and target pytest modules pass. Deprecated tests under `tests/pytests/deprecated/` still fail due to missing resource files and legacy API expectations.

| Phase | Status |
|-------|--------|
| 0 Tooling | Done |
| 1 Base + util | Done |
| 2 Geometric planners | Done |
| 3 Informed trees | Done |
| 4 Control | Done |
| 5 Tools | Done |
| 6 Multilevel | Done |
| 7 CI / docs | Done |
