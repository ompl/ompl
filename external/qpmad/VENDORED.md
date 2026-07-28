# Vendored qpmad

[qpmad](https://github.com/asherikov/qpmad) is a header-only, Eigen-based
implementation of the Goldfarb-Idnani **dual active set** algorithm for
quadratic programming. It is the QP solver behind the CBF steering step.

- **Upstream:** <https://github.com/asherikov/qpmad>
- **Commit:** `11602ec9f87a62a97ba8318ebd81da6646f3c631` (2026-07-22)
- **License:** Apache 2.0 — see `LICENSE` and `NOTICE` (both copied verbatim)

## Why vendored rather than a submodule

qpmad is 140 KB of headers with no build step and no dependency beyond Eigen,
which OMPL already requires. Copying it in means the CBF module builds from a
plain checkout, with no `git submodule update --init` step to forget.

## What was copied

Only `include/qpmad/*.h`, plus `LICENSE` and `NOTICE`. Upstream's tests, docs,
CMake package files, Makefiles, and MATLAB/Octave bindings are omitted.

`include/qpmad/config.h` is **not** from upstream: upstream generates it from
`config.h.in`, which resolves three `#cmakedefine` options that all default to
OFF. The vendored file therefore defines nothing, and documents the three
options so they can still be set on the compile line.

## Updating

Copy `include/qpmad/*.h`, `LICENSE`, and `NOTICE` from a fresh checkout, leave
`config.h` alone (or regenerate it if upstream adds an option), and update the
commit hash above.

## Why a dual active set solver suits this problem

The CBF steering QP is tiny and *usually inactive*: 6 variables (one per joint),
up to 40 one-sided clearance constraints, and a diagonal positive-definite
Hessian. A dual active set method starts from the unconstrained minimum — which
for a diagonal Hessian is exactly the nominal control — and activates
constraints one at a time only as they are violated. So a step that is already
safe costs almost nothing, and there is no need to pre-select which spheres
might matter: all 40 rows can be handed over, and the solver finds the active
ones itself.
