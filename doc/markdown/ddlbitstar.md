# Dual-Density Lock BIT* {#ddlbitstar}

`ompl::geometric::DDLBITstar` implements the sampling and perception mechanisms
of Song et al., *DDL-BIT*: An Improved Batch Informed Trees Algorithm Using
Dual-Density Lock for Motion Planning*, IEEE Access 14 (2026), 102542–102557,
[DOI 10.1109/ACCESS.2026.3710692](https://doi.org/10.1109/ACCESS.2026.3710692).
It reuses BIT*'s search, lazy edge checking, rewiring and pruning.

## Supported problems

Use finite bounded `RealVectorStateSpace`, of any positive dimension. Rotations,
compound spaces, constrained manifolds and just-in-time sampling are unsupported
and rejected explicitly. Density keys use the floor of each coordinate divided
by the grid width, including negative coordinates. Bounds must be representable
by signed grid indices. Collision validity is supplied through the normal OMPL
state validity checker and motion validator.

## Sampling and perception

Before each batch, scout from randomly chosen retained graph states. In 2D,
directions are evenly spaced with a random angular offset; higher dimensions use
directions uniformly distributed on the unit sphere. March at the state space's
validity resolution, then bisect the first invalid transition twelve times.
Keep its free-side point, deduplicating by boundary-grid cell. Each ray is
limited to 4096 validity intervals to bound work on large domains; thin features
still require an appropriate validity resolution.
Scouting honors the segment-count factor and checks the active termination
condition during traversal, bisection and sample generation. BIT* may finish a
pending edge check and exhaust its current finite sampling-attempt budget after
termination; a user-supplied collision check cannot be interrupted internally.

Choose boundary Gaussian proposals with probability 0.2 before a solution and
0.5 afterward; otherwise use the objective's informed sampler. Reject proposals
outside the bounds, informed cost interval, or free space. The Gaussian covariance
is isotropic. Samples from the two sources use capacities 3 and 1 respectively.
For occupancy `N` and source capacity `tau`, admission probability is
`1 / (1 + 2 * max(0, N - tau + 1))`. This is a soft filter: saturated cells
can continue accepting states. Only valid admitted samples increase occupancy.
The density grid is reconstructed from retained graph states at the next batch
following a solution improvement. `clear()` frees perception state and resets
statistics, while retaining parameters.

The paper's prose says scouting runs on initialization/stagnation, but Algorithm
2 also triggers it on every new batch. This implementation follows the new-batch
trigger. The paper leaves the choice of the current scouting state and ray
traversal implementation unspecified; the choices above are OMPL adaptations.
Occupancy reconstruction happens before the next sampling batch. Boundary points
remain available across improvements; every Gaussian candidate is checked against
the current informed interval. State validity is checked before density admission
and again by the existing BIT* graph insertion path.

## Scale and use

The defaults reproduce Table 1's values: batch size 50, rewiring factor 1.1,
density cell width 15, boundary cell width 2, Gaussian standard deviation 10,
18 scouting rays, and the weights/capacities described above. Spatial settings
are in problem units: the paper uses 500-unit maps. On a map bounded by [-1,1],
use smaller widths and standard deviation, for example:

```cpp
auto planner = std::make_shared<ompl::geometric::DDLBITstar>(si);
planner->setGridResolution(0.15);
planner->setBoundaryResolution(0.025);
planner->setBoundaryStandardDeviation(0.08);
```

The same class and parameter methods are available in `ompl.geometric` Python
bindings. Inherited BIT* settings remain available, except just-in-time sampling.
Call `clear()` before changing DDL sampling parameters on an initialized planner.
To increase both capacities, raise the boundary capacity first; it must always
exceed the positive open capacity.

## Guarantees and comparison

The uniform proposal component and soft admission probabilities preserve
exploration support. BIT*'s existing connection rules are reused with a
nonuniform accepted distribution. This implementation does not assert a separate
proof of asymptotic optimality, and the paper's performance results are not
assumed to transfer to OMPL. In particular, nonzero proposal probabilities alone
are not a proof that BIT*'s uniform-RGG connection bounds remain sufficient.

Build the `demo_DDLBITstar` comparison and run each planner in a fresh process
with the same scenario, seed and time budget:

```sh
./build/demos/demo_DDLBITstar BITstar passage 1 1.0
./build/demos/demo_DDLBITstar DDLBITstar passage 1 1.0
```

The demo also accepts `RRTstar`, `RRTConnect` and `InformedRRTstar`. Scenarios
`open`, `passage`, `narrow`, `maze`, `surround`, `clutter` and `blocked` use
[-1,1]^2; `passage6d` extrudes a slit across four additional real coordinates.
All planners use the same analytic rectangle/circle motion validator, with
independent dense returned-path checks. The deadline uses `steady_clock`.

The CSV reports exact success, path validity, first-solution time/cost, final
cost, retained states, batches, DDL statistics, state-checker callback counts,
motion-validator calls, and costs at 50/100/250/500 ms. RRT-Connect stops at its
first returned solution; it does not optimize within the remaining budget.
For the other planners first-solution times come from improvement callbacks.
The callback count excludes endpoint predicates inside the motion validator.

DDL spatial values are scaled from the paper by 2/500: density width 0.06,
boundary width 0.008 and sigma 0.04. RRT*/Informed RRT* use range 0.1, goal bias
0.1 and rewiring factor 1.1; RRT-Connect uses range 0.1. Repeat with multiple
seeds. These fixtures and validator choices are not a reproduction of the
paper's FCL environments or SE(3) experiments.
