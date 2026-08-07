# CBF steering (`ompl::cbf`)

A **control barrier function safety filter** for a spherized robot in a workspace
signed distance field, wired into OMPL so that **collision checking can be removed
entirely**. Instead of proposing a motion and asking a collision checker whether it
was allowed, every step is *constructed* to satisfy a barrier condition — so there is
nothing left to check, and a motion that would have hit an obstacle bends around it
rather than being thrown away.

- **Headers:** `#include <ompl/cbf/…>` (header-only except `CBFControlFilter`)
- **Namespace:** `ompl::cbf`
- **Depends on:** Eigen, [`ompl::sdf`](README_SDF_USAGE.md), `ompl::robots::UR5`,
  and the vendored [qpmad](external/qpmad/VENDORED.md) QP solver (private — the
  qpmad include appears only in `cbf/src/CBFControlFilter.cpp`, never in an
  installed header).
- **Build flag:** `OMPL_HAVE_QPMAD`, set automatically when `external/qpmad` is
  present. Without it the module is not compiled.

## The idea

For each collision sphere *i* of the robot, with radius `r_i` and centre `p_i(q)`:

```
h_i(q) = d(p_i(q)) - r_i - margin          (clearance; safe when h_i >= 0)
dh_i/dq = (dp_i/dq)^T grad d(p_i)          (chain rule through the sphere Jacobian)
```

A discrete-time CBF asks each step to retain a fraction of its clearance,
`h_i(q + u dt) >= (1 - gamma) h_i(q)`, which linearises to one row per sphere:

```
(dh_i/dq) u >= -gamma h_i(q) / dt
```

The filter then solves, for a nominal control `u_nom`:

```
min  ½ (u - u_nom)^T W (u - u_nom)
s.t. A u >= -gamma h / dt                  (40 rows, one per sphere)
     u_min <= u <= u_max                   (velocity and joint limits)
```

qpmad is a **dual active set** solver: it starts from the unconstrained minimum,
which for diagonal `W` *is* `u_nom`. So a step that was already safe costs **zero
solver iterations** and returns the nominal control untouched.

## The pieces

| Header | Role |
| --- | --- |
| `ClearanceBarrier.h` | Assembles `h(q)` and the constraint rows from a robot + a `GridSDF`. |
| `ControlFilter.h` | The abstract seam: `filter(q, u_nom, dt) -> u`. `PassthroughFilter` is the identity, which is what makes with/without-CBF comparisons honest. |
| `CBFControlFilter.h` | The QP filter above. Returns `Unchanged`, `Filtered`, or `Blocked`. |
| `FilteredStatePropagator.h` | Control-space use: a `control::StatePropagator` that filters before integrating. |
| `FilteredStateSpace.h` | Geometric use: a state space whose `interpolate()` **is** a CBF rollout. |
| `FilteredMotionValidator.h` | The validator that goes with it: "did the rollout arrive?", not "is it safe". |
| `JointSteeringControlSampler.h` | A directed control sampler that actually aims at the target, unlike OMPL's random-shooting default. |

## Two ways to use it

### Geometric (recommended — this is the fast one)

The rollout lives behind `interpolate()`, so a *geometric* planner gets long-range
extensions while every intermediate state is barrier-certified. In free space the
rollout **is** linear interpolation to floating-point precision, so the space is a
drop-in `RealVectorStateSpace`.

```cpp
auto space = std::make_shared<ompl::cbf::FilteredStateSpace>(filter, stepSize, maxSpeed);
space->setBounds(jointBounds);

auto si = std::make_shared<ob::SpaceInformation>(space);
si->setStateValidityChecker(std::make_shared<ob::AllValidStateValidityChecker>(si));  // no checking
si->setMotionValidator(std::make_shared<ompl::cbf::FilteredMotionValidator>(si));
si->setup();

auto planner = std::make_shared<og::RRTConnect>(si);   // stock, unmodified
```

Because the executed motion between two waypoints is *by definition* `interpolate()`
between them, `PathGeometric::interpolate()` reconstructs the real trajectory rather
than a straight-line fiction. That self-consistency is why the rollout belongs in the
state space and not in the motion validator alone.

### Control-space

Filtering inside propagation makes the planned system the *closed-loop* one — the
robot together with the filter it will actually run. The reached state stays a
deterministic function of (state, control, steps), which is exactly OMPL's edge
model, so every stock control planner works unmodified and
`PathControl::check()` — which re-propagates and compares — passes.

```cpp
si->setStatePropagator(std::make_shared<ompl::cbf::FilteredStatePropagator>(si, filter));
si->setDirectedControlSamplerAllocator([](const oc::SpaceInformation *s)
    { return std::make_shared<ompl::cbf::JointSteeringControlSampler>(s); });
```

This is correct but **slow** — see the numbers below. A control edge is capped at
`maxControlDuration * stepSize * maxSpeed` of joint travel, so a wide motion needs
many short edges and the tree grows deep.

## Removing the collision checker

This is the point of the module, so it is worth stating plainly. A filter that
certifies every step leaves a `StateValidityChecker` nothing to do, and keeping one
means paying for the same geometry twice per step: the barrier already runs the
forward kinematics and queries the field for all 40 spheres to build its rows, and
`isSafe()` would repeat that to learn what the barrier already knows.

What you give up, all of which becomes yours to handle:

- **Anything outside the barrier's model.** The checker was the last line of defence
  against errors in the sphere approximation, the SDF, or the step linearisation.
  `ClearanceBarrier`'s margin is now the only thing covering them.
- **Start-state validation.** `Planner::checkValidity()` will accept a start state in
  collision. Check it yourself.
- **`Blocked` truncation.** A blocked step holds a *valid* state, so
  `propagateWhileValid` does not truncate — the edge runs to full length having gone
  nowhere. Counted in `statistics().blocked`.

**Audit instead of assuming.** Interpolate a returned path to every step and evaluate
an *unbuffered* barrier at each one. The demo does this (`unsafe/audited` column) and
so do the tests; that is how the buffer below was found to be necessary.

## Margins, and the buffer

`ClearanceBarrier::defaultMargin` (6 cm) absorbs three errors, none optional:

1. **Sphere under-coverage.** VAMP's sphere model does not enclose the UR5's links —
   mesh vertices sit up to **30.5 mm** outside it (`scripts/ur5_sphere_coverage.py`).
   Without this term `h_i >= 0` does not imply the real robot is clear.
2. **SDF discretisation**, up to about a third of a voxel, which can err optimistically.
3. **Step linearisation**, since the sphere centres travel on arcs.

Separately — and this bit is easy to get wrong — **a filter enforcing `h >= 0` does
not deliver `h >= 0`.** Riding the boundary overshoots it by a few mm. Use
`ClearanceBarrier::guarding()`, which buffers what the filter enforces relative to
what you intend to audit:

```cpp
const Barrier guard = Barrier::guarding(robot, field, margin);   // what the filter enforces
const Barrier audit(robot, field, margin);                       // what you hold it to
```

Measured with a 20-run audit at several buffer values: **0 leaks (7 violations up to
2.2 mm), 0.5 cm leaks (7 violations), 0.75 cm holds with 1.4 mm to spare, 1 cm holds
with 3.3 mm.** 1 cm is the default.

> **Known documentation defect.** `interpolationBuffer()` attributes this to SDF
> interpolation error and returns one voxel. That explanation is wrong: the required
> buffer is **voxel-independent** — the same 0.5 cm/1 cm boundary appears at 2, 3 and
> 4 cm grids. It is driven by the step size, not the grid. The default is the measured
> 1 cm; the doc comment still needs rewriting and the scaling law establishing.

## Measured

UR5, 6-DoF, 40 spheres, 3 cm voxel, `stepSize` 0.05 s, `gamma` 0.4, 20 runs, medians.
Scene: start and goal are the same arm shape rotated about the base, with obstacles
where the arm would be at the midpoint — so the direct joint-space motion is blocked.
Reproduce with `./build/demos/demo_UR5CBFPlanning`.

| setup | ms | vertices | steps | collision checks | unsafe |
| --- | --- | --- | --- | --- | --- |
| `geom-rrtconnect` (ordinary collision checking) | **0.149** | 6 | – | 72 | 0 |
| control RRT + checker | 38.3 | 2422 | 13404 | 13404 | 0 |
| control RRT + CBF + checker | 76.2 | 2242 | 12474 | 12475 | 0 |
| control RRT + CBF, no checker | 25.9 | 1446 | 7928 | **0** | 0 |
| geometric RRT + CBF rollout | 131.8 | 859 | 46684 | **0** | 0 |
| **RRTConnect + CBF rollout** | **0.963** | **6** | 274 | **0** | 1/2191 |

`steps` counts filter calls, and the two rollout rows only spend that many because
`FilteredStateSpace` **keeps the trajectory it produced and treats it as the edge**.
A tree planner rolls to produce a state and then asks the validator about the state it
produced; the rollout aimed at the deflected endpoint is a *different motion* from the
one that produced it, so answering that question by re-rolling was both expensive and
a re-derivation rather than a check. Now the rollout is recorded under the edge it
made and every later question — validity, densification, export — is a lookup.

Measured A/B on one machine, same binary, same arguments. MotionBenchMaker is 210
problems (30 per scene), voxel 0.02, margin 0.002, buffer 0.008, range 2.0, 5 s:

| | before | after | |
| --- | --- | --- | --- |
| MotionBenchMaker `cbf-rrtc` ms | 6.14 | **1.34** | **4.6x** |
| MotionBenchMaker `cbf-rrtc` filter calls | 1478 | **341** | **4.3x** |
| MotionBenchMaker median vertices | 13 | **8** | |
| `cage` (the hard scene) ms | 196.97 | **41.60** | **4.7x** |
| `cbf-geom-rrtc` ms (single scene) | 1.829 | **0.963** | 1.9x |
| `cbf-geom-rrt` ms (single scene) | 141.3 | 131.8 | 1.07x |

Every scene improved, by 2.0x to 7.5x. The collision-checked `rrtconnect` control row
moved 0.81 → 0.80 ms across the two runs, which is the noise floor.

The gap between MotionBenchMaker and the single-scene demo is `RRTConnect`'s goal tree.
That tree validates its edges *backwards* (`RRTConnect.cpp:143`), which no re-rolling
validator could ever answer — so every goal-tree extension paid a second rollout, and
deflected ones were rejected outright. A recorded polyline reads the same either way
round, so the goal tree works for the first time. Trees change shape as a result, so
this is not a pure like-for-like speedup.

> **Open: the audit got worse, and it is not just resolution.** `cbf-rrtc` went from
> 5/146188 unsafe (worst -0.0087) to 93/147922 (worst -0.0163), concentrated in
> `bookshelf_thin` (81) and `cage` (5). Re-auditing the *same* trajectories at step
> boundaries only still shows 39 and 4 at the identical worst depth, so this is **not**
> an artefact of `executedPath()` sampling inside steps — that roughly doubles the count
> but does not create the finding. Two candidate causes, not yet separated: the trees
> genuinely changed, or the old audit never looked at the executed motion in the first
> place. The latter is plausible and would be the more important one: densifying used to
> re-roll each edge *aimed at its stored endpoint*, which is a different, freshly
> filtered motion, and any freshly filtered motion is certified against the **buffered**
> barrier by construction — so it tends to audit clean whatever the real trajectory did.
> Settle this before trusting any pre-2026-08-06 clean `cbf-rrtc` audit.

Per-step cost, over 100k random `(q, u)`:

| | µs | |
| --- | --- | --- |
| `isSafe(q)` | 1.70 | one collision check |
| `filter(q, u)` | 2.44 | barrier rows + QP (**1.43× a check**) |

Where the extra goes, timed as nested stages over 200k random `(q, u)`: FK plus 40
sphere centers 0.46 µs, plus 40 interpolated SDF values 1.70 (this *is* `isSafe`), plus
40 interpolated gradients ~2.2, plus 40 constraint rows ~3.1, plus the solve 3.64. So
roughly **three quarters of the gap is building constraint rows and one quarter is
qpmad** — the solver is the cheapest part, costing less than half a collision check,
consistent with safe steps taking 0 inequality iterations. Optimise the rows, not the
QP: `UR5::barrierGradient()` uses the scalar triple product against
`Kinematics::jointMoment` rather than forming 40 Jacobians, which is bit-identical and
was worth 2.5× on that stage (21% end to end).

**Then do not build most of the rows at all.** `ClearanceBarrier::decreaseRates()` bounds
how fast each sphere's clearance can possibly fall — `maxGrad * sum_k maxSpeed_k * L[i][k]`,
where `L` is `UR5::leverArmBounds()` and `maxGrad` is `GridSDF::maxGradientNorm()`, both
configuration-independent — so a sphere clear by more than `rate_i * dt` cannot bind
within the step and needs no gradient, no Jacobian and no row. Measured over random
configurations, that leaves **0.8 rows of 40** and an identical control every time, taking
`filter()` from 3.69 to **2.44 µs**. It is on by default (`Parameters::screening`).

The bound is a Lipschitz argument, not a linearisation: `L[i][k]` bounds sphere i's
distance to joint k's axis at *every* configuration, so integrating it bounds the true
change in `h` over the whole step rather than predicting it. What screening gives up is
the CBF *decay* condition for the spheres it drops — they are guaranteed to stay safe, but
not to decay at rate `gamma`. Safety is the invariant; the decay rate is a smoothness
preference. `screening = false` restores the exact original semantics.

**A filtered step still cannot beat a bare collision check** — even fully screened it
computes everything `isSafe()` does, and the remaining 0.74 µs is the floor for
certifying rather than merely testing. The CBF wins on edges it does not waste, not on
cost per step. The 8.0× gap that remains to ordinary
RRTConnect is fully accounted for by 419 filter calls versus 72 collision checks: the
rollout must integrate at `dt`, while a collision checker samples a straight line at
whatever resolution it likes.

## On a standard benchmark

The scene above is one the bar solves in six vertices, so it cannot show whether the
rollout earns its cost. `demos/UR5MBMBenchmark.cpp` runs the same comparison over
**MotionBenchMaker's UR5 set** (689 valid problems, 7 scenes, as shipped by VAMP):

```
./scripts/mbm_to_scenes.py /path/to/vamp/resources/ur5/problems.json scenes.txt
./build/demos/demo_UR5MBMBenchmark scenes.txt 15 2.0 0.02 0.05 2.0 0.002 0.006 0.0006
```

Every obstacle there is a box or a cylinder, both of which have an exact closed-form
signed distance, so the field is exact up to the grid with no mesh and no FCL.

15 problems per scene, medians, both rows audit-clean at the same 0.02 rad resolution:

| scene | rrtconnect ms | cbf-rrtc ms | rrtc evals | cbf evals | rrtc vertices | cbf vertices |
| --- | --- | --- | --- | --- | --- | --- |
| bookshelf_small | 7.53 | **2.93** | 4443 | 940 | 23 | 10 |
| bookshelf_tall | 12.14 | **6.54** | 7147 | 2004 | 34 | 16 |
| bookshelf_thin | **5.84** | 6.65 | 3371 | 1833 | 17 | 14 |
| box | 6.80 | **3.81** | 4058 | 1087 | 21 | 12 |
| cage | 674.03 | **229.24** | 385881 | 62667 | 2276 | 391 |
| table_pick | **2.15** | 1.81 | 1301 | 662 | 8 | 8 |
| table_under_pick | **3.63** | 5.19 | 2189 | 1559 | 12 | 13 |
| **all (105)** | 6.92 | **5.08** | 4023 | **1455** | 21 | **13** |

Both solve 105/105 with zero unsafe states. The rollout wins 5 scenes of 7, uses fewer
evaluations on all 7, and is **2.9× faster on `cage`** — the tight one — with 5.8× fewer
vertices. Screening changed no planner decision anywhere in this table: identical
evaluation counts, identical vertices, identical clearances, still zero unsafe. `cage`
gains least from it (7%) precisely because it is cluttered enough that spheres really are
close to binding, which is the screen reporting the truth rather than failing. That is the edges-not-wasted claim, measured on a problem set that was not
chosen to suit it.

### Two things that flip this result

Both were wrong in earlier runs of this comparison, in the flattering direction, so they
are worth stating:

- **OMPL's default `longestValidSegmentFraction` is 0.01, which on this space is 0.31 rad
  between collision checks** — an order of magnitude coarser than the audit. At the
  default the baseline runs in 1.12 ms and looks 6× faster, while leaving 131 audited
  states in collision, the worst by 34 mm. It is not safe at that resolution; it just is
  not being asked. Tighten it (0.0006 here) until the baseline is audit-clean before
  comparing anything.
- **Audit both rows at the same resolution.** `PathGeometric::interpolate()` with no
  argument uses that same fraction, which sampled the rollout 7× more coarsely than the
  baseline and reported a spurious zero. Pass an explicit state count.

### The margin does not fit this benchmark

MotionBenchMaker endpoints are grasp poses. Measured over the set, `min(start, goal)`
clearance at zero margin has a **median of ~8 mm** on five of the seven scenes (`cage`
21 mm, `box` 117 mm). Every problem is feasible at margin 0 — which confirms the sphere
model and `vampBasePose()` are aligned — but `defaultMargin` (0.06 m) admits only ~15% of
the set — and the filter's guard buffer (measured at ~1 cm above, and one voxel as
`interpolationBuffer()` still reports it) is on its own comparable to the 8 mm available.

So the run above uses `margin 0.002`, `buffer 0.006`, `voxel 0.02`, which was audit-clean
over all 105 problems. Not monotonically, though: 0.004 and 0.008 each left 3 states short
by a few mm, so which buffer is clean depends on the paths found and this is a sample, not
a bound. The numbers are therefore *not* a claim about mesh-level safety. Both rows use the same spheres, so the
under-coverage is orthogonal to the comparison — but closing it needs a tighter sphere
decomposition, not a bigger margin, since inflating radii and adding margin are the same
geometry.

## Validation

- Forward kinematics cross-checked three ways: vs `vamp.ur5.fk` **2.67e-06 m**, vs
  PyBullet **2.62e-07 m**, analytic Jacobian vs central differences **2.51e-10**.
- 52 test cases across 6 binaries (`test_ur5`, `test_clearance_barrier`,
  `test_qpmad_vendored`, `test_cbf_control_filter`, `test_filtered_propagator`,
  `test_filtered_state_space`).
- Filtered clearance saturates at 0.04369 against a promised floor of 0.04399 —
  0.3 mm of linearisation error. Single-active-row projections match the closed form
  `u = â·b/|a|` exactly. Safe steps cost 0 qpmad iterations.

## Modularity

**The environment is genuinely swappable.** `ompl::sdf::GridSDF` depends on Eigen and
one callback, `std::function<double(const Eigen::Vector3d &)>`. Everything above takes
`const GridSDF &`, so a new scene is a new lambda — analytic primitives, a mesh/FCL
signed-distance query, a point cloud, another field. Caveats: the bake is
all-or-nothing (no incremental update, so a *changing* environment means rebaking),
and `GridSDF` clamps out-of-bounds queries **optimistically**, which is why
`inBounds` is a first-class output that the filter treats as "no usable barrier".

**The robot is not.** `ControlFilter::Configuration` is typedef'd directly to
`robots::UR5::Configuration`, `ClearanceBarrier::Robot` is `robots::UR5`, and 6 joints
/ 40 spheres are **compile-time template parameters of the solver**
(`qpmad::SolverTemplate<double, nJoints, 1, nSpheres>`). Changing robots means editing
four files. `ClearanceBarrier`'s header names the seam — sphere centres, sphere
Jacobians, radii — but it is not built. In cost order: move joint limits into
`Parameters` (`CBFControlFilter.cpp:50` reaches past the barrier straight to
`robots::UR5::lowerBounds()`, which is a wart regardless); parameterise
`Configuration` by dimension; template `ClearanceBarrier` on the robot; make
`nSpheres` dynamic in qpmad.

The planner-facing layer *is* modular: `FilteredStatePropagator` and
`FilteredStateSpace` take a `const ControlFilter &` and never look inside.

## Open items

- **`RRTConnect.cpp:294` is `while (gsc == ADVANCED)` with no termination-condition
  guard.** It assumes every ADVANCED closes the gap by `maxDistance_` — true of a
  straight line, false of a rollout that slides along a boundary, which **hangs**
  rather than timing out. Worked around in `FilteredStateSpace::interpolate()`: an
  extension achieving less than `minProgressFraction` of its free-space progress is
  reported as no motion, turning the endless ADVANCED into TRAPPED
  (`statistics().abandoned`).
- **A Lipschitz fast path is the obvious next speedup.** Since
  `|dh_i/dq| <= sqrt(6) · maxReach = 2.768` and `|grad d| <= 1`, a step with
  `min_i h_i > 2.768 |u| dt` (= 0.17 m) cannot activate any constraint, so the QP
  provably returns `u_nom` — skip the gradients, the Jacobians and the solve.
  Measured **61.3% skippable** on a real RRTConnect workload (not the 85.5% that
  uniform random sampling suggests — the planner concentrates work near obstacles).
  Worth **1.33×** bolted on, **1.64×** if `evaluate()` is restructured to reuse the
  values pass. Two things to settle first: `|grad d| <= 1` holds for a true SDF but an
  interpolated gradient can exceed unit norm, which would make the bound unsound; and
  `sqrt(6) · maxReach` is crude — the base sphere cannot move at all yet is often the
  worst sphere, so per-sphere constants precomputed offline would raise the rate for
  free.
- **The margin is blunt.** A tighter per-sphere bound follows from the Jacobian column
  norms, but wants validating against brute-force rollouts first.
- **Benchmarks are indicative, not rigorous.** OMPL only honours a seed set before the
  first random draw, so runs **cannot be paired by seed**; these are medians over
  independent draws with no variance reported.

## Demos, tests, scripts

```sh
# The comparison table above, plus a solution path dumped for the viewer.
./build/demos/demo_UR5CBFPlanning 5 20 /tmp/path.json 1.0

# Sphere model, FK, and a barrier preview at three configurations
./build/demos/demo_UR5Sphere

# Replay a planned path over the real meshes in PyBullet
python scripts/ur5_sphere_viz.py /tmp/path.json --gui --loops 0
python scripts/ur5_sphere_viz.py /tmp/path.json --screenshot frame.png --config 70

# Regenerate the C++ sphere table from ur5_spherized.urdf, validating against vamp
python scripts/generate_ur5_spheres.py --emit

# How far mesh vertices poke outside their link's spheres (the 30.5 mm above)
python scripts/ur5_sphere_coverage.py
```

Every `demo_UR5CBFPlanning` argument is positional and optional:

| # | argument | default | notes |
| --- | --- | --- | --- |
| 1 | seconds | 5 | per-run planning time limit |
| 2 | runs | 10 | medians are taken over these |
| 3 | `dump.json` \| `-` | none | writes the densified solution path for the viewer |
| 4 | obstacleScale | 1.0 | inflates the obstacles; >2 is where the free space gets tight |
| 5 | buffer | 0.010 | filter margin above the audited one; **0 and 0.005 leak** |
| 6 | voxel | 0.03 | SDF resolution |
| 7 | stepSize | 0.05 | rollout/propagation step; a correctness parameter, not a speed knob |
| 8 | maxSteps | 10 | max propagation steps per control edge |
| 9 | range | 2.0 | geometric planner extension range; a flat knob, 0.5–8 all work |
| 10 | rowMask | 63 | bitmask: 1 geom-rrtconnect, 2 collision-check, 4 cbf+check, 8 cbf-only, 16 cbf-geom-rrt, 32 cbf-geom-rrtc |

The row mask matters in practice: `cbf-geom-rrt` is ~250 ms a run, so a sweep over
one of the other knobs wants `32` (or `33`) rather than all six rows.

### What the viewer shows

Gold spheres are the C++ collision model, grey is the URDF mesh, red is the scene the
planner avoided. The dump carries an `obstacles` array so the two cannot disagree, and
the camera frames the whole trajectory plus the obstacles. `--screenshot` renders one
configuration offscreen with a hand-rolled PNG writer (no imaging dependency), which
is the useful mode over ssh.

The overlay is also the FK regression check: PyBullet runs its own forward kinematics
from the same URDF, and the script prints the worst disagreement over every
configuration × every sphere. It should be ~1e-07 m. Two things are visible by eye at
the closest approach — the CBF moving joints the nominal control never asked for, and
grey mesh escaping between the forearm spheres, which is the 30.5 mm the margin covers.
