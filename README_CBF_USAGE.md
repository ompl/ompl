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
- **Self-collision *used* to be here.** It no longer is: the barrier carries a row per
  sphere pair in `UR5::selfPairs()` and the arm is constrained against itself in the
  same QP. See below for what that does and does not guarantee.
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

## The arm against itself

A workspace field cannot see the arm folding through itself, so self-collision is a
second family of rows on the *same* barrier — a pair is a sphere whose obstacle is
another sphere:

```
h_ab(q) = |p_a(q) - p_b(q)| - r_a - r_b - margin_ab - selfMargin
row     = n^T (J_a - J_b),   n = (p_a - p_b) / |p_a - p_b|
```

`margin_ab` is per pair and lives in `UR5::SelfPair`; `selfMargin` is a global offset on
top of it, and defaults to zero. The per-pair term is what makes this sound against real
geometry rather than against the sphere model alone — see **What it guarantees** below.

Everything downstream is indifferent to which kind a row is: one flat index over
`nConstraints = nSpheres + nSelfPairs`, one screening pass, one certificate. The QP
grew by a capacity constant and nothing else. `RRTConnect` is untouched.

**Only 303 of the 780 pairs get a row**, and pruning them is most of the work.
`scripts/generate_ur5_self_pairs.py` keeps a pair only if its clearance *crosses* a
40 mm band **around that pair's own margin** somewhere in configuration space, dropping the
rest for two opposite reasons:

- **Never separates** (`max h < band`). The geometry holds these in permanent
  near-contact — `wrist_1` sphere 16 against `wrist_3` sphere 22 is two joints apart and
  never clearer than 10.3 mm — so the row is either infeasible everywhere or binding
  everywhere, and no planner can act on it. This is what an SRDF allowed-collision matrix
  would say, measured rather than assumed. At 50 mm **none of the 33 dropped pairs
  straddles zero**: each either always overlaps, so no barrier could have helped, or never
  overlaps at all. Past ~80 mm that stops being true (spheres 0 and 2 run from −23.2 mm to
  77.0 mm) and the script refuses to emit. The band had to come down from 50 mm to 40 mm
  once the margins arrived: `forearm` sphere 14 against `wrist_2` sphere 21 needs a 44.7 mm
  margin and is never more than 90.0 mm clear, so at 50 mm it was dropped as
  never-separating while still able to breach its own margin — the script caught it and
  refused, which is what that check is for.
- **Never approaches** (`min h > band`). Not merely wasted work: `certifiedDuration()`
  minimises over *all* rows, so a vacuous row shortens the certified step exactly as a
  dangerous one would.

Two exact rules run first: spheres on the same frame are rigid (205 pairs, and it must key
on `frame`, not `Sphere::link` — 18 spheres share frame 6 under a dozen link names), and
some cross-frame pairs are still constant because a sphere sits on the axis between them.
Spheres 0 and 6 are both centred on the shoulder axis, 162.5 mm apart at every
configuration against a 160 mm radius sum — a permanent `h` of 2.5 mm that would have made
the QP infeasible everywhere.

**The rows are sparse, exactly.** With sphere `a` on frame `f` and `b` on frame `g ≥ f`,
only joints `f..g-1` can change the separation: a joint below `f` rotates both spheres as
one rigid body (`n^T(axis × (p_a - p_b)) = 0`, identically), and a joint at or above `g`
moves neither. So the Lipschitz bound is `leverArmBounds()` of the *later* sphere restricted
to that window — **not** the sum of the two spheres' bounds, which is loose by ~2× and
non-zero on columns where the true derivative vanishes.

### What it costs, and what it buys

`certifiedDuration()` is now permanently capped. No kept pair is ever clearer than
**58.2 mm** (`forearm` sphere 15 against `wrist_2` sphere 21) at *any* configuration, so
one filter call can no longer certify a whole extension however empty the workspace is.
That property is gone; what remains is a constant factor. On MotionBenchMaker, 105
problems, 5 s, margin 5 mm:

| gamma | rows | solved | rad/call | coarse | self-colliding audited states |
|-------|------|--------|----------|--------|-------------------------------|
| 0.4   | off  | 37/101 | 0.0519   | 47%    | **3117** |
| 0.4   | on   | 37/101 | 0.0282   | 12%    | **0** |
| 1.0   | off  | 36/101 | 0.0804   | 48%    | **4132** |
| 1.0   | on   | 33/101 | 0.0318   | 12%    | **0** |

So: the certified step shortens by 1.8–2.5×, the solve rate is within noise, and the
self-collisions go away entirely. Pass a negative `selfMargin` to
`demo_UR5MBMBenchmark` (argument 12) to reproduce the "off" rows.

### What it guarantees

**That the arm is clear of itself as PyBullet's URDF check sees it** — not merely that the
sphere model does not self-intersect. That is a change: it used to be the weaker claim, and
`defaultSelfMargin` was zero because the strong one looked unaffordable.

It looked unaffordable because it was costed as a *single* margin. Covering the 30.5 mm of
sphere under-coverage on both bodies needs over 60 mm, which exceeds the 58.2 mm the
tightest pair is ever clear by, so one global margin is infeasible everywhere. But the
deficit is not uniform. It sits almost entirely in two links — `forearm_link` and
`upper_arm_link` — and a margin fitted **per pair** is affordable where a global one is not.
The largest calibrated margin is 143.1 mm; most pairs are zero.

`scripts/calibrate_self_margins.py` measures it rather than deriving it. For each link pair
it samples configurations and records

```
margin = sup { sphere-pair clearance : the two links' collision bodies overlap in PyBullet }
```

Any margin at least this large makes `h_ab >= margin` imply "PyBullet says clear", over the
sampled set. It is a sampled bound, not a proof — the same standing as the band above.

Two refinements are load-bearing:

- **One row per link pair carries the margin**, the one needing the smallest. The barrier is
  a conjunction, so a collision only needs *one* row to fire; margining every row of a link
  pair also fires rows whose spheres are nowhere near the contact, and each of those refuses
  configurations of its own. Smallest-margin gives 9.6% refusals against 13.9% for a shared
  per-link margin. Choosing by *widest separating band* instead is much worse — it selects
  distant spheres needing 270 mm.
- **Per sphere pair, not per link pair.** `base_link` against `upper_arm_link` is
  inexpressible at link granularity: its minimum is spheres 0 and 6, both centred on the
  shoulder axis, a constant 2.5 mm at every configuration, so the same number describes the
  colliding states and the clear ones. It was 37% of all reports and unfixable until the
  calibration dropped a level.

Held out on 3000 configurations the margins were not fitted to: **0.10% leaks** (safe to the
spheres, colliding to PyBullet — the only unsound direction) and **9.6% refusals** (clear to
PyBullet, rejected by the spheres — the price).

#### What "PyBullet's check" actually is

**Convex hulls of the collision meshes, not the meshes.** Loading a URDF gives each link the
convex hull of its collision geometry, and for this arm that is a materially larger body:
`forearm.stl` is 0.64 of its own hull by volume, `upperarm.stl` 0.68 (the wrists are ~0.93,
and the Robotiq collision STLs are the `_coarse` ones, already convex).

This was measured, not assumed. Sample points at least 4 mm inside `forearm.stl`'s hull but
outside the mesh, put a 1 mm sphere at each, and ask PyBullet the distance to
`forearm_link`: **200 of 200 come back penetrating, to −27.9 mm**. Independently, 85 of
MotionBenchMaker's own 1378 endpoints — which its mesh checker calls collision-free — are
flagged by this check, up to −22 mm, every one involving `forearm_link`; the UR5 SRDF does
not disable those pairs, so it is not an allowed-collision-matrix gap.

So the barrier is now calibrated against a body strictly larger than the real robot. That is
a deliberate choice of reference, and the reason the arm will refuse some poses it could
physically reach. Closing the gap properly means convex-decomposing the two offending
meshes (VHACD), not changing the margins.

Two link pairs sit outside the scheme and are excluded rather than margined: `ee_link` is a
1 cm box with no spheres and no hardware behind it (a MoveIt frame), and `base_link` against
`upper_arm_link` needed the per-sphere-pair treatment described above.

#### Three independent audits, none sharing an assumption with the barrier

- `demos/UR5SelfCollisionAudit.h` walks **all** pairs from the sphere table, so it can
  contradict the pair search. Do not rewrite it in terms of `selfPairs()`.
- `ur5_experiments/scripts/audit_self_collision.py` replays a path against the URDF bodies
  in PyBullet, with its own hand-built ignore set. `--env` is optional: self-collision is a
  function of the configuration alone, which is what makes MotionBenchMaker auditable here
  without rebuilding its scenes.
- `scripts/calibrate_self_margins.py --verify` scores the margins on held-out
  configurations, which is the only one of the three that can price the refusals.

#### The result

Replaying every planned motion at 0.02 rad, **self rows at zero margin → calibrated**:

| bench | row | before | after |
|-------|-----|--------|-------|
| MotionBenchMaker | `cbf-rrtc` | 13916 / 72814, −33.2 mm | **0 / 54815**, +0.11 mm |
| MotionBenchMaker | `rrtconnect` | 16071 / 118076, −22.2 mm | **0 / 82433**, +2.93 mm |
| empty | both | 169 / 161 | **0 / 0** |
| corridor | both | 411 / 778 | **0 / 0** |
| pillars | both | 0 / 0 | **0 / 0** |
| shelf | both | 115 / 167 | **0 / 0** |
| clutter | both | 222 / 166 | **0 / 0** |
| wall_gap | both | 354 / 1568 | no path — see below |

Environment collisions are 0 everywhere in both settings, so the workspace side was never
the problem. Note the "before" column is the *sphere-model-clean* build: every one of those
13916 states satisfied every self row at zero margin.

#### What it costs

- **The certificate ceiling drops** 58.2 → **41.9 mm** above margin, and steps that run past
  `stepSize` on a certificate fall from 18% to 11% of filter calls (`rad/call` 0.0365 →
  0.0343).
- **23 of 105 MotionBenchMaker problems** now have a start or goal inside the margin and are
  excluded rather than scored.
- **`wall_gap` becomes unsolvable**, 2/2 → 0/2 — and for *both* planners, since they share
  `isSafe()`. Reaching through the gap needs a folded pose the margins reject. This is the
  clearest single price of aligning to the hulls.
- Only ~28% of uniformly random configurations pass, against 37.5% that the hulls call clear.

> Audit whole motions, not whole files. `demo_UR5PyBulletScene` writes one path per goal
> into a single file, each re-prefixed with the start configuration, so interpolating
> straight through the file invents a return-to-home from inside a shelf. Before
> `split_runs()` accounted for that, the old version of this table read 2012 → 1107
> self-colliding states and claimed hundreds of environment collisions that were never
> planned. `demo_UR5MBMBenchmark` writes an explicit `# motion` marker for the same reason,
> and the replay viewer snaps across those seams instead of animating them.

#### Goal selection has to know about these rows

Anything deciding whether a configuration is acceptable must evaluate the *same* barrier the
planner does. `ur5_experiments/scripts/export_scene.py` scored goals on the workspace rows
alone and so certified goals at `h = +0.0768` that the planner saw at `+0.0129`; both
planners then spent their whole time budget failing to reach them, which reads as a planner
regression and is not one. It now reads `UR5::selfPairs()` out of the header
(`ur5_nav/self_pairs.py`, with a sphere-index alignment check) instead of re-deriving it.

The second half of that fix matters as much: **the two kinds of row are held to different
buffers.** `buffer` covers the SDF's interpolation error, a property of the grid; between
two spheres there is no grid, so the self rows reserve only
`ClearanceBarrier::defaultSelfBuffer` — **1 mm against 15 mm**. Judging both by the larger
one rejected every goal in `pillars` and `clutter` over about 2 mm of clearance the planner
never asked for. With each row judged by its own threshold, goal usability returns to
exactly what it was before self-collision rows existed (19 of 22 across the six scenes), so
the calibrated margins cost **no goals at all**.

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
./build/demos/demo_UR5MBMBenchmark scenes.txt 15 2.0 0.02 0.05 2.0 0.002 0.006 -1 0.4 -1 0 out/mbm
```

Argument 9 is the baseline's edge-checking resolution and **−1 now means "match the rollout
step", not "leave OMPL's default"** — see below. Argument 13 dumps both rows' audited
motions (`out/mbm.rrtc`, `out/mbm.cbf`) for the PyBullet check.

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

### The comparison is a comparison of resolutions unless you tie them

This is the single easiest way to get a wrong answer here, and it has bitten this benchmark
in *both* directions.

The rollout advances `velocityLimits().maxCoeff() * stepSize` = 0.5 × 0.05 = **0.025 rad**
per filter call. OMPL states the baseline's resolution as a fraction of the state space's
maximum extent, and its default 0.01 is **0.154 rad** on this space — six times coarser. A
baseline that looks at the world six times less often is faster for a reason that has
nothing to do with either method.

So both demos now **derive the baseline's spacing from the rollout step** unless explicitly
overridden (`UR5MBMBenchmark` argument 9, `UR5PyBulletScene` argument 6; negative means
"match"). The header line reports what was used:

```
baseline segment: 0.001624 of extent = 0.0250 rad, rollout step 0.0250 rad (matched to the rollout)
```

How much this matters, on the six PyBullet scenes: at OMPL's default the baseline appeared
**4–9× faster** than the rollout; matched, the rollout wins three of the five solvable
scenes. In the other direction, earlier MotionBenchMaker runs here passed `0.0006`
(0.0092 rad), which made the baseline check *2.7× finer* than the rollout stepped and cost
it accordingly — 3307 checks and 11.21 ms, against 1305 and 5.51 ms when matched.

Two related traps:

- **Audit both rows at the same resolution.** `PathGeometric::interpolate()` with no
  argument uses that same fraction, which sampled the rollout 7× more coarsely than the
  baseline and reported a spurious zero. Pass an explicit state count.
- **The audit is finer than either row now samples** (0.02 rad against 0.025). That is
  deliberate — it asks whether the unsampled interiors were safe — but it means neither row
  is guaranteed audit-clean, and the current run leaves 2 states of 52272 short on the
  baseline and 15 of 54810 on the rollout. Those are inside the margin, not in contact: the
  external PyBullet check reports **0** actual self-collisions for both.

### Where it stands, matched

105 problems, 23 skipped as endpoint-infeasible, 82 scored, 2 s limit:

| | rrtconnect | cbf-rrtc |
| --- | --- | --- |
| solved | 76/82 | 76/82 |
| median ms | 5.51 | **3.10** (1.8×) |
| median evals | 1305 collision checks | **380** filter calls |
| median vertices | 17 | **9** |
| audited states | 52272 | 54810 |
| PyBullet self-collisions | **0** (+2.93 mm) | **0** (+0.11 mm) |

`evals` are not the same unit on the two rows — collision checks against filter calls, and a
filter call integrates one `dt` step where a check samples a whole edge. The comparable
columns are milliseconds and vertices.

On the six PyBullet scenes, medians over 5 trials at 10 s:

| scene | solved (both rows) | rrtconnect | cbf-rrtc | |
| --- | --- | --- | --- | --- |
| corridor | 3/3 | 19.03 ms | **7.94 ms** | **2.40×** |
| shelf | 4/4 | 5.90 ms | **5.07 ms** | 1.17× |
| pillars | 4/4 | 0.83 ms | **0.72 ms** | 1.15× |
| empty | 3/4 | 0.82 ms | 0.84 ms | 0.98× |
| clutter | 2/2 | **1.70 ms** | 6.36 ms | 0.27× |
| wall_gap | 0/2 | — | — | — |

The rollout wins the cluttered scenes and loses `clutter`. Note the audited-state counts in
these tables are **path length** (arc length ÷ 0.02 rad), not planner effort: on MBM the
rollout's path is 34% shorter than RRTConnect's unsimplified zigzag, while on the small
scenes, where a straight edge is already near-optimal, deflecting costs length.

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
- **Self-collision is sound at the sphere level and only better at the mesh level.**
  718 mesh-colliding states remain across the six PyBullet scenes, up to 26 mm deep, all
  of them clear in the sphere model. A margin cannot fix it — 60 mm is needed and 58.2 mm
  exists. What would: re-spherising `forearm_link` and the Robotiq knuckles, which are the
  two worst-covered links and account for essentially all of the residual. That also buys
  back certificate length, since a tighter model raises the 58.2 mm ceiling.
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

# Regenerate the self-collision pair table. --band is the certificate ceiling; with
# --margins each pair bands around its own margin instead of around zero. Refuses to
# emit if a droppable pair could still breach its margin.
python scripts/generate_ur5_self_pairs.py --margins margins.txt --band 0.04 --emit

# Re-fit the per-pair self margins against PyBullet. --verify scores them on held-out
# configurations (leaks must stay ~0); --endpoints prices them on MotionBenchMaker.
python scripts/calibrate_self_margins.py --samples 3000 --verify 3000 \
    --endpoints scenes.txt --emit margins.txt
```

### Reproducing the whole thing

```sh
# Outputs go outside the repo on purpose: paths, grids and scene dumps are large,
# regenerable, and turn up in `git status` right when you are trying to commit.
export WORK=/tmp/ur5-check && mkdir -p $WORK
cd build && cmake .. && make -j8 demo_UR5MBMBenchmark demo_UR5PyBulletScene && cd ..

# --- MotionBenchMaker ---------------------------------------------------------
python3 scripts/mbm_to_scenes.py /home/mani/vamp/resources/ur5/problems.json $WORK/scenes.txt
./build/demos/demo_UR5MBMBenchmark $WORK/scenes.txt 15 2.0 0.02 0.05 2.0 0.002 0.006 -1 0.4 -1 0 $WORK/mbm
(cd ur5_experiments && python3 scripts/audit_self_collision.py --path $WORK/mbm.cbf)
(cd ur5_experiments && python3 scripts/audit_self_collision.py --path $WORK/mbm.rrtc)

# --- the six PyBullet scenes --------------------------------------------------
# export_scene.py must run first and from inside ur5_experiments/: the .problem file
# names its .grid by path, and the grids are gitignored.
(cd ur5_experiments && python3 scripts/export_scene.py --env all --out out/)

for s in empty corridor pillars shelf clutter wall_gap; do
  ./build/demos/demo_UR5PyBulletScene ur5_experiments/out/$s.problem 10 $WORK/$s.path 5
done

cd ur5_experiments
for s in empty corridor pillars shelf clutter wall_gap; do
  python3 scripts/audit_self_collision.py --env $s --path $WORK/$s.path
  python3 scripts/audit_self_collision.py --env $s --path $WORK/$s.path.rrtc
done

# --- watch one ----------------------------------------------------------------
python3 scripts/replay_path.py --env shelf --path $WORK/shelf.path --gui --hold --speed 8 --fps 240
```

The A/B knobs are the same commands with one argument changed:

```sh
# self rows off (MBM argument 12, scene demo argument 8)
./build/demos/demo_UR5MBMBenchmark $WORK/scenes.txt 15 2.0 0.02 0.05 2.0 0.002 0.006 -1 0.4 -1 -100 $WORK/off
./build/demos/demo_UR5PyBulletScene ur5_experiments/out/shelf.problem 10 $WORK/off.path 5 -1 -1 -1 -100

# baseline back at OMPL's coarse default instead of matched (MBM argument 9)
./build/demos/demo_UR5MBMBenchmark $WORK/scenes.txt 15 2.0 0.02 0.05 2.0 0.002 0.006 0.01 0.4 -1 0 $WORK/coarse
```

Three things that will bite if skipped: `--env` must match the path's scene (the export
uses seed 0, which the auditor and viewer both default to); `wall_gap` currently solves
0/2, so its path file is near-empty and is not a setup error; and a path file replayed
from an older build shows the *old* behaviour — `out/shelf.path` from before the
calibration has 528 self-colliding states, which is what you will see in red in the GUI if
you replay a stale file.

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
