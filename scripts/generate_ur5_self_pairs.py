#!/usr/bin/env python
"""Derive the UR5 self-collision sphere-pair table baked into ``src/ompl/robots/UR5.h``.

A CBF that only knows about the world SDF has nothing to say about the arm folding
through itself. The fix is one extra QP row per sphere pair,

    h_ab(q) = |p_a(q) - p_b(q)| - r_a - r_b - margin,

which needs a *pair set*. All 780 of them is the wrong answer, and not only for cost.

### The band

Everything here turns on one number, ``--band``. A pair earns a row only if its
clearance *crosses* the band somewhere in configuration space:

- ``max h_raw < band`` -- the pair never separates. The geometry holds these two
  spheres in permanent near-contact, so a barrier on them is either infeasible
  everywhere (when they always overlap) or permanently binding (when they never do).
  Either way the planner cannot act on it. This is the rule that stands in for an
  SRDF allowed-collision matrix, which this repo does not have -- and it is measured
  rather than assumed, so it catches cases frame-adjacency would miss. ``wrist_1``
  sphere 16 against ``wrist_3`` sphere 22 is two joints apart yet never clearer than
  10.3 mm.
- ``min h_raw > band`` -- the pair never approaches, so its row can never bind. This
  is not merely a saving. ``certifiedDuration()`` takes a minimum over *all* rows, so
  a vacuous row's clearance caps the certificate exactly as a dangerous one's would,
  and the Lipschitz jump that pays for this planner's wall time is charged for a
  collision that cannot happen.

So the band sets the certificate ceiling directly: no kept pair is ever clearer than
it, which is the price of the whole feature. Raising it buys coverage and costs
speed. It cannot be raised freely -- past about 80 mm the *first* rule starts
discarding a pair that genuinely does collide (sphere 0 against sphere 2, whose
clearance runs from -23.2 to 77.0 mm), which would be a silent hole. The script
refuses to emit if that happens.

At the 50 mm default, 33 pairs are dropped as never-separating and **none of them
straddles zero**: every one either always overlaps -- so no barrier could have helped
-- or never overlaps at all. The drop costs no collision coverage, and the resulting
certificate ceiling is 58.2 mm.

### The exact rules

Two more drops come before the band, and both are exact rather than sampled:

- **Same frame.** Both spheres ride one rigid body. 18 of the 40 hang off frame 6
  (``wrist_3_link`` plus the whole Robotiq gripper) under a dozen URDF link names,
  which is why this keys on the frame and not on ``Sphere::link``.
- **Constant separation.** Some pairs on *different* frames still never move relative
  to each other, because a sphere sits on the axis that separates them. Spheres 0 and
  6 are both centred on the shoulder axis and are 162.5 mm apart at every
  configuration, against a radius sum of 160 mm. Only sampling finds these. The band
  would drop them all anyway; they are called out separately because the reason is
  qualitatively different and worth seeing.

Whatever comes out, ``worstSelfOverlap()`` in the demos stays an independent check on
it. It must not be rewritten in terms of this table, or it loses the ability to catch
a bad one.

Usage:
    python scripts/generate_ur5_self_pairs.py [--urdf PATH] [--samples N] [--emit]

Forward kinematics comes from ``generate_ur5_spheres.py``, so the sphere indices here
are the indices ``UR5::spheres()`` uses.
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

from generate_ur5_spheres import DEFAULT_URDF, SpherizedUrdf, sphere_centers

# Joint limits from VAMP's ur5.urdf, matching UR5::lowerBounds()/upperBounds().
JOINT_LIMIT = 3.14159265

# See the module docstring. 50 mm is the largest band that drops no pair capable of
# actually colliding, measured over 20k configurations.
DEFAULT_BAND = 0.05

# Below this spread between the sampled extremes, a pair's separation is constant.
CONSTANT_TOLERANCE = 1e-6


def candidate_pairs(table) -> list[tuple[int, int]]:
    """Every sphere pair on distinct frames, ordered so the earlier frame comes first.

    The ordering is load-bearing on the C++ side rather than cosmetic: with frames
    ``f <= g``, the joints that can change the separation are exactly ``f..g-1``, and
    the pair's constraint row and its Lipschitz bound are both built assuming it.
    """
    pairs = []
    for a in range(len(table)):
        for b in range(a + 1, len(table)):
            if table[a][0] == table[b][0]:
                continue
            pairs.append((a, b) if table[a][0] <= table[b][0] else (b, a))
    return pairs


def sampled_extremes(urdf, table, pairs, samples: int, seed: int, chunk: int = 512):
    """Running min/max of ``|p_a - p_b| - r_a - r_b`` over uniformly sampled configurations."""
    first = np.array([p[0] for p in pairs])
    second = np.array([p[1] for p in pairs])
    radii = np.array([t[2] for t in table])
    radius_sum = radii[first] + radii[second]

    lo = np.full(len(pairs), np.inf)
    hi = np.full(len(pairs), -np.inf)
    rng = np.random.default_rng(seed)

    taken = 0
    while taken < samples:
        n = min(chunk, samples - taken)
        centers = np.empty((n, len(table), 3))
        for r in range(n):
            # Start at the home configuration, which is where the arm is most folded
            # and several pairs are at their tightest.
            q = np.zeros(6) if taken + r == 0 else rng.uniform(-JOINT_LIMIT, JOINT_LIMIT, 6)
            centers[r] = sphere_centers(urdf, table, q)

        gap = np.linalg.norm(centers[:, first, :] - centers[:, second, :], axis=2) - radius_sum
        lo = np.minimum(lo, gap.min(axis=0))
        hi = np.maximum(hi, gap.max(axis=0))

        taken += n
        print(f"\r  sampling {taken}/{samples}", end="", file=sys.stderr, flush=True)
    print(file=sys.stderr)
    return lo, hi


def describe(table, pair) -> str:
    a, b = pair
    return f"{a:2d} ({table[a][3]}) vs {b:2d} ({table[b][3]})"


def load_margins(path: str, pairs) -> np.ndarray:
    """Per-pair margins from scripts/calibrate_self_margins.py, in pair order.

    A pair with a margin is guarded at ``h >= margin`` rather than at ``h >= 0``, because
    the spheres are smaller than the hulls PyBullet checks and the difference is not
    uniform. Every rule below therefore reads the *effective* clearance ``h - margin``:
    a pair earns a row by crossing the band in that quantity, and caps the certificate by
    it too. Pairs absent from the file keep a margin of zero and behave exactly as before.
    """
    index = {pair: i for i, pair in enumerate(pairs)}
    margins = np.zeros(len(pairs))
    missing = []
    with open(path) as handle:
        for line in handle:
            if line.startswith("#") or not line.strip():
                continue
            a, b, margin = line.split()
            key = (int(a), int(b))
            if key not in index:
                missing.append(key)
                continue
            margins[index[key]] = float(margin)
    if missing:
        raise SystemExit(f"{path} names {len(missing)} pairs that are not candidates: {missing}")
    print(f"loaded {int((margins > 0).sum())} calibrated margins from {path}, "
          f"largest {margins.max() * 1e3:.1f} mm", file=sys.stderr)
    return margins


def select(table, pairs, lo, hi, band: float, margins):
    """Apply the constant and band rules. Returns the kept pairs; same-frame is already gone."""
    kept: list[tuple[tuple[int, int], float, float]] = []
    constant: list[int] = []
    never_separates: list[int] = []
    never_approaches: list[int] = []

    for index, pair in enumerate(pairs):
        # The band is applied to the effective clearance h - margin, which is what the
        # barrier actually holds at or above zero.
        threshold = band + margins[index]
        if hi[index] - lo[index] < CONSTANT_TOLERANCE:
            constant.append(index)
        elif hi[index] < threshold:
            never_separates.append(index)
        elif lo[index] > threshold:
            never_approaches.append(index)
        else:
            kept.append((pair, lo[index], hi[index], margins[index]))

    def show(indices, heading, detail):
        print(f"\ndropped {len(indices)} pairs -- {heading}", file=sys.stderr)
        if not detail:
            return
        for index in indices:
            print(f"  {describe(table, pairs[index]):58s} "
                  f"h in [{lo[index] * 1e3:7.1f}, {hi[index] * 1e3:7.1f}] mm", file=sys.stderr)

    show(constant, "constant separation (the sphere-0/6 case)", True)
    show(never_separates, f"never separates (max clearance < {band * 1e3:.0f} mm)", True)
    show(never_approaches, f"never approaches (min clearance > {band * 1e3:.0f} mm)", False)

    # The one drop that could cost real coverage: a pair that both overlaps somewhere
    # and separates nowhere far enough to have earned a row. None exist at 50 mm, and
    # emitting a table with one in it would be a silent hole in the barrier.
    holes = [i for i in never_separates if lo[i] < margins[i] < hi[i]]
    if holes:
        print(f"\nERROR: {len(holes)} never-separating pairs can still collide. "
              f"Lower --band; these would be unprotected:", file=sys.stderr)
        for index in holes:
            print(f"  {describe(table, pairs[index]):58s} "
                  f"h in [{lo[index] * 1e3:7.1f}, {hi[index] * 1e3:7.1f}] mm", file=sys.stderr)
        raise SystemExit(1)

    return kept


def report(table, kept) -> None:
    """What the kept set costs, in the terms the certificate is charged in."""
    print(f"\nkept {len(kept)} pairs", file=sys.stderr)

    # certifiedDuration() mins over rows, so the pair with the smallest *maximum*
    # clearance caps it no matter how open the scene is.
    pair, _, high, margin = min(kept, key=lambda entry: entry[2] - entry[3])
    print(f"  certificate ceiling: {describe(table, pair)} is never clearer than "
          f"{(high - margin) * 1e3:.1f} mm above its own margin", file=sys.stderr)

    collide = sum(1 for _, low, _, margin in kept if low < margin)
    print(f"  {collide} of them can actually breach their margin; the other "
          f"{len(kept) - collide} are inside the band but never do", file=sys.stderr)


def emit_cxx(table, kept) -> None:
    print("                // clang-format off")
    for pair, low, high, margin in kept:
        a, b = pair
        print(f"                {{{a}, {b}, {margin:.6f}}},  // {table[a][3]} vs {table[b][3]}, "
              f"h in [{low * 1e3:.1f}, {high * 1e3:.1f}] mm")
    print("                // clang-format on")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", default=DEFAULT_URDF, help="path to ur5_spherized.urdf")
    parser.add_argument("--samples", type=int, default=200000, help="configurations to sample")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--band", type=float, default=DEFAULT_BAND,
                        help="clearance a pair must cross to earn a row; also the certificate ceiling")
    parser.add_argument("--margins", default=None,
                        help="per-pair margins from scripts/calibrate_self_margins.py --emit")
    parser.add_argument("--emit", action="store_true", help="print the C++ table to stdout")
    args = parser.parse_args()

    urdf = SpherizedUrdf(args.urdf)
    table = urdf.table()

    pairs = candidate_pairs(table)
    total = len(table) * (len(table) - 1) // 2
    print(f"{total} sphere pairs, {total - len(pairs)} dropped as same-frame (rigid), "
          f"{len(pairs)} to sample", file=sys.stderr)

    margins = (load_margins(args.margins, pairs) if args.margins
               else np.zeros(len(pairs)))

    lo, hi = sampled_extremes(urdf, table, pairs, args.samples, args.seed)
    kept = select(table, pairs, lo, hi, args.band, margins)
    report(table, kept)

    if args.emit:
        emit_cxx(table, kept)


if __name__ == "__main__":
    main()
