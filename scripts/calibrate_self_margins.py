#!/usr/bin/env python
"""Calibrate the CBF's self-collision margins against PyBullet's own verdict.

The barrier's self rows keep the 40 VAMP spheres apart; PyBullet's URDF check keeps the
*convex hulls of the collision meshes* apart. Those are different bodies, and the spheres
are the smaller ones -- so a configuration can satisfy every self row and still be a
self-collision to the checker. On MotionBenchMaker it does, on 13916 of 72814 audited
states, with the sphere rows reporting zero.

This measures the gap the way it is used, rather than deriving it from geometry. For each
pair of links it samples configurations and records

    required = sup { h_sphere(q) : the hulls of the two links overlap at q }

where ``h_sphere`` is the tightest sphere-pair clearance between those two links. Any self
margin at least this large makes ``h_sphere >= margin`` imply "PyBullet says clear", over
the sampled set. It is a sampled bound, not a proof -- the same standing as
``generate_ur5_self_pairs.py``'s band, and for the same reason: there is no closed form for
the hull of an STL against a sphere set across a 6-DoF sweep.

It also records the *ceiling*: the largest clearance that pair ever reaches. A margin above
its own ceiling is a row that can never be satisfied, so the pair would make the QP
infeasible everywhere rather than safe. Pairs where required >= ceiling cannot be fixed by
a margin at all and are reported as such -- that is a statement about the sphere model, not
about the number.

    python scripts/calibrate_self_margins.py --samples 4000

Why a *per-pair* number and not one global margin: the deficit is concentrated. Two links
(``forearm_link`` and ``upper_arm_link``) hold collision meshes whose convex hulls are 1.5x
their own volume, and a single margin big enough for those would exceed the ceiling of
every other pair at once.
"""

from __future__ import annotations

import argparse
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "ur5_experiments"))

from generate_ur5_spheres import DEFAULT_URDF, SpherizedUrdf, sphere_centers  # noqa: E402
from generate_ur5_self_pairs import JOINT_LIMIT, candidate_pairs  # noqa: E402

from ur5_nav import UR5, SimSession  # noqa: E402


def link_pair_index(table, pairs):
    """Group sphere pairs by the pair of link names they connect.

    Keyed on the *link* rather than the frame, because PyBullet reports link names: the 18
    spheres on frame 6 live under a dozen of them, and a margin that lumped them together
    would be set by the worst gripper part for all of the rest.
    """
    groups: dict[tuple[str, str], list[int]] = defaultdict(list)
    for index, (a, b) in enumerate(pairs):
        key = tuple(sorted((table[a][3], table[b][3])))
        groups[key].append(index)
    return groups


def sample_configurations(rng, count: int):
    """Uniform over the joint limits, starting from home -- the most folded pose."""
    for n in range(count):
        yield np.zeros(6) if n == 0 else rng.uniform(-JOINT_LIMIT, JOINT_LIMIT, 6)


def hull_contacts(robot):
    """Tightest hull distance per link pair, PyBullet's verdict at the current config.

    `ee_link` is dropped here rather than margined. It is a 1 cm box at the wrist -- a
    MoveIt convenience frame with no hardware behind it -- so VAMP gave it no spheres, and
    a self row for it would be a row against a phantom.
    """
    out: dict[tuple[str, str], float] = {}
    for contact in robot._self_contacts(0.0):
        key = tuple(sorted(contact.link.split("/")))
        if "ee_link" in key:
            continue
        out[key] = min(out.get(key, np.inf), contact.distance)
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--urdf", default=DEFAULT_URDF, help="spherized UR5 URDF")
    parser.add_argument("--samples", type=int, default=4000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--verify", type=int, default=0,
                        help="held-out configurations to score the calibrated margins on")
    parser.add_argument("--emit", default=None,
                        help="write the calibrated margins to this file, as "
                             "'sphere_a sphere_b margin' -- feed it to "
                             "generate_ur5_self_pairs.py --margins")
    parser.add_argument("--endpoints", default=None,
                        help="scenes.txt from mbm_to_scenes.py: reports how many of its "
                             "problems keep both endpoints feasible under these margins")
    args = parser.parse_args()

    urdf = SpherizedUrdf(args.urdf)
    table = urdf.table()
    pairs = candidate_pairs(table)
    groups = link_pair_index(table, pairs)

    first = np.array([p[0] for p in pairs])
    second = np.array([p[1] for p in pairs])
    radii = np.array([t[2] for t in table])
    radius_sum = radii[first] + radii[second]

    required: dict[tuple[str, str], float] = defaultdict(lambda: -np.inf)
    ceiling: dict[tuple[str, str], float] = defaultdict(lambda: -np.inf)
    floor: dict[tuple[str, str], float] = defaultdict(lambda: np.inf)
    hits: dict[tuple[str, str], int] = defaultdict(int)
    unmodelled: dict[tuple[str, str], int] = defaultdict(int)
    colliding_states = 0

    def gaps(q):
        centers = sphere_centers(urdf, table, q)
        return np.linalg.norm(centers[first] - centers[second], axis=1) - radius_sum

    def clearances(q):
        gap = gaps(q)
        return {key: gap[idx].min() for key, idx in groups.items()}

    # Per *sphere* pair, the same two statistics. The barrier's rows are sphere pairs and
    # its verdict is their conjunction, so a collision only needs *one* row to fire -- it
    # does not need the tightest sphere pair to be the one that fires. Taking the min over
    # a link pair first throws that freedom away, and on base_link against upper_arm_link
    # it throws away the whole pair: the minimum there is spheres 0 and 6, both centred on
    # the shoulder axis, whose separation is the same 2.5 mm in every configuration.
    pair_required = np.full(len(pairs), -np.inf)
    pair_ceiling = np.full(len(pairs), -np.inf)

    rng = np.random.default_rng(args.seed)
    with SimSession(gui=False) as session:
        robot = UR5(session.client, self_collision=True)

        for n, q in enumerate(sample_configurations(rng, args.samples)):
            gap = gaps(q)
            clearance = {key: gap[idx].min() for key, idx in groups.items()}
            for key, value in clearance.items():
                ceiling[key] = max(ceiling[key], value)
                floor[key] = min(floor[key], value)
            pair_ceiling = np.maximum(pair_ceiling, gap)

            robot.set_config(q)
            hulls = hull_contacts(robot)
            for key, distance in hulls.items():
                if distance < 0.0 and key in groups:
                    index = groups[key]
                    pair_required[index] = np.maximum(pair_required[index], gap[index])
            if any(d < 0.0 for d in hulls.values()):
                colliding_states += 1
            for key, distance in hulls.items():
                if distance >= 0.0:
                    continue
                hits[key] += 1
                if key not in clearance:
                    # No sphere pair spans these two links at all -- the spheres cannot
                    # see this collision, and no margin can make them.
                    unmodelled[key] += 1
                    continue
                required[key] = max(required[key], clearance[key])

            if n % 100 == 0:
                print(f"\r  sampling {n}/{args.samples}", end="", file=sys.stderr, flush=True)
        print(file=sys.stderr)

        print(f"\n{args.samples} configurations, {colliding_states} "
              f"({1e2 * colliding_states / args.samples:.1f}%) self-colliding as PyBullet "
              f"sees it\n")
        print(f"  {'link pair':52s} {'states':>7s} {'required':>10s} {'ceiling':>9s}  verdict")

        margins: dict[tuple[str, str], float] = {}
        blind: list[tuple[str, str]] = []
        for key, value in sorted(required.items(), key=lambda kv: -kv[1]):
            head = ceiling[key]
            # A pair whose clearance never varies cannot be margined into agreement: the
            # same number describes the colliding states and the clear ones. Spheres 0 and
            # 6 are both centred on the shoulder axis, which is why base_link against
            # upper_arm_link lands here.
            expressible = value < head
            print(f"  {key[0] + ' / ' + key[1]:52s} {hits[key]:7d} {value * 1e3:9.1f}mm "
                  f"{head * 1e3:8.1f}mm  {'ok' if expressible else 'UNSATISFIABLE'}")
            if expressible:
                margins[key] = max(0.0, value)
            else:
                blind.append(key)
        for key, count in sorted(unmodelled.items(), key=lambda kv: -kv[1]):
            print(f"  {key[0] + ' / ' + key[1]:52s} {count:7d} {'no spheres span it':>21s}")
            blind.append(key)

        if margins:
            print(f"\nlargest required margin {max(margins.values()) * 1e3:.1f} mm. "
                  f"{len(blind)} link pairs cannot be expressed\nby any margin and are "
                  f"listed above; every other pair's ceiling clears the margin it is given.")

        # Now the same calibration one level down. For each link pair that ever collides,
        # keep the *one* sphere pair needing the smallest margin, and leave every other row
        # of that link pair at zero.
        #
        # Soundness is unchanged: at a colliding configuration that row's clearance is at
        # or below its margin by construction, so it fires, and one firing row is all a
        # conjunction needs. What changes is the cost. Giving the whole link pair a common
        # margin makes every one of its rows fire on every state near the bound, including
        # rows belonging to spheres nowhere near the contact -- and each of those refuses
        # configurations of its own. The smallest-margin row is the sphere pair closest to
        # where the hulls actually touch, so it is also the one whose refusals are the most
        # nearly deserved.
        chosen: dict[int, float] = {}
        still_blind: list[tuple[str, str]] = []
        for key in hits:
            if key not in groups:
                still_blind.append(key)
                continue
            index = np.asarray(groups[key])
            feasible = index[pair_required[index] < pair_ceiling[index]]
            if feasible.size == 0:
                still_blind.append(key)
                continue
            best = int(feasible[int(np.argmin(pair_required[feasible]))])
            chosen[best] = max(chosen.get(best, 0.0), max(0.0, float(pair_required[best])))

        print(f"\nOne sphere pair per link pair, the one needing the smallest margin:")
        print(f"  {'sphere pair':44s} {'margin':>9s} {'ceiling':>9s} {'link pair'}")
        for best, margin in sorted(chosen.items(), key=lambda kv: -kv[1])[:12]:
            a, b = pairs[best]
            print(f"  {f'{a:2d} ({table[a][3]}) vs {b:2d} ({table[b][3]})':44s} "
                  f"{margin * 1e3:8.1f}mm {pair_ceiling[best] * 1e3:8.1f}mm")
        print(f"  ... {len(chosen)} rows in total, "
              f"{len(still_blind)} link pairs still inexpressible")

        if args.emit:
            with open(args.emit, "w") as out:
                out.write("# calibrated self-collision margins, in metres, from\n"
                          "# scripts/calibrate_self_margins.py --samples "
                          f"{args.samples} --seed {args.seed}\n"
                          "# margin = sup{ sphere-pair clearance : the two links' convex\n"
                          "# hulls overlap in PyBullet }, for the one sphere pair per link\n"
                          "# pair that needs the smallest one.\n")
                for index, margin in sorted(chosen.items()):
                    a, b = pairs[index]
                    out.write(f"{a} {b} {margin:.6f}\n")
            print(f"  wrote {args.emit}")

        if args.verify:
            verify(robot, gaps, chosen, still_blind, args.verify, args.seed + 104729)
        if args.endpoints:
            endpoints(robot, gaps, chosen, still_blind, args.endpoints)
    return 0


def score(gap, hulls, margins, blind):
    """(sphere model says safe, PyBullet says clear) at one configuration."""
    safe = all(gap[index] >= margin for index, margin in margins.items())
    clear = all(distance >= 0.0 or key in blind for key, distance in hulls.items())
    return safe, clear


def verify(robot, gaps, margins, blind, samples: int, seed: int) -> None:
    """Score the calibrated margins on configurations they were not fitted to.

    Two numbers matter and they pull against each other. A *leak* is a configuration the
    padded sphere model calls safe and PyBullet calls colliding -- the thing the margins
    exist to remove, and the only one that is unsound. A *refusal* is the opposite: safe
    to PyBullet, rejected by the padded model. Refusals are the price, and they are what
    decides whether a planner can still move.
    """
    rng = np.random.default_rng(seed)
    leaks = refusals = safe_count = clear_count = 0
    worst_leak = 0.0
    for q in sample_configurations(rng, samples):
        robot.set_config(q)
        safe, clear = score(gaps(q), hull_contacts(robot), margins, blind)
        safe_count += safe
        clear_count += clear
        if safe and not clear:
            leaks += 1
        if clear and not safe:
            refusals += 1

    print(f"\nheld out {samples} configurations, margins fitted on a disjoint set:")
    print(f"  PyBullet calls {1e2 * clear_count / samples:5.1f}% of them clear")
    print(f"  padded spheres call {1e2 * safe_count / samples:5.1f}% of them safe")
    print(f"  leaks    {leaks:5d} ({1e2 * leaks / samples:5.2f}%)  safe to the spheres, "
          f"colliding to PyBullet -- must be ~0")
    print(f"  refusals {refusals:5d} ({1e2 * refusals / samples:5.2f}%)  clear to PyBullet, "
          f"rejected by the spheres -- the cost")


def endpoints(robot, gaps, margins, blind, scenes: str) -> None:
    """How much of MotionBenchMaker survives these margins.

    A problem whose start or goal the padded model rejects cannot be planned at all, so
    this is the margin's bill on a standard benchmark rather than an opinion about it.
    """
    problems: list[tuple[str, np.ndarray, np.ndarray]] = []
    name = start = goal = None
    with open(scenes) as handle:
        for line in handle:
            fields = line.split()
            if not fields or fields[0].startswith("#"):
                continue
            if fields[0] == "problem":
                if name and start is not None and goal is not None:
                    problems.append((name, start, goal))
                name, start, goal = f"{fields[1]}", None, None
            elif fields[0] in ("start", "goal"):
                q = np.array([float(v) for v in fields[1:7]])
                if fields[0] == "start":
                    start = q
                else:
                    goal = q
    if name and start is not None and goal is not None:
        problems.append((name, start, goal))

    kept: dict[str, list[int]] = defaultdict(lambda: [0, 0])
    for scene, start, goal in problems:
        ok = True
        for q in (start, goal):
            robot.set_config(q)
            safe, _ = score(gaps(q), hull_contacts(robot), margins, blind)
            ok = ok and safe
        kept[scene][0] += ok
        kept[scene][1] += 1

    print(f"\nMotionBenchMaker endpoints still feasible under these margins:")
    for scene, (good, total) in sorted(kept.items()):
        print(f"  {scene:22s} {good:4d}/{total:<4d} {1e2 * good / total:5.1f}%")
    good = sum(v[0] for v in kept.values())
    total = sum(v[1] for v in kept.values())
    print(f"  {'all':22s} {good:4d}/{total:<4d} {1e2 * good / total:5.1f}%")


if __name__ == "__main__":
    raise SystemExit(main())
