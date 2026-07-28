#!/usr/bin/env python
"""Derive the UR5 collision-sphere table baked into ``src/ompl/robots/UR5.h``.

The sphere approximation itself is not invented here: it is lifted from VAMP's
hand-authored ``ur5_spherized.urdf``, so ``ompl::robots::UR5`` and VAMP's own
planner see the same 40 spheres and results stay directly comparable.

Every sphere in that URDF hangs off one of 7 frames that the 6 revolute joints
move (``base_link`` plus the 6 child links). Everything past ``wrist_3_link``
(the FTS and the Robotiq 85 gripper) is attached by *fixed* joints, so this
script pre-composes those chains and re-expresses each of those sphere centers
in ``wrist_3_link``'s frame. That collapses the whole model to 40 (frame index,
center, radius) triples, which is all the C++ side needs for both forward
kinematics and exact sphere-center Jacobians.

Usage:
    python scripts/generate_ur5_spheres.py [--urdf PATH] [--emit]

Validates against ``vamp.ur5.fk`` when the ``vamp`` module is importable, and
always checks the analytic Jacobian against central differences. ``--emit``
prints the C++ table to stdout for pasting into ``src/ompl/robots/UR5.h``.
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np

DEFAULT_URDF = "/home/mani/vamp/resources/ur5/ur5_spherized.urdf"

# base_link plus the 6 links moved by the 6 revolute joints, in chain order.
# Frame i is moved by joints 0..i-1, so frame 0 (base_link) is fixed.
FRAMES = (
    "base_link",
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
)


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF ``<origin rpy="r p y"/>`` convention: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


def axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def _vec(text: str) -> np.ndarray:
    return np.array([float(x) for x in text.split()])


class SpherizedUrdf:
    """The parts of the spherized URDF this script needs."""

    def __init__(self, path: str) -> None:
        root = ET.parse(path).getroot()

        self.joints: dict[str, dict] = {}  # keyed by child link
        for joint in root.iter("joint"):
            origin = joint.find("origin")
            axis = joint.find("axis")
            rpy = origin.get("rpy") if origin is not None else None
            self.joints[joint.find("child").get("link")] = dict(
                name=joint.get("name"),
                type=joint.get("type"),
                parent=joint.find("parent").get("link"),
                xyz=_vec(origin.get("xyz")) if origin is not None else np.zeros(3),
                rotation=rpy_matrix(*_vec(rpy)) if rpy else np.eye(3),
                axis=_vec(axis.get("xyz")) if axis is not None else None,
            )

        # (link, center in link frame, radius), in URDF document order.
        self.spheres: list[tuple[str, np.ndarray, float]] = []
        for link in root.iter("link"):
            for collision in link.iter("collision"):
                geom = collision.find("geometry/sphere")
                if geom is None:
                    continue
                origin = collision.find("origin")
                self.spheres.append(
                    (
                        link.get("name"),
                        _vec(origin.get("xyz")) if origin is not None else np.zeros(3),
                        float(geom.get("radius")),
                    )
                )

    def to_moving_frame(self, link: str, center: np.ndarray) -> tuple[int, np.ndarray]:
        """Walk up fixed joints to the nearest frame in FRAMES, carrying the center along."""
        p = center.astype(float).copy()
        while link not in FRAMES:
            joint = self.joints[link]
            if joint["type"] != "fixed":
                raise ValueError(f"movable joint {joint['name']} above a sphere on {link}")
            p = joint["rotation"] @ p + joint["xyz"]
            link = joint["parent"]
        return FRAMES.index(link), p

    def table(self) -> list[tuple[int, np.ndarray, float, str]]:
        return [(*self.to_moving_frame(link, c), r, link) for link, c, r in self.spheres]

    def frame_transforms(self, q) -> tuple[list[np.ndarray], list[np.ndarray], np.ndarray, np.ndarray]:
        """Per-frame pose plus each joint's world-frame origin and axis at ``q``.

        The base pose is the fixed ``offset_link -> base_link`` hop, so everything
        comes out in ``offset_link`` coordinates -- the frame ``vamp.ur5.fk`` uses.
        """
        base = self.joints["base_link"]
        rot, trans = base["rotation"].copy(), base["xyz"].astype(float).copy()
        rotations, translations = [rot], [trans]
        origins, axes = [], []
        for i, link in enumerate(FRAMES[1:]):
            joint = self.joints[link]
            trans = trans + rot @ joint["xyz"]
            rot = rot @ joint["rotation"]
            origins.append(trans.copy())
            axes.append(rot @ joint["axis"])
            rot = rot @ axis_angle(joint["axis"], float(q[i]))
            rotations.append(rot.copy())
            translations.append(trans.copy())
        return rotations, translations, np.array(origins), np.array(axes)


def sphere_centers(urdf: SpherizedUrdf, table, q) -> np.ndarray:
    rotations, translations, _, _ = urdf.frame_transforms(q)
    return np.array([rotations[f] @ c + translations[f] for f, c, _, _ in table])


def sphere_jacobian(urdf: SpherizedUrdf, table, q, i: int) -> np.ndarray:
    """d(center_i)/dq -- column k is axis_k x (p - origin_k) for the joints that move it."""
    rotations, translations, origins, axes = urdf.frame_transforms(q)
    frame, center, _, _ = table[i]
    p = rotations[frame] @ center + translations[frame]
    jac = np.zeros((3, 6))
    for k in range(frame):  # frame f is moved by joints 0..f-1
        jac[:, k] = np.cross(axes[k], p - origins[k])
    return jac


def validate_against_vamp(urdf: SpherizedUrdf, table) -> None:
    try:
        import vamp
    except ImportError:
        print("vamp not importable -- skipping cross-validation", file=sys.stderr)
        return

    # VAMP's sphere order is a permutation of URDF document order (its codegen
    # reorders for SIMD packing). Establish the permutation once at q = 0 by
    # nearest-center matching among same-radius spheres, then hold it fixed.
    reference = vamp.ur5.fk(np.zeros(6, dtype=np.float32))
    mine = sphere_centers(urdf, table, np.zeros(6))
    if not len(reference) == len(mine) == len(table):
        raise AssertionError(f"sphere count mismatch: vamp {len(reference)} vs ours {len(table)}")

    permutation: list[int] = []
    for sphere in reference:
        candidates = [
            (float(np.linalg.norm(np.asarray(sphere.position) - mine[i])), i)
            for i in range(len(table))
            if i not in permutation and abs(table[i][2] - sphere.r) < 1e-6
        ]
        distance, index = min(candidates)
        if distance > 1e-5:
            raise AssertionError(f"no match for vamp sphere at {sphere.position} r={sphere.r}")
        permutation.append(index)
    if len(set(permutation)) != len(table):
        raise AssertionError("urdf <-> vamp sphere matching is not a bijection")

    rng = np.random.default_rng(0)
    worst = 0.0
    for k in range(200):
        q = np.zeros(6) if k == 0 else rng.uniform(-math.pi, math.pi, 6)
        reference = np.array([s.position for s in vamp.ur5.fk(np.asarray(q, dtype=np.float32))])
        worst = max(worst, float(np.abs(reference - sphere_centers(urdf, table, q)[permutation]).max()))
    # vamp.ur5.fk is single precision, so ~1e-6 m is exact agreement.
    print(f"vs vamp.ur5.fk over 200 configs: max center error = {worst:.2e} m")
    print(f"urdf-order -> vamp-order permutation: {permutation}")


def validate_jacobian(urdf: SpherizedUrdf, table) -> None:
    rng = np.random.default_rng(1)
    worst = 0.0
    for _ in range(50):
        q = rng.uniform(-math.pi, math.pi, 6)
        for i in range(len(table)):
            finite = np.zeros((3, 6))
            for k in range(6):
                step = np.zeros(6)
                step[k] = 1e-6
                finite[:, k] = (
                    sphere_centers(urdf, table, q + step)[i] - sphere_centers(urdf, table, q - step)[i]
                ) / 2e-6
            worst = max(worst, float(np.abs(sphere_jacobian(urdf, table, q, i) - finite).max()))
    print(f"analytic Jacobian vs central differences: max error = {worst:.2e}")


def emit_cxx(table) -> None:
    print("        // clang-format off")
    for frame, center, radius, link in table:
        # repr() gives the shortest decimal that round-trips exactly.
        x, y, z = (repr(float(v)) for v in center)
        print(f'        {{{frame}, {{{x}, {y}, {z}}}, {radius!r}, "{link}"}},')
    print("        // clang-format on")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", default=DEFAULT_URDF, help="path to ur5_spherized.urdf")
    parser.add_argument("--emit", action="store_true", help="print the C++ table to stdout")
    args = parser.parse_args()

    urdf = SpherizedUrdf(args.urdf)
    table = urdf.table()
    print(f"{len(table)} spheres over {len(FRAMES)} frames from {args.urdf}", file=sys.stderr)

    validate_against_vamp(urdf, table)
    validate_jacobian(urdf, table)
    if args.emit:
        emit_cxx(table)


if __name__ == "__main__":
    main()
