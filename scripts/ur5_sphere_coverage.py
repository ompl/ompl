#!/usr/bin/env python
"""Measure how well the UR5 sphere model covers the actual collision meshes.

The sphere set baked into ``src/ompl/robots/UR5.h`` comes from VAMP, where it is
tuned for fast collision *checking* — it is not guaranteed to enclose the links.
That distinction matters for a CBF: the barrier h_i = d(p_i) - r_i certifies
clearance for the *spheres*, so wherever the mesh pokes outside its spheres, a
strictly positive barrier does not imply the robot is clear.

This script reports that gap per link: the furthest a collision-mesh vertex sits
outside every sphere attached to it. Inflating the radii by the worst value makes
the sphere set a true outer bound, and hence the barrier genuinely conservative.

    python scripts/ur5_sphere_coverage.py

Distances are computed in each link's own frame, so no forward kinematics is
involved and the numbers hold at every configuration.
"""

from __future__ import annotations

import argparse
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from generate_ur5_spheres import SpherizedUrdf, _vec, rpy_matrix  # noqa: E402

DEFAULT_DIR = "/home/mani/vamp/resources/ur5"


def load_binary_stl(path: Path) -> np.ndarray:
    """Vertices of a binary STL, as an (n, 3) array."""
    data = path.read_bytes()
    count = struct.unpack("<I", data[80:84])[0]
    records = np.frombuffer(data, dtype=np.uint8, count=count * 50, offset=84).reshape(count, 50)
    triangles = np.zeros((count, 3, 3), dtype=np.float32)
    for corner in range(3):  # bytes 0:12 are the normal, then three vertices
        start = 12 + 12 * corner
        triangles[:, corner, :] = records[:, start:start + 12].copy().view(np.float32).reshape(count, 3)
    return triangles.reshape(-1, 3).astype(np.float64)


def collision_meshes(urdf_path: Path) -> dict[str, tuple[str, np.ndarray, np.ndarray]]:
    """Link name -> (mesh file, rotation, translation) for each <collision><mesh>."""
    out = {}
    for link in ET.parse(urdf_path).getroot().iter("link"):
        for collision in link.iter("collision"):
            mesh = collision.find("geometry/mesh")
            if mesh is None:
                continue
            origin = collision.find("origin")
            rpy = origin.get("rpy") if origin is not None else None
            out[link.get("name")] = (
                mesh.get("filename").replace("package://", ""),
                rpy_matrix(*_vec(rpy)) if rpy else np.eye(3),
                _vec(origin.get("xyz")) if origin is not None else np.zeros(3),
            )
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dir", default=DEFAULT_DIR, help="directory holding ur5.urdf and meshes/")
    args = parser.parse_args()
    root = Path(args.dir)

    urdf = SpherizedUrdf(str(root / "ur5_spherized.urdf"))
    by_link: dict[str, list[tuple[np.ndarray, float]]] = {}
    for link, center, radius in urdf.spheres:
        by_link.setdefault(link, []).append((center, radius))

    print(f"{'link':36s} {'verts':>7s} {'max poke-out':>14s} {'verts outside':>14s}")
    worst = 0.0
    for link, (filename, rotation, translation) in collision_meshes(root / "ur5.urdf").items():
        if link not in by_link:
            continue
        path = root / filename
        if path.suffix.lower() != ".stl":
            print(f"{link:36s} {'-':>7s} {'(not an STL)':>14s}")
            continue

        vertices = load_binary_stl(path) @ rotation.T + translation  # into the link frame
        centers = np.array([c for c, _ in by_link[link]])
        radii = np.array([r for _, r in by_link[link]])

        # Signed distance from each vertex to the union of the link's spheres.
        gap = (np.linalg.norm(vertices[:, None, :] - centers[None, :, :], axis=2) - radii[None, :]).min(axis=1)
        worst = max(worst, float(gap.max()))
        print(f"{link:36s} {len(vertices):7d} {gap.max() * 1000:11.1f} mm {100 * (gap > 0).mean():13.1f}%")

    print(f"\nworst uncovered mesh vertex: {worst * 1000:.1f} mm")
    print(f"inflate every radius by >= {worst:.4f} m to make the sphere set a true outer bound")
    return 0


if __name__ == "__main__":
    sys.exit(main())
