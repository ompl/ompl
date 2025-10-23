#!/usr/bin/env python3
import argparse
from pathlib import Path

import numpy as np

import vamp
from vamp import pybullet_interface as vpb


def load_path(path_file: Path) -> np.ndarray:
    data = []
    with open(path_file, 'r') as f:
        for line in f:
            vals = [float(x) for x in line.strip().split() if x]
            if vals:
                data.append(vals)
    if not data:
        raise RuntimeError(f"No states loaded from {path_file}")
    # Ensure consistent dimensionality
    dim = len(data[0])
    for row in data:
        if len(row) != dim:
            raise RuntimeError("Inconsistent state dimensionality in path file")
    return np.asarray(data, dtype=float)


def build_sphere_cage(sim: vpb.PyBulletSimulator):
    spheres = [
        (0.55, 0.0, 0.25), (0.35, 0.35, 0.25), (0.0, 0.55, 0.25),
        (-0.55, 0.0, 0.25), (-0.35, -0.35, 0.25), (0.0, -0.55, 0.25),
        (0.35, -0.35, 0.25), (0.35, 0.35, 0.8), (0.0, 0.55, 0.8),
        (-0.35, 0.35, 0.8), (-0.55, 0.0, 0.8), (-0.35, -0.35, 0.8),
        (0.0, -0.55, 0.8), (0.35, -0.35, 0.8),
    ]
    for p in spheres:
        sim.add_sphere(0.15, list(p), name="obstacle")


def main():
    parser = argparse.ArgumentParser(description="Visualize VAMP OMPL demo path with PyBullet")
    parser.add_argument("--path", type=str, default="vamp_path.txt", help="Path file exported by demo_Vamp")
    parser.add_argument("--robot", type=str, default="panda", choices=["panda", "ur5", "fetch", "baxter"], help="Robot model")
    args = parser.parse_args()

    path_file = Path(args.path)
    plan = load_path(path_file)

    # Setup simulator with VAMP URDFs and joint order
    robot_dir = Path(__file__).resolve().parents[2] / "external" / "vamp" / "resources" / args.robot
    urdf = robot_dir / f"{args.robot}_spherized.urdf"
    joint_names = getattr(vamp, args.robot).joint_names()

    sim = vpb.PyBulletSimulator(str(urdf), joint_names, visualize=True)
    build_sphere_cage(sim)

    # Optional: adjust camera
    sim.set_camera([1.2, 1.2, 1.0], [0.0, 0.0, 0.5])

    # Animate plan (NxD array)
    sim.animate(plan)


if __name__ == "__main__":
    main()


