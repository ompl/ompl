#!/usr/bin/env python
"""Overlay ``ompl::robots::UR5``'s collision spheres on the real UR5 in PyBullet.

This is a verification harness, not part of the planner. It loads the UR5 URDF
and, for each configuration in the dump written by ``demo_UR5Sphere``, draws the
sphere centers the C++ side computed. If the C++ forward kinematics were wrong,
the spheres would visibly drift off the links as the joints sweep.

It also reports the hard number: PyBullet runs its own forward kinematics from
the same URDF, so the two can be compared directly. Every sphere is checked
against the pose PyBullet reports for the link it was authored on.

    ./build/demos/demo_UR5Sphere /tmp/ur5_spheres.json
    python scripts/ur5_sphere_viz.py /tmp/ur5_spheres.json --gui

It also replays a planned path, which ``demo_UR5CBFPlanning`` writes in the same
format given a dump path as its third argument:

    ./build/demos/demo_UR5CBFPlanning 2 1 /tmp/path.json
    python scripts/ur5_sphere_viz.py /tmp/path.json --gui

The end-effector's swept path is drawn along with it (``--no-ee-trace`` to omit).
On a CBF path that curve is the interesting part: where it bends away from a
straight line is where the filter deflected the rollout.

Without ``--gui`` it runs headless and only prints the comparison, which is the
useful mode over ssh.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import numpy as np
import pybullet as p

DEFAULT_URDF = "/home/mani/vamp/resources/ur5/ur5.urdf"
ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
SPHERE_RGBA = (0.95, 0.80, 0.15, 0.35)
OBSTACLE_RGBA = (0.85, 0.15, 0.15, 0.55)
TRACE_RGB = (0.10, 0.55, 0.95)
TRACE_RGBA = TRACE_RGB + (0.9,)
# Link the tool frame is read from, in order of preference. VAMP's ur5.urdf has all
# three; other UR5 URDFs drop `ee_link`.
EE_LINK_CANDIDATES = ("ee_link", "tool0", "wrist_3_link")


def movable_joints(body: int) -> dict[str, int]:
    out = {}
    for j in range(p.getNumJoints(body)):
        info = p.getJointInfo(body, j)
        if info[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
            out[info[1].decode()] = j
    return out


def link_indices(body: int) -> dict[str, int]:
    """Link name -> PyBullet link index (base link is -1)."""
    out = {p.getBodyInfo(body)[0].decode(): -1}
    for j in range(p.getNumJoints(body)):
        out[p.getJointInfo(body, j)[12].decode()] = j
    return out


def set_configuration(body: int, joints: dict[str, int], q) -> None:
    for name, value in zip(ARM_JOINTS, q):
        p.resetJointState(body, joints[name], float(value))


def link_pose(body: int, index: int):
    """World pose of a link's URDF frame (not its inertial frame)."""
    if index == -1:
        pos, orn = p.getBasePositionAndOrientation(body)
        return np.asarray(pos), np.asarray(orn)
    state = p.getLinkState(body, index, computeForwardKinematics=True)
    return np.asarray(state[4]), np.asarray(state[5])  # worldLinkFramePosition/Orientation


def compare_against_pybullet(body, joints, links, dump) -> float:
    """Max distance between a C++ sphere center and the same point placed by PyBullet.

    The C++ center is re-expressed in its source link's frame using the first
    configuration, then PyBullet's own FK moves it for every other configuration.
    Any disagreement in joint origins or axis conventions shows up here.
    """
    names = dump["links"]
    configs = dump["configs"]

    # Recover each sphere's offset in its source link frame, from config 0.
    set_configuration(body, joints, configs[0]["q"])
    local = []
    for name, center in zip(names, configs[0]["centers"]):
        pos, orn = link_pose(body, links[name])
        inv_pos, inv_orn = p.invertTransform(pos.tolist(), orn.tolist())
        local.append(p.multiplyTransforms(inv_pos, inv_orn, list(center), [0, 0, 0, 1])[0])

    worst = 0.0
    for config in configs:
        set_configuration(body, joints, config["q"])
        poses = {name: link_pose(body, links[name]) for name in set(names)}
        for name, offset, center in zip(names, local, config["centers"]):
            pos, orn = poses[name]
            expected = p.multiplyTransforms(pos.tolist(), orn.tolist(), list(offset), [0, 0, 0, 1])[0]
            worst = max(worst, float(np.linalg.norm(np.asarray(expected) - np.asarray(center))))
    return worst


def make_sphere_bodies(radii):
    bodies = []
    for radius in radii:
        visual = p.createVisualShape(p.GEOM_SPHERE, radius=float(radius), rgbaColor=SPHERE_RGBA)
        bodies.append(p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual))
    return bodies


def camera_framing(centers) -> tuple[list[float], float]:
    """Center and distance that fit the whole sphere set in view."""
    centers = np.asarray(centers)
    target = 0.5 * (centers.min(axis=0) + centers.max(axis=0))
    extent = float(np.linalg.norm(centers.max(axis=0) - centers.min(axis=0)))
    return target.tolist(), max(1.2, 1.5 * extent)


def save_screenshot(path: str, centers, width: int = 900, height: int = 900) -> None:
    """Render offscreen with PyBullet's software renderer, so no display is needed."""
    target, distance = camera_framing(centers)
    view = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=target, distance=distance,
                                               yaw=55, pitch=-20, roll=0, upAxisIndex=2)
    projection = p.computeProjectionMatrixFOV(fov=55, aspect=width / height, nearVal=0.05, farVal=8.0)
    image = p.getCameraImage(width, height, view, projection, renderer=p.ER_TINY_RENDERER)
    rgb = np.reshape(image[2], (height, width, 4))[:, :, :3].astype(np.uint8)

    # Minimal PNG writer: no imaging dependency needed for a debug screenshot.
    import struct
    import zlib

    raw = b"".join(b"\x00" + rgb[row].tobytes() for row in range(height))

    def chunk(kind: bytes, payload: bytes) -> bytes:
        return (struct.pack(">I", len(payload)) + kind + payload
                + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF))

    with open(path, "wb") as handle:
        handle.write(b"\x89PNG\r\n\x1a\n")
        handle.write(chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)))
        handle.write(chunk(b"IDAT", zlib.compress(raw, 6)))
        handle.write(chunk(b"IEND", b""))


def end_effector_link(links: dict[str, int], preferred: str | None) -> tuple[str, int]:
    """Which link to trace. Explicit choice wins; otherwise the first candidate present."""
    if preferred is not None:
        if preferred not in links:
            raise KeyError(f"URDF has no link {preferred!r}; available: {sorted(links)}")
        return preferred, links[preferred]
    for name in EE_LINK_CANDIDATES:
        if name in links:
            return name, links[name]
    raise KeyError(f"none of {EE_LINK_CANDIDATES} in URDF; pass --ee-link")


def ee_positions(body, joints, index, configs) -> np.ndarray:
    """World position of the tool frame at every configuration in the dump.

    PyBullet's own forward kinematics is used rather than anything from the dump,
    which only carries sphere centers. `compare_against_pybullet` has already shown
    the two kinematics agree, so this is the same arm the C++ side planned for.
    """
    out = np.empty((len(configs), 3))
    for row, config in enumerate(configs):
        set_configuration(body, joints, config["q"])
        out[row] = link_pose(body, index)[0]
    return out


def thinned(points: np.ndarray, limit: int) -> np.ndarray:
    """At most `limit` points, evenly spaced along the sequence, endpoints kept.

    An audited path is interpolated to every propagation step, so it can run to
    thousands of configurations -- more line segments than are worth drawing.
    """
    if len(points) <= limit:
        return points
    return points[np.linspace(0, len(points) - 1, limit).round().astype(int)]


def draw_ee_trace_lines(points: np.ndarray, width: float, limit: int = 2000) -> None:
    """Debug lines: crisp and cheap, but only visible in the GUI -- `getCameraImage`
    does not render them, which is what `draw_ee_trace_markers` is for."""
    path = thinned(points, limit)
    for start, end in zip(path[:-1], path[1:]):
        p.addUserDebugLine(start.tolist(), end.tolist(), TRACE_RGB, lineWidth=width)


def draw_ee_trace_markers(points: np.ndarray, radius: float = 0.007, limit: int = 250) -> None:
    """Real bodies, so the trace survives an offscreen render. One visual shape shared
    across every marker, since the count is the only thing that costs anything."""
    path = thinned(points, limit)
    shape = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=TRACE_RGBA)
    for point in path:
        p.createMultiBody(baseVisualShapeIndex=shape, basePosition=point.tolist())


def make_obstacle_bodies(obstacles) -> list[int]:
    """Draw the scene the planner avoided. Without these the overlay shows an arm
    striking poses for no visible reason."""
    bodies = []
    for obstacle in obstacles:
        shape = p.createVisualShape(p.GEOM_SPHERE, radius=obstacle["radius"], rgbaColor=OBSTACLE_RGBA)
        bodies.append(p.createMultiBody(baseVisualShapeIndex=shape,
                                        basePosition=list(obstacle["center"])))
    return bodies


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dump", help="JSON written by demo_UR5Sphere or demo_UR5CBFPlanning")
    parser.add_argument("--urdf", default=DEFAULT_URDF)
    parser.add_argument("--gui", action="store_true", help="open a window and animate the sweep")
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--loops", type=int, default=1, help="times to replay the sweep (0 = forever)")
    parser.add_argument("--screenshot", metavar="PATH",
                        help="render one configuration offscreen to a PNG instead of animating")
    parser.add_argument("--config", type=int, default=0, help="configuration index to screenshot")
    parser.add_argument("--ee-link", default=None,
                        help=f"link to trace; default is the first of {EE_LINK_CANDIDATES} present")
    parser.add_argument("--no-ee-trace", dest="ee_trace", action="store_false",
                        help="do not draw the end-effector path")
    parser.add_argument("--ee-width", type=float, default=2.5, help="trace line width in the GUI")
    args = parser.parse_args()

    with open(args.dump) as handle:
        dump = json.load(handle)
    radii, configs = dump["radii"], dump["configs"]
    print(f"{len(radii)} spheres, {len(configs)} configurations from {args.dump}")

    p.connect(p.GUI if args.gui else p.DIRECT)
    p.setGravity(0, 0, 0)
    body = p.loadURDF(args.urdf, useFixedBase=True)
    joints = movable_joints(body)
    links = link_indices(body)

    missing = sorted(set(dump["links"]) - set(links))
    if missing:
        print(f"URDF is missing links the dump refers to: {missing}", file=sys.stderr)
        return 1

    obstacles = dump.get("obstacles", [])
    if obstacles:
        make_obstacle_bodies(obstacles)
        print(f"{len(obstacles)} obstacle sphere(s) in the scene")

    worst = compare_against_pybullet(body, joints, links, dump)
    print(f"max disagreement with PyBullet's forward kinematics: {worst:.2e} m over "
          f"{len(configs)} configurations x {len(radii)} spheres")

    trace = None
    if args.ee_trace:
        try:
            ee_name, ee_index = end_effector_link(links, args.ee_link)
        except KeyError as error:
            print(error, file=sys.stderr)
            return 1
        trace = ee_positions(body, joints, ee_index, configs)
        travelled = float(np.linalg.norm(np.diff(trace, axis=0), axis=1).sum())
        straight = float(np.linalg.norm(trace[-1] - trace[0]))
        print(f"end-effector ({ee_name}) travels {travelled:.3f} m over a "
              f"{straight:.3f} m straight-line gap")

    if args.screenshot:
        config = configs[args.config]
        set_configuration(body, joints, config["q"])
        for sphere, center in zip(make_sphere_bodies(radii), config["centers"]):
            p.resetBasePositionAndOrientation(sphere, list(center), [0, 0, 0, 1])
        framing = config["centers"] + [o["center"] for o in obstacles]
        if trace is not None:
            # Markers rather than debug lines: ER_TINY_RENDERER draws bodies only.
            draw_ee_trace_markers(trace)
            framing = framing + trace.tolist()
        save_screenshot(args.screenshot, framing)
        print(f"wrote {args.screenshot} (configuration {args.config}, q = "
              f"{np.round(config['q'], 3).tolist()})")
        p.disconnect()
        return 0

    if not args.gui:
        p.disconnect()
        return 0

    framing = ([c for config in configs for c in config["centers"]]
               + [o["center"] for o in obstacles])
    if trace is not None:
        framing = framing + trace.tolist()
    target, distance = camera_framing(framing)
    p.resetDebugVisualizerCamera(cameraDistance=distance, cameraYaw=50, cameraPitch=-25,
                                 cameraTargetPosition=target)
    spheres = make_sphere_bodies(radii)

    # Drawn once, before the sweep, so the whole path is visible from the first frame
    # rather than accumulating behind the arm.
    if trace is not None:
        draw_ee_trace_lines(trace, args.ee_width)

    loop = 0
    while args.loops == 0 or loop < args.loops:
        for config in configs:
            set_configuration(body, joints, config["q"])
            for sphere, center in zip(spheres, config["centers"]):
                p.resetBasePositionAndOrientation(sphere, list(center), [0, 0, 0, 1])
            time.sleep(1.0 / args.fps)
        loop += 1

    p.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
