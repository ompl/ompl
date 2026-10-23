#!/usr/bin/env python3

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2026, Rice University
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Rice University nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
######################################################################
# Author: Clayton W. Ramsey

# Plans a quadrotor through a room of pillars and out a window as a differentially flat system.
# A quadrotor's flat output is its position and its yaw, so the flat output space here is R^3 and SO(2)
# side by side, and yaw wraps around.
# The rotor forces can then be derived from the flat output state and its derivatives.

import argparse
import math
import sys

import numpy as np

from ompl import base as ob
from ompl import geometric as og
from ompl import util as ou

ORDER = 2
SOLVE_SECONDS = 10.0
GRAVITY = 9.81

# Size of the quadrotor, in meters.
BODY_RADIUS = 0.3

# The quadrotor's limits on speed along each axis, in meters per second, and on yaw rate, in radians per
# second.
SPEED_LIMIT = 3.0
YAW_RATE_LIMIT = 1.5

# The room runs 10 meters on a side and 4 meters up.
HALF_WIDTH = 5.0
HEIGHT = 4.0

# A row of pillars stands on either side of a wall splitting the room at x of zero, and a window in the
# wall is the only way through.
PILLAR_RADIUS = 0.4
PILLARS = [(px, py) for px in (-2.5, 2.5) for py in (-3.0, -1.0, 1.0, 3.0)]
WALL_HALF_THICKNESS = 0.1
WINDOW_LOW_Y, WINDOW_HIGH_Y = 1.0, 2.5
WINDOW_LOW_Z, WINDOW_HIGH_Z = 1.5, 2.8


def collisionFree(x, y, z):
    """Whether a quadrotor centered at x, y, z clears every obstacle in the room."""
    clearance = PILLAR_RADIUS + BODY_RADIUS
    for px, py in PILLARS:
        dx = x - px
        dy = y - py
        if dx * dx + dy * dy < clearance * clearance:
            return False

    if abs(x) < WALL_HALF_THICKNESS + BODY_RADIUS:
        throughWindow = (
            WINDOW_LOW_Y + BODY_RADIUS < y < WINDOW_HIGH_Y - BODY_RADIUS
            and WINDOW_LOW_Z + BODY_RADIUS < z < WINDOW_HIGH_Z - BODY_RADIUS
        )
        if not throughWindow:
            return False

    return BODY_RADIUS < z < HEIGHT - BODY_RADIUS


def hover(space, state, x, y, z, yaw):
    """Hover at x, y, z facing yaw."""
    flat = np.zeros((ORDER, 4))
    flat[0] = [x, y, z, yaw]
    space.fromFlatState(flat, state)


def plan(seed=None):
    if seed is not None:
        ou.RNG.setSeed(seed)

    # The flat setup.
    position = ob.RealVectorStateSpace(3)
    room = ob.RealVectorBounds(3)
    room.setLow(0, -HALF_WIDTH)
    room.setHigh(0, HALF_WIDTH)
    room.setLow(1, -HALF_WIDTH)
    room.setHigh(1, HALF_WIDTH)
    room.setLow(2, 0.0)
    room.setHigh(2, HEIGHT)
    position.setBounds(room)

    output = ob.CompoundStateSpace()
    output.addSubspace(position, 1.0)
    output.addSubspace(ob.SO2StateSpace(), 0.5)

    space = ob.FlatStateSpace(output, ORDER)

    rates = ob.RealVectorBounds(4)
    for i in range(3):
        rates.setLow(i, -SPEED_LIMIT)
        rates.setHigh(i, SPEED_LIMIT)
    rates.setLow(3, -YAW_RATE_LIMIT)
    rates.setHigh(3, YAW_RATE_LIMIT)
    space.getDerivativeSpace(1).setBounds(rates)

    setup = og.SimpleSetup(space)

    # The steering polynomial overshoots, so the validity checker holds every state to its bounds.
    def isStateValid(state):
        p = state.output()[0]
        return space.satisfiesBounds(state) and collisionFree(p[0], p[1], p[2])

    setup.setStateValidityChecker(isStateValid)

    setup.setOptimizationObjective(ob.FlatEffortObjective(setup.getSpaceInformation()))

    # Set start and goal states.
    start = space.allocState()
    goal = space.allocState()
    hover(space, start, -4.0, -4.0, 1.0, 3.0)
    hover(space, goal, 4.0, 4.0, 2.5, -3.0)
    setup.setStartAndGoalStates(start, goal)

    # RRTConnect supports asymmetric edges.
    setup.setPlanner(og.RRTConnect(setup.getSpaceInformation()))

    if setup.solve(SOLVE_SECONDS) != ob.PlannerStatus.EXACT_SOLUTION:
        print(f"No solution in {SOLVE_SECONDS} seconds")
        return False

    # Apply path simplification.
    path = setup.getSolutionPath()
    planned = path.getStateCount()
    og.PathSimplifier(setup.getSpaceInformation()).reduceVertices(path)
    print(f"Solved over {planned} states, reduced to {path.getStateCount()}")
    print(f"The path passes its own validity check: {path.check()}")
    cost = path.cost(setup.getOptimizationObjective()).value()
    print(f"Path costs {cost}")

    trajectory = ob.FlatTrajectory(path)
    print(
        f"Trajectory of {trajectory.size()} segments lasting {trajectory.duration()} seconds"
    )

    # A quadrotor's thrust points along its body axis, so the rotors have to supply the acceleration the
    # trajectory asks for plus whatever holds the airframe up against gravity.
    # Its direction gives the tilt and its size over gravity gives the thrust to weight ratio.
    samples = 20000
    peakThrust = 0.0
    peakTilt = 0.0
    peakSpeed = 0.0
    for i in range(samples + 1):
        t = trajectory.duration() * i / samples
        acceleration = trajectory.evaluate(t, 2)
        rate = trajectory.evaluate(t, 1)

        thrust = np.array([acceleration[0], acceleration[1], acceleration[2] + GRAVITY])
        norm = np.linalg.norm(thrust)
        peakThrust = max(peakThrust, norm / GRAVITY)
        peakTilt = max(peakTilt, math.acos(thrust[2] / norm))
        peakSpeed = max(peakSpeed, np.abs(rate[:3]).max())

    print(
        f"Fastest along any axis {peakSpeed} meters per second against a cap of {SPEED_LIMIT}"
    )
    print(
        f"Peak thrust {peakThrust} times the weight, peak tilt {math.degrees(peakTilt)} degrees"
    )

    # Level 0 of the trajectory unwraps the yaw, so it finishes a whole number of turns from the goal
    # heading rather than jumping at the branch cut.
    print(
        f"Yaw runs from {trajectory.evaluate(0.0)[3]} to "
        f"{trajectory.evaluate(trajectory.duration())[3]} radians unwrapped"
    )

    return path.check()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Plan a quadrotor flight as a differentially flat system."
    )
    parser.add_argument("--seed", type=int, help="seed for the random number generator")
    args = parser.parse_args()
    sys.exit(0 if plan(args.seed) else 1)
