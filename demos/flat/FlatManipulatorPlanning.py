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

# Plans a Panda through the sphere cage as a differentially flat system.
# The planner's states are joint angles stacked with joint velocities, edges between them are the
# polynomials that spend the least effort getting from one to the other.
#
# The whole flat setup is the handful of lines under "flat setup" below.
# Everything else is the scenario and the reporting.
#
# VAMP's Python package checks collisions, so this needs `pip install vamp-planner`.

import argparse
import sys
import time

import numpy as np
import vamp

from ompl import base as ob
from ompl import geometric as og
from ompl import util as ou

ORDER = 2
SOLVE_SECONDS = 10.0

robot = vamp.panda

# The Panda's velocity limit at each joint in radians per second.
SPEED_LIMITS = np.array([2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61])

# The sphere cage, as the center of each sphere.
CAGE = [
    [0.55, 0, 0.25],
    [0.35, 0.35, 0.25],
    [0, 0.55, 0.25],
    [-0.55, 0, 0.25],
    [-0.35, -0.35, 0.25],
    [0, -0.55, 0.25],
    [0.35, -0.35, 0.25],
    [0.35, 0.35, 0.8],
    [0, 0.55, 0.8],
    [-0.35, 0.35, 0.8],
    [-0.55, 0, 0.8],
    [-0.35, -0.35, 0.8],
    [0, -0.55, 0.8],
    [0.35, -0.35, 0.8],
]
CAGE_RADIUS = 0.2


def sphereCage():
    """Construct a sphere cage environment."""
    environment = vamp.Environment()
    for center in CAGE:
        environment.add_sphere(vamp.Sphere(center, CAGE_RADIUS))
    return environment


def atRest(space, state, joints):
    """Put the arm at joints and hold it still there."""
    flat = np.zeros((ORDER, robot.dimension()))
    flat[0] = joints
    space.fromFlatState(flat, state)


def plan(seed=None):
    if seed is not None:
        ou.RNG.setSeed(seed)

    environment = sphereCage()
    dimension = robot.dimension()

    # The flat setup.
    # The joints move independently under separate velocity limits, so the velocity level is a box with
    # a side per joint.
    output = ob.RealVectorStateSpace(dimension)
    joints = ob.RealVectorBounds(dimension)
    for i, (low, high) in enumerate(zip(robot.lower_bounds(), robot.upper_bounds())):
        joints.setLow(i, float(low))
        joints.setHigh(i, float(high))
    output.setBounds(joints)

    space = ob.FlatStateSpace(output, ORDER)

    speeds = ob.RealVectorBounds(dimension)
    for i, limit in enumerate(SPEED_LIMITS):
        speeds.setLow(i, -limit)
        speeds.setHigh(i, limit)
    space.getDerivativeSpace(1).setBounds(speeds)

    setup = og.SimpleSetup(space)

    resolution = 1.0 / (robot.resolution() * output.getMaximumExtent())
    setup.getSpaceInformation().setStateValidityCheckingResolution(resolution)

    # The collision checker only knows about joint angles, so it gets the flat output and nothing else.
    def isStateValid(state):
        return space.satisfiesBounds(state) and robot.validate(
            state.output()[0:dimension], environment
        )

    setup.setStateValidityChecker(isStateValid)

    setup.setOptimizationObjective(ob.FlatEffortObjective(setup.getSpaceInformation()))

    # The arm starts tucked and finishes reaching across itself, both at rest.
    start = space.allocState()
    goal = space.allocState()
    atRest(space, start, [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
    atRest(space, goal, [2.35, 1.0, 0.0, -0.8, 0.0, 2.5, 0.785])
    setup.setStartAndGoalStates(start, goal)

    # RRTConnect supports asymmetric edges.
    setup.setPlanner(og.RRTConnect(setup.getSpaceInformation()))

    began = time.perf_counter()
    status = setup.solve(SOLVE_SECONDS)
    elapsed = time.perf_counter() - began

    if status != ob.PlannerStatus.EXACT_SOLUTION:
        print(f"No solution in {SOLVE_SECONDS} seconds")
        return False

    path = setup.getSolutionPath()
    print(f"Solved in {elapsed} seconds over {path.getStateCount()} states")
    print(f"The path passes its own validity check: {path.check()}")

    # A controller runs the trajectory.
    trajectory = ob.FlatTrajectory(path)
    print(
        f"Trajectory of {trajectory.size()} segments lasting {trajectory.duration()} seconds"
    )
    print(f"Steering cost {path.cost(setup.getOptimizationObjective()).value()}")

    # Sampling the whole run confirms every joint held to its limit everywhere, not only at waypoints.
    samples = 20000
    fastest = np.zeros(dimension)
    for i in range(samples + 1):
        rate = trajectory.evaluate(trajectory.duration() * i / samples, 1)
        fastest = np.maximum(fastest, np.abs(rate))

    print(f"Fastest each joint moves over the run {fastest}")
    print(f"Against caps of {SPEED_LIMITS}")
    return bool(np.all(fastest <= SPEED_LIMITS))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Plan a Panda through the sphere cage as a differentially flat system."
    )
    parser.add_argument("--seed", type=int, help="seed for the random number generator")
    args = parser.parse_args()
    sys.exit(0 if plan(args.seed) else 1)
