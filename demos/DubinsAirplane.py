#!/usr/bin/env python

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2024, Metron, Inc.
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
#   * Neither the name of the Metron, Inc. nor the names of its
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

# Author: Mark Moll

import sys
try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
import argparse
from math import fmod, sqrt
from typing import Union

radius = 1.
maxPitch = .5

def allocSpace(space: str):
    bounds = ob.RealVectorBounds(3)
    bounds.setLow(-10)
    bounds.setHigh(10)
    if space.lower() == "vana":
        stateSpace = ob.VanaStateSpace(radius, maxPitch)
        stateSpace.setBounds(bounds)
        return stateSpace
    elif space.lower() == "owen":
        stateSpace = ob.OwenStateSpace(radius, maxPitch)
        stateSpace.setBounds(bounds)
        return stateSpace
    else:
        if space.lower() != "vanaowen":
            ou.OMPL_WARN(f"Unknown state space {space}; defaulting to VanaOwen")
        stateSpace = ob.VanaOwenStateSpace(radius, maxPitch)
        stateSpace.setBounds(bounds)
        return stateSpace

def allocatePlanner(si: ob.SpaceInformation, planner: str):
    if planner.lower() == "rrt":
        return og.RRT(si)
    elif planner.lower() == "rrtstar":
        return og.RRTstar(si)
    elif planner.lower() == "est":
        return og.EST(si)
    elif planner.lower() == "kpiece":
        return og.KPIECE1(si)
    elif planner.lower() == "sst":
        return og.SST(si)
    else:
        ou.OMPL_ERROR(f"Planner type {planner} is not implemented in allocation function.")


def isStateValid(state: Union[ob.OwenState, ob.VanaState, ob.VanaOwenState]):
    dist = 0.
    for i in range(3):
        d = fmod(abs(state[i]), 2. * radius) - radius
        dist += d * d
    return sqrt(dist) > .75 * radius


def plan(space : str, planner : str):
    stateSpace = allocSpace(space)
    start = ob.State(stateSpace)
    goal = ob.State(stateSpace)
    ss = og.SimpleSetup(stateSpace)
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
    while True:
        start.random()
        if isStateValid(start):
            break
    while True:
        goal.random()
        if isStateValid(goal):
            break
    ss.setStartAndGoalStates(start, goal)
    si = ss.getSpaceInformation()
    si.setStateValidityCheckingResolution(0.001)
    ss.setPlanner(allocatePlanner(si, planner))
    ss.setup()
    print(ss)
    result = ss.solve(10.0)
    if result:
        path = ss.getSolutionPath()
        length = path.length()
        path.interpolate()
        print(path.printAsMatrix())

        if result == ob.PlannerStatus.APPROXIMATE_SOLUTION:
            print("Approximate solution. Distance to goal is ", ss.getProblemDefinition().getSolutionDifference())
        print("Path length is ", length)

if __name__ == "__main__":
    # Create an argument parser
    parser = argparse.ArgumentParser(description='Optimal motion planning demo program.')

    # Add a filename argument
    parser.add_argument('-s', '--space', default="VanaOwen", \
        choices=['Owen', 'Vana', 'VanaOwen'], \
        help='Type of 3D Dubins state space to use, defaults to vanaowen.')
    parser.add_argument('-p', '--planner', default='KPIECE', \
        choices=['RRT', 'RRTstar', 'EST', 'KPIECE', 'SST'], \
        help='Planning algorithm to use, defaults to KPIECE if not given.')

    # Parse the arguments
    args = parser.parse_args()
    # Solve the planning problem
    plan(args.space, args.planner)
