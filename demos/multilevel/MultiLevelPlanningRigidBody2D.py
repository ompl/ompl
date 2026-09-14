#!/usr/bin/env python3

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2019, Rice University
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

# Author: Mark Moll

from math import pi, sqrt

from ompl import base as ob
from ompl import multilevel as ml


def box_constraint(x, y):
    x = x - 0.5
    y = y - 0.5
    return sqrt(x * x + y * y) > 0.2


def is_state_valid_se2(state):
    return box_constraint(state.getX(), state.getY()) and state.getYaw() < pi / 2.0


def is_state_valid_r2(state):
    return box_constraint(state[0], state[1])


def plan():
    se2 = ob.SE2StateSpace()
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0)
    bounds.setHigh(1)
    se2.setBounds(bounds)

    si_se2 = ob.SpaceInformation(se2)
    si_se2.setStateValidityChecker(ob.StateValidityCheckerFn(is_state_valid_se2))
    si_se2.setup()

    r2 = ob.RealVectorStateSpace(2)
    r2.setBounds(0, 1)
    si_r2 = ob.SpaceInformation(r2)
    si_r2.setStateValidityChecker(ob.StateValidityCheckerFn(is_state_valid_r2))
    si_r2.setup()

    start = ob.State(se2)
    goal = ob.State(se2)
    start().setXY(0, 0)
    start().setYaw(0)
    goal().setXY(1, 1)
    goal().setYaw(0)

    pdef = ob.ProblemDefinition(si_se2)
    pdef.setStartAndGoalStates(start, goal)

    planner = ml.QRRT([si_r2, si_se2])
    planner.setProblemDefinition(pdef)
    planner.setup()

    solved = planner.solve(5.0)
    if solved:
        print("-" * 80)
        print("Configuration-space path (SE2):")
        print("-" * 80)
        print(pdef.getSolutionPath())

        print("-" * 80)
        print("Quotient-space path (R2):")
        print("-" * 80)
        print(planner.getProblemDefinition(0).getSolutionPath())


if __name__ == "__main__":
    plan()
