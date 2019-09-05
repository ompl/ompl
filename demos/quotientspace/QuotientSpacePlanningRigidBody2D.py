#!/usr/bin/env python

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

from os.path import abspath, dirname, join
import sys
try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    sys.path.insert(0, join(dirname(dirname(dirname(abspath(__file__)))), 'py-bindings'))
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
from math import sqrt, pi

# Path Planning in SE2 = R2 \times SO2
# using quotient-spaces R2 and SE2

def boxConstraint(x, y):
    x = x - 0.5
    y = y - 0.5
    pos_cnstr = sqrt(x * x + y * y)
    return pos_cnstr > 0.2

def isStateValid_SE2(state):
    return boxConstraint(state.getX(), state.getY()) and state.getYaw() < pi / 2.0

def isStateValid_R2(state):
    return boxConstraint(state[0], state[1])

if __name__ == "__main__":
    # Setup SE2
    SE2 = ob.SE2StateSpace()
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0)
    bounds.setHigh(1)
    SE2.setBounds(bounds)
    si_SE2 = ob.SpaceInformation(SE2)
    si_SE2.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid_SE2))

    # Setup Quotient-Space R2
    R2 = ob.RealVectorStateSpace(2)
    R2.setBounds(0, 1)
    si_R2 = ob.SpaceInformation(R2)
    si_R2.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid_R2))

    # Create vector of spaceinformationptr
    si_vec = og.vectorSpaceInformation()
    si_vec.append(si_R2)
    si_vec.append(si_SE2)

    # Define Planning Problem
    start_SE2 = ob.State(SE2)
    goal_SE2 = ob.State(SE2)
    start_SE2().setXY(0, 0)
    start_SE2().setYaw(0)
    goal_SE2().setXY(1, 1)
    goal_SE2().setYaw(0)

    pdef = ob.ProblemDefinition(si_SE2)
    pdef.setStartAndGoalStates(start_SE2, goal_SE2)

    # Setup Planner using vector of spaceinformationptr
    planner = og.QRRT(si_vec)

    # Planner can be used as any other OMPL algorithm
    planner.setProblemDefinition(pdef)
    planner.setup()

    solved = planner.solve(1.0)

    if solved:
        print('-' * 80)
        print('Configuration-Space Path (SE2):')
        print('-' * 80)
        print(pdef.getSolutionPath())

        print('-' * 80)
        print('Quotient-Space Path (R2):')
        print('-' * 80)
        print(planner.getProblemDefinition(0).getSolutionPath())

        nodes = planner.getFeasibleNodes()
        print('-' * 80)
        for (i, node) in enumerate(nodes):
            print('QuotientSpace%d has %d nodes.' % (i, node))
