#!/usr/bin/env python

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2018, Rice University
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

try:
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(dirname(abspath(__file__)))), 'py-bindings'))
    from ompl import base as ob
    from ompl import geometric as og
import numpy as np

class SphereConstraint(ob.Constraint):
    def __init__(self):
        super(SphereConstraint, self).__init__(3, 1)
    def function(self, x, out):
        out[0] = np.linalg.norm(x) - 1
    # def jacobian(self, x, out):
    #     nrm = np.linalg.norm(x)
    #     out = np.transpose(x/nrm if nrm > 0 else x)

def obstacles(x):
    if -0.8 < x[2] and x[2] < -0.6:
        if -0.05 < x[1] and x[1] < 0.05:
            return x[0] > 0
        return False
    elif -0.1 < x[2] and x[2] < 0.1:
        if -0.05 < x[0] and x[0] < 0.05:
            return x[1] < 0
        return False
    elif 0.6 < x[2] and x[2] < 0.8:
        if -0.05 < x[1] and x[1] < 0.05:
            return x[0] < 0
        return False
    return True

def plan():
    # Create the ambient space state space for the problem.
    rvss = ob.RealVectorStateSpace(3)
    bounds = ob.RealVectorBounds(3)
    bounds.setLow(-2)
    bounds.setHigh(2)
    rvss.setBounds(bounds)

    # Create our constraint.
    constraint = SphereConstraint()

    # Combine the ambient state space and the constraint to create the
    # constrained state space.
    css = ob.ProjectedStateSpace(rvss, constraint)
    csi = ob.ConstrainedSpaceInformation(css)
    ss = og.SimpleSetup(csi)

    ss.setStateValidityChecker(ob.StateValidityCheckerFn(obstacles))

    csi.setValidStateSamplerAllocator(ob.ValidStateSamplerAllocator(
        lambda si: ob.ConstrainedValidStateSampler(si)))
    start = ob.State(css)
    goal = ob.State(css)
    start[0] = 0
    start[1] = 0
    start[2] = -1
    goal[0] = 0
    goal[1] = 0
    goal[2] = 1
    ss.setStartAndGoalStates(start, goal)

    planner = og.RRTConnect(csi)
    ss.setPlanner(planner)

    ss.setup()

    result = ss.solve(15.)
    if result:
        path = ss.getSolutionPath()

        print("Simplifying solution...")
        originalLength = path.length()
        ss.simplifySolution(5.)
        print("Path Length %g -> %g " % (originalLength, path.length()))

        if result.getStatus() == ob.PlannerStatus.APPROXIMATE_SOLUTION:
            print("Solution is approximate.")

        print("Interpolating path...")
        path.interpolate()

        print("Dumping animation file...")

        with open("sphere_path.txt", 'w') as f:
            print(path.printAsMatrix(), file=f)
    else:
        print("No solution found.")

if __name__ == "__main__":
    plan()
