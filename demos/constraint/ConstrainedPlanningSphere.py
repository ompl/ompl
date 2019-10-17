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

from __future__ import print_function
import argparse
import math
import numpy as np
from ConstrainedPlanningCommon import *


class SphereConstraint(ob.Constraint):

    def __init__(self):
        super(SphereConstraint, self).__init__(3, 1)

    def function(self, x, out):
        out[0] = np.linalg.norm(x) - 1

    def jacobian(self, x, out):
        nrm = np.linalg.norm(x)
        if np.isfinite(nrm) and nrm > 0:
            out[0, :] = x / nrm
        else:
            out[0, :] = [1, 0, 0]


class SphereProjection(ob.ProjectionEvaluator):

    def __init__(self, space):
        super(SphereProjection, self).__init__(space)

    def getDimension(self):
        return 2

    def defaultCellSizes(self):
        self.cellSizes_ = list2vec([.1, .1])

    def project(self, state, projection):
        projection[0] = math.atan2(state[1], state[0])
        projection[1] = math.acos(state[2])


def obstacles(x):
    if x[2] > -0.8 and x[2] < -0.6:
        if x[1] > -0.05 and x[1] < 0.05:
            return x[0] > 0
        return False
    elif x[2] > -0.1 and x[2] < 0.1:
        if x[0] > -0.05 and x[0] < 0.05:
            return x[1] < 0
        return False
    elif x[2] > 0.6 and x[2] < 0.8:
        if x[1] > -0.05 and x[1] < 0.05:
            return x[0] < 0
        return False
    return True


def spherePlanningOnce(cp, plannername, output):
    cp.setPlanner(plannername, "sphere")

    # Solve the problem
    stat = cp.solveOnce(output, "sphere")

    if output:
        ou.OMPL_INFORM("Dumping problem information to `sphere_info.txt`.")
        with open("sphere_info.txt", "w") as infofile:
            print(cp.spaceType, file=infofile)

    cp.atlasStats()
    if output:
        cp.dumpGraph("sphere")
    return stat


def spherePlanningBench(cp, planners):
    cp.setupBenchmark(planners, "sphere")
    cp.runBenchmark()


def spherePlanning(options):
    # Create the ambient space state space for the problem.
    rvss = ob.RealVectorStateSpace(3)
    bounds = ob.RealVectorBounds(3)
    bounds.setLow(-2)
    bounds.setHigh(2)
    rvss.setBounds(bounds)

    # Create our constraint.
    constraint = SphereConstraint()

    cp = ConstrainedProblem(options.space, rvss, constraint, options)
    cp.css.registerProjection("sphere", SphereProjection(cp.css))

    start = ob.State(cp.css)
    goal = ob.State(cp.css)
    start[0] = 0
    start[1] = 0
    start[2] = -1
    goal[0] = 0
    goal[1] = 0
    goal[2] = 1
    cp.setStartAndGoalStates(start, goal)
    cp.ss.setStateValidityChecker(ob.StateValidityCheckerFn(obstacles))

    planners = options.planner.split(",")
    if not options.bench:
        spherePlanningOnce(cp, planners[0], options.output)
    else:
        spherePlanningBench(cp, planners)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output", action="store_true",
                        help="Dump found solution path (if one exists) in plain text and planning "
                        "graph in GraphML to `sphere_path.txt` and `sphere_graph.graphml` "
                        "respectively.")
    parser.add_argument("--bench", action="store_true",
                        help="Do benchmarking on provided planner list.")
    addSpaceOption(parser)
    addPlannerOption(parser)
    addConstrainedOptions(parser)
    addAtlasOptions(parser)

    spherePlanning(parser.parse_args())
