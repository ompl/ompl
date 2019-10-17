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
from functools import partial
import numpy as np
from ConstrainedPlanningCommon import *

PI2 = 2 * math.pi

# Torus manifold.


class TorusConstraint(ob.Constraint):

    def __init__(self, outer, inner, maze):
        super(TorusConstraint, self).__init__(3, 1)
        self.outer = outer
        self.inner = inner
        self.ppm = ou.PPM()
        self.ppm.loadFile(maze)

    def getStartAndGoalStates(self):
        h = self.ppm.getHeight()
        w = self.ppm.getWidth()

        for x in range(w):
            for y in range(h):
                p = np.array([x / (w - 1.), y / (h - 1.)], dtype=np.float64)
                c = self.ppm.getPixel(x, y)
                if c.red == 255 and c.blue == 0 and c.green == 0:
                    start = self.mazeToAmbient(p)
                elif c.green == 255 and c.blue == 0 and c.red == 0:
                    goal = self.mazeToAmbient(p)
        return start, goal

    def function(self, x, out):
        c = np.array([x[0], x[1], 0])
        nrm = math.sqrt(x[0] * x[0] + x[1] * x[1])
        if not np.isfinite(nrm) or nrm == 0:
            nrm = 1
        out[0] = np.linalg.norm(x - self.outer * c / nrm) - self.inner

    def jacobian(self, x, out):
        xySquaredNorm = x[0] * x[0] + x[1] * x[1]
        xyNorm = math.sqrt(xySquaredNorm)
        denom = math.sqrt(x[2] * x[2] + (xyNorm - self.outer)
                          * (xyNorm - self.outer))
        c = (xyNorm - self.outer) * (xyNorm * xySquaredNorm) / \
            (xySquaredNorm * xySquaredNorm * denom)
        out[0, :] = [x[0] * c, x[1] * c, x[2] / denom]

    def ambientToMaze(self, x):
        nrm = math.sqrt(x[0] * x[0] + x[1] * x[1])
        h = self.ppm.getHeight()
        w = self.ppm.getWidth()

        mx = math.atan2(x[2], nrm - self.outer) / PI2
        mx += (mx < 0)
        mx *= h
        my = math.atan2(x[1], x[0]) / PI2
        my += (my < 0)
        my *= w
        return mx, my

    def mazeToAmbient(self, x):
        a = x * PI2
        b = [math.cos(a[0]), 0, math.sin(a[0])] * self.inner
        b[0] += self.outer

        norm = math.sqrt(b[0] * b[0] + b[1] * b[1])
        out = np.array([math.cos(a[1]), math.sin(a[1]), 0], dtype=np.float64)
        out *= norm
        out[2] = b[2]
        return out

    def mazePixel(self, x):
        h = self.ppm.getHeight()
        w = self.ppm.getWidth()

        if x[0] < 0 or x[0] >= w or x[1] < 0 or x[1] >= h:
            return False

        c = self.ppm.getPixel(int(x[0]), int(x[1]))
        return not (c.red == 0 and c.blue == 0 and c.green == 0)

    def isValid(self, state):
        return self.mazePixel(self.ambientToMaze(state))


def torusPlanningBench(cp, planners):
    print(planners)
    cp.setupBenchmark(planners, "torus")
    cp.runBenchmark()


def torusPlanningOnce(cp, planner, output):
    cp.setPlanner(planner)

    # Solve the problem
    stat = cp.solveOnce(output, "torus")

    if output:
        ou.OMPL_INFORM("Dumping problem information to `torus_info.txt`.")
        with open("torus_info.txt", "w") as infofile:
            print(cp.spaceType, file=infofile)

    cp.atlasStats()

    if output:
        cp.dumpGraph("torus")

    return stat


def torusPlanning(options):
    # Create the ambient space state space for the problem.
    rvss = ob.RealVectorStateSpace(3)

    bounds = ob.RealVectorBounds(3)
    bounds.setLow(-(options.outer + options.inner))
    bounds.setHigh(options.outer + options.inner)

    rvss.setBounds(bounds)

    # Create our constraint.
    constraint = TorusConstraint(options.outer, options.inner, options.maze)

    cp = ConstrainedProblem(options.space, rvss, constraint, options)

    start, goal = constraint.getStartAndGoalStates()
    print("Start = ", start)
    print("Goal = ", goal)

    sstart = ob.State(cp.css)
    sgoal = ob.State(cp.css)
    for i in range(3):
        sstart[i] = start[i]
        sgoal[i] = goal[i]
    cp.setStartAndGoalStates(sstart, sgoal)
    cp.ss.setStateValidityChecker(ob.StateValidityCheckerFn(partial(
        TorusConstraint.isValid, constraint)))

    planners = options.planner.split(",")
    if not options.bench:
        torusPlanningOnce(cp, planners[0], options.output)
    else:
        torusPlanningBench(cp, planners)

if __name__ == "__main__":
    defaultMaze = join(join(dirname(__file__), "mazes"), "normal.ppm")
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output", action="store_true",
                        help="Dump found solution path (if one exists) in plain text and planning "
                        "graph in GraphML to `torus_path.txt` and `torus_graph.graphml` "
                        "respectively.")
    parser.add_argument("--bench", action="store_true",
                        help="Do benchmarking on provided planner list.")
    parser.add_argument("--outer", type=float, default=2,
                        help="Outer radius of torus.")
    parser.add_argument("--inner", type=float, default=1,
                        help="Inner radius of torus.")
    parser.add_argument("--maze", default=defaultMaze,
                        help="Filename of maze image (in .ppm format) to use as obstacles on the "
                        "surface of the torus.")
    addSpaceOption(parser)
    addPlannerOption(parser)
    addConstrainedOptions(parser)
    addAtlasOptions(parser)

    torusPlanning(parser.parse_args())
