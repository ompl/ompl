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


def normalize(x):
    norm = np.linalg.norm(x)
    if norm > 0 and np.isfinite(norm):
        return x / norm
    else:
        return x


class ChainConstraint(ob.Constraint):

    class Wall(object):
        # Container class for the "wall" obstacles that are on the surface of the
        # sphere constraint (when extra = 1).

        def __init__(self, offset, thickness, width, joint_radius, wallType):
            self.offset = offset
            self.thickness = thickness + joint_radius
            self.width = width + joint_radius
            self.type = wallType

        # Checks if an x coordinate places the sphere within the boundary of
        # the wall.
        def within(self, x):
            if x < (self.offset - self.thickness) or x > (self.offset + self.thickness):
                return False
            return True

        def checkJoint(self, v):
            x, y, z = v

            if not self.within(x):
                return True

            if z <= self.width:
                if self.type == 0:
                    if y < 0:
                        return True
                else:
                    if y > 0:
                        return True
            return False

    WALL_WIDTH = 0.5
    JOINT_RADIUS = 0.2
    LINK_LENGTH = 1.0

    # An implicit kinematic chain, formed out of balls each in R^3, with
    # distance constraints between successive balls creating spherical joint
    # kinematics for the system.
    #
    # Extra constraints are as follows:
    # 1 - End-effector is constrained to be on the surface of a sphere of
    #     radius links - 2
    # 2 - The (links - 5)th and (links - 4)th ball have the same z-value
    # 3 - The (links - 4)th and (links - 3)th ball have the same x-value
    # 4 - The (links - 3)th and (links - 2)th ball have the same z-value
    def __init__(self, links, obstacles=0, extra=1):
        super(ChainConstraint, self).__init__(3 * links, links + extra)
        self.links = links
        self.length = ChainConstraint.LINK_LENGTH
        self.width = ChainConstraint.WALL_WIDTH
        self.radius = links - 2
        self.jointRadius = ChainConstraint.JOINT_RADIUS
        self.obstacles = obstacles
        self.extra = extra
        step = 2. * self.radius / (obstacles + 1.)
        self.walls = [ChainConstraint.Wall(-self.radius + i * step, self.radius / 8.,
                                           self.width, self.jointRadius, i % 2)
                      for i in range(obstacles)]

    def function(self, x, out):
        joint1 = np.zeros(3)
        for i in range(self.links):
            joint2 = x[(3 * i):(3 * i + 3)]
            out[i] = np.linalg.norm(joint1 - joint2) - self.length
            joint1 = joint2

        if self.extra >= 1:
            out[self.links] = np.linalg.norm(x[-3:]) - self.radius

        o = self.links - 5

        if self.extra >= 2:
            out[self.links + 1] = x[(o + 0) * 3 + 2] - x[(o + 1) * 3 + 2]
        if self.extra >= 3:
            out[self.links + 2] = x[(o + 1) * 3 + 0] - x[(o + 2) * 3 + 0]
        if self.extra >= 4:
            out[self.links + 3] = x[(o + 2) * 3 + 2] - x[(o + 3) * 3 + 2]

    def jacobian(self, x, out):
        out[:, :] = np.zeros(
            (self.getCoDimension(), self.getAmbientDimension()), dtype=np.float64)

        plus = np.zeros(3 * (self.links + 1))
        plus[:(3 * self.links)] = x[:(3 * self.links)]

        minus = np.zeros(3 * (self.links + 1))
        minus[-(3 * self.links):] = x[:(3 * self.links)]

        diagonal = plus - minus

        for i in range(self.links):
            out[i, (3 * i):(3 * i + 3)] = normalize(diagonal[(3 * i):(3 * i + 3)])
        out[1:self.links, 0:(3 * self.links - 3)
            ] -= out[1:self.links, 3:(3 * self.links)]

        if self.extra >= 1:
            out[self.links, -3:] = -normalize(diagonal[-3:])

        o = self.links - 5

        if self.extra >= 2:
            out[self.links + 1, (o * 3 + 2):(o * 3 + 5)] = [1, -1]
        if self.extra >= 3:
            out[self.links + 2, (o * 3 + 2):(o * 3 + 5)] = [1, -1]
        if self.extra >= 4:
            out[self.links + 3, (o * 3 + 2):(o * 3 + 5)] = [1, -1]

    # Checks if there are no self-collisions (of the joints themselves) or
    # collisions with the extra obstacles on the surface of the sphere.
    def isValid(self, state):
        x = np.array([state[i] for i in range(self.getAmbientDimension())])
        for i in range(self.links):
            joint = x[(3 * i):(3 * i + 3)]
            if joint[2] < 0:
                return False
            if np.linalg.norm(joint) >= self.radius - self.jointRadius:
                for wall in self.walls:
                    if not wall.checkJoint(joint):
                        return False

        for i in range(self.links):
            joint1 = x[(3 * i):(3 * i + 3)]
            if np.max(np.absolute(joint1)) < self.jointRadius:
                return False

            for j in range(i + 1, self.links):
                joint2 = x[(3 * j):(3 * j + 3)]
                if np.max(np.absolute(joint1 - joint2)) < self.jointRadius:
                    return False

        return True

    def createSpace(self):
        rvss = ob.RealVectorStateSpace(3 * self.links)
        bounds = ob.RealVectorBounds(3 * self.links)

        for i in range(self.links):
            bounds.setLow(3 * i + 0, -i - 1)
            bounds.setHigh(3 * i + 0, i + 1)

            bounds.setLow(3 * i + 1, -i - 1)
            bounds.setHigh(3 * i + 1, i + 1)

            bounds.setLow(3 * i + 2, -i - 1)
            bounds.setHigh(3 * i + 2, i + 1)

        rvss.setBounds(bounds)
        return rvss

    def getStartAndGoalStates(self):
        start = np.zeros(3 * self.links)
        goal = np.zeros(3 * self.links)

        for i in range(self.links - 3):
            start[3 * i] = i + 1
            start[3 * i + 1] = 0
            start[3 * i + 2] = 0

            goal[3 * i] = -(i + 1)
            goal[3 * i + 1] = 0
            goal[3 * i + 2] = 0

        i = self.links - 3

        start[3 * i] = i
        start[3 * i + 1] = -1
        start[3 * i + 2] = 0

        goal[3 * i] = -i
        goal[3 * i + 1] = 1
        goal[3 * i + 2] = 0

        i += 1

        start[3 * i] = i
        start[3 * i + 1] = -1
        start[3 * i + 2] = 0

        goal[3 * i] = -i
        goal[3 * i + 1] = 1
        goal[3 * i + 2] = 0

        i += 1

        start[3 * i] = i - 1
        start[3 * i + 1] = 0
        start[3 * i + 2] = 0

        goal[3 * i] = -(i - 1)
        goal[3 * i + 1] = 0
        goal[3 * i + 2] = 0

        return start, goal

    # Create a projection evaluator for the chain constraint. Finds the
    # spherical coordinates of the end-effector on the surface of the sphere of
    # radius equal to that of the constraint (when extra = 1). */
    def getProjection(self, space):
        class ChainProjection(ob.ProjectionEvaluator):

            def __init__(self, space, links, radius):
                super(ChainProjection, self).__init__(space)
                self.links = links  # Number of chain links.
                # Radius of sphere end-effector lies on (for extra = 1)
                self.radius = radius
                self.defaultCellSizes()

            def getDimension(self):
                return 2

            def defaultCellSizes(self):
                self.cellSizes_ = list2vec([.1, .1])

            def project(self, state, projection):
                s = 3 * (self.links - 1)
                projection[0] = math.atan2(state[s + 1], state[s])
                projection[1] = math.acos(state[s + 2] / self.radius)

        return ChainProjection(space, self.links, self.radius)

    def dump(self, outfile):
        print(self.links, file=outfile)
        print(self.obstacles, file=outfile)
        print(self.extra, file=outfile)
        print(self.jointRadius, file=outfile)
        print(self.length, file=outfile)
        print(self.radius, file=outfile)
        print(self.width, file=outfile)

    def addBenchmarkParameters(self, bench):
        bench.addExperimentParameter("links", "INTEGER", str(self.links))
        bench.addExperimentParameter(
            "obstacles", "INTEGER", str(self.obstacles))
        bench.addExperimentParameter("extra", "INTEGER", str(self.extra))


def chainPlanningOnce(cp, planner, output):
    cp.setPlanner(planner, "chain")

    # Solve the problem
    stat = cp.solveOnce(output, "chain")

    if output:
        ou.OMPL_INFORM("Dumping problem information to `chain_info.txt`.")
        with open("chain_info.txt", "w") as infofile:
            print(cp.spaceType, file=infofile)
            cp.constraint.dump(infofile)

    cp.atlasStats()
    return stat


def chainPlanningBench(cp, planners):
    cp.setupBenchmark(planners, "chain")
    cp.constraint.addBenchmarkParameters(cp.bench)
    cp.runBenchmark()


def chainPlanning(options):
    # Create our constraint.
    constraint = ChainConstraint(
        options.links, options.obstacles, options.extra)

    cp = ConstrainedProblem(
        options.space, constraint.createSpace(), constraint, options)

    cp.css.registerProjection("chain", constraint.getProjection(cp.css))

    start, goal = constraint.getStartAndGoalStates()
    sstart = ob.State(cp.css)
    sgoal = ob.State(cp.css)
    for i in range(cp.css.getDimension()):
        sstart[i] = start[i]
        sgoal[i] = goal[i]
    cp.setStartAndGoalStates(sstart, sgoal)
    cp.ss.setStateValidityChecker(ob.StateValidityCheckerFn(partial(
        ChainConstraint.isValid, constraint)))

    planners = options.planner.split(",")
    if not options.bench:
        chainPlanningOnce(cp, planners[0], options.output)
    else:
        chainPlanningBench(cp, planners)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output", action="store_true",
                        help="Dump found solution path (if one exists) in plain text and planning "
                        "graph in GraphML to `torus_path.txt` and `torus_graph.graphml` "
                        "respectively.")
    parser.add_argument("--bench", action="store_true",
                        help="Do benchmarking on provided planner list.")
    parser.add_argument("-l", "--links", type=int, default=5,
                        help="Number of links in the kinematic chain. Minimum is 4.")
    parser.add_argument("-x", "--obstacles", type=int, default=0, choices=[0, 1, 2],
                        help="Number of `wall' obstacles on the surface of the sphere. Ranges from "
                        "[0, 2]")
    parser.add_argument("-e", "--extra", type=int, default=1,
                        help="Number of extra constraints to add to the chain. Extra constraints "
                        "are as follows:\n"
                        "1: End-effector is constrained to be on the surface of a sphere of radius "
                        "links - 2\n"
                        "2: (links-5)th and (links-4)th ball have the same z-value\n"
                        "3: (links-4)th and (links-3)th ball have the same x-value\n"
                        "4: (links-3)th and (links-2)th ball have the same z-value")
    addSpaceOption(parser)
    addPlannerOption(parser)
    addConstrainedOptions(parser)
    addAtlasOptions(parser)

    chainPlanning(parser.parse_args())
