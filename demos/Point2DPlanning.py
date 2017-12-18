#!/usr/bin/env python

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2010, Rice University
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
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
######################################################################

# Author: Ioan Sucan, Mark Moll

from os.path import abspath, dirname, join
import sys
try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
from functools import partial

class Plane2DEnvironment:
    def __init__(self, ppm_file):
        self.ppm_ = ou.PPM()
        self.ppm_.loadFile(ppm_file)
        space = ob.RealVectorStateSpace()
        space.addDimension(0.0, self.ppm_.getWidth())
        space.addDimension(0.0, self.ppm_.getHeight())
        self.maxWidth_ = self.ppm_.getWidth() - 1
        self.maxHeight_ = self.ppm_.getHeight() - 1
        self.ss_ = og.SimpleSetup(space)

        # set state validity checking for this space
        self.ss_.setStateValidityChecker(ob.StateValidityCheckerFn(
            partial(Plane2DEnvironment.isStateValid, self)))
        space.setup()
        self.ss_.getSpaceInformation().setStateValidityCheckingResolution( \
            1.0 / space.getMaximumExtent())
        #      self.ss_.setPlanner(og.RRTConnect(self.ss_.getSpaceInformation()))

    def plan(self, start_row, start_col, goal_row, goal_col):
        if not self.ss_:
            return False
        start = ob.State(self.ss_.getStateSpace())
        start()[0] = start_row
        start()[1] = start_col
        goal = ob.State(self.ss_.getStateSpace())
        goal()[0] = goal_row
        goal()[1] = goal_col
        self.ss_.setStartAndGoalStates(start, goal)
        # generate a few solutions; all will be added to the goal
        for _ in range(10):
            if self.ss_.getPlanner():
                self.ss_.getPlanner().clear()
            self.ss_.solve()
        ns = self.ss_.getProblemDefinition().getSolutionCount()
        print("Found %d solutions" % ns)
        if self.ss_.haveSolutionPath():
            self.ss_.simplifySolution()
            p = self.ss_.getSolutionPath()
            ps = og.PathSimplifier(self.ss_.getSpaceInformation())
            ps.simplifyMax(p)
            ps.smoothBSpline(p)
            return True
        return False

    def recordSolution(self):
        if not self.ss_ or not self.ss_.haveSolutionPath():
            return
        p = self.ss_.getSolutionPath()
        p.interpolate()
        for i in range(p.getStateCount()):
            w = min(self.maxWidth_, int(p.getState(i)[0]))
            h = min(self.maxHeight_, int(p.getState(i)[1]))
            c = self.ppm_.getPixel(h, w)
            c.red = 255
            c.green = 0
            c.blue = 0

    def save(self, filename):
        if not self.ss_:
            return
        self.ppm_.saveFile(filename)

    def isStateValid(self, state):
        w = min(int(state[0]), self.maxWidth_)
        h = min(int(state[1]), self.maxHeight_)

        c = self.ppm_.getPixel(h, w)
        return c.red > 127 and c.green > 127 and c.blue > 127


if __name__ == "__main__":
    fname = join(join(join(join(dirname(dirname(abspath(__file__))), \
        'tests'), 'resources'), 'ppm'), 'floor.ppm')
    env = Plane2DEnvironment(fname)

    if env.plan(0, 0, 777, 1265):
        env.recordSolution()
        env.save("result_demo.ppm")
