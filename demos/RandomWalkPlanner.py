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
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), "py-bindings"))
    from ompl import base as ob
    from ompl import geometric as og

## @cond IGNORE

class RandomWalkPlanner(ob.Planner):
    def __init__(self, si):
        super(RandomWalkPlanner, self).__init__(si, "RandomWalkPlanner")
        self.states_ = []
        self.sampler_ = si.allocStateSampler()

    def solve(self, ptc):
        pdef = self.getProblemDefinition()
        goal = pdef.getGoal()
        si = self.getSpaceInformation()
        pi = self.getPlannerInputStates()
        st = pi.nextStart()
        while st:
            self.states_.append(st)
            st = pi.nextStart()
        solution = None
        approxsol = 0
        approxdif = 1e6
        while not ptc():
            rstate = si.allocState()
            # pick a random state in the state space
            self.sampler_.sampleUniform(rstate)
            # check motion
            if si.checkMotion(self.states_[-1], rstate):
                self.states_.append(rstate)
                sat = goal.isSatisfied(rstate)
                dist = goal.distanceGoal(rstate)
                if sat:
                    approxdif = dist
                    solution = len(self.states_)
                    break
                if dist < approxdif:
                    approxdif = dist
                    approxsol = len(self.states_)
        solved = False
        approximate = False
        if not solution:
            solution = approxsol
            approximate = True
        if solution:
            path = og.PathGeometric(si)
            for s in self.states_[:solution]:
                path.append(s)
            pdef.addSolutionPath(path)
            solved = True
        return ob.PlannerStatus(solved, approximate)

    def clear(self):
        super(RandomWalkPlanner, self).clear()
        self.states_ = []

## @endcond

def isStateValid(state):
    x = state[0]
    y = state[1]
    z = state[2]
    return x*x + y*y + z*z > .8

def plan():
    # create an R^3 state space
    space = ob.RealVectorStateSpace(3)
    # set lower and upper bounds
    bounds = ob.RealVectorBounds(3)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)
    # create a simple setup object
    ss = og.SimpleSetup(space)
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
    start = ob.State(space)
    start()[0] = -1.
    start()[1] = -1.
    start()[2] = -1.
    goal = ob.State(space)
    goal()[0] = 1.
    goal()[1] = 1.
    goal()[2] = 1.
    ss.setStartAndGoalStates(start, goal, .05)
    # set the planner
    planner = RandomWalkPlanner(ss.getSpaceInformation())
    ss.setPlanner(planner)

    result = ss.solve(10.0)
    if result:
        if result.getStatus() == ob.PlannerStatus.APPROXIMATE_SOLUTION:
            print("Solution is approximate")
        # try to shorten the path
        ss.simplifySolution()
        # print the simplified path
        print(ss.getSolutionPath())


if __name__ == "__main__":
    plan()
