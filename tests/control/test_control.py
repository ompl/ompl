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

import sys
from os.path import abspath, dirname, join
sys.path.insert(0, join(dirname(dirname(dirname(abspath(__file__)))), 'py-bindings'))
from functools import partial
from time import perf_counter
from math import fabs
import unittest
import copy
import ompl.util as ou
import ompl.base as ob
import ompl.control as oc
from ompl.util import setLogLevel, LogLevel

SOLUTION_TIME = 10.0
MAX_VELOCITY = 3.0

class Environment(object):
    def __init__(self, fname):
        fp = open(fname, 'r')
        lines = fp.readlines()
        fp.close()
        self.width, self.height = [int(i) for i in lines[0].split(' ')[1:3]]
        self.grid = []
        self.start = [int(i) for i in lines[1].split(' ')[1:3]]
        self.goal = [int(i) for i in lines[2].split(' ')[1:3]]
        for i in range(self.width):
            self.grid.append(
                [int(j) for j in lines[4+i].split(' ')[0:self.height]])
        self.char_mapping = ['__', '##', 'oo', 'XX']

    def __str__(self):
        result = ''
        for line in self.grid:
            result = result + ''.join([self.char_mapping[c] for c in line]) + '\n'
        return result

def isValid(grid, state):
    # planning is done in a continuous space, but our collision space
    # representation is discrete
    x = int(state[0])
    y = int(state[1])
    if x < 0 or y < 0 or x >= len(grid) or y >= len(grid[0]):
        return False
    return grid[x][y] == 0 # 0 means valid state

class MyStateSpace(ob.RealVectorStateSpace):
    def __init__(self):
        super(MyStateSpace, self).__init__(4)

    def distance(self, state1, state2):
        x1 = int(state1[0])
        y1 = int(state1[1])
        x2 = int(state2[0])
        y2 = int(state2[1])
        return fabs(x1-x2) + fabs(y1-y2)

class MyProjectionEvaluator(ob.ProjectionEvaluator):
    def __init__(self, space, cellSizes):
        super(MyProjectionEvaluator, self).__init__(space)
        self.setCellSizes(cellSizes)

    def getDimension(self):
        return 2

    def project(self, state, projection):
        projection[0] = state[0]
        projection[1] = state[1]

class MyStatePropagator(oc.StatePropagator):
    def propagate(self, state, control, duration, result):
        result[0] = state[0] + duration*control[0]
        result[1] = state[1] + duration*control[1]
        result[2] = control[0]
        result[3] = control[1]

class TestPlanner(object):

    def execute(self, env, time, pathLength, show=False):
        result = True

        sSpace = MyStateSpace()
        sbounds = ob.RealVectorBounds(4)
        # dimension 0 (x) spans between [0, width)
        # dimension 1 (y) spans between [0, height)
        # since sampling is continuous and we round down, we allow values until
        # just under the max limit
        # the resolution is 1.0 since we check cells only
        sbounds.low = ou.vectorDouble()
        sbounds.low.extend([0.0, 0.0, -MAX_VELOCITY, -MAX_VELOCITY])
        sbounds.high = ou.vectorDouble()
        sbounds.high.extend([float(env.width) - 0.000000001, \
            float(env.height) - 0.000000001, \
            MAX_VELOCITY, MAX_VELOCITY])
        sSpace.setBounds(sbounds)

        cSpace = oc.RealVectorControlSpace(sSpace, 2)
        cbounds = ob.RealVectorBounds(2)
        cbounds.low[0] = -MAX_VELOCITY
        cbounds.high[0] = MAX_VELOCITY
        cbounds.low[1] = -MAX_VELOCITY
        cbounds.high[1] = MAX_VELOCITY
        cSpace.setBounds(cbounds)

        ss = oc.SimpleSetup(cSpace)
        isValidFn = ob.StateValidityCheckerFn(partial(isValid, env.grid))
        ss.setStateValidityChecker(isValidFn)
        propagator = MyStatePropagator(ss.getSpaceInformation())
        ss.setStatePropagator(propagator)

        planner = self.newplanner(ss.getSpaceInformation())
        ss.setPlanner(planner)

        # the initial state
        start = ob.State(sSpace)
        start()[0] = env.start[0]
        start()[1] = env.start[1]
        start()[2] = 0.0
        start()[3] = 0.0

        goal = ob.State(sSpace)
        goal()[0] = env.goal[0]
        goal()[1] = env.goal[1]
        goal()[2] = 0.0
        goal()[3] = 0.0

        ss.setStartAndGoalStates(start, goal, 0.05)

        startTime = perf_counter()
        if ss.solve(SOLUTION_TIME):
            elapsed = perf_counter() - startTime
            time = time + elapsed
            if show:
                print('Found solution in %f seconds!' % elapsed)

            path = ss.getSolutionPath()
            path.interpolate()
            if not path.check():
                return (False, time, pathLength)
            pathLength = pathLength + path.length()

            if show:
                print(env, '\n')
                temp = copy.deepcopy(env)
                for i in range(len(path.states)):
                    x = int(path.states[i][0])
                    y = int(path.states[i][1])
                    if temp.grid[x][y] in [0, 2]:
                        temp.grid[x][y] = 2
                    else:
                        temp.grid[x][y] = 3
                print(temp, '\n')
        else:
            result = False

        return (result, time, pathLength)

    def newplanner(self, si):
        raise NotImplementedError('pure virtual method')

class RRTTest(TestPlanner):
    def newplanner(self, si):
        planner = oc.RRT(si)
        return planner

class ESTTest(TestPlanner):
    def newplanner(self, si):
        planner = oc.EST(si)
        cdim = ou.vectorDouble()
        cdim.extend([1, 1])
        ope = MyProjectionEvaluator(si.getStateSpace(), cdim)
        planner.setProjectionEvaluator(ope)
        return planner

class SyclopDecomposition(oc.GridDecomposition):
    def __init__(self, length, bounds):
        super(SyclopDecomposition, self).__init__(length, 2, bounds)

    def project(self, state, coord):
        coord[0] = state[0]
        coord[1] = state[1]

    def sampleFullState(self, sampler, coord, s):
        sampler.sampleUniform(s)
        s[0] = coord[0]
        s[1] = coord[1]

class SyclopRRTTest(TestPlanner):
    def newplanner(self, si):
        spacebounds = si.getStateSpace().getBounds()

        bounds = ob.RealVectorBounds(2)
        bounds.setLow(0, spacebounds.low[0])
        bounds.setLow(1, spacebounds.low[1])
        bounds.setHigh(0, spacebounds.high[0])
        bounds.setHigh(1, spacebounds.high[1])

        # Create a 10x10 grid decomposition for Syclop
        decomp = SyclopDecomposition(10, bounds)
        planner = oc.SyclopRRT(si, decomp)
        # Set syclop parameters conducive to a tiny workspace
        planner.setNumFreeVolumeSamples(1000)
        planner.setNumRegionExpansions(10)
        planner.setNumTreeExpansions(5)
        return planner

class SyclopESTTest(TestPlanner):
    def newplanner(self, si):
        spacebounds = si.getStateSpace().getBounds()

        bounds = ob.RealVectorBounds(2)
        bounds.setLow(0, spacebounds.low[0])
        bounds.setLow(1, spacebounds.low[1])
        bounds.setHigh(0, spacebounds.high[0])
        bounds.setHigh(1, spacebounds.high[1])

        # Create a 10x10 grid decomposition for Syclop
        decomp = SyclopDecomposition(10, bounds)
        planner = oc.SyclopEST(si, decomp)
        # Set syclop parameters conducive to a tiny workspace
        planner.setNumFreeVolumeSamples(1000)
        planner.setNumRegionExpansions(10)
        planner.setNumTreeExpansions(5)
        return planner

class KPIECE1Test(TestPlanner):
    def newplanner(self, si):
        planner = oc.KPIECE1(si)
        cdim = ou.vectorDouble()
        cdim.extend([1, 1])
        ope = MyProjectionEvaluator(si.getStateSpace(), cdim)
        planner.setProjectionEvaluator(ope)
        return planner

class PlanTest(unittest.TestCase):
    def setUp(self):
        self.env = Environment(dirname(abspath(__file__))+'/../../tests/resources/env1.txt')
        if self.env.width * self.env.height == 0:
            self.fail('The environment has a 0 dimension. Cannot continue')
        self.verbose = True

    def runPlanTest(self, planner):
        time = 0.0
        length = 0.0
        good = 0
        N = 25

        for _ in range(N):
            (result, time, length) = planner.execute(self.env, time, length, False)
            if result:
                good = good + 1

        success = 100.0 * float(good) / float(N)
        avgruntime = time / float(N)
        avglength = length / float(N)

        if self.verbose:
            print('    Success rate: %f%%' % success)
            print('    Average runtime: %f' % avgruntime)
            print('    Average path length: %f' % avglength)

        return (success, avgruntime, avglength)

    def testControl_RRT(self):
        planner = RRTTest()
        (success, avgruntime, avglength) = self.runPlanTest(planner)
        self.assertTrue(success >= 99.0)
        self.assertTrue(avgruntime < 5)
        self.assertTrue(avglength < 100.0)

    def testControl_EST(self):
        planner = ESTTest()
        (success, avgruntime, avglength) = self.runPlanTest(planner)
        self.assertTrue(success >= 99.0)
        self.assertTrue(avgruntime < 5)
        self.assertTrue(avglength < 100.0)

    def testControl_KPIECE1(self):
        planner = KPIECE1Test()
        (success, avgruntime, avglength) = self.runPlanTest(planner)
        self.assertTrue(success >= 99.0)
        self.assertTrue(avgruntime < 2.5)
        self.assertTrue(avglength < 100.0)

    def testControl_SyclopRRT(self):
        planner = SyclopRRTTest()
        (success, avgruntime, avglength) = self.runPlanTest(planner)
        self.assertTrue(success >= 99.0)
        self.assertTrue(avgruntime < 2.5)
        self.assertTrue(avglength < 100.0)

    def testControl_SyclopEST(self):
        planner = SyclopESTTest()
        (success, avgruntime, avglength) = self.runPlanTest(planner)
        self.assertTrue(success >= 99.0)
        self.assertTrue(avgruntime < 2.5)
        self.assertTrue(avglength < 100.0)


def suite():
    suites = (unittest.makeSuite(PlanTest))
    return unittest.TestSuite(suites)

if __name__ == '__main__':
    setLogLevel(LogLevel.LOG_ERROR)
    unittest.main()
