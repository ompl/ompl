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
import ompl.geometric as og
from ompl.util import setLogLevel, LogLevel

SOLUTION_TIME = 10.0

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
    return grid[x][y] == 0 # 0 means valid state

class mySpace(ob.RealVectorStateSpace):
    def __init__(self):
        super(mySpace, self).__init__(2)

    def distance(self, state1, state2):
        x1 = int(state1[0])
        y1 = int(state1[1])
        x2 = int(state2[0])
        y2 = int(state2[1])
        return fabs(x1-x2) + fabs(y1-y2)

class mySpaceInformation(ob.SpaceInformation):
    def __init__(self, env):
        self.sMan = mySpace()
        super(mySpaceInformation, self).__init__(self.sMan)
        sbounds = ob.RealVectorBounds(2)

        # dimension 0 (x) spans between [0, width)
        # dimension 1 (y) spans between [0, height)
        # since sampling is continuous and we round down, we allow values until
        # just under the max limit
        # the resolution is 1.0 since we check cells only
        sbounds.low[0] = 0.0
        sbounds.high[0] = float(env.width) - 0.000000001

        sbounds.low[1] = 0.0
        sbounds.high[1] = float(env.height) - 0.000000001

        self.sMan.setBounds(sbounds)
        self.setStateValidityCheckingResolution(0.5)
        isValidFn = ob.StateValidityCheckerFn(partial(isValid, env.grid))
        self.setStateValidityChecker(isValidFn)
        self.setup()

class TestPlanner(object):

    def execute(self, env, time, pathLength, show=False):
        result = True
        # instantiate space information
        si = mySpaceInformation(env)
        # instantiate problem definition
        pdef = ob.ProblemDefinition(si)
        # instantiate motion planner
        planner = self.newplanner(si)
        planner.setProblemDefinition(pdef)
        planner.setup()

        # the initial state
        state = ob.State(si)
        state()[0] = env.start[0]
        state()[1] = env.start[1]
        pdef.addStartState(state)

        goal = ob.GoalState(si)
        gstate = ob.State(si)
        gstate()[0] = env.goal[0]
        gstate()[1] = env.goal[1]
        goal.setState(gstate)
        goal.threshold = 1e-3
        pdef.setGoal(goal)

        startTime = perf_counter()
        if planner.solve(SOLUTION_TIME):
            elapsed = perf_counter() - startTime
            time = time + elapsed
            if show:
                print('Found solution in %f seconds!' % elapsed)

            path = pdef.getSolutionPath()
            sm = og.PathSimplifier(si)
            startTime = perf_counter()
            sm.reduceVertices(path)
            elapsed = perf_counter() - startTime
            time = time + elapsed
            if show:
                print('Simplified solution in %f seconds!' % elapsed)

            path.interpolate(100)
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
        planner = og.RRT(si)
        planner.setRange(10.0)
        return planner

class TRRTTest(TestPlanner):
    def newplanner(self, si):
        planner = og.TRRT(si)
        planner.setRange(10.0)
        return planner

class RRTConnectTest(TestPlanner):
    def newplanner(self, si):
        planner = og.RRTConnect(si)
        planner.setRange(10.0)
        return planner

class pRRTTest(TestPlanner):
    def newplanner(self, si):
        planner = og.pRRT(si)
        planner.setRange(10.0)
        planner.setThreadCount(4)
        return planner

class LazyRRTTest(TestPlanner):
    def newplanner(self, si):
        planner = og.LazyRRT(si)
        planner.setRange(10.0)
        return planner

class SBLTest(TestPlanner):
    def newplanner(self, si):
        planner = og.SBL(si)
        planner.setRange(10.0)
        projection = ou.vectorUint()
        projection.extend([0, 1])
        cdim = ou.vectorDouble()
        cdim.extend([1, 1])
        proj = ob.RealVectorOrthogonalProjectionEvaluator(si.getStateSpace(), cdim, projection)
        planner.setProjectionEvaluator(proj)
        return planner

class pSBLTest(TestPlanner):
    def newplanner(self, si):
        planner = og.pSBL(si)
        planner.setRange(10.0)
        planner.setThreadCount(4)
        projection = ou.vectorUint()
        projection.extend([0, 1])
        cdim = ou.vectorDouble()
        cdim.extend([1, 1])
        proj = ob.RealVectorOrthogonalProjectionEvaluator(si.getStateSpace(), cdim, projection)
        planner.setProjectionEvaluator(proj)
        return planner

class KPIECE1Test(TestPlanner):
    def newplanner(self, si):
        planner = og.KPIECE1(si)
        planner.setRange(10.0)
        projection = ou.vectorUint()
        projection.extend([0, 1])
        cdim = ou.vectorDouble()
        cdim.extend([1, 1])
        proj = ob.RealVectorOrthogonalProjectionEvaluator(si.getStateSpace(), cdim, projection)
        planner.setProjectionEvaluator(proj)
        return planner

class LBKPIECE1Test(TestPlanner):
    def newplanner(self, si):
        planner = og.LBKPIECE1(si)
        planner.setRange(10.0)
        projection = ou.vectorUint()
        projection.extend([0, 1])
        cdim = ou.vectorDouble()
        cdim.extend([1, 1])
        proj = ob.RealVectorOrthogonalProjectionEvaluator(si.getStateSpace(), cdim, projection)
        planner.setProjectionEvaluator(proj)
        return planner

class ESTTest(TestPlanner):
    def newplanner(self, si):
        planner = og.EST(si)
        planner.setRange(10.0)
        return planner

class BiESTTest(TestPlanner):
    def newplanner(self, si):
        planner = og.BiEST(si)
        planner.setRange(10.0)
        return planner

class ProjESTTest(TestPlanner):
    def newplanner(self, si):
        planner = og.ProjEST(si)
        planner.setRange(10.0)
        projection = ou.vectorUint()
        projection.extend([0, 1])
        cdim = ou.vectorDouble()
        cdim.extend([1, 1])
        proj = ob.RealVectorOrthogonalProjectionEvaluator(si.getStateSpace(), cdim, projection)
        planner.setProjectionEvaluator(proj)
        return planner

class PRMTest(TestPlanner):
    def newplanner(self, si):
        planner = og.PRM(si)
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
        N = 50

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

    def testGeometric_RRT(self):
        planner = RRTTest()
        (success, avgruntime, avglength) = self.runPlanTest(planner)
        self.assertTrue(success >= 99.0)
        self.assertTrue(avgruntime < 0.3)
        self.assertTrue(avglength < 100.0)

    def testGeometric_TRRT(self):
        planner = TRRTTest()
        (success, avgruntime, avglength) = self.runPlanTest(planner)
        self.assertTrue(success >= 99.0)
        self.assertTrue(avgruntime < 0.3)
        self.assertTrue(avglength < 100.0)

    def testGeometric_RRTConnect(self):
        planner = RRTConnectTest()
        (success, avgruntime, avglength) = self.runPlanTest(planner)
        self.assertTrue(success >= 99.0)
        self.assertTrue(avgruntime < 0.5)
        self.assertTrue(avglength < 100.0)

    # need to make bindings threadsafe
    # see http://wiki.python.org/moin/boost.python/HowTo#MultithreadingSupportformyfunction
    # def testGeometric_pRRT(self):
    #     planner = pRRTTest()
    #     (success, avgruntime, avglength) = self.runPlanTest(planner)
    #     self.assertTrue(success >= 99.0)
    #     self.assertTrue(avgruntime < 2.5)
    #     self.assertTrue(avglength < 100.0)

    def testGeometric_LazyRRT(self):
        planner = LazyRRTTest()
        (success, avgruntime, avglength) = self.runPlanTest(planner)
        self.assertTrue(success >= 60.0)
        self.assertTrue(avgruntime < 1)
        self.assertTrue(avglength < 100.0)

    def testGeometric_SBL(self):
        planner = SBLTest()
        (success, avgruntime, avglength) = self.runPlanTest(planner)
        self.assertTrue(success >= 99.0)
        self.assertTrue(avgruntime < 0.1)
        self.assertTrue(avglength < 100.0)

    # need to make bindings threadsafe
    # see http://wiki.python.org/moin/boost.python/HowTo#MultithreadingSupportformyfunction
    # def testGeometric_pSBL(self):
    #     planner = pSBLTest()
    #     (success, avgruntime, avglength) = self.runPlanTest(planner)
    #     self.assertTrue(success >= 99.0)
    #     self.assertTrue(avgruntime < 0.1)
    #     self.assertTrue(avglength < 100.0)

    def testGeometric_KPIECE1(self):
        planner = KPIECE1Test()
        (success, avgruntime, avglength) = self.runPlanTest(planner)
        self.assertTrue(success >= 99.0)
        self.assertTrue(avgruntime < 0.1)
        self.assertTrue(avglength < 100.0)

    def testGeometric_LBKPIECE1(self):
        planner = LBKPIECE1Test()
        (success, avgruntime, avglength) = self.runPlanTest(planner)
        self.assertTrue(success >= 99.0)
        self.assertTrue(avgruntime < 0.1)
        self.assertTrue(avglength < 100.0)

    def testGeometric_EST(self):
        planner = ESTTest()
        (success, avgruntime, avglength) = self.runPlanTest(planner)
        self.assertTrue(success >= 99.0)
        self.assertTrue(avgruntime < 0.1)
        self.assertTrue(avglength < 100.0)

    def testGeometric_PRM(self):
        planner = PRMTest()
        (success, avgruntime, avglength) = self.runPlanTest(planner)
        self.assertTrue(success >= 99.0)
        self.assertTrue(avgruntime < 2.0)
        self.assertTrue(avglength < 100.0)

def suite():
    suites = (unittest.makeSuite(PlanTest))
    return unittest.TestSuite(suites)

if __name__ == '__main__':
    setLogLevel(LogLevel.LOG_ERROR)
    unittest.main()
