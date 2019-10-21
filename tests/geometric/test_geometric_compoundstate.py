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
import ompl.base as ob
import ompl.geometric as og
from ompl.util import setLogLevel, LogLevel

SOLUTION_TIME = 5.0

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
    x = int(state[0][0])
    y = int(state[1][0])
    return grid[x][y] == 0 # 0 means valid state

class mySpace1(ob.RealVectorStateSpace):
    def __init__(self):
        super(mySpace1, self).__init__(1)

    def distance(self, state1, state2):
        x1 = int(state1[0])
        x2 = int(state2[0])
        return fabs(x1-x2)

class mySetup(object):
    def __init__(self, env):
        self.space = ob.CompoundStateSpace()
        self.setup = og.SimpleSetup(self.space)
        bounds = ob.RealVectorBounds(1)
        bounds.setLow(0)
        bounds.setHigh(float(env.width) - 0.000000001)
        self.m1 = mySpace1()
        self.m1.setBounds(bounds)

        bounds.setHigh(float(env.height) - 0.000000001)
        self.m2 = mySpace1()
        self.m2.setBounds(bounds)

        self.space.addSubspace(self.m1, 1.0)
        self.space.addSubspace(self.m2, 1.0)

        isValidFn = ob.StateValidityCheckerFn(partial(isValid, env.grid))
        self.setup.setStateValidityChecker(isValidFn)

        state = ob.CompoundState(self.space)
        state()[0][0] = env.start[0]
        state()[1][0] = env.start[1]
        self.start = ob.State(state)

        gstate = ob.CompoundState(self.space)
        gstate()[0][0] = env.goal[0]
        gstate()[1][0] = env.goal[1]
        self.goal = ob.State(gstate)

        self.setup.setStartAndGoalStates(self.start, self.goal)


def testPlanner(env, time, pathLength, show=False):
    result = True
    setup = mySetup(env)
    startTime = perf_counter()
    if setup.setup.solve(SOLUTION_TIME):
        elapsed = perf_counter() - startTime
        time = time + elapsed
        if show:
            print('Found solution in %f seconds!' % elapsed)

        startTime = perf_counter()
        setup.setup.simplifySolution()
        elapsed = perf_counter() - startTime
        time = time + elapsed
        if show:
            print('Simplified solution in %f seconds!' % elapsed)

        path = setup.setup.getSolutionPath()
        path.interpolate(100)
        pathLength = pathLength + path.length()
        if show:
            print(env, '\n')
            temp = copy.deepcopy(env)
            for i in range(len(path.states)):
                x = int(path.states[i][0][0])
                y = int(path.states[i][1][0])
                if temp.grid[x][y] in [0, 2]:
                    temp.grid[x][y] = 2
                else:
                    temp.grid[x][y] = 3
            print(temp, '\n')
    else:
        result = False

    return (result, time, pathLength)


class PlanTest(unittest.TestCase):
    def setUp(self):
        self.env = Environment(dirname(abspath(__file__))+'/../../tests/resources/env1.txt')
        if self.env.width * self.env.height == 0:
            self.fail('The environment has a 0 dimension. Cannot continue')
        self.verbose = True

    def testRunPlanner(self):
        time = 0.0
        length = 0.0
        good = 0
        N = 25

        for _ in range(N):
            (result, time, length) = testPlanner(self.env, time, length, False)
            if result:
                good = good + 1

        success = 100.0 * float(good) / float(N)
        avgruntime = time / float(N)
        avglength = length / float(N)

        if self.verbose:
            print('    Success rate: %f%%' % success)
            print('    Average runtime: %f' % avgruntime)
            print('    Average path length: %f' % avglength)

        self.assertTrue(success >= 99.0)
        self.assertTrue(avgruntime < 2.5)
        self.assertTrue(avglength < 70.0)


def suite():
    suites = (unittest.makeSuite(PlanTest))
    return unittest.TestSuite(suites)

if __name__ == '__main__':
    setLogLevel(LogLevel.LOG_ERROR)
    unittest.main()
