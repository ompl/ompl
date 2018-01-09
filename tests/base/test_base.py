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

import unittest
from math import pi
import sys
from os.path import abspath, dirname, join
sys.path.insert(0, join(dirname(dirname(dirname(abspath(__file__)))), 'py-bindings'))
from ompl.base import *

def isValid(_):
    return True

class TestSO2(unittest.TestCase):
    def testSimple(self):
        m = SO2StateSpace()
        s1 = SO2State(m)
        s2 = SO2State(m)
        s3 = SO2State(m)
        s1().value = pi - 0.1
        s2().value = -pi + 0.1
        self.assertAlmostEqual(m.distance(s2(), s1()), 0.2, 3)
        self.assertAlmostEqual(m.distance(s1(), s2()), 0.2, 3)
        self.assertAlmostEqual(m.distance(s1(), s1()), 0.0, 3)

        s1().value = pi - 0.08
        m.interpolate(s1(), s2(), 0.5, s3())
        self.assertAlmostEqual(s3().value, -pi+0.01, 3)

        s1().value = pi - 0.1
        s2().value = 0.1
        self.assertAlmostEqual(m.distance(s2(), s1()), pi - 0.2, 3)

        m.interpolate(s1(), s2(), 0.5, s3())
        self.assertAlmostEqual(s3().value, pi / 2.0, 3)

class TestSO3(unittest.TestCase):
    def testSimple(self):
        m = SO3StateSpace()
        s1 = SO3State(m)
        s1.random()
        s2 = s1
        self.assertAlmostEqual(m.distance(s1(), s2()), 0.0, 3)
        s2.random()

        si = SpaceInformation(m)
        si.setStateValidityChecker(StateValidityCheckerFn(isValid))
        si.setup()

        states = vectorState()
        count = si.getMotionStates(s1(), s2(), states, 10, True, True)
        self.assertEqual(count, len(states))

        for state in states:
            nrm = m.norm(state)
            self.assertAlmostEqual(nrm, 1.0, 15)
            si.freeState(state)


def suite():
    suites = (
        unittest.makeSuite(TestSO2),
        unittest.makeSuite(TestSO3))
    return unittest.TestSuite(suites)

if __name__ == '__main__':
    unittest.main()
