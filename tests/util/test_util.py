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
import sys
from os.path import abspath, dirname, join
sys.path.insert(0, join(dirname(dirname(dirname(abspath(__file__)))), 'py-bindings'))
from ompl.util import *

class TestRNG(unittest.TestCase):
    def testDifferentSeeds(self):
        r = [RNG() for i in range(4)]
        same = 0
        eq = 0
        N = 100
        for _ in range(N):
            v = [r[j].uniformInt(0, 100) for j in range(4)]
            if v[0] == v[1] and v[1] == v[2] and v[2] == v[3] and v[3] == v[4]:
                eq = eq + 1
            for j in range(4):
                if v[j] == r[j].uniformInt(0, 100):
                    same = same+1
        self.assertFalse(eq > N / 2)
        self.assertTrue(same < 2 * N)

    def testValidRangeInts(self):
        r = RNG()
        N = 100
        V = 10000 * N
        c = [0 for i in range(N+1)]
        for _ in range(V):
            v = r.uniformInt(0, N)
            self.assertTrue(v >= 0)
            self.assertTrue(v <= N)
            c[v] = c[v]+1
        for j in c:
            self.assertTrue(j > float(V) / float(N) / 3.)

def suite():
    suites = (unittest.makeSuite(TestRNG, 'test'))
    return unittest.TestSuite(suites)

if __name__ == '__main__':
    unittest.main()
