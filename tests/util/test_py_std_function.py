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
from py_std_function import *

def myIntFun0(i, j):
    i.value = j
    return i

def myIntFun1(i, j):
    i2 = i # this copies a reference, not the contents, if type of i is T&
    i2.value = j
    return i2

def myIntFun2(i, j):
    i.value = j

class TestPyStdFunction(unittest.TestCase):
    def testIntClassFunObj(self):
        i = IntClass(0)
        j = intClassFun0_obj(i, 1)
        self.assertEqual((i.value, j.value), (0, 1))
        j = intClassFun1_obj(i, 1)
        self.assertEqual((i.value, j.value), (1, 1))
        j = intClassFun2_obj(i, 2)
        self.assertEqual((i.value, j.value), (1, 2))
        # k = 2
        # j = intClassFun3_obj(i, k) # can't pass ints by reference yet
        # self.assertEqual((i.value,j.value), (2,2))
        intClassFun4_obj(i, 3)
        self.assertEqual(i.value, 3)
    def testMyIntClassFun0(self):
        i = IntClass(0)
        f = IntClassFun0_t(myIntFun0)
        j = f(i, 1)
        self.assertEqual((i.value, j.value), (0, 1))
        f = IntClassFun1_t(myIntFun0)
        j = f(i, 1)
        self.assertEqual((i.value, j.value), (1, 1))
        f = IntClassFun2_t(myIntFun0)
        j = f(i, 2)
        # const-ness is ignored
        self.assertEqual((i.value, j.value), (2, 2))
    def testMyIntClassFun1(self):
        i = IntClass(0)
        f = IntClassFun0_t(myIntFun1)
        j = f(i, 1)
        self.assertEqual((i.value, j.value), (0, 1))
        f = IntClassFun1_t(myIntFun1)
        j = f(i, 1)
        self.assertEqual((i.value, j.value), (1, 1))
        f = IntClassFun2_t(myIntFun1)
        j = f(i, 2)
        self.assertEqual((i.value, j.value), (2, 2))
    def testMyIntClassFun2(self):
        i = IntClass(0)
        f = IntClassFun4_t(myIntFun2)
        f(i, 3)
        self.assertEqual(i.value, 3)


def suite():
    suites = (unittest.makeSuite(TestPyStdFunction, 'test'))
    return unittest.TestSuite(suites)

if __name__ == '__main__':
    unittest.main()
