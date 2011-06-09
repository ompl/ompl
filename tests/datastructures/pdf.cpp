/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Matt Maly */

#include <gtest/gtest.h>
#include "ompl/datastructures/PDF.h"

TEST(PDF, Simple)
{
    typedef ompl::PDF<int>::Element Element;
    ompl::PDF<int> p;
    EXPECT_TRUE(p.empty());
    Element& e = p.add(0, 1.0);
    EXPECT_EQ(0, p.sample(0.5));
    EXPECT_EQ(1, p.size());
    EXPECT_FALSE(p.empty());
    p.add(1, 0.0);
    p.add(2, 0.0);
    p.add(3, 0.0);
    EXPECT_EQ(0, p.sample(0.5));
    EXPECT_EQ(4, p.size());

    p.clear();
    Element& e25 = p.add(0, 25);
    // 25
    EXPECT_EQ(0, p.sample(1.0));

    Element& e50 = p.add(1, 50);
    // 25 50
    EXPECT_EQ(0, p.sample(0.3));
    EXPECT_EQ(1, p.sample(0.5));

    Element& e15 = p.add(2, 15);
    // 25 50 15
    EXPECT_EQ(0, p.sample(0.25));
    EXPECT_EQ(1, p.sample(0.5));
    EXPECT_EQ(2, p.sample(0.85));

    Element& e10 = p.add(3, 10);
    // 25 50 15 10
    EXPECT_EQ(0, p.sample(0.1));
    EXPECT_EQ(1, p.sample(0.7));
    EXPECT_EQ(2, p.sample(0.8));
    EXPECT_EQ(3, p.sample(0.95));

    Element& e6 = p.add(4, 6);
    Element& e30 = p.add(5, 30);
    Element& e1 = p.add(6, 1);
    // 25 50 15 10 6 30 1

    p.remove(e1);
    // 25 50 15 10 6 30
    EXPECT_EQ(5, p.sample(0.95));
    EXPECT_EQ(0, p.sample(0.05));

    p.remove(e6);
    // 25 50 15 10 30
    EXPECT_EQ(5, p.size());
    EXPECT_EQ(5, p.sample(0.8));
    EXPECT_EQ(1, p.sample(0.2));

    p.remove(e25);
    // 30 50 15 10
    EXPECT_EQ(5, p.sample(0.25));
    EXPECT_EQ(1, p.sample(0.4));
    EXPECT_EQ(2, p.sample(0.85));
    EXPECT_EQ(3, p.sample(0.95));

    p.remove(e50);
    // 30 10 15
    EXPECT_EQ(2, p.sample(0.75));

    p.remove(e10);
    p.remove(e30);
    // 15
    EXPECT_EQ(2, p.sample(1.0));

    p.remove(e15);
    EXPECT_EQ(0, p.size());
    EXPECT_TRUE(p.empty());

    p.clear();
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
