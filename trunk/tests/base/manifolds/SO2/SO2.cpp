/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2010, Rice University
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

/* Author: Ioan Sucan */

#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <libgen.h>
#include <iostream>

#include "ompl/base/ScopedState.h"
#include "ompl/base/manifolds/SO2StateManifold.h"

using namespace ompl;

TEST(SO2, Simple)
{
    base::StateManifoldPtr m(new base::SO2StateManifold());
    base::ScopedState<base::SO2StateManifold> s1(m);
    base::ScopedState<base::SO2StateManifold> s2(m);
    base::ScopedState<base::SO2StateManifold> s3(m);
    
    s1->value = M_PI - 0.1;
    s2->value = -M_PI + 0.1;
    EXPECT_NEAR(m->distance(s2.get(), s1.get()), 0.2, 1e-3);
    EXPECT_NEAR(m->distance(s1.get(), s2.get()), 0.2, 1e-3);
    EXPECT_NEAR(m->distance(s1.get(), s1.get()), 0.0, 1e-3);

    s1->value = M_PI - 0.08;
    m->interpolate(s1.get(), s2.get(), 0.5, s3.get());
    EXPECT_NEAR(s3->value, -M_PI + 0.01, 1e-3);

    s1->value = M_PI - 0.1;
    s2->value = 0.1;
    EXPECT_NEAR(m->distance(s2.get(), s1.get()), M_PI - 0.2, 1e-3);

    m->interpolate(s1.get(), s2.get(), 0.5, s3.get());
    EXPECT_NEAR(s3->value, M_PI / 2.0, 1e-3);

    s2 = s1;
    m->interpolate(s1.get(), s1.get(), 0.5, s1.get());
    EXPECT_EQ(s1, s2);
    
    m->interpolate(s1.get(), s2.get(), 0.5, s1.get());
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

