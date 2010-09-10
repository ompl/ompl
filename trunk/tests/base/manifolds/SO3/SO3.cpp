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
#include "ompl/base/manifolds/SO3StateManifold.h"
#include "ompl/base/SpaceInformation.h"

using namespace ompl;

bool isValid(const base::State *)
{
    return true;
}

TEST(SO3, Simple)
{
    base::StateManifoldPtr m(new base::SO3StateManifold());
    base::ScopedState<base::SO3StateManifold> s1(m);
    base::ScopedState<base::SO3StateManifold> s2(m);
    
    s1.random();
    s2 = s1;

    EXPECT_NEAR(m->distance(s1.get(), s2.get()), 0.0, 1e-3);
    
    s2.random();

    base::SpaceInformation si(m);
    si.setStateValidityChecker(boost::bind(&isValid, _1));
    si.setup();
    
    std::vector<base::State*> states;
    unsigned int count = si.getMotionStates(s1.get(), s2.get(), states, 0.1, true, true);
    EXPECT_TRUE(states.size() == count);
    
    for (unsigned int i = 0 ; i < states.size() ; ++i)
    {
	double nrm = m->as<base::SO3StateManifold>()->norm(states[i]->as<base::SO3StateManifold::StateType>());
	EXPECT_NEAR(nrm, 1.0, 1e-15);
	si.freeState(states[i]);
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

