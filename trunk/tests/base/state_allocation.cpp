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
#include "ompl/base/manifolds/SE3StateManifold.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Time.h"

using namespace ompl;

bool isValid(const base::State *)
{
    return true;
}

TEST(Allocation, Simple)
{
    base::StateManifoldPtr m(new base::SE3StateManifold());
    base::SpaceInformation si(m);
    si.setStateValidityChecker(boost::bind(&isValid, _1));
    si.setup();
    
    const unsigned int N = 100000;
    const unsigned int M = 100;
    std::vector<base::State*> states(N, NULL);

    time::point start = time::now();
    for (unsigned int j = 0 ; j < M ; ++j)
    {	
	for (unsigned int i = 0 ; i < N ; ++i)
	    states[i] = si.allocState();
	
	for (unsigned int i = 0 ; i < N ; ++i)
	    si.freeState(states[i]);
    }
    double d = time::seconds(time::now() - start);
    std::cout << (double)N * (double)M / d << " state allocations then frees per second" << std::endl;
    

    start = time::now();
    for (unsigned int j = 0 ; j < M ; ++j)
    {	
	for (unsigned int i = 0 ; i < N ; ++i)
	{
	    base::State *s = si.allocState();
	    si.freeState(s);
	}
    }    
    d = time::seconds(time::now() - start);
    std::cout << (double)N * (double)M / d << " mixed state allocations & frees per second" << std::endl;


    start = time::now();
    for (unsigned int j = 0 ; j < M ; ++j)
    {	
	for (unsigned int i = 0 ; i < N ; ++i)
	{
	    base::State *s = si.allocState();
	    si.freeState(s);
	    states[i] = si.allocState();
	}
	for (unsigned int i = 0 ; i < N ; ++i)
	    si.freeState(states[i]);
    }
    d = time::seconds(time::now() - start);
    std::cout << (double)N * (double)M / d << " allocations per second" << std::endl;    
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

