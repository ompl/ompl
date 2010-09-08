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

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/manifolds/RealVectorStateManifold.h"
#include "ompl/base/GoalState.h"
#include "ompl/geometric/ik/GAIK.h"
#include "ompl/util/Time.h"

#include "../../resources/config.h"
#include "../../resources/environment2D.h"

using namespace ompl;


/** \brief Declare a class used in validating states. Such a class
    definition is needed for any use of a kinematic planner */
class myStateValidityChecker : public base::StateValidityChecker
{
public:

    myStateValidityChecker(base::SpaceInformation *si, const std::vector< std::vector<int> > &grid) :
	base::StateValidityChecker(si), grid_(grid)
    {
    }
    
    virtual bool isValid(const base::State *state) const
    {
	/* planning is done in a continuous space, but our collision space representation is discrete */
	int x = (int)(state->as<base::RealVectorStateManifold::StateType>()->values[0]);
	int y = (int)(state->as<base::RealVectorStateManifold::StateType>()->values[1]);
	return grid_[x][y] == 0; // 0 means valid state
    }
    
protected:
    
    std::vector< std::vector<int> > grid_;

};

class myManifold : public base::RealVectorStateManifold
{
public:
    
    myManifold() : base::RealVectorStateManifold(2)
    {
    }
    
    virtual double distance(const base::State *state1, const base::State *state2) const
    {
	/* planning is done in a continuous space, but our collision space representation is discrete */
	int x1 = (int)(state1->as<base::RealVectorStateManifold::StateType>()->values[0]);
	int y1 = (int)(state1->as<base::RealVectorStateManifold::StateType>()->values[1]);

	int x2 = (int)(state2->as<base::RealVectorStateManifold::StateType>()->values[0]);
	int y2 = (int)(state2->as<base::RealVectorStateManifold::StateType>()->values[1]);

	return abs(x1 - x2) + abs(y1 - y2);
    }
};
    
/** \brief Space information */
base::SpaceInformationPtr mySpaceInformation(Environment2D &env)
{
    base::RealVectorStateManifold *sMan = new myManifold();
    
    base::RealVectorBounds sbounds(2);
    
    // dimension 0 (x) spans between [0, width) 
    // dimension 1 (y) spans between [0, height) 
    // since sampling is continuous and we round down, we allow values until just under the max limit
    // the resolution is 1.0 since we check cells only
    
    sbounds.low[0] = 0.0;
    sbounds.high[0] = (double)env.width - 0.000000001;
    
    sbounds.low[1] = 0.0;
    sbounds.high[1] = (double)env.height - 0.000000001;

    sMan->setBounds(sbounds);
    
    base::StateManifoldPtr sManPtr(sMan);
    
    base::SpaceInformationPtr si(new base::SpaceInformation(sManPtr));
    si->setStateValidityCheckingResolution(0.016);
    
    si->setStateValidityChecker(base::StateValidityCheckerPtr(new myStateValidityChecker(si.get(), env.grid)));
    
    si->setup();
    
    return si;
}

TEST(GAIK, Simple)
{
    /* load environment */
    Environment2D env;
    
    boost::filesystem::path path(TEST_RESOURCES_DIR);
    path = path / "env1.txt";
    loadEnvironment(path.string().c_str(), env);
    printEnvironment(std::cout, env);
    
    if (env.width * env.height == 0)
    {
	std::cerr << "The environment has a 0 dimension. Cannot continue" << std::endl;
	FAIL();	    
    }
    
    /* instantiate space information */
    base::SpaceInformationPtr si = mySpaceInformation(env);
        
    /* set the goal state; the memory for this is automatically cleaned by SpaceInformation */
    base::GoalState goal(si);
    base::ScopedState<base::RealVectorStateManifold> gstate(si);
    gstate->values[0] = env.goal.first;
    gstate->values[1] = env.goal.second;
    goal.setState(gstate);
    goal.setThreshold(1e-3); // this is basically 0, but we want to account for numerical instabilities 
    
    geometric::GAIK gaik(si);
    gaik.setRange(5.0);
    base::ScopedState<base::RealVectorStateManifold> found(si);
    double time = 0.0;
    
    /* start counting time */
    const int N = 100;
    for (int i = 0 ; i < N ; ++i)
    {
	ompl::time::point startTime = ompl::time::now();	
	bool solved = gaik.solve(1.0, goal, found.get());
	ompl::time::duration elapsed = ompl::time::now() - startTime;
	time += ompl::time::seconds(elapsed);
	EXPECT_TRUE(solved);
	EXPECT_TRUE(si->distance(found.get(), gstate.get()) < 1e-3);
    }
    time = time / (double)N;
    EXPECT_TRUE(time < 0.01);    
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
