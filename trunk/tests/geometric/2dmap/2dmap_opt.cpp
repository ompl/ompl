/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Rice University
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

#include "ompl/geometric/SimpleSetup.h"
#include "ompl/base/manifolds/RealVectorStateManifold.h"
#include "ompl/geometric/planners/rrt/OptRRT.h"
#include "ompl/geometric/planners/rrt/RRT.h"

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

base::PlannerPtr allocPlanner(const base::SpaceInformationPtr &si)
{
    //    geometric::RRT *rrt = new geometric::RRT(si);
    geometric::OptRRT *rrt = new geometric::OptRRT(si);
    rrt->setRange(8.0);
    rrt->setMaxBallRadius(5.0);
    rrt->setBallRadiusConstant(10.0);
    return base::PlannerPtr(rrt);
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
    
    geometric::SimpleSetup ss(base::StateManifoldPtr(new base::RealVectorStateManifold(2)));
    ss.setStateValidityChecker(base::StateValidityCheckerPtr(new myStateValidityChecker(ss.getSpaceInformation().get(), env.grid)));
    ss.setPlannerAllocator(boost::bind(&allocPlanner, _1));

    base::RealVectorBounds bounds(2);
    bounds.setLow(0.0);
    bounds.high[0] = (double)env.width;
    bounds.high[1] = (double)env.height;
    ss.getStateManifold()->as<base::RealVectorStateManifold>()->setBounds(bounds);
	
    base::ScopedState<base::RealVectorStateManifold> state(ss.getSpaceInformation());
    state->values[0] = env.start.first;
    state->values[1] = env.start.second;
    
    base::ScopedState<base::RealVectorStateManifold> gstate(ss.getSpaceInformation());
    gstate->values[0] = env.goal.first;
    gstate->values[1] = env.goal.second;
    
    ss.setStartAndGoalStates(state, gstate);
    ss.getGoal()->setMaximumPathLength(23.0);
    
    EXPECT_TRUE(ss.solve(1.0));
    std::cout << "Path length: " << ss.getSolutionPath().length() << std::endl;
    
    EXPECT_TRUE(ss.getSolutionPath().length() < ss.getGoal()->getMaximumPathLength());

    /*    
    base::PlannerData pd;
    ss.getPlanner()->getPlannerData(pd);
    pd.print();
    */
}

        
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
