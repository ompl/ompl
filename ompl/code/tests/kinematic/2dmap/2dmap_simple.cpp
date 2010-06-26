/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/** \author Ioan Sucan */

#include <gtest/gtest.h>
#include <boost/filesystem.hpp>

#include "ompl/base/GoalState.h"
#include "ompl/kinematic/planners/rrt/RRTConnect.h"
#include "ompl/kinematic/SimpleSetup.h"

#include "../../resources/config.h"
#include "environment2D.h"
#include <iostream>
#include <libgen.h>

using namespace ompl;

static const double SOLUTION_TIME = 1.0;

/** Declare a class used in validating states. Such a class definition is needed for any use
 * of a kinematic planner */
class myStateValidityChecker : public base::StateValidityChecker
{
public:

    myStateValidityChecker(const base::SpaceInformation *si, const std::vector< std::vector<int> > &grid) :
	base::StateValidityChecker(si), m_grid(grid)
    {
    }
    
    virtual bool operator()(const base::State *state) const
    {
	/* planning is done in a continuous space, but our collision space representation is discrete */
	int x = (int)(state->values[0]);
	int y = (int)(state->values[1]);
	return m_grid[x][y] == 0; // 0 means valid state
    }
    
protected:
    
    std::vector< std::vector<int> > m_grid;

};

/** Declare a class used in evaluating distance between states (Manhattan distance) */
class myStateDistanceEvaluator : public base::StateDistanceEvaluator
{
public:

    myStateDistanceEvaluator(const base::SpaceInformation *si) : base::StateDistanceEvaluator(si)
    {
    }
    
    virtual double operator()(const base::State *state1, const base::State *state2) const
    {
	/* planning is done in a continuous space, but our collision space representation is discrete */
	int x1 = (int)(state1->values[0]);
	int y1 = (int)(state1->values[1]);
	
	int x2 = (int)(state2->values[0]);
	int y2 = (int)(state2->values[1]);

	return abs(x1 - x2) + abs(y1 - y2);
    }
    
};

class mySetup : public kinematic::SimpleSetup
{
public:
    
    mySetup(const Environment2D &env) : kinematic::SimpleSetup(), env_(env)
    {
    }
    
    virtual base::Goal* allocGoal(const base::SpaceInformation *si)
    {
	base::GoalState *goal = new base::GoalState(si);
	goal->state = new base::State(2);
	goal->state->values[0] = env_.goal.first;
	goal->state->values[1] = env_.goal.second;
	goal->threshold = 1e-3; // this is basically 0, but we want to account for numerical instabilities 
	return goal;
    }
    
    virtual base::Planner* allocPlanner(kinematic::SpaceInformationKinematic *si)
    {
	kinematic::RRTConnect *rrt = new kinematic::RRTConnect(si);
	rrt->setRange(0.95);
	return rrt;
    }
    
    virtual base::StateValidityChecker* allocStateValidityChecker(const base::SpaceInformation *si)
    {
	return new myStateValidityChecker(si, env_.grid);
    }
    
    virtual base::StateDistanceEvaluator* allocStateDistanceEvaluator(const base::SpaceInformation *si)
    {
	return new myStateDistanceEvaluator(si);
    }
    
    virtual void configureSpaceInformation(kinematic::SpaceInformationKinematic *si)
    {
	std::vector<base::StateComponent> stateComponent(2);

	// dimension 0 (x) spans between [0, width) 
	// dimension 1 (y) spans between [0, height) 
	// since sampling is continuous and we round down, we allow values until just under the max limit
	// the resolution is 1.0 since we check cells only

	stateComponent[0].minValue = 0.0;
	stateComponent[0].maxValue = (double)env_.width - 0.000000001;
	stateComponent[0].resolution = 0.5;
	stateComponent[0].type = base::StateComponent::LINEAR;
	
	stateComponent[1].minValue = 0.0;
	stateComponent[1].maxValue = (double)env_.height - 0.000000001;
	stateComponent[1].resolution = 0.5;
	stateComponent[1].type = base::StateComponent::LINEAR;
	
	si->setStateComponents(stateComponent);
    }
    
protected:
    
    Environment2D env_;
    
};

/** A base class for testing planners */
class TestPlanner
{
public:
    TestPlanner(void)
    {
    }
    
    virtual ~TestPlanner(void)
    {
    }
    
    virtual bool execute(Environment2D &env, bool show = false, double *time = NULL, double *pathLength = NULL)
    {	 
	bool result = true;
	
	mySetup setup(env);
	setup.configure();
	
	/* set the initial state; the memory for this is automatically cleaned by SpaceInformation */
	base::State *state = new base::State(2);
	state->values[0] = env.start.first;
	state->values[1] = env.start.second;
	setup.getProblemDefinition()->addStartState(state);
	
	/* start counting time */
	ompl::time::point startTime = ompl::time::now();	
	
	/* call the planner to solve the problem */
	if (setup.getPlanner()->solve(SOLUTION_TIME))
	{
	    ompl::time::duration elapsed = ompl::time::now() - startTime;
	    if (time)
		*time += ompl::time::seconds(elapsed);
	    if (show)
		printf("Found solution in %f seconds!\n", ompl::time::seconds(elapsed));
	    
	    kinematic::PathKinematic *path = static_cast<kinematic::PathKinematic*>(setup.getGoal()->getSolutionPath());
	    
	    /* make the solution more smooth */
	    setup.getPathSimplifier()->setMaxSteps(50);
	    setup.getPathSimplifier()->setMaxEmptySteps(10);

	    startTime = ompl::time::now();
	    setup.getPathSimplifier()->reduceVertices(path);
	    elapsed = ompl::time::now() - startTime;

	    if (time)
		*time += ompl::time::seconds(elapsed);
	    
	    if (show)
		printf("Simplified solution in %f seconds!\n", ompl::time::seconds(elapsed));

	    /* fill in values that were linearly interpolated */
	    setup.getSpaceInformation()->interpolatePath(path);
	    
	    if (pathLength)
		*pathLength += path->states.size();

	    if (show)
	    {
		printEnvironment(std::cout, env);
		std::cout << std::endl;	    
	    }
	    
	    Environment2D temp = env;
	    /* display the solution */	    
	    for (unsigned int i = 0 ; i < path->states.size() ; ++i)
	    {
		int x = (int)(path->states[i]->values[0]);
		int y = (int)(path->states[i]->values[1]);
		if (temp.grid[x][y] == T_FREE || temp.grid[x][y] == T_PATH)
		    temp.grid[x][y] = T_PATH;
		else
		{
		    temp.grid[x][y] = T_ERROR;
		    result = false;
		}		
	    }
	    
	    if (show)
		printEnvironment(std::cout, temp);
	}
	else
	    result = false;
	
	// free memory for start states
	setup.getProblemDefinition()->clearStartStates();
	
	return result;
    }    
};

class PlanTest : public testing::Test
{
public:
    
    void runPlanTest(TestPlanner *p, double *success, double *avgruntime, double *avglength)
    {    
	double time   = 0.0;
	double length = 0.0;
	int    good   = 0;
	int    N      = 100;

	for (int i = 0 ; i < N ; ++i)
	    if (p->execute(env, false, &time, &length))
		good++;
	
	*success    = 100.0 * (double)good / (double)N;
	*avgruntime = time / (double)N;
	*avglength  = length / (double)N;

	if (verbose)
	{
	    printf("    Success rate: %f%%\n", *success);
	    printf("    Average runtime: %f\n", *avgruntime);
	    printf("    Average path length: %f\n", *avglength);
	}
    }
    
protected:
    
    PlanTest(void) 
    {
	verbose = true;
    }
    
    void SetUp(void)
    {
	/* load environment */
	boost::filesystem::path path(TEST_RESOURCES_DIR);
	path = path / "env1.txt";
	loadEnvironment(path.string().c_str(), env);
	
	if (env.width * env.height == 0)
	{
	    std::cerr << "The environment has a 0 dimension. Cannot continue" << std::endl;
	    FAIL();	    
	}
    }
    
    void TearDown(void)
    {
    }

    Environment2D env;
    bool          verbose;
};

TEST_F(PlanTest, kinematic_RRTConnect)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;
    
    TestPlanner p;
    runPlanTest(&p, &success, &avgruntime, &avglength);
    
    EXPECT_TRUE(success >= 99.0);
    EXPECT_TRUE(avgruntime < 0.01);
    EXPECT_TRUE(avglength < 70.0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
