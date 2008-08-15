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

#include "ompl/extension/samplingbased/kinematic/extension/rrt/RRT.h"
#include "ompl/extension/samplingbased/kinematic/extension/rrt/LazyRRT.h"
#include "environment2D.h"
#include <iostream>
using namespace ompl;


/** Declare a class used in validating states. Such a class definition is needed for any use
 * of a kinematic planner */
class myStateValidityChecker : public SpaceInformation::StateValidityChecker
{
public:

    virtual bool operator()(const SpaceInformation::State_t state)
    {
	const SpaceInformationKinematic::StateKinematic_t kstate = static_cast<const SpaceInformationKinematic::StateKinematic_t>(state);
	
	/* planning is done in a continuous space, but our collision space representation is discrete */
	int x = (int)(kstate->values[0]);
	int y = (int)(kstate->values[1]);
	return m_grid[x][y] == 0; // 0 means valid state
    }
    
    void setGrid(const std::vector< std::vector<int> > &grid)
    {
	m_grid = grid;
    }
    
protected:
    
    std::vector< std::vector<int> > m_grid;

};

/** Declare a class used in evaluating distance between states (Manhattan distance) */
class myStateDistanceEvaluator : public SpaceInformation::StateDistanceEvaluator
{
public:

    virtual double operator()(const SpaceInformation::State_t state1, const SpaceInformation::State_t state2)
    {
	const SpaceInformationKinematic::StateKinematic_t kstate1 = static_cast<const SpaceInformationKinematic::StateKinematic_t>(state1);
	const SpaceInformationKinematic::StateKinematic_t kstate2 = static_cast<const SpaceInformationKinematic::StateKinematic_t>(state2);
	
	/* planning is done in a continuous space, but our collision space representation is discrete */
	int x1 = (int)(kstate1->values[0]);
	int y1 = (int)(kstate1->values[1]);
	
	int x2 = (int)(kstate2->values[0]);
	int y2 = (int)(kstate2->values[1]);

	return abs(x1 - x2) + abs(y1 - y2);
    }
    
};

    
/** Space information */
class mySpaceInformation : public SpaceInformationKinematic
{
public:
    mySpaceInformation(int width, int height) : SpaceInformationKinematic()
    {
	// we have 2 dimensions : x, y
	m_stateDimension = 2;
	m_stateComponent.resize(2);

	// dimension 0 (x) spans between [0, width) 
	// dimension 1 (y) spans between [0, height) 
	// since sampling is continuous and we round down, we allow values until just under the max limit
	// the resolution is 1.0 since we check cells only

	m_stateComponent[0].minValue = 0.0;
	m_stateComponent[0].maxValue = (double)width - 0.000000001;
	m_stateComponent[0].resolution = 0.5;
	m_stateComponent[0].type = StateComponent::NORMAL;
	
	m_stateComponent[1].minValue = 0.0;
	m_stateComponent[1].maxValue = (double)height - 0.000000001;
	m_stateComponent[1].resolution = 0.5;
	m_stateComponent[1].type = StateComponent::NORMAL;
	
	smoother->setMaxSteps(50);
	smoother->setMaxEmptySteps(10);
    }
};

void usage(const char *progname)
{
    printf("\nUsage: %s <environment file>\n", progname);    
}

int main(int argc, char **argv)
{
    if (argc != 2)
	usage(argv[0]);
    else
    {
	/* load environment */
	Environment2D env;
	loadEnvironment(argv[1], env);
	
	if (env.width * env.height == 0)
	{
	    std::cerr << "The environment has a 0 dimension. Cannot continue" << std::endl;
	    return 1;
	}
	
	/* instantiate state validity checker */
	myStateValidityChecker *svc = new myStateValidityChecker();
	svc->setGrid(env.grid);

	/* instantiate state distance evaluator  */
	myStateDistanceEvaluator *sde = new myStateDistanceEvaluator();

	
	/* instantiate space information */
	mySpaceInformation *si = new mySpaceInformation(env.width, env.height);
	si->setStateValidityChecker(svc);
	si->setStateDistanceEvaluator(sde);
	si->setup();

	/* instantiate motion planner */
	LazyRRT_t planner = new LazyRRT(si);
	planner->setRange(10.0);
	
	/* set the initial state; the memory for this is automatically cleaned by SpaceInformation */
	SpaceInformationKinematic::StateKinematic_t state = new SpaceInformationKinematic::StateKinematic(2);
	state->values[0] = env.start.first;
	state->values[1] = env.start.second;
	si->addStartState(state);
	
	/* set the goal state; the memory for this is automatically cleaned by SpaceInformation */
	SpaceInformationKinematic::GoalStateKinematic_t goal = new SpaceInformationKinematic::GoalStateKinematic(si);
	goal->state = new SpaceInformationKinematic::StateKinematic(2);
	goal->state->values[0] = env.goal.first;
	goal->state->values[1] = env.goal.second;
	goal->threshold = 1e-1; // this is basically 0, but we want to account for numerical instabilities 
	si->setGoal(goal);
	
	/* start counting time */
	time_utils::Time startTime = time_utils::Time::now();	
	
	/* call the planner to solve the problem */
	if (planner->solve(10.0))
	{
	    time_utils::Duration elapsed = time_utils::Time::now() - startTime;
	    printf("Found solution in %f seconds!\n", elapsed.to_double());
	    
	    SpaceInformationKinematic::PathKinematic_t path = static_cast<SpaceInformationKinematic::PathKinematic_t>(goal->getSolutionPath());
	    
	    startTime = time_utils::Time::now();
	    
	    /* make the solution more smooth */
	    si->smoother->smoothVertices(path);

	    elapsed = time_utils::Time::now() - startTime;
	    printf("Smooth solution in %f seconds!\n", elapsed.to_double());

	    /* fill in values that were linearly interpolated */
	    si->interpolatePath(path);
	    
	    printEnvironment(std::cout, env);
	    std::cout << std::endl;	    

	    /* display the solution */	    
	    for (unsigned int i = 0 ; i < path->states.size() ; ++i)
	    {
		int x = (int)(path->states[i]->values[0]);
		int y = (int)(path->states[i]->values[1]);
		if (env.grid[x][y] == T_FREE || env.grid[x][y] == T_PATH)
		    env.grid[x][y] = T_PATH;
		else
		    env.grid[x][y] = T_ERROR;
	    }
	    
	    printEnvironment(std::cout, env);	    
	}
	
	
	delete planner;
	delete si;
	delete svc;
	delete sde;	
    }    
}
