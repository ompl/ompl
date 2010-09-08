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

/* Author: Ioan Sucan */

#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <libgen.h>
#include <iostream>

#include "ompl/geometric/PathSimplifier.h"
#include "ompl/base/manifolds/RealVectorStateManifold.h"
#include "ompl/base/manifolds/RealVectorStateProjections.h"
#include "ompl/base/GoalState.h"

#include "ompl/geometric/planners/kpiece/LBKPIECE1.h"
#include "ompl/geometric/planners/kpiece/KPIECE1.h"
#include "ompl/geometric/planners/sbl/SBL.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/sbl/pSBL.h"
#include "ompl/geometric/planners/rrt/pRRT.h"
#include "ompl/geometric/planners/rrt/LazyRRT.h"
#include "ompl/geometric/planners/est/EST.h"
#include "ompl/geometric/planners/prm/PRM.h"

#include "../../resources/config.h"
#include "../../resources/environment2D.h"

using namespace ompl;

static const double SOLUTION_TIME = 1.0;

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

/** \brief A base class for testing planners */
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
	
	/* instantiate space information */
	base::SpaceInformationPtr si = mySpaceInformation(env);
	
	/* instantiate problem definition */
	base::ProblemDefinitionPtr pdef(new base::ProblemDefinition(si));
	
	/* instantiate motion planner */
	base::PlannerPtr planner = newPlanner(si);
	planner->setProblemDefinition(pdef);
	planner->setup();
	
	/* set the initial state; the memory for this is automatically cleaned by SpaceInformation */
	base::ScopedState<base::RealVectorStateManifold> state(si);
	state->values[0] = env.start.first;
	state->values[1] = env.start.second;
	pdef->addStartState(state);
	
	/* set the goal state; the memory for this is automatically cleaned by SpaceInformation */
	base::GoalState *goal = new base::GoalState(si);
	base::ScopedState<base::RealVectorStateManifold> gstate(si);
	gstate->values[0] = env.goal.first;
	gstate->values[1] = env.goal.second;
	goal->setState(gstate);
	goal->setThreshold(1e-3); // this is basically 0, but we want to account for numerical instabilities 
	pdef->setGoal(base::GoalPtr(goal));
	
	/* start counting time */
	ompl::time::point startTime = ompl::time::now();	
	
	/* call the planner to solve the problem */
	if (planner->solve(SOLUTION_TIME))
	{
	    ompl::time::duration elapsed = ompl::time::now() - startTime;
	    if (time)
		*time += ompl::time::seconds(elapsed);
	    if (show)
		printf("Found solution in %f seconds!\n", ompl::time::seconds(elapsed));
	    
	    geometric::PathGeometric *path = static_cast<geometric::PathGeometric*>(goal->getSolutionPath().get());
	    
	    
	    /* make the solution more smooth */
	    geometric::PathSimplifierPtr sm(new geometric::PathSimplifier(si));
	    sm->setMaxSteps(50);
	    sm->setMaxEmptySteps(10);

	    startTime = ompl::time::now();
	    sm->reduceVertices(*path);
	    elapsed = ompl::time::now() - startTime;

	    if (time)
		*time += ompl::time::seconds(elapsed);
	    
	    if (show)
		printf("Simplified solution in %f seconds!\n", ompl::time::seconds(elapsed));

	    /* fill in values that were linearly interpolated */
	    path->interpolate();
	    
	    if (pathLength)
		*pathLength += path->length();

	    if (show)
	    {
		printEnvironment(std::cout, env);
		std::cout << std::endl;	    
	    }
	    
	    Environment2D temp = env;
	    /* display the solution */	    
	    for (unsigned int i = 0 ; i < path->states.size() ; ++i)
	    {
		int x = (int)(path->states[i]->as<base::RealVectorStateManifold::StateType>()->values[0]);
		int y = (int)(path->states[i]->as<base::RealVectorStateManifold::StateType>()->values[1]);
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
	
	return result;
    }
    
protected:
    
    virtual base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) = 0;
    
};

class RRTTest : public TestPlanner 
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si)
    {
	geometric::RRT *rrt = new geometric::RRT(si);
	rrt->setRange(10.0);
	return base::PlannerPtr(rrt);
    }    
};

class RRTConnectTest : public TestPlanner 
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si)
    {
	geometric::RRTConnect *rrt = new geometric::RRTConnect(si);
	rrt->setRange(10.0);
	return base::PlannerPtr(rrt);
    }    
};

class pRRTTest : public TestPlanner 
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si)
    {
	geometric::pRRT *rrt = new geometric::pRRT(si);
	rrt->setRange(10.0);
	rrt->setThreadCount(4);
	return base::PlannerPtr(rrt);
    }    
};

class LazyRRTTest : public TestPlanner 
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si)
    {
	geometric::LazyRRT *rrt = new geometric::LazyRRT(si);
	rrt->setRange(10.0);
	return base::PlannerPtr(rrt);
    }    
};

class SBLTest : public TestPlanner 
{
    
protected:
    
    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si)
    {
	geometric::SBL *sbl = new geometric::SBL(si);
	sbl->setRange(10.0);
	
	std::vector<unsigned int> projection;
	projection.push_back(0);
	projection.push_back(1);
	
	std::vector<double> cdim;
	cdim.push_back(1);
	cdim.push_back(1);
	
	sbl->setProjectionEvaluator(base::ProjectionEvaluatorPtr(new base::RealVectorOrthogonalProjectionEvaluator(si->getStateManifold(), cdim, projection)));

	return base::PlannerPtr(sbl);
    }    
};


class pSBLTest : public TestPlanner 
{
    
protected:
    
    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si)
    {
	geometric::pSBL *sbl = new geometric::pSBL(si);
	sbl->setRange(10.0);
	sbl->setThreadCount(4);
	
	std::vector<unsigned int> projection;
	projection.push_back(0);
	projection.push_back(1);

	std::vector<double> cdim;
	cdim.push_back(1);
	cdim.push_back(1);
	
	sbl->setProjectionEvaluator(base::ProjectionEvaluatorPtr(new base::RealVectorOrthogonalProjectionEvaluator(si->getStateManifold(), cdim, projection)));

	return base::PlannerPtr(sbl);
    }
    
};

class KPIECE1Test : public TestPlanner 
{
protected:
    
    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si)
    {
	geometric::KPIECE1 *kpiece = new geometric::KPIECE1(si);
	kpiece->setRange(10.0);
	
	std::vector<unsigned int> projection;
	projection.push_back(0);
	projection.push_back(1);

	std::vector<double> cdim;
	cdim.push_back(1);
	cdim.push_back(1);

	kpiece->setProjectionEvaluator(base::ProjectionEvaluatorPtr(new base::RealVectorOrthogonalProjectionEvaluator(si->getStateManifold(), cdim, projection)));

	return base::PlannerPtr(kpiece);
    }
};

class LBKPIECE1Test : public TestPlanner 
{
protected:
    
    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si)
    {
	geometric::LBKPIECE1 *kpiece = new geometric::LBKPIECE1(si);
	kpiece->setRange(10.0);
	
	std::vector<unsigned int> projection;
	projection.push_back(0);
	projection.push_back(1);

	std::vector<double> cdim;
	cdim.push_back(1);
	cdim.push_back(1);
	
	kpiece->setProjectionEvaluator(base::ProjectionEvaluatorPtr(new base::RealVectorOrthogonalProjectionEvaluator(si->getStateManifold(), cdim, projection)));

	return base::PlannerPtr(kpiece);
    }
    
};

class ESTTest : public TestPlanner 
{    
protected:
    
    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si)
    {
	geometric::EST *est = new geometric::EST(si);
	est->setRange(10.0);

	std::vector<double> cdim;
	cdim.push_back(1);
	cdim.push_back(1);
	
	std::vector<unsigned int> projection;
	projection.push_back(0);
	projection.push_back(1);

	est->setProjectionEvaluator(base::ProjectionEvaluatorPtr(new base::RealVectorOrthogonalProjectionEvaluator(si->getStateManifold(), cdim, projection)));
	
	return base::PlannerPtr(est);
    }
    
};

class PRMTest : public TestPlanner 
{    
protected:
    
    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si)
    {
	geometric::PRM *prm = new geometric::PRM(si);
	return base::PlannerPtr(prm);
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

TEST_F(PlanTest, geometric_RRT)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;
    
    TestPlanner *p = new RRTTest();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    EXPECT_TRUE(success >= 99.0);
    EXPECT_TRUE(avgruntime < 0.01);
    EXPECT_TRUE(avglength < 100.0);
}

TEST_F(PlanTest, geometric_RRTConnect)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;
    
    TestPlanner *p = new RRTConnectTest();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    EXPECT_TRUE(success >= 99.0);
    EXPECT_TRUE(avgruntime < 0.01);
    EXPECT_TRUE(avglength < 100.0);
}

TEST_F(PlanTest, geometric_pRRT)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;
    
    TestPlanner *p = new pRRTTest();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    EXPECT_TRUE(success >= 99.0);
    EXPECT_TRUE(avgruntime < 0.02);
    EXPECT_TRUE(avglength < 100.0);
}

TEST_F(PlanTest, geometric_SBL)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;
    
    TestPlanner *p = new SBLTest();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    EXPECT_TRUE(success >= 99.0);
    EXPECT_TRUE(avgruntime < 0.1);
    EXPECT_TRUE(avglength < 100.0);
}


TEST_F(PlanTest, geometric_pSBL)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;
    
    TestPlanner *p = new pSBLTest();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    EXPECT_TRUE(success >= 99.0);
    EXPECT_TRUE(avgruntime < 0.2);
    EXPECT_TRUE(avglength < 100.0);
}


TEST_F(PlanTest, geometric_KPIECE1)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;
    
    TestPlanner *p = new KPIECE1Test();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    EXPECT_TRUE(success >= 99.0);
    EXPECT_TRUE(avgruntime < 0.1);
    EXPECT_TRUE(avglength < 100.0);
}

TEST_F(PlanTest, geometric_LBKPIECE1)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;
    
    TestPlanner *p = new LBKPIECE1Test();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    EXPECT_TRUE(success >= 99.0);
    EXPECT_TRUE(avgruntime < 0.1);
    EXPECT_TRUE(avglength < 100.0);
}

TEST_F(PlanTest, geometric_EST)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;
    
    TestPlanner *p = new ESTTest();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    EXPECT_TRUE(success >= 99.0);
    EXPECT_TRUE(avgruntime < 0.1);
    EXPECT_TRUE(avglength < 100.0);
}

TEST_F(PlanTest, geometric_LazyRRT)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;
    
    TestPlanner *p = new LazyRRTTest();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;
    
    EXPECT_TRUE(success >= 70.0);
    EXPECT_TRUE(avgruntime < 1.0);
    EXPECT_TRUE(avglength < 100.0);
}

TEST_F(PlanTest, geometric_PRM)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;
    
    TestPlanner *p = new PRMTest();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    EXPECT_TRUE(success >= 99.0);
    EXPECT_TRUE(avgruntime < 0.1);
    EXPECT_TRUE(avglength < 100.0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
