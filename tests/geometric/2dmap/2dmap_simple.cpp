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
#include <boost/bind.hpp>
#include <libgen.h>
#include <iostream>

#include "ompl/base/GoalState.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/geometric/SimpleSetup.h"

#include "../../resources/config.h"
#include "../../resources/environment2D.h"

using namespace ompl;

static const double SOLUTION_TIME = 2.0;

bool isValid(const std::vector< std::vector<int> > *grid, const base::State *state)
{
    const base::CompoundState *cstate = state->as<base::CompoundState>();

    /* planning is done in a continuous space, but our collision space representation is discrete */
    int x = (int)(cstate->as<base::RealVectorStateSpace::StateType>(0)->values[0]);
    int y = (int)(cstate->as<base::RealVectorStateSpace::StateType>(1)->values[0]);
    return (*grid)[x][y] == 0; // 0 means valid state
}

class mySpace1 : public base::RealVectorStateSpace
{
public:

    mySpace1() : base::RealVectorStateSpace(1)
    {
    }

    virtual double distance(const base::State *state1, const base::State *state2) const
    {
        int x1 = (int)(state1->as<base::RealVectorStateSpace::StateType>()->values[0]);
        int x2 = (int)(state2->as<base::RealVectorStateSpace::StateType>()->values[0]);

        return abs(x1 - x2);
    }
};

class mySetup
{
public:

    mySetup(Environment2D &env) : setup(base::StateSpacePtr(new base::CompoundStateSpace()))
    {
        base::RealVectorBounds bounds(1);
        bounds.low[0] = 0.0;
        bounds.high[0] = (double)env.width - 0.000000001;
        mySpace1 *m1 = new mySpace1();
        m1->setBounds(bounds);

        bounds.high[0] = (double)env.height - 0.000000001;
        mySpace1 *m2 = new mySpace1();
        m2->setBounds(bounds);

        setup.getStateSpace()->as<base::CompoundStateSpace>()->addSubSpace(base::StateSpacePtr(m1), 1.0);
        setup.getStateSpace()->as<base::CompoundStateSpace>()->addSubSpace(base::StateSpacePtr(m2), 1.0);

        setup.setStateValidityChecker(boost::bind(&isValid, &env.grid, _1));

        base::ScopedState<base::CompoundStateSpace> state(setup.getSpaceInformation());
        state->as<base::RealVectorStateSpace::StateType>(0)->values[0] = env.start.first;
        state->as<base::RealVectorStateSpace::StateType>(1)->values[0] = env.start.second;

        base::ScopedState<base::CompoundStateSpace> gstate(setup.getSpaceInformation());
        gstate->as<base::RealVectorStateSpace::StateType>(0)->values[0] = env.goal.first;
        gstate->as<base::RealVectorStateSpace::StateType>(1)->values[0] = env.goal.second;

        setup.setStartAndGoalStates(state, gstate);
    }

    geometric::SimpleSetup* operator->(void)
    {
        return &setup;
    }

private:

    geometric::SimpleSetup setup;
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

        /* start counting time */
        ompl::time::point startTime = ompl::time::now();

        /* call the planner to solve the problem */
        if (setup->solve(SOLUTION_TIME))
        {
            ompl::time::duration elapsed = ompl::time::now() - startTime;
            if (time)
                *time += ompl::time::seconds(elapsed);
            if (show)
                printf("Found solution in %f seconds!\n", ompl::time::seconds(elapsed));

            geometric::PathGeometric &path = setup->getSolutionPath();

            /* make the solution more smooth */

            startTime = ompl::time::now();
            setup->getPathSimplifier()->reduceVertices(path);
            elapsed = ompl::time::now() - startTime;

            if (time)
                *time += ompl::time::seconds(elapsed);

            if (show)
                printf("Simplified solution in %f seconds!\n", ompl::time::seconds(elapsed));

            /* fill in values that were linearly interpolated */
            path.interpolate(path.states.size() * 2);

            if (pathLength)
                *pathLength += path.length();

            if (show)
            {
                printEnvironment(std::cout, env);
                std::cout << std::endl;
            }

            Environment2D temp = env;
            /* display the solution */
            for (unsigned int i = 0 ; i < path.states.size() ; ++i)
            {
                int x = (int)path.states[i]->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(0)->values[0];
                int y = (int)path.states[i]->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1)->values[0];
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

TEST_F(PlanTest, SimpleSetup)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;

    TestPlanner p;
    runPlanTest(&p, &success, &avgruntime, &avglength);

    EXPECT_TRUE(success >= 99.0);
    EXPECT_TRUE(avgruntime < 0.2);
    EXPECT_TRUE(avglength < 70.0);
}

TEST(ScopedStateTest, Simple)
{
    base::StateSpacePtr m(new base::RealVectorStateSpace(2));

    base::ScopedState<base::RealVectorStateSpace> s1(m);
    s1->values[0] = 1.0;
    s1->values[1] = 2.0;

    base::ScopedState<base::RealVectorStateSpace> s2 = s1;
    EXPECT_TRUE(s2->values[1] == s1->values[1]);

    base::ScopedState<> s3(m);
    s3 = s1;
    base::ScopedState<> s4 = s3;
    EXPECT_TRUE(s4 == s3);
    EXPECT_TRUE(s4 == s1);

    base::ScopedState<base::RealVectorStateSpace> s5 = s2;
    EXPECT_TRUE(s5 == s1);

    s1->values[1] = 4.0;

    EXPECT_TRUE(s5 != s1);

    base::ScopedState<base::RealVectorStateSpace> s6(s5);
    EXPECT_TRUE(s6 != s1);
    s1 = s5;
    s5 = s1;
    EXPECT_TRUE(s6 == s1);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
