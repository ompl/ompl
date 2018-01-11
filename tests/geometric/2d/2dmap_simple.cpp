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

#define BOOST_TEST_MODULE "2D Map Simple"
#include <boost/test/unit_test.hpp>

#include "2DmapSetup1.h"
#include <iostream>

using namespace ompl;

static const double SOLUTION_TIME = 2.0;

/** A base class for testing planners */
class TestPlanner
{
public:
    TestPlanner()
    {
        msg::setLogLevel(msg::LOG_ERROR);
    }

    virtual ~TestPlanner() = default;

    virtual bool execute(Environment2D &env, bool show = false, double *time = nullptr, double *pathLength = nullptr)
    {
        bool result = true;

        geometric::SimpleSetup2DMap1 setup(env);

        /* start counting time */
        ompl::time::point startTime = ompl::time::now();

        /* call the planner to solve the problem */
        if (setup.solve(SOLUTION_TIME))
        {
            ompl::time::duration elapsed = ompl::time::now() - startTime;
            if (time != nullptr)
                *time += ompl::time::seconds(elapsed);
            if (show)
                printf("Found solution in %f seconds!\n", ompl::time::seconds(elapsed));

            geometric::PathGeometric &path = setup.getSolutionPath();

            /* make the solution more smooth */

            startTime = ompl::time::now();
            setup.getPathSimplifier()->reduceVertices(path);
            elapsed = ompl::time::now() - startTime;

            if (time != nullptr)
                *time += ompl::time::seconds(elapsed);

            if (show)
                printf("Simplified solution in %f seconds!\n", ompl::time::seconds(elapsed));

            /* fill in values that were linearly interpolated */
            path.interpolate(path.getStateCount() * 2);

            if (pathLength != nullptr)
                *pathLength += path.length();

            if (show)
            {
                printEnvironment(std::cout, env);
                std::cout << std::endl;
            }

            Environment2D temp = env;
            /* display the solution */
            for (unsigned int i = 0 ; i < path.getStateCount() ; ++i)
            {
                auto x = (int)path.getState(i)->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(0)->values[0];
                auto y = (int)path.getState(i)->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1)->values[0];
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

class PlanTest
{
public:

    PlanTest()
    {
        verbose = true;
        boost::filesystem::path path(TEST_RESOURCES_DIR);
        path = path / "env1.txt";
        loadEnvironment(path.string().c_str(), env);

        if (env.width * env.height == 0)
        {
            BOOST_FAIL("The environment has a 0 dimension. Cannot continue");
        }
    }

    ~PlanTest() = default;

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

    Environment2D env;
    bool          verbose;
};

BOOST_FIXTURE_TEST_SUITE( MyPlanTestFixture, PlanTest )

BOOST_AUTO_TEST_CASE(Simple)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;

    TestPlanner p;
    runPlanTest(&p, &success, &avgruntime, &avglength);

    BOOST_CHECK(success >= 99.0);
    BOOST_CHECK(avgruntime < 0.2);
    BOOST_CHECK(avglength < 70.0);
}

BOOST_AUTO_TEST_SUITE_END()
