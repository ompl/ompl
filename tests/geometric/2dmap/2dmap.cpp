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

#define BOOST_TEST_MODULE "GeometricPlanning"
#include <boost/test/unit_test.hpp>

#include "2DmapSetup.h"
#include <iostream>

#include "ompl/base/spaces/RealVectorStateProjections.h"

#include "ompl/geometric/planners/kpiece/LBKPIECE1.h"
#include "ompl/geometric/planners/kpiece/BKPIECE1.h"
#include "ompl/geometric/planners/kpiece/KPIECE1.h"
#include "ompl/geometric/planners/sbl/SBL.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/sbl/pSBL.h"
#include "ompl/geometric/planners/rrt/pRRT.h"
#include "ompl/geometric/planners/rrt/LazyRRT.h"
#include "ompl/geometric/planners/est/EST.h"
#include "ompl/geometric/planners/prm/PRM.h"

#include "../../BoostTestTeamCityReporter.h"
#include "../../base/PlannerTest.h"

using namespace ompl;

static const double SOLUTION_TIME = 1.0;

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
        base::SpaceInformationPtr si = geometric::spaceInformation2DMap(env);

        /* instantiate problem definition */
        base::ProblemDefinitionPtr pdef = geometric::problemDefinition2DMap(si, env);

        /* instantiate motion planner */
        base::PlannerPtr planner = newPlanner(si);
        planner->setProblemDefinition(pdef);
        planner->setup();

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

            geometric::PathGeometric *path = static_cast<geometric::PathGeometric*>(pdef->getSolutionPath().get());


            /* make the solution more smooth */
            geometric::PathSimplifierPtr sm(new geometric::PathSimplifier(si));

            startTime = ompl::time::now();
            sm->reduceVertices(*path);
            elapsed = ompl::time::now() - startTime;

            if (time)
                *time += ompl::time::seconds(elapsed);

            if (show)
                printf("Simplified solution in %f seconds!\n", ompl::time::seconds(elapsed));

            /* fill in values that were linearly interpolated */
            path->interpolate(path->getStateCount() * 2);

            if (pathLength)
                *pathLength += path->length();

            if (show)
            {
                printEnvironment(std::cout, env);
                std::cout << std::endl;
            }

            Environment2D temp = env;
            /* display the solution */
            for (unsigned int i = 0 ; i < path->getStateCount() ; ++i)
            {
                int x = (int)(path->getState(i)->as<base::RealVectorStateSpace::StateType>()->values[0]);
                int y = (int)(path->getState(i)->as<base::RealVectorStateSpace::StateType>()->values[1]);
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

        sbl->setProjectionEvaluator(base::ProjectionEvaluatorPtr(new base::RealVectorOrthogonalProjectionEvaluator(si->getStateSpace(), cdim, projection)));

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

        sbl->setProjectionEvaluator(base::ProjectionEvaluatorPtr(new base::RealVectorOrthogonalProjectionEvaluator(si->getStateSpace(), cdim, projection)));

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

        kpiece->setProjectionEvaluator(base::ProjectionEvaluatorPtr(new base::RealVectorOrthogonalProjectionEvaluator(si->getStateSpace(), cdim, projection)));

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

        kpiece->setProjectionEvaluator(base::ProjectionEvaluatorPtr(new base::RealVectorOrthogonalProjectionEvaluator(si->getStateSpace(), cdim, projection)));

        return base::PlannerPtr(kpiece);
    }

};

class BKPIECE1Test : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si)
    {
        geometric::BKPIECE1 *kpiece = new geometric::BKPIECE1(si);
        kpiece->setRange(10.0);

        std::vector<unsigned int> projection;
        projection.push_back(0);
        projection.push_back(1);

        std::vector<double> cdim;
        cdim.push_back(1);
        cdim.push_back(1);

        kpiece->setProjectionEvaluator(base::ProjectionEvaluatorPtr(new base::RealVectorOrthogonalProjectionEvaluator(si->getStateSpace(), cdim, projection)));

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

        est->setProjectionEvaluator(base::ProjectionEvaluatorPtr(new base::RealVectorOrthogonalProjectionEvaluator(si->getStateSpace(), cdim, projection)));

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

class PlanTest
{
public:

    void simpleTest(void)
    {
        geometric::SimpleSetup2DMap s(env);
        s.setup();
        base::PlannerTest pt(s.getPlanner());
        pt.test();
    }

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
        boost::filesystem::path path(TEST_RESOURCES_DIR);
        path = path / "env1.txt";
        loadEnvironment(path.string().c_str(), env);

        if (env.width * env.height == 0)
        {
            BOOST_FAIL( "The environment has a 0 dimension. Cannot continue" );
        }
    }

    Environment2D env;
    bool          verbose;
};

BOOST_FIXTURE_TEST_SUITE( MyPlanTestFixture, PlanTest )

BOOST_AUTO_TEST_CASE(geometric_RRT)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;

    simpleTest();

    TestPlanner *p = new RRTTest();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    BOOST_CHECK(success >= 99.0);
    BOOST_CHECK(avgruntime < 0.01);
    BOOST_CHECK(avglength < 100.0);
}

BOOST_AUTO_TEST_CASE(geometric_RRTConnect)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;

    simpleTest();

    TestPlanner *p = new RRTConnectTest();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    BOOST_CHECK(success >= 99.0);
    BOOST_CHECK(avgruntime < 0.01);
    BOOST_CHECK(avglength < 100.0);
}

BOOST_AUTO_TEST_CASE(geometric_pRRT)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;

    simpleTest();

    TestPlanner *p = new pRRTTest();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    BOOST_CHECK(success >= 99.0);
    BOOST_CHECK(avgruntime < 0.02);
    BOOST_CHECK(avglength < 100.0);
}

BOOST_AUTO_TEST_CASE(geometric_pSBL)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;

    simpleTest();

    TestPlanner *p = new pSBLTest();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    BOOST_CHECK(success >= 99.0);
    BOOST_CHECK(avgruntime < 0.2);
    BOOST_CHECK(avglength < 100.0);
}


BOOST_AUTO_TEST_CASE(geometric_KPIECE1)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;

    simpleTest();

    TestPlanner *p = new KPIECE1Test();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    BOOST_CHECK(success >= 99.0);
    BOOST_CHECK(avgruntime < 0.1);
    BOOST_CHECK(avglength < 100.0);
}

BOOST_AUTO_TEST_CASE(geometric_LBKPIECE1)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;

    simpleTest();

    TestPlanner *p = new LBKPIECE1Test();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    BOOST_CHECK(success >= 99.0);
    BOOST_CHECK(avgruntime < 0.1);
    BOOST_CHECK(avglength < 100.0);
}

BOOST_AUTO_TEST_CASE(geometric_BKPIECE1)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;

    simpleTest();

    TestPlanner *p = new BKPIECE1Test();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    BOOST_CHECK(success >= 99.0);
    BOOST_CHECK(avgruntime < 0.1);
    BOOST_CHECK(avglength < 100.0);
}

BOOST_AUTO_TEST_CASE(geometric_EST)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;

    simpleTest();

    TestPlanner *p = new ESTTest();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    BOOST_CHECK(success >= 99.0);
    BOOST_CHECK(avgruntime < 0.1);
    BOOST_CHECK(avglength < 100.0);
}

BOOST_AUTO_TEST_CASE(geometric_LazyRRT)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;

    simpleTest();

    TestPlanner *p = new LazyRRTTest();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    BOOST_CHECK(success >= 70.0);
    BOOST_CHECK(avgruntime < 1.0);
    BOOST_CHECK(avglength < 100.0);
}

BOOST_AUTO_TEST_CASE(geometric_PRM)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;

    simpleTest();

    TestPlanner *p = new PRMTest();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    BOOST_CHECK(success >= 99.0);
    BOOST_CHECK(avgruntime < 0.1);
    BOOST_CHECK(avglength < 100.0);
}

BOOST_AUTO_TEST_CASE(geometric_SBL)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;

    simpleTest();

    TestPlanner *p = new SBLTest();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    BOOST_CHECK(success >= 99.0);
    BOOST_CHECK(avgruntime < 0.1);
    BOOST_CHECK(avglength < 100.0);
}

BOOST_AUTO_TEST_SUITE_END()
