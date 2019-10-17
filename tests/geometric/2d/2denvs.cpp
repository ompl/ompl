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
#include "ompl/util/DisableCompilerWarning.h"
OMPL_PUSH_DISABLE_CLANG_WARNING(-Wunused-function)
OMPL_PUSH_DISABLE_GCC_WARNING(-Wunused-function)
#include "2DcirclesSetup.h"
OMPL_POP_CLANG
#include <iostream>

#include "ompl/base/spaces/RealVectorStateProjections.h"

#include "ompl/geometric/planners/kpiece/LBKPIECE1.h"
#include "ompl/geometric/planners/kpiece/BKPIECE1.h"
#include "ompl/geometric/planners/kpiece/KPIECE1.h"
#include "ompl/geometric/planners/sbl/SBL.h"
#include "ompl/geometric/planners/sbl/pSBL.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/rrt/pRRT.h"
#include "ompl/geometric/planners/rrt/TRRT.h"
#include "ompl/geometric/planners/rrt/LazyRRT.h"
#include "ompl/geometric/planners/pdst/PDST.h"
#include "ompl/geometric/planners/est/EST.h"
#include "ompl/geometric/planners/est/BiEST.h"
#include "ompl/geometric/planners/est/ProjEST.h"
#include "ompl/geometric/planners/stride/STRIDE.h"
#include "ompl/geometric/planners/prm/PRM.h"
#include "ompl/geometric/planners/prm/PRMstar.h"
#include "ompl/geometric/planners/prm/LazyPRM.h"
#include "ompl/geometric/planners/prm/LazyPRMstar.h"
#include "ompl/geometric/planners/prm/SPARS.h"
#include "ompl/geometric/planners/prm/SPARStwo.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"

#include "../../base/PlannerTest.h"

using namespace ompl;

static const double SOLUTION_TIME = 1.0;
static const bool VERBOSE = true;

/** \brief A base class for testing planners */
class TestPlanner
{
public:
    TestPlanner()
    {
        msg::setLogLevel(msg::LOG_ERROR);
    }

    virtual ~TestPlanner() = default;

    /* test a planner in a planar environment with circular obstacles */
    double test2DCircles(const Circles2D &circles, bool show = false, double *time = nullptr, double *pathLength = nullptr)
    {
        /* instantiate space information */
        base::SpaceInformationPtr si = geometric::spaceInformation2DCircles(circles);

        /* instantiate problem definition */
        auto pdef(std::make_shared<base::ProblemDefinition>(si));
        auto opt(std::make_shared<base::PathLengthOptimizationObjective>(si));
        /* make optimizing planners stop when any solution is found */
        opt->setCostThreshold(opt->infiniteCost());
        pdef->setOptimizationObjective(opt);

        /* instantiate motion planner */
        base::PlannerPtr planner = newPlanner(si);
        planner->setProblemDefinition(pdef);
        planner->setup();

        base::ScopedState<> start(si);
        base::ScopedState<> goal(si);
        unsigned int good = 0;

        for (std::size_t i = 0 ; i < circles.getQueryCount() ; ++i)
        {
            const Circles2D::Query &q = circles.getQuery(i);
            start[0] = q.startX_;
            start[1] = q.startY_;
            goal[0] = q.goalX_;
            goal[1] = q.goalY_;
            pdef->setStartAndGoalStates(start, goal, 1e-3);
            planner->clear();
            pdef->clearSolutionPaths();

            /* start counting time */
            ompl::time::point startTime = ompl::time::now();

            if (planner->solve(SOLUTION_TIME))
            {
                ompl::time::duration elapsed = ompl::time::now() - startTime;
                good++;
                if (time != nullptr)
                    *time += ompl::time::seconds(elapsed);
                if (show)
                    printf("Found solution in %f seconds!\n", ompl::time::seconds(elapsed));

                geometric::PathGeometric *path = static_cast<geometric::PathGeometric*>(pdef->getSolutionPath().get());

                /* make the solution more smooth */
                auto sm(std::make_shared<geometric::PathSimplifier>(si));

                startTime = ompl::time::now();
                sm->simplify(*path, SOLUTION_TIME);
                elapsed = ompl::time::now() - startTime;
                if (pathLength != nullptr)
                    *pathLength += path->length();
                if (time != nullptr)
                    *time += ompl::time::seconds(elapsed);
            }
        }

        if (pathLength != nullptr)
            *pathLength /= (double)circles.getQueryCount();
        if (time != nullptr)
            *time /= (double)circles.getQueryCount();

        return (double)good / (double)circles.getQueryCount();
    }


    /* test a planner in a planar grid environment where some cells are occupied */
    bool test2DEnv(const Environment2D &env, bool show = false, double *time = nullptr, double *pathLength = nullptr)
    {
        bool result = true;

        /* instantiate space information */
        base::SpaceInformationPtr si = geometric::spaceInformation2DMap(env);

        /* instantiate problem definition */
        base::ProblemDefinitionPtr pdef = geometric::problemDefinition2DMap(si, env);
        auto opt(std::make_shared<base::PathLengthOptimizationObjective>(si));
        /* make optimizing planners stop when any solution is found */
        opt->setCostThreshold(opt->infiniteCost());
        pdef->setOptimizationObjective(opt);

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
            if (time != nullptr)
                *time += ompl::time::seconds(elapsed);
            if (show)
                printf("Found solution in %f seconds!\n", ompl::time::seconds(elapsed));

            geometric::PathGeometric *path = static_cast<geometric::PathGeometric*>(pdef->getSolutionPath().get());


            /* make the solution more smooth */
            auto sm(std::make_shared<geometric::PathSimplifier>(si));

            startTime = ompl::time::now();
            sm->reduceVertices(*path);
            elapsed = ompl::time::now() - startTime;

            if (time != nullptr)
                *time += ompl::time::seconds(elapsed);

            if (show)
                printf("Simplified solution in %f seconds!\n", ompl::time::seconds(elapsed));

            /* fill in values that were linearly interpolated */
            path->interpolate(path->getStateCount() * 2);

            if (pathLength != nullptr)
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
                auto x = (int)(path->getState(i)->as<base::RealVectorStateSpace::StateType>()->values[0]);
                auto y = (int)(path->getState(i)->as<base::RealVectorStateSpace::StateType>()->values[1]);
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

    virtual base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) = 0;

};

class RRTTest : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) override
    {
        auto rrt(std::make_shared<geometric::RRT>(si));
        rrt->setRange(10.0);
        return rrt;
    }
};

class RRTConnectTest : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) override
    {
        auto rrt(std::make_shared<geometric::RRTConnect>(si));
        rrt->setRange(10.0);
        return rrt;
    }
};

class pRRTTest : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) override
    {
        auto rrt(std::make_shared<geometric::pRRT>(si));
        rrt->setRange(10.0);
        rrt->setThreadCount(std::min(4u, std::thread::hardware_concurrency()));
        return rrt;
    }
};

class TRRTTest : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) override
    {
        auto rrt(std::make_shared<geometric::TRRT>(si));
        rrt->setRange(10.0);
        return rrt;
    }
};

class LazyRRTTest : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) override
    {
        auto rrt(std::make_shared<geometric::LazyRRT>(si));
        rrt->setRange(10.0);
        return rrt;
    }
};

class SBLTest : public TestPlanner
{

protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) override
    {
        auto sbl(std::make_shared<geometric::SBL>(si));
        sbl->setRange(10.0);

        std::vector<unsigned int> projection = {0, 1};
        std::vector<double> cdim = {1, 1};

        sbl->setProjectionEvaluator(
            std::make_shared<base::RealVectorOrthogonalProjectionEvaluator>(
                si->getStateSpace(), cdim, projection));

        return sbl;
    }
};


class pSBLTest : public TestPlanner
{

protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) override
    {
        auto sbl(std::make_shared<geometric::pSBL>(si));
        sbl->setRange(10.0);
        sbl->setThreadCount(std::min(4u, std::thread::hardware_concurrency()));

        std::vector<unsigned int> projection = {0, 1};
        std::vector<double> cdim = {1, 1};

        sbl->setProjectionEvaluator(
            std::make_shared<base::RealVectorOrthogonalProjectionEvaluator>(
                si->getStateSpace(), cdim, projection));

        return sbl;
    }

};

class KPIECE1Test : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) override
    {
        auto kpiece(std::make_shared<geometric::KPIECE1>(si));
        kpiece->setRange(10.0);

        std::vector<unsigned int> projection = {0, 1};
        std::vector<double> cdim = {1, 1};

        kpiece->setProjectionEvaluator(
            std::make_shared<base::RealVectorOrthogonalProjectionEvaluator>(
                si->getStateSpace(), cdim, projection));

        return kpiece;
    }
};

class LBKPIECE1Test : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) override
    {
        auto kpiece(std::make_shared<geometric::LBKPIECE1>(si));
        kpiece->setRange(10.0);

        std::vector<unsigned int> projection = {0, 1};
        std::vector<double> cdim = {1, 1};

        kpiece->setProjectionEvaluator(
            std::make_shared<base::RealVectorOrthogonalProjectionEvaluator>(
                si->getStateSpace(), cdim, projection));

        return kpiece;
    }

};

class BKPIECE1Test : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) override
    {
        auto kpiece(std::make_shared<geometric::BKPIECE1>(si));
        kpiece->setRange(10.0);

        std::vector<unsigned int> projection = {0, 1};
        std::vector<double> cdim = {1, 1};

        kpiece->setProjectionEvaluator(
            std::make_shared<base::RealVectorOrthogonalProjectionEvaluator>(
                si->getStateSpace(), cdim, projection));

        return kpiece;
    }

};

class ESTTest : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) override
    {
        auto est(std::make_shared<geometric::EST>(si));
        est->setRange(10.0);
        return est;
    }

};

class BiESTTest : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) override
    {
        auto est(std::make_shared<geometric::BiEST>(si));
        est->setRange(10.0);
        return est;
    }

};

class ProjESTTest : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) override
    {
        auto est(std::make_shared<geometric::ProjEST>(si));
        est->setRange(10.0);

        std::vector<unsigned int> projection = {0, 1};
        std::vector<double> cdim = {1, 1};

        est->setProjectionEvaluator(
            std::make_shared<base::RealVectorOrthogonalProjectionEvaluator>(
                si->getStateSpace(), cdim, projection));

        return est;
    }

};

class STRIDETest : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) override
    {
        auto stride(std::make_shared<geometric::STRIDE>(si));
        stride->setRange(10.0);
        return stride;
    }

};

class PDSTTest : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) override
    {
        auto pdst(std::make_shared<geometric::PDST>(si));

        std::vector<unsigned int> projection = {0, 1};
        std::vector<double> cdim = {1, 1};

        pdst->setProjectionEvaluator(
            std::make_shared<base::RealVectorOrthogonalProjectionEvaluator>(
                si->getStateSpace(), cdim, projection));

        return pdst;
    }
};

class PRMTest : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) override
    {
        auto prm(std::make_shared<geometric::PRM>(si));
        return prm;
    }
};

class PRMstarTest : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) override
    {
        auto prm(std::make_shared<geometric::PRMstar>(si));
        return prm;
    }
};

class LazyPRMTest : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) override
    {
        auto prm(std::make_shared<geometric::LazyPRM>(si));
        return prm;
    }

};

class LazyPRMstarTest : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) override
    {
        auto prm(std::make_shared<geometric::LazyPRMstar>(si));
        return prm;
    }

};

class SPARSTest : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) override
    {
        auto spars(std::make_shared<geometric::SPARS>(si));
        return spars;
    }
};

class SPARStwoTest : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) override
    {
        auto sparstwo(std::make_shared<geometric::SPARStwo>(si));
        return sparstwo;
    }
};

class PlanTest
{
public:

    void simpleTest(TestPlanner *p)
    {
        geometric::SimpleSetup2DMap s(env_);
        s.setPlanner(p->newPlanner(s.getSpaceInformation()));

        auto opt(std::make_shared<base::PathLengthOptimizationObjective>(s.getSpaceInformation()));
        /* make optimizing planners stop when any solution is found */
        opt->setCostThreshold(opt->infiniteCost());
        s.setOptimizationObjective(opt);

        s.setup();
        base::PlannerTest pt(s.getPlanner());
        pt.test();
    }

    void terminationTest(TestPlanner *p)
    {
        class NeverValidStateValidityChecker : public base::StateValidityChecker
        {
        public:
            NeverValidStateValidityChecker(const base::SpaceInformationPtr &si,
                                           const base::State *start,
                                           const base::State *goal) :
                StateValidityChecker(si),
                start_(start),
                goal_(goal)
            {
            }

            bool isValid(const base::State *state) const override
            {
                return si_->equalStates(state, start_) || si_->equalStates(state, goal_);
            }

        private:
          const base::State *start_;
          const base::State *goal_;
        };

        geometric::SimpleSetup2DMap s(env_);
        s.setPlanner(p->newPlanner(s.getSpaceInformation()));

        // change the state validity checker to one that reports true only for the query states (no sampling will succeed)
        s.getSpaceInformation()->setStateValidityChecker(
            std::make_shared<NeverValidStateValidityChecker>(s.getSpaceInformation(),
                s.getProblemDefinition()->getStartState(0),
                s.getProblemDefinition()->getGoal()->as<base::GoalState>()->getState()));
        s.setup();
        if (verbose_)
            printf("Testing planner termination for %s. The planner should terminate within 0.1s; "
                   "If the test now hangs, it likely means there is an infinite loop in the planner.\n", s.getPlanner()->getName().c_str());
        s.solve(0.1);
        if (verbose_)
            printf("Terminated! Seeing this message means the test has passed!\n");
    }

    void run2DMapTest(TestPlanner *p, double *success, double *avgruntime, double *avglength)
    {
        double time   = 0.0;
        double length = 0.0;
        int    good   = 0;
        int    N      = 100;

        for (int i = 0 ; i < N ; ++i)
            if (p->test2DEnv(env_, false, &time, &length))
                good++;

        *success    = 100.0 * (double)good / (double)N;
        *avgruntime = time / (double)N;
        *avglength  = length / (double)N;

        if (verbose_)
        {
            printf("    Success rate: %f%%\n", *success);
            printf("    Average runtime: %f\n", *avgruntime);
            printf("    Average path length: %f\n", *avglength);
        }
    }

    void run2DCirclesTest(TestPlanner *p, double *success, double *avgruntime, double *avglength)
    {
        *success = 100.0 * p->test2DCircles(circles_, false, avgruntime, avglength);

        if (verbose_)
        {
            printf("    Success rate: %f%%\n", *success);
            printf("    Average runtime: %f\n", *avgruntime);
            printf("    Average path length: %f\n", *avglength);
        }
    }

    void runAllTests(TestPlanner *p, double min_success, double max_avgtime)
    {
        double success    = 0.0;
        double avgruntime = 0.0;
        double avglength  = 0.0;

        if (verbose_)
            printf("\n========= Running simple test\n\n");
        simpleTest(p);
        if (verbose_)
            printf("\n========= Running termination test\n\n");
        terminationTest(p);

        if (verbose_)
            printf("\n========= Running 2D map test\n\n");
        run2DMapTest(p, &success, &avgruntime, &avglength);
        BOOST_CHECK(success >= min_success);
        BOOST_CHECK(avgruntime < max_avgtime);
        BOOST_CHECK(avglength < 100.0);

        success    = 0.0;
        avgruntime = 0.0;
        avglength  = 0.0;

        if (verbose_)
            printf("\n========= Running 2D circles test\n\n");
        run2DCirclesTest(p, &success, &avgruntime, &avglength);

        BOOST_CHECK(success >= min_success);
        // this problem is a little more difficult than the one above, so we allow more time for its solution
        BOOST_CHECK(avgruntime < max_avgtime * 2.0);
        BOOST_CHECK(avglength < 100.0);
    }

    template<typename T>
    void runAllTests(double min_success, double max_avgtime)
    {
        TestPlanner *p = new T();
        runAllTests(p, min_success, max_avgtime);
        delete p;
    }

protected:

    PlanTest()
    {
        verbose_ = VERBOSE;
        boost::filesystem::path path(TEST_RESOURCES_DIR);
        loadEnvironment((path / "env1.txt").string().c_str(), env_);

        if (env_.width * env_.height == 0)
        {
            BOOST_FAIL( "The environment has a 0 dimension. Cannot continue" );
        }

        circles_.loadCircles((path / "circle_obstacles.txt").string());
        circles_.loadQueries((path / "circle_queries.txt").string());
    }

    Environment2D env_;
    Circles2D     circles_;
    bool          verbose_;
};

BOOST_FIXTURE_TEST_SUITE(MyPlanTestFixture, PlanTest)

#ifndef MACHINE_SPEED_FACTOR
#  define MACHINE_SPEED_FACTOR 3.0
#endif

// define boost tests for a planner assuming the naming convention is followed
#define OMPL_PLANNER_TEST(Name, MinSuccess, MaxAvgTime)                 \
    BOOST_AUTO_TEST_CASE(geometric_##Name)                                \
    {                                                                        \
        if (VERBOSE)                                                        \
            printf("\n\n\n*****************************\nTesting %s ...\n", #Name); \
        runAllTests<Name##Test>(MinSuccess, (MaxAvgTime) * MACHINE_SPEED_FACTOR); \
        if (VERBOSE)                                                        \
            printf("Done with %s.\n", #Name);                                \
    }

OMPL_PLANNER_TEST(RRT, 95.0, 0.01)
OMPL_PLANNER_TEST(RRTConnect, 95.0, 0.01)
OMPL_PLANNER_TEST(pRRT, 95.0, 0.02)

// LazyRRT is a not so great, so we use more relaxed bounds
//OMPL_PLANNER_TEST(LazyRRT, 80.0, 0.3)

OMPL_PLANNER_TEST(TRRT, 95.0, 0.01)

OMPL_PLANNER_TEST(PDST, 95.0, 0.03)

//OMPL_PLANNER_TEST(pSBL, 95.0, 0.04)
OMPL_PLANNER_TEST(SBL, 95.0, 0.02)

OMPL_PLANNER_TEST(KPIECE1, 95.0, 0.01)
OMPL_PLANNER_TEST(LBKPIECE1, 95.0, 0.02)
OMPL_PLANNER_TEST(BKPIECE1, 95.0, 0.01)

OMPL_PLANNER_TEST(EST, 95.0, 0.02)
OMPL_PLANNER_TEST(STRIDE, 95.0, 0.02)

OMPL_PLANNER_TEST(PRM, 95.0, 0.04)
OMPL_PLANNER_TEST(PRMstar, 95.0, 0.04)
//OMPL_PLANNER_TEST(LazyPRM, 98.0, 0.04)
OMPL_PLANNER_TEST(LazyPRMstar, 95.0, 0.04)
OMPL_PLANNER_TEST(SPARS, 95.0, 0.04)
OMPL_PLANNER_TEST(SPARStwo, 95.0, 0.04)

BOOST_AUTO_TEST_SUITE_END()
