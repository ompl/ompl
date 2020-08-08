/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Willow Garage, Inc.
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

/* Author: Ioan Sucan, Luis G. Torres */

#define BOOST_TEST_MODULE "GeometricPlanningOpt"
#include <boost/test/unit_test.hpp>

#include "ompl/util/DisableCompilerWarning.h"
OMPL_PUSH_DISABLE_CLANG_WARNING(-Wunused-function)
OMPL_PUSH_DISABLE_GCC_WARNING(-Wunused-function)
#include "2DcirclesSetup.h"
OMPL_POP_CLANG
#include <iostream>

#include "ompl/base/goals/GoalState.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/geometric/planners/informedtrees/AITstar.h"
#include "ompl/geometric/planners/informedtrees/ABITstar.h"
#include "ompl/geometric/planners/informedtrees/BITstar.h"
#include "ompl/geometric/planners/cforest/CForest.h"
#include "ompl/geometric/planners/prm/PRMstar.h"
#include "ompl/geometric/planners/rrt/RRTstar.h"
#include "ompl/util/RandomNumbers.h"

#include "../../base/PlannerTest.h"

using namespace ompl;

static const double DT_SOLUTION_TIME = 0.1;
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

    virtual void test2DCircles(const Circles2D& circles)
    {
        base::SpaceInformationPtr si = geometric::spaceInformation2DCircles(circles);
        test2DCirclesGeneral(circles, si, 1.0);
    }

protected:

    virtual base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) const = 0;

    /* test a planner in a planar environment with circular obstacles */
    void test2DCirclesGeneral(const Circles2D &circles,
                              const base::SpaceInformationPtr &si,
                              double solutionTime)
    {
        /* instantiate problem definition */
        auto pdef(std::make_shared<base::ProblemDefinition>(si));

        // define an objective that is met the moment the solution is found
        auto opt(std::make_shared<base::PathLengthOptimizationObjective>(si));
        opt->setCostThreshold(opt->infiniteCost());
        pdef->setOptimizationObjective(opt);

        /* instantiate motion planner */
        base::PlannerPtr planner = newPlanner(si);
        planner->setProblemDefinition(pdef);
        planner->setup();

        std::size_t nt = std::min<std::size_t>(6, circles.getQueryCount());

        // run a simple test first
        if (nt > 0)
        {
            const Circles2D::Query &q = circles.getQuery(0);
            setupProblem(q, si, pdef);

            base::PlannerTest pt(planner);
            pt.test();
            planner->clear();
            pdef->clearSolutionPaths();
        }

        for (std::size_t i = 0 ; i < nt ; ++i)
        {
            const Circles2D::Query &q = circles.getQuery(i);
            setupProblem(q, si, pdef);

            base::Cost min_cost(std::sqrt(std::pow(q.goalX_ - q.startX_, 2.0) + std::pow(q.goalY_ - q.startY_, 2.0))); //The straight-line cost

            planner->clear();
            pdef->clearSolutionPaths();

            // we change the optimization objective so the planner runs until the first solution
            opt->setCostThreshold(opt->infiniteCost());

            time::point start = time::now();
            auto solved = planner->solve(solutionTime);
            if (solved == base::PlannerStatus::EXACT_SOLUTION)
            {
              // we change the optimization objective so the planner runs until timeout
              opt->setCostThreshold(base::Cost(std::numeric_limits<double>::epsilon()));

              geometric::PathGeometric *path = static_cast<geometric::PathGeometric*>(pdef->getSolutionPath().get());
              base::Cost ini_cost = path->cost(pdef->getOptimizationObjective());
              base::Cost prev_cost = ini_cost;
              double time_spent = time::seconds(time::now() - start);

              while (time_spent + DT_SOLUTION_TIME < solutionTime)
              {
                pdef->clearSolutionPaths();
                solved = planner->solve(DT_SOLUTION_TIME);
                BOOST_CHECK(solved);
                if (solved)
                {
                    geometric::PathGeometric *path = static_cast<geometric::PathGeometric*>(pdef->getSolutionPath().get());
                    base::Cost new_cost = path->cost(pdef->getOptimizationObjective());

                    BOOST_CHECK(!opt->isCostBetterThan(prev_cost, new_cost));

                    prev_cost = new_cost;
                    BOOST_CHECK(!pdef->hasOptimizedSolution());
                    BOOST_CHECK(!pdef->hasApproximateSolution());
                }
                time_spent = time::seconds(time::now() - start);
              }
              BOOST_CHECK(!opt->isCostBetterThan(ini_cost, prev_cost));

              pdef->clearSolutionPaths();
              // we change the optimization objective so the planner can achieve the objective
              opt->setCostThreshold(ini_cost);
              if (planner->solve(DT_SOLUTION_TIME))
              {
                  path = static_cast<geometric::PathGeometric*>(pdef->getSolutionPath().get());
                  prev_cost  = path->cost(pdef->getOptimizationObjective());
                  BOOST_CHECK(pdef->hasOptimizedSolution() || opt->isCostEquivalentTo(ini_cost, prev_cost));
              }

              // make sure not better than the minimum
              BOOST_CHECK(!opt->isCostBetterThan(prev_cost, min_cost));
            }
        }
    }

    // Similar test to above, but less strict about strictly
    // decreasing cost. This is because in this test we do not expect
    // to find goal states, so the planner effectively optimizes for
    // nearness to goal instead of path cost.
    void test2DCirclesNoGoalBias(const Circles2D &circles,
                                 const base::SpaceInformationPtr &si,
                                 double solutionTime)
    {
        /* instantiate problem definition */
        auto pdef(std::make_shared<base::ProblemDefinition>(si));

        // define an objective that is met the moment the solution is found
        auto opt(std::make_shared<base::PathLengthOptimizationObjective>(si));
        opt->setCostThreshold(base::Cost(std::numeric_limits<double>::infinity()));
        pdef->setOptimizationObjective(base::OptimizationObjectivePtr(opt));

        /* instantiate motion planner */
        base::PlannerPtr planner = newPlanner(si);
        planner->setProblemDefinition(pdef);
        planner->setup();

        std::size_t nt = std::min<std::size_t>(5, circles.getQueryCount());

        // run a simple test first
        if (nt > 0)
        {
            const Circles2D::Query &q = circles.getQuery(0);
            setupProblem(q, si, pdef);

            base::PlannerTest pt(planner);
            pt.test();
            planner->clear();
            pdef->clearSolutionPaths();
        }

        for (std::size_t i = 0 ; i < nt ; ++i)
        {
            const Circles2D::Query &q = circles.getQuery(i);
            setupProblem(q, si, pdef);

            planner->clear();
            pdef->clearSolutionPaths();

            // we change the optimization objective so the planner runs until the first solution
            opt->setCostThreshold(base::Cost(std::numeric_limits<double>::infinity()));

            time::point start = time::now();
            bool solved = planner->solve(solutionTime);
            if (solved)
            {
              // we change the optimization objective so the planner runs until timeout
              opt->setCostThreshold(base::Cost(std::numeric_limits<double>::epsilon()));

              geometric::PathGeometric *path = static_cast<geometric::PathGeometric*>(pdef->getSolutionPath().get());
              base::Cost ini_cost = path->cost(pdef->getOptimizationObjective());
              base::Cost prev_cost = ini_cost;
              double time_spent = time::seconds(time::now() - start);

              while (time_spent + DT_SOLUTION_TIME < solutionTime)
              {
                pdef->clearSolutionPaths();
                solved = planner->solve(DT_SOLUTION_TIME);
                BOOST_CHECK(solved);
                if (solved)
                {
                    geometric::PathGeometric *path = static_cast<geometric::PathGeometric*>(pdef->getSolutionPath().get());
                    base::Cost new_cost = path->cost(pdef->getOptimizationObjective());

                    BOOST_CHECK(!opt->isCostBetterThan(prev_cost, new_cost));

                    prev_cost = new_cost;
                    BOOST_CHECK(!pdef->hasOptimizedSolution());
                    BOOST_CHECK(!pdef->hasApproximateSolution());
                }
                time_spent = time::seconds(time::now() - start);
              }

              // In dubins no-goal-bias case, we can't guarantee that
              // ini_cost is actually greater than prev_cost
              BOOST_CHECK(ini_cost.value() >= prev_cost.value());

              pdef->clearSolutionPaths();
            }
        }
    }

    static void setupProblem(const Circles2D::Query &q,
                             const base::SpaceInformationPtr &si,
                             base::ProblemDefinitionPtr &pdef)
    {
        base::ScopedState<> start(si);
        start[0] = q.startX_;
        start[1] = q.startY_;

        base::GoalPtr goal = genGoalFromQuery(q, si);

        pdef->clearStartStates();
        pdef->addStartState(start);
        pdef->clearGoal();
        pdef->setGoal(goal);
    }

    static base::GoalPtr genGoalFromQuery(const Circles2D::Query &q,
                                          const base::SpaceInformationPtr &si)
    {
        base::ScopedState<> goal2D(si);
        goal2D[0] = q.goalX_;
        goal2D[1] = q.goalY_;

        auto goalState(std::make_shared<base::GoalState>(si));
        goalState->setState(goal2D);
        goalState->setThreshold(1e-3);
        return goalState;
    }
};

class RRTstarTest : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) const override
    {
        return std::make_shared<geometric::RRTstar>(si);
    }
};

class PRMstarTest : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) const override
    {
        return std::make_shared<geometric::PRMstar>(si);
    }
};

class PRMTest : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) const override
    {
        return std::make_shared<geometric::PRM>(si);
    }
};

class CForestTest : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) const override
    {
        return std::make_shared<geometric::CForest>(si);
    }
};

class AITstarTest : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) const override
    {
        return std::make_shared<geometric::AITstar>(si);
    }
};

class BITstarTest : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) const override
    {
        return std::make_shared<geometric::BITstar>(si);
    }
};

class ABITstarTest : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) const override
    {
        return std::make_shared<geometric::ABITstar>(si);
    }
};

// Seed the RNG so that a known edge case occurs in DubinsNoGoalBias
struct InitializeRandomSeed
{
    InitializeRandomSeed()
    {
        RNG::setSeed(3);
    }
};
static const InitializeRandomSeed seed_initializer;

class PlanTest
{
public:

    void run2DCirclesTest(TestPlanner *p)
    {
        p->test2DCircles(circles_);
    }

    template<typename T>
    void runAllTests()
    {
        TestPlanner *p = new T();
        run2DCirclesTest(p);
        delete p;
    }

protected:

    PlanTest()
    {
        verbose_ = VERBOSE;
        boost::filesystem::path path(TEST_RESOURCES_DIR);
        circles_.loadCircles((path / "circle_obstacles.txt").string());
        circles_.loadQueries((path / "circle_queries.txt").string());
    }

    Circles2D     circles_;
    bool          verbose_;
};

BOOST_FIXTURE_TEST_SUITE(MyPlanTestFixture, PlanTest)

// define boost tests for a planner assuming the naming convention is followed
#define OMPL_PLANNER_TEST(Name)                                                \
    BOOST_AUTO_TEST_CASE(geometric_##Name)                                \
    {                                                                        \
        if (VERBOSE)                                                        \
            printf("\n\n\n*****************************\nTesting %s ...\n", #Name); \
        runAllTests<Name##Test>();                                        \
        if (VERBOSE)                                                        \
            printf("Done with %s.\n", #Name);                                \
    }

OMPL_PLANNER_TEST(ABITstar)
OMPL_PLANNER_TEST(AITstar)
OMPL_PLANNER_TEST(BITstar)
OMPL_PLANNER_TEST(CForest)
OMPL_PLANNER_TEST(PRM)
OMPL_PLANNER_TEST(PRMstar)
OMPL_PLANNER_TEST(RRTstar)

BOOST_AUTO_TEST_SUITE_END()
