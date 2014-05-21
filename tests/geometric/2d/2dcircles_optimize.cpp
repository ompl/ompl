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

/* Author: Ioan Sucan */

#define BOOST_TEST_MODULE "GeometricPlanningOpt"
#include <boost/test/unit_test.hpp>

#include "2DcirclesSetup.h"
#include "2dcirclesSetupDubins.h"
#include <iostream>

#include "ompl/geometric/planners/rrt/TRRT.h"
#include "ompl/geometric/planners/prm/PRMstar.h"
#include "ompl/geometric/planners/rrt/RRTstar.h"
#include "ompl/geometric/planners/rrt/LBTRRT.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/util/RandomNumbers.h"

#include "../../BoostTestTeamCityReporter.h"
#include "../../base/PlannerTest.h"

using namespace ompl;

static const double DT_SOLUTION_TIME = 0.1;
static const bool VERBOSE = true;

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

    virtual base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si) = 0;


    virtual void test2DCircles(const Circles2D& circles)
    {
        base::SpaceInformationPtr si = geometric::spaceInformation2DCircles(circles);
        test2DCirclesGeneral(circles, si, 1.0, true);
    }

protected:

    /* test a planner in a planar environment with circular obstacles */
    void test2DCirclesGeneral(const Circles2D &circles,
                              const base::SpaceInformationPtr &si,
                              double solutionTime, // need different time for dubins
                              bool checkOptimized  // Must test dubins
                                                   // w/o goal bias,
                                                   // but w/o g/b
                                                   // dubins fails a
                                                   // check unless we
                                                   // disable with
                                                   // this flag
                              )
    {
        /* instantiate problem definition */
        base::ProblemDefinitionPtr pdef(new base::ProblemDefinition(si));

        // define an objective that is met the moment the solution is found
        base::PathLengthOptimizationObjective *opt = new base::PathLengthOptimizationObjective(si);
        opt->setCostThreshold(base::Cost(std::numeric_limits<double>::infinity()));
        pdef->setOptimizationObjective(base::OptimizationObjectivePtr(opt));

        /* instantiate motion planner */
        base::PlannerPtr planner = newPlanner(si);
        planner->setProblemDefinition(pdef);
        planner->setup();

        base::ScopedState<> start(si);

        // If this is a Dubins space, zero-out the SO2 subspace element.
        if (boost::dynamic_pointer_cast<base::DubinsStateSpace>(si->getStateSpace()))
            start[2] = 0.0;

        std::size_t nt = std::min<std::size_t>(5, circles.getQueryCount());

        // run a simple test first
        if (nt > 0)
        {
            const Circles2D::Query &q = circles.getQuery(0);
            start[0] = q.startX_;
            start[1] = q.startY_;

            base::GoalPtr goal = genGoalFromQuery(q, si);

            pdef->clearStartStates();
            pdef->addStartState(start);
            pdef->clearGoal();
            pdef->setGoal(goal);

            base::PlannerTest pt(planner);
            pt.test();
            planner->clear();
            pdef->clearSolutionPaths();
        }

        for (std::size_t i = 0 ; i < nt ; ++i)
        {
            const Circles2D::Query &q = circles.getQuery(i);
            start[0] = q.startX_;
            start[1] = q.startY_;

            base::GoalPtr goal = genGoalFromQuery(q, si);

            pdef->clearStartStates();
            pdef->addStartState(start);
            pdef->clearGoal();
            pdef->setGoal(goal);

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

                    // We don't use opt->isCostBetterThan() because
                    // isCostBetterThan() defaults to '<', which can
                    // cause this test to fail
                    BOOST_CHECK(new_cost.v <= prev_cost.v);

                    prev_cost = new_cost;
                    BOOST_CHECK(!pdef->hasOptimizedSolution());
                    BOOST_CHECK(!pdef->hasApproximateSolution());
                }
                time_spent = time::seconds(time::now() - start);
              }
              if (checkOptimized)
                  BOOST_CHECK(opt->isCostBetterThan(prev_cost, ini_cost));
              else
                  BOOST_CHECK(ini_cost.v >= prev_cost.v); // for dubins goal bias case

              pdef->clearSolutionPaths();
              // we change the optimization objective so the planner can achieve the objective
              opt->setCostThreshold(ini_cost);
              if (checkOptimized && planner->solve(DT_SOLUTION_TIME))
                  BOOST_CHECK(pdef->hasOptimizedSolution());
            }
        }
    }

    static base::GoalPtr genGoalFromQuery(const Circles2D::Query &q,
                                          const base::SpaceInformationPtr &si)
    {
        base::ScopedState<> goal2D(si);
        goal2D[0] = q.goalX_;
        goal2D[1] = q.goalY_;

        base::GoalPtr goal;
        if (boost::dynamic_pointer_cast<base::DubinsStateSpace>(si->getStateSpace()))
        {
            geometric::DubinsXYGoal* dubinsGoal =
                new geometric::DubinsXYGoal(si, goal2D.get());
            dubinsGoal->setThreshold(1e-3);
            goal = base::GoalPtr(dubinsGoal);
        }
        else
        {
            base::GoalState* goalState = new base::GoalState(si);
            goalState->setState(goal2D);
            goalState->setThreshold(1e-3);
            goal = base::GoalPtr(goalState);
        }

        return goal;
    }
};

class RRTstarTest : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si)
    {
        geometric::RRTstar *rrt = new geometric::RRTstar(si);
        return base::PlannerPtr(rrt);
    }
};

class PRMstarTest : public TestPlanner
{
protected:

    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si)
    {
        geometric::PRMstar *prm = new geometric::PRMstar(si);
        return base::PlannerPtr(prm);
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

class RRTstarDubinsTest : public TestPlanner
{
protected:
    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si)
    {
        geometric::RRTstar *rrt = new geometric::RRTstar(si);
        return base::PlannerPtr(rrt);
    }

    void test2DCircles(const Circles2D& circles)
    {
        base::SpaceInformationPtr si = geometric::spaceInformation2DCirclesDubins(circles);
        test2DCirclesGeneral(circles, si, 5.0, true);
    }
};

// This is to test an edge case that I've only been able to reproduce
// without goal bias.
class RRTstarDubinsNoGoalBiasTest : public TestPlanner
{
protected:
    base::PlannerPtr newPlanner(const base::SpaceInformationPtr &si)
    {
        geometric::RRTstar *rrt = new geometric::RRTstar(si);
        rrt->setGoalBias(0.0);
        return base::PlannerPtr(rrt);
    }

    void test2DCircles(const Circles2D& circles)
    {
        base::SpaceInformationPtr si = geometric::spaceInformation2DCirclesDubins(circles);
        test2DCirclesGeneral(circles, si, 5.0, false);
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
    void runAllTests(void)
    {
        TestPlanner *p = new T();
        run2DCirclesTest(p);
        delete p;
    }

protected:

    PlanTest(void)
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

OMPL_PLANNER_TEST(PRMstar)
OMPL_PLANNER_TEST(PRM)
OMPL_PLANNER_TEST(RRTstar)
OMPL_PLANNER_TEST(RRTstarDubinsNoGoalBias)
OMPL_PLANNER_TEST(RRTstarDubins)

BOOST_AUTO_TEST_SUITE_END()
