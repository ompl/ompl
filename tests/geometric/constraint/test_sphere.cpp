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

/* Author: Zachary Kingston */

#define BOOST_TEST_MODULE "ConstrainedPlanning"
#include <boost/test/unit_test.hpp>

#include <fstream>

#include <ompl/base/Constraint.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <ompl/base/spaces/constraint/TangentBundleStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include "ompl/geometric/planners/kpiece/BKPIECE1.h"
#include "ompl/geometric/planners/kpiece/KPIECE1.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/est/EST.h"
#include "ompl/geometric/planners/est/BiEST.h"
#include "ompl/geometric/planners/prm/PRM.h"

#include "ompl/base/Planner.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"

using namespace ompl;
namespace ob = ompl::base;
namespace og = ompl::geometric;

static const double SOLUTION_TIME = 5.0;
static const bool VERBOSE = true;

class Sphere : public ob::Constraint
{
public:
    Sphere() : ob::Constraint(3, 1)
    {
    }

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {
        out[0] = x.norm() - 1;
    }

    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
    {
        out = x.transpose().normalized();
    }
};

class SphereProjection : public ob::ProjectionEvaluator
{
public:
    SphereProjection(const ob::StateSpacePtr &space) : ob::ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        return 2;
    }

    void defaultCellSizes() override
    {
        cellSizes_.resize(2);
        cellSizes_[0] = 0.1;
        cellSizes_[1] = 0.1;
    }

    void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        auto &&x = *state->as<ob::ConstrainedStateSpace::StateType>();
        projection(0) = atan2(x[1], x[0]);
        projection(1) = acos(x[2]);
    }
};

bool isValid(const ob::State *state)
{
    auto &&x = *state->as<ob::ConstrainedStateSpace::StateType>();

    if (-0.80 < x[2] && x[2] < -0.6)
    {
        if (-0.05 < x[1] && x[1] < 0.05)
            return x[0] > 0;
        return false;
    }
    else if (-0.1 < x[2] && x[2] < 0.1)
    {
        if (-0.05 < x[0] && x[0] < 0.05)
            return x[1] < 0;
        return false;
    }
    else if (0.6 < x[2] && x[2] < 0.80)
    {
        if (-0.05 < x[1] && x[1] < 0.05)
            return x[0] < 0;
        return false;
    }

    return true;
}

enum SPACE_TYPE
{
    PJ,
    AT,
    TB
};

class TestPlanner
{
public:
    TestPlanner()
    {
        msg::setLogLevel(msg::LOG_ERROR);
    }

    virtual ~TestPlanner() = default;
    virtual ob::PlannerPtr newPlanner(const ob::SpaceInformationPtr &si) = 0;

    void setupSpace()
    {
        switch (type)
        {
            case PJ:
                OMPL_INFORM("Using Projection-Based State Space!");
                css = std::make_shared<ob::ProjectedStateSpace>(space, constraint);
                csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
                break;
            case AT:
                OMPL_INFORM("Using Atlas-Based State Space!");
                css = std::make_shared<ob::AtlasStateSpace>(space, constraint);
                csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
                break;
            case TB:
                OMPL_INFORM("Using Tangent Bundle-Based State Space!");
                css = std::make_shared<ob::TangentBundleStateSpace>(space, constraint);
                csi = std::make_shared<ob::TangentBundleSpaceInformation>(css);
                break;
        }

        css->setup();
    }

    void setStartAndGoalStates(ob::ProblemDefinitionPtr &pdef)
    {
        Eigen::VectorXd start(3), goal(3);
        start << 0, 0, -1;
        goal << 0, 0, 1;

        // Create start and goal states (poles of the sphere)
        ob::ScopedState<> sstart(css);
        ob::ScopedState<> sgoal(css);

        sstart->as<ob::ConstrainedStateSpace::StateType>()->copy(start);
        sgoal->as<ob::ConstrainedStateSpace::StateType>()->copy(goal);

        switch (type)
        {
            case AT:
            case TB:
                css->as<ob::AtlasStateSpace>()->anchorChart(sstart.get());
                css->as<ob::AtlasStateSpace>()->anchorChart(sgoal.get());
                break;
            default:
                break;
        }

        // Setup problem
        pdef->setStartAndGoalStates(sstart, sgoal, 1e-3);
    }

    bool testSphereEnv(enum SPACE_TYPE sp, double *time = nullptr, double *pathLength = nullptr)
    {
        type = sp;

        space = std::make_shared<ob::RealVectorStateSpace>(3);
        // Set bounds on the space.
        ob::RealVectorBounds bounds(3);
        bounds.setLow(-2);
        bounds.setHigh(2);
        space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

        constraint = std::make_shared<Sphere>();

        setupSpace();

        csi->setStateValidityChecker(isValid);

        /* instantiate problem definition */
        auto pdef(std::make_shared<ob::ProblemDefinition>(csi));
        auto opt(std::make_shared<ob::PathLengthOptimizationObjective>(csi));
        /* make optimizing planners stop when any solution is found */
        opt->setCostThreshold(opt->infiniteCost());
        pdef->setOptimizationObjective(opt);

        /* instantiate motion planner */
        ob::PlannerPtr planner = newPlanner(csi);
        planner->setProblemDefinition(pdef);
        planner->setup();

        setStartAndGoalStates(pdef);

        planner->clear();
        pdef->clearSolutionPaths();

        /* start counting time */
        ompl::time::point startTime = ompl::time::now();

        if (planner->solve(SOLUTION_TIME))
        {
            ompl::time::duration elapsed = ompl::time::now() - startTime;
            if (time != nullptr)
                *time += ompl::time::seconds(elapsed);

            og::PathGeometric *path = static_cast<og::PathGeometric *>(pdef->getSolutionPath().get());

            /* make the solution more smooth */
            auto sm(std::make_shared<og::PathSimplifier>(csi));

            startTime = ompl::time::now();
            sm->simplify(*path, SOLUTION_TIME);
            elapsed = ompl::time::now() - startTime;
            if (pathLength != nullptr)
                *pathLength += path->length();
            if (time != nullptr)
                *time += ompl::time::seconds(elapsed);

            return true;
        }
        return false;
    }

    ob::StateSpacePtr space;
    ob::ConstraintPtr constraint;

    ob::ConstrainedStateSpacePtr css;
    ob::ConstrainedSpaceInformationPtr csi;
    enum SPACE_TYPE type;
};

class PlanTest
{
public:
    void runSphereTest(TestPlanner *p, enum SPACE_TYPE type, double *success, double *avgruntime, double *avglength)
    {
        double time = 0.0;
        double length = 0.0;
        int good = 0;
        int N = 10;

        for (int i = 0; i < N; ++i)
            if (p->testSphereEnv(type, &time, &length))
                good++;

        *success = 100.0 * (double)good / (double)N;
        *avgruntime = time / (double)N;
        *avglength = length / (double)N;

        if (verbose_)
        {
            printf("    Success rate: %f%%\n", *success);
            printf("    Average runtime: %f\n", *avgruntime);
            printf("    Average path length: %f\n", *avglength);
        }
    }

    template <typename T, SPACE_TYPE N>
    void runAllTests(double min_success, double max_avgtime)
    {
        TestPlanner *p = new T();

        double success = 0.0;
        double avgruntime = 0.0;
        double avglength = 0.0;

        if (verbose_)
            printf("\n========= Running sphere test\n\n");

        runSphereTest(p, N, &success, &avgruntime, &avglength);
        BOOST_CHECK(success >= min_success);
        BOOST_CHECK(avgruntime < max_avgtime);
        BOOST_CHECK(avglength < 8.0);

        delete p;
    }

protected:
    PlanTest()
    {
        verbose_ = VERBOSE;
    }

    bool verbose_;
};

class RRTTest : public TestPlanner
{
protected:
    ob::PlannerPtr newPlanner(const ob::SpaceInformationPtr &si) override
    {
        auto rrt(std::make_shared<og::RRT>(si));
        return rrt;
    }
};

class RRTConnectTest : public TestPlanner
{
protected:
    ob::PlannerPtr newPlanner(const ob::SpaceInformationPtr &si) override
    {
        auto rrt(std::make_shared<og::RRTConnect>(si));
        return rrt;
    }
};

class KPIECE1Test : public TestPlanner
{
protected:
    ob::PlannerPtr newPlanner(const ob::SpaceInformationPtr &si) override
    {
        auto kpiece(std::make_shared<og::KPIECE1>(si));

        kpiece->setProjectionEvaluator(std::make_shared<SphereProjection>(si->getStateSpace()));

        return kpiece;
    }
};

class BKPIECE1Test : public TestPlanner
{
protected:
    ob::PlannerPtr newPlanner(const ob::SpaceInformationPtr &si) override
    {
        auto kpiece(std::make_shared<og::BKPIECE1>(si));

        kpiece->setProjectionEvaluator(std::make_shared<SphereProjection>(si->getStateSpace()));

        return kpiece;
    }
};

class ESTTest : public TestPlanner
{
protected:
    ob::PlannerPtr newPlanner(const ob::SpaceInformationPtr &si) override
    {
        auto est(std::make_shared<og::EST>(si));
        return est;
    }
};

class BiESTTest : public TestPlanner
{
protected:
    ob::PlannerPtr newPlanner(const ob::SpaceInformationPtr &si) override
    {
        auto est(std::make_shared<og::BiEST>(si));
        return est;
    }
};

class PRMTest : public TestPlanner
{
protected:
    ob::PlannerPtr newPlanner(const ob::SpaceInformationPtr &si) override
    {
        auto prm(std::make_shared<og::PRM>(si));
        return prm;
    }
};

BOOST_FIXTURE_TEST_SUITE(MyPlanTestFixture, PlanTest)

#ifndef MACHINE_SPEED_FACTOR
#define MACHINE_SPEED_FACTOR 3.0
#endif

// define boost tests for a planner assuming the naming convention is followed
#define OMPL_PLANNER_TEST(Name, Type, MinSuccess, MaxAvgTime)                                                          \
    BOOST_AUTO_TEST_CASE(geometric_##Name##Type)                                                                       \
    {                                                                                                                  \
        if (VERBOSE)                                                                                                   \
            printf("\n\n\n*****************************\nTesting %s %s ...\n", #Type, #Name);                          \
        runAllTests<Name##Test, Type>(MinSuccess, (MaxAvgTime) * MACHINE_SPEED_FACTOR);                                  \
        if (VERBOSE)                                                                                                   \
            printf("Done with %s.\n", #Name);                                                                          \
    }

OMPL_PLANNER_TEST(RRT, PJ, 95.0, 1.0)
OMPL_PLANNER_TEST(RRT, AT, 95.0, 1.0)
OMPL_PLANNER_TEST(RRT, TB, 95.0, 1.0)

OMPL_PLANNER_TEST(RRTConnect, PJ, 95.0, 1.0)
OMPL_PLANNER_TEST(RRTConnect, AT, 95.0, 1.0)
OMPL_PLANNER_TEST(RRTConnect, TB, 95.0, 1.0)

OMPL_PLANNER_TEST(KPIECE1, PJ, 95.0, 1.0)
OMPL_PLANNER_TEST(KPIECE1, AT, 95.0, 1.0)
OMPL_PLANNER_TEST(KPIECE1, TB, 95.0, 1.0)

OMPL_PLANNER_TEST(BKPIECE1, PJ, 95.0, 1.0)
OMPL_PLANNER_TEST(BKPIECE1, AT, 95.0, 1.0)
OMPL_PLANNER_TEST(BKPIECE1, TB, 95.0, 1.0)

OMPL_PLANNER_TEST(EST, PJ, 95.0, 1.0)
OMPL_PLANNER_TEST(EST, AT, 95.0, 1.0)
OMPL_PLANNER_TEST(EST, TB, 95.0, 1.0)

OMPL_PLANNER_TEST(BiEST, PJ, 95.0, 1.0)
OMPL_PLANNER_TEST(BiEST, AT, 95.0, 1.0)
OMPL_PLANNER_TEST(BiEST, TB, 95.0, 1.0)

OMPL_PLANNER_TEST(PRM, PJ, 95.0, 1.0)
OMPL_PLANNER_TEST(PRM, AT, 95.0, 1.0)
OMPL_PLANNER_TEST(PRM, TB, 95.0, 1.0)

BOOST_AUTO_TEST_SUITE_END()
