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

#define BOOST_TEST_MODULE "ControlPlanning"
#include <boost/test/unit_test.hpp>
#include <boost/filesystem.hpp>
#include <iostream>

#include "ompl/base/goals/GoalState.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/control/planners/rrt/RRT.h"
#include "ompl/control/planners/kpiece/KPIECE1.h"
#include "ompl/control/planners/est/EST.h"
#include "ompl/control/planners/pdst/PDST.h"
#include "ompl/control/planners/syclop/SyclopEST.h"
#include "ompl/control/planners/syclop/SyclopRRT.h"
#include "ompl/control/planners/syclop/GridDecomposition.h"

#include "../../resources/environment2D.h"

using namespace ompl;

static const double SOLUTION_TIME = 1.0;
static const double MAX_VELOCITY = 3.0;
static const bool VERBOSE = true;

/** Declare a class used in validating states. Such a class definition is needed for any use
 * of a kinematic planner */
class myStateValidityChecker : public base::StateValidityChecker
{
public:
    myStateValidityChecker(base::SpaceInformation *si, const std::vector<std::vector<int>> &grid)
      : base::StateValidityChecker(si)
    {
        setGrid(grid);
    }

    bool isValid(const base::State *state) const override
    {
        /* planning is done in a continuous space, but our collision space representation is discrete */
        auto x = (int)(state->as<base::RealVectorStateSpace::StateType>()->values[0]);
        auto y = (int)(state->as<base::RealVectorStateSpace::StateType>()->values[1]);

        if (x < 0 || y < 0 || x >= w_ || y >= h_)
            return false;

        return grid_[x][y] == 0;  // 0 means valid state
    }

    void setGrid(const std::vector<std::vector<int>> &grid)
    {
        grid_ = grid;
        w_ = grid_.size();
        h_ = grid_[0].size();
    }

protected:
    std::vector<std::vector<int>> grid_;
    int w_, h_;
};

class myStateSpace : public base::RealVectorStateSpace
{
public:
    myStateSpace() : base::RealVectorStateSpace(4)
    {
    }

    double distance(const base::State *state1, const base::State *state2) const override
    {
        /* planning is done in a continuous space, but our collision space representation is discrete */
        auto x1 = (int)(state1->as<base::RealVectorStateSpace::StateType>()->values[0]);
        auto y1 = (int)(state1->as<base::RealVectorStateSpace::StateType>()->values[1]);

        auto x2 = (int)(state2->as<base::RealVectorStateSpace::StateType>()->values[0]);
        auto y2 = (int)(state2->as<base::RealVectorStateSpace::StateType>()->values[1]);

        return abs(x1 - x2) + abs(y1 - y2);
    }
};

class myStatePropagator : public control::StatePropagator
{
public:
    myStatePropagator(const control::SpaceInformationPtr &si) : control::StatePropagator(si)
    {
    }

    void propagate(const base::State *state, const control::Control *control, const double duration,
                   base::State *result) const override
    {
        result->as<base::RealVectorStateSpace::StateType>()->values[0] =
            state->as<base::RealVectorStateSpace::StateType>()->values[0] +
            duration * control->as<control::RealVectorControlSpace::ControlType>()->values[0];
        result->as<base::RealVectorStateSpace::StateType>()->values[1] =
            state->as<base::RealVectorStateSpace::StateType>()->values[1] +
            duration * control->as<control::RealVectorControlSpace::ControlType>()->values[1];

        result->as<base::RealVectorStateSpace::StateType>()->values[2] =
            control->as<control::RealVectorControlSpace::ControlType>()->values[0];
        result->as<base::RealVectorStateSpace::StateType>()->values[3] =
            control->as<control::RealVectorControlSpace::ControlType>()->values[1];
        si_->getStateSpace()->enforceBounds(result);
    }
};

class myProjectionEvaluator : public base::ProjectionEvaluator
{
public:
    myProjectionEvaluator(const base::StateSpacePtr &space, const std::vector<double> &cellSizes)
      : base::ProjectionEvaluator(space)
    {
        setCellSizes(cellSizes);
        bounds_.resize(2);
        const base::RealVectorBounds &spacebounds = space->as<base::RealVectorStateSpace>()->getBounds();
        bounds_.setLow(0, spacebounds.low[0]);
        bounds_.setLow(1, spacebounds.low[1]);
        bounds_.setHigh(0, spacebounds.high[0]);
        bounds_.setHigh(1, spacebounds.high[1]);
    }

    unsigned int getDimension() const override
    {
        return 2;
    }

    void project(const base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        projection(0) = state->as<base::RealVectorStateSpace::StateType>()->values[0];
        projection(1) = state->as<base::RealVectorStateSpace::StateType>()->values[1];
    }
};

/** Space information */
control::SpaceInformationPtr mySpaceInformation(Environment2D &env)
{
    auto sMan(std::make_shared<myStateSpace>());

    base::RealVectorBounds sbounds(4);

    // dimension 0 (x) spans between [0, width)
    // dimension 1 (y) spans between [0, height)
    // since sampling is continuous and we round down, we allow values until just under the max limit
    // the resolution is 1.0 since we check cells only

    sbounds.low[0] = 0.0;
    sbounds.high[0] = (double)env.width - 0.000000001;

    sbounds.low[1] = 0.0;
    sbounds.high[1] = (double)env.height - 0.000000001;

    sbounds.low[2] = -MAX_VELOCITY;
    sbounds.high[2] = MAX_VELOCITY;

    sbounds.low[3] = -MAX_VELOCITY;
    sbounds.high[3] = MAX_VELOCITY;
    sMan->setBounds(sbounds);

    auto cMan(std::make_shared<control::RealVectorControlSpace>(sMan, 2));
    base::RealVectorBounds cbounds(2);

    cbounds.low[0] = -MAX_VELOCITY;
    cbounds.high[0] = MAX_VELOCITY;
    cbounds.low[1] = -MAX_VELOCITY;
    cbounds.high[1] = MAX_VELOCITY;
    cMan->setBounds(cbounds);

    auto si(std::make_shared<control::SpaceInformation>(sMan, cMan));
    si->setMinMaxControlDuration(2, 25);
    si->setPropagationStepSize(0.25);

    si->setStateValidityChecker(std::make_shared<myStateValidityChecker>(si.get(), env.grid));
    si->setStatePropagator(std::make_shared<myStatePropagator>(si));

    si->setup();

    return si;
}

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

        /* instantiate space information */
        control::SpaceInformationPtr si = mySpaceInformation(env);
        auto pdef(std::make_shared<base::ProblemDefinition>(si));

        /* instantiate motion planner */
        base::PlannerPtr planner = newPlanner(si);
        planner->setProblemDefinition(pdef);
        planner->setup();

        /* set the initial state; the memory for this is automatically cleaned by SpaceInformation */
        base::ScopedState<base::RealVectorStateSpace> state(si);
        state->values[0] = env.start.first;
        state->values[1] = env.start.second;
        state->values[2] = 0.0;
        state->values[3] = 0.0;
        pdef->addStartState(state);

        /* set the goal state; the memory for this is automatically cleaned by SpaceInformation */
        auto goal(std::make_shared<base::GoalState>(si));
        base::ScopedState<base::RealVectorStateSpace> gstate(si);
        gstate->values[0] = env.goal.first;
        gstate->values[1] = env.goal.second;
        gstate->values[2] = 0.0;
        gstate->values[3] = 0.0;
        goal->setState(gstate);
        goal->setThreshold(1e-3);  // this is basically 0, but we want to account for numerical instabilities
        pdef->setGoal(goal);

        planner->getProblemDefinition()->isStraightLinePathValid();

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

            control::PathControl *path = static_cast<control::PathControl *>(pdef->getSolutionPath().get());
            path->interpolate();

            if (!path->check())
                exit(1);

            elapsed = ompl::time::now() - startTime;

            if (time != nullptr)
                *time += ompl::time::seconds(elapsed);

            if (pathLength != nullptr)
                *pathLength += path->length();

            if (show)
            {
                printEnvironment(std::cout, env);
                std::cout << std::endl;
            }

            Environment2D temp = env;
            /* display the solution */
            for (unsigned int i = 0; i < path->getStateCount(); ++i)
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

protected:
    virtual base::PlannerPtr newPlanner(const control::SpaceInformationPtr &si) = 0;
};

class RRTTest : public TestPlanner
{
protected:
    base::PlannerPtr newPlanner(const control::SpaceInformationPtr &si) override
    {
        auto rrt(std::make_shared<control::RRT>(si));
        rrt->setIntermediateStates(false);
        return rrt;
    }
};

class RRTIntermediateTest : public TestPlanner
{
protected:
    base::PlannerPtr newPlanner(const control::SpaceInformationPtr &si) override
    {
        auto rrt(std::make_shared<control::RRT>(si));
        rrt->setIntermediateStates(true);
        return rrt;
    }
};

// A 2D workspace grid-decomposition for Syclop planners
class SyclopDecomposition : public control::GridDecomposition
{
public:
    SyclopDecomposition(const int len, const base::RealVectorBounds &b) : GridDecomposition(len, 2, b)
    {
    }

    void project(const base::State *s, std::vector<double> &coord) const override
    {
        coord.resize(2);
        coord[0] = s->as<base::RealVectorStateSpace::StateType>()->values[0];
        coord[1] = s->as<base::RealVectorStateSpace::StateType>()->values[1];
    }

    void sampleFullState(const base::StateSamplerPtr &sampler, const std::vector<double> &coord,
                         base::State *s) const override
    {
        sampler->sampleUniform(s);
        s->as<base::RealVectorStateSpace::StateType>()->values[0] = coord[0];
        s->as<base::RealVectorStateSpace::StateType>()->values[1] = coord[1];
    }

private:
    ompl::RNG rng_;
};

class SyclopRRTTest : public TestPlanner
{
    base::PlannerPtr newPlanner(const control::SpaceInformationPtr &si) override
    {
        base::RealVectorBounds bounds(2);

        const base::RealVectorBounds &spacebounds = si->getStateSpace()->as<base::RealVectorStateSpace>()->getBounds();
        bounds.setLow(0, spacebounds.low[0]);
        bounds.setLow(1, spacebounds.low[1]);
        bounds.setHigh(0, spacebounds.high[0]);
        bounds.setHigh(1, spacebounds.high[1]);

        // Create a 10x10 grid decomposition for Syclop
        auto decomp(std::make_shared<SyclopDecomposition>(10, bounds));

        auto srrt(std::make_shared<control::SyclopRRT>(si, decomp));
        // Set syclop parameters conducive to a tiny workspace
        srrt->setNumFreeVolumeSamples(1000);
        srrt->setNumRegionExpansions(10);
        srrt->setNumTreeExpansions(5);
        return srrt;
    }
};

class SyclopESTTest : public TestPlanner
{
    base::PlannerPtr newPlanner(const control::SpaceInformationPtr &si) override
    {
        base::RealVectorBounds bounds(2);

        const base::RealVectorBounds &spacebounds = si->getStateSpace()->as<base::RealVectorStateSpace>()->getBounds();
        bounds.setLow(0, spacebounds.low[0]);
        bounds.setLow(1, spacebounds.low[1]);
        bounds.setHigh(0, spacebounds.high[0]);
        bounds.setHigh(1, spacebounds.high[1]);

        // Create a 10x10 grid decomposition for Syclop
        auto decomp(std::make_shared<SyclopDecomposition>(10, bounds));

        auto sest(std::make_shared<control::SyclopEST>(si, decomp));
        // Set syclop parameters conducive to a tiny workspace
        sest->setNumFreeVolumeSamples(1000);
        sest->setNumRegionExpansions(10);
        sest->setNumTreeExpansions(5);
        return sest;
    }
};

class KPIECETest : public TestPlanner
{
protected:
    base::PlannerPtr newPlanner(const control::SpaceInformationPtr &si) override
    {
        auto kpiece(std::make_shared<control::KPIECE1>(si));

        std::vector<double> cdim = {1, 1};
        kpiece->setProjectionEvaluator(std::make_shared<myProjectionEvaluator>(si->getStateSpace(), cdim));

        return kpiece;
    }
};

class ESTTest : public TestPlanner
{
protected:
    base::PlannerPtr newPlanner(const control::SpaceInformationPtr &si) override
    {
        auto est(std::make_shared<control::EST>(si));

        std::vector<double> cdim = {1, 1};
        est->setProjectionEvaluator(std::make_shared<myProjectionEvaluator>(si->getStateSpace(), cdim));

        return est;
    }
};

class PDSTTest : public TestPlanner
{
protected:
    base::PlannerPtr newPlanner(const control::SpaceInformationPtr &si) override
    {
        auto pdst(std::make_shared<control::PDST>(si));

        std::vector<double> cdim = {1, 1};
        pdst->setProjectionEvaluator(std::make_shared<myProjectionEvaluator>(si->getStateSpace(), cdim));

        return pdst;
    }
};

class PlanTest
{
public:
    void runPlanTest(TestPlanner *p, double *success, double *avgruntime, double *avglength)
    {
        double time = 0.0;
        double length = 0.0;
        int good = 0;
        int N = 100;

        for (int i = 0; i < N; ++i)
            if (p->execute(env, false, &time, &length))
                good++;

        *success = 100.0 * (double)good / (double)N;
        *avgruntime = time / (double)N;
        *avglength = length / (double)N;

        if (verbose)
        {
            printf("    Success rate: %f%%\n", *success);
            printf("    Average runtime: %f\n", *avgruntime);
            printf("    Average path length: %f\n", *avglength);
        }
    }

    template <typename T>
    void runAllTests(double min_success, double max_avgtime)
    {
        double success = 0.0;
        double avgruntime = 0.0;
        double avglength = 0.0;

        TestPlanner *p = new T();
        runPlanTest(p, &success, &avgruntime, &avglength);
        delete p;

        BOOST_CHECK(success >= min_success);
        BOOST_CHECK(avgruntime < max_avgtime);
        BOOST_CHECK(avglength < 100.0);
    }

protected:
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

    Environment2D env;
    bool verbose;
};

BOOST_FIXTURE_TEST_SUITE(MyPlanTestFixture, PlanTest)

#define MACHINE_SPEED_FACTOR 1.0

// define boost tests for a planner assuming the naming convention is followed
#define OMPL_PLANNER_TEST(Name, MinSuccess, MaxAvgTime)                                                                \
    BOOST_AUTO_TEST_CASE(control_##Name)                                                                               \
    {                                                                                                                  \
        if (VERBOSE)                                                                                                   \
            printf("\n\n\n*****************************\nTesting %s ...\n", #Name);                                    \
        runAllTests<Name##Test>(MinSuccess, (MaxAvgTime) * MACHINE_SPEED_FACTOR);                                        \
        if (VERBOSE)                                                                                                   \
            printf("Done with %s.\n", #Name);                                                                          \
    }

OMPL_PLANNER_TEST(RRT, 99.0, 0.05)
OMPL_PLANNER_TEST(RRTIntermediate, 99.0, 0.25)
OMPL_PLANNER_TEST(KPIECE, 99.0, 0.05)
OMPL_PLANNER_TEST(EST, 99.0, 0.05)
OMPL_PLANNER_TEST(SyclopRRT, 99.0, 0.05)
OMPL_PLANNER_TEST(SyclopEST, 99.0, 0.05)
OMPL_PLANNER_TEST(PDST, 99.0, 0.05)

BOOST_AUTO_TEST_SUITE_END()
