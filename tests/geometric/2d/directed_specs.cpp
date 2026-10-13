/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Rice University
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
 *   * Neither the name of the Rice University nor the names of its
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

/* Author: Clayton Ramsey */

#define BOOST_TEST_MODULE "DirectedSpecs"
#include <boost/test/unit_test.hpp>

#include "ompl/base/ScopedState.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/util/Console.h"

#include "ompl/geometric/planners/est/BiEST.h"
#include "ompl/geometric/planners/est/EST.h"
#include "ompl/geometric/planners/fmt/FMT.h"
#include "ompl/geometric/planners/kpiece/KPIECE1.h"
#include "ompl/geometric/planners/rlrt/BiRLRT.h"
#include "ompl/geometric/planners/rlrt/RLRT.h"
#include "ompl/geometric/planners/rrt/AOXRRTConnect.h"
#include "ompl/geometric/planners/rrt/ATRRT.h"
#include "ompl/geometric/planners/rrt/BiTRRT.h"
#include "ompl/geometric/planners/rrt/LazyRRT.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/rrt/RRTstar.h"
#include "ompl/geometric/planners/rrt/TRRT.h"
#include "ompl/geometric/planners/rrt/TRRTstar.h"

#include <cmath>
#include <functional>
#include <string>
#include <vector>

// Some planners set `specs_.directed`, promising their solutions hold up when a motion from A to B says something
// different from a motion from B to A.
// Dubins curves with `isSymmetric_` false give an asymmetric space for testing these promises.

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace
{
    constexpr double EXTENT = 10.0;
    constexpr double TURNING_RADIUS = 0.6;
    constexpr double SOLVE_TIME = 1.0;
    constexpr unsigned int RUNS = 3u;

    /// A grid of square pillars.
    /// A Dubins car has to weave through them.
    bool isValid(const ob::State *state)
    {
        const auto *se2 = state->as<ob::DubinsStateSpace::StateType>();
        const double x = se2->getX();
        const double y = se2->getY();

        for (double cx = 2.0; cx < EXTENT - 1.0; cx += 2.0)
            for (double cy = 2.0; cy < EXTENT - 1.0; cy += 2.0)
                if (std::abs(x - cx) < 0.6 && std::abs(y - cy) < 0.6)
                    return false;

        return true;
    }

    og::SimpleSetup makeSetup()
    {
        auto space = std::make_shared<ob::DubinsStateSpace>(TURNING_RADIUS);

        ob::RealVectorBounds bounds(2);
        bounds.setLow(0.0);
        bounds.setHigh(EXTENT);
        space->setBounds(bounds);

        og::SimpleSetup setup(space);
        setup.setStateValidityChecker(isValid);

        ob::ScopedState<ob::DubinsStateSpace> start(space);
        start->setXY(0.5, 0.5);
        start->setYaw(0.0);

        ob::ScopedState<ob::DubinsStateSpace> goal(space);
        goal->setXY(EXTENT - 0.5, EXTENT - 0.5);
        goal->setYaw(0.0);

        setup.setStartAndGoalStates(start, goal, 0.3);
        return setup;
    }

    struct PlannerCase
    {
        std::function<ob::PlannerPtr(const ob::SpaceInformationPtr &)> make;
        double solveTime;
    };

    template <typename T>
    PlannerCase planner(double solveTime = SOLVE_TIME)
    {
        return {[](const ob::SpaceInformationPtr &si) { return std::make_shared<T>(si); }, solveTime};
    }

    /// BiRLRT has two growth modes.
    /// The range-limited one takes short steps and the keep-last one takes a step toward a uniform
    /// sample and truncates it, so the two exercise different edges.
    PlannerCase keepLastBiRLRT()
    {
        return {[](const ob::SpaceInformationPtr &si)
                {
                    auto planner = std::make_shared<og::BiRLRT>(si);
                    planner->setKeepLast(true);
                    planner->setName("BiRLRT-keep-last");
                    return planner;
                },
                SOLVE_TIME};
    }

    std::vector<PlannerCase> plannerCases()
    {
        // FMT draws one batch of samples and searches it, and on this problem that takes about a second
        // and a half, so it gets a longer budget than the planners that grow a tree.
        return {planner<og::BiEST>(),         planner<og::EST>(),     planner<og::FMT>(6.0),  planner<og::KPIECE1>(),
                planner<og::BiRLRT>(),        keepLastBiRLRT(),       planner<og::RLRT>(),    planner<og::ATRRT>(),
                planner<og::AOXRRTConnect>(), planner<og::BiTRRT>(),  planner<og::LazyRRT>(), planner<og::RRT>(),
                planner<og::RRTConnect>(),    planner<og::RRTstar>(), planner<og::TRRT>(),    planner<og::TRRTstar>()};
    }
}  // namespace

/// Make sure the search space is actually asymmetric with some regularity.
BOOST_AUTO_TEST_CASE(DirectionChangesTheVerdict)
{
    og::SimpleSetup setup = makeSetup();
    setup.setup();
    const ob::SpaceInformationPtr &si = setup.getSpaceInformation();

    BOOST_CHECK(!si->getStateSpace()->hasSymmetricInterpolate());

    ob::ScopedState<> a(si), b(si);
    auto sampler = si->allocValidStateSampler();

    unsigned int disagreements = 0;
    constexpr unsigned int pairs = 2000u;
    for (unsigned int i = 0; i < pairs; ++i)
    {
        sampler->sample(a.get());
        sampler->sample(b.get());
        if (si->checkMotion(a.get(), b.get()) != si->checkMotion(b.get(), a.get()))
            ++disagreements;
    }

    BOOST_CHECK_GT(disagreements, pairs / 100u);
}

BOOST_AUTO_TEST_CASE(DirectedPlannersReturnPathsThatCheck)
{
    ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

    for (const PlannerCase &plannerCase : plannerCases())
    {
        og::SimpleSetup setup = makeSetup();
        ob::PlannerPtr planner = plannerCase.make(setup.getSpaceInformation());
        setup.setPlanner(planner);
        setup.setup();

        const std::string name = planner->getName();
        BOOST_TEST_MESSAGE("planner " << name << " declares directed " << planner->getSpecs().directed);

        if (!planner->getSpecs().directed)
            continue;

        unsigned int solutions = 0;
        for (unsigned int run = 0; run < RUNS; ++run)
        {
            setup.clear();
            if (!setup.solve(plannerCase.solveTime) || !setup.haveExactSolutionPath())
                continue;

            ++solutions;
            BOOST_CHECK_MESSAGE(setup.getSolutionPath().check(),
                                name << " declares directed, and the space rejects its solution path");
        }

        BOOST_CHECK_MESSAGE(solutions > 0u, name << " solved nothing, so its promise went untested");
    }
}
