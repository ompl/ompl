/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Open Motion Planning Library contributors
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
 *   * Neither the name of the copyright holder nor the names of its
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

/* Author: Raphaël Cloutier */

// Regression test for null goalState_ dereference in FMT.
//
// When assureGoalIsSampled() cannot place a valid state in the goal region
// (e.g. every IK solution collides with the scene), goalState_ stays nullptr.
// Before the fix, motionCostHeuristic(state, goalState_) was called without a
// null guard in two places — expandTreeFromNode() and the extendedFMT branch of
// solve() — causing a segfault at address 0x8 inside StateSpace::distance().

#define BOOST_TEST_MODULE "FMT null goalState regression"
#include <boost/test/unit_test.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/base/PlannerTerminationCondition.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Validity checker: the entire goal region [0.8, 1.0]^2 is invalid, so
// assureGoalIsSampled() will fail to assign goalState_.
static bool isValid(const ob::State *state)
{
    const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
    return !(s->values[0] > 0.8 && s->values[1] > 0.8);
}

static ob::SpaceInformationPtr buildSI()
{
    auto space = std::make_shared<ob::RealVectorStateSpace>(2);
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1.0);
    bounds.setHigh(1.0);
    space->setBounds(bounds);

    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker(&isValid);
    si->setup();
    return si;
}

// Run FMT with the given flags and verify it returns FAILURE without crashing.
static void runFMT(bool heuristics, bool extendedFmt)
{
    auto si = buildSI();
    auto pdef = std::make_shared<ob::ProblemDefinition>(si);

    ob::ScopedState<> start(si->getStateSpace());
    start[0] = -0.5;
    start[1] = -0.5;
    pdef->addStartState(start);

    auto goal = std::make_shared<ob::GoalState>(si);
    ob::ScopedState<> goalState(si->getStateSpace());
    goalState[0] = 0.9;
    goalState[1] = 0.9;
    goal->setState(goalState);
    pdef->setGoal(goal);

    pdef->setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(si));

    auto planner = std::make_shared<og::FMT>(si);
    planner->setHeuristics(heuristics);
    planner->setExtendedFMT(extendedFmt);
    planner->setNumSamples(200);
    planner->setProblemDefinition(pdef);
    planner->setup();

    // Must return without crashing. Solution is not expected.
    ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(1.0));
    BOOST_CHECK(!status);
}

// Triggers the null-guard in expandTreeFromNode() (heuristics path).
BOOST_AUTO_TEST_CASE(fmt_null_goal_heuristics)
{
    runFMT(/*heuristics=*/true, /*extendedFmt=*/false);
}

// Triggers the null-guard in solve() (extendedFMT path).
BOOST_AUTO_TEST_CASE(fmt_null_goal_extended)
{
    runFMT(/*heuristics=*/false, /*extendedFmt=*/true);
}

// Both flags active simultaneously.
BOOST_AUTO_TEST_CASE(fmt_null_goal_heuristics_and_extended)
{
    runFMT(/*heuristics=*/true, /*extendedFmt=*/true);
}
