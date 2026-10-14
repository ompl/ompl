/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Open Motion Planning Library contributors
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

/* Author: Elias (OMPL contribution) */
#define BOOST_TEST_MODULE "DDL BITstar"
#include <boost/test/unit_test.hpp>
#include <ompl/base/ScopedState.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/informedtrees/DDLBITstar.h>
#include <ompl/util/Exception.h>
#include <limits>

namespace ob = ompl::base;
namespace og = ompl::geometric;

struct Passage
{
    std::shared_ptr<ob::RealVectorStateSpace> space = std::make_shared<ob::RealVectorStateSpace>(2);
    std::shared_ptr<og::SimpleSetup> setup;
    std::shared_ptr<og::DDLBITstar> planner;

    Passage()
    {
        ob::RealVectorBounds bounds(2);
        bounds.setLow(-1.0);
        bounds.setHigh(1.0);
        space->setBounds(bounds);
        setup = std::make_shared<og::SimpleSetup>(space);
        // A wall separates the endpoints; its off-axis opening requires a detour.
        setup->setStateValidityChecker(
            [](const ob::State *state)
            {
                const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
                return std::abs(s->values[0]) > 0.12 || (s->values[1] > 0.45 && s->values[1] < 0.7);
            });
        setup->getSpaceInformation()->setStateValidityCheckingResolution(0.002);
        ob::ScopedState<> start(space), goal(space);
        start[0] = -0.8;
        start[1] = 0.0;
        goal[0] = 0.8;
        goal[1] = 0.0;
        setup->setStartAndGoalStates(start, goal);
        planner = std::make_shared<og::DDLBITstar>(setup->getSpaceInformation());
        planner->setGridResolution(0.15);
        planner->setBoundaryResolution(0.025);
        planner->setBoundaryStandardDeviation(0.08);
        setup->setPlanner(planner);
    }
};

BOOST_AUTO_TEST_CASE(solves_passage_and_reuses_after_clear)
{
    Passage p;
    BOOST_REQUIRE(p.setup->solve(2.0));
    BOOST_REQUIRE(p.setup->haveExactSolutionPath());
    BOOST_CHECK(p.setup->getSolutionPath().check());
    BOOST_CHECK_GT(p.planner->numBoundaryPoints(), 0u);
    BOOST_CHECK_GT(p.planner->numBoundarySamples(), 0u);
    BOOST_CHECK_GT(p.planner->numDensityRejections(), 0u);
    const double cost = p.planner->bestCost().value();
    p.setup->solve(0.1);
    BOOST_CHECK_LE(p.planner->bestCost().value(), cost);
    p.setup->clear();
    BOOST_CHECK_EQUAL(p.planner->numBoundaryPoints(), 0u);
    BOOST_CHECK_EQUAL(p.planner->numBoundarySamples(), 0u);
    BOOST_CHECK_EQUAL(p.planner->numDensityRejections(), 0u);
    BOOST_CHECK_CLOSE(p.planner->getGridResolution(), 0.15, 1e-8);
    BOOST_REQUIRE(p.setup->solve(2.0));
    BOOST_CHECK(p.setup->getSolutionPath().check());
}

BOOST_AUTO_TEST_CASE(soft_lock_allows_more_than_cell_capacity)
{
    Passage p;
    p.planner->setGridResolution(10.0);
    p.planner->setExplorationBias(0.0);
    p.planner->setOptimizationBias(0.0);
    // An impossible barrier prevents cost-triggered resets from hiding saturation.
    p.setup->setStateValidityChecker(
        [](const ob::State *state)
        { return std::abs(state->as<ob::RealVectorStateSpace::StateType>()->values[0]) > 0.12; });
    p.setup->solve(0.2);
    ob::PlannerData data(p.setup->getSpaceInformation());
    p.planner->getPlannerData(data);
    // At most four signed grid cells cover [-1,1]^2. A hard cap of one fails this.
    BOOST_CHECK_GT(data.numVertices(), 8u);
    BOOST_CHECK_GT(p.planner->numDensityRejections(), 0u);
    BOOST_CHECK_EQUAL(p.planner->numBoundarySamples(), 0u);
    BOOST_CHECK(!p.setup->haveExactSolutionPath());
}

BOOST_AUTO_TEST_CASE(rejects_invalid_configuration)
{
    Passage p;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    BOOST_CHECK_THROW(p.planner->setGridResolution(0.0), ompl::Exception);
    BOOST_CHECK_THROW(p.planner->setGridResolution(nan), ompl::Exception);
    BOOST_CHECK_THROW(p.planner->setBoundaryResolution(-1.0), ompl::Exception);
    BOOST_CHECK_THROW(p.planner->setBoundaryStandardDeviation(inf), ompl::Exception);
    BOOST_CHECK_THROW(p.planner->setExplorationBias(1.0), ompl::Exception);
    BOOST_CHECK_THROW(p.planner->setOptimizationBias(nan), ompl::Exception);
    BOOST_CHECK_THROW(p.planner->setOpenCellCapacity(0u), ompl::Exception);
    BOOST_CHECK_THROW(p.planner->setBoundaryCellCapacity(0u), ompl::Exception);
    BOOST_CHECK_THROW(p.planner->setScoutingRays(0u), ompl::Exception);
    p.setup->setup();
    BOOST_CHECK_THROW(p.planner->setGridResolution(0.1), ompl::Exception);
    p.planner->setUseKNearest(false);
    p.planner->setJustInTimeSampling(true);
    BOOST_CHECK_THROW(p.setup->solve(0.1), ompl::Exception);
}

BOOST_AUTO_TEST_CASE(rejects_unsupported_spaces)
{
    auto space = std::make_shared<ob::SE2StateSpace>();
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1.0);
    bounds.setHigh(1.0);
    space->setBounds(bounds);
    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker([](const ob::State *) { return true; });
    og::DDLBITstar planner(si);
    BOOST_CHECK_THROW(planner.setup(), ompl::Exception);
}

BOOST_AUTO_TEST_CASE(immediate_termination_does_not_generate_a_batch)
{
    Passage p;
    p.setup->solve(ob::plannerAlwaysTerminatingCondition());
    BOOST_CHECK_EQUAL(p.planner->numBatches(), 0u);
    BOOST_CHECK_EQUAL(p.planner->numBoundaryPoints(), 0u);
}

BOOST_AUTO_TEST_CASE(supports_higher_dimensional_real_vectors)
{
    auto space = std::make_shared<ob::RealVectorStateSpace>(6);
    ob::RealVectorBounds bounds(6);
    bounds.setLow(-1.0);
    bounds.setHigh(1.0);
    space->setBounds(bounds);
    og::SimpleSetup setup(space);
    setup.setStateValidityChecker(
        [](const ob::State *state)
        {
            const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
            return std::abs(s->values[0]) > 0.1 || s->values[1] > 0.2;
        });
    ob::ScopedState<> start(space), goal(space);
    for (unsigned int i = 0; i < 6; ++i)
        start[i] = goal[i] = 0.0;
    start[0] = -0.8;
    goal[0] = 0.8;
    setup.setStartAndGoalStates(start, goal);
    auto planner = std::make_shared<og::DDLBITstar>(setup.getSpaceInformation());
    planner->setGridResolution(0.3);
    planner->setBoundaryResolution(0.05);
    planner->setBoundaryStandardDeviation(0.15);
    setup.setPlanner(planner);
    BOOST_REQUIRE(setup.solve(2.0));
    BOOST_REQUIRE(setup.haveExactSolutionPath());
    BOOST_CHECK(setup.getSolutionPath().check());
    BOOST_CHECK_GT(planner->numBoundaryPoints(), 0u);
}

BOOST_AUTO_TEST_CASE(rejects_unbounded_and_unrepresentable_grids)
{
    Passage p;
    p.planner->setBoundaryResolution(std::numeric_limits<double>::min());
    BOOST_CHECK_THROW(p.setup->setup(), ompl::Exception);
    p.planner->setBoundaryResolution(0.02);
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-std::numeric_limits<double>::infinity());
    bounds.setHigh(std::numeric_limits<double>::infinity());
    p.space->setBounds(bounds);
    BOOST_CHECK_THROW(p.planner->setup(), ompl::Exception);
}

BOOST_AUTO_TEST_CASE(stops_scouting_when_termination_becomes_true)
{
    Passage p;
    unsigned int checks = 0;
    p.setup->setStateValidityChecker(
        [&checks](const ob::State *state)
        {
            ++checks;
            // BIT* can finish one pending edge check after perception returns.
            // A barrier keeps that check short, isolating cancellation of scouting.
            return std::abs(state->as<ob::RealVectorStateSpace::StateType>()->values[0]) > 0.12;
        });
    p.setup->setup();
    checks = 0;
    p.setup->solve(ob::PlannerTerminationCondition([&checks] { return checks >= 30; }));
    BOOST_CHECK_LE(checks, 32u);
}

BOOST_AUTO_TEST_CASE(scouting_honors_valid_segment_count_factor)
{
    auto space = std::make_shared<ob::RealVectorStateSpace>(1);
    ob::RealVectorBounds bounds(1);
    bounds.setLow(-1.0);
    bounds.setHigh(1.0);
    space->setBounds(bounds);
    space->setValidSegmentCountFactor(10);
    og::SimpleSetup setup(space);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.1);
    setup.setStateValidityChecker(
        [](const ob::State *state)
        {
            const double x = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
            return x < 0.09 || x > 0.11;
        });
    ob::ScopedState<> start(space), goal(space);
    start[0] = -0.8;
    goal[0] = 0.8;
    setup.setStartAndGoalStates(start, goal);
    auto planner = std::make_shared<og::DDLBITstar>(setup.getSpaceInformation());
    planner->setGridResolution(0.1);
    planner->setBoundaryResolution(0.01);
    planner->setBoundaryStandardDeviation(0.05);
    setup.setPlanner(planner);
    setup.solve(ob::PlannerTerminationCondition([planner] { return planner->numBatches() > 1; }));
    BOOST_CHECK_GT(planner->numBoundaryPoints(), 0u);
    BOOST_CHECK(!setup.haveExactSolutionPath());
}
