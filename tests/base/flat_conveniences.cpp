/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Rice University
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

#define BOOST_TEST_MODULE "FlatConveniences"
#include <boost/test/unit_test.hpp>

#include "ompl/base/ScopedState.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/FlatTrajectory.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/util/Exception.h"

#include <cmath>
#include <memory>
#include <vector>

using namespace ompl::base;
namespace og = ompl::geometric;

namespace
{
    constexpr unsigned int ORDER = 2;
    constexpr unsigned int DIMENSION = 2;

    /** \brief A flat space over a bounded Euclidean output with a box bound on the velocity. */
    std::shared_ptr<FlatStateSpace> makeSpace(double velocity = 2.)
    {
        auto output = std::make_shared<RealVectorStateSpace>(DIMENSION);
        output->setBounds(-1., 1.);
        auto space = std::make_shared<FlatStateSpace>(output, ORDER);
        space->setDerivativeBound(1, velocity);
        space->setup();
        return space;
    }

    /** \brief Write a flat output and a velocity into \e state. */
    void setState(const FlatStateSpace *space, State *state, std::initializer_list<double> output,
                  std::initializer_list<double> velocity)
    {
        Eigen::MatrixXd flat(ORDER, space->getOutputDimension());
        flat.row(0) = Eigen::RowVectorXd::Map(output.begin(), static_cast<Eigen::Index>(output.size()));
        flat.row(1) = Eigen::RowVectorXd::Map(velocity.begin(), static_cast<Eigen::Index>(velocity.size()));
        space->fromFlatState(flat, state);
    }

    /** \brief Whether the flat output of \e state sits outside a square hole of half width \e half. */
    bool outsideTheBox(const State *state, double half)
    {
        const double *values =
            state->as<FlatStateSpace::StateType>()->output()->as<RealVectorStateSpace::StateType>()->values;
        return std::abs(values[0]) > half || std::abs(values[1]) > half;
    }

    /** \brief A check that keeps the flat output out of a square hole and holds every state to the bounds
        of the space.

        The bounds belong in the check because the steering polynomial overshoots.
        A velocity inside its bound at both ends of an edge runs over it in the middle, and the motion
        validator samples those middles.
    */
    std::function<bool(const State *)> makeCheck(const FlatStateSpace *space, double half)
    {
        return [space, half](const State *state)
        { return space->satisfiesBounds(state) && outsideTheBox(state, half); };
    }

    /** \brief Space information over \e space holding the check from \ref makeCheck. */
    SpaceInformationPtr makeSpaceInformation(const std::shared_ptr<FlatStateSpace> &space, double half = 0.3)
    {
        auto si = std::make_shared<SpaceInformation>(space);
        si->setStateValidityChecker(makeCheck(space.get(), half));
        si->setup();
        return si;
    }
}  // namespace

BOOST_AUTO_TEST_CASE(ABoundBrokenMidwayRejectsTheMotion)
{
    // A velocity bound loose enough that both ends sit well inside it, and a pair of flat outputs far
    // enough apart that the polynomial joining them has to break it somewhere in between.
    const double bound = 1.;
    auto space = makeSpace(bound);
    auto si = makeSpaceInformation(space, 0.);

    ScopedState<> from(space), to(space);
    setState(space.get(), from.get(), {-0.9, -0.9}, {0., 0.});
    setState(space.get(), to.get(), {0.9, 0.9}, {0., 0.});

    const std::optional<FlatMotion> motion = space->steer(from.get(), to.get());
    BOOST_REQUIRE(motion.has_value());
    BOOST_REQUIRE_GT(motion->peakSpeed(), bound);

    BOOST_CHECK(si->isValid(from.get()));
    BOOST_CHECK(si->isValid(to.get()));
    BOOST_CHECK(!si->checkMotion(from.get(), to.get()));

    // raise the derivative cap to check that it's valid now.
    space->setDerivativeBound(1, 2. * motion->peakSpeed());
    BOOST_CHECK(si->checkMotion(from.get(), to.get()));
}

BOOST_AUTO_TEST_CASE(OtherSpacesAreRejected)
{
    auto plain = std::make_shared<RealVectorStateSpace>(DIMENSION);
    plain->setBounds(-1., 1.);
    auto si = std::make_shared<SpaceInformation>(plain);
    si->setStateValidityChecker([](const State *) { return true; });
    si->setup();

    ScopedState<> a(plain), b(plain);
    og::PathGeometric path(si, a.get(), b.get());

    // The extra parentheses keep this an expression.
    // Without them it declares a trajectory named path.
    BOOST_CHECK_THROW((FlatTrajectory(path)), ompl::Exception);
    BOOST_CHECK_THROW((FlatTrajectory(nullptr, std::vector<const State *>{})), ompl::Exception);
}

BOOST_AUTO_TEST_CASE(DurationsAddUpToTheWhole)
{
    auto space = makeSpace();
    StateSamplerPtr sampler = space->allocStateSampler();

    std::vector<ScopedState<>> owned;
    owned.reserve(6u);
    for (unsigned int i = 0; i < 6u; ++i)
    {
        owned.emplace_back(space);
        sampler->sampleUniform(owned.back().get());
    }

    std::vector<const State *> states;
    for (const ScopedState<> &state : owned)
        states.push_back(state.get());

    const FlatTrajectory trajectory(space.get(), states);
    BOOST_REQUIRE_EQUAL(trajectory.size(), states.size() - 1u);
    BOOST_CHECK_EQUAL(trajectory.outputDimension(), DIMENSION);

    double total = 0.;
    for (std::size_t i = 0; i < trajectory.size(); ++i)
    {
        BOOST_CHECK_CLOSE(trajectory.startTime(i), total, 1e-9);
        total += trajectory.motion(i).duration();
    }

    BOOST_CHECK_CLOSE(trajectory.duration(), total, 1e-9);
    BOOST_CHECK_CLOSE(trajectory.startTime(trajectory.size()), total, 1e-9);
    BOOST_CHECK_THROW(trajectory.motion(trajectory.size()), ompl::Exception);
}

BOOST_AUTO_TEST_CASE(ResamplingRecoversTheStates)
{
    auto space = makeSpace();
    StateSamplerPtr sampler = space->allocStateSampler();

    std::vector<ScopedState<>> owned;
    owned.reserve(6u);
    for (unsigned int i = 0; i < 6u; ++i)
    {
        owned.emplace_back(space);
        sampler->sampleUniform(owned.back().get());
    }

    std::vector<const State *> states;
    for (const ScopedState<> &state : owned)
        states.push_back(state.get());

    const FlatTrajectory trajectory(space.get(), states);
    BOOST_REQUIRE_EQUAL(trajectory.size(), states.size() - 1u);

    // Reading the trajectory at the time each segment starts gives back the state the planner steered
    // from, and reading it at the end gives the one it steered to.
    ScopedState<> resampled(space);
    for (std::size_t i = 0; i <= trajectory.size(); ++i)
    {
        trajectory.toState(space.get(), trajectory.startTime(i), resampled.get());
        BOOST_CHECK_SMALL(space->distance(states[i], resampled.get()), 1e-9);
    }

    // Times off either end clamp rather than running off the polynomial.
    trajectory.toState(space.get(), -1., resampled.get());
    BOOST_CHECK_SMALL(space->distance(states.front(), resampled.get()), 1e-9);
    trajectory.toState(space.get(), trajectory.duration() + 1., resampled.get());
    BOOST_CHECK_SMALL(space->distance(states.back(), resampled.get()), 1e-9);
}

BOOST_AUTO_TEST_CASE(ASolvedPathConvertsToATrajectory)
{
    auto space = makeSpace();
    og::SimpleSetup setup(space);
    setup.setStateValidityChecker(makeCheck(space.get(), 0.3));

    ScopedState<> start(space), goal(space);
    setState(space.get(), start.get(), {-0.8, -0.8}, {0., 0.});
    setState(space.get(), goal.get(), {0.8, 0.8}, {0., 0.});
    setup.setStartAndGoalStates(start, goal, 0.1);
    setup.setPlanner(std::make_shared<og::RRT>(setup.getSpaceInformation()));

    BOOST_REQUIRE(setup.solve(10.) == PlannerStatus::EXACT_SOLUTION);

    og::PathGeometric &path = setup.getSolutionPath();
    const FlatTrajectory trajectory(path);
    BOOST_REQUIRE_EQUAL(trajectory.size(), path.getStateCount() - 1u);
    BOOST_CHECK_GT(trajectory.duration(), 0.);

    ScopedState<> resampled(space);
    for (std::size_t i = 0; i < path.getStateCount(); ++i)
    {
        trajectory.toState(space.get(), trajectory.startTime(i), resampled.get());
        BOOST_CHECK_SMALL(space->distance(path.getState(i), resampled.get()), 1e-9);
    }
}

BOOST_AUTO_TEST_CASE(SimplifiedPathsRunTheWayTheyWereChecked)
{
    auto space = makeSpace();
    og::SimpleSetup setup(space);
    setup.setStateValidityChecker(makeCheck(space.get(), 0.3));

    ScopedState<> start(space), goal(space);
    setState(space.get(), start.get(), {-0.8, -0.8}, {0., 0.});
    setState(space.get(), goal.get(), {0.8, 0.8}, {0., 0.});
    setup.setStartAndGoalStates(start, goal, 0.1);
    setup.setPlanner(std::make_shared<og::RRT>(setup.getSpaceInformation()));

    BOOST_REQUIRE(setup.solve(10.) == PlannerStatus::EXACT_SOLUTION);

    const SpaceInformationPtr si = setup.getSpaceInformation();
    og::PathGeometric &path = setup.getSolutionPath();
    og::PathSimplifier simplifier(si);

    // Dropping vertices re-steers between the states it keeps and checks each new edge at the
    // discretization a fresh check will use, so its output holds up.
    simplifier.reduceVertices(path);
    BOOST_CHECK(path.check());
    BOOST_CHECK_GT(path.getStateCount(), 1u);

    // Smoothing rechecks both halves of every split in the direction the path runs through them, so it
    // leaves behind no edge that holds up only backwards.
    // Whether every edge survives a fresh check is a separate question, since smoothing splits edges and
    // a shorter edge gets sampled at different places than the one it came from.
    // Over two thousand runs of this problem that costs a flat space 0.75 percent of its smoothed paths
    // and a RealVectorStateSpace 7.8 percent of its own, so it belongs to the discretization rather than
    // to the space.
    simplifier.smoothBSpline(path);
    for (std::size_t i = 0; i + 1 < path.getStateCount(); ++i)
    {
        const State *from = path.getState(i);
        const State *to = path.getState(i + 1);
        if (!si->checkMotion(from, to))
            BOOST_CHECK(!si->checkMotion(to, from));
    }
}

BOOST_AUTO_TEST_CASE(ADerivativeSamplerLeavesTheOutputAlone)
{
    auto output = std::make_shared<RealVectorStateSpace>(DIMENSION);
    output->setBounds(-1., 1.);
    auto derivative = std::make_shared<RealVectorStateSpace>(DIMENSION);
    derivative->setBounds(-2., 2.);

    // An allocator on the derivative subspace alone, which is the light route for anyone who wants
    // different sampling without a different bound.
    derivative->setStateSamplerAllocator(
        [](const StateSpace *space)
        {
            class FixedSampler : public StateSampler
            {
            public:
                FixedSampler(const StateSpace *space) : StateSampler(space)
                {
                }

                void sampleUniform(State *state) override
                {
                    state->as<RealVectorStateSpace::StateType>()->values[0] = 0.25;
                    state->as<RealVectorStateSpace::StateType>()->values[1] = -0.25;
                }

                void sampleUniformNear(State *state, const State *, double) override
                {
                    sampleUniform(state);
                }

                void sampleGaussian(State *state, const State *, double) override
                {
                    sampleUniform(state);
                }
            };

            return std::make_shared<FixedSampler>(space);
        });

    auto space = std::make_shared<FlatStateSpace>(output, std::vector<StateSpacePtr>{derivative});
    space->setup();

    StateSamplerPtr sampler = space->allocStateSampler();
    ScopedState<> state(space);

    double lowest = 1.;
    double highest = -1.;
    for (unsigned int trial = 0; trial < 500u; ++trial)
    {
        sampler->sampleUniform(state.get());

        const auto *flat = state->as<FlatStateSpace::StateType>();
        BOOST_REQUIRE_CLOSE(flat->derivative(1)->values[0], 0.25, 1e-9);
        BOOST_REQUIRE_CLOSE(flat->derivative(1)->values[1], -0.25, 1e-9);

        const double value = flat->output()->as<RealVectorStateSpace::StateType>()->values[0];
        lowest = std::min(lowest, value);
        highest = std::max(highest, value);
    }

    // The flat output kept the sampler it had, so it still spreads over its whole range.
    BOOST_CHECK_LT(lowest, -0.9);
    BOOST_CHECK_GT(highest, 0.9);
}
