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

/* Author: Clayton W. Ramsey */

#define BOOST_TEST_MODULE "FlatConveniences"
#include <boost/test/unit_test.hpp>

#include "ompl/base/ScopedState.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/flat/FlatTrajectory.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/geometric/PathGeometric.h"

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

BOOST_AUTO_TEST_CASE(ATrajectoryResamplesToTheStatesItCameFrom)
{
    auto space = makeSpace();
    auto si = makeSpaceInformation(space);
    StateSamplerPtr sampler = space->allocStateSampler();

    // The state pointers below have to outlive the vector growing under them, so it reserves first and
    // they get collected once it has stopped growing.
    std::vector<ScopedState<>> owned;
    owned.reserve(6u);
    for (unsigned int i = 0; i < 6u; ++i)
    {
        owned.emplace_back(space);
        sampler->sampleUniform(owned.back().get());
    }

    std::vector<const State *> states;
    og::PathGeometric path(si);
    for (const ScopedState<> &state : owned)
    {
        states.push_back(state.get());
        path.append(state.get());
    }

    const auto holdsUp = [&](const FlatTrajectory &trajectory)
    {
        BOOST_REQUIRE_EQUAL(trajectory.size(), states.size() - 1u);
        BOOST_CHECK_EQUAL(trajectory.outputDimension(), DIMENSION);

        // Every segment starts where the ones before it left off, and they add up to the whole.
        double total = 0.;
        for (std::size_t i = 0; i < trajectory.size(); ++i)
        {
            BOOST_CHECK_CLOSE(trajectory.startTime(i), total, 1e-9);
            total += trajectory.motion(i).duration();
        }
        BOOST_CHECK_GT(total, 0.);
        BOOST_CHECK_CLOSE(trajectory.duration(), total, 1e-9);
        BOOST_CHECK_CLOSE(trajectory.startTime(trajectory.size()), total, 1e-9);

        // Reading the trajectory at the time each segment starts gives back the state the planner steered
        // from, and reading it at the end gives the one it steered to.
        ScopedState<> resampled(space);
        for (std::size_t i = 0; i <= trajectory.size(); ++i)
        {
            trajectory.toState(trajectory.startTime(i), resampled.get());
            BOOST_CHECK_SMALL(space->distance(states[i], resampled.get()), 1e-9);
        }

        // Times off either end clamp rather than running off the polynomial.
        trajectory.toState(-1., resampled.get());
        BOOST_CHECK_SMALL(space->distance(states.front(), resampled.get()), 1e-9);
        trajectory.toState(trajectory.duration() + 1., resampled.get());
        BOOST_CHECK_SMALL(space->distance(states.back(), resampled.get()), 1e-9);
    };

    // Building from the states and building from a path over those same states give the same trajectory.
    holdsUp(FlatTrajectory(space, states));
    holdsUp(FlatTrajectory(path));
}
