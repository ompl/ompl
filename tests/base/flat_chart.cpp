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

#define BOOST_TEST_MODULE "FlatChart"
#include <boost/test/unit_test.hpp>

#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/FlatStateSpace.h"
#include "ompl/base/spaces/FlatTrajectory.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/spaces/special/TorusStateSpace.h"

#include <boost/math/constants/constants.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

using namespace ompl::base;

namespace
{
    constexpr unsigned int ORDER = 2;
    const double PI = boost::math::constants::pi<double>();

    /** \brief An SE(2) flat output over a square with a box bound on every velocity. */
    std::shared_ptr<FlatStateSpace> makeSE2Space()
    {
        auto output = std::make_shared<SE2StateSpace>();
        RealVectorBounds bounds(2);
        bounds.setLow(-1.);
        bounds.setHigh(1.);
        output->setBounds(bounds);

        auto space = std::make_shared<FlatStateSpace>(output, ORDER);
        space->setDerivativeBound(1, 2.);
        space->setup();
        return space;
    }

    /** \brief A flat output on the torus with a box bound on both angular velocities. */
    std::shared_ptr<FlatStateSpace> makeTorusSpace()
    {
        auto space = std::make_shared<FlatStateSpace>(std::make_shared<TorusStateSpace>(), ORDER);
        space->setDerivativeBound(1, 2.);
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

    /** \brief The heading of an SE(2) flat output. */
    double headingOf(const State *state)
    {
        return state->as<FlatStateSpace::StateType>()->output()->as<SE2StateSpace::StateType>()->getYaw();
    }
}  // namespace

BOOST_AUTO_TEST_CASE(SanityChecksPassOverWrappingOutputs)
{
    BOOST_CHECK_NO_THROW(makeTorusSpace()->sanityChecks());
    BOOST_CHECK_NO_THROW(makeSE2Space()->sanityChecks());
}

BOOST_AUTO_TEST_CASE(SteeringPicksTheCheapestTurnOfEachAngle)
{
    auto space = makeTorusSpace();
    const MinimumEffortSteering &steering = space->getSteering();
    StateSamplerPtr sampler = space->allocStateSampler();

    // A flat output spinning fast can reach its target more cheaply by carrying on round than by turning
    // back, so steering has to weigh every whole number of turns on each angle, and this weighs them all
    // by brute force.
    ScopedState<> from(space), to(space);
    Eigen::MatrixXd initial(ORDER, 2), terminal(ORDER, 2);
    Eigen::VectorXd nearest(2);
    unsigned int wound = 0u;
    for (unsigned int trial = 0; trial < 2000u; ++trial)
    {
        sampler->sampleUniform(from.get());
        sampler->sampleUniform(to.get());

        const std::optional<FlatMotion> motion = space->steer(from.get(), to.get());
        BOOST_REQUIRE(motion.has_value());
        const double cost = steering.cost(*motion);

        space->toFlatState(from.get(), initial);
        space->toFlatState(to.get(), terminal);
        space->getChart()->difference(from->as<FlatStateSpace::StateType>()->output(),
                                      to->as<FlatStateSpace::StateType>()->output(), nearest);
        initial.row(0).setZero();

        double cheapest = std::numeric_limits<double>::infinity();
        for (int first = -3; first <= 3; ++first)
            for (int second = -3; second <= 3; ++second)
            {
                terminal(0, 0) = nearest[0] + 2. * PI * first;
                terminal(0, 1) = nearest[1] + 2. * PI * second;
                const std::optional<FlatMotion> candidate = steering.steer(initial, terminal);
                if (candidate.has_value())
                    cheapest = std::min(cheapest, steering.cost(*candidate));
            }

        BOOST_REQUIRE_LE(cost, cheapest * (1. + 1e-12));
        if (motion->evaluate(motion->duration()).cwiseAbs().maxCoeff() > PI)
            ++wound;
    }

    // Enough of the draws carry on round for the comparison to mean something.
    BOOST_CHECK_GT(wound, 20u);
}

BOOST_AUTO_TEST_CASE(TheHeadingRunsOnAcrossAJoin)
{
    auto space = makeSE2Space();

    // The middle waypoint sits just past the cut, so the second segment starts from a wrapped heading.
    ScopedState<> start(space), across(space), finish(space);
    setState(space.get(), start.get(), {-0.6, -0.8, 3.}, {0., 0., 0.});
    setState(space.get(), across.get(), {0., -0.8, -3.1}, {0.5, 0., 0.5});
    setState(space.get(), finish.get(), {0.6, -0.8, -2.8}, {0., 0., 0.});
    const FlatTrajectory trajectory(space, {start.get(), across.get(), finish.get()});
    BOOST_REQUIRE_EQUAL(trajectory.size(), 2u);

    // Level 0 carries the heading on past a half turn, while the state the trajectory writes wraps and
    // agrees with it modulo a turn.
    constexpr unsigned int SAMPLES = 2000;
    ScopedState<> sampled(space);
    double previous = trajectory.evaluate(0.)[2];
    for (unsigned int i = 0; i <= SAMPLES; ++i)
    {
        const double t = trajectory.duration() * i / SAMPLES;
        const double heading = trajectory.evaluate(t)[2];
        BOOST_REQUIRE_LT(std::abs(heading - previous), 0.05);
        previous = heading;

        trajectory.toState(t, sampled.get());
        BOOST_REQUIRE(space->satisfiesBounds(sampled.get()));
        BOOST_REQUIRE_SMALL(std::remainder(heading - headingOf(sampled.get()), 2. * PI), 1e-9);
    }

    BOOST_CHECK_CLOSE(trajectory.evaluate(trajectory.startTime(1))[2], 2. * PI - 3.1, 1e-9);
    BOOST_CHECK_CLOSE(trajectory.evaluate(trajectory.duration())[2], 2. * PI - 2.8, 1e-9);
}
