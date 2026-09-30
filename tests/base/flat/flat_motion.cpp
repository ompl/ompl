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

#define BOOST_TEST_MODULE "FlatMotion"
#include <boost/test/unit_test.hpp>

#include "ompl/base/spaces/flat/FlatMotion.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <utility>
#include <vector>

using namespace ompl::base;

namespace
{
    constexpr unsigned int ORDER = 2;
    constexpr unsigned int DIMENSION = 3;

    /** \brief Draw a flat state of order \e order whose flat output lies in a box and whose derivatives lie in
        a smaller one. */
    Eigen::MatrixXd randomFlatState(std::mt19937_64 &engine, unsigned int dimension = DIMENSION,
                                    unsigned int order = ORDER)
    {
        std::uniform_real_distribution<double> output(-2., 2.);
        std::uniform_real_distribution<double> derivative(-1., 1.);
        Eigen::MatrixXd state(order, dimension);
        for (unsigned int i = 0; i < dimension; ++i)
        {
            state(0, i) = output(engine);
            for (unsigned int level = 1; level < order; ++level)
                state(level, i) = derivative(engine);
        }
        return state;
    }

    /** \brief Read the flat state of order \e order a motion passes through at time \e t. */
    Eigen::MatrixXd flatStateAt(const FlatMotion &motion, double t, unsigned int order = ORDER)
    {
        Eigen::MatrixXd state(order, motion.outputDimension());
        Eigen::VectorXd level(motion.outputDimension());
        for (unsigned int derivative = 0; derivative < order; ++derivative)
        {
            motion.evaluate(t, derivative, level);
            state.row(derivative) = level.transpose();
        }
        return state;
    }

    /** \brief The cost of steering from \e from to \e to over exactly \e duration. */
    double costOverDuration(const Eigen::MatrixXd &from, const Eigen::MatrixXd &to, double duration, double rho)
    {
        FixedDurationSteering steering(static_cast<unsigned int>(from.rows()), duration);
        steering.setRho(rho);
        return steering.cost(*steering.steer(from, to));
    }
}  // namespace

BOOST_AUTO_TEST_CASE(BoundaryConditions)
{
    std::mt19937_64 engine(1u);

    // The top half of the coefficients solves a system whose matrix grows ill conditioned with the order,
    // and dividing out the duration scales the solve by up to T^(2k - 1), so this runs up to order 5 over
    // durations spanning two decades either side of 1.
    for (unsigned int order = 1; order <= 5u; ++order)
    {
        MinimumEffortSteering effort(order);
        for (unsigned int trial = 0; trial < 200u; ++trial)
        {
            const Eigen::MatrixXd from = randomFlatState(engine, DIMENSION, order);
            const Eigen::MatrixXd to = randomFlatState(engine, DIMENSION, order);

            const auto motion = effort.steer(from, to);
            BOOST_REQUIRE(motion.has_value());

            BOOST_CHECK_SMALL((flatStateAt(*motion, 0., order) - from).cwiseAbs().maxCoeff(), 1e-9);
            BOOST_CHECK_SMALL((flatStateAt(*motion, motion->duration(), order) - to).cwiseAbs().maxCoeff(), 1e-9);
        }

        // Pinning the duration lands on both flat states too.
        for (double duration : {0.05, 1., 20.})
        {
            FixedDurationSteering steering(order, duration);

            for (unsigned int trial = 0; trial < 100u; ++trial)
            {
                const Eigen::MatrixXd from = randomFlatState(engine, DIMENSION, order);
                const Eigen::MatrixXd to = randomFlatState(engine, DIMENSION, order);

                const auto motion = steering.steer(from, to);
                BOOST_REQUIRE(motion.has_value());
                BOOST_CHECK_EQUAL(motion->duration(), duration);
                BOOST_CHECK_EQUAL(motion->degree(), 2u * order - 1u);
                BOOST_CHECK_EQUAL(motion->outputDimension(), DIMENSION);

                // Each level of the curve is a sum of terms far larger than the level itself over a short
                // duration, so the error gets measured against the largest of those terms.
                for (unsigned int level = 0; level < order; ++level)
                {
                    double scale = 1.;
                    for (Eigen::Index i = level; i < motion->coefficients().rows(); ++i)
                    {
                        double weight = std::pow(duration, static_cast<double>(i - level));
                        for (unsigned int j = 0; j < level; ++j)
                            weight *= static_cast<double>(i - j);
                        scale = std::max(scale, weight * motion->coefficients().row(i).cwiseAbs().maxCoeff());
                    }

                    BOOST_CHECK_SMALL((flatStateAt(*motion, 0., order) - from).row(level).cwiseAbs().maxCoeff(),
                                      1e-12 * scale);
                    BOOST_CHECK_SMALL((flatStateAt(*motion, duration, order) - to).row(level).cwiseAbs().maxCoeff(),
                                      1e-12 * scale);
                }
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(Order2MatchesTheClosedForm)
{
    std::mt19937_64 engine(2u);

    // The cubic joining two order 2 flat states over a duration T, and what it costs, in closed form.
    for (double duration : {0.05, 1., 7.5})
    {
        FixedDurationSteering steering(ORDER, duration);
        const double T = duration;
        for (unsigned int trial = 0; trial < 200u; ++trial)
        {
            const Eigen::MatrixXd from = randomFlatState(engine);
            const Eigen::MatrixXd to = randomFlatState(engine);

            const Eigen::RowVectorXd v0 = from.row(1);
            const Eigen::RowVectorXd vf = to.row(1);
            const Eigen::RowVectorXd offset = to.row(0) - from.row(0) - v0 * T;
            const Eigen::RowVectorXd change = vf - v0;

            Eigen::MatrixXd expected(4, DIMENSION);
            expected.row(0) = from.row(0);
            expected.row(1) = v0;
            expected.row(2) = 3. * offset / (T * T) - change / T;
            expected.row(3) = -2. * offset / (T * T * T) + change / (T * T);

            const auto motion = steering.steer(from, to);
            BOOST_REQUIRE(motion.has_value());
            const double scale = expected.cwiseAbs().maxCoeff();
            BOOST_CHECK_SMALL((motion->coefficients() - expected).cwiseAbs().maxCoeff(), 1e-12 * scale);

            const Eigen::RowVectorXd displacement = to.row(0) - from.row(0);
            const double cost = 4. * (v0.squaredNorm() + v0.dot(vf) + vf.squaredNorm()) / T -
                                12. * displacement.dot(v0 + vf) / (T * T) +
                                12. * displacement.squaredNorm() / (T * T * T) + steering.getRho() * T;
            const auto steered = steering.steeringCost(from, to);
            BOOST_REQUIRE(steered.has_value());
            BOOST_CHECK_CLOSE(*steered, cost, 1e-9);
        }
    }
}

BOOST_AUTO_TEST_CASE(CostMatchesNumericIntegration)
{
    std::mt19937_64 engine(4u);
    MinimumEffortSteering steering(ORDER);
    const double rho = steering.getRho();

    for (unsigned int trial = 0; trial < 100u; ++trial)
    {
        const auto motion = steering.steer(randomFlatState(engine), randomFlatState(engine));
        BOOST_REQUIRE(motion.has_value());

        // Simpson's rule over the squared norm of the acceleration.
        const FlatMotion acceleration = motion->derivative().derivative();
        const unsigned int panels = 200u;
        const double width = motion->duration() / static_cast<double>(panels);
        double effort = 0.;
        for (unsigned int panel = 0; panel < panels; ++panel)
        {
            const double left = static_cast<double>(panel) * width;
            effort += width / 6. *
                      (acceleration.evaluate(left).squaredNorm() +
                       4. * acceleration.evaluate(left + 0.5 * width).squaredNorm() +
                       acceleration.evaluate(left + width).squaredNorm());
        }

        const double expected = effort + rho * motion->duration();
        BOOST_CHECK_CLOSE(steering.cost(*motion), expected, 1e-6);
        BOOST_CHECK_CLOSE(motion->cost(ORDER, rho), expected, 1e-6);
    }
}

BOOST_AUTO_TEST_CASE(SteeringCostMatchesTheSteeredMotion)
{
    std::mt19937_64 engine(9u);

    // The price comes out of a quadratic form over the conditions at the far end, and the motion out of a
    // linear solve, so agreement checks one against the other at every order.
    for (unsigned int order = 1; order <= 5u; ++order)
    {
        MinimumEffortSteering effort(order);
        for (unsigned int trial = 0; trial < 200u; ++trial)
        {
            const Eigen::MatrixXd from = randomFlatState(engine, DIMENSION, order);
            const Eigen::MatrixXd to = randomFlatState(engine, DIMENSION, order);

            const auto motion = effort.steer(from, to);
            const auto cost = effort.steeringCost(from, to);
            BOOST_REQUIRE(motion.has_value());
            BOOST_REQUIRE(cost.has_value());
            BOOST_CHECK_CLOSE(*cost, effort.cost(*motion), 1e-7);
        }

        for (double duration : {0.05, 1., 7.5})
        {
            FixedDurationSteering steering(order, duration);

            for (unsigned int trial = 0; trial < 100u; ++trial)
            {
                const Eigen::MatrixXd from = randomFlatState(engine, DIMENSION, order);
                const Eigen::MatrixXd to = randomFlatState(engine, DIMENSION, order);

                const auto cost = steering.steeringCost(from, to);
                BOOST_REQUIRE(cost.has_value());
                BOOST_CHECK_CLOSE(*cost, steering.cost(*steering.steer(from, to)), 1e-7);
            }
        }
    }

    // Flat states that already sit still at the same flat output have no motion and so no cost.
    for (unsigned int order = 1; order <= 5u; ++order)
    {
        const Eigen::MatrixXd still = Eigen::MatrixXd::Zero(order, DIMENSION);
        BOOST_CHECK(!MinimumEffortSteering(order).steeringCost(still, still).has_value());
    }
}

BOOST_AUTO_TEST_CASE(OptimalDurationIsTheCheapestDuration)
{
    std::mt19937_64 engine(5u);

    // Only a few flat state pairs in a thousand put a second dip in the cost at order 2, so the trial count
    // gives this test something to find there.
    // Above order 2 the slope of the cost is a polynomial of degree 6 or more, whose roots come from the
    // companion matrix rather than a closed form.
    for (const auto &[order, trials] : {std::pair{2u, 1000u}, std::pair{3u, 200u}, std::pair{4u, 200u}})
    {
        MinimumEffortSteering steering(order);
        FixedDurationSteering fixed(order, 1.);
        const auto costOver = [&](const Eigen::MatrixXd &from, const Eigen::MatrixXd &to, double duration)
        {
            fixed.setDuration(duration);
            return *fixed.steeringCost(from, to);
        };

        for (unsigned int trial = 0; trial < trials; ++trial)
        {
            const Eigen::MatrixXd from = randomFlatState(engine, DIMENSION, order);
            const Eigen::MatrixXd to = randomFlatState(engine, DIMENSION, order);

            const auto optimal = steering.optimalDuration(from, to);
            BOOST_REQUIRE(optimal.has_value());
            const double duration = optimal->duration;
            const double best = costOver(from, to, duration);
            BOOST_CHECK_CLOSE(optimal->cost, best, 1e-9);

            // The cost can bottom out at more than one duration, so it isn't enough for the slope to vanish
            // at the one that comes back.
            // A sweep of the whole range has to find nothing cheaper.
            const double step = 5e-3;
            for (unsigned int index = 1; index <= 4000u; ++index)
                BOOST_REQUIRE_LE(best, costOver(from, to, static_cast<double>(index) * step) + 1e-9);

            // The cost is flat there, which makes it a minimum rather than an endpoint of the sweep.
            const double slope = (costOver(from, to, duration + 1e-7) - costOver(from, to, duration - 1e-7)) / 2e-7;
            BOOST_CHECK_SMALL(slope, 1e-3);
        }
    }
}

BOOST_AUTO_TEST_CASE(TheCheaperOfTwoLocalMinimaWins)
{
    MinimumEffortSteering steering(ORDER);
    const double rho = steering.getRho();

    // Barely any ground to cover and a lot of speed to pick up along the way.
    // Slamming through the gap right away puts one dip in the cost just after zero, and taking the long
    // way around puts a much deeper one out past three seconds.
    Eigen::MatrixXd from(ORDER, 1), to(ORDER, 1);
    from << 0.8, 1.4;
    to << 0.82, 2.8;

    const auto optimal = steering.optimalDuration(from, to);
    BOOST_REQUIRE(optimal.has_value());
    const double duration = optimal->duration;

    // The early dip is the one a walk out from zero reaches first, and it costs six times what the later
    // one does.
    const double best = costOverDuration(from, to, duration, rho);
    BOOST_CHECK_GT(duration, 1.);
    BOOST_CHECK_GT(costOverDuration(from, to, 0.01, rho), 6. * best);
    for (unsigned int index = 1; index <= 20000u; ++index)
        BOOST_REQUIRE_LE(best, costOverDuration(from, to, static_cast<double>(index) * 1e-3, rho) + 1e-9);
}

BOOST_AUTO_TEST_CASE(BellmanConsistency)
{
    std::mt19937_64 engine(6u);
    MinimumEffortSteering steering(ORDER);

    for (unsigned int trial = 0; trial < 200u; ++trial)
    {
        const Eigen::MatrixXd from = randomFlatState(engine);
        const Eigen::MatrixXd to = randomFlatState(engine);

        const auto whole = steering.steer(from, to);
        BOOST_REQUIRE(whole.has_value());
        const double duration = whole->duration();

        for (double fraction : {0.2, 0.5, 0.8})
        {
            const double split = fraction * duration;
            const Eigen::MatrixXd middle = flatStateAt(*whole, split);

            const double parts = costOverDuration(from, middle, split, steering.getRho()) +
                                 costOverDuration(middle, to, duration - split, steering.getRho());
            BOOST_CHECK_CLOSE(parts, steering.cost(*whole), 1e-6);

            // The tail of an optimal motion is the optimal motion for the pair it joins, duration
            // included, which lets a planner charge edge costs that add up along a path.
            const auto tail = steering.optimalDuration(middle, to);
            BOOST_REQUIRE(tail.has_value());
            BOOST_CHECK_CLOSE(tail->duration, duration - split, 1e-6);
        }
    }
}

BOOST_AUTO_TEST_CASE(DegenerateInputs)
{
    MinimumEffortSteering steering(ORDER);
    const double rho = steering.getRho();

    // Flat states that coincide leave nothing to steer through, so steering reports failure rather than
    // handing back a curve of zero or negative duration.
    Eigen::MatrixXd stationary = Eigen::MatrixXd::Zero(ORDER, DIMENSION);
    stationary.row(0) << 1., -2., 0.5;
    BOOST_CHECK(!steering.steer(stationary, stationary).has_value());
    BOOST_CHECK(!steering.optimalDuration(stationary, stationary).has_value());

    // Coinciding flat outputs carrying the same nonzero velocity need a loop out and back, which costs
    // effort 12 |v|^2 / T and settles at the duration below.
    Eigen::MatrixXd moving = Eigen::MatrixXd::Zero(ORDER, DIMENSION);
    moving.row(0) << 0.25, 0.25, 0.25;
    moving.row(1) << 0.6, -0.8, 0.;
    const auto looping = steering.optimalDuration(moving, moving);
    BOOST_REQUIRE(looping.has_value());
    BOOST_CHECK_CLOSE(looping->duration, moving.row(1).norm() * std::sqrt(12. / rho), 1e-6);

    // Coming to rest at both ends costs effort 12 |offset|^2 / T^3, which settles at the fourth root below.
    Eigen::MatrixXd start = Eigen::MatrixXd::Zero(ORDER, DIMENSION);
    Eigen::MatrixXd finish = Eigen::MatrixXd::Zero(ORDER, DIMENSION);
    finish.row(0) << 3., 4., 0.;
    const auto restToRest = steering.optimalDuration(start, finish);
    BOOST_REQUIRE(restToRest.has_value());
    BOOST_CHECK_CLOSE(restToRest->duration, std::pow(36. * finish.row(0).squaredNorm() / rho, 0.25), 1e-6);

    // A fixed-duration steer between identical resting flat states is the curve that stays put.
    const auto still = FixedDurationSteering(ORDER, 1.5).steer(start, start);
    BOOST_REQUIRE(still.has_value());
    BOOST_CHECK_SMALL(still->coefficients().cwiseAbs().maxCoeff(), 1e-12);
    BOOST_CHECK_SMALL(still->peakSpeed(), 1e-12);
    BOOST_CHECK_CLOSE(still->cost(ORDER, rho), rho * 1.5, 1e-9);
}

BOOST_AUTO_TEST_CASE(PeakSpeedBoundsTheCurve)
{
    std::mt19937_64 engine(7u);
    MinimumEffortSteering steering(ORDER);

    for (unsigned int trial = 0; trial < 200u; ++trial)
    {
        const auto motion = steering.steer(randomFlatState(engine), randomFlatState(engine));
        BOOST_REQUIRE(motion.has_value());

        const FlatMotion velocity = motion->derivative();
        const double peak = motion->peakSpeed();

        const unsigned int samples = 2001u;
        double sampled = 0.;
        for (unsigned int index = 0; index < samples; ++index)
        {
            const double t = motion->duration() * static_cast<double>(index) / (samples - 1.);
            sampled = std::max(sampled, velocity.evaluate(t).norm());
        }

        BOOST_CHECK_LE(sampled, peak * (1. + 1e-9));
        BOOST_CHECK_GE(sampled, peak * (1. - 1e-4));
    }
}
