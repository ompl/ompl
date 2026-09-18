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

#define BOOST_TEST_MODULE "FlatMotion"
#include <boost/test/unit_test.hpp>

#include "ompl/base/spaces/FlatMotion.h"
#include "ompl/util/Exception.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <vector>

using namespace ompl::base;

namespace
{
    constexpr unsigned int ORDER = 2;
    constexpr unsigned int DIMENSION = 3;

    /** \brief Draw a flat state whose flat output lies in a box and whose velocity lies in a smaller one. */
    Eigen::MatrixXd randomFlatState(std::mt19937_64 &engine, unsigned int dimension = DIMENSION)
    {
        std::uniform_real_distribution<double> output(-2., 2.);
        std::uniform_real_distribution<double> velocity(-1., 1.);
        Eigen::MatrixXd state(ORDER, dimension);
        for (unsigned int i = 0; i < dimension; ++i)
        {
            state(0, i) = output(engine);
            state(1, i) = velocity(engine);
        }
        return state;
    }

    /** \brief Read the flat state a motion passes through at time \e t. */
    Eigen::MatrixXd flatStateAt(const FlatMotion &motion, double t)
    {
        Eigen::MatrixXd state(ORDER, motion.outputDimension());
        state.row(0) = motion.evaluate(t).transpose();
        state.row(1) = motion.derivative().evaluate(t).transpose();
        return state;
    }

    /** \brief The cost of steering from \e from to \e to over exactly \e duration. */
    double costOverDuration(const Eigen::MatrixXd &from, const Eigen::MatrixXd &to, double duration, double rho)
    {
        FixedDurationSteering steering(ORDER, duration);
        steering.setRho(rho);
        return steering.cost(*steering.steer(from, to));
    }
}  // namespace

BOOST_AUTO_TEST_CASE(BoundaryConditions)
{
    std::mt19937_64 engine(1u);
    MinimumEffortSteering steering(ORDER);

    for (unsigned int trial = 0; trial < 500u; ++trial)
    {
        const Eigen::MatrixXd from = randomFlatState(engine);
        const Eigen::MatrixXd to = randomFlatState(engine);

        const auto motion = steering.steer(from, to);
        BOOST_REQUIRE(motion.has_value());

        BOOST_CHECK_SMALL((flatStateAt(*motion, 0.) - from).cwiseAbs().maxCoeff(), 1e-9);
        BOOST_CHECK_SMALL((flatStateAt(*motion, motion->duration()) - to).cwiseAbs().maxCoeff(), 1e-9);
    }
}

BOOST_AUTO_TEST_CASE(FixedDurationBoundaryConditions)
{
    std::mt19937_64 engine(2u);

    for (double duration : {0.05, 1., 7.5})
    {
        FixedDurationSteering steering(ORDER, duration);
        BOOST_CHECK_EQUAL(steering.getDuration(), duration);

        for (unsigned int trial = 0; trial < 200u; ++trial)
        {
            const Eigen::MatrixXd from = randomFlatState(engine);
            const Eigen::MatrixXd to = randomFlatState(engine);

            const auto motion = steering.steer(from, to);
            BOOST_REQUIRE(motion.has_value());
            BOOST_CHECK_EQUAL(motion->duration(), duration);
            BOOST_CHECK_EQUAL(motion->degree(), 2u * ORDER - 1u);
            BOOST_CHECK_EQUAL(motion->outputDimension(), DIMENSION);

            BOOST_CHECK_SMALL((flatStateAt(*motion, 0.) - from).cwiseAbs().maxCoeff(), 1e-9);
            BOOST_CHECK_SMALL((flatStateAt(*motion, duration) - to).cwiseAbs().maxCoeff(), 1e-9);
        }
    }
}

BOOST_AUTO_TEST_CASE(DerivativeAndIntegral)
{
    std::mt19937_64 engine(3u);
    MinimumEffortSteering steering(ORDER);

    for (unsigned int trial = 0; trial < 100u; ++trial)
    {
        const auto motion = steering.steer(randomFlatState(engine), randomFlatState(engine));
        BOOST_REQUIRE(motion.has_value());

        const FlatMotion velocity = motion->derivative();
        BOOST_CHECK_EQUAL(velocity.degree(), motion->degree() - 1u);
        BOOST_CHECK_EQUAL(velocity.duration(), motion->duration());

        // A central difference of the curve reproduces its derivative.
        const double step = 1e-5;
        for (double fraction : {0.1, 0.4, 0.9})
        {
            const double t = fraction * motion->duration();
            const Eigen::VectorXd difference = (motion->evaluate(t + step) - motion->evaluate(t - step)) / (2. * step);
            BOOST_CHECK_SMALL((difference - velocity.evaluate(t)).cwiseAbs().maxCoeff(), 1e-6);
        }

        // Integrating and differentiating returns the curve, and the antiderivative starts at zero.
        const FlatMotion antiderivative = motion->integral();
        BOOST_CHECK_EQUAL(antiderivative.degree(), motion->degree() + 1u);
        BOOST_CHECK_SMALL(antiderivative.evaluate(0.).cwiseAbs().maxCoeff(), 1e-12);
        BOOST_CHECK_SMALL((antiderivative.derivative().coefficients() - motion->coefficients()).cwiseAbs().maxCoeff(),
                          1e-9);
    }

    // Differentiating past the degree leaves the zero curve rather than an empty one.
    Eigen::MatrixXd constant(1, DIMENSION);
    constant << 1., 2., 3.;
    FlatMotion flat(constant, 2.);
    BOOST_CHECK_EQUAL(flat.degree(), 0u);
    BOOST_CHECK_SMALL(flat.derivative().evaluate(1.).cwiseAbs().maxCoeff(), 1e-12);
    BOOST_CHECK_EQUAL(flat.derivative().outputDimension(), DIMENSION);
    BOOST_CHECK_SMALL(flat.peakSpeed(), 1e-12);
}

BOOST_AUTO_TEST_CASE(EvaluateIntoABufferMatchesTheDerivatives)
{
    std::mt19937_64 engine(9u);
    MinimumEffortSteering steering(ORDER);

    for (unsigned int trial = 0; trial < 200u; ++trial)
    {
        const auto motion = steering.steer(randomFlatState(engine), randomFlatState(engine));
        BOOST_REQUIRE(motion.has_value());

        // Each level agrees with differentiating the curve that many times and evaluating that.
        FlatMotion level = *motion;
        for (unsigned int derivative = 0; derivative <= motion->degree() + 1u; ++derivative)
        {
            for (double fraction : {0., 0.3, 1.})
            {
                const double t = fraction * motion->duration();
                Eigen::VectorXd buffer(motion->outputDimension());
                motion->evaluate(t, derivative, buffer);
                BOOST_CHECK_SMALL((buffer - level.evaluate(t)).cwiseAbs().maxCoeff(), 1e-9);
            }
            level = level.derivative();
        }

        // Past the degree the curve is flat, so every level above it reads zero.
        Eigen::VectorXd buffer(motion->outputDimension());
        motion->evaluate(0.5 * motion->duration(), motion->degree() + 1u, buffer);
        BOOST_CHECK_SMALL(buffer.cwiseAbs().maxCoeff(), 1e-12);

        // Level 0 is the curve itself, which is the allocating overload.
        motion->evaluate(0.25 * motion->duration(), 0u, buffer);
        BOOST_CHECK_SMALL((buffer - motion->evaluate(0.25 * motion->duration())).cwiseAbs().maxCoeff(), 1e-12);
    }

    // The buffer can be a view over memory the caller already owns, so a state space can write a sample
    // straight into a state.
    Eigen::MatrixXd from = Eigen::MatrixXd::Zero(ORDER, DIMENSION);
    Eigen::MatrixXd to = Eigen::MatrixXd::Zero(ORDER, DIMENSION);
    to.row(0) << 1., 2., 3.;
    const auto motion = steering.steer(from, to);
    BOOST_REQUIRE(motion.has_value());

    std::vector<double> owned(DIMENSION, -1.);
    Eigen::Map<Eigen::VectorXd> view(owned.data(), DIMENSION);
    motion->evaluate(motion->duration(), 0u, view);
    for (unsigned int axis = 0; axis < DIMENSION; ++axis)
        BOOST_CHECK_CLOSE(owned[axis], to(0, axis), 1e-6);
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

BOOST_AUTO_TEST_CASE(OptimalDurationIsTheCheapestDuration)
{
    std::mt19937_64 engine(5u);
    MinimumEffortSteering steering(ORDER);
    const double rho = steering.getRho();

    // Only a few flat state pairs in a thousand put a second dip in the cost, so the trial count gives this test
    // something to find.
    for (unsigned int trial = 0; trial < 1000u; ++trial)
    {
        const Eigen::MatrixXd from = randomFlatState(engine);
        const Eigen::MatrixXd to = randomFlatState(engine);

        const auto duration = steering.optimalDuration(from, to);
        BOOST_REQUIRE(duration.has_value());
        const double best = costOverDuration(from, to, *duration, rho);

        // The cost has up to two durations where it bottoms out, so it isn't enough for the slope to
        // vanish at the one that comes back.
        // A sweep of the whole range has to find nothing cheaper.
        const double step = 5e-3;
        for (unsigned int index = 1; index <= 4000u; ++index)
            BOOST_REQUIRE_LE(best, costOverDuration(from, to, static_cast<double>(index) * step, rho) + 1e-9);

        // The cost is flat there, which makes it a minimum rather than an endpoint of the sweep.
        const double slope =
            (costOverDuration(from, to, *duration + 1e-7, rho) - costOverDuration(from, to, *duration - 1e-7, rho)) /
            2e-7;
        BOOST_CHECK_SMALL(slope, 1e-3);
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

    const auto duration = steering.optimalDuration(from, to);
    BOOST_REQUIRE(duration.has_value());

    // The early dip is the one a walk out from zero reaches first, and it costs six times what the later
    // one does.
    const double best = costOverDuration(from, to, *duration, rho);
    BOOST_CHECK_GT(*duration, 1.);
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
            BOOST_CHECK_CLOSE(*tail, duration - split, 1e-6);
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
    BOOST_CHECK_CLOSE(*looping, moving.row(1).norm() * std::sqrt(12. / rho), 1e-6);

    // Coming to rest at both ends costs effort 12 |offset|^2 / T^3, which settles at the fourth root below.
    Eigen::MatrixXd start = Eigen::MatrixXd::Zero(ORDER, DIMENSION);
    Eigen::MatrixXd finish = Eigen::MatrixXd::Zero(ORDER, DIMENSION);
    finish.row(0) << 3., 4., 0.;
    const auto restToRest = steering.optimalDuration(start, finish);
    BOOST_REQUIRE(restToRest.has_value());
    BOOST_CHECK_CLOSE(*restToRest, std::pow(36. * finish.row(0).squaredNorm() / rho, 0.25), 1e-6);

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

BOOST_AUTO_TEST_CASE(RejectedArguments)
{
    BOOST_CHECK_THROW(MinimumEffortSteering(1u), ompl::Exception);
    BOOST_CHECK_THROW(MinimumEffortSteering(3u), ompl::Exception);
    BOOST_CHECK_THROW(FixedDurationSteering(ORDER, 0.), ompl::Exception);
    BOOST_CHECK_THROW(FixedDurationSteering(ORDER, -1.), ompl::Exception);

    MinimumEffortSteering steering(ORDER);
    BOOST_CHECK_EQUAL(steering.getOrder(), ORDER);
    BOOST_CHECK_EQUAL(steering.getRho(), 5.);
    BOOST_CHECK_THROW(steering.setRho(0.), ompl::Exception);
    BOOST_CHECK_THROW(steering.setRho(-2.), ompl::Exception);
    steering.setRho(20.);
    BOOST_CHECK_EQUAL(steering.getRho(), 20.);

    const Eigen::MatrixXd good = Eigen::MatrixXd::Identity(ORDER, DIMENSION);
    BOOST_CHECK_THROW(steering.steer(Eigen::MatrixXd::Zero(ORDER + 1u, DIMENSION), good), ompl::Exception);
    BOOST_CHECK_THROW(steering.steer(good, Eigen::MatrixXd::Zero(ORDER, DIMENSION + 1u)), ompl::Exception);

    BOOST_CHECK_THROW(FlatMotion(Eigen::MatrixXd::Zero(2, DIMENSION), 0.), ompl::Exception);
    BOOST_CHECK_THROW(FlatMotion(Eigen::MatrixXd::Zero(2, DIMENSION), -1.), ompl::Exception);
    BOOST_CHECK_THROW(FlatMotion(Eigen::MatrixXd::Zero(0, 0), 1.), ompl::Exception);
}

BOOST_AUTO_TEST_CASE(RaisingRhoShortensTheTrajectory)
{
    std::mt19937_64 engine(8u);
    MinimumEffortSteering slow(ORDER);
    MinimumEffortSteering quick(ORDER);
    quick.setRho(20.);

    for (unsigned int trial = 0; trial < 200u; ++trial)
    {
        const Eigen::MatrixXd from = randomFlatState(engine);
        const Eigen::MatrixXd to = randomFlatState(engine);

        const auto patient = slow.optimalDuration(from, to);
        const auto hurried = quick.optimalDuration(from, to);
        BOOST_REQUIRE(patient.has_value());
        BOOST_REQUIRE(hurried.has_value());
        BOOST_CHECK_LT(*hurried, *patient);
    }
}

BOOST_AUTO_TEST_CASE(MatchesFlaskReferenceData)
{
    // The FLASK implementation produces the reference data in single precision.
    // Its columns and the program that wrote it are described in
    // tests/resources/generate_flat_steering_reference.cpp.
    std::filesystem::path path(TEST_RESOURCES_DIR);
    std::ifstream file(path / "flat_steering_reference.txt");
    BOOST_REQUIRE(file.good());

    std::stringstream numbers;
    for (std::string line; std::getline(file, line);)
        if (line.empty() || line[0] != '#')
            numbers << line << ' ';

    std::string key;
    unsigned int dimension, samples, cases;
    double rho;
    numbers >> key >> dimension >> key >> rho >> key >> samples >> key >> cases;
    BOOST_REQUIRE_GT(cases, 0u);

    MinimumEffortSteering steering(ORDER);
    steering.setRho(rho);

    for (unsigned int index = 0; index < cases; ++index)
    {
        Eigen::MatrixXd from(ORDER, dimension);
        Eigen::MatrixXd to(ORDER, dimension);
        for (unsigned int level = 0; level < ORDER; ++level)
            for (unsigned int axis = 0; axis < dimension; ++axis)
                numbers >> from(level, axis);
        for (unsigned int level = 0; level < ORDER; ++level)
            for (unsigned int axis = 0; axis < dimension; ++axis)
                numbers >> to(level, axis);

        double success, referenceDuration, referenceCost;
        numbers >> success >> referenceDuration >> referenceCost;

        std::vector<double> referenceSamples(samples * dimension);
        for (double &value : referenceSamples)
            numbers >> value;
        BOOST_REQUIRE(!numbers.fail());

        const bool coincident = from.row(0) == to.row(0);
        if (success < 0.5)
        {
            // FLASK only ever comes up empty where its quartic degenerates, where the flat
            // outputs already coincide.
            BOOST_CHECK(coincident);
            continue;
        }

        // Over the duration FLASK picked, the curve itself has to agree to single precision, which pins
        // the coefficients and the effort integral against the reference.
        FixedDurationSteering fixed(ORDER, referenceDuration);
        fixed.setRho(rho);
        const auto reference = fixed.steer(from, to);
        BOOST_REQUIRE(reference.has_value());

        for (unsigned int sample = 1; sample <= samples; ++sample)
        {
            const double t = referenceDuration * static_cast<double>(sample) / (samples + 1.);
            const Eigen::VectorXd value = reference->evaluate(t);
            for (unsigned int axis = 0; axis < dimension; ++axis)
                BOOST_CHECK_SMALL(value[axis] - referenceSamples[(sample - 1) * dimension + axis], 1e-5);
        }

        const double referenceCurveCost = fixed.cost(*reference);
        BOOST_CHECK_SMALL((referenceCurveCost - referenceCost) / std::max(1., referenceCost), 1e-5);

        // We have higher precision durations, so the implementation can differ from here on.
        // Our duration never costs more than FLASK's.
        const auto motion = steering.steer(from, to);
        BOOST_REQUIRE(motion.has_value());
        const double cost = steering.cost(*motion);
        BOOST_CHECK_LE(cost, referenceCurveCost * (1. + 1e-9) + 1e-9);

        // Away from the degenerate case the two durations also cost within a few percent of each other,
        // which rules out a different time penalty or a different effort integral.
        if (!coincident)
            BOOST_CHECK_LE(referenceCurveCost - cost, 0.05 * std::max(1., cost));
    }
}
