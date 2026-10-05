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

#define BOOST_TEST_MODULE "FlatVampMotionValidator"
#include <boost/test/unit_test.hpp>

#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/flat/FlatMotionValidator.h>
#include <ompl/base/spaces/flat/FlatStateSpace.h>
#include <ompl/util/RandomNumbers.h>

#include <ompl/vamp/FlatVampMotionValidator.h>
#include <ompl/vamp/VampStateSpace.h>
#include <ompl/vamp/VampStateValidityChecker.h>

#include <vamp/collision/factory.hh>
#include <vamp/robots/panda.hh>

#include <array>
#include <cmath>
#include <memory>
#include <vector>

namespace ob = ompl::base;
namespace ov = ompl::vamp;

using Robot = vamp::robots::Panda;
using Environment = vamp::collision::Environment<vamp::FloatVector<vamp::FloatVectorWidth>>;
using Validator = ov::FlatVampMotionValidator<Robot>;
using Configuration = std::array<float, Robot::dimension>;

namespace
{
    constexpr std::size_t RAKE = vamp::FloatVectorWidth;

    /** \brief The sphere cage from the manipulator demo. */
    Environment sphereCage()
    {
        vamp::collision::Environment<float> environment;
        const std::vector<std::array<float, 3>> centers = {
            {0.55, 0, 0.25},  {0.35, 0.35, 0.25},  {0, 0.55, 0.25},   {-0.55, 0, 0.25},   {-0.35, -0.35, 0.25},
            {0, -0.55, 0.25}, {0.35, -0.35, 0.25}, {0.35, 0.35, 0.8}, {0, 0.55, 0.8},     {-0.35, 0.35, 0.8},
            {-0.55, 0, 0.8},  {-0.35, -0.35, 0.8}, {0, -0.55, 0.8},   {0.35, -0.35, 0.8},
        };
        for (const auto &center : centers)
            environment.spheres.emplace_back(vamp::collision::factory::sphere::array(center, 0.2f));
        environment.sort();
        return Environment(environment);
    }

    /** \brief A Panda flat space whose derivative level \e l sits inside plus and minus \e bound to the
        \e l. */
    std::shared_ptr<ob::FlatStateSpace> pandaSpace(unsigned int order, double bound)
    {
        auto space = std::make_shared<ob::FlatStateSpace>(std::make_shared<ov::VampStateSpace<Robot>>(), order);
        for (unsigned int level = 1; level < order; ++level)
            space->setDerivativeBound(level, std::pow(bound, level));
        return space;
    }

    /** \brief Space information whose validity check holds a state to its bounds and keeps the arm out of
        \e environment, which is the check ompl::vamp::FlatVampMotionValidator makes. */
    ob::SpaceInformationPtr pandaInformation(const std::shared_ptr<ob::FlatStateSpace> &space,
                                             const Environment &environment)
    {
        auto si = std::make_shared<ob::SpaceInformation>(space);
        auto outputInformation = std::make_shared<ob::SpaceInformation>(space->getOutputSpace());
        auto collision = std::make_shared<ov::VampStateValidityChecker<Robot>>(outputInformation, environment);
        si->setStateValidityChecker(
            [space, collision](const ob::State *state)
            {
                return space->satisfiesBounds(state) &&
                       collision->isValid(state->as<ob::FlatStateSpace::StateType>()->output());
            });
        si->setup();
        return si;
    }

    /** \brief The joint values of \e state narrowed to single precision. */
    Configuration joints(const ob::State *state)
    {
        const double *values =
            state->as<ob::FlatStateSpace::StateType>()->output()->as<ob::RealVectorStateSpace::StateType>()->values;
        Configuration out{};
        for (std::size_t i = 0; i < Robot::dimension; ++i)
            out[i] = static_cast<float>(values[i]);
        return out;
    }

    /** \brief Whether some configuration in \e among lies within \e tolerance of \e configuration in
        every joint. */
    bool contains(const std::vector<Configuration> &among, const Configuration &configuration, float tolerance)
    {
        for (const Configuration &other : among)
        {
            bool near = true;
            for (std::size_t i = 0; i < Robot::dimension && near; ++i)
                near = std::abs(other[i] - configuration[i]) <= tolerance;
            if (near)
                return true;
        }
        return false;
    }

    /** \brief A batched validator recording every configuration it would check for collisions, and
        passing all of them, since a random configuration can collide with the arm itself. */
    class RecordingValidator : public Validator
    {
    public:
        using Validator::Validator;

        mutable std::vector<Configuration> checked;

    protected:
        bool checkCollisions(const Block &block) const override
        {
            std::array<std::array<float, RAKE>, Robot::dimension> rows{};
            for (std::size_t joint = 0; joint < Robot::dimension; ++joint)
            {
                const auto values = block[joint].to_array();
                std::copy(values.begin(), values.begin() + RAKE, rows[joint].begin());
            }
            for (std::size_t lane = 0; lane < RAKE; ++lane)
            {
                Configuration configuration{};
                for (std::size_t joint = 0; joint < Robot::dimension; ++joint)
                    configuration[joint] = rows[joint][lane];
                checked.push_back(configuration);
            }
            return true;
        }
    };
}  // namespace

BOOST_AUTO_TEST_CASE(BlockEvaluationMatchesTheMotion)
{
    ompl::RNG rng(7u);
    using Motion = ov::FlatVampMotion<Robot::dimension, RAKE>;

    for (unsigned int order = 1; order <= 3; ++order)
    {
        const ob::MinimumEffortSteering steering(order);
        for (unsigned int trial = 0; trial < 200u; ++trial)
        {
            Eigen::MatrixXd from(order, Robot::dimension), to(order, Robot::dimension);
            for (unsigned int level = 0; level < order; ++level)
                for (std::size_t axis = 0; axis < Robot::dimension; ++axis)
                {
                    from(level, axis) = level == 0u ? 0. : rng.uniformReal(-2., 2.);
                    to(level, axis) = rng.uniformReal(-3., 3.);
                }

            const std::optional<ob::FlatMotion> motion = steering.steer(from, to);
            BOOST_REQUIRE(motion.has_value());
            const Motion block(*motion, order);

            // The lanes span the whole motion, from its start through its duration.
            alignas(Motion::Vector::S::Alignment) std::array<float, RAKE> times{};
            for (std::size_t lane = 0; lane < RAKE; ++lane)
                times[lane] = static_cast<float>(motion->duration() * lane / (RAKE - 1u));
            const Motion::Vector time(times.data());

            Motion::Block values;
            Eigen::VectorXd expected(Robot::dimension);
            for (unsigned int level = 0; level < order; ++level)
            {
                block.evaluate(time, level, values);
                for (std::size_t axis = 0; axis < Robot::dimension; ++axis)
                {
                    const auto lanes = values[axis].to_array();
                    for (std::size_t lane = 0; lane < RAKE; ++lane)
                    {
                        motion->evaluate(times[lane], level, expected);
                        BOOST_REQUIRE_SMALL(lanes[lane] - expected[axis], 1e-4 * (1. + std::abs(expected[axis])));
                    }
                }
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(SamplesSpreadEvenlyOverTheMotion)
{
    // Nothing gets rejected and the bounds sit far out, so the validator walks every sample of every edge.
    const Environment environment = sphereCage();
    auto space = pandaSpace(2u, 100.);
    space->getOutputSpace()->as<ob::RealVectorStateSpace>()->setBounds(-10., 10.);
    ompl::RNG rng(11u);
    Eigen::MatrixXd flat(2, Robot::dimension);
    const auto sample = [&](ob::State *state)
    {
        for (Eigen::Index i = 0; i < flat.size(); ++i)
            flat(i) = rng.uniformReal(-2., 2.);
        space->fromFlatState(flat, state);
    };
    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker([](const ob::State *) { return true; });

    ob::ScopedState<> from(space), to(space), between(space);

    // The coarsest spacing puts one segment on an edge, the next fewer interior samples than a batch has
    // lanes, and the finest many batches.
    bool sawFew = false, sawMany = false;
    for (double fraction : {0.9, 0.1, 0.01, 0.002})
    {
        si->setStateValidityCheckingResolution(fraction);
        si->setup();
        const RecordingValidator batched(si, environment);

        for (unsigned int trial = 0; trial < 30u; ++trial)
        {
            sample(from.get());
            sample(to.get());
            const std::optional<ob::FlatMotion> motion = space->steer(from.get(), to.get());
            BOOST_REQUIRE(motion.has_value());
            const unsigned int count = space->validSegmentCount(*motion);
            sawFew = sawFew || (count > 1u && count <= RAKE);
            sawMany = sawMany || count > 4u * RAKE;

            batched.checked.clear();
            BOOST_REQUIRE(batched.checkMotion(from.get(), to.get()));

            // The far end gets checked first, alone in every lane.
            const Configuration end = joints(to.get());
            BOOST_REQUIRE_GE(batched.checked.size(), RAKE);
            for (std::size_t lane = 0; lane < RAKE; ++lane)
                BOOST_REQUIRE(batched.checked[lane] == end);

            // The interior rounds up to whole batches, spaced evenly and at least as closely as the count
            // asks for.
            const std::size_t batches = (count - 1u + RAKE - 1u) / RAKE;
            const std::size_t interior = RAKE * batches;
            BOOST_REQUIRE_EQUAL(batched.checked.size(), RAKE + interior);
            std::vector<Configuration> expected;
            for (std::size_t i = 1; i <= interior; ++i)
            {
                space->interpolate(from.get(), *motion, static_cast<double>(i) / (interior + 1u), between.get());
                expected.push_back(joints(between.get()));
            }

            // Both sets hold the same number of samples and each holds the other, and the expected
            // samples are distinct, so no lane repeats a sample.
            const std::vector<Configuration> lanes(batched.checked.begin() + RAKE, batched.checked.end());
            for (const Configuration &configuration : expected)
                BOOST_REQUIRE(contains(lanes, configuration, 1e-4f));
            for (const Configuration &configuration : lanes)
                BOOST_REQUIRE(contains(expected, configuration, 1e-4f));
        }
    }
    BOOST_CHECK(sawFew);
    BOOST_CHECK(sawMany);
}

BOOST_AUTO_TEST_CASE(VerdictsMatchTheGenericValidator)
{
    const Environment environment = sphereCage();
    for (unsigned int order = 1; order <= 3; ++order)
    {
        auto space = pandaSpace(order, 1.);
        auto si = pandaInformation(space, environment);
        const ob::FlatMotionValidator generic(si);
        const Validator batched(si, environment);

        ob::StateSamplerPtr sampler = space->allocStateSampler();
        ob::ScopedState<> from(space), to(space), last(space);

        unsigned int rejected = 0u, disagreements = 0u;
        constexpr unsigned int TRIALS = 1000u;
        for (unsigned int trial = 0; trial < TRIALS; ++trial)
        {
            do
                sampler->sampleUniform(from.get());
            while (!si->isValid(from.get()));

            // Short edges mix verdicts, since most long ones hit a sphere.
            sampler->sampleUniform(to.get());
            space->interpolate(from.get(), to.get(), 0.05, to.get());

            const bool expected = generic.checkMotion(from.get(), to.get());
            const bool verdict = batched.checkMotion(from.get(), to.get());
            if (verdict != expected)
                ++disagreements;

            // Walking in order checks the same samples as striding, so both overloads agree exactly.
            std::pair<ob::State *, double> lastValid(last.get(), -1.);
            BOOST_REQUIRE_EQUAL(batched.checkMotion(from.get(), to.get(), lastValid), verdict);
            if (expected)
                continue;

            ++rejected;
            if (verdict)
                continue;

            // The last valid state is a sample that passed, somewhere short of the far end.
            BOOST_REQUIRE_GE(lastValid.second, 0.);
            BOOST_REQUIRE_LT(lastValid.second, 1.);
            BOOST_REQUIRE(si->isValid(last.get()));
        }

        BOOST_TEST_MESSAGE("order " << order << ": " << rejected << " rejected, " << disagreements << " disagreements");
        BOOST_CHECK_GT(rejected, TRIALS / 10u);
        BOOST_CHECK_LT(rejected, TRIALS - TRIALS / 10u);
        BOOST_CHECK_LE(disagreements, TRIALS / 200u);
    }
}
