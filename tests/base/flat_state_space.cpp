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

/* Author: Clayton W. Ramsey */

#define BOOST_TEST_MODULE "FlatStateSpace"
#include <boost/test/unit_test.hpp>

#include "ompl/base/ScopedState.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/FlatStateSpace.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/geometric/planners/rrt/RRT.h"

#include <cmath>
#include <limits>
#include <optional>
#include <thread>
#include <vector>

using namespace ompl::base;
namespace og = ompl::geometric;

namespace
{
    constexpr unsigned int ORDER = 2;
    constexpr unsigned int DIMENSION = 2;

    /** \brief A flat space over a bounded Euclidean output with a box bound on the velocity. */
    std::shared_ptr<FlatStateSpace> makeSpace()
    {
        auto output = std::make_shared<RealVectorStateSpace>(DIMENSION);
        output->setBounds(-1., 1.);
        auto space = std::make_shared<FlatStateSpace>(output, ORDER);
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

}  // namespace

BOOST_AUTO_TEST_CASE(SanityChecksPassUnderTrajectoryCost)
{
    auto space = makeSpace();

    // The stock battery covers the metric axioms, interpolation at either end, continued interpolation,
    // and interpolating through overlapping memory, so the flat suite doesn't repeat any of that.
    BOOST_CHECK_EQUAL(space->getDistanceType(), FlatStateSpace::FLAT_STATE_METRIC);
    BOOST_CHECK(space->isMetricSpace());
    BOOST_CHECK(space->hasSymmetricDistance());
    BOOST_CHECK_NO_THROW(space->sanityChecks());

    // Charging the steering cost gives up the metric, and the flags have to come off with it.
    space->setDistanceType(FlatStateSpace::TRAJECTORY_COST);
    space->setup();
    BOOST_CHECK(!space->isMetricSpace());
    BOOST_CHECK(!space->hasSymmetricDistance());
    BOOST_CHECK_NO_THROW(space->sanityChecks());

    ScopedState<> from(space), to(space);
    setState(space.get(), from.get(), {-0.5, 0.1}, {0.2, -0.3});
    setState(space.get(), to.get(), {0.6, -0.2}, {-0.1, 0.4});
    BOOST_CHECK_CLOSE(space->distance(from.get(), to.get()), space->steeringCost(from.get(), to.get()), 1e-9);

    // A flat state carrying a velocity has a polynomial that loops back around to it, and nobody has to
    // fly that loop to be where they already are.
    BOOST_CHECK_EQUAL(space->distance(from.get(), from.get()), 0.);

    // Edges run forward in time, so the two directions cost different amounts.
    BOOST_CHECK_GT(std::abs(space->distance(from.get(), to.get()) - space->distance(to.get(), from.get())), 1e-6);
}

BOOST_AUTO_TEST_CASE(RecycledStateMemoryGivesFreshMotions)
{
    auto space = makeSpace();

    // Solve an edge, then hand the memory back and take it again.
    State *from = space->allocState();
    State *to = space->allocState();
    State *result = space->allocState();

    setState(space.get(), from, {-0.8, -0.8}, {0., 0.});
    setState(space.get(), to, {0.8, 0.8}, {0., 0.});
    space->interpolate(from, to, 0.5, result);
    const double firstX =
        result->as<FlatStateSpace::StateType>()->output()->as<RealVectorStateSpace::StateType>()->values[0];

    space->freeState(from);
    space->freeState(to);

    State *recycledFrom = space->allocState();
    State *recycledTo = space->allocState();
    setState(space.get(), recycledFrom, {0.8, -0.8}, {0., 0.});
    setState(space.get(), recycledTo, {-0.8, 0.8}, {0., 0.});
    space->interpolate(recycledFrom, recycledTo, 0.5, result);
    const double secondX =
        result->as<FlatStateSpace::StateType>()->output()->as<RealVectorStateSpace::StateType>()->values[0];

    // The two edges run opposite ways along x, so stale state would report the first answer twice.
    BOOST_CHECK_SMALL(firstX, 1e-9);
    BOOST_CHECK_SMALL(secondX, 1e-9);

    // A sharper version, where the midpoints differ.
    setState(space.get(), recycledFrom, {-0.9, 0.}, {0., 0.});
    setState(space.get(), recycledTo, {0.5, 0.}, {0., 0.});
    space->interpolate(recycledFrom, recycledTo, 0.5, result);
    const double thirdX =
        result->as<FlatStateSpace::StateType>()->output()->as<RealVectorStateSpace::StateType>()->values[0];
    BOOST_CHECK_CLOSE(thirdX, -0.2, 1e-6);

    space->freeState(recycledFrom);
    space->freeState(recycledTo);
    space->freeState(result);
}

BOOST_AUTO_TEST_CASE(CallerHeldMotionMatchesTheStatelessPath)
{
    auto space = makeSpace();
    StateSamplerPtr sampler = space->allocStateSampler();

    ScopedState<> from(space), to(space), plain(space), held(space), aliased(space);
    for (unsigned int trial = 0; trial < 200u; ++trial)
    {
        sampler->sampleUniform(from.get());
        sampler->sampleUniform(to.get());

        // Walking the edge while holding the motion gives the same states as solving every time.
        bool firstTime = true;
        std::optional<FlatMotion> motion;
        for (double t : {0., 0.25, 0.5, 0.75, 1.})
        {
            space->interpolate(from.get(), to.get(), t, plain.get());
            space->interpolate(from.get(), to.get(), t, firstTime, motion, held.get());
            BOOST_CHECK(space->equalStates(plain.get(), held.get()));
        }

        // The first interior sample solves the steering and leaves it in the caller's hands.
        BOOST_CHECK(!firstTime);
        BOOST_REQUIRE(motion.has_value());

        // The segment count agrees whether it comes from the endpoints or from the motion already held.
        BOOST_CHECK_EQUAL(space->validSegmentCount(from.get(), to.get()), space->validSegmentCount(*motion));

        // Following the motion by itself matches following the endpoints.
        space->interpolate(*motion, 0.4, held.get());
        space->interpolate(from.get(), to.get(), 0.4, plain.get());
        BOOST_CHECK(space->equalStates(plain.get(), held.get()));

        bool aliasedFirstTime = true;
        std::optional<FlatMotion> aliasedMotion;
        space->copyState(aliased.get(), to.get());
        space->interpolate(from.get(), aliased.get(), 0.5, aliasedFirstTime, aliasedMotion, aliased.get());
        space->interpolate(from.get(), to.get(), 0.5, plain.get());
        BOOST_CHECK(space->equalStates(aliased.get(), plain.get()));
    }
}

BOOST_AUTO_TEST_CASE(InterpolationIsThreadSafe)
{
    auto space = makeSpace();

    std::vector<std::thread> workers;
    std::vector<int> failures(4, 0);
    for (unsigned int worker = 0; worker < 4u; ++worker)
        workers.emplace_back(
            [&space, &failures, worker]
            {
                StateSamplerPtr sampler = space->allocStateSampler();
                State *from = space->allocState();
                State *to = space->allocState();
                State *result = space->allocState();
                for (unsigned int trial = 0; trial < 2000u; ++trial)
                {
                    sampler->sampleUniform(from);
                    sampler->sampleUniform(to);
                    space->interpolate(from, to, 0., result);
                    if (!space->equalStates(result, from))
                        ++failures[worker];
                    space->interpolate(from, to, 1., result);
                    if (!space->equalStates(result, to))
                        ++failures[worker];
                    space->interpolate(from, to, 0.5, result);
                    if (space->validSegmentCount(from, to) < 1u)
                        ++failures[worker];
                }
                space->freeState(from);
                space->freeState(to);
                space->freeState(result);
            });

    for (std::thread &thread : workers)
        thread.join();
    for (int count : failures)
        BOOST_CHECK_EQUAL(count, 0);
}

BOOST_AUTO_TEST_CASE(CheckSpacingTracksTheSegmentFraction)
{
    auto space = makeSpace();

    ScopedState<> from(space), to(space);
    setState(space.get(), from.get(), {-0.9, -0.9}, {0., 0.});
    setState(space.get(), to.get(), {0.9, 0.9}, {0., 0.});

    space->setLongestValidSegmentFraction(0.02);
    space->setup();
    const unsigned int coarse = space->validSegmentCount(from.get(), to.get());

    space->setLongestValidSegmentFraction(0.01);
    space->setup();
    const unsigned int fine = space->validSegmentCount(from.get(), to.get());

    BOOST_CHECK_GT(coarse, 1u);
    BOOST_CHECK_CLOSE(static_cast<double>(fine), 2. * static_cast<double>(coarse), 5.);

    // The count is the distance the flat output covers at its fastest over the longest valid segment.
    const auto motion = space->steer(from.get(), to.get());
    BOOST_REQUIRE(motion.has_value());
    const double expected = std::ceil(motion->duration() * motion->peakSpeed() / space->getLongestValidSegmentLength());
    BOOST_CHECK_CLOSE(static_cast<double>(fine), expected, 1e-6);

    // A faster edge needs more checks than a slower one of the same length.
    ScopedState<> quick(space), slow(space);
    setState(space.get(), quick.get(), {-0.9, 0.}, {2., 0.});
    setState(space.get(), slow.get(), {-0.9, 0.}, {0., 0.});
    ScopedState<> target(space);
    setState(space.get(), target.get(), {0.9, 0.}, {0., 0.});
    BOOST_CHECK_GT(space->validSegmentCount(quick.get(), target.get()),
                   space->validSegmentCount(slow.get(), target.get()));
}

namespace
{
    /** \brief A derivative component bounding the norm of the whole vector rather than each axis.

        It overrides the three places a hard constraint has to reach, so the space treats the ball as a
        bound rather than as a preference the sampler happens to follow.
    */
    class BallDerivativeSpace : public RealVectorStateSpace
    {
    public:
        BallDerivativeSpace(unsigned int dimension, double radius) : RealVectorStateSpace(dimension), radius_(radius)
        {
            setBounds(-radius, radius);
        }

        bool satisfiesBounds(const State *state) const override
        {
            return norm(state) <= radius_;
        }

        void enforceBounds(State *state) const override
        {
            const double length = norm(state);
            if (length > radius_)
            {
                // Aiming at exactly the radius rounds the wrong way often enough to matter, so this aims
                // a few of the last bits inside and satisfiesBounds above can compare exactly.
                const double target = radius_ * (1. - 4. * std::numeric_limits<double>::epsilon());
                double *values = state->as<StateType>()->values;
                for (unsigned int i = 0; i < getDimension(); ++i)
                    values[i] *= target / length;
            }
        }

        StateSamplerPtr allocDefaultStateSampler() const override;

        double norm(const State *state) const
        {
            const double *values = state->as<StateType>()->values;
            double total = 0.;
            for (unsigned int i = 0; i < getDimension(); ++i)
                total += values[i] * values[i];
            return std::sqrt(total);
        }

    private:
        double radius_;
    };

    /** \brief Draws from the ball by rejecting draws from the box around it. */
    class BallSampler : public StateSampler
    {
    public:
        explicit BallSampler(const BallDerivativeSpace *space) : StateSampler(space), ball_(space)
        {
        }

        void sampleUniform(State *state) override
        {
            double *values = state->as<RealVectorStateSpace::StateType>()->values;
            const RealVectorBounds &bounds = ball_->getBounds();
            do
            {
                for (unsigned int i = 0; i < ball_->getDimension(); ++i)
                    values[i] = rng_.uniformReal(bounds.low[i], bounds.high[i]);
            } while (!ball_->satisfiesBounds(state));
        }

        void sampleUniformNear(State *state, const State *near, double distance) override
        {
            space_->copyState(state, near);
            double *values = state->as<RealVectorStateSpace::StateType>()->values;
            for (unsigned int i = 0; i < ball_->getDimension(); ++i)
                values[i] += rng_.uniformReal(-distance, distance);
            ball_->enforceBounds(state);
        }

        void sampleGaussian(State *state, const State *mean, double stdDev) override
        {
            space_->copyState(state, mean);
            double *values = state->as<RealVectorStateSpace::StateType>()->values;
            for (unsigned int i = 0; i < ball_->getDimension(); ++i)
                values[i] += rng_.gaussian(0., stdDev);
            ball_->enforceBounds(state);
        }

    private:
        const BallDerivativeSpace *ball_;
    };

    StateSamplerPtr BallDerivativeSpace::allocDefaultStateSampler() const
    {
        return std::make_shared<BallSampler>(this);
    }
}  // namespace

BOOST_AUTO_TEST_CASE(SubstitutedDerivativeSpaceIsHonored)
{
    auto output = std::make_shared<RealVectorStateSpace>(DIMENSION);
    output->setBounds(-1., 1.);

    const double radius = 1.5;
    std::vector<StateSpacePtr> derivatives{std::make_shared<BallDerivativeSpace>(DIMENSION, radius)};
    auto space = std::make_shared<FlatStateSpace>(output, derivatives);
    space->setup();

    auto *ball = dynamic_cast<const BallDerivativeSpace *>(space->getDerivativeSpace(1));
    BOOST_REQUIRE(ball != nullptr);

    ScopedState<> state(space);

    // The corner of the box around the ball sits outside the ball, and the compound says so.
    setState(space.get(), state.get(), {0., 0.}, {1.4, 1.4});
    BOOST_CHECK(!space->satisfiesBounds(state.get()));
    space->enforceBounds(state.get());
    BOOST_CHECK(space->satisfiesBounds(state.get()));
    BOOST_CHECK_CLOSE(ball->norm(state->as<CompoundState>()->components[1]), radius, 1e-6);

    // The flat output is untouched by any of that.
    const double *values =
        state->as<FlatStateSpace::StateType>()->output()->as<RealVectorStateSpace::StateType>()->values;
    BOOST_CHECK_SMALL(values[0], 1e-12);
    BOOST_CHECK_SMALL(values[1], 1e-12);

    // The component's sampler reaches the compound through delegation, so every draw lands in the ball.
    StateSamplerPtr sampler = space->allocStateSampler();
    for (unsigned int trial = 0; trial < 2000u; ++trial)
    {
        sampler->sampleUniform(state.get());
        BOOST_REQUIRE(space->satisfiesBounds(state.get()));
    }

    // Whatever enforceBounds leaves behind, satisfiesBounds has to accept.
    // Scaling a vector to a given length rounds either way, so a ball has to aim inside where a box can
    // assign its bound exactly.
    ompl::RNG rng;
    for (unsigned int trial = 0; trial < 5000u; ++trial)
    {
        double *velocity = state->as<CompoundState>()->components[1]->as<RealVectorStateSpace::StateType>()->values;
        for (unsigned int axis = 0; axis < DIMENSION; ++axis)
            velocity[axis] = rng.uniformReal(-4. * radius, 4. * radius);

        space->enforceBounds(state.get());
        BOOST_REQUIRE(space->satisfiesBounds(state.get()));
    }
}

namespace
{
    /** \brief Rejects flat outputs inside an axis-aligned box, ignoring the derivatives. */
    class BoxObstacle : public StateValidityChecker
    {
    public:
        BoxObstacle(const SpaceInformationPtr &si, double half) : StateValidityChecker(si), half_(half)
        {
        }

        bool isValid(const State *state) const override
        {
            const double *values =
                state->as<FlatStateSpace::StateType>()->output()->as<RealVectorStateSpace::StateType>()->values;
            return std::abs(values[0]) > half_ || std::abs(values[1]) > half_;
        }

    private:
        double half_;
    };
}  // namespace

BOOST_AUTO_TEST_CASE(RRTSolvesAndThePathChecks)
{
    auto space = makeSpace();

    // KPIECE1 and its relatives refuse to run without a projection.
    // Use flat output as the default projection.
    BOOST_REQUIRE(space->hasDefaultProjection());
    BOOST_CHECK_EQUAL(space->getDefaultProjection()->getDimension(), space->getOutputSpace()->getDimension());

    og::SimpleSetup setup(space);
    setup.setStateValidityChecker(std::make_shared<BoxObstacle>(setup.getSpaceInformation(), 0.3));

    ScopedState<> start(space), goal(space);
    setState(space.get(), start.get(), {-0.8, -0.8}, {0., 0.});
    setState(space.get(), goal.get(), {0.8, 0.8}, {0., 0.});
    setup.setStartAndGoalStates(start, goal, 0.1);
    setup.setPlanner(std::make_shared<og::RRT>(setup.getSpaceInformation()));

    BOOST_REQUIRE(setup.solve(10.) == PlannerStatus::EXACT_SOLUTION);

    // The path has to survive the validity contract of the space, which rules out a planner that
    // traversed an edge in a direction the steering never produced.
    og::PathGeometric &path = setup.getSolutionPath();
    BOOST_CHECK(path.check());
    BOOST_CHECK_GT(path.getStateCount(), 1u);

    // Every state on the path is a flat state the obstacle accepts.
    for (std::size_t i = 0; i < path.getStateCount(); ++i)
        BOOST_CHECK(setup.getSpaceInformation()->isValid(path.getState(i)));
}
