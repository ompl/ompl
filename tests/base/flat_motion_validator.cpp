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

#define BOOST_TEST_MODULE "FlatMotionValidator"
#include <boost/test/unit_test.hpp>

#include "ompl/base/DiscreteMotionValidator.h"
#include "ompl/base/ScopedState.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/FlatMotionValidator.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/util/Exception.h"

#include <cmath>
#include <memory>

using namespace ompl::base;

namespace
{
    constexpr unsigned int ORDER = 2;
    constexpr unsigned int DIMENSION = 2;

    /** \brief A flat space that counts the steering solves the space is asked for. */
    class CountingFlatStateSpace : public FlatStateSpace
    {
    public:
        CountingFlatStateSpace(const StateSpacePtr &output) : FlatStateSpace(output, ORDER)
        {
        }

        std::optional<FlatMotion> steer(const State *from, const State *to) const override
        {
            ++solves;
            return FlatStateSpace::steer(from, to);
        }

        mutable unsigned int solves{0u};
    };

    /** \brief A grid of square pillars in the flat output plane. */
    class PillarField : public StateValidityChecker
    {
    public:
        PillarField(const SpaceInformationPtr &si) : StateValidityChecker(si)
        {
        }

        bool isValid(const State *state) const override
        {
            const double *values =
                state->as<FlatStateSpace::StateType>()->output()->as<RealVectorStateSpace::StateType>()->values;
            const double x = values[0] - 0.5 * std::floor(values[0] / 0.5) - 0.25;
            const double y = values[1] - 0.5 * std::floor(values[1] / 0.5) - 0.25;
            return std::abs(x) > 0.03 || std::abs(y) > 0.03;
        }
    };

    std::shared_ptr<CountingFlatStateSpace> makeSpace()
    {
        auto output = std::make_shared<RealVectorStateSpace>(DIMENSION);
        output->setBounds(-1., 1.);
        auto space = std::make_shared<CountingFlatStateSpace>(output);
        space->setDerivativeBound(1, 2.);
        return space;
    }
}  // namespace

BOOST_AUTO_TEST_CASE(FlatSpacesGetTheFlatValidatorByDefault)
{
    SpaceInformation si(makeSpace());
    si.setStateValidityChecker([](const State *) { return true; });
    si.setup();

    BOOST_CHECK(std::dynamic_pointer_cast<FlatMotionValidator>(si.getMotionValidator()) != nullptr);
}

BOOST_AUTO_TEST_CASE(OtherSpacesAreRejected)
{
    auto space = std::make_shared<RealVectorStateSpace>(DIMENSION);
    space->setBounds(-1., 1.);
    auto si = std::make_shared<SpaceInformation>(space);

    BOOST_CHECK_THROW(FlatMotionValidator validator(si), ompl::Exception);
}

BOOST_AUTO_TEST_CASE(VerdictsMatchTheDiscreteValidator)
{
    auto space = makeSpace();
    auto si = std::make_shared<SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<PillarField>(si));
    si->setup();

    FlatMotionValidator flat(si);
    DiscreteMotionValidator discrete(si);

    StateSamplerPtr sampler = space->allocStateSampler();
    ScopedState<> from(space), to(space);

    unsigned int rejected = 0u;
    for (unsigned int trial = 0; trial < 2000u; ++trial)
    {
        sampler->sampleUniform(from.get());
        sampler->sampleUniform(to.get());

        const bool expected = discrete.checkMotion(from.get(), to.get());
        BOOST_REQUIRE_EQUAL(flat.checkMotion(from.get(), to.get()), expected);
        rejected += expected ? 0u : 1u;
    }

    // A field the sampler always clears would make the agreement above worth nothing.
    BOOST_CHECK_GT(rejected, 100u);
    BOOST_CHECK_LT(rejected, 1900u);
    BOOST_CHECK_EQUAL(flat.getValidMotionCount(), discrete.getValidMotionCount());
    BOOST_CHECK_EQUAL(flat.getInvalidMotionCount(), discrete.getInvalidMotionCount());
}

BOOST_AUTO_TEST_CASE(TheLastValidStateMatchesTheDiscreteValidator)
{
    auto space = makeSpace();
    auto si = std::make_shared<SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<PillarField>(si));
    si->setup();

    FlatMotionValidator flat(si);
    DiscreteMotionValidator discrete(si);

    StateSamplerPtr sampler = space->allocStateSampler();
    ScopedState<> from(space), to(space);

    std::pair<State *, double> flatLast(si->allocState(), 0.);
    std::pair<State *, double> discreteLast(si->allocState(), 0.);

    unsigned int rejected = 0u;
    for (unsigned int trial = 0; trial < 2000u; ++trial)
    {
        sampler->sampleUniform(from.get());
        sampler->sampleUniform(to.get());
        if (!si->isValid(from.get()))
            continue;

        flatLast.second = -1.;
        discreteLast.second = -1.;
        const bool expected = discrete.checkMotion(from.get(), to.get(), discreteLast);
        BOOST_REQUIRE_EQUAL(flat.checkMotion(from.get(), to.get(), flatLast), expected);
        if (expected)
            continue;

        ++rejected;
        BOOST_REQUIRE_EQUAL(flatLast.second, discreteLast.second);
        BOOST_REQUIRE_SMALL(space->distance(flatLast.first, discreteLast.first), 1e-12);

        // The motion stayed valid through the reported fraction, so the state there has to clear the
        // obstacles.
        BOOST_REQUIRE(si->isValid(flatLast.first));
    }

    BOOST_CHECK_GT(rejected, 100u);

    si->freeState(flatLast.first);
    si->freeState(discreteLast.first);
}

BOOST_AUTO_TEST_CASE(EachMotionCostsOneSteeringSolve)
{
    auto space = makeSpace();
    auto si = std::make_shared<SpaceInformation>(space);
    si->setStateValidityChecker([](const State *) { return true; });
    si->setup();

    FlatMotionValidator flat(si);
    DiscreteMotionValidator discrete(si);

    ScopedState<> from(space), to(space);
    Eigen::MatrixXd start(ORDER, DIMENSION), goal(ORDER, DIMENSION);
    start << -0.9, -0.9, 0., 0.;
    goal << 0.9, 0.9, 0., 0.;
    space->fromFlatState(start, from.get());
    space->fromFlatState(goal, to.get());

    // The edge has to be long enough that the samples outnumber the solve the validator makes.
    const std::optional<FlatMotion> motion = space->steer(from.get(), to.get());
    BOOST_REQUIRE(motion.has_value());
    const unsigned int count = space->validSegmentCount(*motion);
    BOOST_REQUIRE_GT(count, 10u);

    space->solves = 0u;
    BOOST_CHECK(flat.checkMotion(from.get(), to.get()));
    BOOST_CHECK_EQUAL(space->solves, 1u);

    space->solves = 0u;
    std::pair<State *, double> last(nullptr, 0.);
    BOOST_CHECK(flat.checkMotion(from.get(), to.get(), last));
    BOOST_CHECK_EQUAL(space->solves, 1u);

    // Without the validator, every sample along the edge pays for a solve of its own, on top of the one
    // that sizes the walk.
    space->solves = 0u;
    BOOST_CHECK(discrete.checkMotion(from.get(), to.get()));
    BOOST_CHECK_EQUAL(space->solves, count);
}
