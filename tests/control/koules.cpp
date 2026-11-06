/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: OpenAI Codex (AI-assisted contribution for human review) */

#define BOOST_TEST_MODULE "KoulesSimulator"
#include <boost/test/unit_test.hpp>

#include "KoulesSimulator.h"
#include "KoulesStateSpace.h"
#include <ompl/base/ScopedState.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <algorithm>
#include <cmath>
#include <memory>

namespace ob = ompl::base;
namespace oc = ompl::control;

namespace
{
    struct KoulesFixture
    {
        std::shared_ptr<KoulesStateSpace> space;
        std::shared_ptr<oc::RealVectorControlSpace> controls = std::make_shared<oc::RealVectorControlSpace>(space, 2);
        oc::SpaceInformation si{space, controls};
        ob::ScopedState<KoulesStateSpace> start{space};
        ob::ScopedState<KoulesStateSpace> result{space};
        oc::Control *control = controls->allocControl();

        explicit KoulesFixture(unsigned int numKoules = 1) : space(std::make_shared<KoulesStateSpace>(numKoules))
        {
            std::fill_n(start->values, space->getDimension(), 0.);
            start->values[0] = start->values[1] = .5;
            start->values[5] = start->values[6] = .25;
            controls->nullControl(control);
        }

        ~KoulesFixture()
        {
            controls->freeControl(control);
        }

        void step()
        {
            KoulesSimulator simulator(&si);
            simulator.step(start.get(), control, .02, result.get());
            for (unsigned int i = 0; i < space->getDimension(); ++i)
                BOOST_CHECK(std::isfinite(result->values[i]));
        }
    };
}  // namespace

// Run with AddressSanitizer to detect an invalid wall index used to access a koule radius.
BOOST_AUTO_TEST_CASE(KouleHitsEachWall)
{
    for (unsigned int dimension = 0; dimension < 2; ++dimension)
        for (const double direction : {-1., 1.})
        {
            BOOST_TEST_CONTEXT("dimension=" << dimension << " direction=" << direction)
            {
                KoulesFixture fixture;
                fixture.start->values[5 + dimension] = direction < 0. ? .02 : .98;
                fixture.start->values[7 + dimension] = direction;
                fixture.step();
                BOOST_CHECK(fixture.space->isDead(fixture.result.get(), 1));
                BOOST_CHECK(!fixture.space->isDead(fixture.result.get(), 0));
            }
        }
}

BOOST_AUTO_TEST_CASE(NoWallCollision)
{
    KoulesFixture fixture;
    fixture.step();
    BOOST_CHECK(!fixture.space->isDead(fixture.result.get(), 0));
    BOOST_CHECK(!fixture.space->isDead(fixture.result.get(), 1));
}

BOOST_AUTO_TEST_CASE(ShipHitsWall)
{
    KoulesFixture fixture;
    fixture.start->values[0] = .965;
    fixture.start->values[2] = 1.;
    fixture.step();
    BOOST_CHECK(fixture.space->isDead(fixture.result.get(), 0));
}

BOOST_AUTO_TEST_CASE(KoulesCollideWithEachOther)
{
    KoulesFixture fixture(2);
    fixture.start->values[5] = .45;
    fixture.start->values[6] = .25;
    fixture.start->values[7] = 1.;
    fixture.start->values[9] = .49;
    fixture.start->values[10] = .25;
    fixture.start->values[11] = -1.;
    fixture.step();
    BOOST_CHECK(!fixture.space->isDead(fixture.result.get(), 1));
    BOOST_CHECK(!fixture.space->isDead(fixture.result.get(), 2));
    BOOST_CHECK_LT(fixture.result->values[7], 0.);
    BOOST_CHECK_GT(fixture.result->values[11], 0.);
}
