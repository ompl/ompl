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

#define BOOST_TEST_MODULE "Planner comparison geometry"
#include <boost/test/unit_test.hpp>
#include <ompl/base/ScopedState.h>
#include "../../demos/PlannerComparisonGeometry.h"

namespace ob = ompl::base;

struct GeometryFixture
{
    std::shared_ptr<ob::RealVectorStateSpace> space = std::make_shared<ob::RealVectorStateSpace>(2);
    ob::ScopedState<> a{space}, b{space};
};

BOOST_FIXTURE_TEST_CASE(detects_arbitrarily_short_rectangle_corner_crossing, GeometryFixture)
{
    PlannerComparisonWorld world("open");
    world.rectangles = {{0.0, 0.2, 0.0, 0.2}};
    a[0] = -0.5;
    a[1] = 0.89999;
    b[0] = 0.89999;
    b[1] = -0.5;
    BOOST_CHECK(world.valid(a.get()));
    BOOST_CHECK(world.valid(b.get()));
    BOOST_CHECK(!world.segmentValid(a.get(), b.get()));
    a[1] = b[0] = 0.90001;
    BOOST_CHECK(world.segmentValid(a.get(), b.get()));
}

BOOST_FIXTURE_TEST_CASE(detects_circle_tangency_and_zero_length_collision, GeometryFixture)
{
    PlannerComparisonWorld world("open");
    world.circles = {{0.0, 0.0, 0.2}};
    a[0] = -0.5;
    b[0] = 0.5;
    a[1] = b[1] = 0.2;
    BOOST_CHECK(!world.segmentValid(a.get(), b.get()));
    a[1] = b[1] = 0.20001;
    BOOST_CHECK(world.segmentValid(a.get(), b.get()));
    a[0] = a[1] = b[0] = b[1] = 0.0;
    BOOST_CHECK(!world.segmentValid(a.get(), b.get()));
}

BOOST_FIXTURE_TEST_CASE(narrow_slit_requires_a_detour_and_blocked_wall_is_impossible, GeometryFixture)
{
    PlannerComparisonWorld narrow("narrow"), blocked("blocked");
    a[0] = -0.9;
    b[0] = 0.9;
    a[1] = b[1] = 0.0;
    BOOST_CHECK(!narrow.segmentValid(a.get(), b.get()));
    a[1] = b[1] = 0.545;
    BOOST_CHECK(narrow.segmentValid(a.get(), b.get()));
    BOOST_CHECK(!blocked.segmentValid(a.get(), b.get()));
}

BOOST_FIXTURE_TEST_CASE(last_valid_prefix_stops_before_first_obstacle, GeometryFixture)
{
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1.0);
    bounds.setHigh(1.0);
    space->setBounds(bounds);
    auto si = std::make_shared<ob::SpaceInformation>(space);
    PlannerComparisonWorld world("blocked");
    si->setStateValidityChecker([&world](const ob::State *state) { return world.valid(state); });
    std::size_t checks = 0;
    PlannerComparisonMotionValidator validator(si, world, checks);
    si->setup();
    a[0] = -0.9;
    b[0] = 0.9;
    a[1] = b[1] = 0.0;
    ob::ScopedState<> prefix(space);
    std::pair<ob::State *, double> lastValid{prefix.get(), -1.0};
    BOOST_CHECK(!validator.checkMotion(a.get(), b.get(), lastValid));
    // The first wall face is x=-0.12, at fraction 0.78/1.8 = 13/30.
    BOOST_CHECK_SMALL(lastValid.second - 13.0 / 30.0, 1e-9);
    BOOST_CHECK(world.valid(prefix.get()));
    BOOST_CHECK(world.segmentValid(a.get(), prefix.get()));
    BOOST_CHECK_EQUAL(checks, 1u);
}
