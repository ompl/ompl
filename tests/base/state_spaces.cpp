/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
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

/* Author: Ioan Sucan */

#define BOOST_TEST_MODULE "StateSpaces"
#include <boost/test/unit_test.hpp>
#include <iostream>

#include "ompl/base/ScopedState.h"
#include "ompl/base/SpaceInformation.h"

#include "ompl/base/spaces/TimeStateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"
#include "ompl/base/spaces/SO3StateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/spaces/SE3StateSpace.h"
#include "ompl/base/spaces/DiscreteStateSpace.h"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "ompl/base/spaces/DubinsStateSpace.h"

#include "ompl/base/spaces/special/TorusStateSpace.h"
#include "ompl/base/spaces/special/SphereStateSpace.h"
#include "ompl/base/spaces/special/MobiusStateSpace.h"
#include "ompl/base/spaces/special/KleinBottleStateSpace.h"

#include <boost/math/constants/constants.hpp>

#include "StateSpaceTest.h"

using namespace ompl;

// define a convenience macro
#define BOOST_OMPL_EXPECT_NEAR(a, b, diff) BOOST_CHECK_SMALL((a) - (b), diff)

const double PI = boost::math::constants::pi<double>();

BOOST_AUTO_TEST_CASE(Dubins_Simple)
{
    auto d(std::make_shared<base::DubinsStateSpace>());
    auto dsym(std::make_shared<base::DubinsStateSpace>(1., true));

    base::RealVectorBounds bounds2(2);
    bounds2.setLow(-3);
    bounds2.setHigh(3);
    d->setBounds(bounds2);
    dsym->setBounds(bounds2);

    d->setup();
    d->sanityChecks();

    dsym->setup();
    dsym->sanityChecks();
}

BOOST_AUTO_TEST_CASE(ReedsShepp_Simple)
{
    auto d(std::make_shared<base::ReedsSheppStateSpace>());

    base::RealVectorBounds bounds2(2);
    bounds2.setLow(-3);
    bounds2.setHigh(3);
    d->setBounds(bounds2);

    d->setup();
    d->sanityChecks();
}

BOOST_AUTO_TEST_CASE(Discrete_Simple)
{
    auto d(std::make_shared<base::DiscreteStateSpace>(0, 2));
    base::DiscreteStateSpace &dm = *d;
    d->setup();
    d->sanityChecks();

    BOOST_CHECK_EQUAL(d->getDimension(), 1u);
    BOOST_CHECK_EQUAL(d->getMaximumExtent(), 2);
    BOOST_CHECK_EQUAL(dm.getStateCount(), 3u);
    base::ScopedState<base::DiscreteStateSpace> s1(d);
    base::ScopedState<base::DiscreteStateSpace> s2(d);
    base::ScopedState<base::DiscreteStateSpace> s3(d);
    s1->value = 0;
    s2->value = 2;
    s3->value = 0;
    BOOST_CHECK_EQUAL(s1, s3);
    BOOST_CHECK_EQUAL(s2->value, 2);
    d->interpolate(s1.get(), s2.get(), 0.3, s3.get());
    BOOST_CHECK_EQUAL(s3->value, 1);
    d->interpolate(s1.get(), s2.get(), 0.2, s3.get());
    BOOST_CHECK_EQUAL(s3->value, 0);
}

BOOST_AUTO_TEST_CASE(SO2_Simple)
{
    auto m(std::make_shared<base::SO2StateSpace>());
    m->setup();
    m->sanityChecks();

    StateSpaceTest mt(m, 1000, 1e-15);
    mt.test();

    BOOST_CHECK_EQUAL(m->getDimension(), 1u);
    BOOST_CHECK_EQUAL(m->getMaximumExtent(), PI);

    base::ScopedState<base::SO2StateSpace> s1(m);
    base::ScopedState<base::SO2StateSpace> s2(m);
    base::ScopedState<base::SO2StateSpace> s3(m);

    s1->value = PI - 0.1;
    s2->value = -PI + 0.1;
    BOOST_OMPL_EXPECT_NEAR(m->distance(s2.get(), s1.get()), 0.2, 1e-3);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s1.get(), s2.get()), 0.2, 1e-3);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s1.get(), s1.get()), 0.0, 1e-3);

    s1->value = PI - 0.08;
    m->interpolate(s1.get(), s2.get(), 0.5, s3.get());
    BOOST_OMPL_EXPECT_NEAR(s3->value, -PI + 0.01, 1e-3);

    s1->value = PI - 0.1;
    s2->value = 0.1;
    BOOST_OMPL_EXPECT_NEAR(m->distance(s2.get(), s1.get()), PI - 0.2, 1e-3);

    m->interpolate(s1.get(), s2.get(), 0.5, s3.get());
    BOOST_OMPL_EXPECT_NEAR(s3->value, PI / 2.0, 1e-3);

    s2 = s1;
    m->interpolate(s1.get(), s1.get(), 0.5, s1.get());
    BOOST_CHECK_EQUAL(s1, s2);

    m->interpolate(s1.get(), s2.get(), 0.5, s1.get());
    BOOST_CHECK_EQUAL(s1, s2);
    s1->value = 0.5;
    s2->value = 1.5;

    m->interpolate(s1.get(), s2.get(), 0.0, s3.get());
    BOOST_OMPL_EXPECT_NEAR(s3->value, s1->value, 1e-3);
    m->interpolate(s1.get(), s2.get(), 1.0, s3.get());
    BOOST_OMPL_EXPECT_NEAR(s3->value, s2->value, 1e-3);
}

BOOST_AUTO_TEST_CASE(SO2_Projection)
{
    auto m(std::make_shared<base::SO2StateSpace>());
    m->setup();

    base::ProjectionEvaluatorPtr proj = m->getDefaultProjection();
    BOOST_CHECK_EQUAL(proj->getDimension(), 1u);

    Eigen::VectorXd p(proj->getDimension());
    base::ScopedState<base::SO2StateSpace> s(m);
    proj->project(s.get(), p);
    BOOST_CHECK_EQUAL(p[0], s->value);
}

BOOST_AUTO_TEST_CASE(SO2_Sampler)
{
    auto m(std::make_shared<base::SO2StateSpace>());
    m->setup();
    base::StateSamplerPtr s = m->allocStateSampler();
    base::ScopedState<base::SO2StateSpace> x(m);
    base::ScopedState<base::SO2StateSpace> y(m);
    x.random();
    for (int i = 0; i < 100; ++i)
    {
        s->sampleUniformNear(y.get(), x.get(), 10000);
        BOOST_CHECK(y.satisfiesBounds());
    }
}

BOOST_AUTO_TEST_CASE(SO3_Simple)
{
    auto m(std::make_shared<base::SO3StateSpace>());
    m->setup();
    m->sanityChecks();

    StateSpaceTest mt(m, 1000, 1e-12);
    mt.test();

    BOOST_CHECK_EQUAL(m->getDimension(), 3u);
    BOOST_CHECK_EQUAL(m->getMaximumExtent(), .5 * PI);

    base::ScopedState<base::SO3StateSpace> s1(m);
    base::ScopedState<base::SO3StateSpace> s2(m);

    s1.random();
    s2 = s1;

    BOOST_OMPL_EXPECT_NEAR(m->distance(s1.get(), s2.get()), 0.0, 1e-3);
    BOOST_CHECK_EQUAL(s1, s2);

    s2.random();

    base::SpaceInformation si(m);
    si.setStateValidityChecker([](const base::State *) { return true; });
    si.setup();

    std::vector<base::State *> states;
    unsigned int ns = 100;
    unsigned int count = si.getMotionStates(s1.get(), s2.get(), states, ns, true, true);
    BOOST_CHECK(states.size() == count);
    BOOST_CHECK(ns + 2 == count);

    for (auto &state : states)
    {
        double nrm = m->as<base::SO3StateSpace>()->norm(state->as<base::SO3StateSpace::StateType>());
        BOOST_OMPL_EXPECT_NEAR(nrm, 1.0, 1e-15);
        BOOST_CHECK(m->satisfiesBounds(state));
        si.freeState(state);
    }

    base::ProjectionEvaluatorPtr proj = m->getDefaultProjection();
    BOOST_CHECK_EQUAL(proj->getDimension(), 3u);
}

BOOST_AUTO_TEST_CASE(RealVector_Bounds)
{
    base::RealVectorBounds bounds1(1);
    bounds1.setLow(0);
    bounds1.setHigh(1);
    base::RealVectorStateSpace rsm1(1);
    rsm1.setBounds(bounds1);
    rsm1.setup();
    BOOST_CHECK_EQUAL(rsm1.getDimension(), 1u);
    BOOST_OMPL_EXPECT_NEAR(rsm1.getMaximumExtent(), 1.0, 1e-3);

    base::RealVectorBounds bounds3(3);
    bounds3.setLow(0);
    bounds3.setHigh(1);
    base::RealVectorStateSpace rsm3(3);
    rsm3.setBounds(bounds3);
    rsm3.setup();
    BOOST_CHECK(rsm3.getDimension() == 3);
    BOOST_OMPL_EXPECT_NEAR(rsm3.getMaximumExtent(), sqrt(3.0), 1e-3);

    BOOST_CHECK(rsm3.getBounds().low == bounds3.low);
    BOOST_CHECK(rsm3.getBounds().high == bounds3.high);
}

BOOST_AUTO_TEST_CASE(RealVector_Simple)
{
    base::RealVectorBounds bounds3(3);
    bounds3.setLow(0);
    bounds3.setHigh(1);
    auto m(std::make_shared<base::RealVectorStateSpace>(3));
    base::RealVectorStateSpace &rm = *m->as<base::RealVectorStateSpace>();
    rm.setBounds(bounds3);
    rm.setDimensionName(2, "testDim");
    m->setup();
    m->sanityChecks();

    StateSpaceTest mt(m, 1000, 1e-12);
    mt.test();

    base::ScopedState<base::RealVectorStateSpace> s0(m);
    (*s0)[0] = 0;
    (*s0)[1] = 0;
    (*s0)[2] = 0;

    BOOST_CHECK(m->satisfiesBounds(s0.get()));

    base::ScopedState<base::RealVectorStateSpace> s1 = s0;
    BOOST_CHECK_EQUAL(s0, s1);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s0.get(), s1.get()), 0.0, 1e-3);
    m->interpolate(s0.get(), s0.get(), 0.6, s0.get());
    BOOST_CHECK_EQUAL(s0, s1);
    s1->values[2] = 1.0;

    BOOST_CHECK(m->satisfiesBounds(s1.get()));
    m->interpolate(s0.get(), s1.get(), 0.5, s0.get());

    BOOST_OMPL_EXPECT_NEAR((*s0)[rm.getDimensionIndex("testDim")], 0.5, 1e-3);

    auto m2(std::make_shared<base::RealVectorStateSpace>());
    m2->addDimension(1, 2);
    m2->setup();
    BOOST_OMPL_EXPECT_NEAR(m2->getMaximumExtent(), 1.0, 1e-3);
    BOOST_CHECK_EQUAL(m2->getDimension(), 1u);
}

BOOST_AUTO_TEST_CASE(Time_Bounds)
{
    base::TimeStateSpace t;
    t.setup();

    BOOST_CHECK_EQUAL(t.getDimension(), 1u);
    BOOST_CHECK_EQUAL(t.isBounded(), false);
    BOOST_OMPL_EXPECT_NEAR(t.getMaximumExtent(), 1.0, 1e-3);

    t.setBounds(-1, 1);
    BOOST_CHECK(t.isBounded());
    BOOST_OMPL_EXPECT_NEAR(t.getMaximumExtent(), 2.0, 1e-3);
}

BOOST_AUTO_TEST_CASE(Time_Simple)
{
    auto t(std::make_shared<base::TimeStateSpace>());
    t->setup();
    t->sanityChecks();
    t->params()["valid_segment_count_factor"] = 1;

    StateSpaceTest mt(t, 1000, 1e-12);
    mt.test();

    base::ScopedState<base::TimeStateSpace> ss(t);
    ss.random();
    BOOST_CHECK_EQUAL(ss->position, 0.0);
    BOOST_CHECK(t->satisfiesBounds(ss.get()));

    t->as<base::TimeStateSpace>()->setBounds(-1, 1);
    t->setup();

    for (int i = 0; i < 100; ++i)
    {
        ss.random();
        if (fabs(ss->position) > 0.0)
            break;
    }
    BOOST_CHECK(fabs(ss->position) > 0.0);

    ss->position = 2.0;
    BOOST_CHECK_EQUAL(t->satisfiesBounds(ss.get()), false);
    t->enforceBounds(ss.get());
    BOOST_OMPL_EXPECT_NEAR(ss->position, 1.0, 1e-3);
    BOOST_CHECK(t->satisfiesBounds(ss.get()));

    base::ScopedState<base::TimeStateSpace> s0 = ss;
    s0->position = 0.5;
    t->interpolate(s0.get(), ss.get(), 0.5, ss.get());
    BOOST_OMPL_EXPECT_NEAR(ss->position, 0.75, 1e-3);
    BOOST_CHECK_NE(s0, ss);
    ss = s0;
    BOOST_CHECK(s0 == ss);
}

BOOST_AUTO_TEST_CASE(Compound_Simple)
{
    auto m1(std::make_shared<base::SE2StateSpace>());
    auto m2(std::make_shared<base::SE3StateSpace>());
    auto m3(std::make_shared<base::SO2StateSpace>());
    auto m4(std::make_shared<base::SO3StateSpace>());

    BOOST_CHECK(m1 + m1 == m1);

    base::StateSpacePtr s = m1 + m2 + m3;
    BOOST_CHECK(s + s == s);
    BOOST_CHECK(s - s + s == s);
    BOOST_CHECK(s + s - s + s == s);
    BOOST_CHECK(s * s == s);
    BOOST_CHECK(s * m2 == m2);
    BOOST_CHECK(m1 * s == m1);
    BOOST_CHECK(m1 + s == s);
    BOOST_CHECK(s + m2 == s);
    BOOST_CHECK(m1 + s + m2 == s);
    BOOST_CHECK(m1 + s + m2 - "x" == s);
    BOOST_CHECK(s - "random" == s);
    BOOST_CHECK(m3 + m3 == m3);
    BOOST_CHECK(m3 + s * m3 * s - m2 - m1 == m3 * s);

    BOOST_CHECK(base::StateSpacePtr() + m1 == m1);
    BOOST_CHECK(m1 + base::StateSpacePtr() == m1);
    BOOST_CHECK_EQUAL(m1 + base::StateSpacePtr() == m2 * s, false);
    BOOST_CHECK(base::StateSpacePtr() * s + m1 == m1);
    BOOST_CHECK(m3 * base::StateSpacePtr() + m1 == m1);
    BOOST_CHECK(base::StateSpacePtr() * base::StateSpacePtr() + m4 == m4);
    BOOST_CHECK(base::StateSpacePtr() + base::StateSpacePtr() + m4 == m4);
    BOOST_CHECK(base::StateSpacePtr() - base::StateSpacePtr() + m4 == m4);
    BOOST_CHECK((base::StateSpacePtr() - "")->getDimension() == 0);

    BOOST_CHECK_EQUAL(s->getDimension(), m1->getDimension() + m2->getDimension() + m3->getDimension());
    base::StateSpacePtr d = s - m2;
    BOOST_CHECK_EQUAL(d->getDimension(), m1->getDimension() + m3->getDimension());
    BOOST_CHECK((s + d)->getDimension() == s->getDimension());

    m4->setName("test");
    BOOST_CHECK(m4->getName() == "test");

    base::StateSpacePtr t = m1 + m4;
    BOOST_CHECK_EQUAL((t - "test")->getDimension(), m1->getDimension());
    BOOST_CHECK_EQUAL((m1 - m1)->getDimension(), 0u);
    t->setName(t->getName());
    base::ScopedState<> st(t);
    BOOST_CHECK(t->getValueAddressAtIndex(st.get(), 10000) == nullptr);
    BOOST_CHECK(t->includes(m1));
    BOOST_CHECK_EQUAL(t->includes(m2), false);
    BOOST_CHECK_EQUAL(m1->includes(t), false);
    BOOST_CHECK(m3->includes(m3));
    BOOST_CHECK(t->includes(t));
}

BOOST_AUTO_TEST_CASE(Torus_Simple)
{
    auto m(std::make_shared<base::TorusStateSpace>());
    m->setup();
    m->sanityChecks();

    StateSpaceTest mt(m, 1000, 1e-12);
    mt.test();

    BOOST_CHECK_EQUAL(m->getDimension(), 2u);
    base::ScopedState<base::TorusStateSpace> s1(m);
    base::ScopedState<base::TorusStateSpace> s2(m);

    //Check distances on first dimension
    s1->setS1(PI - 0.1);
    s1->setS2(0);
    s2->setS1(-PI + 0.1);
    s2->setS2(0);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s2.get(), s1.get()), 0.2, 1e-3);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s1.get(), s2.get()), 0.2, 1e-3);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s1.get(), s1.get()), 0.0, 1e-3);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s2.get(), s2.get()), 0.0, 1e-3);

    //Check distances on second dimension
    s1->setS1(0);
    s1->setS2(+PI - 0.1);
    s2->setS1(0);
    s2->setS2(-PI + 0.1);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s2.get(), s1.get()), 0.2, 1e-3);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s1.get(), s2.get()), 0.2, 1e-3);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s1.get(), s1.get()), 0.0, 1e-3);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s2.get(), s2.get()), 0.0, 1e-3);

    //Check interpolation
    s2 = s1;
    m->interpolate(s1.get(), s1.get(), 0.5, s1.get());
    BOOST_CHECK_EQUAL(s1, s2);
    m->interpolate(s1.get(), s2.get(), 0.5, s1.get());
    BOOST_CHECK_EQUAL(s1, s2);

}

BOOST_AUTO_TEST_CASE(Mobius_Simple)
{
    const double intervalMax = 1.0;

    auto m(std::make_shared<base::MobiusStateSpace>(intervalMax));
    m->setup();
    m->sanityChecks();

    StateSpaceTest mt(m, 1000, 1e-12);
    mt.test();

    BOOST_CHECK_EQUAL(m->getDimension(), 2u);

    base::ScopedState<base::MobiusStateSpace> s1(m);
    base::ScopedState<base::MobiusStateSpace> s2(m);

    s1->setU(-PI);
    s1->setV(-intervalMax);
    s2->setU(-PI);
    s2->setV(+intervalMax);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s2.get(), s1.get()), 2*intervalMax, 1e-3);

    s1->setU(+PI);
    s1->setV(-intervalMax);
    s2->setU(+PI);
    s2->setV(+intervalMax);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s2.get(), s1.get()), 2*intervalMax, 1e-3);

    //check that the both endings are correctly glued together
    s1->setU(+PI - 0.1);
    s1->setV(+intervalMax);
    s2->setU(-PI + 0.1);
    s2->setV(-intervalMax); //segment should be inverted!
    BOOST_OMPL_EXPECT_NEAR(m->distance(s2.get(), s1.get()), 0.2, 1e-3);

    s1->setU(+PI - 0.1);
    s1->setV(+0);
    s2->setU(-PI + 0.1);
    s2->setV(+0);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s2.get(), s1.get()), 0.2, 1e-3);

    s1->setU(+PI - 0.1);
    s1->setV(+intervalMax);
    s2->setU(-PI + 0.1);
    s2->setV(+intervalMax);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s2.get(), s1.get()), 2*intervalMax + 0.2, 1e-3);
}

BOOST_AUTO_TEST_CASE(Sphere_Simple)
{
    auto m(std::make_shared<base::SphereStateSpace>(1.0));
    m->setup();

    unsigned int flags = ~0;
    double eps = std::numeric_limits<double>::epsilon();
    m->sanityChecks(eps, 1e-6, flags);

    StateSpaceTest mt(m, 1000, 1e-12);
    mt.testDistance();

    BOOST_CHECK_EQUAL(m->getDimension(), 2u);
    base::ScopedState<base::SphereStateSpace> s1(m);
    base::ScopedState<base::SphereStateSpace> s2(m);

    //north-south interpolation
    s1->setTheta(0);
    s1->setPhi(0);
    s2->setTheta(0);
    s2->setPhi(PI);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s2.get(), s1.get()), PI, 1e-3);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s1.get(), s2.get()), PI, 1e-3);

    //east-west interpolation (at south pole)
    s1->setTheta(0);
    s1->setPhi(0);
    s2->setTheta(+PI);
    s2->setPhi(0);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s2.get(), s1.get()), 0.0, 1e-3);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s1.get(), s2.get()), 0.0, 1e-3);

    //east-west interpolation (at north pole)
    s1->setTheta(PI);
    s1->setPhi(PI);
    s2->setTheta(0);
    s2->setPhi(PI);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s2.get(), s1.get()), 0.0, 1e-3);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s1.get(), s2.get()), 0.0, 1e-3);

    //east-west interpolation (at equator)
    s1->setTheta(0);
    s1->setPhi(+PI/2.0);
    s2->setTheta(+PI/3);
    s2->setPhi(+PI/2.0);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s2.get(), s1.get()), PI/3, 1e-3);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s1.get(), s2.get()), PI/3, 1e-3);
}

BOOST_AUTO_TEST_CASE(KleinBottle_Simple)
{
    auto m(std::make_shared<base::KleinBottleStateSpace>());
    m->setup();
    m->sanityChecks();
    BOOST_CHECK_EQUAL(m->getDimension(), 2u);

    StateSpaceTest mt(m, 1000, 1e-12);
    mt.test();

    base::ScopedState<base::KleinBottleStateSpace> s1(m);
    base::ScopedState<base::KleinBottleStateSpace> s2(m);

    //Check that gluing overlaps in the correct way
    s1->setUV(0.1, 0); 
    s2->setUV(PI-0.1, -PI);

    BOOST_OMPL_EXPECT_NEAR(m->distance(s1.get(), s2.get()), 0.2, 1e-3);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s2.get(), s1.get()), 0.2, 1e-3);

    s1->setUV(0.1, 0); 
    s2->setUV(PI-0.1, +PI);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s1.get(), s2.get()), 0.2, 1e-3);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s2.get(), s1.get()), 0.2, 1e-3);

    s1->setUV(0.1, 0.5*PI); 
    s2->setUV(PI-0.1, 0.5*PI);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s1.get(), s2.get()), 0.2, 1e-3);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s2.get(), s1.get()), 0.2, 1e-3);

    s1->setUV(0.1, -0.3*PI); 
    s2->setUV(PI-0.1, -PI+0.3*PI);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s1.get(), s2.get()), 0.2, 1e-3);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s2.get(), s1.get()), 0.2, 1e-3);

    //Check interpolations along gluing
    base::ScopedState<base::KleinBottleStateSpace> s3(m);
    base::ScopedState<base::KleinBottleStateSpace> s4(m);

    s1->setUV(0.1, -0.3*PI); 
    s2->setUV(PI-0.1, -PI+0.3*PI);

    m->interpolate(s1.get(), s2.get(), 0.3, s3.get());
    BOOST_OMPL_EXPECT_NEAR(m->distance(s3.get(), s1.get()), 0.3*0.2, 1e-3);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s3.get(), s2.get()), 0.7*0.2, 1e-3);
    m->interpolate(s1.get(), s2.get(), 0.7, s3.get());
    BOOST_OMPL_EXPECT_NEAR(m->distance(s3.get(), s1.get()), 0.7*0.2, 1e-3);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s3.get(), s2.get()), 0.3*0.2, 1e-3);

    m->interpolate(s1.get(), s2.get(), 0.5, s3.get());

    s4->setUV(PI, -PI+0.3*PI);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s3.get(), s4.get()), 0.0, 1e-3);

    s4->setUV(0, -0.3*PI);
    BOOST_OMPL_EXPECT_NEAR(m->distance(s3.get(), s4.get()), 0.0, 1e-3);

    //Check interpolation
    s2 = s1;
    m->interpolate(s1.get(), s1.get(), 0.5, s1.get());
    BOOST_CHECK_EQUAL(s1, s2);
    m->interpolate(s1.get(), s2.get(), 0.5, s1.get());
    BOOST_CHECK_EQUAL(s1, s2);
}

