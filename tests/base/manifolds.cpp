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

#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <libgen.h>
#include <iostream>

#include "ompl/base/ScopedState.h"
#include "ompl/base/SpaceInformation.h"

#include "ompl/base/manifolds/TimeStateManifold.h"
#include "ompl/base/manifolds/RealVectorStateManifold.h"
#include "ompl/base/manifolds/SO2StateManifold.h"
#include "ompl/base/manifolds/SO3StateManifold.h"
#include "ompl/base/manifolds/SE2StateManifold.h"
#include "ompl/base/manifolds/SE3StateManifold.h"

#include <boost/math/constants/constants.hpp>

using namespace ompl;

const double PI = boost::math::constants::pi<double>();

bool isValid(const base::State *)
{
    return true;
}

TEST(SO2, Simple)
{
    base::StateManifoldPtr m(new base::SO2StateManifold());
    m->setup();

    EXPECT_EQ(m->getDimension(), 1u);
    EXPECT_EQ(m->getMaximumExtent(), PI);

    base::ScopedState<base::SO2StateManifold> s1(m);
    base::ScopedState<base::SO2StateManifold> s2(m);
    base::ScopedState<base::SO2StateManifold> s3(m);

    s1->value = PI - 0.1;
    s2->value = -PI + 0.1;
    EXPECT_NEAR(m->distance(s2.get(), s1.get()), 0.2, 1e-3);
    EXPECT_NEAR(m->distance(s1.get(), s2.get()), 0.2, 1e-3);
    EXPECT_NEAR(m->distance(s1.get(), s1.get()), 0.0, 1e-3);

    s1->value = PI - 0.08;
    m->interpolate(s1.get(), s2.get(), 0.5, s3.get());
    EXPECT_NEAR(s3->value, -PI + 0.01, 1e-3);

    s1->value = PI - 0.1;
    s2->value = 0.1;
    EXPECT_NEAR(m->distance(s2.get(), s1.get()), PI - 0.2, 1e-3);

    m->interpolate(s1.get(), s2.get(), 0.5, s3.get());
    EXPECT_NEAR(s3->value, PI / 2.0, 1e-3);

    s2 = s1;
    m->interpolate(s1.get(), s1.get(), 0.5, s1.get());
    EXPECT_EQ(s1, s2);

    m->interpolate(s1.get(), s2.get(), 0.5, s1.get());
    EXPECT_EQ(s1, s2);
    s1->value = 0.5;
    s2->value = 1.5;

    m->interpolate(s1.get(), s2.get(), 0.0, s3.get());
    EXPECT_NEAR(s3->value, s1->value, 1e-3);
    m->interpolate(s1.get(), s2.get(), 1.0, s3.get());
    EXPECT_NEAR(s3->value, s2->value, 1e-3);
}

TEST(SO2, Projection)
{
    base::StateManifoldPtr m(new base::SO2StateManifold());
    m->setup();

    base::ProjectionEvaluatorPtr proj = m->getDefaultProjection();
    EXPECT_EQ(proj->getDimension(), 1u);

    base::EuclideanProjection p(proj->getDimension());
    base::ScopedState<base::SO2StateManifold> s(m);
    proj->project(s.get(), p);
    EXPECT_EQ(p[0], s->value);
}

TEST(SO3, Simple)
{
    base::StateManifoldPtr m(new base::SO3StateManifold());
    m->setup();

    EXPECT_EQ(m->getDimension(), 3u);
    EXPECT_EQ(m->getMaximumExtent(), PI);

    base::ScopedState<base::SO3StateManifold> s1(m);
    base::ScopedState<base::SO3StateManifold> s2(m);

    s1.random();
    s2 = s1;

    EXPECT_NEAR(m->distance(s1.get(), s2.get()), 0.0, 1e-3);
    EXPECT_EQ(s1, s2);

    s2.random();

    base::SpaceInformation si(m);
    si.setStateValidityChecker(boost::bind(&isValid, _1));
    si.setup();

    std::vector<base::State*> states;
    unsigned int ns = 100;
    unsigned int count = si.getMotionStates(s1.get(), s2.get(), states, ns, true, true);
    EXPECT_TRUE(states.size() == count);
    EXPECT_TRUE(ns + 2 == count);

    for (unsigned int i = 0 ; i < states.size() ; ++i)
    {
        double nrm = m->as<base::SO3StateManifold>()->norm(states[i]->as<base::SO3StateManifold::StateType>());
        EXPECT_NEAR(nrm, 1.0, 1e-15);
        EXPECT_TRUE(m->satisfiesBounds(states[i]));
        si.freeState(states[i]);
    }

    base::ProjectionEvaluatorPtr proj = m->getDefaultProjection();
    EXPECT_EQ(proj->getDimension(), 3u);
}

TEST(RealVector, Bounds)
{
    base::RealVectorBounds bounds1(1);
    bounds1.setLow(0);
    bounds1.setHigh(1);
    base::RealVectorStateManifold rsm1(1);
    rsm1.setBounds(bounds1);
    rsm1.setup();
    EXPECT_EQ(rsm1.getDimension(), 1u);
    EXPECT_NEAR(rsm1.getMaximumExtent(), 1.0, 1e-3);

    base::RealVectorBounds bounds3(3);
    bounds3.setLow(0);
    bounds3.setHigh(1);
    base::RealVectorStateManifold rsm3(3);
    rsm3.setBounds(bounds3);
    rsm3.setup();
    EXPECT_TRUE(rsm3.getDimension() == 3);
    EXPECT_NEAR(rsm3.getMaximumExtent(), sqrt(3.0), 1e-3);

    EXPECT_TRUE(rsm3.getBounds().low == bounds3.low);
    EXPECT_TRUE(rsm3.getBounds().high == bounds3.high);
}

TEST(RealVector, Simple)
{
    base::RealVectorBounds bounds3(3);
    bounds3.setLow(0);
    bounds3.setHigh(1);
    base::StateManifoldPtr m(new base::RealVectorStateManifold(3));
    base::RealVectorStateManifold &rm = *m->as<base::RealVectorStateManifold>();
    rm.setBounds(bounds3);
    rm.setDimensionName(2, "testDim");
    m->setup();

    base::ScopedState<base::RealVectorStateManifold> s0(m);
    (*s0)[0] = 0;
    (*s0)[1] = 0;
    (*s0)[2] = 0;

    EXPECT_TRUE(m->satisfiesBounds(s0.get()));

    base::ScopedState<base::RealVectorStateManifold> s1 = s0;
    EXPECT_EQ(s0, s1);
    EXPECT_NEAR(m->distance(s0.get(), s1.get()), 0.0, 1e-3);
    m->interpolate(s0.get(), s0.get(), 0.6, s0.get());
    EXPECT_EQ(s0, s1);
    s1->values[2] = 1.0;

    EXPECT_TRUE(m->satisfiesBounds(s1.get()));
    m->interpolate(s0.get(), s1.get(), 0.5, s0.get());

    EXPECT_NEAR((*s0)[rm.getDimensionIndex("testDim")], 0.5, 1e-3);

    base::StateManifoldPtr m2(new base::RealVectorStateManifold());
    m2->as<base::RealVectorStateManifold>()->addDimension(1, 2);
    m2->setup();
    EXPECT_NEAR(m2->getMaximumExtent(), 1.0, 1e-3);
    EXPECT_EQ(m2->getDimension(), 1u);
}

TEST(Time, Bounds)
{
    base::TimeStateManifold t;
    t.setup();

    EXPECT_EQ(t.getDimension(), 1u);
    EXPECT_FALSE(t.isBounded());
    EXPECT_NEAR(t.getMaximumExtent(), 1.0, 1e-3);

    t.setBounds(-1, 1);
    EXPECT_TRUE(t.isBounded());
    EXPECT_NEAR(t.getMaximumExtent(), 2.0, 1e-3);
}

TEST(Time, Simple)
{
    base::StateManifoldPtr t(new base::TimeStateManifold());
    t->setup();

    base::ScopedState<base::TimeStateManifold> ss(t);
    ss.random();
    EXPECT_EQ(ss->position, 0.0);
    EXPECT_TRUE(t->satisfiesBounds(ss.get()));

    t->as<base::TimeStateManifold>()->setBounds(-1, 1);
    t->setup();

    for (int i = 0 ; i < 100 ; ++i)
    {
        ss.random();
        if (ss->position != 0.0)
            break;
    }
    EXPECT_TRUE(ss->position != 0.0);

    ss->position = 2.0;
    EXPECT_FALSE(t->satisfiesBounds(ss.get()));
    t->enforceBounds(ss.get());
    EXPECT_NEAR(ss->position, 1.0, 1e-3);
    EXPECT_TRUE(t->satisfiesBounds(ss.get()));

    base::ScopedState<base::TimeStateManifold> s0 = ss;
    s0->position = 0.5;
    t->interpolate(s0.get(), ss.get(), 0.5, ss.get());
    EXPECT_NEAR(ss->position, 0.75, 1e-3);
    EXPECT_FALSE(s0 == ss);
    ss = s0;
    EXPECT_TRUE(s0 == ss);
}

TEST(Compound, Simple)
{
    base::StateManifoldPtr m1(new base::SE2StateManifold());
    base::StateManifoldPtr m2(new base::SE3StateManifold());
    base::StateManifoldPtr m3(new base::SO2StateManifold());
    base::StateManifoldPtr m4(new base::SO3StateManifold());

    EXPECT_TRUE(m1 + m1 == m1);
    
    base::StateManifoldPtr s = m1 + m2 + m3;
    EXPECT_TRUE(s + s == s);
    EXPECT_TRUE(s - s + s == s);
    EXPECT_TRUE(s + s - s + s  == s);
    EXPECT_TRUE(s * s  == s);
    EXPECT_TRUE(s * m2 == m2);
    EXPECT_TRUE(m1 * s == m1);
    EXPECT_TRUE(m1 + s == s);
    EXPECT_TRUE(s + m2 == s);
    EXPECT_TRUE(m1 + s + m2 == s);
    EXPECT_TRUE(m1 + s + m2 - "x" == s);
    EXPECT_TRUE(s - "random" == s);
    EXPECT_TRUE(m3 + m3 == m3);
    EXPECT_TRUE(m3 + s * m3 * s - m2 - m1 == m3 * s);
    
    EXPECT_TRUE(base::StateManifoldPtr() + m1 == m1);
    EXPECT_TRUE(m1 + base::StateManifoldPtr() == m1);
    EXPECT_FALSE(m1 + base::StateManifoldPtr() == m2 * s);
    EXPECT_TRUE(base::StateManifoldPtr() * s + m1 == m1);
    EXPECT_TRUE(m3 * base::StateManifoldPtr() + m1 == m1);
    EXPECT_TRUE(base::StateManifoldPtr() * base::StateManifoldPtr() + m4 == m4);
    EXPECT_TRUE(base::StateManifoldPtr() + base::StateManifoldPtr() + m4 == m4);
    EXPECT_TRUE(base::StateManifoldPtr() - base::StateManifoldPtr() + m4 == m4);
    EXPECT_TRUE((base::StateManifoldPtr() - "")->getDimension() == 0);

    EXPECT_EQ(s->getDimension(), m1->getDimension() + m2->getDimension() + m3->getDimension());
    base::StateManifoldPtr d = s - m2;
    EXPECT_EQ(d->getDimension(), m1->getDimension() + m3->getDimension());
    EXPECT_TRUE((s + d)->getDimension() == s->getDimension());

    m4->setName("test");
    EXPECT_TRUE(m4->getName() == "test");

    base::StateManifoldPtr t = m1 + m4;
    EXPECT_EQ((t - "test")->getDimension(), m1->getDimension());
    EXPECT_EQ((m1 - m1)->getDimension(), 0u);
    t->setName(t->getName());
    base::ScopedState<> st(t);
    EXPECT_TRUE(t->getValueAddressAtIndex(st.get(), 10000) == NULL);
    EXPECT_TRUE(t->includes(m1));
    EXPECT_FALSE(t->includes(m2));
    EXPECT_FALSE(m1->includes(t));
    EXPECT_TRUE(m3->includes(m3));
    EXPECT_TRUE(t->includes(t));
    base::StateManifold::diagram(std::cout);
    
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
