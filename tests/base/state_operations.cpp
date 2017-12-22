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

#define BOOST_TEST_MODULE "State"
#include <boost/test/unit_test.hpp>
#include <thread>
#include <iostream>

#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/SE3StateSpace.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Time.h"

using namespace ompl;

// define a convenience macro
#define BOOST_OMPL_EXPECT_NEAR(a, b, diff) BOOST_CHECK_SMALL((a) - (b), diff)

BOOST_AUTO_TEST_CASE(Scoped)
{
    auto mSE3(std::make_shared<base::SE3StateSpace>());

    base::RealVectorBounds b(3);
    b.setLow(0);
    b.setHigh(1);
    mSE3->setBounds(b);
    mSE3->setup();

    auto mC0(std::make_shared<base::CompoundStateSpace>());
    mC0->addSubspace(mSE3, 1.0);
    mC0->setup();

    auto mC1(std::make_shared<base::CompoundStateSpace>());
    mC1->addSubspace(mC0, 1.0);
    mC1->setup();

    auto mC2(std::make_shared<base::CompoundStateSpace>());
    mC2->addSubspace(mSE3->getSubspace(1), 1.0);
    mC2->addSubspace(mSE3->getSubspace(0), 1.0);
    mC2->setup();

    base::ScopedState<base::SE3StateSpace> sSE3(mSE3);
    base::ScopedState<base::RealVectorStateSpace> sSE3_R(mSE3->getSubspace(0));
    base::ScopedState<base::SO3StateSpace> sSE3_SO2(mSE3->getSubspace(1));
    base::ScopedState<base::CompoundStateSpace> sC0(mC0);
    base::ScopedState<> sC1(mC1);
    base::ScopedState<> sC2(mC2);

    sSE3.random();

    sSE3 >> sSE3_SO2;

    BOOST_CHECK_EQUAL(sSE3->rotation().x, sSE3_SO2->x);
    BOOST_CHECK_EQUAL(sSE3->rotation().y, sSE3_SO2->y);
    BOOST_CHECK_EQUAL(sSE3->rotation().z, sSE3_SO2->z);
    BOOST_CHECK_EQUAL(sSE3->rotation().w, sSE3_SO2->w);

    base::ScopedState<> sSE3_copy(mSE3);
    sSE3_copy << sSE3;
    BOOST_CHECK_EQUAL(sSE3_copy, sSE3);
    sSE3 >> sSE3_copy;
    BOOST_CHECK_EQUAL(sSE3_copy, sSE3);

    sSE3_R << sSE3_copy;

    BOOST_CHECK_EQUAL(sSE3->getX(), sSE3_R->values[0]);
    BOOST_CHECK_EQUAL(sSE3->getY(), sSE3_R->values[1]);
    BOOST_CHECK_EQUAL(sSE3->getZ(), sSE3_R->values[2]);

    sSE3_SO2 >> sC1;
    sC1 << sSE3_R;

    sC1 >> sC0;
    sSE3_copy = sC0->components[0];
    BOOST_CHECK_EQUAL(sSE3_copy, sSE3);

    sSE3.random();

    sSE3 >> sC2;
    sSE3_copy << sC2;
    BOOST_CHECK_EQUAL(sSE3_copy, sSE3);


    sSE3.random();
    sSE3 >> sSE3_SO2;
    sSE3 >> sSE3_R;

    (sSE3_R ^ sSE3_SO2) >> sSE3_copy;
    BOOST_CHECK_EQUAL(sSE3_copy, sSE3);
    BOOST_CHECK_EQUAL(sSE3_copy[mSE3 * sSE3_R.getSpace()], sSE3_R);
    BOOST_CHECK_EQUAL(sSE3_copy[sSE3_SO2.getSpace()], sSE3_SO2);

    sSE3->setY(1.0);
    BOOST_OMPL_EXPECT_NEAR(sSE3.reals()[1], 1.0, 1e-12);

    BOOST_OMPL_EXPECT_NEAR(sSE3[1], 1.0, 1e-12);
    sSE3[2] = 0.1;
    BOOST_OMPL_EXPECT_NEAR(sSE3.reals()[2], 0.1, 1e-12);

    sSE3.random();
    std::vector<double> r = sSE3.reals();
    BOOST_CHECK_EQUAL(r.size(), 7u);
    sSE3_copy = r;
    BOOST_CHECK_EQUAL(sSE3_copy, sSE3);
    BOOST_CHECK_EQUAL(sSE3[6], r[6]);
    BOOST_CHECK_EQUAL(sSE3[0], r[0]);
    BOOST_CHECK_EQUAL(sSE3.getSpace()->getValueAddressAtIndex(sSE3.get(), 7), (double*)nullptr);

    sSE3_R = 0.5;
    BOOST_CHECK_EQUAL(sSE3_R[0], 0.5);

    sSE3 << sSE3_R;
    mSE3->setName("test");
    BOOST_CHECK_EQUAL(sSE3["test"], 0.5);
    sSE3["test"] = 0.1;
    BOOST_CHECK_EQUAL(sSE3[0], 0.1);
}

BOOST_AUTO_TEST_CASE(ScopedRV)
{
    auto m(std::make_shared<base::RealVectorStateSpace>(2));

    base::ScopedState<base::RealVectorStateSpace> s1(m);
    s1->values[0] = 1.0;
    s1->values[1] = 2.0;

    base::ScopedState<base::RealVectorStateSpace> s2 = s1;
    BOOST_CHECK(s2->values[1] == s1->values[1]);

    base::ScopedState<> s3(m);
    s3 = s1;
    base::ScopedState<> s4 = s3;
    BOOST_CHECK(s4 == s3);
    BOOST_CHECK(s4 == s1);

    base::ScopedState<base::RealVectorStateSpace> s5 = s2;
    BOOST_CHECK(s5 == s1);

    s1->values[1] = 4.0;

    BOOST_CHECK(s5 != s1);

    base::ScopedState<base::RealVectorStateSpace> s6(s5);
    BOOST_CHECK(s6 != s1);
    s1 = s5;
    s5 = s1;
    BOOST_CHECK(s6 == s1);
}

BOOST_AUTO_TEST_CASE(Allocation)
{
    auto m(std::make_shared<base::SE3StateSpace>());
    base::RealVectorBounds b(3);
    b.setLow(0);
    b.setHigh(1);
    m->setBounds(b);
    base::SpaceInformation si(m);
    si.setup();

    const unsigned int N = 30000;
    const unsigned int M = 20;
    std::vector<base::State*> states(N, nullptr);

    ompl::time::point start = ompl::time::now();
    for (unsigned int j = 0 ; j < M ; ++j)
    {
        for (unsigned int i = 0 ; i < N ; ++i)
            states[i] = si.allocState();

        for (unsigned int i = 0 ; i < N ; ++i)
            si.freeState(states[i]);
    }
    double d = ompl::time::seconds(ompl::time::now() - start);
    std::cout << (double)N * (double)M / d << " state allocations then frees per second" << std::endl;


    start = ompl::time::now();
    for (unsigned int j = 0 ; j < M ; ++j)
    {
        for (unsigned int i = 0 ; i < N ; ++i)
        {
            base::State *s = si.allocState();
            si.freeState(s);
        }
    }
    d = ompl::time::seconds(ompl::time::now() - start);
    std::cout << (double)N * (double)M / d << " mixed state allocations & frees per second" << std::endl;


    start = ompl::time::now();
    for (unsigned int j = 0 ; j < M ; ++j)
    {
        for (unsigned int i = 0 ; i < N ; ++i)
        {
            base::State *s = si.allocState();
            si.freeState(s);
            states[i] = si.allocState();
        }
        for (unsigned int i = 0 ; i < N ; ++i)
            si.freeState(states[i]);
    }
    d = ompl::time::seconds(ompl::time::now() - start);
    std::cout << (double)N * (double)M / d << " allocations per second" << std::endl;
}

void randomizedAllocator(const base::SpaceInformation *si)
{
    RNG r;
    const unsigned int n = 500;

    std::vector<base::State*> states(n + 1, nullptr);
    for (unsigned int i = 0 ; i < n * 1000 ; ++i)
    {
        int j = r.uniformInt(0, n);
        if (states[j] == nullptr)
            states[j] = si->allocState();
        else
        {
            si->freeState(states[j]);
            states[j] = nullptr;
        }
    }
    for (auto & state : states)
        if (state != nullptr)
            si->freeState(state);
}

BOOST_AUTO_TEST_CASE(AllocationWithThreads)
{
    auto m(std::make_shared<base::SE3StateSpace>());
    base::RealVectorBounds b(3);
    b.setLow(0);
    b.setHigh(1);
    m->setBounds(b);
    base::SpaceInformation si(m);
    si.setup();
    const int NT = 10;
    ompl::time::point start = ompl::time::now();
    std::vector<std::thread*> threads;
    for (int i = 0 ; i < NT ; ++i)
        threads.push_back(new std::thread([&si] { randomizedAllocator(&si); }));
    for (int i = 0 ; i < NT ; ++i)
    {
        threads[i]->join();
        delete threads[i];
    }
    std::cout << "Time spent randomly allocating & freeing states: "
        << ompl::time::seconds(ompl::time::now() - start) << std::endl;
}

BOOST_AUTO_TEST_CASE(PartialCopy)
{
    auto m(std::make_shared<base::SE3StateSpace>());
    base::RealVectorBounds b(3);
    b.setLow(0);
    b.setHigh(1);
    m->setBounds(b);
    m->setup();
    base::StateSpacePtr r3 = m->getSubspace(0);
    base::StateSpacePtr q = m->getSubspace(1);
    base::StateSamplerPtr s1 = m->base::StateSpace::allocSubspaceStateSampler(r3);
    base::StateSamplerPtr s2 = m->base::StateSpace::allocSubspaceStateSampler(q);
    base::ScopedState<base::SE3StateSpace> state(m);
    base::ScopedState<base::SE3StateSpace> tmp(m);
    std::vector<std::string> subspaces;
    m->getCommonSubspaces(m, subspaces);
    BOOST_CHECK(subspaces.size() == 1);
    BOOST_CHECK(subspaces[0] == m->getName());
    q->getCommonSubspaces(r3, subspaces);
    BOOST_CHECK(subspaces.empty());
    m->getCommonSubspaces(q, subspaces);
    BOOST_CHECK(subspaces.size() == 1);
    base::ScopedState<> dummy(q);

    for (int i = 0 ; i < 100 ; ++i)
    {
        state.random();
        tmp = state;
        s1->sampleUniform(tmp.get());
        BOOST_CHECK(tmp[q] == state[q]);
        BOOST_CHECK(tmp[r3] != state[r3]);
        s2->sampleUniform(tmp.get());
        BOOST_CHECK(tmp[q] != state[q]);
        BOOST_CHECK(copyStateData(m, state.get(), q, tmp[q].get(), subspaces) == base::ALL_DATA_COPIED);
        BOOST_CHECK(tmp[q] == state[q]);
        BOOST_CHECK(copyStateData(m, state.get(), q, tmp[q].get()) == base::ALL_DATA_COPIED);
        BOOST_CHECK(copyStateData(q, dummy.get(), m, state.get()) == base::SOME_DATA_COPIED);
        BOOST_CHECK(copyStateData(q, dummy.get(), r3, state[r3].get()) == base::NO_DATA_COPIED);
    }
}
