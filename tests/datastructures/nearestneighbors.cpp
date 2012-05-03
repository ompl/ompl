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

/* Author: Mark Moll */

#define BOOST_TEST_MODULE "NearestNeighbors"
#include <boost/test/unit_test.hpp>

#include <algorithm>
#include <boost/unordered_set.hpp>

#include "ompl/datastructures/NearestNeighborsSqrtApprox.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/SE3StateSpace.h"

using namespace ompl;

// define a convenience macro
#define BOOST_OMPL_EXPECT_NEAR(a, b, diff) BOOST_CHECK_SMALL((a) - (b), diff)

template<typename T>
double distance(const T* space, base::State* const &s0, base::State* const &s1)
{
    return space->distance(s0, s1);
}

double intDistance(int i, int j)
{
    return fabs(double(i-j));
}

void intTest(NearestNeighbors<int>& proximity, bool approximate=false)
{
    RNG rng;
    int i, j, n = 200, s;
    std::vector<int> states(n), nghbr, nghbrGroundTruth;
    NearestNeighborsLinear<int> proximityLinear;

    proximity.setDistanceFunction(intDistance);
    proximityLinear.setDistanceFunction(intDistance);
    for (i=0; i<n; ++i)
        states[i] = rng.uniformInt(0,20);
    proximity.add(states);
    if (!approximate)
        proximityLinear.add(states);

    BOOST_CHECK_EQUAL((int)proximity.size(), n);

    proximity.list(nghbr);
    BOOST_CHECK_EQUAL(nghbr.size(),proximity.size());

    for(i=0,j=0; i<n; ++i)
    {
        s = proximity.nearest(states[i]);
        if (s==states[i]) j++;

        proximity.nearestK(states[i], 10, nghbr);
        BOOST_CHECK_EQUAL(nghbr[0], states[i]);
        BOOST_CHECK_EQUAL(nghbr.size(), 10u);
        if (!approximate)
        {
            proximityLinear.nearestK(states[i], nghbr.size(), nghbrGroundTruth);
            for (unsigned int k=0; k<nghbr.size(); ++k)
                BOOST_CHECK_EQUAL(intDistance(states[i], nghbrGroundTruth[k]), intDistance(states[i], nghbr[k]));
        }

        proximity.nearestR(states[i], std::numeric_limits<double>::infinity(), nghbr);
        BOOST_CHECK_EQUAL(nghbr[0], states[i]);
        BOOST_CHECK_EQUAL(nghbr.size(),proximity.size());
        if (!approximate)
        {
            proximityLinear.nearestR(states[i], std::numeric_limits<double>::infinity(), nghbrGroundTruth);
            for (unsigned int k=0; k<nghbr.size(); ++k)
                BOOST_CHECK_EQUAL(intDistance(states[i], nghbrGroundTruth[k]), intDistance(states[i], nghbr[k]));
        }

        proximity.nearestK(states[i], 2*n, nghbr);
        BOOST_CHECK_EQUAL(nghbr[0], states[i]);
        BOOST_CHECK_EQUAL((int) nghbr.size(), n);
        if (!approximate)
        {
            proximityLinear.nearestK(states[i], 2*n, nghbrGroundTruth);
            for (unsigned int k=0; k<nghbr.size(); ++k)
                BOOST_CHECK_EQUAL(intDistance(states[i], nghbrGroundTruth[k]), intDistance(states[i], nghbr[k]));
        }

    }
    BOOST_CHECK_GE(j, 10);

    for(i=n-1; i>=0; --i)
    {
        proximity.remove(states[i]);
        BOOST_CHECK_EQUAL((int)proximity.size(), i);
    }
    try
    {
        s = proximity.nearest(states[0]);
    }
    catch (std::exception& e)
    {
        BOOST_CHECK_EQUAL("No elements found", e.what());
    }
}

void stateTest(NearestNeighbors<base::State*>& proximity, bool approximate=false)
{
    int i, j, n = 200;
    base::SE3StateSpace SE3;
    base::StateSamplerPtr sampler;
    base::RealVectorBounds b(3);
    std::vector<base::State*> states(n), nghbr, nghbrGroundTruth;
    NearestNeighborsLinear<base::State*> proximityLinear;
    base::State* s;
    double eps = 1e-6;

    b.setLow(0);
    b.setHigh(1);
    SE3.setBounds(b);
    sampler = SE3.allocStateSampler();

    proximity.setDistanceFunction(boost::bind(&distance<base::SE3StateSpace>, &SE3, _1, _2));
    proximityLinear.setDistanceFunction(boost::bind(&distance<base::SE3StateSpace>, &SE3, _1, _2));

    for(i=0; i<n; ++i)
    {
        states[i] = SE3.allocState();
        sampler->sampleUniform(states[i]);
    }
    proximity.add(states);
    proximityLinear.add(states);

    BOOST_CHECK_EQUAL((int)proximity.size(), n);

    proximity.list(nghbr);
    BOOST_CHECK_EQUAL(nghbr.size(),proximity.size());


    for(i=0,j=0; i<n; ++i)
    {
        s = proximity.nearest(states[i]);
        if (s==states[i]) j++;

        proximity.nearestK(states[i], 10, nghbr);
        BOOST_CHECK_EQUAL(nghbr[0], states[i]);
        BOOST_CHECK_EQUAL(nghbr.size(), 10u);
        if (!approximate)
        {
            proximityLinear.nearestK(states[i], nghbr.size(), nghbrGroundTruth);
            for (unsigned int k=0; k<nghbr.size(); ++k)
                BOOST_OMPL_EXPECT_NEAR(distance(&SE3, states[i], nghbrGroundTruth[k]), distance(&SE3, states[i], nghbr[k]), eps);
        }

        proximity.nearestR(states[i], std::numeric_limits<double>::infinity(), nghbr);
        BOOST_CHECK_EQUAL(nghbr[0], states[i]);
        BOOST_CHECK_EQUAL(nghbr.size(),proximity.size());
        if (!approximate)
        {
            proximityLinear.nearestR(states[i], nghbr.size(), nghbrGroundTruth);
            for (unsigned int k=0; k<nghbr.size(); ++k)
                BOOST_OMPL_EXPECT_NEAR(distance(&SE3, states[i], nghbrGroundTruth[k]), distance(&SE3, states[i], nghbr[k]), eps);
        }

        proximity.nearestK(states[i], 2*n, nghbr);
        BOOST_CHECK_EQUAL(nghbr[0], states[i]);
        BOOST_CHECK_EQUAL((int) nghbr.size(), n);
        if (!approximate)
        {
            proximityLinear.nearestK(states[i], nghbr.size(), nghbrGroundTruth);
            for (unsigned int k=0; k<nghbr.size(); ++k)
                BOOST_OMPL_EXPECT_NEAR(distance(&SE3, states[i], nghbrGroundTruth[k]), distance(&SE3, states[i], nghbr[k]), eps);
        }
    }
    BOOST_CHECK_GE(j, 10);

    for(i=n-1; i>=0; --i)
    {
        proximity.remove(states[i]);
        BOOST_CHECK_EQUAL((int)proximity.size(), i);
    }
    try
    {
        s = proximity.nearest(states[0]);
    }
    catch (std::exception& e)
    {
        BOOST_CHECK_EQUAL("No elements found", e.what());
    }
}

void randomAccessPatternIntTest(NearestNeighbors<int>& proximity)
{
    RNG rng;
    int i, j, k, m = 200, n = 10;
    unsigned int p;
    std::vector<int> nghbr, nghbrGroundTruth, lst, lstGroundTruth;
    NearestNeighborsLinear<int> proximityLinear;
    boost::unordered_multiset<int> states;
    boost::unordered_multiset<int>::iterator it;
    int s;
    double eps = 1e-6, r;

    proximity.setDistanceFunction(intDistance);
    proximityLinear.setDistanceFunction(intDistance);

    for (i=0; i<m; ++i)
    {
        for (j=0; j<n; ++j)
        {
            s = rng.uniformInt(0,20);
            proximity.add(s);
            proximityLinear.add(s);
            states.insert(s);
        }

        for (j=0; j<n; ++j)
        {
            s = rng.uniformInt(0,20);

            k = rng.uniformInt(1, 20);
            proximityLinear.nearestK(s, k, nghbrGroundTruth);
            proximity.nearestK(s, k, nghbr);
            BOOST_CHECK_EQUAL(nghbr.size(), nghbrGroundTruth.size());
            for (p=0; p<nghbr.size(); ++p)
                BOOST_OMPL_EXPECT_NEAR(intDistance(s, nghbrGroundTruth[p]), intDistance(s, nghbr[p]), eps);

            r = rng.uniformReal(0, 3);
            proximityLinear.nearestR(s, r, nghbrGroundTruth);
            proximity.nearestR(s, r, nghbr);
            BOOST_CHECK_EQUAL(nghbr.size(), nghbrGroundTruth.size());
            for (p=0; p<nghbr.size(); ++p)
                BOOST_OMPL_EXPECT_NEAR(intDistance(s, nghbrGroundTruth[p]), intDistance(s, nghbr[p]), eps);
        }

        for (it=states.begin(); it!=states.end();)
        {
            if (rng.uniform01()<.5)
            {
                assert(proximityLinear.remove(*it));
                assert(proximity.remove(*it));
                it = states.erase(it);
            }
            else
                ++it;
        }
    }
}

void randomAccessPatternStateTest(NearestNeighbors<base::State*>& proximity)
{
    RNG rng;
    int i, j, k, m = 200, n = 10;
    unsigned int p;
    base::SE3StateSpace SE3;
    base::StateSamplerPtr sampler;
    base::RealVectorBounds b(3);
    std::vector<base::State*> nghbr, nghbrGroundTruth;
    NearestNeighborsLinear<base::State*> proximityLinear;
    boost::unordered_set<base::State*> states;
    boost::unordered_set<base::State*>::iterator it;
    base::State* s;
    double eps = 1e-6, r;

    b.setLow(0);
    b.setHigh(1);
    SE3.setBounds(b);
    sampler = SE3.allocStateSampler();

    proximity.setDistanceFunction(boost::bind(&distance<base::SE3StateSpace>, &SE3, _1, _2));
    proximityLinear.setDistanceFunction(boost::bind(&distance<base::SE3StateSpace>, &SE3, _1, _2));

    for (i=0; i<m; ++i)
    {
        for (j=0; j<n; ++j)
        {
            s = SE3.allocState();
            sampler->sampleUniform(s);
            proximity.add(s);
            proximityLinear.add(s);
            states.insert(s);
        }

        for (j=0; j<n; ++j)
        {
            s = SE3.allocState();
            sampler->sampleUniform(s);

            k = rng.uniformInt(1, 20);
            proximityLinear.nearestK(s, k, nghbrGroundTruth);
            proximity.nearestK(s, k, nghbr);
            BOOST_CHECK_EQUAL(nghbr.size(), nghbrGroundTruth.size());
            for (p=0; p<nghbr.size(); ++p)
                BOOST_OMPL_EXPECT_NEAR(distance(&SE3, s, nghbrGroundTruth[p]), distance(&SE3, s, nghbr[p]), eps);

            r = rng.uniformReal(0, 3);
            proximityLinear.nearestR(s, r, nghbrGroundTruth);
            proximity.nearestR(s, r, nghbr);
            BOOST_CHECK_EQUAL(nghbr.size(), nghbrGroundTruth.size());
            for (p=0; p<nghbr.size(); ++p)
                BOOST_OMPL_EXPECT_NEAR(distance(&SE3, s, nghbrGroundTruth[p]), distance(&SE3, s, nghbr[p]), eps);

            SE3.freeState(s);
        }

        for (it=states.begin(); it!=states.end();)
        {
            if (rng.uniform01()<.5)
            {
                proximityLinear.remove(*it);
                proximity.remove(*it);
                it = states.erase(it);
            }
            else
                ++it;
        }
    }
}

BOOST_AUTO_TEST_CASE(IntLinear)
{
    NearestNeighborsLinear<int> proximity;
    intTest(proximity, true);
}

BOOST_AUTO_TEST_CASE(StateLinear)
{
    NearestNeighborsLinear<base::State*> proximity;
    stateTest(proximity, true);
}

BOOST_AUTO_TEST_CASE(IntSqrtLinear)
{
    NearestNeighborsSqrtApprox<int> proximity;
    intTest(proximity, true);
}

BOOST_AUTO_TEST_CASE(StateSqrtLinear)
{
    NearestNeighborsSqrtApprox<base::State*> proximity;
    stateTest(proximity, true);
}

BOOST_AUTO_TEST_CASE(IntGNAT)
{
    NearestNeighborsGNAT<int> proximity;
    intTest(proximity);
}

BOOST_AUTO_TEST_CASE(StateGNAT)
{
    NearestNeighborsGNAT<base::State*> proximity;
    stateTest(proximity);
}

BOOST_AUTO_TEST_CASE(RandomAccessPatternIntGNAT)
{
    NearestNeighborsGNAT<int> proximity(4,2,6,5);
    randomAccessPatternIntTest(proximity);
}

BOOST_AUTO_TEST_CASE(RandomAccessPatternStateGNAT)
{
    NearestNeighborsGNAT<base::State*> proximity(4,2,6,5);
    randomAccessPatternStateTest(proximity);
}

