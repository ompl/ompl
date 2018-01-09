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
#include <unordered_set>

#include "ompl/config.h"
#include "ompl/datastructures/NearestNeighborsSqrtApprox.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include "ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h"
#if OMPL_HAVE_FLANN
#include "ompl/datastructures/NearestNeighborsFLANN.h"
#endif
#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/DiscreteStateSpace.h"
#include "ompl/base/spaces/SE3StateSpace.h"

using namespace ompl;

// define a convenience macro
#define BOOST_OMPL_EXPECT_NEAR(a, b, diff) BOOST_CHECK_SMALL((a) - (b), diff)

const int n = 200; // # random numbers in test set
const int approx_correct = n/20; // number of times an approximate method has to return exact nearest neighbor on n trials
const int k = 10; // # nearest neighbors
const int maxk = 30; // max. nearest neighbors (when sampling random values for k)
// Integers will be sampled in the range of [0,range].
// Since we are actually storing pointers to ints, there are issues with
// storing and removing the same value multiple times, so range is chosen
// to be "large" to minimize the probability of this happening.
const int range = std::numeric_limits<int>::max();
const double eps = 1e-6; // tolerance on distance for equality test

// fixture
struct NearestNeighborConfig
{
    NearestNeighborConfig() : space0(0, range)
    {
        base::RealVectorBounds b(3);
        b.setLow(0);
        b.setHigh(1);
        space1.setBounds(b);
    }
    ~NearestNeighborConfig() = default;

    base::DiscreteStateSpace space0;
    base::SE3StateSpace     space1;
};

// a GNAT with a small number of data points per leaf and small cache
// for removed points, so that more corner cases will be hit by the tests.
template<typename _T>
class NearestNeighborsGNATs : public NearestNeighborsGNAT<_T>
{
public:
    NearestNeighborsGNATs() : NearestNeighborsGNAT<_T>(4,2,6,5,5)
    {
    }
};
template<typename _T>
class NearestNeighborsGNATNoThreadSafetys : public NearestNeighborsGNATNoThreadSafety<_T>
{
public:
    NearestNeighborsGNATNoThreadSafetys() : NearestNeighborsGNATNoThreadSafety<_T>(4,2,6,5,5)
    {
    }
};


NearestNeighborConfig nnConfig;

// helper function to determine if a state is stored in a vector of states
bool find(base::State* s, std::vector<base::State*> states)
{
    for (auto & state : states)
        if (s == state)
            return true;
    return false;
}

void stateSpaceTest(base::StateSpace& space, NearestNeighbors<base::State*>& proximity, bool approximate=false)
{
    int i, j;
    base::StateSamplerPtr sampler(space.allocStateSampler());
    std::vector<base::State*> states(n), nghbr, nghbrGroundTruth;
    NearestNeighborsLinear<base::State*> proximityLinear;
    base::State* s;

    proximity.setDistanceFunction([&space](const base::State *a, const base::State *b)
        {
            return space.distance(a, b);
        });
    proximityLinear.setDistanceFunction([&space](const base::State *a, const base::State *b)
        {
            return space.distance(a, b);
        });

    for(i=0; i<n; ++i)
    {
        states[i] = space.allocState();
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
        if (space.distance(s,states[i])<eps) j++;

        proximity.nearestK(states[i], k, nghbr);
        BOOST_CHECK_EQUAL(nghbr.size(), (unsigned int)k);

        if (!approximate)
        {
            proximityLinear.nearestK(states[i], nghbr.size(), nghbrGroundTruth);
            for (unsigned int k=0; k<nghbr.size(); ++k)
                BOOST_OMPL_EXPECT_NEAR(space.distance(states[i], nghbrGroundTruth[k]),
                    space.distance(states[i], nghbr[k]), eps);
        }

        proximity.nearestR(states[i], std::numeric_limits<double>::infinity(), nghbr);
        BOOST_OMPL_EXPECT_NEAR(space.distance(nghbr[0], states[i]), 0., eps);
        if (!approximate)
            BOOST_CHECK_EQUAL(nghbr.size(), proximity.size());
        if (!approximate)
        {
            proximityLinear.nearestR(states[i], std::numeric_limits<double>::infinity(), nghbrGroundTruth);
            for (unsigned int k=0; k<nghbr.size(); ++k)
                BOOST_OMPL_EXPECT_NEAR(space.distance(states[i], nghbrGroundTruth[k]),
                    space.distance(states[i], nghbr[k]), eps);
        }

        proximity.nearestK(states[i], 2*n, nghbr);
        BOOST_CHECK_EQUAL((int) nghbr.size(), n);
        BOOST_CHECK(find(states[i], nghbr));
        if (!approximate)
        {
            proximityLinear.nearestK(states[i], nghbr.size(), nghbrGroundTruth);
            for (unsigned int k=0; k<nghbr.size(); ++k)
                BOOST_OMPL_EXPECT_NEAR(space.distance(states[i], nghbrGroundTruth[k]),
                    space.distance(states[i], nghbr[k]), eps);
        }
    }
    BOOST_CHECK_GE(j, approx_correct);

    for(i=n-1; i>=0; --i)
    {
        proximity.remove(states[i]);
        BOOST_CHECK_EQUAL((int)proximity.size(), i);
        proximity.list(nghbr);
        BOOST_CHECK_EQUAL((int)nghbr.size(), i);
    }
    try
    {
        s = proximity.nearest(states[0]);
    }
    catch (std::exception& e)
    {
        BOOST_CHECK_EQUAL("No elements found", std::string(e.what()).substr(0, 17));
    }

    for(i=0; i<n; ++i)
        space.freeState(states[i]);
}

void randomAccessPatternTest(base::StateSpace& space, NearestNeighbors<base::State*>& proximity)
{
    RNG rng;
    int i, j, k, m = n, n = m/20;
    unsigned int p;
    base::StateSamplerPtr sampler(space.allocStateSampler());
    std::vector<base::State*> nghbr, nghbrGroundTruth;
    NearestNeighborsLinear<base::State*> proximityLinear;
    std::unordered_set<base::State*> states;
    std::unordered_set<base::State*>::iterator it;
    base::State* s;
    double r;

    proximity.setDistanceFunction([&space](const base::State *a, const base::State *b)
        {
            return space.distance(a, b);
        });
    proximityLinear.setDistanceFunction([&space](const base::State *a, const base::State *b)
        {
            return space.distance(a, b);
        });

    for (i=0; i<m; ++i)
    {
        for (j=0; j<n; ++j)
        {
            s = space.allocState();
            sampler->sampleUniform(s);
            it = states.insert(s).first;
            proximity.add(*it);
            proximityLinear.add(*it);
        }

        for (j=0; j<n; ++j)
        {
            s = space.allocState();
            sampler->sampleUniform(s);

            k = rng.uniformInt(1, maxk);
            proximityLinear.nearestK(s, k, nghbrGroundTruth);
            proximity.nearestK(s, k, nghbr);
            BOOST_CHECK_EQUAL(nghbr.size(), nghbrGroundTruth.size());
            for (p=0; p<nghbr.size(); ++p)
                BOOST_OMPL_EXPECT_NEAR(space.distance(s, nghbrGroundTruth[p]), space.distance(s, nghbr[p]), eps);

            r = rng.uniformReal(0, 3);
            proximityLinear.nearestR(s, r, nghbrGroundTruth);
            proximity.nearestR(s, r, nghbr);
            BOOST_CHECK_EQUAL(nghbr.size(), nghbrGroundTruth.size());
            for (p=0; p<nghbr.size(); ++p)
                BOOST_OMPL_EXPECT_NEAR(space.distance(s, nghbrGroundTruth[p]), space.distance(s, nghbr[p]), eps);

            space.freeState(s);
        }

        for (it=states.begin(); it!=states.end(); )
        {
            if (rng.uniform01()<.5)
            {
                unsigned int szLinear = proximityLinear.size(), sz = proximity.size();
                /* bool removedLinear = */ proximityLinear.remove(*it);
                bool removed = proximity.remove(*it);
                space.freeState(*it);
                it = states.erase(it);
                BOOST_CHECK(removed);
                BOOST_CHECK_EQUAL(proximity.size(), sz-1);
                BOOST_CHECK_EQUAL(sz, szLinear);
                BOOST_CHECK_EQUAL(proximity.size(), states.size());
                proximity.list(nghbr);
                proximityLinear.list(nghbrGroundTruth);
                BOOST_CHECK_EQUAL(nghbr.size(), nghbrGroundTruth.size());
            }
            else
                ++it;
        }
    }

    for(it = states.begin(); it != states.end(); ++it)
        space.freeState(*it);
}

#define NN_TEST_CASES(T,approx)                          \
BOOST_AUTO_TEST_CASE(Int##T)                             \
{                                                        \
    NearestNeighbors##T<base::State*> proximity;         \
    stateSpaceTest(nnConfig.space0, proximity, approx);  \
}                                                        \
BOOST_AUTO_TEST_CASE(SE3##T)                             \
{                                                        \
    NearestNeighbors##T<base::State*> proximity;         \
    stateSpaceTest(nnConfig.space1, proximity, approx);  \
}                                                        \
BOOST_AUTO_TEST_CASE(RandomAccessPatternInt##T)          \
{                                                        \
    NearestNeighbors##T<base::State*> proximity;         \
    randomAccessPatternTest(nnConfig.space0, proximity); \
}                                                        \
BOOST_AUTO_TEST_CASE(RandomAccessPatternSE3##T)          \
{                                                        \
    NearestNeighbors##T<base::State*> proximity;         \
    randomAccessPatternTest(nnConfig.space1, proximity); \
}

NN_TEST_CASES(Linear, false)
NN_TEST_CASES(SqrtApprox, true)
NN_TEST_CASES(GNATs, false)
NN_TEST_CASES(GNATNoThreadSafetys, false)
#if OMPL_HAVE_FLANN
NN_TEST_CASES(FLANNLinear, false)
NN_TEST_CASES(FLANNHierarchicalClustering, true)
#endif
