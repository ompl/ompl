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
#include "ompl/datastructures/NearestNeighborsSqrtApprox.h"
#include "ompl/base/ScopedState.h"
#include "ompl/base/manifolds/SE3StateManifold.h"

using namespace ompl;

template<typename T>
double distance(const T* manifold, base::State* const &s0, base::State* const &s1)
{
    return manifold->distance(s0,s1);
}

TEST(NearestNeighbors, Linear)
{
    int i, n=1000;
    base::SE3StateManifold SE3;
    base::ManifoldStateSamplerPtr sampler;
    base::RealVectorBounds b(3);
    std::vector<base::State*> states(n), nghbr(10);
    NearestNeighborsLinear<base::State*> proximity;
    base::State* s;
    
    b.setLow(0);
    b.setHigh(1);
    SE3.setBounds(b);
    sampler = SE3.allocStateSampler();

    proximity.setDistanceFunction(boost::bind(&distance<base::SE3StateManifold>, &SE3, _1, _2));
    
    for(i=0; i<n; ++i)
    {
        states[i] = SE3.allocState();
        sampler->sampleUniform(states[i]);
        proximity.add(states[i]);
    }
    
    EXPECT_EQ(proximity.size(), n);
    
    for(i=0; i<n; ++i)
    {
        s = proximity.nearest(states[i]);
        EXPECT_EQ(s, states[i]);
        
        proximity.nearestK(states[i], 10, nghbr);
        EXPECT_EQ(nghbr[0], states[i]);
        EXPECT_EQ(nghbr.size(), 10);
        
        proximity.nearestR(states[i], 1000., nghbr);
        EXPECT_EQ(nghbr[0], states[i]);
        EXPECT_EQ(nghbr.size(),proximity.size());
    }
    
    proximity.list(nghbr);
    EXPECT_EQ(nghbr.size(),proximity.size());
    for(i=0; i<n; ++i)
        EXPECT_EQ(states[i], nghbr[i]);
        
    for(i=n-1; i>=0; --i)
    {
        proximity.remove(states[i]);
        EXPECT_EQ(proximity.size(), i);
    }
    try
    {
        s = proximity.nearest(states[0]);
    }
    catch (std::exception& e)
    {
        EXPECT_STREQ("No elements found", e.what());
    }
}

TEST(NearestNeighbors, SqrtApprox)
{
    int i, j, n=1000;
    base::SE3StateManifold SE3;
    base::ManifoldStateSamplerPtr sampler;
    base::RealVectorBounds b(3);
    std::vector<base::State*> states(n), nghbr(10);
    NearestNeighborsSqrtApprox<base::State*> proximity;
    base::State* s;
    
    b.setLow(0);
    b.setHigh(1);
    SE3.setBounds(b);
    sampler = SE3.allocStateSampler();

    proximity.setDistanceFunction(boost::bind(&distance<base::SE3StateManifold>, &SE3, _1, _2));
    
    for(i=0; i<n; ++i)
    {
        states[i] = SE3.allocState();
        sampler->sampleUniform(states[i]);
        proximity.add(states[i]);
    }
    
    EXPECT_EQ(proximity.size(), n);
    
    for(i=0,j=0; i<n; ++i)
    {
        s = proximity.nearest(states[i]);
        if (s==states[i]) j++;
        
        proximity.nearestK(states[i], 10, nghbr);
        EXPECT_EQ(nghbr[0], states[i]);
        EXPECT_EQ(nghbr.size(), 10);
        
        proximity.nearestR(states[i], 1000., nghbr);
        EXPECT_EQ(nghbr[0], states[i]);
        EXPECT_EQ(nghbr.size(),proximity.size());
    }
    EXPECT_GE(j, 10);
    
    proximity.list(nghbr);
    EXPECT_EQ(nghbr.size(),proximity.size());
    for(i=0; i<n; ++i)
        EXPECT_EQ(states[i], nghbr[i]);

    for(i=n-1; i>=0; --i)
    {
        proximity.remove(states[i]);
        EXPECT_EQ(proximity.size(), i);
    }
    try
    {
        s = proximity.nearest(states[0]);
    }
    catch (std::exception& e)
    {
        EXPECT_STREQ("No elements found", e.what());
    }
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
