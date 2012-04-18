/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Rice University
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

/* Author: Ryan Luna */

#include <gtest/gtest.h>
#include <iostream>
#include <vector>

#include "ompl/base/PlannerData.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

using namespace ompl;

TEST(PlannerData, SimpleConstruction)
{
    base::StateSpacePtr space(new base::RealVectorStateSpace(1));
    base::PlannerData data;
    std::vector<base::State*> states;

    // Creating states
    for (unsigned int i = 0; i < 10; ++i)
        states.push_back(space->allocState());

    // Adding vertices
    for (unsigned int i = 1; i <= 10; ++i)
    {
        unsigned int vtx = data.addVertex(base::PlannerDataVertex(states[i-1], i));

        EXPECT_NE( vtx, std::numeric_limits<unsigned int>::max() );
        EXPECT_EQ( data.addVertex(base::PlannerDataVertex(states[i-1], i)), vtx );

        EXPECT_EQ( data.vertexIndex(base::PlannerDataVertex(states[i-1], i)), i-1 );
        EXPECT_TRUE( data.vertexExists(base::PlannerDataVertex(states[i-1], i)) );
    }

    // We should have 10 vertices and 0 edges
    EXPECT_EQ( data.numVertices(), 10u );
    EXPECT_EQ( data.numEdges(), 0u );

    // Adding some edges
    for (unsigned int i = 1; i <= 9; ++i)
    {
        EXPECT_TRUE( data.addEdge(i-1, i) );
        EXPECT_FALSE( data.addEdge(i-1, 11) );  // vertex 11 does not exist

        EXPECT_TRUE( data.edgeExists(i-1, i) );
    }

    // We should have 10 vertices and 9 edges at this point
    EXPECT_EQ( data.numVertices(), 10u );
    EXPECT_EQ( data.numEdges(), 9u );

    // Make sure our edges are where we think they are
    for (unsigned int i = 0; i < 9; ++i)
    {
        std::vector<unsigned int> neighbors;
        EXPECT_EQ( data.getEdges(i, neighbors), 1);
        EXPECT_EQ( neighbors[0], i+1 );
    }

    std::vector<unsigned int> neighbors;
    EXPECT_EQ( data.getEdges(9, neighbors), 0 );

    // Make sure that the vertices can be retrived properly
    for (unsigned int i = 0; i < 10; ++i)
    {
        EXPECT_EQ(data.getVertex(i).getState(), states[i]);
        EXPECT_EQ(data.getVertex(i).getTag(), i+1);
    }

    for (size_t i = 0; i < states.size(); ++i)
        space->freeState(states[i]);
}

TEST(PlannerData, AdvancedConstruction)
{
    base::StateSpacePtr space(new base::RealVectorStateSpace(1));
    base::PlannerData data;
    std::vector<base::State*> states;

    // Creating states
    for (unsigned int i = 0; i < 10; ++i)
        states.push_back(space->allocState());

    // Adding vertices and edges simultaneously
    for (unsigned int i = 1; i < 10; ++i)
    {
        EXPECT_TRUE( data.addEdge (base::PlannerDataVertex(states[i-1], i),
                                   base::PlannerDataVertex(states[i], i+1)) );

        // Duplicates are not allowed
        EXPECT_FALSE( data.addEdge (base::PlannerDataVertex(states[i-1], i),
                                    base::PlannerDataVertex(states[i], i+1)) );

        EXPECT_TRUE( data.edgeExists (i-1, i) );

        EXPECT_EQ( data.vertexIndex(base::PlannerDataVertex(states[i-1], i)), i-1 );
        EXPECT_TRUE( data.vertexExists(base::PlannerDataVertex(states[i-1], i)) );
    }

    // We should have 10 vertices and 9 edges
    EXPECT_EQ( data.numVertices(), 10u );
    EXPECT_EQ( data.numEdges(), 9u );

    // Make sure our edges are where we think they are
    for (unsigned int i = 0; i < 9; ++i)
    {
        std::vector<unsigned int> nbrs;
        EXPECT_EQ( data.getEdges(i, nbrs), 1);
        EXPECT_EQ( nbrs[0], i+1 );
    }
    std::vector<unsigned int> nbrs;
    EXPECT_EQ( data.getEdges(9, nbrs), 0 );

    // Make sure that the vertices can be retrived properly
    for (unsigned int i = 0; i < 10; ++i)
    {
        EXPECT_EQ(data.getVertex(i).getState(), states[i]);
        EXPECT_EQ(data.getVertex(i).getTag(), i+1);
    }

    // Reset the tag for state #0
    EXPECT_TRUE( data.tagState(states[0], 10000) );
    EXPECT_EQ( data.getVertex(0).getTag(), 10000 );
    EXPECT_FALSE( data.tagState(0, 1000) );

    // Reset the edge weight for 0->1
    EXPECT_TRUE( data.setEdgeWeight(0, 1, 1.234) );
    EXPECT_NEAR( data.getEdgeWeight(0, 1), 1.234, 1e-4);

    EXPECT_NEAR( data.getEdgeWeight(0, 5), -1.0, 1e-4 ); // edge does not exist
    EXPECT_FALSE( data.setEdgeWeight(0, 5, 2.345) );

    // Try to tag an invalid state
    EXPECT_FALSE( data.tagState(0, 100) );

    for (size_t i = 0; i < states.size(); ++i)
        space->freeState(states[i]);
}

TEST(PlannerData, AddRemoveVerticesAndEdges)
{
    base::StateSpacePtr space(new base::RealVectorStateSpace(1));
    base::PlannerData data;
    std::vector<base::State*> states;

    // Creating states
    for (unsigned int i = 0; i < 10; ++i)
        states.push_back(space->allocState());

    // Adding vertices and edges
    for (unsigned int i = 1; i < 10; ++i)
    {
        EXPECT_TRUE( data.addEdge (base::PlannerDataVertex(states[i-1], i),
                                   base::PlannerDataVertex(states[i], i+1)) );
    }

    EXPECT_EQ( data.numVertices(), 10u );
    EXPECT_EQ( data.numEdges(), 9u );

    EXPECT_TRUE( data.removeVertex(base::PlannerDataVertex(states[9])) );
    EXPECT_EQ( data.numVertices(), 9u );
    EXPECT_EQ( data.numEdges(), 8u );

    EXPECT_FALSE( data.removeVertex(base::PlannerDataVertex(states[9])) ); // we already removed this vertex
    EXPECT_EQ( data.numVertices(), 9u );
    EXPECT_EQ( data.numEdges(), 8u );

    EXPECT_FALSE( data.removeVertex(10) ); // vertex does not exist
    EXPECT_EQ( data.numVertices(), 9u );
    EXPECT_EQ( data.numEdges(), 8u );

    EXPECT_TRUE( data.removeVertex(6) );
    EXPECT_EQ( data.numVertices(), 8u );
    EXPECT_EQ( data.numEdges(), 6u );  // incoming edge and outgoing edge should be removed

    EXPECT_TRUE( data.removeEdge(base::PlannerDataVertex(states[1]), base::PlannerDataVertex(states[2])) );
    EXPECT_EQ( data.numVertices(), 8u );
    EXPECT_EQ( data.numEdges(), 5u );

    EXPECT_FALSE( data.removeEdge(base::PlannerDataVertex(states[1]), base::PlannerDataVertex(states[2])) ); // we just removed this edge
    EXPECT_EQ( data.numVertices(), 8u );
    EXPECT_EQ( data.numEdges(), 5u );

    EXPECT_FALSE( data.removeEdge(5, 6) ); // edge does not exist
    EXPECT_EQ( data.numVertices(), 8u );
    EXPECT_EQ( data.numEdges(), 5u );

    EXPECT_TRUE( data.removeEdge(4, 5) );
    EXPECT_EQ( data.numVertices(), 8u );
    EXPECT_EQ( data.numEdges(), 4u );

    // Make sure the final graph looks the way we think:
    EXPECT_TRUE( data.vertexExists(base::PlannerDataVertex(states[0])) );
    EXPECT_TRUE( data.vertexExists(base::PlannerDataVertex(states[1])) );
    EXPECT_TRUE( data.vertexExists(base::PlannerDataVertex(states[2])) );
    EXPECT_TRUE( data.vertexExists(base::PlannerDataVertex(states[3])) );
    EXPECT_TRUE( data.vertexExists(base::PlannerDataVertex(states[4])) );
    EXPECT_TRUE( data.vertexExists(base::PlannerDataVertex(states[5])) );
    EXPECT_FALSE( data.vertexExists(base::PlannerDataVertex(states[6])) );  // we removed this vertex
    EXPECT_TRUE( data.vertexExists(base::PlannerDataVertex(states[7])) );
    EXPECT_TRUE( data.vertexExists(base::PlannerDataVertex(states[8])) );
    EXPECT_FALSE( data.vertexExists(base::PlannerDataVertex(states[9])) );  // and this one

    // Checking edges by index and vertex
    EXPECT_TRUE( data.edgeExists(0, 1) );
    EXPECT_TRUE( data.edgeExists(2, 3) );
    EXPECT_TRUE( data.edgeExists(3, 4) );
    EXPECT_TRUE( data.edgeExists(6, 7) );

    EXPECT_TRUE( data.edgeExists(data.vertexIndex(base::PlannerDataVertex(states[0])), data.vertexIndex(base::PlannerDataVertex(states[1]))) );
    EXPECT_TRUE( data.edgeExists(data.vertexIndex(base::PlannerDataVertex(states[2])), data.vertexIndex(base::PlannerDataVertex(states[3]))) );
    EXPECT_TRUE( data.edgeExists(data.vertexIndex(base::PlannerDataVertex(states[3])), data.vertexIndex(base::PlannerDataVertex(states[4]))) );
    EXPECT_TRUE( data.edgeExists(data.vertexIndex(base::PlannerDataVertex(states[7])), data.vertexIndex(base::PlannerDataVertex(states[8]))) );

    std::vector<unsigned int> nbrs;
    EXPECT_EQ ( data.getEdges(0, nbrs), 1 );
    EXPECT_EQ ( data.getEdges(1, nbrs), 0 );
    EXPECT_EQ ( data.getEdges(2, nbrs), 1 );
    EXPECT_EQ ( data.getEdges(3, nbrs), 1 );
    EXPECT_EQ ( data.getEdges(4, nbrs), 0 );
    EXPECT_EQ ( data.getEdges(5, nbrs), 0 );
    EXPECT_EQ ( data.getEdges(6, nbrs), 1 );
    EXPECT_EQ ( data.getEdges(7, nbrs), 0 );

    for (size_t i = 0; i < states.size(); ++i)
        space->freeState(states[i]);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
