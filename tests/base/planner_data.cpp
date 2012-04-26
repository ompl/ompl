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

    // Creating 1000 states
    for (unsigned int i = 0; i < 1000; ++i)
        states.push_back(space->allocState());

    // Adding vertices
    for (unsigned int i = 0; i < states.size(); ++i)
    {
        unsigned int vtx = data.addVertex(base::PlannerDataVertex(states[i], (signed)i));

        EXPECT_NE( vtx, base::PlannerData::INVALID_INDEX );
        EXPECT_EQ( data.addVertex(base::PlannerDataVertex(states[i])), vtx );

        EXPECT_EQ( data.vertexIndex(base::PlannerDataVertex(states[i])), i );
        EXPECT_TRUE( data.vertexExists(base::PlannerDataVertex(states[i])) );
    }

    // We should have #states vertices and 0 edges
    EXPECT_EQ( data.numVertices(), states.size() );
    EXPECT_EQ( data.numEdges(), 0u );

    // Adding edges
    for (unsigned int i = 0; i < states.size()-1; ++i)
    {
        EXPECT_TRUE( data.addEdge(i, i+1) );
        EXPECT_FALSE( data.addEdge(i, states.size()) );  // vertex #states.size() does not exist

        EXPECT_TRUE( data.edgeExists(i, i+1) );
    }

    // We should have #states vertices and #states-1 edges at this point
    EXPECT_EQ( data.numVertices(), states.size() );
    EXPECT_EQ( data.numEdges(), states.size()-1 );

    // Make sure our edges are where we think they are
    for (unsigned int i = 0; i < states.size()-1; ++i)
    {
        std::vector<unsigned int> neighbors;
        EXPECT_EQ( data.getEdges(i, neighbors), 1u );
        EXPECT_EQ( neighbors[0], i+1 );
    }

    std::vector<unsigned int> neighbors;
    EXPECT_EQ( data.getEdges(states.size()-1, neighbors), 0u );

    // Make sure that the vertices can be retrived properly
    for (unsigned int i = 0; i < states.size(); ++i)
    {
        EXPECT_EQ(data.getVertex(i).getState(), states[i]);
        EXPECT_EQ(data.getVertex(i).getTag(), (signed)i);
    }

    for (size_t i = 0; i < states.size(); ++i)
        space->freeState(states[i]);
}

TEST(PlannerData, AdvancedConstruction)
{
    base::StateSpacePtr space(new base::RealVectorStateSpace(1));
    base::PlannerData data;
    std::vector<base::State*> states;

    // Creating 1000 states
    for (unsigned int i = 0; i < 1000; ++i)
        states.push_back(space->allocState());

    // Adding vertices and edges simultaneously
    for (unsigned int i = 0; i < states.size()-1; ++i)
    {
        EXPECT_TRUE( data.addEdge (base::PlannerDataVertex(states[i], (signed)i),
                                   base::PlannerDataVertex(states[i+1], (signed)i+1)) );

        // Duplicates are not allowed
        EXPECT_FALSE( data.addEdge (base::PlannerDataVertex(states[i], (signed)i),
                                    base::PlannerDataVertex(states[i+1], (signed)i+1)) );

        EXPECT_TRUE( data.edgeExists (i, i+1) );

        EXPECT_EQ( data.vertexIndex(base::PlannerDataVertex(states[i], (signed)i)), i );
        EXPECT_TRUE( data.vertexExists(base::PlannerDataVertex(states[i], i)) );
    }

    // Adding two start states and two goal states
    EXPECT_EQ( data.addStartVertex (base::PlannerDataVertex(states[states.size()-4], states.size()-4)), states.size()-4 );
    EXPECT_EQ( data.addGoalVertex (base::PlannerDataVertex(states[states.size()-3], states.size()-3)),  states.size()-3 );
    EXPECT_EQ( data.addStartVertex (base::PlannerDataVertex(states[states.size()-2], states.size()-2)), states.size()-2 );
    EXPECT_EQ( data.addGoalVertex (base::PlannerDataVertex(states[states.size()-1], states.size()-1)),  states.size()-1 );

    EXPECT_EQ( data.numStartVertices(), 2u );
    EXPECT_EQ( data.numGoalVertices (), 2u );
    EXPECT_EQ( data.getStartIndex(0), states.size()-4 );
    EXPECT_EQ( data.getStartIndex(1), states.size()-2 );
    EXPECT_EQ( data.getStartIndex(2), base::PlannerData::INVALID_INDEX );
    EXPECT_EQ( data.getGoalIndex(0), states.size()-3 );
    EXPECT_EQ( data.getGoalIndex(1), states.size()-1 );
    EXPECT_EQ( data.getGoalIndex(2), base::PlannerData::INVALID_INDEX );

    // Make sure that the start and goal indices are where we think they are
    for (unsigned int i = 0; i < states.size(); ++i)
    {
        if (i < states.size()-4)
        {
            EXPECT_FALSE( data.isStartState(i) );
            EXPECT_FALSE( data.isGoalState(i) );
        }
        else if (i == states.size()-4 || i == states.size()-2)
        {
            EXPECT_TRUE( data.isStartState(i) );
            EXPECT_FALSE( data.isGoalState(i) );
        }
        else if (i == states.size()-3 || i == states.size()-1)
        {
            EXPECT_FALSE( data.isStartState(i) );
            EXPECT_TRUE( data.isGoalState(i) );
        }
    }

    // We should have #states vertices and #states-1 edges
    EXPECT_EQ( data.numVertices(), states.size() );
    EXPECT_EQ( data.numEdges(), states.size()-1 );

    // Make sure our edges are where we think they are
    for (unsigned int i = 0; i < states.size()-1; ++i)
    {
        std::vector<unsigned int> nbrs;
        EXPECT_EQ( data.getEdges(i, nbrs), 1u );
        EXPECT_EQ( nbrs[0], i+1 );
    }
    std::vector<unsigned int> nbrs;
    EXPECT_EQ( data.getEdges(states.size()-1, nbrs), 0u );

    // Make sure that the vertices can be retrived properly
    for (unsigned int i = 0; i < states.size(); ++i)
    {
        EXPECT_EQ(data.getVertex(i).getState(), states[i]);
        EXPECT_EQ(data.getVertex(i).getTag(), (signed)i);
    }

    for (size_t i = 0; i < states.size(); ++i)
        space->freeState(states[i]);
}

class TestEdge : public base::PlannerDataEdge
{
public:
    TestEdge (unsigned int _a, unsigned int _b) : base::PlannerDataEdge(), a(_a), b(_b) {}
    TestEdge (const TestEdge &rhs) : base::PlannerDataEdge(), a(rhs.a), b(rhs.b) {}
    virtual ~TestEdge (void) {}

    /// \brief Return a clone of this object, allocated from the heap.
    virtual PlannerDataEdge* clone () const
    {
        return static_cast<PlannerDataEdge*>(new TestEdge(*this));
    }

    unsigned int a, b;
};

TEST(PlannerData, DataIntegrity)
{
    base::StateSpacePtr space(new base::RealVectorStateSpace(1));
    base::PlannerData data;
    std::vector<base::State*> states;

    // Creating 1000 states
    for (unsigned int i = 0; i < 1000; ++i)
    {
        base::State* st = space->allocState();
        st->as<base::RealVectorStateSpace::StateType>()->values[0] = i;
        states.push_back(st);
    }

    // Adding vertices and edges simultaneously
    for (unsigned int i = 1; i < states.size(); ++i)
    {
        EXPECT_TRUE( data.addEdge (base::PlannerDataVertex(states[i-1], i),
                                   base::PlannerDataVertex(states[i], i+1),
                                   TestEdge(i-1, i)) );
    }

    // We should have #states vertices and #states-1 edges
    EXPECT_EQ( data.numVertices(), states.size() );
    EXPECT_EQ( data.numEdges(), states.size()-1 );

    // Attempt to retrieve some vertices
    EXPECT_NE( &data.getVertex(0), &base::PlannerData::NO_VERTEX );
    EXPECT_NE( &data.getVertex(3), &base::PlannerData::NO_VERTEX );
    EXPECT_EQ( &data.getVertex(states.size()), &base::PlannerData::NO_VERTEX ); // vertex #states.size() does not exist

    // Attempt to retrieve some edges
    EXPECT_NE( &data.getEdge(0, 1), &base::PlannerData::NO_EDGE );
    EXPECT_NE( &data.getEdge(4, 5), &base::PlannerData::NO_EDGE );
    EXPECT_EQ( &data.getEdge(0, states.size()), &base::PlannerData::NO_EDGE ); // edge does not exist

    // Ensure vertex data integrity
    for (unsigned int i = 0; i < states.size(); ++i)
        EXPECT_EQ( data.getVertex(i).getState()->as<base::RealVectorStateSpace::StateType>()->values[0], i );

    // Ensure edge data integrity
    for (unsigned int i = 1; i < states.size(); ++i)
    {
        TestEdge& edge = static_cast<TestEdge&>(data.getEdge(i-1, i));
        ASSERT_NE ( &edge, &base::PlannerData::NO_EDGE );
        EXPECT_EQ( edge.a, i-1 );
        EXPECT_EQ( edge.b, i );
    }

    // Reset the tag for state #0
    EXPECT_TRUE( data.tagState(states[0], 10000) );
    EXPECT_EQ( data.getVertex(0).getTag(), 10000 );
    EXPECT_FALSE( data.tagState(0, 1000) ); // state doesn't exist

    // Reset the edge weight for 0->1
    EXPECT_TRUE( data.setEdgeWeight(0, 1, 1.234) );
    EXPECT_NEAR( data.getEdgeWeight(0, 1), 1.234, 1e-4);

    EXPECT_EQ( data.getEdgeWeight(0, 5), base::PlannerData::INVALID_WEIGHT ); // edge does not exist
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

    // Creating 1000 states
    for (unsigned int i = 0; i < 1000; ++i)
        states.push_back(space->allocState());

    // Adding vertices and edges
    for (unsigned int i = 0; i < states.size()-1; ++i)
    {
        EXPECT_TRUE( data.addEdge (base::PlannerDataVertex(states[i], i),
                                   base::PlannerDataVertex(states[i+1], i+1)) );
    }

    EXPECT_EQ( data.numVertices(), states.size() );
    EXPECT_EQ( data.numEdges(), states.size()-1 );

    EXPECT_TRUE( data.removeVertex(base::PlannerDataVertex(states[states.size()-1])) );
    EXPECT_EQ( data.numVertices(), states.size()-1 );
    EXPECT_EQ( data.numEdges(), states.size()-2 );

    EXPECT_FALSE( data.removeVertex(base::PlannerDataVertex(states[states.size()-1])) ); // we already removed this vertex
    EXPECT_EQ( data.numVertices(), states.size()-1 );
    EXPECT_EQ( data.numEdges(), states.size()-2 );

    EXPECT_FALSE( data.removeVertex(states.size()) ); // vertex does not exist
    EXPECT_EQ( data.numVertices(), states.size()-1 );
    EXPECT_EQ( data.numEdges(), states.size()-2 );

    EXPECT_TRUE( data.removeVertex(6) );
    EXPECT_EQ( data.numVertices(), states.size()-2 );
    EXPECT_EQ( data.numEdges(), states.size()-4 );  // incoming edge and outgoing edge should be removed

    EXPECT_TRUE( data.removeEdge(base::PlannerDataVertex(states[1]), base::PlannerDataVertex(states[2])) );
    EXPECT_EQ( data.numVertices(), states.size()-2 );
    EXPECT_EQ( data.numEdges(), states.size()-5 );

    EXPECT_FALSE( data.removeEdge(base::PlannerDataVertex(states[1]), base::PlannerDataVertex(states[2])) ); // we just removed this edge
    EXPECT_EQ( data.numVertices(), states.size()-2  );
    EXPECT_EQ( data.numEdges(), states.size()-5 );

    EXPECT_FALSE( data.removeEdge(5, 6) ); // edge does not exist (we removed vertex 6)
    EXPECT_EQ( data.numVertices(), states.size()-2 );
    EXPECT_EQ( data.numEdges(), states.size()-5 );

    EXPECT_TRUE( data.removeEdge(4, 5) );
    EXPECT_EQ( data.numVertices(), states.size()-2 );
    EXPECT_EQ( data.numEdges(), states.size()-6 );

    // Make sure the final graph looks the way we think:
    for (size_t i = 0; i < states.size(); ++i)
    {
        if (i == 6 || i == states.size()-1)
            EXPECT_FALSE( data.vertexExists(base::PlannerDataVertex(states[i])) );
        else
            EXPECT_TRUE( data.vertexExists(base::PlannerDataVertex(states[i])) );
    }

    for (size_t i = 0; i < states.size()-2; ++i) // we removed two vertices
    {
        std::vector<unsigned int> nbrs;
        if (i == 1 || i == 4 || i == 5 || i == states.size()-3)
            EXPECT_EQ ( data.getEdges(i, nbrs), 0u );
        else
        {
            EXPECT_EQ ( data.getEdges(i, nbrs), 1u );
            EXPECT_EQ ( nbrs[0], i+1 );
            EXPECT_TRUE( data.edgeExists(i, i+1) );
         }
    }

    for (size_t i = 0; i < states.size(); ++i)
        space->freeState(states[i]);
}

TEST(PlannerData, AddRemoveStartAndGoalStates)
{
    base::StateSpacePtr space(new base::RealVectorStateSpace(1));
    base::PlannerData data;
    std::vector<base::State*> states;

    // Creating 1000 states
    for (unsigned int i = 0; i < 1000; ++i)
        states.push_back(space->allocState());

    // Adding vertices and edges
    for (unsigned int i = 0; i < states.size()-1; ++i)
    {
        // Mark states at the 50s as start states
        if (i % 50 == 0 && i % 100 != 0 && i != 0)
            data.addStartVertex(base::PlannerDataVertex(states[i], i));

        // Mark states at the 100s as goal states
        if (i % 100 == 0)
            data.addGoalVertex(base::PlannerDataVertex(states[i], i));

        EXPECT_TRUE( data.addEdge (base::PlannerDataVertex(states[i], i),
                                   base::PlannerDataVertex(states[i+1], i+1)) );
    }

    EXPECT_EQ( data.numStartVertices(), 10u );
    EXPECT_EQ( data.numGoalVertices (), 10u );

    for (unsigned int i = 50; i < states.size()-1; i+=100)
        EXPECT_TRUE( data.isStartState(i) );

    for (unsigned int i = 0; i < states.size()-1; i+=100)
        EXPECT_TRUE( data.isGoalState(i) );

    // Removing a start state
    EXPECT_TRUE( data.removeVertex(50) );

    EXPECT_EQ( data.numStartVertices(), 9u );
    EXPECT_EQ( data.numGoalVertices (), 10u );

    for (unsigned int i = 149; i < states.size()-1; i+=100)
        EXPECT_TRUE( data.isStartState(i) );

    // Removing a goal state
    EXPECT_TRUE( data.removeVertex(0) );

    for (unsigned int i = 98; i < states.size()-2; i+=100)
        EXPECT_TRUE( data.isGoalState(i) );

    for (unsigned int i = 0; i < states.size() - 2; i++)
    {
        if ((i+2) % 100 == 0)
        {
            EXPECT_FALSE( data.isStartState(i) );
            EXPECT_TRUE( data.isGoalState(i) );
        }
        else if ((i+2) % 50 == 0 && i != 48)
        {
            EXPECT_TRUE( data.isStartState(i) );
            EXPECT_FALSE( data.isGoalState(i) );
        }
        else
        {
            EXPECT_FALSE( data.isStartState(i) );
            EXPECT_FALSE( data.isGoalState(i) );
        }
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
