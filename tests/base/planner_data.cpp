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

#define BOOST_TEST_MODULE "PlannerData"
#include <boost/test/unit_test.hpp>
#include <boost/serialization/export.hpp>
#include <iostream>
#include <vector>

#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerDataStorage.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

using namespace ompl;

// define a convenience macro
#define BOOST_OMPL_EXPECT_NEAR(a, b, diff) BOOST_CHECK_SMALL((a) - (b), diff)

BOOST_AUTO_TEST_CASE(SimpleConstruction)
{
    auto space(std::make_shared<base::RealVectorStateSpace>(1));
    auto si(std::make_shared<base::SpaceInformation>(space));
    base::PlannerData data(si);
    std::vector<base::State*> states;

    // Creating 1000 states
    for (unsigned int i = 0; i < 1000; ++i)
        states.push_back(space->allocState());

    // Adding vertices
    for (unsigned int i = 0; i < states.size(); ++i)
    {
        unsigned int vtx = data.addVertex(base::PlannerDataVertex(states[i], (signed)i));

        BOOST_CHECK_NE( vtx, base::PlannerData::INVALID_INDEX );
        BOOST_CHECK_EQUAL( data.addVertex(base::PlannerDataVertex(states[i])), vtx );

        BOOST_CHECK_EQUAL( data.vertexIndex(base::PlannerDataVertex(states[i])), i );
        BOOST_CHECK( data.vertexExists(base::PlannerDataVertex(states[i])) );
    }

    // We should have #states vertices and 0 edges
    BOOST_CHECK_EQUAL( data.numVertices(), states.size() );
    BOOST_CHECK_EQUAL( data.numEdges(), 0u );

    // Adding edges
    for (unsigned int i = 0; i < states.size()-1; ++i)
    {
        BOOST_CHECK( data.addEdge(i, i+1) );
        BOOST_CHECK_EQUAL( data.addEdge(i, states.size()), false ); // vertex #states.size() does not exist

        BOOST_CHECK( data.edgeExists(i, i+1) );
    }

    for (unsigned int i = 1; i < states.size(); ++i)
    {
        std::vector<unsigned int> neighbors;
        BOOST_REQUIRE_EQUAL( data.getIncomingEdges(i, neighbors), 1u );
        BOOST_CHECK_EQUAL( neighbors[0], i-1 );
    }

    // We should have #states vertices and #states-1 edges at this point
    BOOST_CHECK_EQUAL( data.numVertices(), states.size() );
    BOOST_CHECK_EQUAL( data.numEdges(), states.size()-1);

    // Make sure our edges are where we think they are
    for (unsigned int i = 0; i < states.size()-1; ++i)
    {
        std::vector<unsigned int> neighbors;
        BOOST_REQUIRE_EQUAL( data.getEdges(i, neighbors), 1u );
        BOOST_CHECK_EQUAL( neighbors[0], i+1 );
    }

    std::vector<unsigned int> neighbors;
    BOOST_CHECK_EQUAL( data.getEdges(states.size()-1, neighbors), 0u );

    // Make sure that the vertices can be retrived properly
    for (unsigned int i = 0; i < states.size(); ++i)
    {
        BOOST_CHECK_EQUAL( data.getVertex(i).getState(), states[i] );
        BOOST_CHECK_EQUAL( data.getVertex(i).getTag(), (signed)i );
    }

    for (auto & state : states)
        space->freeState(state);
}

BOOST_AUTO_TEST_CASE(AdvancedConstruction)
{
    auto space(std::make_shared<base::RealVectorStateSpace>(1));
    auto si(std::make_shared<base::SpaceInformation>(space));
    base::PlannerData data(si);
    std::vector<base::State*> states;

    // Creating 1000 states
    for (unsigned int i = 0; i < 1000; ++i)
        states.push_back(space->allocState());

    // Adding vertices and edges simultaneously
    for (unsigned int i = 0; i < states.size()-1; ++i)
    {
        BOOST_CHECK( data.addEdge (base::PlannerDataVertex(states[i], (signed)i),
                                   base::PlannerDataVertex(states[i+1], (signed)i+1)) );

        // Duplicates are not allowed
        BOOST_CHECK_EQUAL( data.addEdge (base::PlannerDataVertex(states[i], (signed)i), base::PlannerDataVertex(states[i+1], (signed)i+1)), false );

        BOOST_CHECK( data.edgeExists (i, i+1) );

        BOOST_CHECK_EQUAL( data.vertexIndex(base::PlannerDataVertex(states[i], (signed)i)), i );
        BOOST_CHECK( data.vertexExists(base::PlannerDataVertex(states[i], i)) );
    }

    // Adding two start states and two goal states
    BOOST_CHECK_EQUAL( data.addStartVertex (base::PlannerDataVertex(states[states.size()-4], states.size()-4)), states.size()-4 );
    BOOST_CHECK_EQUAL( data.addGoalVertex (base::PlannerDataVertex(states[states.size()-3], states.size()-3)),  states.size()-3 );
    BOOST_CHECK_EQUAL( data.addStartVertex (base::PlannerDataVertex(states[states.size()-2], states.size()-2)), states.size()-2 );
    BOOST_CHECK_EQUAL( data.addGoalVertex (base::PlannerDataVertex(states[states.size()-1], states.size()-1)),  states.size()-1 );

    BOOST_CHECK_EQUAL( data.numStartVertices(), 2u );
    BOOST_CHECK_EQUAL( data.numGoalVertices (), 2u );
    BOOST_CHECK_EQUAL( data.getStartIndex(0), states.size()-4 );
    BOOST_CHECK_EQUAL( data.getStartIndex(1), states.size()-2 );
    BOOST_CHECK_EQUAL( data.getStartIndex(2), base::PlannerData::INVALID_INDEX );
    BOOST_CHECK_EQUAL( data.getGoalIndex(0), states.size()-3 );
    BOOST_CHECK_EQUAL( data.getGoalIndex(1), states.size()-1 );
    BOOST_CHECK_EQUAL( data.getGoalIndex(2), base::PlannerData::INVALID_INDEX );

    // Make sure that the start and goal indices are where we think they are
    for (unsigned int i = 0; i < states.size(); ++i)
    {
        if (i < states.size()-4)
        {
            BOOST_CHECK_EQUAL( data.isStartVertex(i), false );
            BOOST_CHECK_EQUAL( data.isGoalVertex(i), false );
        }
        else if (i == states.size()-4 || i == states.size()-2)
        {
            BOOST_CHECK_EQUAL( data.isStartVertex(i), true );
            BOOST_CHECK_EQUAL( data.isGoalVertex(i), false );
        }
        else if (i == states.size()-3 || i == states.size()-1)
        {
            BOOST_CHECK_EQUAL( data.isStartVertex(i), false );
            BOOST_CHECK_EQUAL( data.isGoalVertex(i), true );
        }
    }

    // We should have #states vertices and #states-1 edges
    BOOST_CHECK_EQUAL( data.numVertices(), states.size() );
    BOOST_CHECK_EQUAL( data.numEdges(), states.size()-1 );

    // Make sure our edges are where we think they are
    for (unsigned int i = 0; i < states.size()-1; ++i)
    {
        std::vector<unsigned int> nbrs;
        BOOST_CHECK_EQUAL( data.getEdges(i, nbrs), 1u );
        BOOST_CHECK_EQUAL( nbrs[0], i+1 );
    }

    for (unsigned int i = 1; i < states.size(); ++i)
    {
        std::vector<unsigned int> neighbors;
        BOOST_REQUIRE_EQUAL( data.getIncomingEdges(i, neighbors), 1u );
        BOOST_CHECK_EQUAL( neighbors[0], i-1 );
    }

    std::vector<unsigned int> nbrs;
    BOOST_CHECK_EQUAL( data.getEdges(states.size()-1, nbrs), 0u );

    // Make sure that the vertices can be retrived properly
    for (unsigned int i = 0; i < states.size(); ++i)
    {
        BOOST_CHECK_EQUAL(data.getVertex(i).getState(), states[i]);
        BOOST_CHECK_EQUAL(data.getVertex(i).getTag(), (signed)i);
    }

    for (auto & state : states)
        space->freeState(state);
}

class TestEdge : public base::PlannerDataEdge
{
public:
    TestEdge (unsigned int _a, unsigned int _b) : a(_a), b(_b) {}
    TestEdge (const TestEdge &rhs) : a(rhs.a), b(rhs.b) {}
    ~TestEdge () override = default;

    /// \brief Return a clone of this object, allocated from the heap.
    PlannerDataEdge* clone () const override
    {
        return static_cast<PlannerDataEdge*>(new TestEdge(*this));
    }

    unsigned int a, b;
};

BOOST_AUTO_TEST_CASE(DataIntegrity)
{
    auto space(std::make_shared<base::RealVectorStateSpace>(1));
    auto si(std::make_shared<base::SpaceInformation>(space));
    base::PlannerData data(si);
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
        BOOST_CHECK( data.addEdge (base::PlannerDataVertex(states[i-1], i),
                                   base::PlannerDataVertex(states[i], i+1),
                                   TestEdge(i-1, i)) );
    }

    // We should have #states vertices and #states-1 edges
    BOOST_CHECK_EQUAL( data.numVertices(), states.size() );
    BOOST_CHECK_EQUAL( data.numEdges(), states.size()-1 );

    // Attempt to retrieve some vertices
    BOOST_CHECK_NE( &data.getVertex(0), &base::PlannerData::NO_VERTEX );
    BOOST_CHECK_NE( &data.getVertex(3), &base::PlannerData::NO_VERTEX );
    BOOST_CHECK_EQUAL( &data.getVertex(states.size()), &base::PlannerData::NO_VERTEX ); // vertex #states.size() does not exist

    // Attempt to retrieve some edges
    BOOST_CHECK_NE( &data.getEdge(0, 1), &base::PlannerData::NO_EDGE );
    BOOST_CHECK_NE( &data.getEdge(4, 5), &base::PlannerData::NO_EDGE );
    BOOST_CHECK_EQUAL( &data.getEdge(0, states.size()), &base::PlannerData::NO_EDGE ); // edge does not exist

    // Ensure vertex data integrity
    for (unsigned int i = 0; i < states.size(); ++i)
        BOOST_CHECK_EQUAL( data.getVertex(i).getState()->as<base::RealVectorStateSpace::StateType>()->values[0], i );

    // Ensure edge data integrity
    for (unsigned int i = 1; i < states.size(); ++i)
    {
        auto& edge = static_cast<TestEdge&>(data.getEdge(i-1, i));
        BOOST_REQUIRE_NE ( &edge, &base::PlannerData::NO_EDGE );
        BOOST_CHECK_EQUAL( edge.a, i-1 );
        BOOST_CHECK_EQUAL( edge.b, i );
    }

    // Reset the tag for state #0
    BOOST_CHECK( data.tagState(states[0], 10000) );
    BOOST_CHECK_EQUAL( data.getVertex(0).getTag(), 10000 );
    BOOST_CHECK_EQUAL( data.tagState(nullptr, 1000), false ); // state doesn't exist

    // Reset the edge weight for 0->1
    BOOST_CHECK( data.setEdgeWeight(0, 1, base::Cost(1.234)) );

    base::Cost w;
    BOOST_CHECK(data.getEdgeWeight(0, 1, &w));
    BOOST_OMPL_EXPECT_NEAR(w.value(), 1.234, 1e-4);

    BOOST_CHECK_EQUAL(data.getEdgeWeight(0, 5, &w), false ); // edge does not exist

    BOOST_CHECK_EQUAL( data.setEdgeWeight(0, 5, base::Cost(2.345)), false );

    // Try to tag an invalid state
    BOOST_CHECK_EQUAL( data.tagState(nullptr, 100), false );
    for (auto & state : states)
        space->freeState(state);
}

BOOST_AUTO_TEST_CASE(AddRemoveVerticesAndEdges)
{
    auto space(std::make_shared<base::RealVectorStateSpace>(1));
    auto si(std::make_shared<base::SpaceInformation>(space));
    base::PlannerData data(si);
    std::vector<base::State*> states;

    // Creating 1000 states
    for (unsigned int i = 0; i < 1000; ++i)
        states.push_back(space->allocState());

    // Adding vertices and edges
    for (unsigned int i = 0; i < states.size()-1; ++i)
    {
        BOOST_CHECK_EQUAL( data.addEdge (base::PlannerDataVertex(states[i], i), base::PlannerDataVertex(states[i+1], i+1)),
                           true );
    }

    BOOST_CHECK_EQUAL( data.numVertices(), states.size() );
    BOOST_CHECK_EQUAL( data.numEdges(), states.size()-1 );

    BOOST_CHECK( data.removeVertex(base::PlannerDataVertex(states[states.size()-1])) );
    BOOST_CHECK_EQUAL( data.numVertices(), states.size()-1 );
    BOOST_CHECK_EQUAL( data.numEdges(), states.size()-2 );

    BOOST_CHECK_EQUAL( data.removeVertex(base::PlannerDataVertex(states[states.size()-1])), false ); // we already removed this vertex
    BOOST_CHECK_EQUAL( data.numVertices(), states.size()-1 );
    BOOST_CHECK_EQUAL( data.numEdges(), states.size()-2 );

    BOOST_CHECK_EQUAL( data.removeVertex(states.size()), false ); // vertex does not exist
    BOOST_CHECK_EQUAL( data.numVertices(), states.size()-1 );
    BOOST_CHECK_EQUAL( data.numEdges(), states.size()-2 );

    BOOST_CHECK( data.removeVertex(6) );
    BOOST_CHECK_EQUAL( data.numVertices(), states.size()-2 );
    BOOST_CHECK_EQUAL( data.numEdges(), states.size()-4 );  // incoming edge and outgoing edge should be removed

    BOOST_CHECK( data.removeEdge(base::PlannerDataVertex(states[1]), base::PlannerDataVertex(states[2])) );
    BOOST_CHECK_EQUAL( data.numVertices(), states.size()-2 );
    BOOST_CHECK_EQUAL( data.numEdges(), states.size()-5 );

    BOOST_CHECK_EQUAL( data.removeEdge(base::PlannerDataVertex(states[1]), base::PlannerDataVertex(states[2])), false ); // we just removed this edge
    BOOST_CHECK_EQUAL( data.numVertices(), states.size()-2  );
    BOOST_CHECK_EQUAL( data.numEdges(), states.size()-5 );

    BOOST_CHECK_EQUAL( data.removeEdge(5, 6), false ); // edge does not exist (we removed vertex 6)
    BOOST_CHECK_EQUAL( data.numVertices(), states.size()-2 );
    BOOST_CHECK_EQUAL( data.numEdges(), states.size()-5 );

    BOOST_CHECK( data.removeEdge(4, 5) );
    BOOST_CHECK_EQUAL( data.numVertices(), states.size()-2 );
    BOOST_CHECK_EQUAL( data.numEdges(), states.size()-6 );

    // Make sure the final graph looks the way we think:
    for (size_t i = 0; i < states.size(); ++i)
    {
        if (i == 6 || i == states.size()-1)
            BOOST_CHECK_EQUAL( data.vertexExists(base::PlannerDataVertex(states[i])), false );
        else
            BOOST_CHECK( data.vertexExists(base::PlannerDataVertex(states[i])) );
    }

    for (size_t i = 0; i < states.size()-2; ++i) // we removed two vertices
    {
        std::vector<unsigned int> nbrs;
        if (i == 1 || i == 4 || i == 5 || i == states.size()-3)
            BOOST_CHECK_EQUAL ( data.getEdges(i, nbrs), 0u );
        else
        {
            BOOST_CHECK_EQUAL ( data.getEdges(i, nbrs), 1u );
            BOOST_CHECK_EQUAL ( nbrs[0], i+1 );
            BOOST_CHECK( data.edgeExists(i, i+1) );
         }
    }

    for (auto & state : states)
        space->freeState(state);
}

BOOST_AUTO_TEST_CASE(AddRemoveStartAndGoalStates)
{
    auto space(std::make_shared<base::RealVectorStateSpace>(1));
    auto si(std::make_shared<base::SpaceInformation>(space));
    base::PlannerData data(si);
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

        BOOST_CHECK( data.addEdge (base::PlannerDataVertex(states[i], i),
                                   base::PlannerDataVertex(states[i+1], i+1)) );
    }

    BOOST_CHECK_EQUAL( data.numStartVertices(), 10u );
    BOOST_CHECK_EQUAL( data.numGoalVertices (), 10u );

    for (unsigned int i = 50; i < states.size()-1; i+=100)
        BOOST_CHECK( data.isStartVertex(i) );

    for (unsigned int i = 0; i < states.size()-1; i+=100)
        BOOST_CHECK( data.isGoalVertex(i) );

    // Removing a start state
    BOOST_CHECK( data.removeVertex(50) );

    BOOST_CHECK_EQUAL( data.numStartVertices(), 9u );
    BOOST_CHECK_EQUAL( data.numGoalVertices (), 10u );

    for (unsigned int i = 149; i < states.size()-1; i+=100)
        BOOST_CHECK( data.isStartVertex(i) );

    // Removing a goal state
    BOOST_CHECK( data.removeVertex(0) );

    for (unsigned int i = 98; i < states.size()-2; i+=100)
        BOOST_CHECK( data.isGoalVertex(i) );

    for (unsigned int i = 0; i < states.size() - 2; i++)
    {
        if ((i+2) % 100 == 0)
        {
            BOOST_CHECK_EQUAL( data.isStartVertex(i), false );
            BOOST_CHECK_EQUAL( data.isGoalVertex(i), true );
        }
        else if ((i+2) % 50 == 0 && i != 48)
        {
            BOOST_CHECK_EQUAL( data.isStartVertex(i), true );
            BOOST_CHECK_EQUAL( data.isGoalVertex(i), false );
        }
        else
        {
            BOOST_CHECK_EQUAL( data.isStartVertex(i), false );
            BOOST_CHECK_EQUAL( data.isGoalVertex(i), false );
        }
    }

    for (auto & state : states)
        space->freeState(state);
}

class PlannerDataTestVertex : public ompl::base::PlannerDataVertex
{
public:
    PlannerDataTestVertex (base::State* st, int tag = 0, int tag2 = 0) : ompl::base::PlannerDataVertex(st, tag), tag2_(tag2) {}
    PlannerDataTestVertex (const PlannerDataTestVertex &rhs) : ompl::base::PlannerDataVertex(rhs.state_, rhs.tag_), tag2_(rhs.tag2_) {}

    ompl::base::PlannerDataVertex* clone () const override
    {
        return static_cast<ompl::base::PlannerDataVertex*>(new PlannerDataTestVertex(*this));
    }

    int tag2_;

protected:
    PlannerDataTestVertex() = default;

    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive & ar, const unsigned int /*version*/)
    {
        ar & boost::serialization::base_object<ompl::base::PlannerDataVertex>(*this);
        ar & tag2_;
    }
};

// This allows us to serialize the derived class PlannerDataTestVertex
BOOST_CLASS_EXPORT(PlannerDataTestVertex);

BOOST_AUTO_TEST_CASE(Serialization)
{
    auto space(std::make_shared<base::RealVectorStateSpace>(1));
    auto si(std::make_shared<base::SpaceInformation>(space));
    base::PlannerData data(si);
    std::vector<base::State*> states;

    // Creating 1000 states
    for (unsigned int i = 0; i < 1000; ++i)
    {
        states.push_back(space->allocState());
        states[i]->as<base::RealVectorStateSpace::StateType>()->values[0] = (double)i;

        PlannerDataTestVertex vtx(states[i], i, i+1);
        BOOST_CHECK (data.addVertex(vtx) == i );
        BOOST_CHECK (data.getVertex(i).getTag() == (signed)i);
        BOOST_CHECK (static_cast<PlannerDataTestVertex&>(data.getVertex(i)).tag2_ == (signed)i+1);
    }

    // Mark some start and goal states
    data.markStartState(states[0]);
    data.markStartState(states[states.size()/2]);
    data.markStartState(states[states.size()-1]);
    data.markGoalState(states[1]);
    data.markGoalState(states[states.size()-2]);

    // Add a whole bunch of random edges
    unsigned int num_edges_to_add = 10000;
    ompl::RNG rng;

    for (unsigned int i = 0; i < num_edges_to_add; ++i)
    {
        unsigned int v2, v1 = rng.uniformInt(0, states.size()-1);
        do v2 = rng.uniformInt(0, states.size()-1); while (v2 == v1 || data.edgeExists(v1, v2));

        BOOST_CHECK( data.addEdge(v1, v2) );
    }

    BOOST_CHECK_EQUAL ( data.numVertices(), states.size() );
    BOOST_CHECK_EQUAL ( data.numEdges(), num_edges_to_add );

    base::PlannerData data2(si);
    base::PlannerDataStorage storage;
    storage.store(data, "testdata");
    storage.load("testdata", data2);

    // Verify that data == data2
    BOOST_CHECK_EQUAL ( data2.numVertices(), states.size() );
    BOOST_CHECK_EQUAL ( data2.numEdges(), num_edges_to_add );

    // Check our start/goal states
    BOOST_CHECK ( data2.numStartVertices() == 3 );
    BOOST_CHECK ( data2.numGoalVertices() == 2 );
    BOOST_CHECK ( data2.isStartVertex(0) );
    BOOST_CHECK ( data2.isStartVertex(states.size()/2) );
    BOOST_CHECK ( data2.isStartVertex(states.size()-1) );
    BOOST_CHECK ( data2.isGoalVertex(1) );
    BOOST_CHECK ( data2.isGoalVertex(states.size()-2) );

    for (size_t i = 0; i < states.size(); ++i)
    {
        BOOST_CHECK (space->equalStates(data2.getVertex(i).getState(), states[i]) );
        BOOST_CHECK (data.getVertex(i).getTag() == data2.getVertex(i).getTag() );
        BOOST_CHECK (static_cast<PlannerDataTestVertex&>(data2.getVertex(i)).tag2_ == (signed)i+1);
    }

    for (size_t i = 0; i < states.size(); ++i)
    {
        std::vector<unsigned int> neighbors, neighbors2;
        data.getEdges(i, neighbors);
        data2.getEdges(i, neighbors2);

        std::sort (neighbors.begin(), neighbors.end());
        std::sort (neighbors2.begin(), neighbors2.end());
        BOOST_REQUIRE_EQUAL( neighbors.size(), neighbors2.size() );

        for (size_t j = 0; j < neighbors.size(); ++j)
            BOOST_CHECK_EQUAL( neighbors[j], neighbors2[j] );
    }

    for (auto & state : states)
        space->freeState(state);
}
