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

#define BOOST_TEST_MODULE "PlannerDataControl"
#include <boost/test/unit_test.hpp>
#include <boost/serialization/export.hpp>
#include <iostream>
#include <vector>

#include "ompl/control/PlannerData.h"
#include "ompl/control/PlannerDataStorage.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"

using namespace ompl;

// define a convenience macro
#define BOOST_OMPL_EXPECT_NEAR(a, b, diff) BOOST_CHECK_SMALL((a) - (b), diff)

// Define the state and control spaces used throughout the testing
#define SETUP_STATE_CONTROL_SPACES \
base::StateSpacePtr space(std::make_shared<base::RealVectorStateSpace>(1)); \
control::ControlSpacePtr cspace(std::make_shared<control::RealVectorControlSpace>(space, 2)); \
base::RealVectorBounds spacebounds(1); \
spacebounds.setLow(-10); \
spacebounds.setHigh(10); \
base::RealVectorBounds ctrlbounds(2); \
ctrlbounds.setLow(-1); \
ctrlbounds.setHigh(1); \
space->as<base::RealVectorStateSpace>()->setBounds(spacebounds); \
cspace->as<control::RealVectorControlSpace>()->setBounds(ctrlbounds); \
control::SpaceInformationPtr si(std::make_shared<control::SpaceInformation>(space, cspace)); \
si->setStateValidityChecker(isValid); \
si->setStatePropagator(propagate); \
si->setMinMaxControlDuration(1, 10); \
si->setPropagationStepSize(1); \
si->setup();


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

class PlannerDataTestEdge : public ompl::control::PlannerDataEdgeControl
{
public:
    PlannerDataTestEdge(const control::Control *c, double duration, int id) : PlannerDataEdgeControl(c, duration), id_(id) {}

    PlannerDataTestEdge (const PlannerDataTestEdge &rhs) : PlannerDataEdgeControl(rhs.c_, rhs.duration_), id_(rhs.id_)  {}

    ~PlannerDataTestEdge () override = default;

    ompl::base::PlannerDataEdge* clone () const override
    {
        return static_cast<base::PlannerDataEdge*>(new PlannerDataTestEdge(*this));
    }

    bool operator == (const ompl::base::PlannerDataEdge &rhs) const override
    {
        const auto *rhst = static_cast<const PlannerDataTestEdge*> (&rhs);
        if (rhst != nullptr)
        {
            if (id_ == rhst->id_)
                return static_cast<const ompl::control::PlannerDataEdgeControl>(*this) == rhs;
        }

        return false;
    }

    int id_;

protected:
    PlannerDataTestEdge() = default;
    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive & ar, const unsigned int /*version*/)
    {
        ar & boost::serialization::base_object<ompl::control::PlannerDataEdgeControl>(*this);
        ar & id_;
    }
};

// This allows us to serialize the derived vertex and edge classes
BOOST_CLASS_EXPORT(PlannerDataTestVertex);
BOOST_CLASS_EXPORT(PlannerDataTestEdge);

void propagate(const base::State * /*unused*/, const control::Control * /*unused*/, const double /*unused*/, base::State * /*unused*/){}
bool isValid (const base::State* /*unused*/){ return true; }

BOOST_AUTO_TEST_CASE(SimpleConstruction)
{
    SETUP_STATE_CONTROL_SPACES

    control::PlannerData data(si);
    std::vector<base::State*> states;
    std::vector<control::Control*> controls;
    base::StateSamplerPtr stsamp = space->allocDefaultStateSampler();
    control::ControlSamplerPtr csamp = cspace->allocDefaultControlSampler();

    // Creating 1000 states and controls
    for (unsigned int i = 0; i < 1000; ++i)
    {
        states.push_back(space->allocState());
        stsamp->sampleUniform(states[i]);
        controls.push_back(cspace->allocControl());
        csamp->sample(controls[i]);
    }

    // Adding vertices
    for (unsigned int i = 0; i < states.size(); ++i)
    {
        unsigned int vtx = data.addVertex(PlannerDataTestVertex(states[i], (signed)i, (signed)i+1));

        BOOST_CHECK_NE( vtx, base::PlannerData::INVALID_INDEX );
        BOOST_CHECK_EQUAL( data.addVertex(PlannerDataTestVertex(states[i])), vtx );

        BOOST_CHECK_EQUAL( data.vertexIndex(PlannerDataTestVertex(states[i])), i );
        BOOST_CHECK( data.vertexExists(PlannerDataTestVertex(states[i])) );
    }

    // We should have #states vertices and 0 edges
    BOOST_CHECK_EQUAL( data.numVertices(), states.size() );
    BOOST_CHECK_EQUAL( data.numEdges(), 0u );

    // Adding edges
    for (unsigned int i = 0; i < states.size()-1; ++i)
    {
        BOOST_CHECK( data.addEdge(i, i+1, PlannerDataTestEdge(controls[i], (double)i, (double)i)) );
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

    for (size_t i = 0; i < states.size(); ++i)
    {
        space->freeState(states[i]);
        cspace->freeControl(controls[i]);
    }
}

BOOST_AUTO_TEST_CASE(AdvancedConstruction)
{
    SETUP_STATE_CONTROL_SPACES

    control::PlannerData data(si);
    std::vector<base::State*> states;
    std::vector<control::Control*> controls;
    base::StateSamplerPtr stsamp = space->allocDefaultStateSampler();
    control::ControlSamplerPtr csamp = cspace->allocDefaultControlSampler();

    // Creating 1000 states
    for (unsigned int i = 0; i < 1000; ++i)
    {
        states.push_back(space->allocState());
        stsamp->sampleUniform(states[i]);
        controls.push_back(cspace->allocControl());
        csamp->sample(controls[i]);
    }

    // Adding vertices and edges simultaneously
    for (unsigned int i = 0; i < states.size()-1; ++i)
    {
        BOOST_CHECK( data.addEdge (PlannerDataTestVertex(states[i], (signed)i),
                                   PlannerDataTestVertex(states[i+1], (signed)i+1),
                                   PlannerDataTestEdge(controls[i], (double)i, (double)i)) );

        // Duplicates are not allowed
        BOOST_CHECK_EQUAL( data.addEdge (PlannerDataTestVertex(states[i], (signed)i),
                                         PlannerDataTestVertex(states[i+1], (signed)i+1),
                                         PlannerDataTestEdge(controls[i], (double)i, (double)i)), false );

        BOOST_CHECK( data.edgeExists (i, i+1) );

        BOOST_CHECK_EQUAL( data.vertexIndex(PlannerDataTestVertex(states[i])), i );
        BOOST_CHECK( data.vertexExists(PlannerDataTestVertex(states[i])) );
    }

    // Adding two start states and two goal states
    BOOST_CHECK_EQUAL( data.addStartVertex (PlannerDataTestVertex(states[states.size()-4])), states.size()-4 );
    BOOST_CHECK_EQUAL( data.addGoalVertex (PlannerDataTestVertex(states[states.size()-3])),  states.size()-3 );
    BOOST_CHECK_EQUAL( data.addStartVertex (PlannerDataTestVertex(states[states.size()-2])), states.size()-2 );
    BOOST_CHECK_EQUAL( data.addGoalVertex (PlannerDataTestVertex(states[states.size()-1])),  states.size()-1 );

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

    for (size_t i = 0; i < states.size(); ++i)
    {
        space->freeState(states[i]);
        cspace->freeControl(controls[i]);
    }
}

BOOST_AUTO_TEST_CASE(DataIntegrity)
{
    SETUP_STATE_CONTROL_SPACES

    control::PlannerData data(si);
    std::vector<base::State*> states;
    std::vector<control::Control*> controls;
    base::StateSamplerPtr stsamp = space->allocDefaultStateSampler();
    control::ControlSamplerPtr csamp = cspace->allocDefaultControlSampler();

    // Creating 1000 states
    for (unsigned int i = 0; i < 1000; ++i)
    {
        states.push_back(space->allocState());
        stsamp->sampleUniform(states[i]);
        controls.push_back(cspace->allocControl());
        csamp->sample(controls[i]);
    }

    // Adding vertices and edges simultaneously
    for (unsigned int i = 1; i < states.size(); ++i)
    {
        BOOST_CHECK( data.addEdge (PlannerDataTestVertex(states[i-1], i, i+1),
                                   PlannerDataTestVertex(states[i], i+1, i+2),
                                   PlannerDataTestEdge(controls[i], i, i+1)) );
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
        BOOST_CHECK_EQUAL( data.getVertex(i).getState()->as<base::RealVectorStateSpace::StateType>()->values[0], states[i]->as<base::RealVectorStateSpace::StateType>()->values[0] );

    // Ensure edge data integrity
    for (unsigned int i = 1; i < states.size(); ++i)
    {
        //TestEdge& edge = static_cast<TestEdge&>(data.getEdge(i-1, i));
        auto &edge = static_cast<PlannerDataTestEdge&>(data.getEdge(i-1, i));
        BOOST_REQUIRE_NE ( &edge, &base::PlannerData::NO_EDGE );
        BOOST_CHECK_EQUAL( edge.getControl(), controls[i] );
        BOOST_OMPL_EXPECT_NEAR ( edge.getDuration(), i, 1e-9 );
        BOOST_CHECK_EQUAL ( (unsigned int)edge.id_, i+1 );
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

    BOOST_CHECK_EQUAL( data.getEdgeWeight(0, 5, &w), false ); // edge does not exist
    BOOST_CHECK_EQUAL( data.setEdgeWeight(0, 5, base::Cost(2.345)), false );

    // Try to tag an invalid state
    BOOST_CHECK_EQUAL( data.tagState(nullptr, 100), false );

    for (size_t i = 0; i < states.size(); ++i)
    {
        space->freeState(states[i]);
        cspace->freeControl(controls[i]);
    }
}

BOOST_AUTO_TEST_CASE(AddRemoveVerticesAndEdges)
{
    SETUP_STATE_CONTROL_SPACES

    control::PlannerData data(si);
    std::vector<base::State*> states;
    std::vector<control::Control*> controls;
    base::StateSamplerPtr stsamp = space->allocDefaultStateSampler();
    control::ControlSamplerPtr csamp = cspace->allocDefaultControlSampler();

    // Creating 1000 states
    for (unsigned int i = 0; i < 1000; ++i)
    {
        states.push_back(space->allocState());
        stsamp->sampleUniform(states[i]);
        controls.push_back(cspace->allocControl());
        csamp->sample(controls[i]);
    }

    // Adding vertices and edges
    for (unsigned int i = 1; i < states.size(); ++i)
    {
        BOOST_CHECK( data.addEdge (PlannerDataTestVertex(states[i-1], i, i+1),
                                   PlannerDataTestVertex(states[i], i+1, i+2),
                                   PlannerDataTestEdge(controls[i], i, i+1)) );
    }

    BOOST_CHECK_EQUAL( data.numVertices(), states.size() );
    BOOST_CHECK_EQUAL( data.numEdges(), states.size()-1 );

    BOOST_CHECK( data.removeVertex(PlannerDataTestVertex(states[states.size()-1], states.size()-1, states.size())) );
    BOOST_CHECK_EQUAL( data.numVertices(), states.size()-1 );
    BOOST_CHECK_EQUAL( data.numEdges(), states.size()-2 );

    BOOST_CHECK_EQUAL( data.removeVertex(PlannerDataTestVertex(states[states.size()-1], states.size()-1, states.size())), false ); // we already removed this vertex
    BOOST_CHECK_EQUAL( data.numVertices(), states.size()-1 );
    BOOST_CHECK_EQUAL( data.numEdges(), states.size()-2 );

    BOOST_CHECK_EQUAL( data.removeVertex(states.size()), false ); // vertex does not exist
    BOOST_CHECK_EQUAL( data.numVertices(), states.size()-1 );
    BOOST_CHECK_EQUAL( data.numEdges(), states.size()-2 );

    BOOST_CHECK( data.removeVertex(6) );
    BOOST_CHECK_EQUAL( data.numVertices(), states.size()-2 );
    BOOST_CHECK_EQUAL( data.numEdges(), states.size()-4 );  // incoming edge and outgoing edge should be removed

    BOOST_CHECK( data.removeEdge(PlannerDataTestVertex(states[1], 1, 2), PlannerDataTestVertex(states[2], 2, 3)) );
    BOOST_CHECK_EQUAL( data.numVertices(), states.size()-2 );
    BOOST_CHECK_EQUAL( data.numEdges(), states.size()-5 );

    BOOST_CHECK_EQUAL( data.removeEdge(PlannerDataTestVertex(states[1], 1, 2), PlannerDataTestVertex(states[2], 2, 3)), false ); // we just removed this edge
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
            BOOST_CHECK_EQUAL( data.vertexExists(PlannerDataTestVertex(states[i], i, i+1)), false );
        else
            BOOST_CHECK( data.vertexExists(PlannerDataTestVertex(states[i], i, i+1)) );
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

    for (size_t i = 0; i < states.size(); ++i)
    {
        space->freeState(states[i]);
        cspace->freeControl(controls[i]);
    }
}

BOOST_AUTO_TEST_CASE(Serialization)
{
    SETUP_STATE_CONTROL_SPACES

    control::PlannerData data(si);
    std::vector<base::State*> states;
    std::vector<control::Control*> controls;
    base::StateSamplerPtr stsamp = space->allocDefaultStateSampler();
    control::ControlSamplerPtr csamp = cspace->allocDefaultControlSampler();

    // Creating 1000 states, and adding them to PlannerData
    for (unsigned int i = 0; i < 1000; ++i)
    {
        states.push_back(space->allocState());
        stsamp->sampleUniform(states[i]);

        data.addVertex(PlannerDataTestVertex(states[i], i, i+1));
    }

    // Add a whole bunch of random edges
    unsigned int num_edges_to_add = 10000;
    ompl::RNG rng;

    for (unsigned int i = 0; i < num_edges_to_add; ++i)
    {
        unsigned int v2, v1 = rng.uniformInt(0, states.size()-1);
        do v2 = rng.uniformInt(0, states.size()-1); while (v2 == v1 || data.edgeExists(v1, v2));

        // allocate a random control
        controls.push_back(cspace->allocControl());
        csamp->sample(controls[i]);

        BOOST_CHECK( data.addEdge(v1, v2, PlannerDataTestEdge(controls[i], (double)i, i)) );
    }

    BOOST_CHECK_EQUAL ( data.numVertices(), states.size() );
    BOOST_CHECK_EQUAL ( data.numEdges(), num_edges_to_add );

    control::PlannerData data2(si);
    control::PlannerDataStorage storage;
    storage.store(data, "testdata");
    storage.load("testdata", data2);

    // Verify that data == data2
    BOOST_CHECK_EQUAL ( data2.numVertices(), states.size() );
    BOOST_CHECK_EQUAL ( data2.numEdges(), num_edges_to_add );

    // Checking vertices
    for (size_t i = 0; i < states.size(); ++i)
    {
        BOOST_CHECK ( space->equalStates(data2.getVertex(i).getState(), data.getVertex(i).getState()) );
        BOOST_CHECK ( data2.getVertex(i).getTag() == data.getVertex(i).getTag() );
        BOOST_CHECK ( static_cast<PlannerDataTestVertex&>(data2.getVertex(i)).tag2_ == static_cast<PlannerDataTestVertex&>(data.getVertex(i)).tag2_ );
    }

    // Checking edges
    for (size_t i = 0; i < states.size(); ++i)
    {
        std::vector<unsigned int> neighbors, neighbors2;
        data.getEdges(i, neighbors);
        data2.getEdges(i, neighbors2);

        std::sort (neighbors.begin(), neighbors.end());
        std::sort (neighbors2.begin(), neighbors2.end());
        BOOST_REQUIRE_EQUAL( neighbors.size(), neighbors2.size() );

        for (size_t j = 0; j < neighbors.size(); ++j)
        {
            BOOST_CHECK_EQUAL( neighbors[j], neighbors2[j] );

            auto &edge  = static_cast<PlannerDataTestEdge&>(data.getEdge(i, neighbors[j]));
            auto &edge2 = static_cast<PlannerDataTestEdge&>(data2.getEdge(i, neighbors[j]));
            BOOST_CHECK ( cspace->equalControls (edge.getControl(), edge2.getControl()) );
            BOOST_CHECK ( fabs(edge.getDuration() - edge2.getDuration()) < std::numeric_limits<double>::epsilon());
            BOOST_CHECK ( edge.id_ == edge2.id_ );
        }
    }

    for (auto & state : states)
        space->freeState(state);

    for (auto & control : controls)
        cspace->freeControl(control);
}
