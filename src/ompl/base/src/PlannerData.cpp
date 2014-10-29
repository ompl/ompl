/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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

#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerDataGraph.h"
#include "ompl/base/StateStorage.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/ScopedState.h"

#include <boost/graph/graphviz.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/version.hpp>
#if BOOST_VERSION < 105100
#warning Boost version >=1.51 is needed for ompl::base::PlannerData::printGraphML
#else
#include <boost/property_map/function_property_map.hpp>
#endif

// This is a convenient macro to cast the void* graph pointer as the
// Boost.Graph structure from PlannerDataGraph.h
#define graph_ reinterpret_cast<ompl::base::PlannerData::Graph*>(graphRaw_)

const ompl::base::PlannerDataEdge   ompl::base::PlannerData::NO_EDGE = ompl::base::PlannerDataEdge();
const ompl::base::PlannerDataVertex ompl::base::PlannerData::NO_VERTEX = ompl::base::PlannerDataVertex(0);
const unsigned int ompl::base::PlannerData::INVALID_INDEX = std::numeric_limits<unsigned int>::max();

ompl::base::PlannerData::PlannerData (const SpaceInformationPtr &si) : si_(si)
{
    graphRaw_ = new Graph();
}

ompl::base::PlannerData::~PlannerData ()
{
    freeMemory();

    if (graph_)
    {
        delete graph_;
        graphRaw_ = NULL;
    }
}

void ompl::base::PlannerData::clear ()
{
    freeMemory();
    decoupledStates_.clear();
}

void ompl::base::PlannerData::decoupleFromPlanner ()
{
    unsigned int count = 0;
    for (unsigned int i = 0; i < numVertices(); ++i)
    {
        PlannerDataVertex &vtx = getVertex(i);
        // If this vertex's state is not in the decoupled list, clone it and add it
        if (decoupledStates_.find(const_cast<State*>(vtx.getState())) == decoupledStates_.end())
        {
            const State *oldState = vtx.getState();
            State *clone = si_->cloneState(oldState);
            decoupledStates_.insert(clone);
            // Replacing the shallow state pointer with our shiny new clone
            vtx.state_ = clone;

            // Remove oldState from stateIndexMap
            stateIndexMap_.erase(oldState);
            // Add the new, cloned state to stateIndexMap
            stateIndexMap_[clone] = i;
            count++;
        }
    }
}

unsigned int ompl::base::PlannerData::getEdges (unsigned int v, std::vector<unsigned int>& edgeList) const
{
    std::pair<Graph::AdjIterator, Graph::AdjIterator> iterators = boost::adjacent_vertices(boost::vertex(v, *graph_), *graph_);

    edgeList.clear();
    boost::property_map<Graph::Type, boost::vertex_index_t>::type vertices = get(boost::vertex_index, *graph_);
    for (Graph::AdjIterator iter = iterators.first; iter != iterators.second; ++iter)
        edgeList.push_back(vertices[*iter]);

    return edgeList.size();
}

unsigned int ompl::base::PlannerData::getEdges (unsigned int v, std::map<unsigned int, const PlannerDataEdge*>& edgeMap) const
{
    std::pair<Graph::OEIterator, Graph::OEIterator> iterators = boost::out_edges(boost::vertex(v, *graph_), *graph_);

    edgeMap.clear();
    boost::property_map<Graph::Type, edge_type_t>::type edges = get(edge_type_t(), *graph_);
    boost::property_map<Graph::Type, boost::vertex_index_t>::type vertices = get(boost::vertex_index, *graph_);
    for (Graph::OEIterator iter = iterators.first; iter != iterators.second; ++iter)
        edgeMap[vertices[boost::target(*iter, *graph_)]] = boost::get(edges, *iter);

    return edgeMap.size();
}

unsigned int ompl::base::PlannerData::getIncomingEdges (unsigned int v, std::vector<unsigned int>& edgeList) const
{
    std::pair<Graph::IEIterator, Graph::IEIterator> iterators = boost::in_edges(boost::vertex(v, *graph_), *graph_);

    edgeList.clear();
    boost::property_map<Graph::Type, boost::vertex_index_t>::type vertices = get(boost::vertex_index, *graph_);
    for (Graph::IEIterator iter = iterators.first; iter != iterators.second; ++iter)
        edgeList.push_back(vertices[boost::source(*iter, *graph_)]);

    return edgeList.size();
}

unsigned int ompl::base::PlannerData::getIncomingEdges (unsigned int v, std::map<unsigned int, const PlannerDataEdge*> &edgeMap) const
{
    std::pair<Graph::IEIterator, Graph::IEIterator> iterators = boost::in_edges(boost::vertex(v, *graph_), *graph_);

    edgeMap.clear();
    boost::property_map<Graph::Type, edge_type_t>::type edges = get(edge_type_t(), *graph_);
    boost::property_map<Graph::Type, boost::vertex_index_t>::type vertices = get(boost::vertex_index, *graph_);
    for (Graph::IEIterator iter = iterators.first; iter != iterators.second; ++iter)
        edgeMap[vertices[boost::source(*iter, *graph_)]] = boost::get(edges, *iter);

    return edgeMap.size();
}

bool ompl::base::PlannerData::getEdgeWeight(unsigned int v1, unsigned int v2, Cost* weight) const
{
    Graph::Edge e;
    bool exists;
    boost::tie(e, exists) = boost::edge(boost::vertex(v1, *graph_), boost::vertex(v2, *graph_), *graph_);

    if (exists)
    {
        boost::property_map<Graph::Type, boost::edge_weight_t>::type edges = get(boost::edge_weight, *graph_);
        *weight = edges[e];
        return true;
    }

    return false;
}

bool ompl::base::PlannerData::setEdgeWeight(unsigned int v1, unsigned int v2, Cost weight)
{
    Graph::Edge e;
    bool exists;
    boost::tie(e, exists) = boost::edge(boost::vertex(v1, *graph_), boost::vertex(v2, *graph_), *graph_);

    if (exists)
    {
        boost::property_map<Graph::Type, boost::edge_weight_t>::type edges = get(boost::edge_weight, *graph_);
        edges[e] = weight;
    }

    return exists;
}

bool ompl::base::PlannerData::edgeExists (unsigned int v1, unsigned int v2) const
{
    Graph::Edge e;
    bool exists;

    boost::tie(e, exists) = boost::edge(boost::vertex(v1, *graph_), boost::vertex(v2, *graph_), *graph_);
    return exists;
}

bool ompl::base::PlannerData::vertexExists (const PlannerDataVertex &v) const
{
    return vertexIndex(v) != INVALID_INDEX;
}

unsigned int ompl::base::PlannerData::numVertices () const
{
    return boost::num_vertices(*graph_);
}

unsigned int ompl::base::PlannerData::numEdges () const
{
    return boost::num_edges(*graph_);
}

const ompl::base::PlannerDataVertex& ompl::base::PlannerData::getVertex (unsigned int index) const
{
    if (index >= boost::num_vertices(*graph_))
        return NO_VERTEX;

    boost::property_map<Graph::Type, vertex_type_t>::type vertices = get(vertex_type_t(), *graph_);
    return *(vertices[boost::vertex(index, *graph_)]);
}

ompl::base::PlannerDataVertex& ompl::base::PlannerData::getVertex (unsigned int index)
{
    if (index >= boost::num_vertices(*graph_))
        return const_cast<ompl::base::PlannerDataVertex&>(NO_VERTEX);

    boost::property_map<Graph::Type, vertex_type_t>::type vertices = get(vertex_type_t(), *graph_);
    return *(vertices[boost::vertex(index, *graph_)]);
}

const ompl::base::PlannerDataEdge& ompl::base::PlannerData::getEdge (unsigned int v1, unsigned int v2) const
{
    Graph::Edge e;
    bool exists;
    boost::tie(e, exists) = boost::edge(boost::vertex(v1, *graph_), boost::vertex(v2, *graph_), *graph_);

    if (exists)
    {
        boost::property_map<Graph::Type, edge_type_t>::type edges = get(edge_type_t(), *graph_);
        return *(boost::get(edges, e));
    }

    return NO_EDGE;
}

ompl::base::PlannerDataEdge& ompl::base::PlannerData::getEdge (unsigned int v1, unsigned int v2)
{
    Graph::Edge e;
    bool exists;
    boost::tie(e, exists) = boost::edge(boost::vertex(v1, *graph_), boost::vertex(v2, *graph_), *graph_);

    if (exists)
    {
        boost::property_map<Graph::Type, edge_type_t>::type edges = get(edge_type_t(), *graph_);
        return *(boost::get(edges, e));
    }

    return const_cast<ompl::base::PlannerDataEdge&>(NO_EDGE);
}

void ompl::base::PlannerData::printGraphviz (std::ostream& out) const
{
    boost::write_graphviz(out, *graph_);
}

namespace
{
    // Property map for extracting the edge weight of a graph edge as
    // a double for printGraphML.
    double edgeWeightAsDouble(ompl::base::PlannerData::Graph::Type &g,
                              ompl::base::PlannerData::Graph::Edge e)
    {
        return get(boost::edge_weight_t(), g)[e].value();
    }

    // Property map for extracting states as arrays of doubles
    std::string vertexCoords (ompl::base::PlannerData::Graph::Type &g,
                              ompl::base::ScopedState<>& s,
                              ompl::base::PlannerData::Graph::Vertex v)
    {
        s = *get(vertex_type_t(), g)[v]->getState();
        std::vector<double> coords(s.reals());
        std::ostringstream sstream;
        if (coords.size()>0)
        {
            sstream << coords[0];
            for (std::size_t i = 1; i < coords.size(); ++i)
                sstream << ',' << coords[i];
        }
        return sstream.str();
    }
}

void ompl::base::PlannerData::printGraphML (std::ostream& out) const
{
#if BOOST_VERSION < 105100
    OMPL_WARN("Boost version >=1.51 is needed for ompl::base::PlannerData::printGraphML");
#else
    // For some reason, make_function_property_map can't infer its
    // template arguments corresponding to edgeWeightAsDouble's type
    // signature. So, we have to use this horribly verbose
    // instantiation of the property map.
    //
    // \TODO Can we use make_function_property_map() here and have it
    // infer the property template arguments?
    boost::function_property_map<
        boost::function<double (ompl::base::PlannerData::Graph::Edge)>,
        ompl::base::PlannerData::Graph::Edge,
        double>
        weightmap(boost::bind(&edgeWeightAsDouble, *graph_, _1));
    ompl::base::ScopedState<> s(si_);
    boost::function_property_map<
        boost::function<std::string (ompl::base::PlannerData::Graph::Vertex)>,
        ompl::base::PlannerData::Graph::Vertex,
        std::string >
        coordsmap(boost::bind(&vertexCoords, *graph_, s, _1));


    // Not writing vertex or edge structures.
    boost::dynamic_properties dp;
    dp.property("weight", weightmap);
    dp.property("coords", coordsmap);

    boost::write_graphml(out, *graph_, dp);
#endif
}

unsigned int ompl::base::PlannerData::vertexIndex (const PlannerDataVertex &v) const
{
    std::map<const State*, unsigned int>::const_iterator it = stateIndexMap_.find(v.getState());
    if (it != stateIndexMap_.end())
        return it->second;
    return INVALID_INDEX;
}

unsigned int ompl::base::PlannerData::numStartVertices () const
{
    return startVertexIndices_.size();
}

unsigned int ompl::base::PlannerData::numGoalVertices () const
{
    return goalVertexIndices_.size();
}

unsigned int ompl::base::PlannerData::getStartIndex (unsigned int i) const
{
    if (i >= startVertexIndices_.size())
        return INVALID_INDEX;

    return startVertexIndices_[i];
}

unsigned int ompl::base::PlannerData::getGoalIndex (unsigned int i) const
{
    if (i >= goalVertexIndices_.size())
        return INVALID_INDEX;

    return goalVertexIndices_[i];
}

bool ompl::base::PlannerData::isStartVertex (unsigned int index) const
{
    return std::binary_search(startVertexIndices_.begin(), startVertexIndices_.end(), index);
}

bool ompl::base::PlannerData::isGoalVertex (unsigned int index) const
{
    return std::binary_search(goalVertexIndices_.begin(), goalVertexIndices_.end(), index);
}

const ompl::base::PlannerDataVertex& ompl::base::PlannerData::getStartVertex (unsigned int i) const
{
    if (i >= startVertexIndices_.size())
        return NO_VERTEX;

    return getVertex(startVertexIndices_[i]);
}

ompl::base::PlannerDataVertex& ompl::base::PlannerData::getStartVertex (unsigned int i)
{
    if (i >= startVertexIndices_.size())
        return const_cast<ompl::base::PlannerDataVertex&>(NO_VERTEX);

    return getVertex(startVertexIndices_[i]);
}

const ompl::base::PlannerDataVertex& ompl::base::PlannerData::getGoalVertex (unsigned int i) const
{
    if (i >= goalVertexIndices_.size())
        return NO_VERTEX;

    return getVertex(goalVertexIndices_[i]);
}

ompl::base::PlannerDataVertex& ompl::base::PlannerData::getGoalVertex (unsigned int i)
{
    if (i >= goalVertexIndices_.size())
        return const_cast<ompl::base::PlannerDataVertex&>(NO_VERTEX);

    return getVertex(goalVertexIndices_[i]);
}

unsigned int ompl::base::PlannerData::addVertex (const PlannerDataVertex &st)
{
    // Do not add vertices with null states
    if (st.getState() == NULL)
        return INVALID_INDEX;

    unsigned int index = vertexIndex(st);
    if (index == INVALID_INDEX) // Vertex does not already exist
    {
        // Clone the state to prevent object slicing when retrieving this object
        ompl::base::PlannerDataVertex *clone = st.clone();
        Graph::Vertex v = boost::add_vertex(clone, *graph_);
        boost::property_map<Graph::Type, boost::vertex_index_t>::type vertexIndexMap = get(boost::vertex_index, *graph_);

        // Insert this entry into the stateIndexMap_ for fast lookup
        stateIndexMap_[clone->getState()] = numVertices()-1;
        return vertexIndexMap[v];
    }
    return index;
}

unsigned int ompl::base::PlannerData::addStartVertex (const PlannerDataVertex &v)
{
    unsigned int index = addVertex(v);
    if (index != INVALID_INDEX)
        markStartState(v.getState());

    return index;
}

unsigned int ompl::base::PlannerData::addGoalVertex  (const PlannerDataVertex &v)
{
    unsigned int index = addVertex(v);

    if (index != INVALID_INDEX)
        markGoalState(v.getState());

    return index;
}

bool ompl::base::PlannerData::addEdge(unsigned int v1, unsigned int v2, const PlannerDataEdge &edge, Cost weight)
{
    // If either of the vertices do not exist, don't add an edge
    if (v1 >= numVertices() || v2 >= numVertices())
        return false;

     // If an edge already exists, do not add one
     if (edgeExists (v1, v2))
        return false;

    // Clone the edge to prevent object slicing
    ompl::base::PlannerDataEdge *clone = edge.clone();
    const Graph::edge_property_type properties(clone, weight);

    Graph::Edge e;
    bool added = false;
    tie(e, added) = boost::add_edge(boost::vertex(v1, *graph_), boost::vertex(v2, *graph_), properties, *graph_);

    if (!added)
        delete clone;

    return added;
}

bool ompl::base::PlannerData::addEdge (const PlannerDataVertex & v1, const PlannerDataVertex & v2, const PlannerDataEdge &edge, Cost weight)
{
    unsigned int index1 = addVertex(v1);
    unsigned int index2 = addVertex(v2);

    // If neither vertex was added or already exists, return false
    if (index1 == INVALID_INDEX && index2 == INVALID_INDEX)
        return false;

    // Only add the edge if both vertices exist
    if (index1 != INVALID_INDEX && index2 != INVALID_INDEX)
        return addEdge (index1, index2, edge, weight);

    return true;
}

bool ompl::base::PlannerData::removeVertex (const PlannerDataVertex &st)
{
    unsigned int index = vertexIndex (st);
    if (index != INVALID_INDEX)
        return removeVertex (index);
    return false;
}

bool ompl::base::PlannerData::removeVertex (unsigned int vIndex)
{
    if (vIndex >= boost::num_vertices(*graph_))
        return false;

    // Retrieve a list of all edge structures
    boost::property_map<Graph::Type, edge_type_t>::type edgePropertyMap = get(edge_type_t(), *graph_);

    // Freeing memory associated with outgoing edges of this vertex
    std::pair<Graph::OEIterator, Graph::OEIterator> oiterators = boost::out_edges(boost::vertex(vIndex, *graph_), *graph_);
    for (Graph::OEIterator iter = oiterators.first; iter != oiterators.second; ++iter)
        delete edgePropertyMap[*iter];

    // Freeing memory associated with incoming edges of this vertex
    std::pair<Graph::IEIterator, Graph::IEIterator> initerators = boost::in_edges(boost::vertex(vIndex, *graph_), *graph_);
    for (Graph::IEIterator iter = initerators.first; iter != initerators.second; ++iter)
        delete edgePropertyMap[*iter];

    // Remove this vertex from stateIndexMap_, and update the map
    stateIndexMap_.erase(getVertex(vIndex).getState());
    boost::property_map<Graph::Type, vertex_type_t>::type vertices = get(vertex_type_t(), *graph_);
    for (unsigned int i = vIndex+1; i < boost::num_vertices(*graph_); ++i)
         stateIndexMap_[vertices[boost::vertex(i, *graph_)]->getState()]--;

    // Remove this vertex from the start and/or goal index list, if it exists.  Update the lists.
    std::vector<unsigned int>::iterator it = std::find(startVertexIndices_.begin(), startVertexIndices_.end(), vIndex);
    if (it != startVertexIndices_.end())
        startVertexIndices_.erase(it);
    for (size_t i = 0; i < startVertexIndices_.size(); ++i)
        if (startVertexIndices_[i] > vIndex)
            startVertexIndices_[i]--;

    it = std::find(goalVertexIndices_.begin(), goalVertexIndices_.end(), vIndex);
    if (it != goalVertexIndices_.end())
        goalVertexIndices_.erase(it);
    for (size_t i = 0; i < goalVertexIndices_.size(); ++i)
        if (goalVertexIndices_[i] > vIndex)
            goalVertexIndices_[i]--;

    // If the state attached to this vertex was decoupled, free it here
    State *vtxState = const_cast<State*>(getVertex(vIndex).getState());
    if (decoupledStates_.find(vtxState) != decoupledStates_.end())
    {
        decoupledStates_.erase(vtxState);
        si_->freeState(vtxState);
        vtxState = NULL;
    }

    // Slay the vertex
    boost::clear_vertex(boost::vertex(vIndex, *graph_), *graph_);
    boost::property_map<Graph::Type, vertex_type_t>::type vertexTypeMap = get(vertex_type_t(), *graph_);
    delete vertexTypeMap[boost::vertex(vIndex, *graph_)];
    boost::remove_vertex(boost::vertex(vIndex, *graph_), *graph_);

    return true;
}

bool ompl::base::PlannerData::removeEdge (unsigned int v1, unsigned int v2)
{
    Graph::Edge e;
    bool exists;
    boost::tie(e, exists) = boost::edge(boost::vertex(v1, *graph_), boost::vertex(v2, *graph_), *graph_);

    if (!exists)
        return false;

    // Freeing memory associated with this edge
    boost::property_map<Graph::Type, edge_type_t>::type edges = get(edge_type_t(), *graph_);
    delete edges[e];

    boost::remove_edge(boost::vertex(v1, *graph_), boost::vertex(v2, *graph_), *graph_);
    return true;
}

bool ompl::base::PlannerData::removeEdge (const PlannerDataVertex &v1, const PlannerDataVertex &v2)
{
    unsigned int index1, index2;
    index1 = vertexIndex(v1);
    index2 = vertexIndex(v2);

    if (index1 == INVALID_INDEX || index2 == INVALID_INDEX)
        return false;

    return removeEdge (index1, index2);
}

bool ompl::base::PlannerData::tagState (const base::State *st, int tag)
{
    std::map<const State*, unsigned int>::const_iterator it = stateIndexMap_.find(st);
    if (it != stateIndexMap_.end())
    {
        getVertex(it->second).setTag(tag);
        return true;
    }
    return false;
}

bool ompl::base::PlannerData::markStartState (const base::State *st)
{
    // Find the index in the stateIndexMap_
    std::map<const State*, unsigned int>::const_iterator it = stateIndexMap_.find(st);
    if (it != stateIndexMap_.end())
    {
        if (!isStartVertex(it->second))
        {
            startVertexIndices_.push_back(it->second);
            // Sort the indices for quick lookup
            std::sort(startVertexIndices_.begin(), startVertexIndices_.end());
        }
        return true;
    }
    return false;
}

bool ompl::base::PlannerData::markGoalState (const base::State *st)
{
    // Find the index in the stateIndexMap_
    std::map<const State*, unsigned int>::const_iterator it = stateIndexMap_.find(st);
    if (it != stateIndexMap_.end())
    {
        if (!isGoalVertex(it->second))
        {
            goalVertexIndices_.push_back(it->second);
            // Sort the indices for quick lookup
            std::sort(startVertexIndices_.begin(), startVertexIndices_.end());
        }
        return true;
    }
    return false;
}

void ompl::base::PlannerData::computeEdgeWeights (const OptimizationObjective &opt)
{
    unsigned int nv = numVertices();
    for (unsigned int i = 0; i < nv; ++i)
    {
        std::map<unsigned int, const PlannerDataEdge*> nbrs;
        getEdges(i, nbrs);

        std::map<unsigned int, const PlannerDataEdge*>::const_iterator it;
        for (it = nbrs.begin(); it != nbrs.end(); ++it)
        {
            setEdgeWeight(i, it->first, opt.motionCost(getVertex(i).getState(),
                                                       getVertex(it->first).getState()));
        }
    }
}

void ompl::base::PlannerData::computeEdgeWeights ()
{
    // Create a PathLengthOptimizationObjective to compute the edge
    // weights according to state space distance
    ompl::base::PathLengthOptimizationObjective opt(si_);
    computeEdgeWeights(opt);
}

namespace
{
    // Used in minimum spanning tree
    ompl::base::Cost project2nd (ompl::base::Cost /*unused*/, ompl::base::Cost second)
    {
        return second;
    }
}

void ompl::base::PlannerData::extractMinimumSpanningTree (unsigned int v,
                                                          const base::OptimizationObjective &opt,
                                                          base::PlannerData &mst) const
{
    std::vector<ompl::base::PlannerData::Graph::Vertex> pred(numVertices());

    // This is how boost's minimum spanning tree is actually
    // implemented, except it lacks the generality for specifying our
    // own comparison function or zero/inf values.
    //
    // \TODO Once (https://svn.boost.org/trac/boost/ticket/9368) gets
    // into boost we can use the far more direct
    // boost::prim_minimum_spanning_tree().
    boost::dijkstra_shortest_paths
        (*graph_, v,
         boost::predecessor_map(&pred[0]).
         distance_compare(boost::bind(&base::OptimizationObjective::
                                      isCostBetterThan, &opt, _1, _2)).
         distance_combine(&project2nd).
         distance_inf(opt.infiniteCost()).
         distance_zero(opt.identityCost()));

    // Adding vertices to MST
    for (std::size_t i = 0; i < pred.size(); ++i)
    {
        if (isStartVertex(i))
            mst.addStartVertex(getVertex(i));
        else if (isGoalVertex(i))
            mst.addGoalVertex(getVertex(i));
        else
            mst.addVertex(getVertex(i));
    }

    // Adding edges to MST
    for (std::size_t i = 0; i < pred.size(); ++i)
    {
        if (pred[i] != i)
        {
            Cost c;
            getEdgeWeight(pred[i], i, &c);
            mst.addEdge(pred[i], i, getEdge(pred[i], i), c);
        }
    }
}

void ompl::base::PlannerData::extractReachable(unsigned int v, base::PlannerData &data) const
{
    // If this vertex already exists in data, return
    if (data.vertexExists(getVertex(v)))
        return;

    // Adding the vertex corresponding to v into data
    unsigned int idx;
    if (isStartVertex(v))
        idx = data.addStartVertex(getVertex(v));
    else if (isGoalVertex(v))
        idx = data.addGoalVertex(getVertex(v));
    else
        idx = data.addVertex(getVertex(v));

    assert (idx != INVALID_INDEX);

    std::map<unsigned int, const PlannerDataEdge*> neighbors;
    getEdges(v, neighbors);

    // Depth-first traversal of reachable graph
    std::map<unsigned int, const PlannerDataEdge*>::iterator it;
    for (it = neighbors.begin(); it != neighbors.end(); ++it)
    {
        extractReachable(it->first, data);
        Cost weight;
        getEdgeWeight(v, it->first, &weight);
        data.addEdge(idx, data.vertexIndex(getVertex(it->first)), *(it->second), weight);
    }
}

ompl::base::StateStoragePtr ompl::base::PlannerData::extractStateStorage() const
{
    GraphStateStorage *store = new GraphStateStorage(si_->getStateSpace());
    if (graph_)
    {
        // copy the states
        std::map<unsigned int, unsigned int> indexMap;
        for (std::map<const State*, unsigned int>::const_iterator it = stateIndexMap_.begin() ; it != stateIndexMap_.end() ; ++it)
        {
            indexMap[it->second] = store->size();
            store->addState(it->first);
        }

        // add the edges
        for (std::map<unsigned int, unsigned int>::const_iterator it = indexMap.begin() ; it != indexMap.end() ; ++it)
        {
            std::vector<unsigned int> edgeList;
            getEdges(it->first, edgeList);
            GraphStateStorage::MetadataType &md = store->getMetadata(it->second);
            md.resize(edgeList.size());
            // map node indices to index values in StateStorage
            for (std::size_t k = 0 ; k < edgeList.size() ; ++k)
                md[k] = indexMap[edgeList[k]];
        }
    }
    return StateStoragePtr(store);
}

ompl::base::PlannerData::Graph& ompl::base::PlannerData::toBoostGraph()
{
    ompl::base::PlannerData::Graph *boostgraph = reinterpret_cast<ompl::base::PlannerData::Graph*>(graphRaw_);
    return *boostgraph;
}

const ompl::base::PlannerData::Graph& ompl::base::PlannerData::toBoostGraph() const
{
    const ompl::base::PlannerData::Graph *boostgraph = reinterpret_cast<const ompl::base::PlannerData::Graph*>(graphRaw_);
    return *boostgraph;
}

const ompl::base::SpaceInformationPtr& ompl::base::PlannerData::getSpaceInformation() const
{
    return si_;
}

void ompl::base::PlannerData::freeMemory()
{
    // Freeing decoupled states, if any
    for (std::set<State*>::iterator it = decoupledStates_.begin(); it != decoupledStates_.end(); ++it)
        si_->freeState(*it);

    if (graph_)
    {
        std::pair<Graph::EIterator, Graph::EIterator> eiterators = boost::edges(*graph_);
        boost::property_map<Graph::Type, edge_type_t>::type edges = get(edge_type_t(), *graph_);
        for (Graph::EIterator iter = eiterators.first; iter != eiterators.second; ++iter)
            delete boost::get(edges, *iter);

        std::pair<Graph::VIterator, Graph::VIterator> viterators = boost::vertices(*graph_);
        boost::property_map<Graph::Type, vertex_type_t>::type vertices = get(vertex_type_t(), *graph_);
        for (Graph::VIterator iter = viterators.first; iter != viterators.second; ++iter)
           delete vertices[*iter];

        graph_->clear();
    }
}

bool ompl::base::PlannerData::hasControls() const
{
    return false;
}
