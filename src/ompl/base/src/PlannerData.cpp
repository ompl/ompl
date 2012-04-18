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

#include <boost/graph/graphviz.hpp>

// This is a convenient macro to cast the void* graph pointer as the 
// Boost.Graph structure from PlannerDataGraph.h
#define graph_ static_cast<ompl::base::PlannerData::Graph*>(graph)

ompl::base::PlannerData::PlannerData (void)
{
    graph = new Graph();
}

ompl::base::PlannerData::~PlannerData (void)
{
    clear();
    if (graph_)
    {
        delete graph_;
        graph = NULL;
    }
}

void ompl::base::PlannerData::clear (void)
{
    if (graph_)
    {
        std::pair<Graph::EIterator, Graph::EIterator> eiterators = boost::edges(*graph_);
        typename boost::property_map<Graph::Type, edge_type_t>::type edges = get(edge_type, *graph_);
        for (Graph::EIterator iter = eiterators.first; iter != eiterators.second; ++iter)
            delete edges[*iter];

        std::pair<Graph::VIterator, Graph::VIterator> viterators = boost::vertices(*graph_);
        for (Graph::VIterator iter = viterators.first; iter != viterators.second; ++iter)
            delete (*graph_)[*iter];

        graph_->clear();
    }
}

unsigned int ompl::base::PlannerData::getEdges (unsigned int v, std::vector<unsigned int>& edgeList) const
{
    std::pair<Graph::AdjIterator, Graph::AdjIterator> iterators;
    iterators = boost::adjacent_vertices(v, *graph_);

    edgeList.clear();
    for (Graph::AdjIterator iter = iterators.first; iter != iterators.second; ++iter)
        edgeList.push_back(*iter);

    return edgeList.size();
}

unsigned int ompl::base::PlannerData::getEdges (unsigned int v, std::map<unsigned int, const PlannerDataEdge*>& edgeMap) const
{
    std::pair<Graph::OEIterator, Graph::OEIterator> iterators;
    iterators = boost::out_edges(v, *graph_);

    edgeMap.clear();
    typename boost::property_map<Graph::Type, edge_type_t>::type edgePropertyMap = get(edge_type, *graph_);
    for (Graph::OEIterator iter = iterators.first; iter != iterators.second; ++iter)
        edgeMap[boost::target(*iter, *graph_)] = edgePropertyMap[*iter];

    return edgeMap.size();
}

double ompl::base::PlannerData::getEdgeWeight(unsigned int v1, unsigned int v2) const
{
    Graph::Edge e;
    bool exists;
    boost::tie(e, exists) = boost::edge(v1, v2, *graph_);

    if (exists)
    {
        typename boost::property_map<Graph::Type, boost::edge_weight_t>::type edges = get(boost::edge_weight, *graph_);
        return edges[e];
    }
    else
        return -1.0;
}

bool ompl::base::PlannerData::setEdgeWeight(unsigned int v1, unsigned int v2, double weight)
{
    Graph::Edge e;
    bool exists;
    boost::tie(e, exists) = boost::edge(v1, v2, *graph_);

    if (exists)
    {
        typename boost::property_map<Graph::Type, boost::edge_weight_t>::type edges = get(boost::edge_weight, *graph_);
        edges[e] = weight;
    }
    
    return exists;
}

bool ompl::base::PlannerData::edgeExists (unsigned int v1, unsigned int v2) const
{
    Graph::Edge e;
    bool exists;

    boost::tie(e, exists) = boost::edge(v1, v2, *graph_);
    return exists;
}

bool ompl::base::PlannerData::vertexExists (const PlannerDataVertex &v) const
{
    return vertexIndex(v) != std::numeric_limits<unsigned int>::max();
}

unsigned int ompl::base::PlannerData::numVertices (void) const
{
    return boost::num_vertices(*graph_);
}

unsigned int ompl::base::PlannerData::numEdges (void) const
{
    return boost::num_edges(*graph_);
}

const ompl::base::PlannerDataVertex& ompl::base::PlannerData::getVertex (unsigned int index) const
{
    return *(*graph_)[index];
}

const ompl::base::PlannerDataEdge& ompl::base::PlannerData::getEdge (unsigned int v1, unsigned int v2) const
{
    Graph::Edge e;
    bool exists;
    boost::tie(e, exists) = boost::edge(v1, v2, *graph_);

    if (exists)
    {
        typename boost::property_map<Graph::Type, edge_type_t>::type edges = get(edge_type, *graph_);
        return *(edges[e]);
    }
    // Not sure what to do here yet...
    #warning Have not decided what to do here yet...
}

ompl::base::PlannerDataVertex& ompl::base::PlannerData::getVertex (unsigned int index)
{
    return *(*graph_)[index];
}

ompl::base::PlannerDataEdge& ompl::base::PlannerData::getEdge (unsigned int v1, unsigned int v2)
{
    Graph::Edge e;
    bool exists;
    boost::tie(e, exists) = boost::edge(v1, v2, *graph_);

    if (exists)
    {
        typename boost::property_map<Graph::Type, edge_type_t>::type edges = get(edge_type, *graph_);
        return *(edges[e]);
    }
    // Not sure what to do here yet...
    #warning Have not decided what to do here yet...
}

void ompl::base::PlannerData::printGraphviz (std::ostream& out) const
{
    boost::write_graphviz(out, *graph_);
}

unsigned int ompl::base::PlannerData::vertexIndex (const PlannerDataVertex &v) const
{
    unsigned int index = std::numeric_limits<unsigned int>::max();
    std::pair<Graph::VIterator, Graph::VIterator> viterators = boost::vertices(*graph_);
    for (Graph::VIterator iter = viterators.first; iter != viterators.second && index == std::numeric_limits<unsigned int>::max(); ++iter)
    {        
        if ( (*(*graph_)[*iter]) == v)
            index = *iter;
    }

    return index;
}

unsigned int ompl::base::PlannerData::addVertex (const PlannerDataVertex &st)
{
    // Do not add vertices with null states
    if (st.getState() == NULL)
        return std::numeric_limits<unsigned int>::max();

    unsigned int index = vertexIndex(st);
    if (index == std::numeric_limits<unsigned int>::max()) // Vertex does not already exist
    {
        // Clone the state to prevent object slicing when retrieving this object
        ompl::base::PlannerDataVertex *clone = st.clone();

        Graph::Vertex v = boost::add_vertex(clone, *graph_);
        return v;
    }
    return index;
}

bool ompl::base::PlannerData::addEdge(unsigned int v1, unsigned int v2, const PlannerDataEdge &edge, double weight)
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
    tie(e, added) = boost::add_edge(v1, v2, properties, *graph_);

    if (!added)
        delete clone;

    return added;
}

bool ompl::base::PlannerData::addEdge (const PlannerDataVertex & v1, const PlannerDataVertex & v2, const PlannerDataEdge &edge, double weight)
{
    unsigned int index1 = addVertex(v1);
    unsigned int index2 = addVertex(v2);

    // If neither vertex was added or already exists, return false
    if (index1 == std::numeric_limits<unsigned int>::max() && index2 == std::numeric_limits<unsigned int>::max())
        return false;

    // Only add the edge if both vertices exist
    if (index1 != std::numeric_limits<unsigned int>::max() && index2 != std::numeric_limits<unsigned int>::max())
        return addEdge (index1, index2, edge, weight);

    return true;
}

bool ompl::base::PlannerData::removeVertex (const PlannerDataVertex &st)
{
    unsigned int index = vertexIndex (st);
    if (index < std::numeric_limits<unsigned int>::max())
    {
        return removeVertex (index);
    }
    else
        return false;
}

bool ompl::base::PlannerData::removeVertex (unsigned int vIndex)
{
    if (vIndex >= boost::num_vertices(*graph_))
        return false;
    
    // Retrieve a list of all edge structures    
    typename boost::property_map<Graph::Type, edge_type_t>::type edgePropertyMap = get(edge_type, *graph_);

    // Freeing memory associated with outgoing edges of this vertex
    std::pair<Graph::OEIterator, Graph::OEIterator> oiterators;
    oiterators = boost::out_edges(vIndex, *graph_);
    for (Graph::OEIterator iter = oiterators.first; iter != oiterators.second; ++iter)
        delete edgePropertyMap[*iter];

    // Freeing memory associated with incoming edges of this vertex
    std::pair<Graph::IEIterator, Graph::IEIterator> initerators;
    initerators = boost::in_edges(vIndex, *graph_);
    for (Graph::IEIterator iter = initerators.first; iter != initerators.second; ++iter)
        delete edgePropertyMap[*iter];

    // Slay the vertex
    boost::clear_vertex(vIndex, *graph_);    
    delete (*graph_)[vIndex];
    boost::remove_vertex(vIndex, *graph_);

    return true;
}

bool ompl::base::PlannerData::removeEdge (unsigned int v1, unsigned int v2)
{
    Graph::Edge e;
    bool exists;
    boost::tie(e, exists) = boost::edge(v1, v2, *graph_);

    if (!exists)
        return false;

    // Freeing memory associated with this edge
    typename boost::property_map<Graph::Type, edge_type_t>::type edges = get(edge_type, *graph_);
    delete edges[e];

    boost::remove_edge(v1, v2, *graph_);
    return true;
}

bool ompl::base::PlannerData::removeEdge (const PlannerDataVertex &v1, const PlannerDataVertex &v2)
{
    unsigned int index1, index2;
    index1 = vertexIndex(v1);
    index2 = vertexIndex(v2);

    if (index1 == std::numeric_limits<unsigned int>::max() || index2 == std::numeric_limits<unsigned int>::max())
        return false;

    return removeEdge (index1, index2);
}

bool ompl::base::PlannerData::tagState (const base::State* st, int tag)
{
    std::pair<Graph::VIterator, Graph::VIterator> viterators = boost::vertices(*graph_);
    for (Graph::VIterator iter = viterators.first; iter != viterators.second; ++iter)
    {
        if ((*graph_)[*iter]->getState() == st)
        {
            (*graph_)[*iter]->setTag(tag);
            return true;
        }
    }

    return false;
}

void ompl::base::PlannerData::computeEdgeWeights(const ompl::base::PlannerData::EdgeWeightFn& f)
{
    unsigned int nv = numVertices();
    for (unsigned int i = 0; i < nv; ++i)
    {   
        std::map<unsigned int, const PlannerDataEdge*> nbrs;
        getEdges(i, nbrs);
        
        std::map<unsigned int, const PlannerDataEdge*>::iterator it;
        for (it = nbrs.begin(); it != nbrs.end(); ++it)
            setEdgeWeight(i, it->first, f(getVertex(i), getVertex(it->first), *it->second));
    }
}

ompl::base::PlannerData::Graph& ompl::base::PlannerData::toBoostGraph(void)
{    
    ompl::base::PlannerData::Graph* boostgraph = static_cast<ompl::base::PlannerData::Graph*>(graph);
    return *boostgraph;
}

const ompl::base::PlannerData::Graph& ompl::base::PlannerData::toBoostGraph(void) const
{
    const ompl::base::PlannerData::Graph* boostgraph = static_cast<const ompl::base::PlannerData::Graph*>(graph);
    return *boostgraph;
}

