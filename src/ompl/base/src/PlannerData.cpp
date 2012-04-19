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
#include <boost/graph/graphml.hpp>

// This is a convenient macro to cast the void* graph pointer as the
// Boost.Graph structure from PlannerDataGraph.h
#define graph_ static_cast<ompl::base::PlannerData::Graph*>(graph)

const ompl::base::PlannerDataEdge   ompl::base::PlannerData::NO_EDGE = ompl::base::PlannerDataEdge();
const ompl::base::PlannerDataVertex ompl::base::PlannerData::NO_VERTEX = ompl::base::PlannerDataVertex(0);
const double ompl::base::PlannerData::INVALID_WEIGHT = std::numeric_limits<double>::infinity();

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
        typename boost::property_map<Graph::Type, edge_type_t>::type edges = get(edge_type_t(), *graph_);
        for (Graph::EIterator iter = eiterators.first; iter != eiterators.second; ++iter)
            delete boost::get(edges, *iter);

        std::pair<Graph::VIterator, Graph::VIterator> viterators = boost::vertices(*graph_);
        typename boost::property_map<Graph::Type, vertex_type_t>::type vertices = get(vertex_type_t(), *graph_);
        for (Graph::VIterator iter = viterators.first; iter != viterators.second; ++iter)
           delete vertices[*iter];

        graph_->clear();
    }
}

unsigned int ompl::base::PlannerData::getEdges (unsigned int v, std::vector<unsigned int>& edgeList) const
{
    std::pair<Graph::AdjIterator, Graph::AdjIterator> iterators = boost::adjacent_vertices(boost::vertex(v, *graph_), *graph_);

    edgeList.clear();
    typename boost::property_map<Graph::Type, boost::vertex_index_t>::type vertices = get(boost::vertex_index, *graph_);
    for (Graph::AdjIterator iter = iterators.first; iter != iterators.second; ++iter)
        edgeList.push_back(vertices[*iter]);

    return edgeList.size();
}

unsigned int ompl::base::PlannerData::getEdges (unsigned int v, std::map<unsigned int, const PlannerDataEdge*>& edgeMap) const
{
    std::pair<Graph::OEIterator, Graph::OEIterator> iterators = boost::out_edges(boost::vertex(v, *graph_), *graph_);

    edgeMap.clear();
    typename boost::property_map<Graph::Type, edge_type_t>::type edges = get(edge_type_t(), *graph_);
    typename boost::property_map<Graph::Type, boost::vertex_index_t>::type vertices = get(boost::vertex_index, *graph_);
    for (Graph::OEIterator iter = iterators.first; iter != iterators.second; ++iter)
        edgeMap[vertices[boost::target(*iter, *graph_)]] = boost::get(edges, *iter);

    return edgeMap.size();
}

double ompl::base::PlannerData::getEdgeWeight(unsigned int v1, unsigned int v2) const
{
    Graph::Edge e;
    bool exists;
    boost::tie(e, exists) = boost::edge(boost::vertex(v1, *graph_), boost::vertex(v2, *graph_), *graph_);

    if (exists)
    {
        typename boost::property_map<Graph::Type, boost::edge_weight_t>::type edges = get(boost::edge_weight, *graph_);
        return edges[e];
    }
    else
        return INVALID_WEIGHT;
}

bool ompl::base::PlannerData::setEdgeWeight(unsigned int v1, unsigned int v2, double weight)
{
    Graph::Edge e;
    bool exists;
    boost::tie(e, exists) = boost::edge(boost::vertex(v1, *graph_), boost::vertex(v2, *graph_), *graph_);

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

    boost::tie(e, exists) = boost::edge(boost::vertex(v1, *graph_), boost::vertex(v2, *graph_), *graph_);
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
    if (index >= boost::num_vertices(*graph_))
        return NO_VERTEX;

    typename boost::property_map<Graph::Type, vertex_type_t>::type vertices = get(vertex_type_t(), *graph_);
    return *(vertices[boost::vertex(index, *graph_)]);
}

ompl::base::PlannerDataVertex& ompl::base::PlannerData::getVertex (unsigned int index)
{
    if (index >= boost::num_vertices(*graph_))
        return const_cast<ompl::base::PlannerDataVertex&>(NO_VERTEX);

    typename boost::property_map<Graph::Type, vertex_type_t>::type vertices = get(vertex_type_t(), *graph_);
    return *(vertices[boost::vertex(index, *graph_)]);
}

const ompl::base::PlannerDataEdge& ompl::base::PlannerData::getEdge (unsigned int v1, unsigned int v2) const
{
    Graph::Edge e;
    bool exists;
    boost::tie(e, exists) = boost::edge(boost::vertex(v1, *graph_), boost::vertex(v2, *graph_), *graph_);

    if (exists)
    {
        typename boost::property_map<Graph::Type, edge_type_t>::type edges = get(edge_type_t(), *graph_);
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
        typename boost::property_map<Graph::Type, edge_type_t>::type edges = get(edge_type_t(), *graph_);
        return *(boost::get(edges, e));
    }

    return const_cast<ompl::base::PlannerDataEdge&>(NO_EDGE);
}

void ompl::base::PlannerData::printGraphviz (std::ostream& out) const
{
    boost::write_graphviz(out, *graph_);
}

void ompl::base::PlannerData::printGraphML(std::ostream& out) const
{
    // Not writing any dynamic properties (for now)
    boost::dynamic_properties dp;

    boost::write_graphml(out, *graph_, dp);
}

unsigned int ompl::base::PlannerData::vertexIndex (const PlannerDataVertex &v) const
{
    typename boost::property_map<Graph::Type, vertex_type_t>::type vertexTypeMap = get(vertex_type_t(), *graph_);
    typename boost::property_map<Graph::Type, boost::vertex_index_t>::type vertexIndexMap = get(boost::vertex_index, *graph_);

    unsigned int index = std::numeric_limits<unsigned int>::max();
    std::pair<Graph::VIterator, Graph::VIterator> viterators = boost::vertices(*graph_);
    for (Graph::VIterator iter = viterators.first; iter != viterators.second && index == std::numeric_limits<unsigned int>::max(); ++iter)
    {
        if (*(vertexTypeMap[*iter]) == v)
            index = vertexIndexMap[*iter];
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

        typename boost::property_map<Graph::Type, boost::vertex_index_t>::type vertexIndexMap = get(boost::vertex_index, *graph_);
        return vertexIndexMap[v];
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
    tie(e, added) = boost::add_edge(boost::vertex(v1, *graph_), boost::vertex(v2, *graph_), properties, *graph_);

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
    typename boost::property_map<Graph::Type, edge_type_t>::type edgePropertyMap = get(edge_type_t(), *graph_);

    // Freeing memory associated with outgoing edges of this vertex
    std::pair<Graph::OEIterator, Graph::OEIterator> oiterators;
    oiterators = boost::out_edges(boost::vertex(vIndex, *graph_), *graph_);
    for (Graph::OEIterator iter = oiterators.first; iter != oiterators.second; ++iter)
        delete edgePropertyMap[*iter];

    // Freeing memory associated with incoming edges of this vertex
    std::pair<Graph::IEIterator, Graph::IEIterator> initerators;
    initerators = boost::in_edges(boost::vertex(vIndex, *graph_), *graph_);
    for (Graph::IEIterator iter = initerators.first; iter != initerators.second; ++iter)
        delete edgePropertyMap[*iter];

    // Slay the vertex
    boost::clear_vertex(boost::vertex(vIndex, *graph_), *graph_);
    typename boost::property_map<Graph::Type, vertex_type_t>::type vertexTypeMap = get(vertex_type_t(), *graph_);
    //delete (*graph_)[vIndex];
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
    typename boost::property_map<Graph::Type, edge_type_t>::type edges = get(edge_type_t(), *graph_);
    delete edges[e];

    boost::remove_edge(boost::vertex(v1, *graph_), boost::vertex(v2, *graph_), *graph_);
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
        typename boost::property_map<Graph::Type, vertex_type_t>::type vertexTypeMap = get(vertex_type_t(), *graph_);
        if (vertexTypeMap[*iter]->getState() == st)
        {
            vertexTypeMap[*iter]->setTag(tag);
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

