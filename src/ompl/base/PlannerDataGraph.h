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

#ifndef OMPL_BASE_PLANNER_DATA_GRAPH_
#define OMPL_BASE_PLANNER_DATA_GRAPH_

#include "ompl/base/PlannerData.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

enum edge_type_t { edge_type };
namespace boost { BOOST_INSTALL_PROPERTY(edge, type); }

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS,
                               ompl::base::PlannerDataVertex*,
                               boost::property<edge_type_t, ompl::base::PlannerDataEdge*,
                               boost::property<boost::edge_weight_t, double> > > GraphType;

class ompl::base::Graph : public GraphType
{
};

typedef boost::graph_traits<GraphType>::vertex_descriptor  Vertex;
typedef boost::graph_traits<GraphType>::edge_descriptor    Edge;
typedef boost::graph_traits<GraphType>::vertex_iterator    VIterator;
typedef boost::graph_traits<GraphType>::edge_iterator      EIterator;
typedef boost::graph_traits<GraphType>::in_edge_iterator   IEIterator;
typedef boost::graph_traits<GraphType>::out_edge_iterator  OEIterator;
typedef boost::graph_traits<GraphType>::adjacency_iterator AdjIterator;

ompl::base::Graph& ompl::base::PlannerData::toBoostGraph(void)
{    
    Graph* boostgraph = static_cast<Graph*>(graph);
    return *boostgraph;
}

#endif
