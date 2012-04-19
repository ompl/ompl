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

/// @cond IGNORE
// Installing custom vertex and edge properties
enum edge_type_t { edge_type };
enum vertex_type_t { vertex_type };
namespace boost
{
    BOOST_INSTALL_PROPERTY(edge, type);
    BOOST_INSTALL_PROPERTY(vertex, type);
}
/// @endcond

#define PlannerDataGraph boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, \
                                               boost::property<vertex_type_t, ompl::base::PlannerDataVertex*, \
                                               boost::property<boost::vertex_index_t, unsigned int> >, \
                                               boost::property<edge_type_t, ompl::base::PlannerDataEdge*, \
                                               boost::property<boost::edge_weight_t, double> > >

/// Wrapper class for the Boost.Graph representation of the PlannerData
class ompl::base::PlannerData::Graph : public PlannerDataGraph
{
public:
    typedef PlannerDataGraph Type;

    typedef boost::graph_traits<Type>::vertex_descriptor  Vertex;
    typedef boost::graph_traits<Type>::edge_descriptor    Edge;
    typedef boost::graph_traits<Type>::vertex_iterator    VIterator;
    typedef boost::graph_traits<Type>::edge_iterator      EIterator;
    typedef boost::graph_traits<Type>::in_edge_iterator   IEIterator;
    typedef boost::graph_traits<Type>::out_edge_iterator  OEIterator;
    typedef boost::graph_traits<Type>::adjacency_iterator AdjIterator;
};

#endif
