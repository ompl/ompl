/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Willow Garage
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
*   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan, Ryan Luna */

#include "ompl/geometric/planners/prm/LazyPRM.h"
#include "ompl/base/OptimizationObjective.h"
#include <boost/graph/incremental_components.hpp>
#include <boost/graph/lookup_edge.hpp>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

ompl::geometric::LazyPRM::LazyPRM(const base::SpaceInformationPtr &si, bool starStrategy) :
    PRM(si, starStrategy),
    vertexValidityProperty_(boost::get(vertex_flags_t(), g_)),
    edgeValidityProperty_(boost::get(edge_flags_t(), g_))
{
    setName("LazyPRM");
}

ompl::geometric::LazyPRM::~LazyPRM()
{
}

ompl::geometric::PRM::Vertex ompl::geometric::LazyPRM::addMilestone(base::State *state)
{
    boost::mutex::scoped_lock _(graphMutex_);

    Vertex m = boost::add_vertex(g_);
    stateProperty_[m] = state;
    vertexValidityProperty_[m] = VALIDITY_UNKNOWN;

    // Initialize to its own (dis)connected component.
    disjointSets_.make_set(m);

    nn_->add(m);

    // Which milestones will we attempt to connect to?

    const std::vector<Vertex>& neighbors = connectionStrategy_(m);

    foreach (Vertex n, neighbors)
        if (connectionFilter_(m, n))
        {
            const base::Cost weight = opt_->motionCost(stateProperty_[m], stateProperty_[n]);
            const unsigned int id = maxEdgeID_++;
            const Graph::edge_property_type properties(weight, id);
            const Edge &e = boost::add_edge(m, n, properties, g_).first;
            edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
            uniteComponents(n, m);
        }

    return m;
}

void ompl::geometric::LazyPRM::growRoadmap(const base::PlannerTerminationCondition &ptc,
                                           base::State *workState)
{
    /* grow roadmap in lazy fashion -- add vertices and edges without checking validity */
    while (ptc == false)
    {
        simpleSampler_->sampleUniform(workState);
        addMilestone(si_->cloneState(workState));
    }
}

void ompl::geometric::LazyPRM::constructRoadmap(const base::PlannerTerminationCondition &ptc)
{
    if (!isSetup())
        setup();
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();

    growRoadmap(ptc);
}

ompl::base::PathPtr ompl::geometric::LazyPRM::constructGeometricPath(const boost::vector_property_map<Vertex> &prev, const Vertex &start, const Vertex &goal)
{
    // first, get the solution states without copying them
    std::vector<const base::State*> states;
    for (Vertex pos = goal; prev[pos] != pos; pos = prev[pos])
    {
        const base::State *st = stateProperty_[pos];
        unsigned int &vd = vertexValidityProperty_[pos];
        if ((vd & VALIDITY_TRUE) == 0)
            if (si_->isValid(st))
                vd |= VALIDITY_TRUE;
        if ((vd & VALIDITY_TRUE) == 0)
        {
            states.clear();
            // remove vertex from graph
            nn_->remove(pos);
            si_->freeState(stateProperty_[pos]);
            boost::clear_vertex(pos, g_);
            boost::remove_vertex(pos, g_);
            break;
        }
        states.push_back(st);
    }

    // check the edges too, if the vertices were valid
    if (!states.empty())
    {
        // start is checked for validity already
        states.push_back(stateProperty_[start]);

        std::vector<const base::State*>::const_reverse_iterator prevState = states.rbegin(), state = prevState + 1;
        Vertex prevVertex = goal, pos = prev[goal];
        do
        {
            Edge e = boost::lookup_edge(prevVertex, pos, g_).first;
            unsigned int &evd = edgeValidityProperty_[e];
            if ((evd & VALIDITY_TRUE) == 0)
                if (si_->checkMotion(*prevState, *state))
                    evd |= VALIDITY_TRUE;
            if ((evd & VALIDITY_TRUE) == 0)
            {
                states.clear();
                boost::remove_edge(e, g_);
                break;
            }
            prevState = state;
            state++;
            prevVertex = pos;
            pos = prev[pos];
        }
        while (prevVertex != pos);
    }

    if (states.empty())
        return base::PathPtr();

    PathGeometric *p = new PathGeometric(si_);
    for (std::vector<const base::State*>::const_reverse_iterator st = states.rbegin(); st != states.rend(); st++)
        p->append(*st);
    return base::PathPtr(p);
}
