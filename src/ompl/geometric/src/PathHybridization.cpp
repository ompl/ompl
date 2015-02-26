/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include "ompl/geometric/PathHybridization.h"
#include <boost/graph/dijkstra_shortest_paths.hpp>

namespace ompl
{
    namespace magic
    {
        /** \brief The fraction of the path length to consider as gap cost when aligning paths to be hybridized. */
        static const double GAP_COST_FRACTION = 0.05;
    }
}

ompl::geometric::PathHybridization::PathHybridization(const base::SpaceInformationPtr &si) :
    si_(si),
    stateProperty_(boost::get(vertex_state_t(), g_)),
    name_("PathHybridization")
{
    root_ = boost::add_vertex(g_);
    stateProperty_[root_] = NULL;
    goal_ = boost::add_vertex(g_);
    stateProperty_[goal_] = NULL;
}

ompl::geometric::PathHybridization::~PathHybridization()
{
}

void ompl::geometric::PathHybridization::clear()
{
    hpath_.reset();
    paths_.clear();

    g_.clear();
    root_ = boost::add_vertex(g_);
    stateProperty_[root_] = NULL;
    goal_ = boost::add_vertex(g_);
    stateProperty_[goal_] = NULL;
}

void ompl::geometric::PathHybridization::print(std::ostream &out) const
{
    out << "Path hybridization is aware of " << paths_.size() << " paths" << std::endl;
    int i = 1;
    for (std::set<PathInfo>::const_iterator it = paths_.begin() ; it != paths_.end() ; ++it, ++i)
        out << "  path " << i << " of length " << it->length_ << std::endl;
    if (hpath_)
        out << "Hybridized path of length " << hpath_->length() << std::endl;
}

const std::string& ompl::geometric::PathHybridization::getName() const
{
    return name_;
}

void ompl::geometric::PathHybridization::computeHybridPath()
{
    boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));
    boost::dijkstra_shortest_paths(g_, root_, boost::predecessor_map(prev));
    if (prev[goal_] != goal_)
    {
        PathGeometric *h = new PathGeometric(si_);
        for (Vertex pos = prev[goal_]; prev[pos] != pos; pos = prev[pos])
            h->append(stateProperty_[pos]);
        h->reverse();
        hpath_.reset(h);
    }
}

const ompl::base::PathPtr& ompl::geometric::PathHybridization::getHybridPath() const
{
    return hpath_;
}

unsigned int ompl::geometric::PathHybridization::recordPath(const base::PathPtr &pp, bool matchAcrossGaps)
{
    PathGeometric *p = dynamic_cast<PathGeometric*>(pp.get());
    if (!p)
    {
        OMPL_ERROR("Path hybridization only works for geometric paths");
        return 0;
    }

    if (p->getSpaceInformation() != si_)
    {
        OMPL_ERROR("Paths for hybridization must be from the same space information");
        return 0;
    }

    // skip empty paths
    if (p->getStateCount() == 0)
        return 0;

    PathInfo pi(pp);

    // if this path was previously included in the hybridization, skip it
    if (paths_.find(pi) != paths_.end())
        return 0;

    // the number of connection attempts
    unsigned int nattempts = 0;

    // start from virtual root
    Vertex v0 = boost::add_vertex(g_);
    stateProperty_[v0] = pi.states_[0];
    pi.vertices_.push_back(v0);

    // add all the vertices of the path, and the edges between them, to the HGraph
    // also compute the path length for future use (just for computational savings)
    const HGraph::edge_property_type prop0(0.0);
    boost::add_edge(root_, v0, prop0, g_);
    double length = 0.0;
    for (std::size_t j = 1 ; j < pi.states_.size() ; ++j)
    {
        Vertex v1 = boost::add_vertex(g_);
        stateProperty_[v1] = pi.states_[j];
        double weight = si_->distance(pi.states_[j-1], pi.states_[j]);
        const HGraph::edge_property_type properties(weight);
        boost::add_edge(v0, v1, properties, g_);
        length += weight;
        pi.vertices_.push_back(v1);
        v0 = v1;
    }

    // connect to virtual goal
    boost::add_edge(v0, goal_, prop0, g_);
    pi.length_ = length;

    // find matches with previously added paths
    for (std::set<PathInfo>::const_iterator it = paths_.begin() ; it != paths_.end() ; ++it)
    {
        const PathGeometric *q = static_cast<const PathGeometric*>(it->path_.get());
        std::vector<int> indexP, indexQ;
        matchPaths(*p, *q, (pi.length_ + it->length_) / (2.0 / magic::GAP_COST_FRACTION), indexP, indexQ);

        if (matchAcrossGaps)
        {
            int lastP = -1;
            int lastQ = -1;
            int gapStartP = -1;
            int gapStartQ = -1;
            bool gapP = false;
            bool gapQ = false;
            for (std::size_t i = 0 ; i < indexP.size() ; ++i)
            {
                // a gap is found in p
                if (indexP[i] < 0)
                {
                    // remember this as the beginning of the gap, if needed
                    if (!gapP)
                        gapStartP = i;
                    // mark the fact we are now in a gap on p
                    gapP = true;
                }
                else
                {
                    // check if a gap just ended;
                    // if it did, try to match the endpoint with the elements in q
                    if (gapP)
                        for (std::size_t j = gapStartP ; j < i ; ++j)
                        {
                            attemptNewEdge(pi, *it, indexP[i], indexQ[j]);
                            ++nattempts;
                        }
                    // remember the last non-negative index in p
                    lastP = i;
                    gapP = false;
                }
                if (indexQ[i] < 0)
                {
                    if (!gapQ)
                        gapStartQ = i;
                    gapQ = true;
                }
                else
                {
                    if (gapQ)
                        for (std::size_t j = gapStartQ ; j < i ; ++j)
                        {
                            attemptNewEdge(pi, *it, indexP[j], indexQ[i]);
                            ++nattempts;
                        }
                    lastQ = i;
                    gapQ = false;
                }

                // try to match corresponding index values and gep beginnings
                if (lastP >= 0 && lastQ >= 0)
                {
                    attemptNewEdge(pi, *it, indexP[lastP], indexQ[lastQ]);
                    ++nattempts;
                }
            }
        }
        else
        {
            // attempt new edge only when states align
            for (std::size_t i = 0 ; i < indexP.size() ; ++i)
                if (indexP[i] >= 0 && indexQ[i] >= 0)
                {
                    attemptNewEdge(pi, *it, indexP[i], indexQ[i]);
                    ++nattempts;
                }
        }
    }

    // remember this path is part of the hybridization
    paths_.insert(pi);
    return nattempts;
}

void ompl::geometric::PathHybridization::attemptNewEdge(const PathInfo &p, const PathInfo &q, int indexP, int indexQ)
{
    if (si_->checkMotion(p.states_[indexP], q.states_[indexQ]))
    {
        double weight = si_->distance(p.states_[indexP], q.states_[indexQ]);
        const HGraph::edge_property_type properties(weight);
        boost::add_edge(p.vertices_[indexP], q.vertices_[indexQ], properties, g_);
    }
}

std::size_t ompl::geometric::PathHybridization::pathCount() const
{
    return paths_.size();
}

void ompl::geometric::PathHybridization::matchPaths(const PathGeometric &p, const PathGeometric &q, double gapCost,
                                                    std::vector<int> &indexP, std::vector<int> &indexQ) const
{
    std::vector<std::vector<double> > C(p.getStateCount());
    std::vector<std::vector<char> >   T(p.getStateCount());

    for (std::size_t i = 0 ; i < p.getStateCount() ; ++i)
    {
        C[i].resize(q.getStateCount(), 0.0);
        T[i].resize(q.getStateCount(), '\0');
        for (std::size_t j = 0 ; j < q.getStateCount() ; ++j)
        {
            // as far as I can tell, there is a bug in the algorithm as presented in the paper
            // so I am doing things slightly differently ...
            double match = si_->distance(p.getState(i), q.getState(j)) + ((i > 0 && j > 0) ? C[i - 1][j - 1] : 0.0);
            double up    = gapCost + (i > 0 ? C[i - 1][j] : 0.0);
            double left  = gapCost + (j > 0 ? C[i][j - 1] : 0.0);
            if (match <= up && match <= left)
            {
                C[i][j] = match;
                T[i][j] = 'm';
            }
            else
                if (up <= match && up <= left)
                {
                    C[i][j] = up;
                    T[i][j] = 'u';
                }
                else
                {
                    C[i][j] = left;
                    T[i][j] = 'l';
                }
        }
    }
    // construct the sequences with gaps (only index positions)
    int m = p.getStateCount() - 1;
    int n = q.getStateCount() - 1;

    indexP.clear();
    indexQ.clear();
    indexP.reserve(std::max(n,m));
    indexQ.reserve(indexP.size());

    while (n >= 0 && m >= 0)
    {
        if (T[m][n] == 'm')
        {
            indexP.push_back(m);
            indexQ.push_back(n);
            --m; --n;
        }
        else
            if (T[m][n] == 'u')
            {
                indexP.push_back(m);
                indexQ.push_back(-1);
                --m;
            }
            else
            {
                indexP.push_back(-1);
                indexQ.push_back(n);
                --n;
            }
    }
    while (n >= 0)
    {
        indexP.push_back(-1);
        indexQ.push_back(n);
        --n;
    }
    while (m >= 0)
    {
        indexP.push_back(m);
        indexQ.push_back(-1);
        --m;
    }
}
