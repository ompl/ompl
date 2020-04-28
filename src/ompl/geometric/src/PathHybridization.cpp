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
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <utility>
#include <Eigen/Core>

namespace ompl
{
    namespace magic
    {
        /** \brief The fraction of the path length to consider as gap cost when aligning paths to be hybridized. */
        static const double GAP_COST_FRACTION = 0.05;
    }  // namespace magic
}  // namespace ompl

ompl::geometric::PathHybridization::PathHybridization(base::SpaceInformationPtr si)
  : si_(std::move(si))
  , obj_(new base::PathLengthOptimizationObjective(si_))
  , stateProperty_(boost::get(vertex_state_t(), g_))
  , name_("PathHybridization")
{
    root_ = boost::add_vertex(g_);
    stateProperty_[root_] = nullptr;
    goal_ = boost::add_vertex(g_);
    stateProperty_[goal_] = nullptr;
}

ompl::geometric::PathHybridization::PathHybridization(base::SpaceInformationPtr si, base::OptimizationObjectivePtr obj)
  : si_(std::move(si))
  , obj_(std::move(obj))
  , stateProperty_(boost::get(vertex_state_t(), g_))
  , name_("PathHybridization")
{
    std::stringstream ss;
    ss << "PathHybridization over " << obj_->getDescription() << " cost";
    name_ = ss.str();
    root_ = boost::add_vertex(g_);
    stateProperty_[root_] = nullptr;
    goal_ = boost::add_vertex(g_);
    stateProperty_[goal_] = nullptr;
}

ompl::geometric::PathHybridization::~PathHybridization() = default;

void ompl::geometric::PathHybridization::clear()
{
    hpath_.reset();
    paths_.clear();

    g_.clear();
    root_ = boost::add_vertex(g_);
    stateProperty_[root_] = nullptr;
    goal_ = boost::add_vertex(g_);
    stateProperty_[goal_] = nullptr;
}

void ompl::geometric::PathHybridization::print(std::ostream &out) const
{
    out << "Path hybridization is aware of " << paths_.size() << " paths" << std::endl;
    int i = 1;
    for (auto it = paths_.begin(); it != paths_.end(); ++it, ++i)
        out << "  path " << i << " of cost " << it->cost_.value() << std::endl;
    if (hpath_)
        out << "Hybridized path of cost " << hpath_->cost(obj_) << std::endl;
}

const std::string &ompl::geometric::PathHybridization::getName() const
{
    return name_;
}

void ompl::geometric::PathHybridization::computeHybridPath()
{
    boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));
    boost::dijkstra_shortest_paths(
        g_, root_,
        boost::predecessor_map(prev)
            .distance_compare([this](base::Cost c1, base::Cost c2) { return obj_->isCostBetterThan(c1, c2); })
            .distance_combine([this](base::Cost c1, base::Cost c2) { return obj_->combineCosts(c1, c2); })
            .distance_inf(obj_->infiniteCost())
            .distance_zero(obj_->identityCost()));
    if (prev[goal_] != goal_)
    {
        auto h(std::make_shared<PathGeometric>(si_));
        for (Vertex pos = prev[goal_]; prev[pos] != pos; pos = prev[pos])
            h->append(stateProperty_[pos]);
        h->reverse();
        hpath_ = h;
    }
    else
    {
        OMPL_WARN("No path to goal was found");
    }
}

const ompl::geometric::PathGeometricPtr &ompl::geometric::PathHybridization::getHybridPath() const
{
    return hpath_;
}

unsigned int ompl::geometric::PathHybridization::recordPath(const geometric::PathGeometricPtr &p, bool matchAcrossGaps)
{
    if (p->getSpaceInformation() != si_)
    {
        OMPL_ERROR("Paths for hybridization must be from the same space information");
        return 0;
    }

    // skip empty paths
    if (p->getStateCount() == 0)
        return 0;

    // interpolate path to get more potential crossover points between paths
    PathInfo pi(p);

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
    // also compute the path cost for future use (just for computational savings)
    const HGraph::edge_property_type prop0(obj_->identityCost());
    boost::add_edge(root_, v0, prop0, g_);
    base::Cost cost = obj_->identityCost();
    for (std::size_t j = 1; j < pi.states_.size(); ++j)
    {
        Vertex v1 = boost::add_vertex(g_);
        stateProperty_[v1] = pi.states_[j];
        base::Cost weight = obj_->motionCost(pi.states_[j - 1], pi.states_[j]);
        const HGraph::edge_property_type properties(weight);
        boost::add_edge(v0, v1, properties, g_);
        cost = obj_->combineCosts(cost, weight);
        pi.vertices_.push_back(v1);
        v0 = v1;
    }

    // connect to virtual goal
    boost::add_edge(v0, goal_, prop0, g_);
    pi.cost_ = cost;

    // find matches with previously added paths
    for (const auto &path : paths_)
    {
        const auto *q = static_cast<const PathGeometric *>(path.path_.get());
        std::vector<int> indexP, indexQ;
        matchPaths(*p, *q, obj_->combineCosts(pi.cost_, path.cost_).value() / (2.0 / magic::GAP_COST_FRACTION), indexP,
                   indexQ);

        if (matchAcrossGaps)
        {
            int lastP = -1;
            int lastQ = -1;
            int gapStartP = -1;
            int gapStartQ = -1;
            bool gapP = false;
            bool gapQ = false;
            for (std::size_t i = 0; i < indexP.size(); ++i)
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
                        for (std::size_t j = gapStartP; j < i; ++j)
                        {
                            attemptNewEdge(pi, path, indexP[i], indexQ[j]);
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
                        for (std::size_t j = gapStartQ; j < i; ++j)
                        {
                            attemptNewEdge(pi, path, indexP[j], indexQ[i]);
                            ++nattempts;
                        }
                    lastQ = i;
                    gapQ = false;
                }

                // try to match corresponding index values and gep beginnings
                if (lastP >= 0 && lastQ >= 0)
                {
                    attemptNewEdge(pi, path, indexP[lastP], indexQ[lastQ]);
                    ++nattempts;
                }
            }
        }
        else
        {
            // attempt new edge only when states align
            for (std::size_t i = 0; i < indexP.size(); ++i)
                if (indexP[i] >= 0 && indexQ[i] >= 0)
                {
                    attemptNewEdge(pi, path, indexP[i], indexQ[i]);
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
        base::Cost weight = obj_->motionCost(p.states_[indexP], q.states_[indexQ]);
        const HGraph::edge_property_type properties(weight);
        boost::add_edge(p.vertices_[indexP], q.vertices_[indexQ], properties, g_);
    }
}

std::size_t ompl::geometric::PathHybridization::pathCount() const
{
    return paths_.size();
}

void ompl::geometric::PathHybridization::matchPaths(const PathGeometric &p, const PathGeometric &q, double gapValue,
                                                    std::vector<int> &indexP, std::vector<int> &indexQ) const
{
    using CostMatrix = Eigen::Matrix<base::Cost, Eigen::Dynamic, Eigen::Dynamic>;
    using TMatrix = Eigen::Matrix<char, Eigen::Dynamic, Eigen::Dynamic>;
    CostMatrix C = CostMatrix::Constant(p.getStateCount(), q.getStateCount(), obj_->identityCost());
    TMatrix T = TMatrix::Constant(p.getStateCount(), q.getStateCount(), '\0');

    base::Cost gapCost(gapValue);
    for (std::size_t i = 0; i < p.getStateCount(); ++i)
    {
        for (std::size_t j = 0; j < q.getStateCount(); ++j)
        {
            // as far as I can tell, there is a bug in the algorithm as presented in the paper
            // so I am doing things slightly differently ...
            base::Cost match = obj_->combineCosts(obj_->motionCost(p.getState(i), q.getState(j)),
                                                  ((i > 0 && j > 0) ? C(i - 1, j - 1) : obj_->identityCost()));
            base::Cost up = obj_->combineCosts(gapCost, (i > 0 ? C(i - 1, j) : obj_->identityCost()));
            base::Cost left = obj_->combineCosts(gapCost, (j > 0 ? C(i, j - 1) : obj_->identityCost()));
            if (!(obj_->isCostBetterThan(up, match) || obj_->isCostBetterThan(left, match)))
            {
                C(i, j) = match;
                T(i, j) = 'm';
            }
            else if (!(obj_->isCostBetterThan(match, up) || obj_->isCostBetterThan(left, up)))
            {
                C(i, j) = up;
                T(i, j) = 'u';
            }
            else
            {
                C(i, j) = left;
                T(i, j) = 'l';
            }
        }
    }
    // construct the sequences with gaps (only index positions)
    int m = p.getStateCount() - 1;
    int n = q.getStateCount() - 1;

    indexP.clear();
    indexQ.clear();
    indexP.reserve(std::max(n, m));
    indexQ.reserve(indexP.size());

    while (n >= 0 && m >= 0)
    {
        if (T(m, n) == 'm')
        {
            indexP.push_back(m);
            indexQ.push_back(n);
            --m;
            --n;
        }
        else if (T(m, n) == 'u')
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
