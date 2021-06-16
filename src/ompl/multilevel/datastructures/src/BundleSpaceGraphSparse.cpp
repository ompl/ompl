/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020,
 *  Max Planck Institute for Intelligent Systems (MPI-IS).
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
 *   * Neither the name of the MPI-IS nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
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

/* Author: Andreas Orthey, Sohaib Akbar */

#include <ompl/multilevel/datastructures/src/BundleSpaceGraphGoalVisitor.hpp>
#include <ompl/multilevel/datastructures/BundleSpaceGraphSparse.h>
#include <ompl/multilevel/datastructures/PlannerDataVertexAnnotated.h>
#include <ompl/multilevel/datastructures/graphsampler/VisibilityRegion.h>

// #include <ompl/base/objectives/PathLengthOptimizationObjective.h>
// #include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/util/Exception.h>
#include <ompl/control/PathControl.h>
#include <ompl/geometric/PathSimplifier.h>

#include <boost/property_map/vector_property_map.hpp>
#include <boost/property_map/transform_value_property_map.hpp>
#include <boost/foreach.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>  //same_component
#include <boost/math/constants/constants.hpp>
#include <boost/range/adaptor/map.hpp>

#define foreach BOOST_FOREACH

using namespace ompl::base;
using namespace ompl::multilevel;

BundleSpaceGraphSparse::BundleSpaceGraphSparse(const SpaceInformationPtr &si, BundleSpace *parent)
  : BaseT(si, parent), geomPath_(si)
{
    setName("BundleSpaceGraphSparse");

    Planner::declareParam<double>("sparse_delta_fraction", 
        this, 
        &BundleSpaceGraphSparse::setSparseDeltaFraction,
        &BundleSpaceGraphSparse::getSparseDeltaFraction, 
        "0.0:0.01:1.0");
}

BundleSpaceGraphSparse::~BundleSpaceGraphSparse()
{
}

void BundleSpaceGraphSparse::setGraphSampler(const std::string &sGraphSampler)
{
    if (sGraphSampler == "visibilityregion")
    {
        OMPL_DEBUG("Visibility Region Sampler Selected");
        graphSampler_ = std::make_shared<BundleSpaceGraphSamplerVisibilityRegion>(this);
    }
    else
    {
        BaseT::setGraphSampler(sGraphSampler);
    }
}

void BundleSpaceGraphSparse::setup()
{
    BaseT::setup();

    double maxExt = getBundle()->getMaximumExtent();
    sparseDelta_ = sparseDeltaFraction_ * maxExt;
    OMPL_DEBUG("Visibility region set to %f (max extent %f)", sparseDelta_, maxExt);

}

void BundleSpaceGraphSparse::clear()
{
    BaseT::clear();

    graphNeighborhood.clear();
    visibleNeighborhood.clear();
    // vrankSparse.clear();
    // vparentSparse.clear();
    Nold_v = 0;
    Nold_e = 0;

    consecutiveFailures_ = 0;

    startGoalVertexPath_.clear();
    lengthsStartGoalVertexPath_.clear();
}

void BundleSpaceGraphSparse::init()
{
    BaseT::init();
}

BundleSpaceGraphSparse::Vertex BundleSpaceGraphSparse::addConfiguration(Configuration *q)
{
    const Vertex v = BaseT::addConfiguration(q);
    consecutiveFailures_ = 0;
    return v;
}

bool BundleSpaceGraphSparse::addConfigurationConditional(Configuration *q)
{
    findGraphNeighbors(q, graphNeighborhood, visibleNeighborhood);

    if (!checkAddCoverage(q, visibleNeighborhood))
    {
        if (!checkAddConnectivity(q, visibleNeighborhood))
        {
            if (!checkAddInterface(q, graphNeighborhood, visibleNeighborhood))
            {
                // if (!hasTotalSpace())
                // {
                //     // optimality only on last level
                //     if (!checkAddPath(q))
                //     {
                //         ++consecutiveFailures_;
                //         return false;
                //     }
                // }
                // else
                {
                    ++consecutiveFailures_;
                    return false;
                }
            }
        }
    }
    return true;
}

void BundleSpaceGraphSparse::findGraphNeighbors(Configuration *q, std::vector<Configuration *> &graphNeighborhood,
                                                std::vector<Configuration *> &visibleNeighborhood)
{
    graphNeighborhood.clear();
    visibleNeighborhood.clear();

    nearestDatastructure_->nearestR(q, sparseDelta_, graphNeighborhood);

    for (Configuration *qn : graphNeighborhood)
    {
        if (getBundle()->checkMotion(q->state, qn->state))
        {
            visibleNeighborhood.push_back(qn);
        }
    }
}

bool BundleSpaceGraphSparse::checkAddCoverage(Configuration *q, std::vector<Configuration *> &visibleNeighborhood)
{
    // No free paths means we add for coverage
    if (visibleNeighborhood.empty())
    {
        addConfiguration(q);
        return true;
    }
    return false;
}

bool BundleSpaceGraphSparse::checkAddConnectivity(Configuration *q, std::vector<Configuration *> &visibleNeighborhood)
{
    // The sample q is able to connect to at least two nodes that are otherwise disconnected:
    std::vector<Vertex> links;
    if (visibleNeighborhood.size() > 1)
    {
        for (unsigned int i = 0; i < visibleNeighborhood.size(); i++)
        {
            for (unsigned int j = i + 1; j < visibleNeighborhood.size(); j++)
            {
                const Vertex &vi = visibleNeighborhood[i]->index;
                const Vertex &vj = visibleNeighborhood[j]->index;
                if (!sameComponent(vi, vj))
                {
                    links.push_back(vi);
                    links.push_back(vj);
                }
            }
        }

        if (!links.empty())
        {
            // there exist at least a pair of disconnected neighbors
            Vertex v = addConfiguration(q);

            for (Vertex link : links)
            {
                // (1) check that no previous edge exist
                if (!boost::edge(v, link, getGraph()).second)
                {
                    // And the components haven't been united by previous links
                    if (!sameComponent(link, v))
                    {
                        addEdge(v, link);
                    }
                }
            }
            return true;
        }
    }
    return false;
}

bool BundleSpaceGraphSparse::checkAddInterface(Configuration *q, std::vector<Configuration *> &graphNeighborhood,
                                               std::vector<Configuration *> &visibleNeighborhood)
{
    // Pairs of nodes that share an interface to also be connected with an edge
    // If we have more than 1 or 0 neighbors
    if (visibleNeighborhood.size() > 1)
    {
        // Add q if sample q reveals the existence of an interface between two nodes that do not share an edge
        Configuration *qn0 = graphNeighborhood[0];
        Configuration *qn1 = graphNeighborhood[1];
        Configuration *qv0 = visibleNeighborhood[0];
        Configuration *qv1 = visibleNeighborhood[1];

        if (qn0 == qv0 && qn1 == qv1)
        {
            // If our two closest neighbors don't share an edge
            const Vertex &v0idx = qv0->index;
            const Vertex &v1idx = qv1->index;
            if (!boost::edge(v0idx, v1idx, getGraph()).second)
            {
                // If they can be directly connected
                if (getBundle()->checkMotion(qv0->state, qv1->state))
                {
                    addEdge(v0idx, v1idx);
                }
                else
                {
                    // Add the new node to the graph, to bridge the interface
                    Vertex v = addConfiguration(q);
                    addEdge(v, v0idx);
                    addEdge(v, v1idx);
                    return true;
                }
            }
        }
    }
    return false;
}

bool BundleSpaceGraphSparse::checkAddPath(Configuration *q)
{
    std::vector<Vertex> neigh;
    getInterfaceNeighborhood(q, neigh);

    if (!neigh.empty())
    {
        return false;
    }

    bool result = false;

    Vertex v = q->representativeIndex;

    std::set<Vertex> n_rep;
    foreach (Vertex qp, neigh)
        n_rep.insert(graph_[qp]->representativeIndex);

    std::vector<Vertex> Xs;
    // for each v' in n_rep
    for (auto it = n_rep.begin(); it != n_rep.end() && !result; ++it)
    {
        Vertex vp = *it;
        // Identify appropriate v" candidates => vpps
        std::vector<Vertex> VPPs;

        computeVPP(v, vp, VPPs);

        foreach (Vertex vpp, VPPs)
        {
            double s_max = 0;

            // Find the X nodes to test
            computeX(v, vp, vpp, Xs);

            // For each x in xs
            foreach (Vertex x, Xs)
            {
                // Compute/Retain MAXimum distance path thorugh S
                double dist = (distance(getGraph()[x], getGraph()[v]) + distance(getGraph()[v], getGraph()[vp])) / 2.0;
                if (dist > s_max)
                    s_max = dist;
            }

            std::deque<State *> bestDPath;  // DensePath
            Vertex best_qpp = boost::graph_traits<Graph>::null_vertex();
            double d_min = std::numeric_limits<double>::infinity();  // Insanely big number
            // For each vpp in vpps
            for (std::size_t j = 0; j < VPPs.size() && !result; ++j)
            {
                Vertex vpp = VPPs[j];
                // For each q", which are stored interface nodes on v for i(vpp,v)
                auto it = getGraph()[v]->interfaceIndexList.find(vpp);
                if (it != getGraph()[v]->interfaceIndexList.end())
                {
                    foreach (Vertex qpp, /*interfaceListsProperty_[v][vpp]*/ it->second)
                    {
                        // check that representatives are consistent
                        assert(/*representativesProperty_[qpp]*/ graph_[qpp]->representativeIndex == (int)v);

                        // If they happen to be the one and same node
                        if (q->index == (int)qpp)
                        {
                            bestDPath.push_front(q->state);
                            best_qpp = qpp;
                            d_min = 0;
                        }
                        else
                        {
                            // Compute/Retain MINimum distance path on D through q, q"
                            std::deque<State *> dPath;  // DensePath
                            computeDensePath(q->index, qpp, dPath);
                            if (!dPath.empty())
                            {
                                // compute path length
                                double length = 0.0;
                                std::deque<State *>::const_iterator jt = dPath.begin();
                                for (auto it = jt + 1; it != dPath.end(); ++it)
                                {
                                    length += getBundle()->distance(*jt, *it);
                                    jt = it;
                                }

                                if (length < d_min)
                                {
                                    d_min = length;
                                    bestDPath.swap(dPath);
                                    best_qpp = qpp;
                                }
                            }
                        }
                    }
                    // If the spanner property is violated for these paths
                    if (s_max > stretchFactor_ * d_min)
                    {
                        // Need to augment this path with the appropriate neighbor information
                        Vertex na = getInterfaceNeighbor(q->index, vp);
                        Vertex nb = getInterfaceNeighbor(best_qpp, vpp);

                        bestDPath.push_front(graph_[na]->state);
                        bestDPath.push_back(graph_[nb]->state);

                        // check consistency of representatives
                        assert(graph_[na]->representativeIndex == (int)vp &&
                               graph_[nb]->representativeIndex == (int)vpp);

                        // Add the dense path to the spanner
                        addPathToSpanner(bestDPath, vpp, vp);

                        // Report success
                        result = true;
                    }
                }
            }
        }
    }
    return result;
}

void BundleSpaceGraphSparse::updateRepresentatives(Configuration *q)
{
    // dense points needed to update
    std::vector<Configuration *> dense_points;
    nearestDatastructure_->nearestR(q, sparseDelta_ + denseDelta_, dense_points);

    for (Configuration *dense_point : dense_points)
    {
        // remove from representative lists
        removeFromRepresentatives(dense_point);

        // update representatives
        std::vector<Configuration *> graphNeighborhood;
        nearestDatastructure_->nearestR(dense_point, sparseDelta_, graphNeighborhood);

        for (Configuration *qn : graphNeighborhood)
            if (getBundle()->checkMotion(dense_point->state, qn->state))
            {
                dense_point->representativeIndex = qn->index;
                break;
            }
    }

    std::set<Vertex> interfaceRepresentatives;  // sparse

    for (Configuration *dense_point : dense_points)
    {
        if (dense_point->representativeIndex < 0)
            continue;
        Vertex rep = dense_point->representativeIndex;
        // Extract the representatives of any interface-sharing neighbors
        getInterfaceNeighborRepresentatives(dense_point, interfaceRepresentatives);

        // For sanity's sake, make sure we clear ourselves out of what this new rep might think of us
        removeFromRepresentatives(dense_point);

        // Add this vertex to it's representative's list for the other representatives
        addToRepresentatives(dense_point->index, rep, interfaceRepresentatives);
    }
}

void BundleSpaceGraphSparse::addToRepresentatives(Vertex q, Vertex rep,
                                                  const std::set<Vertex> &interfaceRepresentatives)
{
    // If this node supports no interfaces
    if (interfaceRepresentatives.empty())
    {
        // Add it to the pool of non-interface nodes
        bool new_insert = getGraph()[rep]->nonInterfaceIndexList.insert(q).second;
        // we expect this was not previously tracked
        if (!new_insert)
            assert(false);
    }
    else
    {
        // otherwise, for every neighbor representative
        foreach (Vertex v, interfaceRepresentatives)
        {
            auto it = getGraph()[rep]->interfaceIndexList.find(v);
            if (it != getGraph()[rep]->interfaceIndexList.end())
            {
                if (!it->second.insert(q).second)
                    assert(false);
            }
            else
            {
                std::set<normalized_index_type> list;
                list.insert(q);
                std::pair<normalized_index_type, std::set<normalized_index_type>> newinterface(v, list);
                if (!getGraph()[rep]->interfaceIndexList.insert(newinterface).second)
                    assert(false);
            }
        }
    }
}

void BundleSpaceGraphSparse::getInterfaceNeighborRepresentatives(Configuration *q,
                                                                 std::set<Vertex> &interfaceRepresentatives)
{
    interfaceRepresentatives.clear();

    // Get our representative
    Vertex rep = q->representativeIndex;
    // For each neighbor we are connected to
    foreach (Vertex n, boost::adjacent_vertices(q->index, graph_))
    {
        // Get his representative
        Vertex orep = graph_[n]->representativeIndex;
        // If that representative is not our own
        if (orep != rep)
            // If he is within denseDelta_
            if (distance(q, graph_[n]) < denseDelta_)
                // Include his rep in the set
                interfaceRepresentatives.insert(orep);
    }
}

void BundleSpaceGraphSparse::removeFromRepresentatives(Configuration *q)
{
    if (q->representativeIndex < 0)
        return;
    // Remove the node from the non-interface points (if there)
    getGraph()[q->representativeIndex]->nonInterfaceIndexList.erase(q->index);

    // From each of the interface lists
    std::unordered_map<normalized_index_type, std::set<normalized_index_type>> interfaceList =
        getGraph()[q->representativeIndex]->interfaceIndexList;

    for (std::unordered_map<normalized_index_type, std::set<normalized_index_type>>::iterator it =
             interfaceList.begin();
         it != interfaceList.end(); it++)
    {
        // Remove this node
        it->second.erase(q->index);
    }
}

void BundleSpaceGraphSparse::getInterfaceNeighborhood(Configuration *q, std::vector<Vertex> &interfaceNeighborhood)
{
    interfaceNeighborhood.clear();

    // Get our representative
    Vertex rep = q->representativeIndex;

    // For each neighbor we are connected to
    foreach (Vertex n, boost::adjacent_vertices(q->index, graph_))
    {
        // If neighbor representative is not our own
        if (graph_[n]->representativeIndex != (int)rep)
        {
            // If he is within denseDelta_
            if (distance(q, graph_[n]) < denseDelta_)
            {
                // Append him to the list
                interfaceNeighborhood.push_back(n);
            }
        }
    }
}

void BundleSpaceGraphSparse::computeVPP(Vertex v, Vertex vp, std::vector<Vertex> &VPPs)
{
    foreach (Vertex cvpp, boost::adjacent_vertices(v, getGraph()))
        if (cvpp != vp)
            if (!boost::edge(cvpp, vp, getGraph()).second)
                VPPs.push_back(cvpp);
}

void BundleSpaceGraphSparse::computeX(Vertex v, Vertex vp, Vertex vpp, std::vector<Vertex> &Xs)
{
    // x are nodes that share an interface and an edge with v, share an edge with v" but do not share with v'. I
    Xs.clear();
    foreach (Vertex cx, boost::adjacent_vertices(vpp, getGraph()))
        if (boost::edge(cx, v, getGraph()).second && !boost::edge(cx, vp, getGraph()).second)
        {
            auto it = getGraph()[vpp]->interfaceIndexList.find(cx);
            if (it != getGraph()[vpp]->interfaceIndexList.end())
                if (!it->second.empty())  // if (!interfaceListsProperty_[vpp][cx].empty())
                    Xs.push_back(cx);
        }
    Xs.push_back(vpp);
}

BundleSpaceGraph::Vertex BundleSpaceGraphSparse::getInterfaceNeighbor(Vertex q, Vertex rep)
{
    foreach (Vertex vp, boost::adjacent_vertices(q, graph_))
        if (/*representativesProperty_[vp]*/ graph_[vp]->representativeIndex == (int)rep)
            if (distance(graph_[q], graph_[vp]) <= denseDelta_)
                return vp;
    throw Exception(name_, "Vertex has no interface neighbor with given representative");
}

void BundleSpaceGraphSparse::computeDensePath(const Vertex &start, const Vertex &goal, std::deque<State *> &path)
{
    path.clear();
    // BaseT::getPathDenseGraphPath(start, goal, graph_, path);

    PathPtr dpath = BaseT::getPath(start, goal, graph_);
    std::vector<State *> states = std::static_pointer_cast<ompl::geometric::PathGeometric>(dpath)->getStates();

    std::move(begin(states), end(states), back_inserter(path));
}

bool BundleSpaceGraphSparse::addPathToSpanner(const std::deque<State *> &dense_path, Vertex vp, Vertex vpp)
{
    // First, check to see that the path has length
    if (dense_path.size() <= 1)
    {
        // The path is 0 length, so simply link the representatives
        addEdge(vp, vpp);
    }
    else
    {
        // We will need to construct a PathGeometric to do this.
        geomPath_.getStates().resize(dense_path.size());
        std::copy(dense_path.begin(), dense_path.end(), geomPath_.getStates().begin());

        // Attempt to simplify the path
        optimizer_->reduceVertices(geomPath_, geomPath_.getStateCount() * 2);

        // we are sure there are at least 2 points left on geomPath_

        std::vector<Vertex> added_nodes;
        added_nodes.reserve(geomPath_.getStateCount());
        for (std::size_t i = 0; i < geomPath_.getStateCount(); ++i)
        {
            // Add each guard
            Configuration *q_path = new Configuration(getBundle(), getBundle()->cloneState(geomPath_.getState(i)));
            Vertex ng = addConfiguration(q_path);
            added_nodes.push_back(ng);
        }
        // Link them up
        for (std::size_t i = 1; i < added_nodes.size(); ++i)
        {
            addEdge(added_nodes[i - 1], added_nodes[i]);
        }
        // link them to their representatives
        addEdge(added_nodes[0], vp);
        addEdge(added_nodes[added_nodes.size() - 1], vpp);
    }
    geomPath_.getStates().clear();
    return true;
}

const std::pair<BundleSpaceGraph::Edge, bool> BundleSpaceGraphSparse::addEdge(const Vertex a, const Vertex b)
{
    // consecutiveFailures_ = 0;  // reset consecutive failures
    auto e = BaseT::addEdge(a, b);
    if(sameComponent(a, vStart_))
    {
      //TODO: This cannot work like that. what if an edge is drawn between two
      //previously unconnected components? In that case we would need to update
      //the edges what we do not do right now.
        initialConnectedComponentEdges.push_back(e.first);
    }
    return e;
}

bool BundleSpaceGraphSparse::hasSparseGraphChanged()
{
    unsigned Nv = boost::num_vertices(getGraph());
    unsigned Ne = boost::num_edges(getGraph());
    if ((Nv > Nold_v) || (Ne > Nold_e))
    {
        Nold_v = Nv;
        Nold_e = Ne;
        return true;
    }
    return false;
}
void BundleSpaceGraphSparse::getPlannerData(ompl::base::PlannerData &data) const
{
    BaseT::getPlannerData(data);
    OMPL_DEBUG(" Sparse graph: %d/%d failures.", consecutiveFailures_, maxFailures_);
}
