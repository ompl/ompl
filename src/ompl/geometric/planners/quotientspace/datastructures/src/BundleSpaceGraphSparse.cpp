/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Stuttgart
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
 *   * Neither the name of the University of Stuttgart nor the names
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

#include "BundleSpaceGraphGoalVisitor.hpp"
#include <ompl/geometric/planners/quotientspace/datastructures/BundleSpaceGraphSparse.h>
#include <ompl/geometric/planners/quotientspace/datastructures/PlannerDataVertexAnnotated.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/util/Exception.h>
#include <ompl/control/PathControl.h>

#include <boost/property_map/vector_property_map.hpp>
#include <boost/property_map/transform_value_property_map.hpp>
#include <boost/foreach.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>  //same_component
#include <boost/math/constants/constants.hpp>
#include <boost/range/adaptor/map.hpp>

using namespace og;
#define foreach BOOST_FOREACH

BundleSpaceGraphSparse::BundleSpaceGraphSparse(const ob::SpaceInformationPtr &si, BundleSpace *parent):
  BaseT(si, parent), geomPath_(si)
{
  setName("BundleSpaceGraphSparse");
  Planner::declareParam<double>("sparse_delta_fraction", this, &BundleSpaceGraphSparse::setSparseDeltaFraction,
                                &BundleSpaceGraphSparse::getSparseDeltaFraction, "0.0:0.01:1.0");

  if (!isSetup())
  {
    setup();
  }
  pathVisibilityChecker_ = new PathVisibilityChecker(getBundle());
  psimp_ = std::make_shared<PathSimplifier>(getBundle());
  psimp_->freeStates(false);
}

BundleSpaceGraphSparse::~BundleSpaceGraphSparse()
{
  delete pathVisibilityChecker_;
}

void BundleSpaceGraphSparse::deleteConfiguration(Configuration *q)
{
    BaseT::deleteConfiguration(q);
}

void BundleSpaceGraphSparse::setup()
{
    BaseT::setup();
    if (!nearestSparse_)
    {
        nearestSparse_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
        nearestSparse_->setDistanceFunction(
            [this](const Configuration *a, const Configuration *b) 
            { 
                return distance(a, b); 
            });
    }


    double maxExt = getBundle()->getMaximumExtent();
    sparseDelta_ = sparseDeltaFraction_ * maxExt;
    pathBias_ = pathBiasFraction_ * maxExt;
    double d = (double) getBundle()->getStateDimension();
    double e = boost::math::constants::e<double>();
    kPRMStarConstant_ = e + (e / d);
}

void BundleSpaceGraphSparse::clear()
{
    BaseT::clear();

    if (nearestSparse_)
    {
        std::vector<Configuration *> configs;
        nearestSparse_->list(configs);
        if (configs.size() > 1)
        {
            for (auto &config : configs)
            {
                deleteConfiguration(config);
            }
        }
        nearestSparse_->clear();
    }
    graphSparse_.clear();

    selectedPath = -1;
    graphNeighborhood.clear();
    visibleNeighborhood.clear();
    vrankSparse.clear();
    vparentSparse.clear();
    v_start_sparse = -1;
    v_goal_sparse = -1;
    Nold_v = 0;
    Nold_e = 0;

    pathStackHead_.clear();
    pathStack_.clear();
}

void BundleSpaceGraphSparse::clearDynamic()
{
    // BaseT::clear();

    if (nearestSparse_)
    {
        std::vector<Configuration *> configs;
        nearestSparse_->list(configs);
        for (auto &config : configs)
        {
            if (config->state != qStart_->state)
                deleteConfiguration(config);
        }
        nearestSparse_->clear();
    }
    graphSparse_.clear();

    // selectedPath = -1;
    graphNeighborhood.clear();
    visibleNeighborhood.clear();
    vrankSparse.clear();
    vparentSparse.clear();
    Nold_v = 0;
    Nold_e = 0;

    const Vertex vl = add_vertex(qStart_, graphSparse_);
    nearestSparse_->add(qStart_);
    disjointSetsSparse_.make_set(vl);
    graphSparse_[vl]->index = vl;
}

const ompl::geometric::BundleSpaceGraph::Configuration *
ompl::geometric::BundleSpaceGraphSparse::nearest(const Configuration *q) const
{
    if (!isDynamic())
        return BaseT::nearest(q);
    else
    {
        return nearestSparse_->nearest(const_cast<Configuration *>(q));
    }
}

ompl::base::Cost ompl::geometric::BundleSpaceGraphSparse::costHeuristicSparse(Vertex u, Vertex v) const
{
    return opt_->motionCostHeuristic(graphSparse_[u]->state, graphSparse_[v]->state);
}

ompl::base::PathPtr ompl::geometric::BundleSpaceGraphSparse::getPathSparse(const Vertex &start, const Vertex &goal)
{
    std::vector<Vertex> prev(boost::num_vertices(graphSparse_));
    auto weight = boost::make_transform_value_property_map(std::mem_fn(&EdgeInternalState::getCost),
                                                           get(boost::edge_bundle, graphSparse_));
    try
    {
        boost::astar_search(graphSparse_, start, [this, goal](const Vertex v) { return costHeuristicSparse(v, goal); },
                            boost::predecessor_map(&prev[0])
                                .weight_map(weight)
                                .distance_compare([this](EdgeInternalState c1, EdgeInternalState c2) {
                                    return opt_->isCostBetterThan(c1.getCost(), c2.getCost());
                                })
                                .distance_combine([this](EdgeInternalState c1, EdgeInternalState c2) {
                                    return opt_->combineCosts(c1.getCost(), c2.getCost());
                                })
                                .distance_inf(opt_->infiniteCost())
                                .distance_zero(opt_->identityCost()));
    }
    catch (BundleSpaceGraphFoundGoal &)
    {
    }

    auto p(std::make_shared<PathGeometric>(getBundle()));
    if (prev[goal] == goal)
    {
        return nullptr;
    }

    std::vector<Vertex> vpath;
    for (Vertex pos = goal; prev[pos] != pos; pos = prev[pos])
    {
        graphSparse_[pos]->on_shortest_path = true;
        vpath.push_back(pos);
        p->append(graphSparse_[pos]->state);
    }
    graphSparse_[start]->on_shortest_path = true;
    vpath.push_back(start);
    p->append(graphSparse_[start]->state);

    shortestVertexPath_.clear();
    shortestVertexPath_.insert(shortestVertexPath_.begin(), vpath.rbegin(), vpath.rend());
    p->reverse();

    return p;
}

// unsigned int ompl::geometric::BundleSpaceGraphSparse::getNumberOfVertices() const
// {
//     return num_vertices(graphSparse_);
// }

// unsigned int ompl::geometric::BundleSpaceGraphSparse::getNumberOfEdges() const
// {
//     return num_edges(graphSparse_);
// }

void BundleSpaceGraphSparse::Init()
{
    BaseT::init();

    v_start_sparse = addConfigurationSparse(qStart_);
    graphSparse_[v_start_sparse]->isStart = true;
    qStart_->representativeIndex = v_start_sparse;

    v_goal_sparse = addConfigurationSparse(qGoal_);
    graphSparse_[v_start_sparse]->isGoal = true;
    qGoal_->representativeIndex = v_goal_sparse;

    //        //     Configuration *ql = new Configuration(Bundle, qGoal_->state);
    //        //     const Vertex vl = add_vertex(ql, graphSparse_);
    //        //     nearestSparse_->add(ql);
    //        //     disjointSetsSparse_.make_set(vl);
    //        //     graphSparse_[vl]->index = vl;

    //        //     v_goal_sparse = vl;
    //        //     graphSparse_[v_goal_sparse]->isGoal = true;

    //        //     assert(boost::num_vertices(graphSparse_) == 2);

    //        //     qGoal_->representativeIndex = v_goal_sparse;

    // BaseT::init(); //sa-> ? gives error maybe pis_.nextStart() can be called once so add base init() code here
    // BundleSpaceGraph init()
    //auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());
    //if (goal == nullptr)
    //{
    //    OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
    //    throw ompl::Exception("Unknown goal type");
    //}

    //if (const base::State *st = pis_.nextStart())
    //{
    //    if (st != nullptr)
    //    {
    //        // dense BundleSpaceGraph
    //        qStart_ = new Configuration(Bundle, st);
    //        qStart_->isStart = true;
    //        vStart_ = BaseT::addConfiguration(qStart_);

    //        //TODO: why not doing it in Sparse::addconfiguration ?
    //        v_start_sparse = addConfigurationSparse(qStart_);
    //        graphSparse_[v_start_sparse]->isStart = true;
    //        qStart_->representativeIndex = v_start_sparse;

    //        // Configuration *ql = new Configuration(Bundle, qStart_->state);
    //        // const Vertex vl = add_vertex(ql, graphSparse_);
    //        // nearestSparse_->add(ql);
    //        // disjointSetsSparse_.make_set(vl);
    //        // graphSparse_[vl]->index = vl;

    //        //
    //        //
    //        // Sohaib:
    //        // Sparse
    //        // // v_start_sparse = addConfigurationSparse(qStart_);
    //        // Configuration *ql = new Configuration(Bundle, qStart_->state);
    //        // const Vertex vl = add_vertex(ql, graphSparse_);
    //        // nearestSparse_->add(ql);
    //        // disjointSetsSparse_.make_set(vl);
    //        // graphSparse_[vl]->index = vl;

    //        // assert(boost::num_vertices(graphSparse_) == 1);
    //        // v_start_sparse = graphSparse_[0]->index;
    //        // graphSparse_[v_start_sparse]->isStart = true;

    //        // qStart_->representativeIndex = v_start_sparse;
    //    }
    //}
    //if (qStart_ == nullptr)
    //{
    //    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    //    throw ompl::Exception("Invalid initial states.");
    //}

    //if (const base::State *st = pis_.nextGoal())
    //{
    //    if (st != nullptr)
    //    {
    //        // dense BundleSpaceGraph
    //        qGoal_ = new Configuration(Bundle, st);
    //        qGoal_->isGoal = true;
    //        vGoal_ = BaseT::addConfiguration(qGoal_);  // sa-> (added) Q:- why goal state was not added in configuration


    //        v_goal_sparse = addConfigurationSparse(qGoal_);
    //        graphSparse_[v_goal_sparse]->isGoal = true;
    //        qGoal_->representativeIndex = v_goal_sparse;


    //        //     Configuration *ql = new Configuration(Bundle, qGoal_->state);
    //        //     const Vertex vl = add_vertex(ql, graphSparse_);
    //        //     nearestSparse_->add(ql);
    //        //     disjointSetsSparse_.make_set(vl);
    //        //     graphSparse_[vl]->index = vl;

    //        //     v_goal_sparse = vl;


    //        // sparse - not added in BundleSpaceGraph .: because it was added in grow() function
    //        // if (!isDynamic())
    //        // {
    //        //     // v_goal_sparse = addConfigurationSparse(qGoal_);
    //        //     Configuration *ql = new Configuration(Bundle, qGoal_->state);
    //        //     const Vertex vl = add_vertex(ql, graphSparse_);
    //        //     nearestSparse_->add(ql);
    //        //     disjointSetsSparse_.make_set(vl);
    //        //     graphSparse_[vl]->index = vl;

    //        //     v_goal_sparse = vl;
    //        //     graphSparse_[v_goal_sparse]->isGoal = true;

    //        //     assert(boost::num_vertices(graphSparse_) == 2);

    //        //     qGoal_->representativeIndex = v_goal_sparse;
    //        // }
    //    }
    //}
    //if (qGoal_ == nullptr)
    //{
    //    OMPL_ERROR("%s: There are no valid goal states!", getName().c_str());
    //    throw ompl::Exception("Invalid goal states.");
    //}
}

void BundleSpaceGraphSparse::uniteComponentsSparse(Vertex m1, Vertex m2)
{
    disjointSetsSparse_.union_set(m1, m2);
}

bool BundleSpaceGraphSparse::sameComponentSparse(Vertex m1, Vertex m2)
{
    return boost::same_component(m1, m2, disjointSetsSparse_);
}

BundleSpaceGraphSparse::Vertex BundleSpaceGraphSparse::addConfigurationSparse(Configuration *q)
{
    Configuration *ql = new Configuration(getBundle(), q->state);  // for sparse create new Configuration ***
    const Vertex vl = add_vertex(ql, graphSparse_);
    nearestSparse_->add(ql);
    disjointSetsSparse_.make_set(vl);
    graphSparse_[vl]->index = vl;
    updateRepresentatives(q);
    consecutiveFailures_ = 0;  // reset consecutive failures
    return vl;
}

void BundleSpaceGraphSparse::findGraphNeighbors(Configuration *q, std::vector<Configuration*> &graphNeighborhood,
                                                   std::vector<Configuration*> &visibleNeighborhood)
{
    graphNeighborhood.clear();
    visibleNeighborhood.clear();

    nearestSparse_->nearestR(q, sparseDelta_, graphNeighborhood);

    for (Configuration *qn : graphNeighborhood)
        if (getBundle()->checkMotion(q->state, qn->state))
            visibleNeighborhood.push_back(qn);
}

void BundleSpaceGraphSparse::addEdgeSparse(const Vertex a, const Vertex b)
{
    ob::Cost weight = opt_->motionCost(graphSparse_[a]->state, graphSparse_[b]->state);
    EdgeInternalState properties(weight);
    boost::add_edge(a, b, properties, graphSparse_);
    uniteComponentsSparse(a, b);
}

bool BundleSpaceGraphSparse::checkAddCoverage(Configuration *q, std::vector<Configuration *> &visibleNeighborhood)
{
    // No free paths means we add for coverage
    if (visibleNeighborhood.empty())
    {
        addConfigurationSparse(q);
        return true;
    }
    return false;
}

bool BundleSpaceGraphSparse::checkAddConnectivity(Configuration* q, std::vector<Configuration*> &visibleNeighborhood)
{
    // The sample q is able to connect to at least two nodes that are otherwise disconnected:
    std::vector<Vertex> links;
    if (visibleNeighborhood.size() > 1)
    {
        // For each neighbor
        for (std::size_t i = 0; i < visibleNeighborhood.size(); ++i)
        {
            // For each other neighbor
            for (std::size_t j = i + 1; j < visibleNeighborhood.size(); ++j)
            {
                // If they are in different components
                if (!sameComponentSparse(visibleNeighborhood[i]->index, visibleNeighborhood[j]->index))
                {
                    // If the paths between are collision free
                  //TODO: Check that we need to call CheckMotion
//                     /*if (Bundle->checkMotion(q->state, visibleNeighborhood[i]->state) &&
//                         Bundle->checkMotion(q->state, visibleNeighborhood[j]->state))*/
                    {
                        links.push_back(visibleNeighborhood[i]->index);
                        links.push_back(visibleNeighborhood[j]->index);
                    }
                }
            }
        }

        if (!links.empty())
        {
            Vertex v = addConfigurationSparse(q);

            for (Vertex link : links)
            {
                // If there's no edge
                if (!boost::edge(v, link, graphSparse_).second)
                {
                    // And the components haven't been united by previous links
                    if (!sameComponentSparse(link, v))
                    {
                        addEdgeSparse(v, link);
                    }
                }
            }
            return true;
        }
    }
    return false;
}

bool BundleSpaceGraphSparse::checkAddInterface(Configuration *q,
    std::vector<Configuration*> &graphNeighborhood, 
    std::vector<Configuration*> &visibleNeighborhood)
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
            if (!boost::edge(qv0->index, qv1->index, graphSparse_).second)
            {
                // If they can be directly connected
                if (getBundle()->checkMotion(qv0->state, qv1->state))
                {
                    addEdgeSparse(qv0->index, qv1->index);
                    consecutiveFailures_ = 0;  // reset consecutive failures
                }
                else
                {
                    // Add the new node to the graph, to bridge the interface
                    Vertex v = addConfigurationSparse(q);
                    addEdgeSparse(v, qv0->index);
                    addEdgeSparse(v, qv1->index);
                }
                return true;
            }
        }
    }
    return false;
}

void ompl::geometric::BundleSpaceGraphSparse::updateRepresentatives(Configuration *q)
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
        nearestSparse_->nearestR(dense_point, sparseDelta_, graphNeighborhood);

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

void ompl::geometric::BundleSpaceGraphSparse::addToRepresentatives(
    Vertex q, Vertex rep, const std::set<Vertex> &interfaceRepresentatives)
{
    // If this node supports no interfaces
    if (interfaceRepresentatives.empty())
    {
        // Add it to the pool of non-interface nodes
        bool new_insert = graphSparse_[rep]->nonInterfaceIndexList.insert(q).second;
        // we expect this was not previously tracked
        if (!new_insert)
            assert(false);
    }
    else
    {
        // otherwise, for every neighbor representative
        foreach (Vertex v, interfaceRepresentatives)
        {
            auto it = graphSparse_[rep]->interfaceIndexList.find(v);
            if (it != graphSparse_[rep]->interfaceIndexList.end())
            {
                if (!it->second.insert(q).second)
                    assert(false);
            }
            else
            {
                std::set<normalized_index_type> list;
                list.insert(q);
                std::pair<normalized_index_type, std::set<normalized_index_type>> newinterface(v, list);
                if (!graphSparse_[rep]->interfaceIndexList.insert(newinterface).second)
                    assert(false);
            }
        }
    }
}

void ompl::geometric::BundleSpaceGraphSparse::getInterfaceNeighborRepresentatives(
    Configuration *q, std::set<Vertex> &interfaceRepresentatives)
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

void ompl::geometric::BundleSpaceGraphSparse::removeFromRepresentatives(Configuration *q)
{
    if (q->representativeIndex < 0)
        return;
    // Remove the node from the non-interface points (if there)
    graphSparse_[q->representativeIndex]->nonInterfaceIndexList.erase(q->index);

    // From each of the interface lists
    std::unordered_map<normalized_index_type, std::set<normalized_index_type>> interfaceList =
        graphSparse_[q->representativeIndex]->interfaceIndexList;

    for (std::unordered_map<normalized_index_type, std::set<normalized_index_type>>::iterator it =
             interfaceList.begin();
         it != interfaceList.end(); it++)
    {
        // Remove this node
        it->second.erase(q->index);
    }
}

/////////////////////#############################################################
void ompl::geometric::BundleSpaceGraphSparse::getInterfaceNeighborhood(
    Configuration *q, std::vector<Vertex> &interfaceNeighborhood)
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

void ompl::geometric::BundleSpaceGraphSparse::computeVPP(Vertex v, Vertex vp, std::vector<Vertex> &VPPs)
{
    foreach (Vertex cvpp, boost::adjacent_vertices(v, graphSparse_))
        if (cvpp != vp)
            if (!boost::edge(cvpp, vp, graphSparse_).second)
                VPPs.push_back(cvpp);
}

void ompl::geometric::BundleSpaceGraphSparse::computeX(Vertex v, Vertex vp, Vertex vpp, std::vector<Vertex> &Xs)
{
    // x are nodes that share an interface and an edge with v, share an edge with v" but do not share with v'. I
    Xs.clear();
    foreach (Vertex cx, boost::adjacent_vertices(vpp, graphSparse_))
        if (boost::edge(cx, v, graphSparse_).second && !boost::edge(cx, vp, graphSparse_).second)
        {
            auto it = graphSparse_[vpp]->interfaceIndexList.find(cx);
            if (it != graphSparse_[vpp]->interfaceIndexList.end())
                if (!it->second.empty())  // if (!interfaceListsProperty_[vpp][cx].empty())
                    Xs.push_back(cx);
        }
    Xs.push_back(vpp);
}

ompl::geometric::BundleSpaceGraph::Vertex ompl::geometric::BundleSpaceGraphSparse::getInterfaceNeighbor(Vertex q, Vertex rep)
{
    foreach (Vertex vp, boost::adjacent_vertices(q, graph_))
        if (/*representativesProperty_[vp]*/ graph_[vp]->representativeIndex == (int)rep)
            if (distance(graph_[q], graph_[vp]) <= denseDelta_)
                return vp;
    throw Exception(name_, "Vertex has no interface neighbor with given representative");
}

void ompl::geometric::BundleSpaceGraphSparse::computeDensePath(const Vertex &start, const Vertex &goal,
                                                                 std::deque<base::State *> &path)
{
    path.clear();
    BaseT::getPathDenseGraphPath(start, goal, graph_, path);
}

bool ompl::geometric::BundleSpaceGraphSparse::addPathToSpanner(const std::deque<base::State *> &dense_path, Vertex vp,
                                                                 Vertex vpp)
{
    // First, check to see that the path has length
    if (dense_path.size() <= 1)
    {
        // The path is 0 length, so simply link the representatives
        addEdgeSparse(vp, vpp);
        consecutiveFailures_ = 0;  // resetFailures();
    }
    else
    {
        // We will need to construct a PathGeometric to do this.
        geomPath_.getStates().resize(dense_path.size());
        std::copy(dense_path.begin(), dense_path.end(), geomPath_.getStates().begin());

        // Attempt to simplify the path
        psimp_->reduceVertices(geomPath_, geomPath_.getStateCount() * 2);

        // we are sure there are at least 2 points left on geomPath_

        std::vector<Vertex> added_nodes;
        added_nodes.reserve(geomPath_.getStateCount());
        for (std::size_t i = 0; i < geomPath_.getStateCount(); ++i)
        {
            // Add each guard
            Configuration *q_path = new Configuration(getBundle(), getBundle()->cloneState(geomPath_.getState(i)));
            Vertex ng = addConfigurationSparse(q_path);
            added_nodes.push_back(ng);
        }
        // Link them up
        for (std::size_t i = 1; i < added_nodes.size(); ++i)
        {
            addEdgeSparse(added_nodes[i - 1], added_nodes[i]);
        }
        // link them to their representatives
        addEdgeSparse(added_nodes[0], vp);
        addEdgeSparse(added_nodes[added_nodes.size() - 1], vpp);
    }
    geomPath_.getStates().clear();
    return true;
}

//---------------------------------------------------------------------------------------------------------------------------********

bool ompl::geometric::BundleSpaceGraphSparse::checkAddPath(Configuration *q)
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
                double dist = (distance(graphSparse_[x], graphSparse_[v]) +
                               distance(graphSparse_[v], graphSparse_[vp])) /
                              2.0;
                if (dist > s_max)
                    s_max = dist;
            }

            std::deque<base::State *> bestDPath;  // DensePath
            Vertex best_qpp = boost::graph_traits<Graph>::null_vertex();
            double d_min = std::numeric_limits<double>::infinity();  // Insanely big number
            // For each vpp in vpps
            for (std::size_t j = 0; j < VPPs.size() && !result; ++j)
            {
                Vertex vpp = VPPs[j];
                // For each q", which are stored interface nodes on v for i(vpp,v)
                auto it = graphSparse_[v]->interfaceIndexList.find(vpp);
                if (it != graphSparse_[v]->interfaceIndexList.end())
                {
                    foreach (Vertex qpp, /*interfaceListsProperty_[v][vpp]*/ it->second)
                    {
                        // check that representatives are consistent
                        assert(/*representativesProperty_[qpp]*/ graph_[qpp]->representativeIndex == v);

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
                            std::deque<base::State *> dPath;  // DensePath
                            computeDensePath(q->index, qpp, dPath);
                            if (!dPath.empty())
                            {
                                // compute path length
                                double length = 0.0;
                                std::deque<base::State *>::const_iterator jt = dPath.begin();
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
                        assert(graph_[na]->representativeIndex == vp && graph_[nb]->representativeIndex == vpp);

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
//**************************************************************************************************************************************************
void BundleSpaceGraphSparse::sampleFromDatastructure(ob::State *q_random_graph)
{
    if (!getChild()->isDynamic() && pathStack_.size() > 0)
    {
        if (selectedPath >= 0 && selectedPath < (int)pathStack_.size())
        {

            std::vector<ob::State *> states = pathStackHead_.at(selectedPath);
            uint N = states.size();

            //############################################################################
            // Vertex Sampling
            // int k = rng_.uniformInt(0, N-1);
            // ob::State *state = states.at(k);
            // getBundle()->getStateSpace()->copyState(q_random_graph, state);
            // Bundle_sampler->sampleUniformNear(q_random_graph, q_random_graph, 0.2);

            //############################################################################
            // Edge Sampling
            uint k = rng_.uniformInt(0, N - 1);
            double r = rng_.uniform01();
            ob::State *s1 = states.at((k < N - 1) ? k : k - 1);
            ob::State *s2 = states.at((k < N - 1) ? k + 1 : k);

            getBundle()->getStateSpace()->interpolate(s1, s2, r, q_random_graph);

            Bundle_sampler_->sampleUniformNear(q_random_graph, q_random_graph, pathBias_);
        }
        else
        {
            OMPL_ERROR("Selected path is %d (have you selected a path?)");
            throw ompl::Exception("Unknown selected path");
        }
    }
    else
    {
        BaseT::sampleFromDatastructure(q_random_graph);
    }
}

unsigned int BundleSpaceGraphSparse::getNumberOfPaths() const
{
    return pathStackHead_.size();
}
//############################################################################
//############################################################################

void BundleSpaceGraphSparse::Rewire(Vertex &v)
{
    Configuration *q = graphSparse_[v];
    std::vector<Configuration *> neighbors;
    uint Nv = boost::degree(v, graphSparse_);
    uint K = Nv + 2;
    nearestSparse_->nearestK(const_cast<Configuration *>(q), K, neighbors);

    for (uint k = Nv + 1; k < neighbors.size(); k++)
    {
        Configuration *qn = neighbors.at(k);
        if(getBundle()->checkMotion(q->state, qn->state))
        {
            addEdge(q->index, qn->index);
        }
    }
}

void BundleSpaceGraphSparse::Rewire()
{
    Vertex v = boost::random_vertex(graphSparse_, rng_boost);
    return Rewire(v);
}

void BundleSpaceGraphSparse::removeLastPathFromStack()
{
    pathStackHead_.erase(pathStackHead_.end() - 1);
}

void BundleSpaceGraphSparse::pushPathToStack(std::vector<ob::State*> &path)
{
    og::PathGeometric gpath(getBundle());
    for(uint k = 0; k < path.size(); k++)
    {
        gpath.append(path.at(k));
    }

    ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(getBundle()));
    ob::OptimizationObjectivePtr clearObj(new ob::MaximizeMinClearanceObjective(getBundle()));
    ob::MultiOptimizationObjective* multiObj = new ob::MultiOptimizationObjective(getBundle());

    multiObj->addObjective(lengthObj, 1.0);
    multiObj->addObjective(clearObj, 1.0);
    ob::OptimizationObjectivePtr pathObj(multiObj);

    if(isDynamic())
    {
        OMPL_WARN("No Optimizer for dynamic paths specified.");
        // shortcutter.shortcutPath(gpath);
    }else{
        og::PathSimplifier shortcutter(getBundle(), ob::GoalPtr(), pathObj);
        //make sure that we have enough vertices so that the right path class is
        //visualized (problems with S1)
        if(getBundle()->getStateSpace()->getType() == ob::STATE_SPACE_SO2)
        {
            gpath.interpolate();
        }else{
            shortcutter.smoothBSpline(gpath);
            shortcutter.simplifyMax(gpath);
        }
    }

    if(!isDynamic() && !isProjectable(gpath.getStates()))
    {
      std::cout << "REJECTED (Not projectable)" << std::endl;
      numberOfFailedAddingPathCalls++;
      return;
    }

    if(!isDynamic() && !pathVisibilityChecker_->CheckValidity(gpath.getStates()))
    {
      std::cout << "REJECTED (Infeasible)" << std::endl;
      numberOfFailedAddingPathCalls++;
      return;
    }

    if (pathStack_.size() <= 0)
    {
        pathStack_.push_back(gpath);
    }
    else
    {
        for (uint k = 0; k < pathStack_.size(); k++)
        {
            og::PathGeometric &pathk = pathStack_.at(k);
            if (pathVisibilityChecker_->IsPathVisible(gpath.getStates(), pathk.getStates()))
            {
                std::cout << "REJECTED (Equal to path " << k << ")" << std::endl;
                numberOfFailedAddingPathCalls++;
                return;
            }
        }
        pathStack_.push_back(gpath);
    }
    std::cout << "Added to stack (" << pathStack_.size() << " paths on stack)" << std::endl;
}

void BundleSpaceGraphSparse::PrintPathStack()
{
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "Path Stack" << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    for(uint k = 0; k < pathStack_.size(); k++)
    {
        std::vector<ob::State*> pathk = pathStack_.at(k).getStates();
        for(uint j = 0; j < pathk.size(); j++)
        {
            getBundle()->printState(pathk.at(j));
        }
    }
}

void BundleSpaceGraphSparse::removeEdgeIfReductionLoop(const Edge &e)
{
    const Vertex v1 = boost::source(e, graphSparse_);
    const Vertex v2 = boost::target(e, graphSparse_);

    //############################################################################
    // (2) Get common neighbors of v1,v2
    std::vector<Vertex> v1_neighbors;
    std::vector<Vertex> v2_neighbors;
    std::vector<Vertex> common_neighbors;

    OEIterator edge_iter, edge_iter_end, next;

    boost::tie(edge_iter, edge_iter_end) = boost::out_edges(v1, graphSparse_);
    for (next = edge_iter; edge_iter != edge_iter_end; edge_iter = next)
    {
        const Vertex v_target = boost::target(*edge_iter, graphSparse_);
        if (v_target != v2)
            v1_neighbors.push_back(v_target);
        ++next;
    }
    boost::tie(edge_iter, edge_iter_end) = boost::out_edges(v2, graphSparse_);
    for (next = edge_iter; edge_iter != edge_iter_end; edge_iter = next)
    {
        const Vertex v_target = boost::target(*edge_iter, graphSparse_);
        if (v_target != v1)
            v2_neighbors.push_back(v_target);
        ++next;
    }

    for (uint k = 0; k < v1_neighbors.size(); k++)
    {
        for (uint j = 0; j < v2_neighbors.size(); j++)
        {
            const Vertex v1k = v1_neighbors.at(k);
            const Vertex v2k = v2_neighbors.at(j);
            if (v1k == v2k)
            {
                common_neighbors.push_back(v1k);
            }
        }
    }

    // rm duplicates
    std::sort(common_neighbors.begin(), common_neighbors.end());
    auto last = std::unique(common_neighbors.begin(), common_neighbors.end());
    common_neighbors.erase(last, common_neighbors.end());

    //############################################################################
    //  (3) Check if face (v1, v2, v3) is feasible
    for (uint k = 0; k < common_neighbors.size(); k++)
    {
        const Vertex v3 = common_neighbors.at(k);
        std::vector<Vertex> vpath1;
        vpath1.push_back(v1);
        vpath1.push_back(v3);
        vpath1.push_back(v2);
        std::vector<Vertex> vpath2;
        vpath2.push_back(v1);
        vpath2.push_back(v2);

        if (pathVisibilityChecker_->IsPathVisible(vpath1, vpath2, graphSparse_))
        {
            // RemoveEdge
            std::cout << "Removing Edge " << v1 << "<->" << v2 << std::endl;
            boost::remove_edge(v1, v2, graphSparse_);
        }
    }
}
void BundleSpaceGraphSparse::removeReducibleLoops()
{
    // Edge e = boost::random_edge(graphSparse_, rng_boost);
    uint Mend = boost::num_edges(graphSparse_);
    for (uint k = 0; k < Mend; k++)
    {
        Edge e = boost::random_edge(graphSparse_, rng_boost);
        removeEdgeIfReductionLoop(e);
    }
}

void BundleSpaceGraphSparse::freePath(std::vector<ob::State*> path, const ob::SpaceInformationPtr &si) const
{
    for (uint k = 0; k < path.size(); k++)
    {
        si->freeState(path.at(k));
    }
    path.clear();
}

std::vector<ob::State*> BundleSpaceGraphSparse::getProjectedPath(const std::vector<ob::State*> pathBundle, const ob::SpaceInformationPtr&) const
{
    std::vector<ob::State*> pathBase;
    for(uint k = 0; k < pathBundle.size(); k++)
    {
        const ob::State *qk = pathBundle.at(k);
        ob::State *qkProjected = getBase()->allocState();
        projectBase(qk, qkProjected);
        pathBase.push_back(qkProjected);
    }
    return pathBase;
}

bool BundleSpaceGraphSparse::isProjectable(const std::vector<ob::State*> &pathBundle) const
{
    return (getProjectionIndex(pathBundle) >= 0);
}

int BundleSpaceGraphSparse::getProjectionIndex(const std::vector<ob::State*> &pathBundle) const
{
    if(!hasParent()) 
    {
        return 0;
    }
    std::vector<ob::State*> pathBase = getProjectedPath(pathBundle, getBase());
    // for(uint k = 0; k < pathBundle.size(); k++){
    //   ob::State *qk = pathBundle.at(k);
    //   ob::State *qkProjected = getBase()->allocState();
    //   projectBase(qk, qkProjected);
    //   pathBase.push_back(qkProjected);
    // }

    BundleSpaceGraphSparse *parent = static_cast<BundleSpaceGraphSparse*>(parent_);
    unsigned int K = parent->getNumberOfPaths();

    for(uint k = 0; k < K; k++)
    {
      std::vector<ob::State*> pathBasek = parent->getKthPath(k);
      bool visible = parent->getPathVisibilityChecker()->IsPathVisible(pathBase, pathBasek);
      if(visible){
        freePath(pathBase, getBase());
        return k;
      }
    }
    freePath(pathBase, getBase());
    return -1;
}

void BundleSpaceGraphSparse::getPathIndices(const std::vector<ob::State*> &states, std::vector<int> &idxPath) const
{

  if(!hasParent())
  {
    return;
  }else{
    BundleSpaceGraphSparse *parent = static_cast<BundleSpaceGraphSparse*>(parent_);
    //TODO: we need to check here to which local minima we project. This is
    //necessary, since sometimes we find a path which actually projects on a
    //different Bundle-space path (and not the selected one).
    // APPROACH 1: Assign them all to selected path
    // BundleSpaceGraphSparse *Bundle = static_cast<BundleSpaceGraphSparse*>(parent_);
    // unsigned int K = getBundle()->getNumberOfPaths();
    // assert(K>0);
    // unsigned int Ks = getBundle()->selectedPath;
    // assert(Ks>=0);
    // idxPath.push_back(Ks);
    // getBundle()->getPathIndices(states, idxPath);
    if(isDynamic())
    {
      int Ks = parent->selectedPath;
      std::cout << "DYNAMIC Projection Index " << Ks << "| " << getName() << std::endl;
      idxPath.push_back(Ks);
    }else{
      int K = getProjectionIndex(states);
      std::cout << "Projection Index " << K << "| " << getName() << std::endl;
      if(K<0){
        K=0;
        OMPL_WARN("Projection not found. Possibly unprojectable path.");
      }
      idxPath.push_back(K);
      // getBundle()->getPathIndices(states, idxPath);
    }
    std::vector<ob::State*> pathBase = getProjectedPath(states, getBase());
    parent->getPathIndices(pathBase, idxPath);

    // APPROACH 2: Assign them to their projection
    //convert CS path to QS path
    //std::vector<ob::State*> pathcur;
    //for(uint k = 0; k < states.size(); k++){
    //  ob::State *qk = states.at(k);
    //  ob::State *qkProjected = getBase()->allocState();
    //  projectBase(qk, qkProjected);
    //  pathcur.push_back(qkProjected);
    //}
    ////Check which path can be deformed into QS path
    // BundleSpaceGraphSparse *Bundle = static_cast<BundleSpaceGraphSparse*>(parent_);
    // unsigned int K = getBundle()->getNumberOfPaths();
    // assert(K>0);

    // bool success = false;
    // for(uint k = 0; k < K; k++){
    //   std::vector<ob::State*> pathk = getBundle()->getKthPath(k);
    //   bool visible = getBundle()->getPathVisibilityChecker()->IsPathVisible(pathcur, pathk);
    //   if(visible){
    //     idxPath.push_back(k);
    //     getBundle()->getPathIndices(pathcur, idxPath);
    //     success = true;
    //     break;
    //   }
    // }
    //if(!success){
    //  //This path is not deformable into any of the BundleSpace paths 
    //  //One way to resolve this issue would be to add the new 
    //  OMPL_INFORM("Could not find projected path on BundleSpace. Creating new one.");

    //  getBundle()->removeLastPathFromStack();
    //  getBundle()->pushPathToStack(pathcur);
    //  idxPath.push_back(0);
    //  getBundle()->getPathIndices(pathcur, idxPath);
    //}else{
    //  //free all states
    //  for(uint k = 0; k < pathcur.size(); k++){
    //    getBase()->freeState(pathcur.at(k));
    //  }
    //}
  }
  // idxPath.insert(shortestVertexPath_.begin(), vpath.rbegin(), vpath.rend());
  // idxPath.insert(idxPath.begin(), idxPath.rbegin(), idxPath.rend());

}

PathVisibilityChecker* BundleSpaceGraphSparse::getPathVisibilityChecker()
{
    return pathVisibilityChecker_;
}

const std::vector<ob::State*> BundleSpaceGraphSparse::getKthPath(uint k) const
{
    return pathStackHead_.at(k);
}

// A recursive function to print all paths from 'u' to 'd'.
// visited[] keeps track of vertices in current path.
// path[] stores actual vertices and path_index is current
// index in path[]
void BundleSpaceGraphSparse::printAllPathsUtil(
		Vertex u, 
		Vertex d, 
		bool visited[], 
		int path[], 
		int &path_index) 
{
    // terminate if we have enough paths in stack
    if (pathStack_.size() > Nhead)
        return;
    if (numberOfFailedAddingPathCalls > 10)
        return;

    // Mark the current node and store it in path[]
    visited[u] = true;
    path[path_index] = u;
    path_index++;

    // If current vertex is same as destination, then print
    // current path[]
    if (u == d)
    {
        std::vector<ob::State *> pp;
        for (int i = 0; i < path_index; i++)
        {
            pp.push_back(graphSparse_[path[i]]->state);
        }
        pushPathToStack(pp);
    }
    else  // If current vertex is not destination
    {
        // Recur for all the vertices adjacent to current vertex
        OEIterator ei, ei_end;
        for (boost::tie(ei, ei_end) = boost::out_edges(u, graphSparse_); ei != ei_end; ++ei)
        {
            Vertex source = boost::source(*ei, graphSparse_);
            Vertex target = boost::target(*ei, graphSparse_);
            Vertex vnext = (source == u ? target : source);
            if (!visited[vnext])
            {
                printAllPathsUtil(vnext, d, visited, path, path_index);
                if (pathStack_.size() > Nhead)
                    break;
            }
        }
    }

    // Remove current vertex from path[] and mark it as unvisited
    path_index--;
    visited[u] = false;
}

bool BundleSpaceGraphSparse::hasSparseGraphChanged()
{
    unsigned Nv = boost::num_vertices(graphSparse_);
    unsigned Ne = boost::num_edges(graphSparse_);
    if ((Nv > Nold_v) || (Ne > Nold_e))
    {
        Nold_v = Nv;
        Nold_e = Ne;
        return true;
    }
    return false;
}

void BundleSpaceGraphSparse::enumerateAllPaths() 
{
    if (!hasSolution_)
        return;

    if (isDynamic())
    {
        ob::PathPtr path;

        const Configuration *q_nearest_to_goal = nearest(qGoal_);
        Configuration *qStartSparse = graphSparse_[v_start_sparse];
        path = getPathSparse(qStartSparse->index, q_nearest_to_goal->index);
        if (path == nullptr)
        {
            OMPL_WARN("No solution found, but hasSolution_ is set.");
            return;
        }
        og::PathGeometric &gpath = static_cast<og::PathGeometric &>(*path);
        // pathStack_.push_back(gpath);

        unsigned int kBefore = pathStack_.size();
        pushPathToStack(gpath.getStates());
        unsigned int kAfter = pathStack_.size();
        if (kAfter > kBefore)
            clearDynamic();
    }
    else
    {
        // Check if we already enumerated all paths. If yes, then the number of
        // vertices has not changed.
        if (!hasSparseGraphChanged())
        {
            return;
        }
        std::cout << "Enumerating paths on " << getName() << std::endl;

        // Remove Edges
        //(1) REDUCIBLE: Removal of reducible loops [Schmitzberger 02]
        removeReducibleLoops();
        //############################################################################

        // PathEnumerator pe(v_start_sparse, v_goal_sparse, graphSparse_);
        // pe.ComputePaths();

        unsigned numberVertices = boost::num_vertices(graphSparse_);
        if (numberVertices <= 0)
            return;
        bool *visited = new bool[numberVertices];
        std::cout << "Sparse Graph has " << boost::num_vertices(graphSparse_) << " vertices and "
                  << boost::num_edges(graphSparse_) << " edges." << std::endl;

        int *path = new int[numberVertices];
        int path_index = 0;  // Initialize path[] as empty

        for (unsigned int i = 0; i < numberVertices; i++)
            visited[i] = false;

        numberOfFailedAddingPathCalls = 0;

        printAllPathsUtil(v_start_sparse, v_goal_sparse, visited, path, path_index);
        //############################################################################
    }
    uint Npathsize = pathStack_.size();
    uint Npaths = std::min(Nhead, Npathsize);
    pathStackHead_.clear();
    for (uint k = 0; k < Npaths; k++)
    {
        // og::PathGeometric& pathK = (*(pathStack_.rbegin()+k));
        og::PathGeometric &pathK = pathStack_.at(k);  //*(pathStack_.rbegin()+k));
        pathStackHead_.push_back(pathK.getStates());
    }
    OMPL_INFORM("Found %d path classes.", pathStackHead_.size());
    OMPL_INFORM("%s", std::string(80, '-').c_str());

}

void BundleSpaceGraphSparse::getPlannerDataRoadmap(ob::PlannerData &data, std::vector<int> pathIdx) const
{
    foreach (const Vertex v, boost::vertices(graphSparse_))
    {
        ob::PlannerDataVertexAnnotated p(graphSparse_[v]->state);
        p.setLevel(level_);
        p.setPath(pathIdx);
        data.addVertex(p);
    }
    foreach (const Edge e, boost::edges(graphSparse_))
    {
        const Vertex v1 = boost::source(e, graphSparse_);
        const Vertex v2 = boost::target(e, graphSparse_);

        ob::PlannerDataVertexAnnotated p1(graphSparse_[v1]->state);
        ob::PlannerDataVertexAnnotated p2(graphSparse_[v2]->state);

        data.addEdge(p1, p2);
    }
}

void BundleSpaceGraphSparse::print(std::ostream &out) const
{
    BaseT::print(out);
    out << "   --[BundleSpaceGraphSparse has " 
        << boost::num_vertices(graphSparse_) << " vertices and " 
        << boost::num_edges(graphSparse_) << " edges.]"
        << std::endl;
}


std::vector<int> BundleSpaceGraphSparse::GetSelectedPathIndex() const
{
    std::vector<int> CurPath;
    BundleSpaceGraphSparse *pparent = static_cast<BundleSpaceGraphSparse*>(parent_);
    while(pparent!=nullptr)
    {
        CurPath.push_back(pparent->selectedPath);
        pparent = static_cast<BundleSpaceGraphSparse*>(pparent->parent_);
    }
    if (selectedPath < 0)
        CurPath.push_back(0);
    else
        CurPath.push_back(selectedPath);

    return CurPath;
}

bool ompl::geometric::BundleSpaceGraphSparse::getSolution(base::PathPtr &solution)
{
    if (hasSolution_)
    {
        solutionPath_ = getPath(v_start_sparse, v_goal_sparse, graphSparse_);
        startGoalVertexPath_ = shortestVertexPath_;
        solution = solutionPath_;

        lengthStartGoalVertexPath_ = 0;
        for(uint k = 1; k < startGoalVertexPath_.size(); k++){
          Configuration* xk = graphSparse_[startGoalVertexPath_.at(k)];
          Configuration* xkk = graphSparse_[startGoalVertexPath_.at(k-1)];
          double d = distance(xk, xkk);
          lengthsStartGoalVertexPath_.push_back(d);
          lengthStartGoalVertexPath_ += d;
        }
        return true;
    }
    else
    {
        base::Goal *g = pdef_->getGoal().get();
        bestCost_ = base::Cost(+base::dInf);
        bool same_component = sameComponent(v_start_sparse, v_goal_sparse);

        if (same_component &&
            g->isStartGoalPairValid(graphSparse_[v_goal_sparse]->state, graphSparse_[v_start_sparse]->state))
        {
            solutionPath_ = getPath(v_start_sparse, v_goal_sparse, graphSparse_);
            if (solutionPath_)
            {
                solution = solutionPath_;
                hasSolution_ = true;
                startGoalVertexPath_ = shortestVertexPath_;

                lengthStartGoalVertexPath_ = 0;
                for(uint k = 1; k < startGoalVertexPath_.size(); k++){
                  Configuration* xk = graphSparse_[startGoalVertexPath_.at(k)];
                  Configuration* xkk = graphSparse_[startGoalVertexPath_.at(k-1)];
                  double d = distance(xk, xkk);
                  lengthsStartGoalVertexPath_.push_back(d);
                  lengthStartGoalVertexPath_ += d;
                }
                return true;
            }
        }
    }
    return false;
}

void BundleSpaceGraphSparse::getPlannerData(base::PlannerData &data) const
{
    OMPL_DEBUG("Sparse Roadmap has %d/%d vertices/edges (Dense has %d/%d).", 
        boost::num_vertices(graphSparse_),
        boost::num_edges(graphSparse_), 
        boost::num_vertices(graph_), 
        boost::num_edges(graph_));

    if(boost::num_vertices(graphSparse_) > 1){
        BaseT::getPlannerDataGraph(data, graphSparse_, v_start_sparse, v_goal_sparse);
    }
}
