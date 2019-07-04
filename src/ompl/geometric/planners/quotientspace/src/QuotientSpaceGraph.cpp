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

/* Author: Andreas Orthey */
#include "GoalVisitor.hpp"
#include "PlannerDataVertexAnnotated.h"
#include <ompl/geometric/planners/quotientspace/QuotientSpaceGraph.h>

#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/datastructures/PDF.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/tools/config/MagicConstants.h>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/property_map/transform_value_property_map.hpp>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH
using namespace og;
using namespace ob;
typedef QuotientSpaceGraph::Configuration Configuration;

QuotientSpaceGraph::QuotientSpaceGraph(const ob::SpaceInformationPtr &si, QuotientSpace *parent_) : BaseT(si, parent_)
{
    setName("QuotientSpaceGraph");
    specs_.recognizedGoal = ob::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = false;
    specs_.optimizingPaths = false;

    if (!isSetup())
    {
        setup();
    }
}

void QuotientSpaceGraph::setup()
{
    if (!nearestDatastructure_)
    {
        nearestDatastructure_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
        nearestDatastructure_->setDistanceFunction(
            [this](const Configuration *a, const Configuration *b) { return distance(a, b); });
    }

    if (pdef_)
    {
        BaseT::setup();
        if (pdef_->hasOptimizationObjective())
        {
            opt_ = pdef_->getOptimizationObjective();
        }
        else
        {
            opt_ = std::make_shared<ob::PathLengthOptimizationObjective>(si_);
        }
        firstRun_ = true;
        setup_ = true;
    }
    else
    {
        setup_ = false;
    }
}

QuotientSpaceGraph::~QuotientSpaceGraph()
{
    clear();
}
QuotientSpaceGraph::Configuration::Configuration(const base::SpaceInformationPtr &si) : state(si->allocState())
{
}
QuotientSpaceGraph::Configuration::Configuration(const base::SpaceInformationPtr &si, const ob::State *state_)
  : state(si->cloneState(state_))
{
}

void QuotientSpaceGraph::deleteConfiguration(Configuration *q)
{
    if (q != nullptr)
    {
        if (q->state != nullptr)
        {
            Q1->freeState(q->state);
        }
        delete q;
        q = nullptr;
    }
}
void QuotientSpaceGraph::clearVertices()
{
    if (nearestDatastructure_)
    {
        std::vector<Configuration *> configs;
        nearestDatastructure_->list(configs);
        for (auto &config : configs)
        {
            deleteConfiguration(config);
        }
        nearestDatastructure_->clear();
    }
    graph_.clear();
}

void QuotientSpaceGraph::clear()
{
    BaseT::clear();

    clearVertices();
    clearQuery();
    graphLength_ = 0;
    bestCost_ = ob::Cost(dInf);
    setup_ = false;
}

void QuotientSpaceGraph::clearQuery()
{
    pis_.restart();
}

double QuotientSpaceGraph::getImportance() const
{
    double N = (double)getNumberOfVertices();
    return 1.0 / (N + 1);
}

void QuotientSpaceGraph::init()
{
    auto *goal = dynamic_cast<ob::GoalSampleableRegion *>(pdef_->getGoal().get());
    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        exit(0);
    }

    if (const ob::State *st = pis_.nextStart())
    {
        if (st != nullptr)
        {
            qStart_ = new Configuration(Q1, st);
            qStart_->isStart = true;
            vStart_ = addConfiguration(qStart_);
        }
    }
    if (qStart_ == nullptr)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        exit(0);
    }

    if (const ob::State *st = pis_.nextGoal())
    {
        if (st != nullptr)
        {
            qGoal_ = new Configuration(Q1, st);
            qGoal_->isGoal = true;
        }
    }
    if (qGoal_ == nullptr)
    {
        OMPL_ERROR("%s: There are no valid goal states!", getName().c_str());
        exit(0);
    }
    // unsigned long int nrStartStates = boost::num_vertices(graph_);
    // OMPL_INFORM("%s: ready with %lu states already in datastructure", getName().c_str(), nrStartStates);
}

void QuotientSpaceGraph::uniteComponents(Vertex m1, Vertex m2)
{
    disjointSets_.union_set(m1, m2);
}

bool QuotientSpaceGraph::sameComponent(Vertex m1, Vertex m2)
{
    return boost::same_component(m1, m2, disjointSets_);
}

const QuotientSpaceGraph::Configuration *QuotientSpaceGraph::nearest(const Configuration *q) const
{
    return nearestDatastructure_->nearest(const_cast<Configuration *>(q));
}

QuotientSpaceGraph::Vertex QuotientSpaceGraph::addConfiguration(Configuration *q)
{
    Vertex m = boost::add_vertex(q, graph_);
    graph_[m]->total_connection_attempts = 1;
    graph_[m]->successful_connection_attempts = 0;
    // disjointSets_.make_set(m);
    // ConnectVertexToNeighbors(m);
    nearestDatastructure_->add(q);
    q->index = m;
    return m;
}
unsigned int QuotientSpaceGraph::getNumberOfVertices() const
{
    return num_vertices(graph_);
}
unsigned int QuotientSpaceGraph::getNumberOfEdges() const
{
    return num_edges(graph_);
}

const og::QuotientSpaceGraph::Graph &QuotientSpaceGraph::getGraph() const
{
    return graph_;
}
const og::QuotientSpaceGraph::RoadmapNeighborsPtr &QuotientSpaceGraph::getRoadmapNeighborsPtr() const
{
    return nearestDatastructure_;
}
ob::Cost QuotientSpaceGraph::costHeuristic(Vertex u, Vertex v) const
{
    return opt_->motionCostHeuristic(graph_[u]->state, graph_[v]->state);
}

template <template <typename T> class NN>
void QuotientSpaceGraph::setNearestNeighbors()
{
    if (nearestDatastructure_ && nearestDatastructure_->size() == 0)
        OMPL_WARN("Calling setNearestNeighbors will clear all states.");
    clear();
    nearestDatastructure_ = std::make_shared<NN<ob::State *>>();
    if (!isSetup())
    {
        setup();
    }
}

double QuotientSpaceGraph::distance(const Configuration *a, const Configuration *b) const
{
    return si_->distance(a->state, b->state);
}

void QuotientSpaceGraph::addEdge(const Vertex a, const Vertex b)
{
    ob::Cost weight = opt_->motionCost(graph_[a]->state, graph_[b]->state);
    EdgeInternalState properties(weight);
    boost::add_edge(a, b, properties, graph_);
    uniteComponents(a, b);
}

double QuotientSpaceGraph::getGraphLength() const
{
    return graphLength_;
}

bool QuotientSpaceGraph::getSolution(ob::PathPtr &solution)
{
    if (hasSolution_)
    {
        solutionPath_ = getPath(vStart_, vGoal_);
        startGoalVertexPath_ = shortestVertexPath_;
        solution = solutionPath_;
        return true;
    }
    else
    {
        ob::Goal *g = pdef_->getGoal().get();
        bestCost_ = ob::Cost(+dInf);
        bool same_component = sameComponent(vStart_, vGoal_);

        if (same_component && g->isStartGoalPairValid(graph_[vGoal_]->state, graph_[vStart_]->state))
        {
            solutionPath_ = getPath(vStart_, vGoal_);
            if (solutionPath_)
            {
                solution = solutionPath_;
                hasSolution_ = true;
                startGoalVertexPath_ = shortestVertexPath_;
                return true;
            }
        }
    }
    return hasSolution_;
}
ob::PathPtr QuotientSpaceGraph::getPath(const Vertex &start, const Vertex &goal)
{
    std::vector<Vertex> prev(boost::num_vertices(graph_));
    auto weight = boost::make_transform_value_property_map(std::mem_fn(&EdgeInternalState::getCost),
                                                           get(boost::edge_bundle, graph_));
    try
    {
        boost::astar_search(graph_, start, [this, goal](const Vertex v) { return costHeuristic(v, goal); },
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
    catch (AStarFoundGoal &)
    {
    }

    auto p(std::make_shared<PathGeometric>(si_));
    if (prev[goal] == goal)
    {
        return nullptr;
    }

    std::vector<Vertex> vpath;
    for (Vertex pos = goal; prev[pos] != pos; pos = prev[pos])
    {
        graph_[pos]->on_shortest_path = true;
        vpath.push_back(pos);
        p->append(graph_[pos]->state);
    }
    graph_[start]->on_shortest_path = true;
    vpath.push_back(start);
    p->append(graph_[start]->state);

    shortestVertexPath_.clear();
    shortestVertexPath_.insert(shortestVertexPath_.begin(), vpath.rbegin(), vpath.rend());
    p->reverse();

    return p;
}

void QuotientSpaceGraph::getPlannerData(ob::PlannerData &data) const
{
    unsigned int startComponent = 0;
    unsigned int goalComponent = 1;

    PlannerDataVertexAnnotated pstart(graph_[vStart_]->state, startComponent);
    data.addStartVertex(pstart);
    if (hasSolution_)
    {
        goalComponent = 0;
        PlannerDataVertexAnnotated pgoal(graph_[vGoal_]->state, goalComponent);
        data.addGoalVertex(pgoal);
    }

    unsigned int ctr = 0;
    foreach (const Edge e, boost::edges(graph_))
    {
        const Vertex v1 = boost::source(e, graph_);
        const Vertex v2 = boost::target(e, graph_);

        PlannerDataVertexAnnotated p1(graph_[v1]->state);
        PlannerDataVertexAnnotated p2(graph_[v2]->state);

        unsigned int vi1 = data.addVertex(p1);
        unsigned int vi2 = data.addVertex(p2);
        data.addEdge(p1, p2);

        ctr++;

        unsigned int v1Component = const_cast<QuotientSpaceGraph *>(this)->disjointSets_.find_set(v1);
        unsigned int v2Component = const_cast<QuotientSpaceGraph *>(this)->disjointSets_.find_set(v2);
        PlannerDataVertexAnnotated &v1a = *static_cast<PlannerDataVertexAnnotated *>(&data.getVertex(vi1));
        PlannerDataVertexAnnotated &v2a = *static_cast<PlannerDataVertexAnnotated *>(&data.getVertex(vi2));

        if (v1Component == startComponent || v2Component == startComponent)
        {
            v1a.setComponent(0);
            v2a.setComponent(0);
        }
        else if (v1Component == goalComponent || v2Component == goalComponent)
        {
            v1a.setComponent(1);
            v2a.setComponent(1);
        }
        else
        {
            v1a.setComponent(2);
            v2a.setComponent(2);
        }
    }
}

bool QuotientSpaceGraph::sampleQuotient(ob::State *q_random_graph)
{
    // RANDOM EDGE SAMPLING
    if (num_edges(graph_) == 0)
        return false;

    Edge e = boost::random_edge(graph_, rng_boost);
    while (!sameComponent(boost::source(e, graph_), vStart_))
    {
        e = boost::random_edge(graph_, rng_boost);
    }

    double s = rng_.uniform01();

    const Vertex v1 = boost::source(e, graph_);
    const Vertex v2 = boost::target(e, graph_);
    const ob::State *from = graph_[v1]->state;
    const ob::State *to = graph_[v2]->state;

    Q1->getStateSpace()->interpolate(from, to, s, q_random_graph);
    return true;
}
void QuotientSpaceGraph::print(std::ostream &out) const
{
    BaseT::print(out);
    out << std::endl
        << " --[QuotientSpaceGraph has " << getNumberOfVertices() << " vertices and " << getNumberOfEdges() << " edges.]"
        << std::endl;
}

void QuotientSpaceGraph::printConfiguration(const Configuration *q) const
{
    Q1->printState(q->state);
}
