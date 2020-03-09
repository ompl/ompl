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

#include <ompl/geometric/planners/quotientspace/datastructures/PlannerDataVertexAnnotated.h>
#include <ompl/geometric/planners/quotientspace/datastructures/BundleSpaceGraph.h>
#include <ompl/geometric/planners/quotientspace/datastructures/src/BundleSpaceGraphGoalVisitor.hpp>
#include <ompl/geometric/planners/quotientspace/datastructures/metrics/BundleSpaceMetric.h>
#include <ompl/geometric/planners/quotientspace/datastructures/metrics/BundleSpaceMetricEuclidean.h>
#include <ompl/geometric/planners/quotientspace/datastructures/metrics/BundleSpaceMetricShortestPath.h>

#include <ompl/geometric/planners/quotientspace/datastructures/propagators/BundleSpacePropagator.h>
#include <ompl/geometric/planners/quotientspace/datastructures/propagators/BundleSpacePropagatorGeometric.h>
#include <ompl/geometric/planners/quotientspace/datastructures/propagators/BundleSpacePropagatorDynamic.h>

#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/datastructures/PDF.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/tools/config/MagicConstants.h>
#include <ompl/util/Exception.h>

#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/property_map/transform_value_property_map.hpp>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

using Configuration = ompl::geometric::BundleSpaceGraph::Configuration;

ompl::geometric::BundleSpaceGraph::BundleSpaceGraph(const base::SpaceInformationPtr &si, BundleSpace *parent_)
  : BaseT(si, parent_)
{
    setName("BundleSpaceGraph");
    setMetric("euclidean");

    if(isDynamic())
    {
        setPropagator("dynamic");
    }else{
        setPropagator("geometric");
    }

    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = false;
    specs_.optimizingPaths = false;

    Planner::declareParam<double>(
        "range", 
        this, 
        &BundleSpaceGraph::setRange, 
        &BundleSpaceGraph::getRange, 
        "0.:1.:10000.");

    Planner::declareParam<double>(
        "goal_bias", 
        this, 
        &BundleSpaceGraph::setGoalBias, 
        &BundleSpaceGraph::getGoalBias, 
        "0.:.1:1.");

    xRandom_ = new Configuration(getBundle());

    if (!isSetup())
    {
        setup();
    }
}

ompl::geometric::BundleSpaceGraph::~BundleSpaceGraph()
{
}

void ompl::geometric::BundleSpaceGraph::setup()
{
    BaseT::setup();
    ompl::tools::SelfConfig sc(getBundle(), getName());
    sc.configurePlannerRange(maxDistance_);
    OMPL_DEBUG("Range distance graph sampling: %f", maxDistance_);

    if (!nearestDatastructure_)
    {
        nearestDatastructure_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
        nearestDatastructure_->setDistanceFunction(
            [this](const Configuration *a, const Configuration *b) 
            { 
                return distance(a, b); 
            }
        );
    }

    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
        {
            opt_ = pdef_->getOptimizationObjective();
        }
        else
        {
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(getBundle());
        }
        firstRun_ = true;
        setup_ = true;
    }
    else
    {
        setup_ = false;
    }
}

void ompl::geometric::BundleSpaceGraph::clear()
{
    BaseT::clear();

    clearVertices();
    clearQuery();
    graphLength_ = 0;
    bestCost_ = base::Cost(base::dInf);
    setup_ = false;
}

void ompl::geometric::BundleSpaceGraph::setGoalBias(double goalBias)
{
    goalBias_ = goalBias;
}

double ompl::geometric::BundleSpaceGraph::getGoalBias() const
{
    return goalBias_;
}

void ompl::geometric::BundleSpaceGraph::setRange(double maxDistance)
{
    maxDistance_ = maxDistance;
}

double ompl::geometric::BundleSpaceGraph::getRange() const
{
    return maxDistance_;
}

ompl::geometric::BundleSpaceGraph::Configuration::Configuration(const base::SpaceInformationPtr &si)
  : state(si->allocState())
{
}
ompl::geometric::BundleSpaceGraph::Configuration::Configuration(const base::SpaceInformationPtr &si,
                                                                  const base::State *state_)
  : state(si->cloneState(state_))
{
}

void ompl::geometric::BundleSpaceGraph::deleteConfiguration(Configuration *q)
{
    if (q != nullptr)
    {
        if (q->state != nullptr)
        {
            getBundle()->freeState(q->state);
        }
        delete q;
        q = nullptr;
    }
}

void ompl::geometric::BundleSpaceGraph::clearVertices()
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

void ompl::geometric::BundleSpaceGraph::clearQuery()
{
    pis_.restart();
}

double ompl::geometric::BundleSpaceGraph::getImportance() const
{
    double N = (double)getNumberOfVertices();
    return 1.0 / (N + 1);
}

void ompl::geometric::BundleSpaceGraph::init()
{
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());
    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        throw ompl::Exception("Unknown goal type");
    }

    if (const base::State *st = pis_.nextStart())
    {
        qStart_ = new Configuration(getBundle(), st);
        qStart_->isStart = true;
        vStart_ = addConfiguration(qStart_);
    }

    if (qStart_ == nullptr)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        throw ompl::Exception("Invalid initial states.");
    }

    if (const base::State *st = pis_.nextGoal())
    {
        qGoal_ = new Configuration(getBundle(), st);
        qGoal_->isGoal = true;
    }

    if (qGoal_ == nullptr)
    {
        OMPL_ERROR("%s: There are no valid goal states!", getName().c_str());
        throw ompl::Exception("Invalid goal states.");
    }
}

void ompl::geometric::BundleSpaceGraph::uniteComponents(Vertex m1, Vertex m2)
{
    disjointSets_.union_set(m1, m2);
}

bool ompl::geometric::BundleSpaceGraph::sameComponent(Vertex m1, Vertex m2)
{
    return boost::same_component(m1, m2, disjointSets_);
}

const ompl::geometric::BundleSpaceGraph::Configuration *
ompl::geometric::BundleSpaceGraph::nearest(const Configuration *q) const
{
    return nearestDatastructure_->nearest(const_cast<Configuration *>(q));
}

ompl::geometric::BundleSpaceGraph::Configuration* 
ompl::geometric::BundleSpaceGraph::addBundleConfiguration(base::State* state)
{
    Configuration *x = new Configuration(getBundle(), state);
    addConfiguration(x);
    return x;
}

void ompl::geometric::BundleSpaceGraph::addBundleEdge(const Configuration *a, const Configuration *b)
{
    addEdge(a->index, b->index);
}

ompl::geometric::BundleSpaceGraph::Vertex ompl::geometric::BundleSpaceGraph::addConfiguration(Configuration *q)
{
    Vertex m = boost::add_vertex(q, graph_);
    graph_[m]->total_connection_attempts = 1;
    graph_[m]->successful_connection_attempts = 0;
    disjointSets_.make_set(m);
    nearestDatastructure_->add(q);
    q->index = m;
    return m;
}

unsigned int ompl::geometric::BundleSpaceGraph::getNumberOfVertices() const
{
    return num_vertices(graph_);
}

unsigned int ompl::geometric::BundleSpaceGraph::getNumberOfEdges() const
{
    return num_edges(graph_);
}

const ompl::geometric::BundleSpaceGraph::Graph &ompl::geometric::BundleSpaceGraph::getGraph() const
{
    return graph_;
}

const ompl::geometric::BundleSpaceGraph::RoadmapNeighborsPtr &
ompl::geometric::BundleSpaceGraph::getRoadmapNeighborsPtr() const
{
    return nearestDatastructure_;
}

ompl::base::Cost ompl::geometric::BundleSpaceGraph::costHeuristic(Vertex u, Vertex v) const
{
    return opt_->motionCostHeuristic(graph_[u]->state, graph_[v]->state);
}

template <template <typename T> class NN>
void ompl::geometric::BundleSpaceGraph::setNearestNeighbors()
{
    if (nearestDatastructure_ && nearestDatastructure_->size() == 0)
        OMPL_WARN("Calling setNearestNeighbors will clear all states.");
    clear();
    nearestDatastructure_ = std::make_shared<NN<base::State *>>();
    if (!isSetup())
    {
        setup();
    }
}

double ompl::geometric::BundleSpaceGraph::distance(const Configuration *a, const Configuration *b) const
{
    return metric_->distanceBundle(a, b);
}

bool ompl::geometric::BundleSpaceGraph::checkMotion(const Configuration *a, const Configuration *b) const
{
    return getBundle()->checkMotion(a->state, b->state);
}

void ompl::geometric::BundleSpaceGraph::interpolate(
    const Configuration *a, 
    const Configuration *b, 
    Configuration *dest) const
{
    double d = distance(a, b);
    if (d > maxDistance_)
    {
        getBundle()->getStateSpace()->interpolate(a->state, b->state, maxDistance_ / d, dest->state);
    }
}

Configuration* ompl::geometric::BundleSpaceGraph::extendGraphTowards(
    const Configuration *from, 
    const Configuration *to)
{
    Configuration *next = new Configuration(getBundle(), to->state);
    if (!propagator_->steer(from, to, next))
    {
        return nullptr;
    }
    addConfiguration(next);
    addBundleEdge(from, next);
    return next;
}

void ompl::geometric::BundleSpaceGraph::setPropagator(const std::string& sPropagator)
{
    if(sPropagator == "geometric"){
        propagator_ = std::make_shared<BundleSpacePropagatorGeometric>(this);
    }else if(sPropagator == "dynamic"){
        propagator_ = std::make_shared<BundleSpacePropagatorDynamic>(this);
    }else{
        OMPL_ERROR("Propagator unknown: %s", sPropagator);
        throw ompl::Exception("Unknown Propagator");
    }
}

void ompl::geometric::BundleSpaceGraph::setMetric(const std::string& sMetric)
{
    if(sMetric == "euclidean"){
        OMPL_DEBUG("Euclidean Metric Selected");
        metric_ = std::make_shared<BundleSpaceMetricEuclidean>(this);
    }else if(sMetric == "shortestpath"){
        OMPL_DEBUG("ShortestPath Metric Selected");
        metric_ = std::make_shared<BundleSpaceMetricShortestPath>(this);
    }else{
        OMPL_ERROR("Metric unknown: %s", sMetric);
        throw ompl::Exception("Unknown Metric");
    }
}

void ompl::geometric::BundleSpaceGraph::addEdge(const Vertex a, const Vertex b)
{
    base::Cost weight = opt_->motionCost(graph_[a]->state, graph_[b]->state);
    EdgeInternalState properties(weight);
    boost::add_edge(a, b, properties, graph_);
    uniteComponents(a, b);
}

double ompl::geometric::BundleSpaceGraph::getGraphLength() const
{
    return graphLength_;
}

bool ompl::geometric::BundleSpaceGraph::getSolution(base::PathPtr &solution)
{
    if (hasSolution_)
    {
        solutionPath_ = getPath(vStart_, vGoal_);
        startGoalVertexPath_ = shortestVertexPath_;
        lengthStartGoalVertexPath_ = 0;
        for(uint k = 1; k < startGoalVertexPath_.size(); k++){
          Configuration* xk = graph_[startGoalVertexPath_.at(k)];
          Configuration* xkk = graph_[startGoalVertexPath_.at(k-1)];
          double d = distance(xk, xkk);
          lengthsStartGoalVertexPath_.push_back(d);
          lengthStartGoalVertexPath_ += d;
        }
        solution = solutionPath_;
        return true;
    }
    else
    {
      return false;
    }
        // base::Goal *g = pdef_->getGoal().get();
        // bestCost_ = base::Cost(+base::dInf);
        // bool same_component = sameComponent(vStart_, vGoal_);

        // if (same_component && g->isStartGoalPairValid(graph_[vGoal_]->state, graph_[vStart_]->state))
        // {
        //     solutionPath_ = getPath(vStart_, vGoal_);
        //     if (solutionPath_)
        //     {
        //         solution = solutionPath_;
        //         hasSolution_ = true;
        //         startGoalVertexPath_ = shortestVertexPath_;
        //         return true;
        //     }
        // }
    // }
    // return hasSolution_;
}

void ompl::geometric::BundleSpaceGraph::getPathDenseGraphPath(const Vertex &start, const Vertex &goal, Graph &graph, std::deque<base::State *> &path)
{
    std::vector<Vertex> prev(boost::num_vertices(graph));
    auto weight = boost::make_transform_value_property_map(std::mem_fn(&EdgeInternalState::getCost),
                                                           get(boost::edge_bundle, graph));
    try
    {
        boost::astar_search(graph, start, [this, goal](const Vertex v) { return costHeuristic(v, goal); },
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

    if (prev[goal] == goal)
    {
        OMPL_WARN("%s: No dense path was found?", getName().c_str());
    }
    else {
        for (Vertex pos = goal; prev[pos] != pos; pos = prev[pos])
            path.push_front(graph_[pos]->state);
        path.push_front(graph_[start]->state);
    }
}

ompl::base::PathPtr ompl::geometric::BundleSpaceGraph::getPath(const Vertex &start, const Vertex &goal)
{
    return getPath(start, goal, graph_);
}

ompl::base::PathPtr ompl::geometric::BundleSpaceGraph::getPath(const Vertex &start, const Vertex &goal, Graph &graph)
{
    std::vector<Vertex> prev(boost::num_vertices(graph));
    auto weight = boost::make_transform_value_property_map(std::mem_fn(&EdgeInternalState::getCost),
                                                           get(boost::edge_bundle, graph));
    try
    {
        boost::astar_search(graph, start, [this, goal](const Vertex v) { return costHeuristic(v, goal); },
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
        graph[pos]->on_shortest_path = true;
        vpath.push_back(pos);
        p->append(graph[pos]->state);
    }
    graph[start]->on_shortest_path = true;
    vpath.push_back(start);
    p->append(graph[start]->state);

    shortestVertexPath_.clear();
    shortestVertexPath_.insert(shortestVertexPath_.begin(), vpath.rbegin(), vpath.rend());
    p->reverse();

    return p;
}

void ompl::geometric::BundleSpaceGraph::sampleBundleGoalBias(base::State *xRandom, double goalBias)
{
    if (hasSolution_)
    {
        // No Goal Biasing if we already found a solution on this bundle space
        sampleBundle(xRandom);
    }
    else
    {
        double s = rng_.uniform01();
        if (s < goalBias)
        {
            getBundle()->copyState(xRandom, qGoal_->state);
        }
        else
        {
            sampleBundle(xRandom);
        }
    }
}

void ompl::geometric::BundleSpaceGraph::sampleFromDatastructure(base::State *xRandom)
{
    // RANDOM VERTEX SAMPLING
    const Vertex v = boost::random_vertex(graph_, rng_boost);
    getBundle()->getStateSpace()->copyState(xRandom, graph_[v]->state);
}

// bool ompl::geometric::BundleSpaceGraph::sampleFromDatastructure(base::State *q_random_graph)
// {
//     // RANDOM EDGE SAMPLING
//     if (num_edges(graph_) == 0)
//         return false;

//     Edge e = boost::random_edge(graph_, rng_boost);
//     while (!sameComponent(boost::source(e, graph_), vStart_))
//     {
//         e = boost::random_edge(graph_, rng_boost);
//     }

//     double s = rng_.uniform01();

//     const Vertex v1 = boost::source(e, graph_);
//     const Vertex v2 = boost::target(e, graph_);
//     const base::State *from = graph_[v1]->state;
//     const base::State *to = graph_[v2]->state;

//     getBundle()->getStateSpace()->interpolate(from, to, s, q_random_graph);
//     return true;
// }

void ompl::geometric::BundleSpaceGraph::print(std::ostream &out) const
{
    BaseT::print(out);
    out << std::endl
        << " --[BundleSpaceGraph has " << getNumberOfVertices() << " vertices and " << getNumberOfEdges()
        << " edges.]" << std::endl;
}

void ompl::geometric::BundleSpaceGraph::printConfiguration(const Configuration *q) const
{
    getBundle()->printState(q->state);
}

void ompl::geometric::BundleSpaceGraph::getPlannerDataGraph(
    base::PlannerData &data, 
    const Graph &graph, 
    const Vertex vStart, 
    const Vertex vGoal) const
{
    std::vector<int> idxPathI;
    BundleSpace *pparent = getParent();
    while (pparent != nullptr)
    {
        idxPathI.push_back(0);
        pparent = pparent->getParent();
    }
    idxPathI.push_back(0);

    base::PlannerDataVertexAnnotated pstart(graph[vStart]->state);
    pstart.setPath(idxPathI);
    data.addStartVertex(pstart);

    if (hasSolution_)
    {
        base::PlannerDataVertexAnnotated pgoal(graph[vGoal]->state);
        pgoal.setPath(idxPathI);
        data.addGoalVertex(pgoal);
    }

    foreach (const Edge e, boost::edges(graph))
    {
        const Vertex v1 = boost::source(e, graph);
        const Vertex v2 = boost::target(e, graph);

        base::PlannerDataVertexAnnotated p1(graph[v1]->state);
        base::PlannerDataVertexAnnotated p2(graph[v2]->state);
        p1.setPath(idxPathI);
        p2.setPath(idxPathI);
        data.addEdge(p1, p2);
    }

}
void ompl::geometric::BundleSpaceGraph::getPlannerData(base::PlannerData &data) const
{
    OMPL_DEBUG("Roadmap has %d/%d vertices/edges", boost::num_vertices(graph_), boost::num_edges(graph_));

    std::vector<int> idxPathI;
    BundleSpace *pparent = getParent();
    while (pparent != nullptr)
    {
        idxPathI.push_back(0);
        pparent = pparent->getParent();
    }
    idxPathI.push_back(0);

    unsigned int startComponent = 0;
    unsigned int goalComponent = 1;

    base::PlannerDataVertexAnnotated pstart(graph_[vStart_]->state, startComponent);
    pstart.setPath(idxPathI);
    data.addStartVertex(pstart);
    if (hasSolution_)
    {
        goalComponent = 0;
        base::PlannerDataVertexAnnotated pgoal(graph_[vGoal_]->state, goalComponent);
        pgoal.setPath(idxPathI);
        data.addGoalVertex(pgoal);
    }

    unsigned int ctr = 0;
    foreach (const Edge e, boost::edges(graph_))
    {
        const Vertex v1 = boost::source(e, graph_);
        const Vertex v2 = boost::target(e, graph_);

        base::PlannerDataVertexAnnotated p1(graph_[v1]->state);
        base::PlannerDataVertexAnnotated p2(graph_[v2]->state);
        p1.setPath(idxPathI);
        p2.setPath(idxPathI);

        unsigned int vi1 = data.addVertex(p1);
        unsigned int vi2 = data.addVertex(p2);
        data.addEdge(p1, p2);

        ctr++;

        unsigned int v1Component = const_cast<BundleSpaceGraph *>(this)->disjointSets_.find_set(v1);
        unsigned int v2Component = const_cast<BundleSpaceGraph *>(this)->disjointSets_.find_set(v2);
        base::PlannerDataVertexAnnotated &v1a = static_cast<base::PlannerDataVertexAnnotated &>(data.getVertex(vi1));
        base::PlannerDataVertexAnnotated &v2a = static_cast<base::PlannerDataVertexAnnotated &>(data.getVertex(vi2));

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
