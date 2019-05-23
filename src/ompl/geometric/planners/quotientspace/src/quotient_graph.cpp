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
*  * Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above
*    copyright notice, this list of conditions and the following
*    disclaimer in the documentation and/or other materials provided
*    with the distribution.
*  * Neither the name of the University of Stuttgart nor the names 
*    of its contributors may be used to endorse or promote products 
*    derived from this software without specific prior written 
*    permission.
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
#include "plannerdata_vertex_annotated.h"
#include <ompl/geometric/planners/quotientspace/quotient_graph.h>


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
typedef QuotientGraph::Configuration Configuration;

QuotientGraph::QuotientGraph(const ob::SpaceInformationPtr &si, Quotient *parent_)
    : BaseT(si, parent_)
{
    setName("QuotientGraph");
    specs_.recognizedGoal = ob::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = false;
    specs_.optimizingPaths = false;

    if (!isSetup())
    {
        setup();
    }

}

void QuotientGraph::setup(){
    if (!nearest_datastructure){
        nearest_datastructure.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration*>(this));
        nearest_datastructure->setDistanceFunction([this](const Configuration *a, const Configuration *b)
                                                         {
                                                             return Distance(a, b);
                                                         });
    }

    if (pdef_){
        BaseT::setup();
        if (pdef_->hasOptimizationObjective()){
            opt_ = pdef_->getOptimizationObjective();
        }else{
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
        }
        firstRun = true;
        setup_ = true;
    }else{
        setup_ = false;
    }
}

QuotientGraph::~QuotientGraph(){
    clear();
}
QuotientGraph::Configuration::Configuration(const base::SpaceInformationPtr &si): 
    state(si->allocState())
{}
QuotientGraph::Configuration::Configuration(const base::SpaceInformationPtr &si, const ob::State *state_): 
    state(si->cloneState(state_))
{}

void QuotientGraph::DeleteConfiguration(Configuration *q)
{
    if (q != nullptr){
        if (q->state != nullptr){
            Q1->freeState(q->state);
        }
        delete q;
        q = nullptr;
    }
}
void QuotientGraph::ClearVertices()
{
    if (nearest_datastructure)
    {
        std::vector<Configuration*> configs;
        nearest_datastructure->list(configs);
        for (auto &config : configs)
        {
            DeleteConfiguration(config);
        }
        nearest_datastructure->clear();
    }
    G.clear();
}

void QuotientGraph::clear()
{
    BaseT::clear();

    ClearVertices();
    clearQuery();
    graphLength = 0;
    bestCost_ = ob::Cost(dInf);
    setup_ = false;
    firstRun = true;
}

void QuotientGraph::clearQuery()
{
    pis_.restart();
}

double QuotientGraph::GetImportance() const{
    double N = (double)GetNumberOfVertices();
    return 1.0/(N+1);
}

void QuotientGraph::Init()
{
    auto *goal = dynamic_cast<ob::GoalSampleableRegion *>(pdef_->getGoal().get());
    if (goal == nullptr){
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        exit(0);
    }

    if(const ob::State *st = pis_.nextStart()){
        if (st != nullptr){
            q_start = new Configuration(Q1, st);
            q_start->isStart = true;
            v_start = AddConfiguration(q_start);
        }
    }
    if (q_start == nullptr){
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        exit(0);
    }

    if(const ob::State *st = pis_.nextGoal()){
        if (st != nullptr){
            q_goal = new Configuration(Q1, st);
            q_goal->isGoal = true;
        }
    }
    if (q_goal == nullptr){
        OMPL_ERROR("%s: There are no valid goal states!", getName().c_str());
        exit(0);
    }
    //unsigned long int nrStartStates = boost::num_vertices(G);
    //OMPL_INFORM("%s: ready with %lu states already in datastructure", getName().c_str(), nrStartStates);
}

void QuotientGraph::uniteComponents(Vertex m1, Vertex m2)
{
    disjointSets_.union_set(m1, m2);
}

bool QuotientGraph::sameComponent(Vertex m1, Vertex m2)
{
    return boost::same_component(m1, m2, disjointSets_);
}

const QuotientGraph::Configuration* QuotientGraph::Nearest(const Configuration* q) const
{
    return nearest_datastructure->nearest(const_cast<Configuration*>(q));
}

QuotientGraph::Vertex QuotientGraph::AddConfiguration(Configuration *q)
{
    Vertex m = boost::add_vertex(q, G);
    G[m]->total_connection_attempts = 1;
    G[m]->successful_connection_attempts = 0;
    //disjointSets_.make_set(m);
    //ConnectVertexToNeighbors(m);
    nearest_datastructure->add(q);
    q->index = m;
    return m;

}
uint QuotientGraph::GetNumberOfVertices() const{
    return num_vertices(G);
}
uint QuotientGraph::GetNumberOfEdges() const{
    return num_edges(G);
}

const og::QuotientGraph::Graph& QuotientGraph::GetGraph() const
{
    return G;
}
const og::QuotientGraph::RoadmapNeighborsPtr& QuotientGraph::GetRoadmapNeighborsPtr() const
{
    return nearest_datastructure;
}
ob::Cost QuotientGraph::costHeuristic(Vertex u, Vertex v) const
{
    return opt_->motionCostHeuristic(G[u]->state, G[v]->state);
}


template <template <typename T> class NN>
void QuotientGraph::setNearestNeighbors()
{
    if (nearest_datastructure && nearest_datastructure->size() == 0)
            OMPL_WARN("Calling setNearestNeighbors will clear all states.");
    clear();
    nearest_datastructure = std::make_shared<NN<ob::State*>>();
    if(!isSetup()){
        setup();
    }
}

double QuotientGraph::Distance(const Configuration* a, const Configuration* b) const
{
    return si_->distance(a->state, b->state);
}

void QuotientGraph::AddEdge(const Vertex a, const Vertex b)
{
    ob::Cost weight = opt_->motionCost(G[a]->state, G[b]->state);
    EdgeInternalState properties(weight);
    boost::add_edge(a, b, properties, G);
    uniteComponents(a, b);
}

double QuotientGraph::GetGraphLength() const{
    return graphLength;
}

bool QuotientGraph::GetSolution(ob::PathPtr &solution)
{
    if(hasSolution){
        solution_path = GetPath(v_start, v_goal);
        startGoalVertexPath_ = shortestVertexPath_;
        solution = solution_path;
        return true;
    }else{
        ob::Goal *g = pdef_->getGoal().get();
        bestCost_ = ob::Cost(+dInf);
        bool same_component = sameComponent(v_start, v_goal);

        if (same_component && g->isStartGoalPairValid(G[v_goal]->state, G[v_start]->state))
        {
            solution_path = GetPath(v_start, v_goal);
            if (solution_path)
            {
                solution = solution_path;
                hasSolution = true;
                startGoalVertexPath_ = shortestVertexPath_;
                return true;
            }
        }
    }
    return hasSolution;
}
ob::PathPtr QuotientGraph::GetPath(const Vertex &start, const Vertex &goal)
{
    std::vector<Vertex> prev(boost::num_vertices(G));
    auto weight = boost::make_transform_value_property_map(std::mem_fn(&EdgeInternalState::getCost), get(boost::edge_bundle, G));
    try
    {
        boost::astar_search(G, start,
                                            [this, goal](const Vertex v)
                                            {
                                                    return costHeuristic(v, goal);
                                            },
                                            boost::predecessor_map(&prev[0])
                                                .weight_map(weight)
                                                .distance_compare([this](EdgeInternalState c1, EdgeInternalState c2)
                                                                                    {
                                                                                            return opt_->isCostBetterThan(c1.getCost(), c2.getCost());
                                                                                    })
                                                .distance_combine([this](EdgeInternalState c1, EdgeInternalState c2)
                                                                                    {
                                                                                            return opt_->combineCosts(c1.getCost(), c2.getCost());
                                                                                    })
                                                .distance_inf(opt_->infiniteCost())
                                                .distance_zero(opt_->identityCost())
                                            );
    }
    catch (AStarFoundGoal &)
    {
    }

    auto p(std::make_shared<PathGeometric>(si_));
    if (prev[goal] == goal){
        return nullptr;
    }

    std::vector<Vertex> vpath;
    for (Vertex pos = goal; prev[pos] != pos; pos = prev[pos]){
        G[pos]->on_shortest_path = true;
        vpath.push_back(pos);
        p->append(G[pos]->state);
    }
    G[start]->on_shortest_path = true;
    vpath.push_back(start);
    p->append(G[start]->state);

    shortestVertexPath_.clear();
    shortestVertexPath_.insert( shortestVertexPath_.begin(), vpath.rbegin(), vpath.rend() );
    p->reverse();

    return p;
}

void QuotientGraph::getPlannerData(ob::PlannerData &data) const
{
    uint startComponent = 0;
    uint goalComponent = 1;

    PlannerDataVertexAnnotated pstart(G[v_start]->state, startComponent);
    data.addStartVertex(pstart);
    if(hasSolution){
        goalComponent = 0;
        PlannerDataVertexAnnotated pgoal(G[v_goal]->state, goalComponent);
        data.addGoalVertex(pgoal);
    }

    uint ctr = 0;
    foreach (const Edge e, boost::edges(G))
    {
        const Vertex v1 = boost::source(e, G);
        const Vertex v2 = boost::target(e, G);

        PlannerDataVertexAnnotated p1(G[v1]->state);
        PlannerDataVertexAnnotated p2(G[v2]->state);

        uint vi1 = data.addVertex(p1);
        uint vi2 = data.addVertex(p2);
        data.addEdge(p1,p2);

        ctr++;

        uint v1Component = const_cast<QuotientGraph *>(this)->disjointSets_.find_set(v1);
        uint v2Component = const_cast<QuotientGraph *>(this)->disjointSets_.find_set(v2);
        PlannerDataVertexAnnotated &v1a = *static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(vi1));
        PlannerDataVertexAnnotated &v2a = *static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(vi2));

        if(v1Component==startComponent || v2Component==startComponent){
            v1a.SetComponent(0);
            v2a.SetComponent(0);
        }else if(v1Component==goalComponent || v2Component==goalComponent){
            v1a.SetComponent(1);
            v2a.SetComponent(1);
        }else{
            v1a.SetComponent(2);
            v2a.SetComponent(2);
        }
    }
}

bool QuotientGraph::SampleQuotient(ob::State *q_random_graph)
{
    //RANDOM EDGE SAMPLING
    if(num_edges(G) == 0) return false;

    Edge e = boost::random_edge(G, rng_boost);
    while(!sameComponent(boost::source(e, G), v_start))
    {
        e = boost::random_edge(G, rng_boost);
    }

    double s = rng_.uniform01();

    const Vertex v1 = boost::source(e, G);
    const Vertex v2 = boost::target(e, G);
    const ob::State *from = G[v1]->state;
    const ob::State *to = G[v2]->state;

    Q1->getStateSpace()->interpolate(from, to, s, q_random_graph);
    return true;
}
void QuotientGraph::Print(std::ostream& out) const
{
    BaseT::Print(out);
    out << std::endl << " --[QuotientGraph has " << GetNumberOfVertices() << " vertices and " << GetNumberOfEdges() << " edges.]" << std::endl;
}

void QuotientGraph::PrintConfiguration(const Configuration* q) const
{
    Q1->printState(q->state);
}
