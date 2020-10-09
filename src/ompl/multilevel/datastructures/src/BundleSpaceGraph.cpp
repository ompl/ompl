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

#include <ompl/multilevel/datastructures/PlannerDataVertexAnnotated.h>
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/multilevel/datastructures/src/BundleSpaceGraphGoalVisitor.hpp>

#include <ompl/multilevel/datastructures/graphsampler/RandomVertex.h>
#include <ompl/multilevel/datastructures/graphsampler/RandomDegreeVertex.h>
#include <ompl/multilevel/datastructures/graphsampler/RandomEdge.h>
#include <ompl/multilevel/datastructures/importance/Greedy.h>
#include <ompl/multilevel/datastructures/importance/Exponential.h>
#include <ompl/multilevel/datastructures/importance/Uniform.h>
#include <ompl/multilevel/datastructures/metrics/Geodesic.h>
#include <ompl/multilevel/datastructures/metrics/ShortestPath.h>
#include <ompl/multilevel/datastructures/propagators/Geometric.h>
#include <ompl/multilevel/datastructures/propagators/Dynamic.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>

#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/datastructures/PDF.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/tools/config/MagicConstants.h>
#include <ompl/util/Exception.h>
#include <ompl/control/SpaceInformation.h>

#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>

#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/property_map/transform_value_property_map.hpp>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

using Configuration = ompl::multilevel::BundleSpaceGraph::Configuration;

using namespace ompl::multilevel;

BundleSpaceGraph::BundleSpaceGraph(const base::SpaceInformationPtr &si, BundleSpace *parent_) : BaseT(si, parent_)
{
    setName("BundleSpaceGraph");

    // Functional primitives
    setMetric("geodesic");
    setGraphSampler("randomvertex");
    setImportance("uniform");

    if (isDynamic())
    {
        setPropagator("dynamic");
    }
    else
    {
        setPropagator("geometric");
    }

    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = false;
    specs_.optimizingPaths = false;

    Planner::declareParam<double>("range", 
        this, &BundleSpaceGraph::setRange, 
        &BundleSpaceGraph::getRange, "0.:1.:10000.");

    Planner::declareParam<double>("goal_bias", 
        this, 
        &BundleSpaceGraph::setGoalBias, 
        &BundleSpaceGraph::getGoalBias, 
        "0.:.1:1.");

    xRandom_ = new Configuration(getBundle());

    if (!isSetup())
    {
        this->setup();
    }

    ompl::base::OptimizationObjectivePtr lengthObj =
        std::make_shared<ompl::base::PathLengthOptimizationObjective>(getBundle());
    ompl::base::OptimizationObjectivePtr clearObj =
        std::make_shared<ompl::base::MaximizeMinClearanceObjective>(getBundle());

    pathRefinementObj_ = std::make_shared<ompl::base::MultiOptimizationObjective>(getBundle());

    std::static_pointer_cast<base::MultiOptimizationObjective>(pathRefinementObj_)
      ->addObjective(lengthObj, 1.0);
    // std::static_pointer_cast<base::MultiOptimizationObjective>(pathRefinementObj_)
    //   ->addObjective(clearObj, 1.0);

    if (getFiberDimension() > 0)
    {
        xFiberTmp1_ = getFiber()->allocState();
        xFiberTmp2_ = getFiber()->allocState();
    }
}

BundleSpaceGraph::~BundleSpaceGraph()
{
    deleteConfiguration(xRandom_);
    if (getFiberDimension() > 0)
    {
        getFiber()->freeState(xFiberTmp1_);
        getFiber()->freeState(xFiberTmp2_);
    }
}

void BundleSpaceGraph::setup()
{
    BaseT::setup();

    ompl::tools::SelfConfig sc(getBundle(), getName());
    sc.configurePlannerRange(maxDistance_);

    OMPL_DEBUG("Range distance graph sampling: %f (max extent %f)", maxDistance_, getBundle()->getMaximumExtent());

    if (!nearestDatastructure_)
    {
        if (!isDynamic())
        {
            nearestDatastructure_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
        }
        else
        {
            // for dynamical systems, we use a quasimetric
            //(non-symmetric metric), so we cannot use NN structures like GNAT
            nearestDatastructure_.reset(new NearestNeighborsSqrtApprox<Configuration *>());
        }
        nearestDatastructure_->setDistanceFunction(
            [this](const Configuration *a, const Configuration *b) { return distance(a, b); });
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
            opt_->setCostThreshold(base::Cost(std::numeric_limits<double>::infinity()));
            pdef_->setOptimizationObjective(opt_);
        }
        firstRun_ = true;
        setup_ = true;
    }
    else
    {
        setup_ = false;
    }
    if(hasBaseSpace())
    {
        pathRestriction_ = std::make_shared<PathRestriction>(this);
    }
}

void BundleSpaceGraph::clear()
{
    BaseT::clear();

    clearVertices();
    pis_.restart();

    graphLength_ = 0;
    bestCost_ = base::Cost(base::dInf);
    setup_ = false;
    vStart_ = 0;
    vGoal_ = 0;
    lengthStartGoalVertexPath_ = base::dInf;
    shortestVertexPath_.clear();

    qStart_ = nullptr;
    qGoal_ = nullptr;

    if (!isDynamic())
    {
        if (solutionPath_ != nullptr)
        {
            std::static_pointer_cast<geometric::PathGeometric>(solutionPath_)->clear();
        }
    }

    numVerticesWhenComputingSolutionPath_ = 0;

    importanceCalculator_->clear();
    graphSampler_->clear();
    if(pathRestriction_ != nullptr)
    {
        pathRestriction_->clear();
    }
}

void BundleSpaceGraph::clearVertices()
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

void BundleSpaceGraph::setGoalBias(double goalBias)
{
    goalBias_ = goalBias;
}

double BundleSpaceGraph::getGoalBias() const
{
    return goalBias_;
}

void BundleSpaceGraph::setRange(double maxDistance)
{
    maxDistance_ = maxDistance;
}

double BundleSpaceGraph::getRange() const
{
    return maxDistance_;
}

BundleSpaceGraph::Configuration::Configuration(const base::SpaceInformationPtr &si) 
  : state(si->allocState())
{
    //Too costly to do dynamic_pointer_cast in every call. Best to create
    //different Configuration classes (dynamic, geometric, etc). Or do inside
    //addconfiguration function.
    //
    //
    // const ompl::control::SpaceInformationPtr siC = std::dynamic_pointer_cast<ompl::control::SpaceInformation>(si);
    // if (siC != nullptr)
    // {
    //     control = siC->allocControl();
    // }
}
BundleSpaceGraph::Configuration::Configuration(
    const base::SpaceInformationPtr &si, const base::State *state_)
  : state(si->cloneState(state_))
{
    // const ompl::control::SpaceInformationPtr siC = std::dynamic_pointer_cast<ompl::control::SpaceInformation>(si);

    // if (siC != nullptr)
    // {
    //     control = siC->allocControl();
    // }
}

void BundleSpaceGraph::deleteConfiguration(Configuration *q)
{
    if (q != nullptr)
    {
        if (q->state != nullptr)
        {
            getBundle()->freeState(q->state);
        }
        for (unsigned int k = 0; k < q->reachableSet.size(); k++)
        {
            Configuration *qk = q->reachableSet.at(k);
            if (qk->state != nullptr)
            {
                getBundle()->freeState(qk->state);
            }
        }
        if (isDynamic())
        {
            const ompl::control::SpaceInformationPtr siC =
                std::static_pointer_cast<ompl::control::SpaceInformation>(getBundle());
            siC->freeControl(q->control);
        }
        q->reachableSet.clear();

        delete q;
        q = nullptr;
    }
}

double BundleSpaceGraph::getImportance() const
{
    return importanceCalculator_->eval();
}

void BundleSpaceGraph::init()
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
        vStart_ = addConfiguration(qStart_);
        qStart_->isStart = true;
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

void BundleSpaceGraph::uniteComponents(Vertex m1, Vertex m2)
{
    disjointSets_.union_set(m1, m2);
}

bool BundleSpaceGraph::sameComponent(Vertex m1, Vertex m2)
{
    return boost::same_component(m1, m2, disjointSets_);
}

const BundleSpaceGraph::Configuration *BundleSpaceGraph::nearest(const Configuration *q) const
{
    return nearestDatastructure_->nearest(const_cast<Configuration *>(q));
}

BundleSpaceGraph::Configuration *BundleSpaceGraph::addBundleConfiguration(base::State *state)
{
    Configuration *x = new Configuration(getBundle(), state);
    addConfiguration(x);
    return x;
}

void BundleSpaceGraph::addBundleEdge(const Configuration *a, const Configuration *b)
{
    //@DEBUG
    // bool feasible = getBundle()->checkMotion(a->state, b->state);
    // if(!feasible)
    // {
    //   std::cout << "Trying to add infeasible bundle edge" << std::endl;
    //   getBundle()->printState(a->state);
    //   if(!getBundle()->isValid(a->state))
    //   {
    //     std::cout << "Infeasible state" << std::endl;
    //   }
    //   getBundle()->printState(b->state);
    //   if(!getBundle()->isValid(b->state))
    //   {
    //     std::cout << "Infeasible state" << std::endl;
    //   }
    //   throw "";
    // }
    //@DEBUG
    addEdge(a->index, b->index);
}

BundleSpaceGraph::Vertex BundleSpaceGraph::addConfiguration(Configuration *q)
{
    // @DEBUG
    // std::vector<double> reals;
    // getBundle()->getStateSpace()->copyToReals(reals, q->state);
    // if(reals != reals)
    // {
    //     getBundle()->printState(q->state);
    //     throw Exception("INVALID");
    // }
    // @DEBUG

    Vertex m = boost::add_vertex(q, graph_);
    graph_[m]->total_connection_attempts = 1;
    graph_[m]->successful_connection_attempts = 0;
    disjointSets_.make_set(m);

    nearestDatastructure_->add(q);
    q->index = m;

    return m;
}

unsigned int BundleSpaceGraph::getNumberOfVertices() const
{
    return num_vertices(graph_);
}

unsigned int BundleSpaceGraph::getNumberOfEdges() const
{
    return num_edges(graph_);
}

const BundleSpaceGraph::Graph &BundleSpaceGraph::getGraph() const
{
    return graph_;
}

const BundleSpaceGraph::RoadmapNeighborsPtr &BundleSpaceGraph::getRoadmapNeighborsPtr() const
{
    return nearestDatastructure_;
}

ompl::base::Cost BundleSpaceGraph::costHeuristic(Vertex u, Vertex v) const
{
    return opt_->motionCostHeuristic(graph_[u]->state, graph_[v]->state);
}

template <template <typename T> class NN>
void BundleSpaceGraph::setNearestNeighbors()
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

double BundleSpaceGraph::distance(const Configuration *a, const Configuration *b) const
{
    return metric_->distanceBundle(a, b);
}

bool BundleSpaceGraph::checkMotion(const Configuration *a, const Configuration *b) const
{
    return getBundle()->checkMotion(a->state, b->state);
}

void BundleSpaceGraph::interpolate(const Configuration *a, const Configuration *b, Configuration *dest) const
{
    metric_->interpolateBundle(a, b, dest);
}

Configuration *BundleSpaceGraph::steerTowards(const Configuration *from, const Configuration *to)
{
    Configuration *next = new Configuration(getBundle(), to->state);

    if (!propagator_->steer(from, to, next))
    {
        deleteConfiguration(next);
        return nullptr;
    }
    return next;
}

Configuration *BundleSpaceGraph::steerTowards_Range(const Configuration *from, Configuration *to)
{
    // Configuration *next = new Configuration(getBundle(), to->state);

    double d = distance(from, to);
    if (d > maxDistance_)
    {
        metric_->interpolateBundle(from, to, maxDistance_ / d, to);
    }

    if (!propagator_->steer(from, to, to))
    {
        return nullptr;
    }
    Configuration *next = new Configuration(getBundle(), to->state);
    return next;
}

Configuration *BundleSpaceGraph::extendGraphTowards_Range(const Configuration *from, Configuration *to)
{
    // Configuration *next = new Configuration(getBundle(), to->state);

    if (!isDynamic())
    {
        double d = distance(from, to);
        if (d > maxDistance_)
        {
            metric_->interpolateBundle(from, to, maxDistance_ / d, to);
        }
    }

    if (!propagator_->steer(from, to, to))
    {
        return nullptr;
    }

    Configuration *next = new Configuration(getBundle(), to->state);
    // if (isDynamic())
    // {
    //     const ompl::control::SpaceInformationPtr siC =
    //         std::dynamic_pointer_cast<ompl::control::SpaceInformation>(getBundle());
    //     const control::Control *lastControl =
    //         std::static_pointer_cast<BundleSpacePropagatorDynamic>(propagator_)->getLastControl();
    //     siC->copyControl(next->control, lastControl);
    // }
    addConfiguration(next);
    addBundleEdge(from, next);
    return next;
}

bool BundleSpaceGraph::connect(const Configuration *from, const Configuration *to)
{
    if (!propagator_->steer(from, to, xRandom_))
    {
        return false;
    }

    addBundleEdge(from, to);
    return true;
}

void BundleSpaceGraph::setPropagator(const std::string &sPropagator)
{
    if (sPropagator == "geometric")
    {
        OMPL_DEBUG("Geometric Propagator Selected");
        propagator_ = std::make_shared<BundleSpacePropagatorGeometric>(this);
    }
    else if (sPropagator == "dynamic")
    {
        OMPL_DEBUG("Dynamic Propagator Selected");
        propagator_ = std::make_shared<BundleSpacePropagatorDynamic>(this);
    }
    else
    {
        OMPL_ERROR("Propagator unknown: %s", sPropagator.c_str());
        throw ompl::Exception("Unknown Propagator");
    }
}

void BundleSpaceGraph::setMetric(const std::string &sMetric)
{
    if (sMetric == "geodesic")
    {
        OMPL_DEBUG("Geodesic Metric Selected");
        metric_ = std::make_shared<BundleSpaceMetricGeodesic>(this);
    }
    else if (sMetric == "shortestpath")
    {
        OMPL_DEBUG("ShortestPath Metric Selected");
        metric_ = std::make_shared<BundleSpaceMetricShortestPath>(this);
    }
    else
    {
        OMPL_ERROR("Metric unknown: %s", sMetric.c_str());
        throw ompl::Exception("Unknown Metric");
    }
}

void BundleSpaceGraph::setImportance(const std::string &sImportance)
{
    if (sImportance == "uniform")
    {
        OMPL_DEBUG("Uniform Importance Selected");
        importanceCalculator_ = std::make_shared<BundleSpaceImportanceUniform>(this);
    }
    else if (sImportance == "greedy")
    {
        OMPL_DEBUG("Greedy Importance Selected");
        importanceCalculator_ = std::make_shared<BundleSpaceImportanceGreedy>(this);
    }
    else if (sImportance == "exponential")
    {
        OMPL_DEBUG("Greedy Importance Selected");
        importanceCalculator_ = std::make_shared<BundleSpaceImportanceExponential>(this);
    }
    else
    {
        OMPL_ERROR("Importance calculator unknown: %s", sImportance.c_str());
        throw ompl::Exception("Unknown Importance");
    }
}

void BundleSpaceGraph::setGraphSampler(const std::string &sGraphSampler)
{
    if (sGraphSampler == "randomvertex")
    {
        OMPL_DEBUG("Random Vertex Sampler Selected");
        graphSampler_ = std::make_shared<BundleSpaceGraphSamplerRandomVertex>(this);
    }
    else if (sGraphSampler == "randomedge")
    {
        OMPL_DEBUG("Random Edge Sampler Selected");
        graphSampler_ = std::make_shared<BundleSpaceGraphSamplerRandomEdge>(this);
    }
    else if (sGraphSampler == "randomdegreevertex")
    {
        OMPL_DEBUG("Random Degree Vertex Sampler Selected");
        graphSampler_ = std::make_shared<BundleSpaceGraphSamplerRandomDegreeVertex>(this);
    }
    else
    {
        OMPL_ERROR("Sampler unknown: %s", sGraphSampler.c_str());
        throw ompl::Exception("Unknown Graph Sampler");
    }
}

BundleSpaceGraphSamplerPtr BundleSpaceGraph::getGraphSampler()
{
    return graphSampler_;
}

void BundleSpaceGraph::addEdge(const Vertex a, const Vertex b)
{
    base::Cost weight = opt_->motionCost(graph_[a]->state, graph_[b]->state);
    EdgeInternalState properties(weight);
    boost::add_edge(a, b, properties, graph_);
    uniteComponents(a, b);
}

double BundleSpaceGraph::getGraphLength() const
{
    return graphLength_;
}

BundleSpaceGraph::Vertex BundleSpaceGraph::getStartIndex() const
{
  return vStart_;
}
BundleSpaceGraph::Vertex BundleSpaceGraph::getGoalIndex() const
{
  return vGoal_;
}
void BundleSpaceGraph::setStartIndex(Vertex idx)
{
  vStart_ = idx;
}
void BundleSpaceGraph::setGoalIndex(Vertex idx)
{
  vGoal_ = idx;
}

bool BundleSpaceGraph::getSolution(base::PathPtr &solution)
{
    if (hasSolution_)
    {
        if ((solutionPath_ != nullptr) 
            && (getNumberOfVertices() == numVerticesWhenComputingSolutionPath_))
        {
        }
        else
        {
            solutionPath_ = getPath(vStart_, vGoal_);
            numVerticesWhenComputingSolutionPath_ = getNumberOfVertices();

            if (!isDynamic() && solutionPath_ != solution && hasTotalSpace())
            {
                // bool optimize = true;
                // int type = getBundle()->getStateSpace()->getType();
                // // if(type == base::STATE_SPACE_DUBINS || type == base::STATE_SPACE_DUBINS_AIRPLANE)
                // if(type == base::STATE_SPACE_DUBINS
                //     || type == base::STATE_SPACE_DUBINS_AIRPLANE)
                // {
                //   optimize = false;
                // }
                // if(!optimize && getBundle()->getStateSpace()->isCompound())
                // {
                //     std::vector<base::StateSpacePtr> Bundle_decomposed;
                //     base::CompoundStateSpace *Bundle_compound =
                //       getBundle()->getStateSpace()->as<base::CompoundStateSpace>();
                //     Bundle_decomposed = Bundle_compound->getSubspaces();
                //     for(unsigned int k = 0; k < Bundle_decomposed.size(); k++)
                //     {
                //       int tk = Bundle_decomposed.at(k)->getType();
                //       if(tk == base::STATE_SPACE_DUBINS || tk == base::STATE_SPACE_DUBINS_AIRPLANE)
                //       {
                //         optimize = false;
                //         break;
                //       }
                //     }
                // }

                // if(optimize)
                // {
                geometric::PathSimplifier shortcutter(getBundle(), base::GoalPtr(), pathRefinementObj_);
                
                // @NOTE: optimization seems to improve feasibility of sections
                // in low-dim problems (up to 20 dof roughly), but will take too
                // much time for high-dim problems. Reducing vertices seems to
                // be the only optimization not significantly slowing everything
                // down.

                bool valid = false;
                // std::cout << "Optimize" << std::endl;
                for(uint k = 0; k < 3; k++)
                {
                    geometric::PathGeometric &gpath = 
                      static_cast<geometric::PathGeometric &>(*solutionPath_);

                    valid = shortcutter.reduceVertices(gpath, 0, 0, 0.1);

                    // valid = shortcutter.simplifyMax(gpath);

                    if (!valid)
                    {
                        // reset solutionPath
                        solutionPath_ = getPath(vStart_, vGoal_);
                    }else{
                        break;
                    }
                }
                // std::cout << "Done" << std::endl;

                // geometric::PathGeometric &gpath2 = 
                //   static_cast<geometric::PathGeometric &>(*solutionPath_);

                // shortcutter.smoothBSpline(gpath2);

                // gpath2.interpolate();
                // }
            }
        }
        solution = solutionPath_;
        return true;
    }
    else
    {
        return false;
    }
}

ompl::base::PathPtr BundleSpaceGraph::getPath(const Vertex &start, const Vertex &goal)
{
    return getPath(start, goal, graph_);
}

ompl::base::PathPtr BundleSpaceGraph::getPath(const Vertex &start, const Vertex &goal, Graph &graph)
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
                                .distance_zero(opt_->identityCost())
                                .visitor(BundleSpaceGraphGoalVisitor<Vertex>(goal)));
    }
    catch (BundleSpaceGraphFoundGoal &)
    {
    }

    auto p(std::make_shared<geometric::PathGeometric>(getBundle()));
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

const PathRestrictionPtr BundleSpaceGraph::getPathRestriction()
{
    if (!hasBaseSpace())
    {
        OMPL_WARN("Tried getting path restriction without base space");
        return nullptr;
    }

    base::PathPtr basePath = static_cast<BundleSpaceGraph *>(getBaseBundleSpace())->solutionPath_;
    pathRestriction_->setBasePath(basePath);

    return pathRestriction_;
}

void BundleSpaceGraph::sampleBundleGoalBias(base::State *xRandom)
{
    if (hasSolution_)
    {
        // No Goal Biasing if we already found a solution on this bundle space
        sampleBundle(xRandom);
    }
    else
    {
        double s = rng_.uniform01();
        if (s < goalBias_)
        {
            getBundle()->copyState(xRandom, qGoal_->state);
        }
        else
        {
            sampleBundle(xRandom);
        }
    }
}

void BundleSpaceGraph::sampleFromDatastructure(base::State *xRandom)
{
    graphSampler_->sample(xRandom);
}

void BundleSpaceGraph::print(std::ostream &out) const
{
    BaseT::print(out);
    out << std::endl
        << " --[BundleSpaceGraph has " << getNumberOfVertices() << " vertices and " << getNumberOfEdges() << " edges.]"
        << std::endl;
}

void BundleSpaceGraph::printConfiguration(const Configuration *q) const
{
    getBundle()->printState(q->state);
}

void BundleSpaceGraph::getPlannerDataGraph(base::PlannerData &data, const Graph &graph, const Vertex vStart,
                                           const Vertex vGoal) const
{
    if (boost::num_vertices(graph) <= 0)
        return;

    multilevel::PlannerDataVertexAnnotated pstart(graph[vStart]->state);
    pstart.setLevel(getLevel());
    data.addStartVertex(pstart);

    if (hasSolution_)
    {
        multilevel::PlannerDataVertexAnnotated pgoal(graph[vGoal]->state);
        pgoal.setLevel(getLevel());
        data.addGoalVertex(pgoal);
        if (solutionPath_ != nullptr)
        {
            geometric::PathGeometric &gpath = static_cast<geometric::PathGeometric &>(*solutionPath_);

            std::vector<base::State *> gstates = gpath.getStates();

            multilevel::PlannerDataVertexAnnotated *pLast = &pstart;
            for (unsigned int k = 1; k < gstates.size() - 1; k++)
            {
                multilevel::PlannerDataVertexAnnotated p(gstates.at(k));
                p.setLevel(getLevel());
                data.addVertex(p);
                data.addEdge(*pLast, p);
                pLast = &p;
            }
            data.addEdge(*pLast, pgoal);
        }
    }

    foreach (const Edge e, boost::edges(graph))
    {
        const Vertex v1 = boost::source(e, graph);
        const Vertex v2 = boost::target(e, graph);

        multilevel::PlannerDataVertexAnnotated p1(graph[v1]->state);
        multilevel::PlannerDataVertexAnnotated p2(graph[v2]->state);
        p1.setLevel(getLevel());
        p2.setLevel(getLevel());
        data.addEdge(p1, p2);
    }
    foreach (const Vertex v, boost::vertices(graph))
    {
        multilevel::PlannerDataVertexAnnotated p(graph[v]->state);
        p.setLevel(getLevel());
        data.addVertex(p);
    }
}

void BundleSpaceGraph::getPlannerData(base::PlannerData &data) const
{
    OMPL_DEBUG("Graph (level %d) has %d/%d vertices/edges", getLevel(), boost::num_vertices(graph_),
               boost::num_edges(graph_));

    if (bestCost_.value() < ompl::base::dInf)
    {
        OMPL_DEBUG("Best Cost: %.2f", bestCost_.value());
    }
    getPlannerDataGraph(data, graph_, vStart_, vGoal_);
}
