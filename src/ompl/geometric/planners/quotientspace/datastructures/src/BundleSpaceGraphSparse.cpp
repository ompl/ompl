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
#include "GoalVisitor.hpp"
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp> //same_component
#include <boost/math/constants/constants.hpp>
using namespace og;
#define foreach BOOST_FOREACH

BundleSpaceGraphSparse::BundleSpaceGraphSparse(const ob::SpaceInformationPtr &si, BundleSpace *parent):
  BaseT(si, parent)
{
  setName("BundleSpaceGraphSparse");
  Planner::declareParam<double>("sparse_delta_fraction", this, &BundleSpaceGraphSparse::setSparseDeltaFraction,
                                &BundleSpaceGraphSparse::getSparseDeltaFraction, "0.0:0.01:1.0");

  if (!isSetup())
  {
    setup();
  }
  pathVisibilityChecker_ = new PathVisibilityChecker(Bundle);
}

BundleSpaceGraphSparse::~BundleSpaceGraphSparse()
{
}

void BundleSpaceGraphSparse::deleteConfiguration(Configuration *q)
{
  BaseT::deleteConfiguration(q);
}

void BundleSpaceGraphSparse::setup()
{

  BaseT::setup();
  if (!nearestSparse_){
    nearestSparse_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration*>(this));
    nearestSparse_->setDistanceFunction([this](const Configuration *a, const Configuration *b)
                             {
                               return distance(a, b);
                             });
  }

  double maxExt = Bundle->getMaximumExtent();
  sparseDelta_ = sparseDeltaFraction_ * maxExt;
  pathBias_ = pathBiasFraction_ * maxExt;
  double d = (double) Bundle->getStateDimension();
  double e = boost::math::constants::e<double>();
  kPRMStarConstant_ = e + (e/d);
}

void BundleSpaceGraphSparse::clear()
{
  BaseT::clear();

  if (nearestSparse_)
  {
    std::vector<Configuration*> configs;
    nearestSparse_->list(configs);
    if(configs.size()>1){
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
    std::vector<Configuration*> configs;
    nearestSparse_->list(configs);
    for (auto &config : configs)
    {
      if(config->state != qStart_->state)
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
  // v_start_sparse = -1;
  // v_goal_sparse = -1;
  Nold_v = 0;
  Nold_e = 0;

  const Vertex vl = add_vertex(qStart_, graphSparse_);
  nearestSparse_->add(qStart_);
  disjointSetsSparse_.make_set(vl);
  graphSparse_[vl]->index = vl;

  // hasSolution_ = false;

}

const ompl::geometric::BundleSpaceGraph::Configuration *
ompl::geometric::BundleSpaceGraphSparse::nearest(const Configuration *q) const
{
    if(!isDynamic()) return BaseT::nearest(q);
    else{
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

void BundleSpaceGraphSparse::Init()
{
  if(const ob::State *sInitial = pis_.nextStart()){
    if (sInitial != nullptr){
      qStart_ = new Configuration(Bundle, sInitial);
      qStart_->isStart = true;
      vStart_ = addConfiguration(qStart_);
      assert(boost::num_vertices(graphSparse_)==1);
      v_start_sparse = graphSparse_[0]->index;
      graphSparse_[v_start_sparse]->isStart = true;
    }
  }else{
      OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
      const base::State *sInvalidInitial = pdef_->getStartState(0);
      debugInvalidState(sInvalidInitial);
      throw ompl::Exception("No valid initial states.");
  }

  const ob::State *sGoal{nullptr};
  if((sGoal = pis_.nextGoal())){
    if (sGoal != nullptr){
      qGoal_ = new Configuration(Bundle, sGoal);
      qGoal_->isGoal = true;

      if(!isDynamic()){
          vGoal_ = addConfiguration(qGoal_);
          if(boost::num_vertices(graphSparse_)<2)
          {
            //Make sure q_goal is added, even if visible from q_start
              addConfigurationSparse(qGoal_);
          }
          assert(boost::num_vertices(graphSparse_)==2);
          v_goal_sparse = graphSparse_[1]->index;
          graphSparse_[v_goal_sparse]->isGoal = true;
      }
    }
  }else{
      OMPL_ERROR("%s: There are no valid goal states!", getName().c_str());
      ob::State *sInvalidGoal = Bundle->allocState();
      const ob::GoalSampleableRegion *goal = pdef_->getGoal()->as<ob::GoalSampleableRegion>();
      goal->sampleGoal(sInvalidGoal);
      debugInvalidState(sInvalidGoal);
      throw ompl::Exception("No valid goal states.");
  }

}

void BundleSpaceGraphSparse::debugInvalidState(const ob::State *s)
{
    const ob::StateSpacePtr space = Bundle->getStateSpace();
    bool bounds = space->satisfiesBounds(s);
    if(!bounds){
        std::vector<ob::StateSpacePtr> Bundle_decomposed;
        if (!space->isCompound())
        {
            Bundle_decomposed.push_back(space);
        }else{
            ob::CompoundStateSpace *Bundle_compound = space->as<ob::CompoundStateSpace>();
            Bundle_decomposed = Bundle_compound->getSubspaces();
        }

        for(uint k = 0; k < Bundle_decomposed.size(); k++){
            ob::StateSpacePtr spacek = Bundle_decomposed.at(k);
            int type = spacek->getType();
            switch (type) {
              case ob::STATE_SPACE_REAL_VECTOR:
              {
                  auto *RN = spacek->as<ob::RealVectorStateSpace>();
                  const ob::RealVectorStateSpace::StateType *sk = 
                    s->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(k);
                  std::vector<double> bl =  RN->getBounds().low;
                  std::vector<double> bh =  RN->getBounds().high;
                  for(uint k = 0; k < bl.size(); k++){
                    double qk = sk->values[k];
                    double qkl = bl.at(k);
                    double qkh = bh.at(k);
                    if(qk < qkl || qk > qkh){
                        std::cout << "OUTOFBOUNDS [" << k << "] " << bl.at(k) << " <= " << qk << " <= " << bh.at(k) << std::endl;
                    }
                  }
                  break;
              }
            }
        }
    }
}

BundleSpaceGraphSparse::Vertex BundleSpaceGraphSparse::addConfiguration(Configuration *q)
{
    Vertex v = BaseT::addConfiguration(q);

    if(isDynamic()){
      //DYNAMIC
        addConfigurationSparse(q);
    }else{
      //GEOMETRIC
        //Add Edges to Delta-Neighbors (PRM* style)
        std::vector<Configuration*> neighbors;
        unsigned N = boost::num_vertices(graph_);
        unsigned K = static_cast<unsigned int>(ceil(kPRMStarConstant_ * log((double)N)));
        nearestDatastructure_->nearestK(q, K, neighbors);

        for(uint k = 0; k < neighbors.size(); k++){

          Configuration *qn = neighbors.at(k);
          q->total_connection_attempts++;
          qn->total_connection_attempts++;

          if(Bundle->checkMotion(qn->state, q->state))
          {
            q->successful_connection_attempts++;
            qn->successful_connection_attempts++;
            addEdge(qn->index, q->index);

          }
        }

        //Sparse Graph addition
        findGraphNeighbors(q, graphNeighborhood, visibleNeighborhood);

        //Possible reasons for adding a node to sparse roadmap
        //(1) VISIBLITY: Add Guard (when no one is visible) [Simeon 00]
        //(2) CONNECTIVITY: Add Connectivity (when two disconnected components are visible) [Simeon 00]
        //(3) INTERFACE: Add Interface (when a connected components get another useful cycle
        //[Jaillet 09, Dobson 14, Nieuwenhausen 04]
        //(4) OPTIMALITY: Optimality based [Dobson 14]

        if(visibleNeighborhood.empty())
        {
          addConfigurationSparse(q);
        }else{
          if(!checkAddConnectivity(q, visibleNeighborhood)){
            if (!checkAddInterface(q, graphNeighborhood, visibleNeighborhood)){
              // if (!visibleNeighborhood.empty())
              // {
              //     base::State *workState = si_->allocState();
              //     std::map<Vertex, base::State *> closeRepresentatives;
              //     findCloseRepresentatives(workState, q->state, visibleNeighborhood[0], closeRepresentatives);
              //     for (auto &closeRepresentative : closeRepresentatives)
              //     {
              //         updatePairPoints(visibleNeighborhood[0], q->state, closeRepresentative.first,
              //                          closeRepresentative.second);
              //         updatePairPoints(closeRepresentative.first, closeRepresentative.second,
              //                          visibleNeighborhood[0], q->state);
              //     }
              //     checkAddPath(visibleNeighborhood[0]);
              //     for (auto &closeRepresentative : closeRepresentatives)
              //     {
              //         checkAddPath(closeRepresentative.first);
              //         si_->freeState(closeRepresentative.second);
              //     }
              // }

            }
          }
        }
    }

    return v;
}
// void ompl::geometric::BundleSpaceGraphSparse::computeVPP(Vertex v, Vertex vp, std::vector<Vertex> &VPPs)
// {
//     VPPs.clear();
//     foreach (Vertex cvpp, boost::adjacent_vertices(v, g_))
//         if (cvpp != vp)
//             if (!boost::edge(cvpp, vp, g_).second)
//                 VPPs.push_back(cvpp);
// }
// void ompl::geometric::BundleSpaceGraphSparse::updatePairPoints(Vertex rep, const base::State *q, Vertex r, const base::State *s)
// {
//     // First of all, we need to compute all candidate r'
//     std::vector<Vertex> VPPs;
//     computeVPP(rep, r, VPPs);

//     // Then, for each pair Pv(r,r')
//     foreach (Vertex rp, VPPs)
//         // Try updating the pair info
//         distanceCheck(rep, q, r, s, rp);
// }
// void ompl::geometric::BundleSpaceGraphSparse::findCloseRepresentatives(base::State *workArea, const base::State *qNew,
//                                                          const Vertex qRep,
//                                                          std::map<Vertex, base::State *> &closeRepresentatives,
//                                                          const base::PlannerTerminationCondition &ptc)
// {
//     for (auto &closeRepresentative : closeRepresentatives)
//         si_->freeState(closeRepresentative.second);
//     closeRepresentatives.clear();

//     // Then, begin searching the space around him
//     for (unsigned int i = 0; i < nearSamplePoints_; ++i)
//     {
//         do
//         {
//             sampler_->sampleNear(workArea, qNew, denseDelta_);
//         } while ((!si_->isValid(workArea) || si_->distance(qNew, workArea) > denseDelta_ ||
//                   !si_->checkMotion(qNew, workArea)) &&
//                  !ptc);

//         // if we were not successful at sampling a desirable state, we are out of time
//         if (ptc)
//             break;

//         // Compute who his graph neighbors are
//         Vertex representative = findGraphRepresentative(workArea);

//         // Assuming this sample is actually seen by somebody (which he should be in all likelihood)
//         if (representative != boost::graph_traits<Graph>::null_vertex())
//         {
//             // If his representative is different than qNew
//             if (qRep != representative)
//                 // And we haven't already tracked this representative
//                 if (closeRepresentatives.find(representative) == closeRepresentatives.end())
//                     // Track the representative
//                     closeRepresentatives[representative] = si_->cloneState(workArea);
//         }
//         else
//         {
//             // This guy can't be seen by anybody, so we should take this opportunity to add him
//             addGuard(si_->cloneState(workArea), COVERAGE);

//             // We should also stop our efforts to add a dense path
//             for (auto &closeRepresentative : closeRepresentatives)
//                 si_->freeState(closeRepresentative.second);
//             closeRepresentatives.clear();
//             break;
//         }
//     }
// }
// bool ompl::geometric::BundleSpaceGraphSparse::checkAddPath(Vertex v)
// {
//     bool ret = false;

//     std::vector<Vertex> rs;
//     foreach (Vertex r, boost::adjacent_vertices(v, g_))
//         rs.push_back(r);

//     /* Candidate x vertices as described in the method, filled by function computeX(). */
//     std::vector<Vertex> Xs;

//     /* Candidate v" vertices as described in the method, filled by function computeVPP(). */
//     std::vector<Vertex> VPPs;

//     for (std::size_t i = 0; i < rs.size() && !ret; ++i)
//     {
//         Vertex r = rs[i];
//         computeVPP(v, r, VPPs);
//         foreach (Vertex rp, VPPs)
//         {
//             // First, compute the longest path through the graph
//             computeX(v, r, rp, Xs);
//             double rm_dist = 0.0;
//             foreach (Vertex rpp, Xs)
//             {
//                 double tmp_dist = (si_->distance(stateProperty_[r], stateProperty_[v]) +
//                                    si_->distance(stateProperty_[v], stateProperty_[rpp])) /
//                                   2.0;
//                 if (tmp_dist > rm_dist)
//                     rm_dist = tmp_dist;
//             }

//             InterfaceData &d = getData(v, r, rp);

//             // Then, if the spanner property is violated
//             if (rm_dist > stretchFactor_ * d.d_)
//             {
//                 ret = true;  // Report that we added for the path
//                 if (si_->checkMotion(stateProperty_[r], stateProperty_[rp]))
//                     connectGuards(r, rp);
//                 else
//                 {
//                     auto p(std::make_shared<PathGeometric>(si_));
//                     if (r < rp)
//                     {
//                         p->append(d.sigmaA_);
//                         p->append(d.pointA_);
//                         p->append(stateProperty_[v]);
//                         p->append(d.pointB_);
//                         p->append(d.sigmaB_);
//                     }
//                     else
//                     {
//                         p->append(d.sigmaB_);
//                         p->append(d.pointB_);
//                         p->append(stateProperty_[v]);
//                         p->append(d.pointA_);
//                         p->append(d.sigmaA_);
//                     }

//                     psimp_->reduceVertices(*p, 10);
//                     psimp_->shortcutPath(*p, 50);

//                     if (p->checkAndRepair(100).second)
//                     {
//                         Vertex prior = r;
//                         Vertex vnew;
//                         std::vector<base::State *> &states = p->getStates();

//                         foreach (base::State *st, states)
//                         {
//                             // no need to clone st, since we will destroy p; we just copy the pointer
//                             vnew = addGuard(st, QUALITY);

//                             connectGuards(prior, vnew);
//                             prior = vnew;
//                         }
//                         // clear the states, so memory is not freed twice
//                         states.clear();
//                         connectGuards(prior, rp);
//                     }
//                 }
//             }
//         }
//     }

//     return ret;
// }

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
    Configuration *ql = new Configuration(Bundle, q->state);
    const Vertex vl = add_vertex(ql, graphSparse_);
    nearestSparse_->add(ql);
    disjointSetsSparse_.make_set(vl);
    graphSparse_[vl]->index = vl;
    return vl;
}

void BundleSpaceGraphSparse::findGraphNeighbors(Configuration *q, std::vector<Configuration*> &graphNeighborhood,
                                                   std::vector<Configuration*> &visibleNeighborhood)
{
    visibleNeighborhood.clear();
    nearestSparse_->nearestR(q, sparseDelta_, graphNeighborhood);

    for (Configuration* qn : graphNeighborhood)
        if (Bundle->checkMotion(q->state, qn->state))
            visibleNeighborhood.push_back(qn);
}

void BundleSpaceGraphSparse::addEdgeSparse(const Vertex a, const Vertex b)
{
  ob::Cost weight = opt_->motionCost(graphSparse_[a]->state, graphSparse_[b]->state);
  EdgeInternalState properties(weight);
  boost::add_edge(a, b, properties, graphSparse_);
  uniteComponentsSparse(a, b);
}
bool BundleSpaceGraphSparse::checkAddConnectivity(Configuration* q, std::vector<Configuration*> &visibleNeighborhood)
{
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
                    links.push_back(visibleNeighborhood[i]->index);
                    links.push_back(visibleNeighborhood[j]->index);
                }
            }
        }

        if (!links.empty())
        {
            Vertex v = addConfigurationSparse(q);

            for (Vertex link : links){
                // If there's no edge
                if (!boost::edge(v, link, graphSparse_).second){
                    // And the components haven't been united by previous links
                    if (!sameComponentSparse(link, v)){
                        // connectGuards(g, link);
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
    // If we have more than 1 or 0 neighbors
    if (visibleNeighborhood.size() > 1)
    {
        Configuration *qn0 = graphNeighborhood[0];
        Configuration *qn1 = graphNeighborhood[1];
        Configuration *qv0 = visibleNeighborhood[0];
        Configuration *qv1 = visibleNeighborhood[1];

        if (qn0 == qv0 && qn1 == qv1){
            // If our two closest neighbors don't share an edge
            if (!boost::edge(qv0->index, qv1->index, graphSparse_).second)
            {
                // If they can be directly connected
                if (si_->checkMotion(qv0->state, qv1->state))
                {
                    addEdgeSparse(qv0->index, qv1->index);
                }else{
                  // Add the new node to the graph, to bridge the interface
                  // Vertex v = addGuard(si_->cloneState(qNew), INTERFACE);
                  Vertex v = addConfigurationSparse(q);
                  addEdgeSparse(v, qv0->index);
                  addEdgeSparse(v, qv1->index);
                }
                // Report success
                return true;
            }
        }
    }
    return false;
}


bool BundleSpaceGraphSparse::sampleBase(ob::State *q_random_graph)
{
    if( !getChild()->isDynamic() && pathStack_.size() > 0)
    {

        if(selectedPath >= 0 && selectedPath < (int)pathStack_.size())
        {
            // std::cout << "Sample " << getName() << " along selected path " << selectedPath 
            //   << "/" << (int)pathStackHead_.size()-1 << std::endl;

            std::vector<ob::State*> states = pathStackHead_.at(selectedPath);
            uint N = states.size();

            //############################################################################
            //Vertex Sampling
            // int k = rng_.uniformInt(0, N-1);
            // ob::State *state = states.at(k);
            // Bundle->getStateSpace()->copyState(q_random_graph, state);
            // Bundle_sampler->sampleUniformNear(q_random_graph, q_random_graph, 0.2);

            //############################################################################
            //Edge Sampling
            uint k = rng_.uniformInt(0, N-1);
            double r = rng_.uniform01();
            ob::State *s1 = states.at((k<N-1)?k:k-1);
            ob::State *s2 = states.at((k<N-1)?k+1:k);
            Bundle->getStateSpace()->interpolate(s1, s2, r, q_random_graph);

            Bundle_sampler_->sampleUniformNear(q_random_graph, q_random_graph, pathBias_);
        }else{
            OMPL_ERROR("Selected path is %d (have you selected a path?)");
            throw ompl::Exception("Unknown selected path");
        }
    }else{
        //no solution path, we can just sample randomly
        const Vertex v = boost::random_vertex(graph_, rng_boost);
        Bundle->getStateSpace()->copyState(q_random_graph, graph_[v]->state);
    }
    return true;
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
  std::vector<Configuration*> neighbors;
  uint Nv = boost::degree(v, graphSparse_);
  uint K = Nv+2;
  nearestSparse_->nearestK(const_cast<Configuration*>(q), K, neighbors);

  for(uint k = Nv+1; k < neighbors.size(); k++){
    Configuration *qn = neighbors.at(k);
    if(Bundle->checkMotion(q->state, qn->state))
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
    pathStackHead_.erase(pathStackHead_.end()-1);
}
void BundleSpaceGraphSparse::pushPathToStack(std::vector<ob::State*> &path)
{
  og::PathGeometric gpath(Bundle);
  for(uint k = 0; k < path.size(); k++){
    gpath.append(path.at(k));
  }

  ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(Bundle));
  ob::OptimizationObjectivePtr clearObj(new ob::MaximizeMinClearanceObjective(Bundle));
  ob::MultiOptimizationObjective* multiObj = new ob::MultiOptimizationObjective(Bundle);

  multiObj->addObjective(lengthObj, 1.0);
  multiObj->addObjective(clearObj, 1.0);
  ob::OptimizationObjectivePtr pathObj(multiObj);

  if(isDynamic()){
      // shortcutter.shortcutPath(gpath);
  }else{
      og::PathSimplifier shortcutter(Bundle, ob::GoalPtr(), pathObj);
      //make sure that we have enough vertices so that the right path class is
      //visualized (problems with S1)
      if(Bundle->getStateSpace()->getType() == ob::STATE_SPACE_SO2)
      {
          gpath.interpolate();
      }else{
          shortcutter.smoothBSpline(gpath);
          shortcutter.simplifyMax(gpath);
      }
  }

  if(!isDynamic() && !isProjectable(gpath.getStates())){
    std::cout << "REJECTED (Not projectable)" << std::endl;
    numberOfFailedAddingPathCalls++;
    return;
  }

  if(!isDynamic() && !pathVisibilityChecker_->CheckValidity(gpath.getStates())){
    std::cout << "REJECTED (Infeasible)" << std::endl;
    numberOfFailedAddingPathCalls++;
    return;
  }

  if(pathStack_.size() <= 0){
    pathStack_.push_back(gpath);
  }else{
    for(uint k = 0; k < pathStack_.size(); k++){
      og::PathGeometric& pathk = pathStack_.at(k);
      if(pathVisibilityChecker_->IsPathVisible(gpath.getStates(), pathk.getStates())){
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
  for(uint k = 0; k < pathStack_.size(); k++){
    std::vector<ob::State*> pathk = pathStack_.at(k).getStates();
    for(uint j = 0; j < pathk.size(); j++){
      Bundle->printState(pathk.at(j));
    }
    std::cout << std::string(80, '-') << std::endl;
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
    for(next = edge_iter; edge_iter != edge_iter_end; edge_iter = next)
    {
        const Vertex v_target = boost::target(*edge_iter, graphSparse_);
        if(v_target != v2) v1_neighbors.push_back(v_target);
        ++next;
    }
    boost::tie(edge_iter, edge_iter_end) = boost::out_edges(v2, graphSparse_);
    for(next = edge_iter; edge_iter != edge_iter_end; edge_iter = next)
    {
        const Vertex v_target = boost::target(*edge_iter, graphSparse_);
        if(v_target != v1) v2_neighbors.push_back(v_target);
        ++next;
    }

    for(uint k = 0; k < v1_neighbors.size(); k++){
      for(uint j = 0; j < v2_neighbors.size(); j++){
        const Vertex v1k = v1_neighbors.at(k);
        const Vertex v2k = v2_neighbors.at(j);
        if(v1k == v2k){
            common_neighbors.push_back(v1k);
        }
      }
    }

    //rm duplicates
    std::sort(common_neighbors.begin(), common_neighbors.end());
    auto last = std::unique(common_neighbors.begin(), common_neighbors.end());
    common_neighbors.erase(last, common_neighbors.end()); 

    //############################################################################
    //  (3) Check if face (v1, v2, v3) is feasible
    for(uint k = 0; k < common_neighbors.size(); k++){
        const Vertex v3 = common_neighbors.at(k);
        std::vector<Vertex> vpath1;
        vpath1.push_back(v1);
        vpath1.push_back(v3);
        vpath1.push_back(v2);
        std::vector<Vertex> vpath2;
        vpath2.push_back(v1);
        vpath2.push_back(v2);
        
        if(pathVisibilityChecker_->IsPathVisible(vpath1, vpath2, graphSparse_)){
          //RemoveEdge
          std::cout << "Removing Edge " << v1 << "<->" << v2 << std::endl;
          boost::remove_edge(v1, v2, graphSparse_);
        }
    }

}
void BundleSpaceGraphSparse::removeReducibleLoops()
{
    
    // Edge e = boost::random_edge(graphSparse_, rng_boost);
    uint Mend = boost::num_edges(graphSparse_);
    for(uint k = 0; k < Mend; k++){
      Edge e = boost::random_edge(graphSparse_, rng_boost);
      removeEdgeIfReductionLoop(e);
    }
}

void BundleSpaceGraphSparse::freePath(std::vector<ob::State*> path, const ob::SpaceInformationPtr &si) const
{
    for(uint k = 0; k < path.size(); k++){
      si->freeState(path.at(k));
    }
    path.clear();
}
std::vector<ob::State*> BundleSpaceGraphSparse::getProjectedPath(const std::vector<ob::State*> pathBundle, const ob::SpaceInformationPtr &si) const
{
    std::vector<ob::State*> pathBase;
    for(uint k = 0; k < pathBundle.size(); k++){
      const ob::State *qk = pathBundle.at(k);
      ob::State *qkProjected = Base->allocState();
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
    if(!hasParent()) return 0;
    std::vector<ob::State*> pathBase = getProjectedPath(pathBundle, Base);
    // for(uint k = 0; k < pathBundle.size(); k++){
    //   ob::State *qk = pathBundle.at(k);
    //   ob::State *qkProjected = Base->allocState();
    //   projectBase(qk, qkProjected);
    //   pathBase.push_back(qkProjected);
    // }

    BundleSpaceGraphSparse *Bundle = static_cast<BundleSpaceGraphSparse*>(parent_);
    unsigned int K = Bundle->getNumberOfPaths();

    for(uint k = 0; k < K; k++){
      std::vector<ob::State*> pathBasek = Bundle->getKthPath(k);
      bool visible = Bundle->getPathVisibilityChecker()->IsPathVisible(pathBase, pathBasek);
      if(visible){
        freePath(pathBase, Base);
        return k;
      }
    }
    freePath(pathBase, Base);
    return -1;
}

void BundleSpaceGraphSparse::getPathIndices(const std::vector<ob::State*> &states, std::vector<int> &idxPath) const
{

  if(!hasParent()){
    return;
  }else{
    BundleSpaceGraphSparse *Bundle = static_cast<BundleSpaceGraphSparse*>(parent_);
    //TODO: we need to check here to which local minima we project. This is
    //necessary, since sometimes we find a path which actually projects on a
    //different Bundle-space path (and not the selected one).
    // APPROACH 1: Assign them all to selected path
    // BundleSpaceGraphSparse *Bundle = static_cast<BundleSpaceGraphSparse*>(parent_);
    // unsigned int K = Bundle->getNumberOfPaths();
    // assert(K>0);
    // unsigned int Ks = Bundle->selectedPath;
    // assert(Ks>=0);
    // idxPath.push_back(Ks);
    // Bundle->getPathIndices(states, idxPath);
    if(isDynamic()){
      int Ks = Bundle->selectedPath;
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
      // Bundle->getPathIndices(states, idxPath);
    }
    std::vector<ob::State*> pathBase = getProjectedPath(states, Base);
    Bundle->getPathIndices(pathBase, idxPath);



    // APPROACH 2: Assign them to their projection
    //convert CS path to QS path
    //std::vector<ob::State*> pathcur;
    //for(uint k = 0; k < states.size(); k++){
    //  ob::State *qk = states.at(k);
    //  ob::State *qkProjected = Base->allocState();
    //  projectBase(qk, qkProjected);
    //  pathcur.push_back(qkProjected);
    //}
    ////Check which path can be deformed into QS path
    // BundleSpaceGraphSparse *Bundle = static_cast<BundleSpaceGraphSparse*>(parent_);
    // unsigned int K = Bundle->getNumberOfPaths();
    // assert(K>0);

    // bool success = false;
    // for(uint k = 0; k < K; k++){
    //   std::vector<ob::State*> pathk = Bundle->getKthPath(k);
    //   bool visible = Bundle->getPathVisibilityChecker()->IsPathVisible(pathcur, pathk);
    //   if(visible){
    //     idxPath.push_back(k);
    //     Bundle->getPathIndices(pathcur, idxPath);
    //     success = true;
    //     break;
    //   }
    // }
    //if(!success){
    //  //This path is not deformable into any of the BundleSpace paths 
    //  //One way to resolve this issue would be to add the new 
    //  OMPL_INFORM("Could not find projected path on BundleSpace. Creating new one.");

    //  Bundle->removeLastPathFromStack();
    //  Bundle->pushPathToStack(pathcur);
    //  idxPath.push_back(0);
    //  Bundle->getPathIndices(pathcur, idxPath);
    //}else{
    //  //free all states
    //  for(uint k = 0; k < pathcur.size(); k++){
    //    Base->freeState(pathcur.at(k));
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
  //terminate if we have enough paths in stack
    if(pathStack_.size() > Nhead) return;
    if(numberOfFailedAddingPathCalls>10) return;

    // Mark the current node and store it in path[]
    visited[u] = true;
    path[path_index] = u;
    path_index++;

    // If current vertex is same as destination, then print
    // current path[]
    if (u == d)
    {
        std::vector<ob::State*> pp;
        for (int i = 0; i<path_index; i++){
            pp.push_back(graphSparse_[path[i]]->state);
        }
        pushPathToStack(pp);
    }
    else // If current vertex is not destination
    {
        // Recur for all the vertices adjacent to current vertex
        OEIterator ei, ei_end;
        for (boost::tie(ei, ei_end) = boost::out_edges(u, graphSparse_); ei != ei_end; ++ei) {
          Vertex source = boost::source ( *ei, graphSparse_ );
          Vertex target = boost::target ( *ei, graphSparse_ );
          Vertex vnext = (source==u? target: source);
          if (!visited[vnext]){
              printAllPathsUtil(vnext, d, visited, path, path_index);
							if(pathStack_.size() > Nhead) break;
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
  if((Nv > Nold_v) || (Ne > Nold_e)){
      Nold_v = Nv;
      Nold_e = Ne;
      return true;
  }
  return false;
}

void BundleSpaceGraphSparse::enumerateAllPaths() 
{
    if(!hasSolution_) return;

    if(isDynamic()){

        ob::PathPtr path;

        const Configuration *q_nearest_to_goal = nearest(qGoal_);
        Configuration *qStartSparse = graphSparse_[v_start_sparse];
        path = getPathSparse(qStartSparse->index, q_nearest_to_goal->index);
        if(path==nullptr){
          OMPL_WARN("No solution found, but hasSolution_ is set.");
          return;
        }
        og::PathGeometric &gpath = static_cast<og::PathGeometric&>(*path);
        // pathStack_.push_back(gpath);

        unsigned int kBefore = pathStack_.size();
        pushPathToStack(gpath.getStates());
        unsigned int kAfter = pathStack_.size();
        if(kAfter > kBefore) clearDynamic();
    }else{

        //Check if we already enumerated all paths. If yes, then the number of
        //vertices has not changed. 
        if(!hasSparseGraphChanged())
        {
            return;
        }
        // TestVisibilityChecker();
        std::cout << "Enumerating paths on " << getName() << std::endl;

        //Remove Edges
        //(1) REDUCIBLE: Removal of reducible loops [Schmitzberger 02]
        removeReducibleLoops();
        //############################################################################

        // PathEnumerator pe(v_start_sparse, v_goal_sparse, graphSparse_);
        // pe.ComputePaths();

        unsigned numberVertices = boost::num_vertices(graphSparse_);
        if(numberVertices<=0) return;
        bool *visited = new bool[numberVertices];
        std::cout << "Sparse Graph has " << boost::num_vertices(graphSparse_) << " vertices and "
          << boost::num_edges(graphSparse_) << " edges." << std::endl;

        int *path = new int[numberVertices];
        int path_index = 0; // Initialize path[] as empty

        for (unsigned int i = 0; i < numberVertices; i++)
            visited[i] = false;

        numberOfFailedAddingPathCalls = 0;

        printAllPathsUtil(v_start_sparse, v_goal_sparse, visited, path, path_index);
        //############################################################################


    }
    uint Npathsize = pathStack_.size();
    uint Npaths = std::min(Nhead, Npathsize);
    pathStackHead_.clear();
    for(uint k = 0; k < Npaths; k++){
        //og::PathGeometric& pathK = (*(pathStack_.rbegin()+k));
        og::PathGeometric& pathK = pathStack_.at(k);//*(pathStack_.rbegin()+k));
        pathStackHead_.push_back(pathK.getStates());
    }
    OMPL_INFORM("Found %d path classes.", pathStackHead_.size());
    OMPL_INFORM("%s", std::string(80,'-').c_str());

    //TODO: update internally BundleSpace hierarchy. Create new BundleSpaces
    //for each path.
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

    data.addEdge(p1,p2);
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
    while(pparent!=nullptr){
      CurPath.push_back(pparent->selectedPath);
      pparent = static_cast<BundleSpaceGraphSparse*>(pparent->parent_);
    }
    if(selectedPath < 0) CurPath.push_back(0);
    else CurPath.push_back(selectedPath);

    return CurPath;
}

void BundleSpaceGraphSparse::getPlannerData(ob::PlannerData &data) const
{
  if(pathStackHead_.size()>0){
      OMPL_DEVMSG1("%s has %d solutions.", getName().c_str(), pathStackHead_.size());
      if(pathStackHead_.empty()){
          OMPL_ERROR("%s has 0 solutions.", getName().c_str());
          throw ompl::Exception("Zero solutions");
      }
      std::vector<int> idxPathI;
      for(uint i = 0; i < pathStackHead_.size(); i++){
          const std::vector<ob::State*> states = pathStackHead_.at(i);

          idxPathI.clear();
          getPathIndices(states, idxPathI);
          std::reverse(idxPathI.begin(), idxPathI.end());
          idxPathI.push_back(i);
          // idxPathI.insert(idxPathI.begin(), idxPathI.rbegin(), idxPathI.rend());

          //############################################################################
          //DEBUG
          std::cout << "[";
          for(uint k = 0; k < idxPathI.size(); k++){
            std::cout << idxPathI.at(k) << " ";
          }
          std::cout << "]" << std::endl;
          //############################################################################

          ob::PlannerDataVertexAnnotated *p1 = new ob::PlannerDataVertexAnnotated(states.at(0));
          p1->setLevel(level_);
          p1->setPath(idxPathI);
          data.addStartVertex(*p1);

          for(uint k = 0; k < states.size()-1; k++){

            ob::PlannerDataVertexAnnotated *p2 = new ob::PlannerDataVertexAnnotated(states.at(k+1));//Bundle->cloneState(graphSparse_[v2]->state));
            p2->setLevel(level_);
            p2->setPath(idxPathI);

            if(k==states.size()-2){
              data.addGoalVertex(*p2);
            }else{
              data.addVertex(*p2);
            }
            data.addEdge(*p1,*p2);

            p1 = p2;
          }
      }
      // idxPathI = GetSelectedPathIndex();
      getPlannerDataRoadmap(data, idxPathI);
  }else{

    OMPL_DEVMSG1("Sparse Roadmap has %d/%d vertices/edges (Dense has %d/%d).", 
        boost::num_vertices(graphSparse_), 
        boost::num_edges(graphSparse_), 
        boost::num_vertices(graph_),
        boost::num_edges(graph_));

    if(boost::num_vertices(graphSparse_) > 0){
      std::vector<int> CurPath = GetSelectedPathIndex();
      // for(uint k = 0; k < CurPath.size(); k++) std::cout << CurPath.at(k) << ",";
      // std::cout << std::endl;
        
      getPlannerDataRoadmap(data, CurPath);
    }

  }
}
