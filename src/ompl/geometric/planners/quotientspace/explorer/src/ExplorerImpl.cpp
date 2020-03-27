#include <ompl/geometric/planners/quotientspace/explorer/ExplorerImpl.h>
#include <ompl/geometric/planners/quotientspace/datastructures/PlannerDataVertexAnnotated.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/PDF.h>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/Control.h>
#include <ompl/control/SimpleDirectedControlSampler.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/base/DynamicalMotionValidator.h>
#include <boost/foreach.hpp>


using namespace og;
using namespace ob;
#define foreach BOOST_FOREACH

ExplorerImpl::ExplorerImpl(const ob::SpaceInformationPtr &si, BundleSpace *parent_ ):
  BaseT(si, parent_)
{
  setName("BundleSpaceExplorer"+std::to_string(id_));
  Planner::declareParam<double>("range", this, &ExplorerImpl::setRange, &ExplorerImpl::getRange, "0.:1.:10000.");
  Planner::declareParam<double>("goal_bias", this, &ExplorerImpl::setGoalBias, &ExplorerImpl::getGoalBias, "0.:.1:1.");

  specs_.approximateSolutions = true;
  approximateDistanceToGoal = std::numeric_limits<double>::infinity();

  if(isDynamic()) {
    ompl::control::SpaceInformation *siC = dynamic_cast<ompl::control::SpaceInformation*>(si.get());
    siC->setMotionValidator(std::make_shared<ob::DynamicalMotionValidator>(siC));
    siC->setup();
    siC->setMinMaxControlDuration(1,controlDuration);
    std::cout << "MaxControlDuration: " << controlDuration << std::endl;

    dCSampler = siC->allocDirectedControlSampler();
    //@TODO: need to write allocator for simpleDirectedControlSampler
    //dCSampler->setNumControlSamples(numberOfControlSamples);
    propStepSize = siC->getPropagationStepSize();
    prop = siC->getStatePropagator();
    c_random = siC->allocControl();
  }

  q_random = new Configuration(getBundle());

  pathVisibilityChecker_ = new PathVisibilityChecker(getBundle());

  double maxExt = getBundle()->getMaximumExtent();
  pathBias_ = pathBiasFraction_ * maxExt;
}

ExplorerImpl::~ExplorerImpl()
{
  deleteConfiguration(q_random);
  delete pathVisibilityChecker_;
}

void ExplorerImpl::setGoalBias(double goalBias_)
{
  goalBias = goalBias_;
}
double ExplorerImpl::getGoalBias() const
{
  return goalBias;
}
void ExplorerImpl::setRange(double maxDistance_)
{
  maxDistance = maxDistance_;
}
double ExplorerImpl::getRange() const
{
  return maxDistance;
}

void ExplorerImpl::setup()
{
  BaseT::setup();
  ompl::tools::SelfConfig sc(getBundle(), getName());
  sc.configurePlannerRange(maxDistance);
}

void ExplorerImpl::clear()
{
  BaseT::clear();
  selectedPath = -1;
  pathStackHead_.clear();
  pathStack_.clear();
}

bool ExplorerImpl::getSolution(ob::PathPtr &solution)
{
  if(hasSolution_){
      if(!isDynamic()) return BaseT::getSolution(solution);
      else{
        const Configuration *q_nearest_to_goal = nearest(qGoal_);
        solution = getPathSparse(qStart_->index, q_nearest_to_goal->index);
        return true;
      }

  }else{
    return false;
  }
}

void ExplorerImpl::grow(){
  if(firstRun_){
    Init();
    firstRun_ = false;
  }
  if(isDynamic()){
    if(hasSolution_ && num_vertices(graphSparse_)<=1){
      hasSolution_ = false;
    }
  }
  if(hasSolution_ && pathStackHead_.size()>0){
    //No Goal Biasing if we already found a solution on this quotient space
    sampleBundle(q_random->state);
  }else{
    double s = rng_.uniform01();
    if(s < goalBias){
      //sets q_random as qGoal_
      getBundle()->copyState(q_random->state, qGoal_->state); 
   }else{
      sampleBundle(q_random->state);
   }
  }
  if(isDynamic()) {
    growControl();
  } else {
    growGeometric();
    //growGeometric();
    //double v = getBundle()->getSpaceMeasure();
    //if(v > 1e4){ 
    //    //TODO: this is a hack to circumvent many local minima on "small" spaces 
    //    //which belong to the un-selected quotient-space path 
    //    //(which would otherwise be found through random walking)
    //    growGeometricExpand();
    //}
  }
}

bool ExplorerImpl::hasSolution()
{
  return BaseT::hasSolution();
  // return ((pathStackHead_.size()>0) && BaseT::hasSolution());
}

void ExplorerImpl::growGeometricExpand()
{
    PDF pdf;
    foreach (Vertex v, boost::vertices(graph_))
    {
        const unsigned long int t = graph_[v]->total_connection_attempts;
        pdf.add(graph_[v], (double)(t - graph_[v]->successful_connection_attempts) / (double)t);
    }
    if (pdf.empty()) return;

    std::vector<base::State *> workStates(5);
    getBundle()->allocStates(workStates);
    Configuration *qv = pdf.sample(rng_.uniform01());
    unsigned int s = getBundle()->randomBounceMotion(Bundle_sampler_, qv->state, workStates.size(), workStates, false);

    if (s > 0)
    {
        s--;
        Configuration *qn = new Configuration(getBundle(), workStates[s]);
        Vertex last = addConfiguration(qn);

        for (unsigned int i = 0; i < s; ++i)
        {
            // add the vertex along the bouncing motion
            Configuration *qs = new Configuration(getBundle(), workStates[i]);
            Vertex m = addConfiguration(qs);

            addEdge(qn->index, m);
            qn = qs;
        }

        if (s > 0 || !sameComponent(qv->index, last))
        {
            addEdge(qn->index, last);
        }
    }
    getBundle()->freeStates(workStates);
}

void ExplorerImpl::growGeometric(){
  
  const Configuration *q_nearest = nearest(q_random);

  double d = getBundle()->distance(q_nearest->state, q_random->state);
  if(d > maxDistance){
    getBundle()->getStateSpace()->interpolate(q_nearest->state, q_random->state, maxDistance / d, q_random->state);
  }

  if(getBundle()->checkMotion(q_nearest->state, q_random->state))
  {
    Configuration *q_next = new Configuration(getBundle(), q_random->state);

    //############################################################################
    //TODO: replace w AddConfig
    Vertex v_next = addConfiguration(q_next);
    // Configuration *q_next = addConfigurationDense(q_random);

    findGraphNeighbors(q_next, graphNeighborhood, visibleNeighborhood);

    if (!checkAddCoverage(q_next, visibleNeighborhood))
        if (!checkAddConnectivity(q_next, visibleNeighborhood))
            if (!checkAddInterface(q_next, graphNeighborhood, visibleNeighborhood))
            {
                if (!checkAddPath(q_next))
                    ++consecutiveFailures_;
            }
    //############################################################################

    addEdge(q_nearest->index, v_next);

    if(!hasSolution_){
      bool satisfied = sameComponentSparse(v_start_sparse, v_goal_sparse);
      if(satisfied)
      {
        hasSolution_ = true;
      }
    }
  }
}

void ExplorerImpl::growControl(){

    const Configuration *q_nearest = nearest(q_random);

    s_random = q_random->state;

    //changes q_random to the state we actually get with directed control c_random
    // ompl::control::SpaceInformation *siC = dynamic_cast<ompl::control::SpaceInformation*>(getBundle().get());
    // unsigned int cd = rng_.uniformInt(siC->getMinControlDuration(), siC->getMaxControlDuration());
    int duration = dCSampler->sampleTo(c_random, q_nearest->state, s_random);

    if(duration > controlDuration){
      //truncate motion
        prop->propagate(q_nearest->state, c_random, duration, s_random);
    }

    Configuration *q_next = new Configuration(getBundle(), s_random);
    Vertex v_next = addConfigurationSparse(q_next);
    addEdgeSparse(q_nearest->index, v_next);

    if(!hasSolution_){
        const Configuration *q_nearest_to_goal = nearest(qGoal_);
        goal_->isSatisfied(q_nearest_to_goal->state, &distanceToGoal);
        // if(hasSolution_){
        //   std::cout << "Found solution " << distanceToGoal << " away from goal." << std::endl;
        // }
        if (distanceToGoal < approximateDistanceToGoal)
        {
            approximateDistanceToGoal = distanceToGoal;
            std::cout << "Found new solution " << distanceToGoal << " away from goal." << std::endl;
            Configuration *qStartSparse = graphSparse_[v_start_sparse];
            // getBundle()->printState(qStartSparse->state);
            // getBundle()->printState(q_nearest_to_goal->state);
            // std::cout << q_nearest_to_goal->index << std::endl;
            // std::cout << qStartSparse->index << std::endl;

            if(approximateDistanceToGoal < 2){
                ob::PathPtr path = getPathSparse(qStartSparse->index, q_nearest_to_goal->index);
                if(path!=nullptr){
                    hasSolution_ = true;
                }
            }
        }
    }
}

void ExplorerImpl::getPlannerData(ob::PlannerData &data) const
{
  if(isDynamic()){
    if(!data.hasControls()){
      OMPL_ERROR("Dynamic Cspace, but PlannerData has no controls.");
    }
  }
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

            ob::PlannerDataVertexAnnotated *p2 = new ob::PlannerDataVertexAnnotated(states.at(k+1));//getBundle()->cloneState(graphSparse_[v2]->state));
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
//############################################################################
//EXPLORER
//############################################################################
void ExplorerImpl::sampleFromDatastructure(ob::State *q_random_graph)
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

unsigned int ExplorerImpl::getNumberOfPaths() const
{
    return pathStackHead_.size();
}

void ExplorerImpl::removeLastPathFromStack()
{
    pathStackHead_.erase(pathStackHead_.end() - 1);
}

void ExplorerImpl::pushPathToStack(std::vector<ob::State*> &path)
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

void ExplorerImpl::PrintPathStack()
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

void ExplorerImpl::removeEdgeIfReductionLoop(const Edge &e)
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
void ExplorerImpl::removeReducibleLoops()
{
    // Edge e = boost::random_edge(graphSparse_, rng_boost);
    uint Mend = boost::num_edges(graphSparse_);
    for (uint k = 0; k < Mend; k++)
    {
        Edge e = boost::random_edge(graphSparse_, rng_boost);
        removeEdgeIfReductionLoop(e);
    }
}

void ExplorerImpl::freePath(std::vector<ob::State*> path, const ob::SpaceInformationPtr &si) const
{
    for (uint k = 0; k < path.size(); k++)
    {
        si->freeState(path.at(k));
    }
    path.clear();
}

std::vector<ob::State*> ExplorerImpl::getProjectedPath(const std::vector<ob::State*> pathBundle, const ob::SpaceInformationPtr&) const
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

bool ExplorerImpl::isProjectable(const std::vector<ob::State*> &pathBundle) const
{
    return (getProjectionIndex(pathBundle) >= 0);
}

int ExplorerImpl::getProjectionIndex(const std::vector<ob::State*> &pathBundle) const
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

    ExplorerImpl *parent = static_cast<ExplorerImpl*>(parent_);
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

void ExplorerImpl::getPathIndices(const std::vector<ob::State*> &states, std::vector<int> &idxPath) const
{
  if(!hasParent())
  {
    return;
  }else{
    ExplorerImpl *parent = static_cast<ExplorerImpl*>(parent_);
    //TODO: we need to check here to which local minima we project. This is
    //necessary, since sometimes we find a path which actually projects on a
    //different Bundle-space path (and not the selected one).
    // APPROACH 1: Assign them all to selected path
    // ExplorerImpl *Bundle = static_cast<ExplorerImpl*>(parent_);
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
    // ExplorerImpl *Bundle = static_cast<ExplorerImpl*>(parent_);
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

PathVisibilityChecker* ExplorerImpl::getPathVisibilityChecker()
{
    return pathVisibilityChecker_;
}

const std::vector<ob::State*> ExplorerImpl::getKthPath(uint k) const
{
    return pathStackHead_.at(k);
}

// A recursive function to print all paths from 'u' to 'd'.
// visited[] keeps track of vertices in current path.
// path[] stores actual vertices and path_index is current
// index in path[]
void ExplorerImpl::printAllPathsUtil(
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
void ExplorerImpl::enumerateAllPaths() 
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

        unsigned int numberVertices = boost::num_vertices(graphSparse_);
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
std::vector<int> ExplorerImpl::GetSelectedPathIndex() const
{
    std::vector<int> CurPath;
    ExplorerImpl *pparent = static_cast<ExplorerImpl*>(parent_);
    while(pparent!=nullptr)
    {
        CurPath.push_back(pparent->selectedPath);
        pparent = static_cast<ExplorerImpl*>(pparent->parent_);
    }
    if (selectedPath < 0)
        CurPath.push_back(0);
    else
        CurPath.push_back(selectedPath);

    return CurPath;
}

//############################################################################

