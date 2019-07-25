#include <ompl/geometric/planners/explorer/QuotientGraphSparse.h>
#include <ompl/geometric/planners/quotientspace/PlannerDataVertexAnnotated.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>

#include <boost/property_map/vector_property_map.hpp>
#include <boost/property_map/transform_value_property_map.hpp>
#include <boost/foreach.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp> //same_component
using namespace og;
#define foreach BOOST_FOREACH

QuotientGraphSparse::QuotientGraphSparse(const ob::SpaceInformationPtr &si, QuotientSpace *parent):
  BaseT(si, parent)
{
  setName("QuotientGraphSparse");
  Planner::declareParam<double>("sparse_delta_fraction", this, &QuotientGraphSparse::setSparseDeltaFraction,
                                &QuotientGraphSparse::getSparseDeltaFraction, "0.0:0.01:1.0");

  if (!isSetup())
  {
    setup();
  }
  pathVisibilityChecker_ = new PathVisibilityChecker(Q1);
}

QuotientGraphSparse::~QuotientGraphSparse()
{
}

void QuotientGraphSparse::deleteConfiguration(Configuration *q)
{
  BaseT::deleteConfiguration(q);
}

void QuotientGraphSparse::setup()
{

  BaseT::setup();
  if (!nearestSparse_){
    nearestSparse_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration*>(this));
    nearestSparse_->setDistanceFunction([this](const Configuration *a, const Configuration *b)
                             {
                               return distance(a, b);
                             });
  }

   //Max 0.2, otherwise the circle would not work?
  // assert(sparseDeltaFraction_ <= 0.2);

  double maxExt = si_->getMaximumExtent();
  sparseDelta_ = sparseDeltaFraction_ * maxExt;

}

void QuotientGraphSparse::clear()
{
  BaseT::clear();

  if (nearestSparse_)
  {
    std::vector<Configuration*> configs;
    nearestSparse_->list(configs);
    for (auto &config : configs)
    {
      deleteConfiguration(config);
    }
    nearestSparse_->clear();
  }
  graphSparse_.clear();

  selectedPath = -1;
  graphNeighborhood.clear();
  visibleNeighborhood.clear();
  vrankSparse.clear();
  vparentSparse.clear();
  numberVertices = 0;
  v_start_sparse = -1;
  v_goal_sparse = -1;

  pathStackHead_.clear();
  pathStack_.clear();
}

void QuotientGraphSparse::Init()
{
  if(const ob::State *st = pis_.nextStart()){
    if (st != nullptr){
      qStart_ = new Configuration(Q1, st);
      qStart_->isStart = true;
      vStart_ = addConfiguration(qStart_);

      assert(boost::num_vertices(graphSparse_)==1);
      v_start_sparse = graphSparse_[0]->index;
      graphSparse_[v_start_sparse]->isStart = true;
    }
  }
  if (qStart_ == nullptr){
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    exit(0);
  }

  if(const ob::State *st = pis_.nextGoal()){
    if (st != nullptr){
      qGoal_ = new Configuration(Q1, st);
      qGoal_->isGoal = true;
      vGoal_ = addConfiguration(qGoal_);

      if(boost::num_vertices(graphSparse_)<2)
      {
        //Make sure q_goal is added, even if visible from q_start
          addConfigurationSparse(qGoal_);
      }
      v_goal_sparse = graphSparse_[1]->index;
      graphSparse_[v_goal_sparse]->isGoal = true;
    }
  }
  if (qGoal_ == nullptr){
    OMPL_ERROR("%s: There are no valid goal states!", getName().c_str());
    exit(0);
  }
  // uint startComponent = disjointSets_.find_set(v_start_sparse);
  // uint goalComponent = disjointSets_.find_set(v_goal_sparse);
  // std::cout << "start component:" << startComponent << "| goalComponent:" << goalComponent << std::endl;
}

QuotientGraphSparse::Vertex QuotientGraphSparse::addConfiguration(Configuration *q)
{
  Vertex v = BaseT::addConfiguration(q);

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
      }
    }
  }

  return v;
}

void QuotientGraphSparse::uniteComponentsSparse(Vertex m1, Vertex m2)
{
  disjointSetsSparse_.union_set(m1, m2);
}
bool QuotientGraphSparse::sameComponentSparse(Vertex m1, Vertex m2)
{
  return boost::same_component(m1, m2, disjointSetsSparse_);
}

QuotientGraphSparse::Vertex QuotientGraphSparse::addConfigurationSparse(Configuration *q)
{
    Configuration *ql = new Configuration(Q1, q->state);
    const Vertex vl = add_vertex(ql, graphSparse_);
    nearestSparse_->add(ql);
    disjointSetsSparse_.make_set(vl);
    graphSparse_[vl]->index = vl;
    return vl;
}

void QuotientGraphSparse::findGraphNeighbors(Configuration *q, std::vector<Configuration*> &graphNeighborhood,
                                                   std::vector<Configuration*> &visibleNeighborhood)
{
    visibleNeighborhood.clear();
    nearestSparse_->nearestR(q, sparseDelta_, graphNeighborhood);

    for (Configuration* qn : graphNeighborhood)
        if (Q1->checkMotion(q->state, qn->state))
            visibleNeighborhood.push_back(qn);
}

void QuotientGraphSparse::addEdgeSparse(const Vertex a, const Vertex b)
{
  ob::Cost weight = opt_->motionCost(graphSparse_[a]->state, graphSparse_[b]->state);
  EdgeInternalState properties(weight);
  boost::add_edge(a, b, properties, graphSparse_);
  uniteComponentsSparse(a, b);
}

bool QuotientGraphSparse::checkAddConnectivity(Configuration* q, std::vector<Configuration*> &visibleNeighborhood)
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
bool QuotientGraphSparse::checkAddInterface(Configuration *q,
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


bool QuotientGraphSparse::sampleQuotient(ob::State *q_random_graph)
{
    if(pathStack_.size() > 0){
      if(selectedPath >= 0 && selectedPath < (int)pathStack_.size()){
        // std::cout << "Sample " << getName() << " along selected path " << selectedPath 
        //   << "/" << (int)pathStackHead_.size()-1 << std::endl;

        std::vector<ob::State*> states = pathStackHead_.at(selectedPath);
        uint N = states.size();

        //############################################################################
        //Vertex Sampling
        // int k = rng_.uniformInt(0, N-1);
        // ob::State *state = states.at(k);
        // Q1->getStateSpace()->copyState(q_random_graph, state);
        // Q1_sampler->sampleUniformNear(q_random_graph, q_random_graph, 0.2);

        //############################################################################
        //Edge Sampling
        uint k = rng_.uniformInt(0, N-1);
        double r = rng_.uniform01();
        ob::State *s1 = states.at((k<N-1)?k:k-1);
        ob::State *s2 = states.at((k<N-1)?k+1:k);
        Q1->getStateSpace()->interpolate(s1, s2, r, q_random_graph);

        Q1_sampler_->sampleUniformNear(q_random_graph, q_random_graph, 0.1);


      }else{
        OMPL_ERROR("Selected path is %d (have you selected a path?)");
        exit(0);
      }
    }else{
        //no solution path, we can just sample randomly
        const Vertex v = boost::random_vertex(graph_, rng_boost);
        Q1->getStateSpace()->copyState(q_random_graph, graph_[v]->state);
    }
    return true;
}

unsigned int QuotientGraphSparse::getNumberOfPaths() const
{
    return pathStackHead_.size();
}
//############################################################################
//############################################################################

void QuotientGraphSparse::Rewire(Vertex &v)
{
  Configuration *q = graphSparse_[v];
  std::vector<Configuration*> neighbors;
  uint Nv = boost::degree(v, graphSparse_);
  uint K = Nv+2;
  nearestSparse_->nearestK(const_cast<Configuration*>(q), K, neighbors);

  for(uint k = Nv+1; k < neighbors.size(); k++){
    Configuration *qn = neighbors.at(k);
    if(Q1->checkMotion(q->state, qn->state))
    {
      addEdge(q->index, qn->index);
    }
  }
}

void QuotientGraphSparse::Rewire()
{
  Vertex v = boost::random_vertex(graphSparse_, rng_boost);
  return Rewire(v);
}


void QuotientGraphSparse::AddPathToStack(std::vector<ob::State*> &path)
{

  og::PathGeometric gpath(Q1);
  for(uint k = 0; k < path.size(); k++){
    gpath.append(path.at(k));
  }

  ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(Q1));
  ob::OptimizationObjectivePtr clearObj(new ob::MaximizeMinClearanceObjective(Q1));
  ob::MultiOptimizationObjective* multiObj = new ob::MultiOptimizationObjective(Q1);

  multiObj->addObjective(lengthObj, 1.0);
  multiObj->addObjective(clearObj, 1.0);
  ob::OptimizationObjectivePtr pathObj(multiObj);

  // std::cout << "PATH" << std::endl;
  // for(uint k = 0; k < path.size(); k++){
  //   ob::State *sk = path.at(k);
  //   Q1->printState(sk);
  // }
  og::PathSimplifier shortcutter(Q1, ob::GoalPtr(), pathObj);

  //make sure that we have enough vertices so that the right path class is
  //visualized (problems with S1)

  if(Q1->getStateSpace()->getType() == ob::STATE_SPACE_SO2)
  {
    gpath.interpolate();
  }else{

    shortcutter.simplifyMax(gpath);
    // shortcutter.smoothBSpline(gpath);
    // gpath.interpolate();
    // gpath.subdivide();
    // shortcutter.reduceVertices(gpath, 0, 0, 0.5);
    // shortcutter.shortcutPath(gpath, 5, 0, sparseDelta_);
  }

  // std::vector<ob::State*> gstates = gpath.getStates();
  // for(uint k = 0; k < gstates.size(); k++){
  //   ob::State *sk = gstates.at(k);
  //   Q1->printState(sk);
  // }

  std::cout << "Check path to be in stack: " << path.size() << std::endl;
  if(!pathVisibilityChecker_->CheckValidity(gpath.getStates())){
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
  std::cout << "Added to stack" << std::endl;
  // std::cout << ">>Inserting new path." << std::endl;
  // std::vector<ob::State*> sp = gpath.getStates();
  // for(uint k = 0; k < sp.size(); k++){
  //   Q1->printState(sp.at(k));
  // }

}
void QuotientGraphSparse::PrintPathStack()
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Path Stack" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  for(uint k = 0; k < pathStack_.size(); k++){
    std::vector<ob::State*> pathk = pathStack_.at(k).getStates();
    for(uint j = 0; j < pathk.size(); j++){
      Q1->printState(pathk.at(j));
    }
    std::cout << std::string(80, '-') << std::endl;
  }
}

void QuotientGraphSparse::removeEdgeIfReductionLoop(const Edge &e)
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
void QuotientGraphSparse::removeReducibleLoops()
{
    
    // Edge e = boost::random_edge(graphSparse_, rng_boost);
    uint Mend = boost::num_edges(graphSparse_);
    for(uint k = 0; k < Mend; k++){
      Edge e = boost::random_edge(graphSparse_, rng_boost);
      removeEdgeIfReductionLoop(e);
    }
}

void QuotientGraphSparse::getPathIndices(const std::vector<ob::State*> &states, std::vector<int> &idxPath) const
{

  if(parent_ == nullptr){
    // idxPath.push_back(0);
    return;
  }else{
    //convert CS path to QS path
    std::vector<ob::State*> pathcur;
    for(uint k = 0; k < states.size(); k++){
      ob::State *qk = states.at(k);
      ob::State *qkProjected = Q0->allocState();
      projectQ0Subspace(qk, qkProjected);
      pathcur.push_back(qkProjected);
    }
    //Check which path can be deformed into QS path
    QuotientGraphSparse *quotient = static_cast<QuotientGraphSparse*>(parent_);
    uint K = quotient->getNumberOfPaths();
    assert(K>0);

    bool success = false;
    for(uint k = 0; k < K; k++){
      std::vector<ob::State*> pathk = quotient->getKthPath(k);
      bool visible = quotient->getPathVisibilityChecker()->IsPathVisible(pathcur, pathk);
      if(visible){
        idxPath.push_back(k);
        quotient->getPathIndices(pathcur, idxPath);
        success = true;
        break;
      }
    }
    if(!success){
      OMPL_ERROR("Could not find projected path on QuotientSpace.");
      exit(0);
    }
    //free all states
    for(uint k = 0; k < pathcur.size(); k++){
      Q0->freeState(pathcur.at(k));
    }
  }
}

PathVisibilityChecker* QuotientGraphSparse::getPathVisibilityChecker()
{
  return pathVisibilityChecker_;
}
const std::vector<ob::State*> QuotientGraphSparse::getKthPath(uint k) const
{
    return pathStackHead_.at(k);
}

// A recursive function to print all paths from 'u' to 'd'.
// visited[] keeps track of vertices in current path.
// path[] stores actual vertices and path_index is current
// index in path[]
void QuotientGraphSparse::printAllPathsUtil(
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
        AddPathToStack(pp);
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

void QuotientGraphSparse::enumerateAllPaths() 
{
    if(!hasSolution_) return;

    //Check if we already enumerated all paths. If yes, then the number of
    //vertices has not changed. 
    if(boost::num_vertices(graphSparse_) <= numberVertices){
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

    numberVertices = boost::num_vertices(graphSparse_);
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

    uint Npathsize = pathStack_.size();
    uint Npaths = std::min(Nhead, Npathsize);

    pathStackHead_.clear();
    for(uint k = 0; k < Npaths; k++){
        //og::PathGeometric& pathK = (*(pathStack_.rbegin()+k));
        og::PathGeometric& pathK = pathStack_.at(k);//*(pathStack_.rbegin()+k));
        pathStackHead_.push_back(pathK.getStates());
    }
    std::cout << "Found " << pathStackHead_.size() << " path classes." << std::endl;
    std::cout << std::string(80, '-') << std::endl;

    //TODO: update internally QuotientSpace hierarchy. Create new QuotientSpaces
    //for each path.
}

void QuotientGraphSparse::getPlannerDataRoadmap(ob::PlannerData &data, std::vector<int> pathIdx) const
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

std::vector<int> QuotientGraphSparse::GetSelectedPathIndex() const
{
    std::vector<int> CurPath;
    QuotientGraphSparse *pparent = static_cast<QuotientGraphSparse*>(parent_);
    while(pparent!=nullptr){
      CurPath.push_back(pparent->selectedPath);
      pparent = static_cast<QuotientGraphSparse*>(pparent->parent_);
    }
    if(selectedPath < 0) CurPath.push_back(0);
    else CurPath.push_back(selectedPath);

    return CurPath;
}

void QuotientGraphSparse::getPlannerData(ob::PlannerData &data) const
{
  if(hasSolution_){
      std::vector<int> idxPathI;
      for(uint i = 0; i < pathStackHead_.size(); i++){
          const std::vector<ob::State*> states = pathStackHead_.at(i);

          idxPathI.clear();
          getPathIndices(states, idxPathI);
          idxPathI.push_back(i);

          ob::PlannerDataVertexAnnotated *p1 = new ob::PlannerDataVertexAnnotated(states.at(0));
          p1->setLevel(level_);
          p1->setPath(idxPathI);
          data.addStartVertex(*p1);

          for(uint k = 0; k < states.size()-1; k++){

            ob::PlannerDataVertexAnnotated *p2 = new ob::PlannerDataVertexAnnotated(states.at(k+1));//Q1->cloneState(graphSparse_[v2]->state));
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
      getPlannerDataRoadmap(data, idxPathI);
  }else{

    if(boost::num_vertices(graphSparse_) > 0){
      std::vector<int> CurPath = GetSelectedPathIndex();
      getPlannerDataRoadmap(data, CurPath);
    }

  }
}
