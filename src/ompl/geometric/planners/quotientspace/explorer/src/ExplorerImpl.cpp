#include <ompl/geometric/planners/quotientspace/explorer/ExplorerImpl.h>
#include <ompl/geometric/planners/quotientspace/datastructures/PlannerDataVertexAnnotated.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/PDF.h>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
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

  q_random = new Configuration(Bundle);
}

ExplorerImpl::~ExplorerImpl()
{
  deleteConfiguration(q_random);
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
  ompl::tools::SelfConfig sc(Bundle, getName());
  sc.configurePlannerRange(maxDistance);
}

void ExplorerImpl::clear()
{
  BaseT::clear();
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
      Bundle->copyState(q_random->state, qGoal_->state); 
   }else{
      sampleBundle(q_random->state);
   }
  }
  if(isDynamic()) {
    growControl();
  } else {
    growGeometric();
    //growGeometric();
    //double v = Bundle->getSpaceMeasure();
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
    Bundle->allocStates(workStates);
    Configuration *qv = pdf.sample(rng_.uniform01());
    unsigned int s = Bundle->randomBounceMotion(Bundle_sampler_, qv->state, workStates.size(), workStates, false);

    if (s > 0)
    {
        s--;
        Configuration *qn = new Configuration(Bundle, workStates[s]);
        Vertex last = addConfiguration(qn);

        for (unsigned int i = 0; i < s; ++i)
        {
            // add the vertex along the bouncing motion
            Configuration *qs = new Configuration(Bundle, workStates[i]);
            Vertex m = addConfiguration(qs);

            addEdge(qn->index, m);
            qn = qs;
        }

        if (s > 0 || !sameComponent(qv->index, last))
        {
            addEdge(qn->index, last);
        }
    }
    Bundle->freeStates(workStates);
}

void ExplorerImpl::growGeometric(){
  
  const Configuration *q_nearest = nearest(q_random);
  double d = Bundle->distance(q_nearest->state, q_random->state);
  if(d > maxDistance){
    Bundle->getStateSpace()->interpolate(q_nearest->state, q_random->state, maxDistance / d, q_random->state);
  }

  if(Bundle->checkMotion(q_nearest->state, q_random->state))
  {
    Configuration *q_next = new Configuration(Bundle, q_random->state);

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

//(1) Directed Graph
//(2) Do not add goal
void ExplorerImpl::growControl(){
  //do this, if control-case
    const Configuration *q_nearest = nearest(q_random);
    s_random = q_random->state;

    //changes q_random to the state we actually get with directed control c_random
    ompl::control::SpaceInformation *siC = dynamic_cast<ompl::control::SpaceInformation*>(Bundle.get());
    unsigned int cd = rng_.uniformInt(siC->getMinControlDuration(), siC->getMaxControlDuration());
    int duration = dCSampler->sampleTo(c_random, q_nearest->state, s_random);

    //c_random is always collisionfree if applied to q_nearest

    if(duration<controlDuration){
        //used control for full duration, add q_random
        Configuration *q_next = new Configuration(Bundle, s_random);
        Vertex v_next = addConfigurationSparse(q_next);
        addEdgeSparse(q_nearest->index, v_next);
    } else {
        //sets q_reached to the State we actually reach with our control for controlDuration
        prop->propagate(q_nearest->state, c_random, duration, s_random);
        Configuration *q_next = new Configuration(Bundle, s_random);
        Vertex v_next = addConfigurationSparse(q_next);
        addEdgeSparse(q_nearest->index, v_next);
    }
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
            // Bundle->printState(qStartSparse->state);
            // Bundle->printState(q_nearest_to_goal->state);
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
