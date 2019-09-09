#include <ompl/geometric/planners/quotientspace/algorithms/QuotientTopology.h>
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

QuotientTopology::QuotientTopology(const ob::SpaceInformationPtr &si, QuotientSpace *parent_ ):
  BaseT(si, parent_)
{
  setName("QuotientSpaceTopology"+std::to_string(id_));
  Planner::declareParam<double>("range", this, &QuotientTopology::setRange, &QuotientTopology::getRange, "0.:1.:10000.");
  Planner::declareParam<double>("goal_bias", this, &QuotientTopology::setGoalBias, &QuotientTopology::getGoalBias, "0.:.1:1.");

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

  q_random = new Configuration(Q1);
}

QuotientTopology::~QuotientTopology()
{
  deleteConfiguration(q_random);
}

void QuotientTopology::setGoalBias(double goalBias_)
{
  goalBias = goalBias_;
}
double QuotientTopology::getGoalBias() const
{
  return goalBias;
}
void QuotientTopology::setRange(double maxDistance_)
{
  maxDistance = maxDistance_;
}
double QuotientTopology::getRange() const
{
  return maxDistance;
}

void QuotientTopology::setup()
{
  BaseT::setup();
  ompl::tools::SelfConfig sc(Q1, getName());
  sc.configurePlannerRange(maxDistance);
}

void QuotientTopology::clear()
{
  BaseT::clear();
}

bool QuotientTopology::getSolution(ob::PathPtr &solution)
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

void QuotientTopology::grow(){
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
    sample(q_random->state);
  }else{
    double s = rng_.uniform01();
    if(s < goalBias){
      //sets q_random as qGoal_
      Q1->copyState(q_random->state, qGoal_->state); 
   }else{
      sample(q_random->state);
   }
  }
  if(isDynamic()) {
    growControl();
  } else {
    growGeometric();
    //growGeometric();
    //double v = Q1->getSpaceMeasure();
    //if(v > 1e4){ 
    //    //TODO: this is a hack to circumvent many local minima on "small" spaces 
    //    //which belong to the un-selected quotient-space path 
    //    //(which would otherwise be found through random walking)
    //    growGeometricExpand();
    //}
  }
}
bool QuotientTopology::hasSolution()
{
  return BaseT::hasSolution();
  // return ((pathStackHead_.size()>0) && BaseT::hasSolution());
}

void QuotientTopology::growGeometricExpand()
{
    PDF pdf;
    foreach (Vertex v, boost::vertices(graph_))
    {
        const unsigned long int t = graph_[v]->total_connection_attempts;
        pdf.add(graph_[v], (double)(t - graph_[v]->successful_connection_attempts) / (double)t);
    }
    if (pdf.empty()) return;

    std::vector<base::State *> workStates(5);
    Q1->allocStates(workStates);
    Configuration *qv = pdf.sample(rng_.uniform01());
    unsigned int s = Q1->randomBounceMotion(Q1_sampler_, qv->state, workStates.size(), workStates, false);

    if (s > 0)
    {
        s--;
        Configuration *qn = new Configuration(Q1, workStates[s]);
        Vertex last = addConfiguration(qn);

        for (unsigned int i = 0; i < s; ++i)
        {
            // add the vertex along the bouncing motion
            Configuration *qs = new Configuration(Q1, workStates[i]);
            Vertex m = addConfiguration(qs);

            addEdge(qn->index, m);
            qn = qs;
        }

        if (s > 0 || !sameComponent(qv->index, last))
        {
            addEdge(qn->index, last);
        }
    }
    Q1->freeStates(workStates);
}
void QuotientTopology::growGeometric(){
  
  const Configuration *q_nearest = nearest(q_random);
  double d = Q1->distance(q_nearest->state, q_random->state);
  if(d > maxDistance){
    Q1->getStateSpace()->interpolate(q_nearest->state, q_random->state, maxDistance / d, q_random->state);
  }

  totalNumberOfSamples_++;
  if(Q1->checkMotion(q_nearest->state, q_random->state))
  {
    totalNumberOfFeasibleSamples_++;
    Configuration *q_next = new Configuration(Q1, q_random->state);

    Vertex v_next = addConfiguration(q_next);
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
void QuotientTopology::growControl(){
  //do this, if control-case
    const Configuration *q_nearest = nearest(q_random);
    s_random = q_random->state;

    //changes q_random to the state we actually get with directed control c_random
    ompl::control::SpaceInformation *siC = dynamic_cast<ompl::control::SpaceInformation*>(Q1.get());
    unsigned int cd = rng_.uniformInt(siC->getMinControlDuration(), siC->getMaxControlDuration());
    int duration = dCSampler->sampleTo(c_random, q_nearest->state, s_random);

    //c_random is always collisionfree if applied to q_nearest
    totalNumberOfSamples_++;
    totalNumberOfFeasibleSamples_++;

    if(duration<controlDuration){
        //used control for full duration, add q_random
        Configuration *q_next = new Configuration(Q1, s_random);
        Vertex v_next = addConfigurationSparse(q_next);
        addEdgeSparse(q_nearest->index, v_next);
    } else {
        //sets q_reached to the State we actually reach with our control for controlDuration
        prop->propagate(q_nearest->state, c_random, duration, s_random);
        Configuration *q_next = new Configuration(Q1, s_random);
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
            // Q1->printState(qStartSparse->state);
            // Q1->printState(q_nearest_to_goal->state);
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

