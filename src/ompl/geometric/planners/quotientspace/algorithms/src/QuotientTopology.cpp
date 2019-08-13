#include <ompl/geometric/planners/quotientspace/algorithms/QuotientTopology.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/Control.h>
#include <ompl/control/SimpleDirectedControlSampler.h>
#include <ompl/control/StatePropagator.h>

using namespace og;
using namespace ob;
//using namespace oc;
#define foreach BOOST_FOREACH

QuotientTopology::QuotientTopology(const ob::SpaceInformationPtr &si, QuotientSpace *parent_ ):
  BaseT(si, parent_)
{
  setName("QuotientSpaceTopology"+std::to_string(id_));
  Planner::declareParam<double>("range", this, &QuotientTopology::setRange, &QuotientTopology::getRange, "0.:1.:10000.");
  Planner::declareParam<double>("goal_bias", this, &QuotientTopology::setGoalBias, &QuotientTopology::getGoalBias, "0.:.1:1.");

  if(isDynamic) {
    ompl::control::SpaceInformation *siC = dynamic_cast<ompl::control::SpaceInformation*>(si.get());

    dCSampler = siC->allocDirectedControlSampler();
    //only works for simpleDirectedControlSampler, which is used as default, but method can't be called
    //dCSampler->setNumControlSamples(numberOfControlSamples);
    propStepSize = siC->getPropagationStepSize();
    prop = siC->getStatePropagator();
    c_random = siC->allocControl();
    //auto *rmotion = new Motion(siC);
    //ompl::control::Control *c_random = rmotion->control;
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
    return BaseT::getSolution(solution);
  }else{
    return false;
  }
}

void QuotientTopology::grow(){
  if(firstRun_){
    Init();
    firstRun_ = false;
  }
  if(hasSolution_){
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
  if(isDynamic) {
    growControl();
  } else {
    growGeometric();
  }
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

void QuotientTopology::growControl(){
  //do this, if control-case
    //std::cout << "Got here 111111111111111111111111111111" << std::endl;
    const Configuration *q_nearest = nearest(q_random);
    s_random = q_random->state;
    //std::cout << "Got here 222222222222222222222222222222" << std::endl;
    //std::cout << c_random << std::endl;
    //ompl::control::Control c_rand = ompl::control::Control();
    //changes q_random to the state we actually get with directed control c_random
    int duration = dCSampler->sampleTo(c_random, q_nearest->state, s_random);
    //c_random is always collisionfree if applied to q_nearest
    //std::cout << "Got here 333333333333333333333333333333" << std::endl;
    totalNumberOfSamples_++;
    totalNumberOfFeasibleSamples_++;
    if(duration<controlDuration){
      //used control for full duration, add q_random
      Configuration *q_next = new Configuration(Q1, s_random);
      Vertex v_next = addConfiguration(q_next);
      addEdge(q_nearest->index, v_next);
      //std::cout<<"1111111111"<<std::endl;
    } else {
      //sets q_reached to the State we actually reach with our control for controlDuration
      prop->propagate(q_nearest->state, c_random, controlDuration,s_random);
      Configuration *q_next = new Configuration(Q1, s_random);
      Vertex v_next = addConfiguration(q_next);
      addEdge(q_nearest->index, v_next);
      //std::cout<<"22222222222"<<std::endl;
    }
    if(!hasSolution_){
      bool satisfied = sameComponentSparse(v_start_sparse, v_goal_sparse);
      if(satisfied)
      {
        hasSolution_ = true;
      }
    }
  }

