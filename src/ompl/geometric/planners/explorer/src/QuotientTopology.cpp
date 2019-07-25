#include <ompl/geometric/planners/explorer/QuotientTopology.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

using namespace og;
using namespace ob;
#define foreach BOOST_FOREACH

QuotientTopology::QuotientTopology(const ob::SpaceInformationPtr &si, QuotientSpace *parent_ ):
  BaseT(si, parent_)
{
  setName("QuotientSpaceTopology"+std::to_string(id_));
  Planner::declareParam<double>("range", this, &QuotientTopology::setRange, &QuotientTopology::getRange, "0.:1.:10000.");
  Planner::declareParam<double>("goal_bias", this, &QuotientTopology::setGoalBias, &QuotientTopology::getGoalBias, "0.:.1:1.");

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
  Q1->printSettings();
  ompl::tools::SelfConfig sc(Q1, getName());
  sc.configurePlannerRange(maxDistance);
  goal = pdef_->getGoal().get();
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
      Q1->copyState(q_random->state, qGoal_->state);
    }else{
      sample(q_random->state);
    }
  }

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
        // std::cout << std::string(80, '*') << std::endl;
        // std::cout << getName() << " Found Solution " << std::endl;
        // std::cout << std::string(80, '*') << std::endl;
        // uint startComponent = disjointSets_.find_set(v_start_sparse);
        // uint goalComponent = disjointSets_.find_set(v_goal_sparse);
        // std::cout << "start component:" << startComponent << "| goalComponent:" << goalComponent << std::endl;
        // EIterator it, end;
        // std::cout << "start vertex:" << v_start_sparse << "| goal vertex:" << v_goal_sparse << std::endl;
        // for(tie(it, end) = boost::edges(graphSparse_); it != end; ++it)
        // {
        //     const Vertex v1 = boost::source(*it, graphSparse_);
        //     const Vertex v2 = boost::target(*it, graphSparse_);
        //     std::cout << v1 << "-" << v2 << std::endl;
        // }
      }
    }
  }
}

