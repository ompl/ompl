#include "ompl/geometric/planners/quotientspace/Optimizer.h"
#include <ompl/geometric/PathGeometric.h>

namespace og = ompl::geometric;
namespace ob = ompl::base;
using namespace og;
using namespace ob;

ompl::geometric::Optimizer::Optimizer(const base::SpaceInformationPtr &si, const base::PathPtr &path)
  : base::Planner(si, "Optimizer"), path_(path)
{
}

ompl::geometric::Optimizer::~Optimizer()
{
}

void ompl::geometric::Optimizer::clear()
{
}

void ompl::geometric::Optimizer::setup()
{
}

ompl::base::PlannerStatus ompl::geometric::Optimizer::solve(const base::PlannerTerminationCondition &ptc)
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "TODO: Optimize Path" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
}

void ompl::geometric::Optimizer::getPlannerData(base::PlannerData &data) const
{
  og::PathGeometric gpath = static_cast<og::PathGeometric&>(*path_);
  std::vector<ob::State *> states = gpath.getStates();
  std::cout << "Path has " << states.size() << " states." << std::endl;

  if(states.size()>1){
    ob::State *last = si_->allocState();
    si_->copyState(last, states.at(0));
    data.addStartVertex(base::PlannerDataVertex(last));

    for(uint k = 1; k < states.size()-1; k++){
      ob::State *next = si_->allocState();
      si_->copyState(next, states.at(k));
      data.addVertex(base::PlannerDataVertex(next));
      data.addEdge(last, next);
      last = next;
    }
    ob::State *goal = si_->allocState();
    si_->copyState(goal, states.back());
    data.addGoalVertex(base::PlannerDataVertex(goal));
    data.addEdge(last, goal);
  }
  data.path_ = path_;
}

