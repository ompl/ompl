#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/control/PathControl.h>
#include <ompl/control/optimizers/PathControlOptimizer.h>
#include <ompl/control/optimizers/Optimizer.h>

namespace og = ompl::geometric;
namespace ob = ompl::base;
namespace oc = ompl::control;
using namespace og;
using namespace ob;

ompl::control::Optimizer::Optimizer(const base::SpaceInformationPtr &si, const base::PathPtr &path)
  : base::Planner(si, "Optimizer"), path_(path)
{
}

ompl::control::Optimizer::~Optimizer()
{
}

void ompl::control::Optimizer::clear()
{
}

void ompl::control::Optimizer::setup()
{
}

bool ompl::control::Optimizer::IsGeometric() const
{
  og::PathGeometric* gpath = dynamic_cast<og::PathGeometric*>(path_.get());
  return (gpath!=nullptr);
}

ompl::base::PlannerStatus ompl::control::Optimizer::solve(const base::PlannerTerminationCondition &ptc)
{
  if(IsGeometric()){
    OMPL_WARN("Optimizing on geometric Path.");
    og::PathGeometric* gpath = static_cast<og::PathGeometric*>(path_.get());
    og::PathSimplifier simplifier(si_);
    simplifier.simplify(*gpath,0);
  }else{
    oc::PathControl* cpath = static_cast<oc::PathControl*>(path_.get());
    base::State* goalState = si_->allocState() ;
    std::static_pointer_cast<const base::GoalSampleableRegion> (pdef_->getGoal())->sampleGoal(goalState) ;
    
    oc::PathControlOptimizer control_simplifier(si_,goalState);
    control_simplifier.simplify(cpath);
  }
}

void ompl::control::Optimizer::getPlannerData(base::PlannerData &data) const
{
  std::vector<ob::State *> states;
  if(IsGeometric()){
    og::PathGeometric* gpath = static_cast<og::PathGeometric*>(path_.get());
    states = gpath->getStates();
  }else{
    oc::PathControl* cpath = dynamic_cast<oc::PathControl*>(path_.get());
    states = cpath->getStates();
  }

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

