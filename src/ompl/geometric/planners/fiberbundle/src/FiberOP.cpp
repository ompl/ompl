#include <ompl/geometric/planners/fiberbundle/FiberOP.h>
#include <ompl/geometric/planners/quotientspace/QRRT.h>

namespace og = ompl::geometric;
namespace ob = ompl::base;
using namespace og;
using namespace ob;

ompl::geometric::FiberOP::FiberOP( ob::SpaceInformationPtr si, std::vector<FiberBundle> &fiberBundles, std::string type)
  : ompl::base::Planner(si, type), fiberBundles_(fiberBundles)
{
  std::cout << "Init " << fiberBundles.size() << " fiber bundles (unfiltered)." << std::endl;
}

ompl::geometric::FiberOP::~FiberOP()
{
}

void ompl::geometric::FiberOP::clear()
{
}

void ompl::geometric::FiberOP::setup()
{
}

ompl::base::PlannerStatus ompl::geometric::FiberOP::solve(const base::PlannerTerminationCondition &ptc)
{
  iteration_ = 1;
  std::cout << "Optimizing fiber bundle." << std::endl;

  for(uint k = 0; k < fiberBundles_.size(); k++){
      FiberBundle bundle = fiberBundles_.at(k);
      base::PlannerPtr planner = std::make_shared<og::QRRT>(bundle);
      planner->setProblemDefinition(pdef_);
      planner->setup();
      std::static_pointer_cast<og::QRRT>(planner)->setStopLevel(1);
      unfilteredPlanners_.push_back(planner);
  }
  std::cout << "Iteration " << iteration_ << std::endl;
  double timeToPlan = 0.5;
  for(uint k = 0; k < unfilteredPlanners_.size(); k++){
    ob::PlannerPtr planner = unfilteredPlanners_.at(k);
    ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(timeToPlan) );
    planner->solve(ptc);

  }
}

void ompl::geometric::FiberOP::getPlannerData(base::PlannerData &data) const
{
}


