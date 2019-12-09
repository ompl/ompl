#include <ompl/control/optimizers/PathControlOptimizer.h>

ompl::control::PathControlOptimizer::PathControlOptimizer(control::SpaceInformationPtr si, const base::OptimizationObjectivePtr& obj)
  : si_(si), obj_(obj)
{
}
void ompl::control::PathControlOptimizer::simplify(PathControl* path)
{
  std::cout << "NYI: Returning original path" << std::endl;

}

