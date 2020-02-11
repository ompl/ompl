#include <ompl/geometric/planners/quotientspace/datastructures/components/None.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

ompl::geometric::BundleSpaceComponent_None::BundleSpaceComponent_None(
    base::StateSpacePtr BundleSpace,
    base::StateSpacePtr BaseSpace):
  BaseT(BundleSpace, BaseSpace)
{
}

void ompl::geometric::BundleSpaceComponent_None::projectFiber(
    const ompl::base::State *,
    ompl::base::State *) const
{
  OMPL_WARN("Trying to project to fiber with non-projectable Bundle Space.");
}

void ompl::geometric::BundleSpaceComponent_None::projectBase(
    const ompl::base::State *,
    ompl::base::State *) const
{
  OMPL_WARN("Trying to project to base with non-projectable Bundle Space.");
}


void ompl::geometric::BundleSpaceComponent_None::mergeStates(
    const ompl::base::State *, 
    const ompl::base::State *, 
    ompl::base::State *) const
{
  OMPL_WARN("Trying to merge States with non-projectable Bundle Space.");
}

ompl::base::StateSpacePtr ompl::geometric::BundleSpaceComponent_None::computeFiberSpace()
{
  return nullptr;
}
