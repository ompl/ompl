#include <ompl/geometric/planners/quotientspace/datastructures/components/NoProjection.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

ompl::geometric::BundleSpaceComponent_NoProjection::BundleSpaceComponent_NoProjection(
    base::StateSpacePtr BundleSpace,
    base::StateSpacePtr BaseSpace):
  BaseT(BundleSpace, BaseSpace)
{
}

void ompl::geometric::BundleSpaceComponent_NoProjection::projectFiber(
    const ompl::base::State *xBundle,
    ompl::base::State *xFiber) const
{
  OMPL_WARN("Trying to project to fiber with non-projectable Bundle Space.");
}

void ompl::geometric::BundleSpaceComponent_NoProjection::projectBase(
    const ompl::base::State *xBundle,
    ompl::base::State *xBase) const
{
  OMPL_WARN("Trying to project to base with non-projectable Bundle Space.");
}


void ompl::geometric::BundleSpaceComponent_NoProjection::mergeStates(
    const ompl::base::State *xBase, 
    const ompl::base::State *xFiber, 
    ompl::base::State *xBundle) const
{
  OMPL_WARN("Trying to merge States with non-projectable Bundle Space.");
}

ompl::base::StateSpacePtr ompl::geometric::BundleSpaceComponent_NoProjection::computeFiberSpace()
{
  return nullptr;
}
