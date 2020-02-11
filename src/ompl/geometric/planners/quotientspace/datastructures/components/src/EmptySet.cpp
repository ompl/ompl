#include <ompl/geometric/planners/quotientspace/datastructures/components/EmptySet.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

ompl::geometric::BundleSpaceComponent_EmptySet::BundleSpaceComponent_EmptySet(
    base::StateSpacePtr BundleSpace,
    base::StateSpacePtr BaseSpace):
  BaseT(BundleSpace, BaseSpace)
{
}

void ompl::geometric::BundleSpaceComponent_EmptySet::projectFiber(
    const ompl::base::State*,
    ompl::base::State*) const
{
    OMPL_WARN("Trying to project to fiber with Empty-Set Projection space.");
}

void ompl::geometric::BundleSpaceComponent_EmptySet::projectBase(
    const ompl::base::State *,
    ompl::base::State *) const
{
}

void ompl::geometric::BundleSpaceComponent_EmptySet::mergeStates(
    const ompl::base::State *, 
    const ompl::base::State *xFiber,
    ompl::base::State *xBundle) const
{
    BundleSpace_->copyState(xBundle, xFiber);
}

ompl::base::StateSpacePtr ompl::geometric::BundleSpaceComponent_EmptySet::computeFiberSpace()
{
    return BundleSpace_;
}

