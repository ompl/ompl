#include <ompl/geometric/planners/quotientspace/datastructures/components/Identity.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

ompl::geometric::BundleSpaceComponent_Identity::BundleSpaceComponent_Identity(
    base::StateSpacePtr BundleSpace,
    base::StateSpacePtr BaseSpace):
  BaseT(BundleSpace, BaseSpace)
{
}

void ompl::geometric::BundleSpaceComponent_Identity::projectFiber(
    const ompl::base::State*,
    ompl::base::State*) const
{
    OMPL_WARN("Trying to project to fiber with non-projectable Bundle Space.");
}

void ompl::geometric::BundleSpaceComponent_Identity::projectBase(
    const ompl::base::State *xBundle,
    ompl::base::State *xBase) const
{
    BundleSpace_->copyState(xBase, xBundle);
}

void ompl::geometric::BundleSpaceComponent_Identity::mergeStates(
    const ompl::base::State *xBase, 
    const ompl::base::State*, 
    ompl::base::State *xBundle) const
{
    BundleSpace_->copyState(xBundle, xBase);
}

ompl::base::StateSpacePtr ompl::geometric::BundleSpaceComponent_Identity::computeFiberSpace()
{
    return std::make_shared<base::RealVectorStateSpace>(0);
}

