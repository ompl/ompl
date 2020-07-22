#include <ompl/multilevel/datastructures/components/EmptySet.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

ompl::multilevel::BundleSpaceComponent_EmptySet::BundleSpaceComponent_EmptySet(base::StateSpacePtr BundleSpace,
                                                                              base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
}

void ompl::multilevel::BundleSpaceComponent_EmptySet::projectFiber(const ompl::base::State *xBundle,
                                                                  ompl::base::State *xFiber) const
{
    BundleSpace_->copyState(xFiber, xBundle);
}

void ompl::multilevel::BundleSpaceComponent_EmptySet::projectBase(const ompl::base::State *, ompl::base::State *) const
{
    OMPL_WARN("Trying to project to base of Empty-Set Projection space.");
}

void ompl::multilevel::BundleSpaceComponent_EmptySet::liftState(const ompl::base::State *,
                                                               const ompl::base::State *xFiber,
                                                               ompl::base::State *xBundle) const
{
    BundleSpace_->copyState(xBundle, xFiber);
}

ompl::base::StateSpacePtr ompl::multilevel::BundleSpaceComponent_EmptySet::computeFiberSpace()
{
    return BundleSpace_;
}
