#include <ompl/multilevel/datastructures/components/Identity.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

ompl::multilevel::BundleSpaceComponent_Identity::BundleSpaceComponent_Identity(base::StateSpacePtr BundleSpace,
                                                                              base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
}

void ompl::multilevel::BundleSpaceComponent_Identity::projectFiber(const ompl::base::State *, ompl::base::State *) const
{
    OMPL_WARN("Trying to project to fiber of Identity Bundle Space.");
}

void ompl::multilevel::BundleSpaceComponent_Identity::projectBase(const ompl::base::State *xBundle,
                                                                 ompl::base::State *xBase) const
{
    BundleSpace_->copyState(xBase, xBundle);
}

void ompl::multilevel::BundleSpaceComponent_Identity::liftState(const ompl::base::State *xBase,
                                                               const ompl::base::State *,
                                                               ompl::base::State *xBundle) const
{
    BundleSpace_->copyState(xBundle, xBase);
}

ompl::base::StateSpacePtr ompl::multilevel::BundleSpaceComponent_Identity::computeFiberSpace()
{
    return std::make_shared<base::RealVectorStateSpace>(0);
}
