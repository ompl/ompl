#include <ompl/multilevel/datastructures/components/None.h>

ompl::multilevel::BundleSpaceComponent_None::BundleSpaceComponent_None(base::StateSpacePtr BundleSpace,
                                                                      base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
}

void ompl::multilevel::BundleSpaceComponent_None::projectFiber(const ompl::base::State *, ompl::base::State *) const
{
    OMPL_WARN("Trying to project to fiber with non-projectable Bundle Space.");
}

void ompl::multilevel::BundleSpaceComponent_None::projectBase(const ompl::base::State *, ompl::base::State *) const
{
    OMPL_WARN("Trying to project to base with non-projectable Bundle Space.");
}

void ompl::multilevel::BundleSpaceComponent_None::liftState(const ompl::base::State *, const ompl::base::State *,
                                                           ompl::base::State *) const
{
    OMPL_WARN("Trying to lift States with non-projectable Bundle Space.");
}

ompl::base::StateSpacePtr ompl::multilevel::BundleSpaceComponent_None::computeFiberSpace()
{
    return nullptr;
}
