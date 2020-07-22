#include <ompl/multilevel/datastructures/components/XRN_X.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/util/Exception.h>

ompl::multilevel::BundleSpaceComponent_XRN_X::BundleSpaceComponent_XRN_X(base::StateSpacePtr BundleSpace,
                                                                        base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
}

void ompl::multilevel::BundleSpaceComponent_XRN_X::projectFiber(const ompl::base::State *xBundle,
                                                               ompl::base::State *xFiber) const
{
    const base::RealVectorStateSpace::StateType *xBundle_RN =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);
    base::RealVectorStateSpace::StateType *xFiber_RN = xFiber->as<base::RealVectorStateSpace::StateType>();

    for (unsigned int k = 0; k < getFiberDimension(); k++)
    {
        xFiber_RN->values[k] = xBundle_RN->values[k];
    }
}

ompl::base::StateSpacePtr ompl::multilevel::BundleSpaceComponent_XRN_X::computeFiberSpace()
{
    base::CompoundStateSpace *Bundle_compound = BundleSpace_->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();

    const base::RealVectorStateSpace *Bundle_RN = Bundle_decomposed.at(1)->as<base::RealVectorStateSpace>();

    unsigned int N = Bundle_RN->getDimension();

    base::StateSpacePtr RN = std::make_shared<base::RealVectorStateSpace>(N);
    std::static_pointer_cast<base::RealVectorStateSpace>(RN)->setBounds(Bundle_RN->getBounds());
    return RN;
}
