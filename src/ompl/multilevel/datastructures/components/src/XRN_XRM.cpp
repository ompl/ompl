#include <ompl/multilevel/datastructures/components/XRN_XRM.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/util/Exception.h>

ompl::multilevel::BundleSpaceComponent_XRN_XRM::BundleSpaceComponent_XRN_XRM(base::StateSpacePtr BundleSpace,
                                                                            base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
}
void ompl::multilevel::BundleSpaceComponent_XRN_XRM::projectFiber(const ompl::base::State *xBundle,
                                                                 ompl::base::State *xFiber) const
{
    const base::RealVectorStateSpace::StateType *xBundle_RN =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    const base::RealVectorStateSpace::StateType *xFiber_RJ = xFiber->as<base::RealVectorStateSpace::StateType>();

    unsigned int Noffset = getBaseDimension() - dimensionBaseFirstSubspace;
    for (unsigned int k = 0; k < getFiberDimension(); k++)
    {
        xFiber_RJ->values[k] = xBundle_RN->values[k + Noffset];
    }
}

ompl::base::StateSpacePtr ompl::multilevel::BundleSpaceComponent_XRN_XRM::computeFiberSpace()
{
    base::CompoundStateSpace *Bundle_compound = BundleSpace_->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();

    base::CompoundStateSpace *Base_compound = BaseSpace_->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();

    dimensionBaseFirstSubspace = Base_decomposed.at(0)->getDimension();

    unsigned int Nbundle = Bundle_decomposed.at(1)->getDimension();
    unsigned int Nbase = Base_decomposed.at(1)->getDimension();
    unsigned int Nfiber = Nbundle - Nbase;

    base::StateSpacePtr Fiber_RN(new base::RealVectorStateSpace(Nfiber));

    const base::RealVectorStateSpace *Bundle_RN = Bundle_decomposed.at(1)->as<base::RealVectorStateSpace>();
    base::RealVectorBounds Bundle_bounds = Bundle_RN->getBounds();
    std::vector<double> low;
    low.resize(Nfiber);
    std::vector<double> high;
    high.resize(Nfiber);
    base::RealVectorBounds Fiber_bounds(Nfiber);
    for (unsigned int k = 0; k < Nfiber; k++)
    {
        Fiber_bounds.setLow(k, Bundle_bounds.low.at(k + Nbase));
        Fiber_bounds.setHigh(k, Bundle_bounds.high.at(k + Nbase));
    }
    std::static_pointer_cast<base::RealVectorStateSpace>(Fiber_RN)->setBounds(Fiber_bounds);

    return Fiber_RN;
}
