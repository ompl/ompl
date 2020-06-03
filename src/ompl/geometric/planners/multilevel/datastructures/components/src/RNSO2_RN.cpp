#include <ompl/geometric/planners/multilevel/datastructures/components/RNSO2_RN.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

ompl::geometric::BundleSpaceComponent_RNSO2_RN::BundleSpaceComponent_RNSO2_RN(base::StateSpacePtr BundleSpace,
                                                                              base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
}

void ompl::geometric::BundleSpaceComponent_RNSO2_RN::projectFiber(const ompl::base::State *xBundle,
                                                                  ompl::base::State *xFiber) const
{
    const base::SO2StateSpace::StateType *xBundle_SO2 =
        xBundle->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(1);

    base::SO2StateSpace::StateType *xFiber_SO2 = xFiber->as<base::SO2StateSpace::StateType>();

    xFiber_SO2->value = xBundle_SO2->value;
}

void ompl::geometric::BundleSpaceComponent_RNSO2_RN::projectBase(const ompl::base::State *xBundle,
                                                                 ompl::base::State *xBase) const
{
    const base::RealVectorStateSpace::StateType *xBundle_R3 =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(0);
    base::RealVectorStateSpace::StateType *xBase_R3 = xBase->as<base::RealVectorStateSpace::StateType>();

    for (unsigned int k = 0; k < 3; k++)
    {
        xBase_R3->values[k] = xBundle_R3->values[k];
    }
}

void ompl::geometric::BundleSpaceComponent_RNSO2_RN::liftState(const ompl::base::State *xBase,
                                                               const ompl::base::State *xFiber,
                                                               ompl::base::State *xBundle) const
{
    base::RealVectorStateSpace::StateType *xBundle_R3 =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(0);
    base::SO2StateSpace::StateType *xBundle_SO2 =
        xBundle->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(1);

    const base::SO2StateSpace::StateType *xFiber_SO2 = xFiber->as<base::SO2StateSpace::StateType>();

    const base::RealVectorStateSpace::StateType *xBase_R3 = xBase->as<base::RealVectorStateSpace::StateType>();

    for (unsigned int k = 0; k < 3; k++)
    {
        xBundle_R3->values[k] = xBase_R3->values[k];
    }
    xBundle_SO2->value = xFiber_SO2->value;
}

ompl::base::StateSpacePtr ompl::geometric::BundleSpaceComponent_RNSO2_RN::computeFiberSpace()
{
    unsigned int N = BundleSpace_->getDimension();
    unsigned int Y = BaseSpace_->getDimension();
    if (Y != (N - 1))
    {
        OMPL_ERROR("Assumed input is SO(2)xRN -> RN, but got %d -> %d dimensions.", N, Y);
        throw "Invalid Dimensionality";
    }
    return std::make_shared<base::SO2StateSpace>();
}
