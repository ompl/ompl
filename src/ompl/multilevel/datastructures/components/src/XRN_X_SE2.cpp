#include <ompl/multilevel/datastructures/components/XRN_X_SE2.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/util/Exception.h>

ompl::multilevel::BundleSpaceComponent_SE2RN_SE2::BundleSpaceComponent_SE2RN_SE2(base::StateSpacePtr BundleSpace,
                                                                                base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
}

void ompl::multilevel::BundleSpaceComponent_SE2RN_SE2::projectBase(const ompl::base::State *xBundle,
                                                                  ompl::base::State *xBase) const
{
    const base::SE2StateSpace::StateType *xBundle_SE2 =
        xBundle->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
    base::SE2StateSpace::StateType *xBase_SE2 = xBase->as<base::SE2StateSpace::StateType>();
    xBase_SE2->setX(xBundle_SE2->getX());
    xBase_SE2->setY(xBundle_SE2->getY());
    xBase_SE2->setYaw(xBundle_SE2->getYaw());
}

void ompl::multilevel::BundleSpaceComponent_SE2RN_SE2::liftState(const ompl::base::State *xBase,
                                                                const ompl::base::State *xFiber,
                                                                ompl::base::State *xBundle) const
{
    base::SE2StateSpace::StateType *xBundle_SE2 =
        xBundle->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
    base::RealVectorStateSpace::StateType *xBundle_RN =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    const base::SE2StateSpace::StateType *xBase_SE2 = xBase->as<base::SE2StateSpace::StateType>();
    const base::RealVectorStateSpace::StateType *xFiber_RN = xFiber->as<base::RealVectorStateSpace::StateType>();

    xBundle_SE2->setX(xBase_SE2->getX());
    xBundle_SE2->setY(xBase_SE2->getY());
    xBundle_SE2->setYaw(xBase_SE2->getYaw());

    for (unsigned int k = 0; k < getFiberDimension(); k++)
    {
        xBundle_RN->values[k] = xFiber_RN->values[k];
    }
}
