#include <ompl/multilevel/datastructures/components/XRN_X_SE3.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

ompl::multilevel::BundleSpaceComponent_SE3RN_SE3::BundleSpaceComponent_SE3RN_SE3(base::StateSpacePtr BundleSpace,
                                                                                base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
}

void ompl::multilevel::BundleSpaceComponent_SE3RN_SE3::projectBase(const ompl::base::State *xBundle,
                                                                  ompl::base::State *xBase) const
{
    const base::SE3StateSpace::StateType *xBundle_SE3 =
        xBundle->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
    const base::SO3StateSpace::StateType *xBundle_SO3 = &xBundle_SE3->rotation();

    base::SE3StateSpace::StateType *xBase_SE3 = xBase->as<base::SE3StateSpace::StateType>();
    base::SO3StateSpace::StateType *xBase_SO3 = &xBase_SE3->rotation();

    xBase_SE3->setXYZ(xBundle_SE3->getX(), xBundle_SE3->getY(), xBundle_SE3->getZ());
    xBase_SO3->x = xBundle_SO3->x;
    xBase_SO3->y = xBundle_SO3->y;
    xBase_SO3->z = xBundle_SO3->z;
    xBase_SO3->w = xBundle_SO3->w;
}

void ompl::multilevel::BundleSpaceComponent_SE3RN_SE3::liftState(const ompl::base::State *xBase,
                                                                const ompl::base::State *xFiber,
                                                                ompl::base::State *xBundle) const
{
    base::SE3StateSpace::StateType *xBundle_SE3 =
        xBundle->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
    base::SO3StateSpace::StateType *xBundle_SO3 = &xBundle_SE3->rotation();
    base::RealVectorStateSpace::StateType *xBundle_RN =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    const base::SE3StateSpace::StateType *xBase_SE3 = xBase->as<base::SE3StateSpace::StateType>();
    const base::SO3StateSpace::StateType *xBase_SO3 = &xBase_SE3->rotation();

    const base::RealVectorStateSpace::StateType *xFiber_RN = xFiber->as<base::RealVectorStateSpace::StateType>();

    xBundle_SE3->setXYZ(xBase_SE3->getX(), xBase_SE3->getY(), xBase_SE3->getZ());
    xBundle_SO3->x = xBase_SO3->x;
    xBundle_SO3->y = xBase_SO3->y;
    xBundle_SO3->z = xBase_SO3->z;
    xBundle_SO3->w = xBase_SO3->w;

    for (unsigned int k = 0; k < getFiberDimension(); k++)
    {
        xBundle_RN->values[k] = xFiber_RN->values[k];
    }
}
