#include <ompl/geometric/planners/quotientspace/datastructures/components/XRN_X_SO3.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/util/Exception.h>

ompl::geometric::BundleSpaceComponent_SO3RN_SO3::BundleSpaceComponent_SO3RN_SO3(
    base::StateSpacePtr BundleSpace,
    base::StateSpacePtr BaseSpace):
  BaseT(BundleSpace, BaseSpace)
{
}

void ompl::geometric::BundleSpaceComponent_SO3RN_SO3::projectBase(
    const ompl::base::State *xBundle,
    ompl::base::State *xBase) const
{
    const base::SO3StateSpace::StateType *xBundle_SO3 =
        xBundle->as<base::CompoundState>()->as<base::SO3StateSpace::StateType>(0);
    base::SO3StateSpace::StateType *xBase_SO3 = xBase->as<base::SO3StateSpace::StateType>();

    xBase_SO3->x = xBundle_SO3->x;
    xBase_SO3->y = xBundle_SO3->y;
    xBase_SO3->z = xBundle_SO3->z;
    xBase_SO3->w = xBundle_SO3->w;
}

void ompl::geometric::BundleSpaceComponent_SO3RN_SO3::liftState(
    const ompl::base::State *xBase, 
    const ompl::base::State *xFiber, 
    ompl::base::State *xBundle) const
{
     base::SO3StateSpace::StateType *xBundle_SO3 =
         xBundle->as<base::CompoundState>()->as<base::SO3StateSpace::StateType>(0);
     base::RealVectorStateSpace::StateType *xBundle_RN =
         xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

     const base::SO3StateSpace::StateType *xBase_SO3 = xBase->as<base::SO3StateSpace::StateType>();
     const base::RealVectorStateSpace::StateType *xFiber_RN = xFiber->as<base::RealVectorStateSpace::StateType>();

     xBundle_SO3->x = xBase_SO3->x;
     xBundle_SO3->y = xBase_SO3->y;
     xBundle_SO3->z = xBase_SO3->z;
     xBundle_SO3->w = xBase_SO3->w;

     for (unsigned int k = 0; k < getFiberDimension(); k++)
     {
         xBundle_RN->values[k] = xFiber_RN->values[k];
     }
}
