#include <ompl/geometric/planners/quotientspace/datastructures/components/XRN_XRM_SO3.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/util/Exception.h>

ompl::geometric::BundleSpaceComponent_SO3RN_SO3RM::BundleSpaceComponent_SO3RN_SO3RM(
    base::StateSpacePtr BundleSpace,
    base::StateSpacePtr BaseSpace):
  BaseT(BundleSpace, BaseSpace)
{
}

void ompl::geometric::BundleSpaceComponent_SO3RN_SO3RM::projectBase(
    const ompl::base::State *xBundle,
    ompl::base::State *xBase) const
{
    const base::SO3StateSpace::StateType *xBundle_SO3 =
        xBundle->as<base::CompoundState>()->as<base::SO3StateSpace::StateType>(0);
    const base::RealVectorStateSpace::StateType *xBundle_RN =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    base::SO3StateSpace::StateType *xBase_SO3 =
        xBase->as<base::CompoundState>()->as<base::SO3StateSpace::StateType>(0);
    base::RealVectorStateSpace::StateType *xBase_RM =
        xBase->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    xBase_SO3->x = xBundle_SO3->x;
    xBase_SO3->y = xBundle_SO3->y;
    xBase_SO3->z = xBundle_SO3->z;
    xBase_SO3->w = xBundle_SO3->w;

    for (unsigned int k = 0; k < getBaseDimension() - 1; k++)
    {
        xBase_RM->values[k] = xBundle_RN->values[k];
    }
}

void ompl::geometric::BundleSpaceComponent_SO3RN_SO3RM::liftState(
    const ompl::base::State *xBase, 
    const ompl::base::State *xFiber, 
    ompl::base::State *xBundle) const
{
    base::SO3StateSpace::StateType *xBundle_SO3 =
        xBundle->as<base::CompoundState>()->as<base::SO3StateSpace::StateType>(0);
    base::RealVectorStateSpace::StateType *xBundle_RN =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    const base::SO3StateSpace::StateType *xBase_SO3 =
        xBase->as<base::CompoundState>()->as<base::SO3StateSpace::StateType>(0);
    const base::RealVectorStateSpace::StateType *xBase_RM =
        xBase->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    const base::RealVectorStateSpace::StateType *xFiber_RJ = xFiber->as<base::RealVectorStateSpace::StateType>();

    xBundle_SO3->x = xBase_SO3->x;
    xBundle_SO3->y = xBase_SO3->y;
    xBundle_SO3->z = xBase_SO3->z;
    xBundle_SO3->w = xBase_SO3->w;

    unsigned int M = getDimension() - getFiberDimension() - 1;
    unsigned int N = getFiberDimension();

    for (unsigned int k = 0; k < M; k++)
    {
        xBundle_RN->values[k] = xBase_RM->values[k];
    }
    for (unsigned int k = M; k < M + N; k++)
    {
        xBundle_RN->values[k] = xFiber_RJ->values[k - M];
    }
}
