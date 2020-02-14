#include <ompl/geometric/planners/quotientspace/datastructures/components/XRN_XRM_SE2.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/util/Exception.h>

ompl::geometric::BundleSpaceComponent_SE2RN_SE2RM::BundleSpaceComponent_SE2RN_SE2RM(
    base::StateSpacePtr BundleSpace,
    base::StateSpacePtr BaseSpace):
  BaseT(BundleSpace, BaseSpace)
{
}

void ompl::geometric::BundleSpaceComponent_SE2RN_SE2RM::projectBase(
    const ompl::base::State *xBundle,
    ompl::base::State *xBase) const
{
    const base::SE2StateSpace::StateType *xBundle_SE2 =
        xBundle->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
    const base::RealVectorStateSpace::StateType *xBundle_RN =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    base::SE2StateSpace::StateType *xBase_SE2 =
        xBase->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
    base::RealVectorStateSpace::StateType *xBase_RN =
        xBase->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    xBase_SE2->setX(xBundle_SE2->getX());
    xBase_SE2->setY(xBundle_SE2->getY());
    xBase_SE2->setYaw(xBundle_SE2->getYaw());

    for (unsigned int k = 0; k < getBaseDimension() - 3; k++)
    {
        xBase_RN->values[k] = xBundle_RN->values[k];
    }
}

void ompl::geometric::BundleSpaceComponent_SE2RN_SE2RM::mergeStates(
    const ompl::base::State *xBase, 
    const ompl::base::State *xFiber, 
    ompl::base::State *xBundle) const
{
     base::SE2StateSpace::StateType *xBundle_SE2 =
         xBundle->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
     base::RealVectorStateSpace::StateType *xBundle_RN =
         xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

     const base::SE2StateSpace::StateType *xBase_SE2 =
         xBase->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
     const base::RealVectorStateSpace::StateType *xBase_RM =
         xBase->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

     const base::RealVectorStateSpace::StateType *xFiber_RJ = xFiber->as<base::RealVectorStateSpace::StateType>();

     xBundle_SE2->setX(xBase_SE2->getX());
     xBundle_SE2->setY(xBase_SE2->getY());
     xBundle_SE2->setYaw(xBase_SE2->getYaw());

     //[X Y YAW] [1...M-1][M...N-1]
     // SE2               RN
     unsigned int M = getDimension() - getFiberDimension() - 3;
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
