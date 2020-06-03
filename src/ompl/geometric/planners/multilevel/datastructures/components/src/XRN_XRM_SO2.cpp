#include <ompl/geometric/planners/multilevel/datastructures/components/XRN_XRM_SO2.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/util/Exception.h>

ompl::geometric::BundleSpaceComponent_SO2RN_SO2RM::BundleSpaceComponent_SO2RN_SO2RM(base::StateSpacePtr BundleSpace,
                                                                                    base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
}

void ompl::geometric::BundleSpaceComponent_SO2RN_SO2RM::projectBase(const ompl::base::State *xBundle,
                                                                    ompl::base::State *xBase) const
{
    const base::SO2StateSpace::StateType *xBundle_SO2 =
        xBundle->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
    const base::RealVectorStateSpace::StateType *xBundle_RN =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    base::SO2StateSpace::StateType *xBase_SO2 = xBase->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
    base::RealVectorStateSpace::StateType *xBase_RM =
        xBase->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    xBase_SO2->value = xBundle_SO2->value;

    for (unsigned int k = 0; k < getBaseDimension() - 1; k++)
    {
        xBase_RM->values[k] = xBundle_RN->values[k];
    }
}

void ompl::geometric::BundleSpaceComponent_SO2RN_SO2RM::liftState(const ompl::base::State *xBase,
                                                                  const ompl::base::State *xFiber,
                                                                  ompl::base::State *xBundle) const
{
    base::SO2StateSpace::StateType *xBundle_SO2 =
        xBundle->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
    base::RealVectorStateSpace::StateType *xBundle_RN =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    const base::SO2StateSpace::StateType *xBase_SO2 =
        xBase->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
    const base::RealVectorStateSpace::StateType *xBase_RM =
        xBase->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    const base::RealVectorStateSpace::StateType *xFiber_RJ = xFiber->as<base::RealVectorStateSpace::StateType>();

    xBundle_SO2->value = xBase_SO2->value;

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
