#include <ompl/geometric/planners/quotientspace/datastructures/components/SO2RN_SO2RM.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/util/Exception.h>

ompl::geometric::BundleSpaceComponent_SO2RN_SO2RM::BundleSpaceComponent_SO2RN_SO2RM(
    base::StateSpacePtr BundleSpace,
    base::StateSpacePtr BaseSpace):
  BaseT(BundleSpace, BaseSpace)
{
}

void ompl::geometric::BundleSpaceComponent_SO2RN_SO2RM::projectFiber(
    const ompl::base::State *xBundle,
    ompl::base::State *xFiber) const
{
    const base::RealVectorStateSpace::StateType *xBundle_RN =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    const base::RealVectorStateSpace::StateType *xFiber_RJ = xFiber->as<base::RealVectorStateSpace::StateType>();

    unsigned int N = getDimension() - getFiberDimension() - 6;
    for (unsigned int k = N; k < getDimension() - 6; k++)
    {
        xFiber_RJ->values[k - N] = xBundle_RN->values[k];
    }
}

void ompl::geometric::BundleSpaceComponent_SO2RN_SO2RM::projectBase(
    const ompl::base::State *xBundle,
    ompl::base::State *xBase) const
{
    const base::SO2StateSpace::StateType *xBundle_SO2 =
        xBundle->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
    const base::RealVectorStateSpace::StateType *xBundle_RN =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    base::SO2StateSpace::StateType *xBase_SO2 =
        xBase->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
    base::RealVectorStateSpace::StateType *xBase_RM =
        xBase->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    xBase_SO2->value = xBundle_SO2->value;

    for (unsigned int k = 0; k < getBaseDimension() - 1; k++)
    {
        xBase_RM->values[k] = xBundle_RN->values[k];
    }
}

void ompl::geometric::BundleSpaceComponent_SO2RN_SO2RM::mergeStates(
    const ompl::base::State *xBase, 
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

ompl::base::StateSpacePtr ompl::geometric::BundleSpaceComponent_SO2RN_SO2RM::computeFiberSpace()
{
    base::CompoundStateSpace *Bundle_compound = BundleSpace_->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();

    base::CompoundStateSpace *Base_compound = BaseSpace_->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();

    unsigned int N = Bundle_decomposed.at(1)->getDimension();
    unsigned int M = Base_decomposed.at(1)->getDimension();
    unsigned int NX = N - M;
    base::StateSpacePtr RN(new base::RealVectorStateSpace(NX));

    const base::RealVectorStateSpace *Bundle_RN = Bundle_decomposed.at(1)->as<base::RealVectorStateSpace>();
    base::RealVectorBounds Bundle_bounds = Bundle_RN->getBounds();
    std::vector<double> low;
    low.resize(NX);
    std::vector<double> high;
    high.resize(NX);
    base::RealVectorBounds Fiber_bounds(NX);
    for (unsigned int k = 0; k < NX; k++)
    {
        Fiber_bounds.setLow(k, Bundle_bounds.low.at(k + M));
        Fiber_bounds.setHigh(k, Bundle_bounds.high.at(k + M));
    }
    std::static_pointer_cast<base::RealVectorStateSpace>(RN)->setBounds(Fiber_bounds);
    return RN;
}
