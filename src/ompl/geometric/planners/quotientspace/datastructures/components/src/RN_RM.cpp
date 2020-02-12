#include <ompl/geometric/planners/quotientspace/datastructures/components/RN_RM.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

ompl::geometric::BundleSpaceComponent_RN_RM::BundleSpaceComponent_RN_RM(
    base::StateSpacePtr BundleSpace,
    base::StateSpacePtr BaseSpace):
  BaseT(BundleSpace, BaseSpace)
{
}

void ompl::geometric::BundleSpaceComponent_RN_RM::projectFiber(
    const ompl::base::State *xBundle,
    ompl::base::State *xFiber) const
{
    const base::RealVectorStateSpace::StateType *xBundle_RN = 
      xBundle->as<base::RealVectorStateSpace::StateType>();
    base::RealVectorStateSpace::StateType *xFiber_RM = 
      xFiber->as<base::RealVectorStateSpace::StateType>();

    for (unsigned int k = getBaseDimension(); k < getDimension(); k++)
    {
        xFiber_RM->values[k - getBaseDimension()] = xBundle_RN->values[k];
    }
}


void ompl::geometric::BundleSpaceComponent_RN_RM::projectBase(
    const ompl::base::State *xBundle,
    ompl::base::State *xBase) const
{
    const base::RealVectorStateSpace::StateType *xBundle_RN = xBundle->as<base::RealVectorStateSpace::StateType>();
    base::RealVectorStateSpace::StateType *xBase_RM = xBase->as<base::RealVectorStateSpace::StateType>();

    for (unsigned int k = 0; k < getBaseDimension(); k++)
    {
        xBase_RM->values[k] = xBundle_RN->values[k];
    }
}


void ompl::geometric::BundleSpaceComponent_RN_RM::mergeStates(
    const ompl::base::State *xBase, 
    const ompl::base::State *xFiber, 
    ompl::base::State *xBundle) const
{
     base::RealVectorStateSpace::StateType *xBundle_RN = xBundle->as<base::RealVectorStateSpace::StateType>();
     const base::RealVectorStateSpace::StateType *xBase_RM = xBase->as<base::RealVectorStateSpace::StateType>();
     const base::RealVectorStateSpace::StateType *xFiber_RJ = xFiber->as<base::RealVectorStateSpace::StateType>();

     for (unsigned int k = 0; k < getBaseDimension(); k++)
     {
         xBundle_RN->values[k] = xBase_RM->values[k];
     }
     for (unsigned int k = getBaseDimension(); k < getDimension(); k++)
     {
         xBundle_RN->values[k] = xFiber_RJ->values[k - getBaseDimension()];
     }
}

ompl::base::StateSpacePtr ompl::geometric::BundleSpaceComponent_RN_RM::computeFiberSpace()
{
    unsigned int N1 = getDimension();
    unsigned int N0 = getBaseDimension();
    unsigned int NX = N1 - N0;
    base::StateSpacePtr FiberSpace = std::make_shared<base::RealVectorStateSpace>(NX);
    base::RealVectorBounds Bundle_bounds = 
      std::static_pointer_cast<base::RealVectorStateSpace>(BundleSpace_)->getBounds();

    std::vector<double> low;
    low.resize(NX);
    std::vector<double> high;
    high.resize(NX);
    base::RealVectorBounds Fiber_bounds(NX);
    for (unsigned int k = 0; k < NX; k++)
    {
        Fiber_bounds.setLow(k, Bundle_bounds.low.at(k + N0));
        Fiber_bounds.setHigh(k, Bundle_bounds.high.at(k + N0));
    }
    std::static_pointer_cast<base::RealVectorStateSpace>(FiberSpace)->setBounds(Fiber_bounds);
    return FiberSpace;
}
