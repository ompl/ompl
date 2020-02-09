#include <ompl/geometric/planners/quotientspace/datastructures/components/SO2RN_SO2.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/util/Exception.h>

ompl::geometric::BundleSpaceComponent_SO2RN_SO2::BundleSpaceComponent_SO2RN_SO2(
    base::StateSpacePtr BundleSpace,
    base::StateSpacePtr BaseSpace):
  BaseT(BundleSpace, BaseSpace)
{
}

void ompl::geometric::BundleSpaceComponent_SO2RN_SO2::projectFiber(
    const ompl::base::State *xBundle,
    ompl::base::State *xFiber) const
{
    const base::RealVectorStateSpace::StateType *xBundle_RN =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);
    base::RealVectorStateSpace::StateType *xFiber_RN = 
        xFiber->as<base::RealVectorStateSpace::StateType>();

    for (unsigned int k = 0; k < getFiberDimension(); k++)
    {
        xFiber_RN->values[k] = xBundle_RN->values[k];
    }
}

void ompl::geometric::BundleSpaceComponent_SO2RN_SO2::projectBase(
    const ompl::base::State *xBundle,
    ompl::base::State *xBase) const
{
    const base::SO2StateSpace::StateType *xBundle_SO2 =
        xBundle->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
    base::SO2StateSpace::StateType *xBase_SO2 = xBase->as<base::SO2StateSpace::StateType>();

    xBase_SO2->value = xBundle_SO2->value;
}

void ompl::geometric::BundleSpaceComponent_SO2RN_SO2::mergeStates(
    const ompl::base::State *xBase, 
    const ompl::base::State *xFiber, 
    ompl::base::State *xBundle) const
{
     base::SO2StateSpace::StateType *xBundle_SO2 =
         xBundle->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
     base::RealVectorStateSpace::StateType *xBundle_RN =
         xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

     const base::SO2StateSpace::StateType *xBase_SO2 = xBase->as<base::SO2StateSpace::StateType>();
     const base::RealVectorStateSpace::StateType *xFiber_RN = xFiber->as<base::RealVectorStateSpace::StateType>();

     xBundle_SO2->value = xBase_SO2->value;

     for (unsigned int k = 0; k < getFiberDimension(); k++)
     {
         xBundle_RN->values[k] = xFiber_RN->values[k];
     }
}

ompl::base::StateSpacePtr ompl::geometric::BundleSpaceComponent_SO2RN_SO2::computeFiberSpace()
{
    base::CompoundStateSpace *Bundle_compound = BundleSpace_->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();

    const base::RealVectorStateSpace *Bundle_RN = 
      Bundle_decomposed.at(1)->as<base::RealVectorStateSpace>();
    unsigned int NX = Bundle_RN->getDimension();

    base::StateSpacePtr RN = std::make_shared<base::RealVectorStateSpace>(NX);
    std::static_pointer_cast<base::RealVectorStateSpace>(RN)->setBounds(Bundle_RN->getBounds());
    return RN;
}
