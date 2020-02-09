#include <ompl/geometric/planners/quotientspace/datastructures/components/SE2RN_SE2.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/util/Exception.h>

ompl::geometric::BundleSpaceComponent_SE2RN_SE2::BundleSpaceComponent_SE2RN_SE2(
    base::StateSpacePtr BundleSpace,
    base::StateSpacePtr BaseSpace):
  BaseT(BundleSpace, BaseSpace)
{
}

void ompl::geometric::BundleSpaceComponent_SE2RN_SE2::projectFiber(
    const ompl::base::State *xBundle,
    ompl::base::State *xFiber) const
{
    const base::RealVectorStateSpace::StateType *xBundle_RN =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);
    base::RealVectorStateSpace::StateType *xFiber_RN = xFiber->as<base::RealVectorStateSpace::StateType>();
    for (unsigned int k = 0; k < getFiberDimension(); k++)
    {
        xFiber_RN->values[k] = xBundle_RN->values[k];
    }
}

void ompl::geometric::BundleSpaceComponent_SE2RN_SE2::projectBase(
    const ompl::base::State *xBundle,
    ompl::base::State *xBase) const
{
    const base::SE2StateSpace::StateType *xBundle_SE2 =
        xBundle->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
    base::SE2StateSpace::StateType *xBase_SE2 = xBase->as<base::SE2StateSpace::StateType>();
    xBase_SE2->setX(xBundle_SE2->getX());
    xBase_SE2->setY(xBundle_SE2->getY());
    xBase_SE2->setYaw(xBundle_SE2->getYaw());
}

void ompl::geometric::BundleSpaceComponent_SE2RN_SE2::mergeStates(
    const ompl::base::State *xBase, 
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

ompl::base::StateSpacePtr ompl::geometric::BundleSpaceComponent_SE2RN_SE2::computeFiberSpace()
{
    unsigned int N = BundleSpace_->getDimension();
    unsigned int Y = BaseSpace_->getDimension();
    if(N > 3 && Y != 3)
    {
      OMPL_ERROR("Assumed input is SE(2)xRN -> SE2, but got %d -> %d dimensions.", N, Y);
      throw Exception("Invalid Dimensionality");
    }

    base::CompoundStateSpace *Bundle_compound = BundleSpace_->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();

    const base::RealVectorStateSpace *Bundle_RN = 
      Bundle_decomposed.at(1)->as<base::RealVectorStateSpace>();
    unsigned int NX = Bundle_RN->getDimension();

    base::StateSpacePtr RN = std::make_shared<base::RealVectorStateSpace>(NX);
    std::static_pointer_cast<base::RealVectorStateSpace>(RN)->setBounds(
        Bundle_RN->getBounds());
    return RN;
}
