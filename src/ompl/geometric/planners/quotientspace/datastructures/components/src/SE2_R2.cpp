#include <ompl/geometric/planners/quotientspace/datastructures/components/SE2_R2.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

ompl::geometric::BundleSpaceComponent_SE2_R2::BundleSpaceComponent_SE2_R2(
    base::StateSpacePtr BundleSpace,
    base::StateSpacePtr BaseSpace):
  BaseT(BundleSpace, BaseSpace)
{
}

void ompl::geometric::BundleSpaceComponent_SE2_R2::projectFiber(
    const ompl::base::State *xBundle,
    ompl::base::State *xFiber) const
{
  const base::SE2StateSpace::StateType *xBundle_SE2 = xBundle->as<base::SE2StateSpace::StateType>();
  base::SO2StateSpace::StateType *xFiber_SO2 = xFiber->as<base::SO2StateSpace::StateType>();
  xFiber_SO2->value = xBundle_SE2->getYaw();
}


void ompl::geometric::BundleSpaceComponent_SE2_R2::projectBase(
    const ompl::base::State *xBundle,
    ompl::base::State *xBase) const
{
   const base::SE2StateSpace::StateType *xBundle_SE2 = xBundle->as<base::SE2StateSpace::StateType>();
   base::RealVectorStateSpace::StateType *xBase_R2 = xBase->as<base::RealVectorStateSpace::StateType>();
   xBase_R2->values[0] = xBundle_SE2->getX();
   xBase_R2->values[1] = xBundle_SE2->getY();
}


void ompl::geometric::BundleSpaceComponent_SE2_R2::liftState(
    const ompl::base::State *xBase, 
    const ompl::base::State *xFiber, 
    ompl::base::State *xBundle) const
{
  base::SE2StateSpace::StateType *xBundle_SE2 = xBundle->as<base::SE2StateSpace::StateType>();
  const base::RealVectorStateSpace::StateType *xBase_R2 = xBase->as<base::RealVectorStateSpace::StateType>();
  const base::SO2StateSpace::StateType *xFiber_SO2 = xFiber->as<base::SO2StateSpace::StateType>();

  xBundle_SE2->setXY(xBase_R2->values[0], xBase_R2->values[1]);
  xBundle_SE2->setYaw(xFiber_SO2->value);
}

ompl::base::StateSpacePtr ompl::geometric::BundleSpaceComponent_SE2_R2::computeFiberSpace()
{
  unsigned int N = BundleSpace_->getDimension();
  unsigned int Y = BaseSpace_->getDimension();
  if(N != 3 && Y != 2)
  {
    OMPL_ERROR("Assumed input is SE(2) -> R2, but got %d -> %d dimensions.", N, Y);
    throw "Invalid Dimensionality";
  }
  return std::make_shared<base::SO2StateSpace>();
}
