#include "../SE2_R2.h"

void BundleSubspaceSE2_R2::projectFiber(
    const ompl::base::State *xBundle,
    ompl::base::State *xFiber) const
{
  const base::SE2StateSpace::StateType *xBundle_SE2 = xBundle->as<base::SE2StateSpace::StateType>();
  base::SO2StateSpace::StateType *xFiber_SO2 = xFiber->as<base::SO2StateSpace::StateType>();
  xFiber_SO2->value = xBundle_SE2->getYaw();
  break;
}


void BundleSubspaceSE2_R2::projectBase(
    const ompl::base::State *xBundle,
    ompl::base::State *xBase) const
{
   const base::SE2StateSpace::StateType *xBundle_SE2 = xBundle->as<base::SE2StateSpace::StateType>();
   base::RealVectorStateSpace::StateType *xBase_R2 = xBase->as<base::RealVectorStateSpace::StateType>();
   xBase_R2->values[0] = xBundle_SE2->getX();
   xBase_R2->values[1] = xBundle_SE2->getY();
   break;
}


void BundleSubspaceSE2_R2::mergeStates(
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

ompl::base::StateSpacePtr BundleSubspaceSE2_R2::getFiberSpace()
{

  unsigned int N = BundleSpace_->getDimension();
  if(N != 3)
  {
    OMPL_ERROR("Assumed input is SE(2), but got %d dimensions.", N);
    throw "Invalid Dimensionality";
  }
  return std::make_shared<base::SO2StateSpace>();
}
std::string getTypeAsString()
{
  return "SE2 -> R2";
}
