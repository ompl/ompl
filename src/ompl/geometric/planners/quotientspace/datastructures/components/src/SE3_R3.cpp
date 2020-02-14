#include <ompl/geometric/planners/quotientspace/datastructures/components/SE3_R3.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

ompl::geometric::BundleSpaceComponent_SE3_R3::BundleSpaceComponent_SE3_R3(
    base::StateSpacePtr BundleSpace,
    base::StateSpacePtr BaseSpace):
  BaseT(BundleSpace, BaseSpace)
{
}

void ompl::geometric::BundleSpaceComponent_SE3_R3::projectFiber(
    const ompl::base::State *xBundle,
    ompl::base::State *xFiber) const
{
  const base::SE3StateSpace::StateType *xBundle_SE3 = xBundle->as<base::SE3StateSpace::StateType>();
  const base::SO3StateSpace::StateType *xBundle_SO3 = &xBundle_SE3->rotation();
  base::SO3StateSpace::StateType *xFiber_SO3 = xFiber->as<base::SO3StateSpace::StateType>();

  xFiber_SO3->x = xBundle_SO3->x;
  xFiber_SO3->y = xBundle_SO3->y;
  xFiber_SO3->z = xBundle_SO3->z;
  xFiber_SO3->w = xBundle_SO3->w;
}


void ompl::geometric::BundleSpaceComponent_SE3_R3::projectBase(
    const ompl::base::State *xBundle,
    ompl::base::State *xBase) const
{
   const base::SE3StateSpace::StateType *xBundle_SE3 = xBundle->as<base::SE3StateSpace::StateType>();
   base::RealVectorStateSpace::StateType *xBase_R3 = xBase->as<base::RealVectorStateSpace::StateType>();
   xBase_R3->values[0] = xBundle_SE3->getX();
   xBase_R3->values[1] = xBundle_SE3->getY();
   xBase_R3->values[2] = xBundle_SE3->getZ();
}


void ompl::geometric::BundleSpaceComponent_SE3_R3::mergeStates(
    const ompl::base::State *xBase, 
    const ompl::base::State *xFiber, 
    ompl::base::State *xBundle) const
{
    base::SE3StateSpace::StateType *xBundle_SE3 = xBundle->as<base::SE3StateSpace::StateType>();
    base::SO3StateSpace::StateType *xBundle_SO3 = &xBundle_SE3->rotation();

    const base::RealVectorStateSpace::StateType *xBase_R3 = xBase->as<base::RealVectorStateSpace::StateType>();
    const base::SO3StateSpace::StateType *xFiber_SO3 = xFiber->as<base::SO3StateSpace::StateType>();

    xBundle_SE3->setXYZ(xBase_R3->values[0], xBase_R3->values[1], xBase_R3->values[2]);

    xBundle_SO3->x = xFiber_SO3->x;
    xBundle_SO3->y = xFiber_SO3->y;
    xBundle_SO3->z = xFiber_SO3->z;
    xBundle_SO3->w = xFiber_SO3->w;
}

ompl::base::StateSpacePtr ompl::geometric::BundleSpaceComponent_SE3_R3::computeFiberSpace()
{
  unsigned int N = BundleSpace_->getDimension();
  unsigned int Y = BaseSpace_->getDimension();
  if(N != 6 && Y != 3)
  {
    OMPL_ERROR("Assumed input is SE(3) -> R3, but got %d -> %d dimensions.", N, Y);
    throw "Invalid Dimensionality";
  }
  return std::make_shared<base::SO3StateSpace>();
}

