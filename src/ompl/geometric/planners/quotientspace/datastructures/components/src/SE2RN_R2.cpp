#include <ompl/geometric/planners/quotientspace/datastructures/components/SE2RN_R2.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/util/Exception.h>

ompl::geometric::BundleSpaceComponent_SE2RN_R2::BundleSpaceComponent_SE2RN_R2(
    base::StateSpacePtr BundleSpace,
    base::StateSpacePtr BaseSpace):
  BaseT(BundleSpace, BaseSpace)
{
}

void ompl::geometric::BundleSpaceComponent_SE2RN_R2::projectFiber(
    const ompl::base::State *xBundle,
    ompl::base::State *xFiber) const
{
    const base::SE2StateSpace::StateType *xBundle_SE2 =
        xBundle->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
    const base::RealVectorStateSpace::StateType *xBundle_RN =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    base::SO2StateSpace::StateType *xFiber_SO2 =
        xFiber->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
    base::RealVectorStateSpace::StateType *xFiber_RN =
        xFiber->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    xFiber_SO2->value = xBundle_SE2->getYaw();
    for (unsigned int k = 0; k < getFiberDimension() - 1; k++)
    {
        xFiber_RN->values[k] = xBundle_RN->values[k];
    }
}

void ompl::geometric::BundleSpaceComponent_SE2RN_R2::projectBase(
    const ompl::base::State *xBundle,
    ompl::base::State *xBase) const
{
    const base::SE2StateSpace::StateType *xBundle_SE2 =
        xBundle->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
    base::RealVectorStateSpace::StateType *xBase_R2 = xBase->as<base::RealVectorStateSpace::StateType>();
    xBase_R2->values[0] = xBundle_SE2->getX();
    xBase_R2->values[1] = xBundle_SE2->getY();
}

void ompl::geometric::BundleSpaceComponent_SE2RN_R2::liftState(
    const ompl::base::State *xBase, 
    const ompl::base::State *xFiber, 
    ompl::base::State *xBundle) const
{
     base::SE2StateSpace::StateType *xBundle_SE2 =
         xBundle->as<base::CompoundState>()->as<base::SE2StateSpace::StateType>(0);
     base::RealVectorStateSpace::StateType *xBundle_RN =
         xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

     const base::RealVectorStateSpace::StateType *xBase_R2 = 
         xBase->as<base::RealVectorStateSpace::StateType>();

     const base::SO2StateSpace::StateType *xFiber_SO2 =
         xFiber->as<base::CompoundState>()->as<base::SO2StateSpace::StateType>(0);
     const base::RealVectorStateSpace::StateType *xFiber_RN =
         xFiber->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

     xBundle_SE2->setX(xBase_R2->values[0]);
     xBundle_SE2->setY(xBase_R2->values[1]);
     xBundle_SE2->setYaw(xFiber_SO2->value);

     for (unsigned int k = 0; k < getFiberDimension() - 1; k++)
     {
         xBundle_RN->values[k] = xFiber_RN->values[k];
     }
}

ompl::base::StateSpacePtr ompl::geometric::BundleSpaceComponent_SE2RN_R2::computeFiberSpace()
{
    unsigned int N = BundleSpace_->getDimension();
    unsigned int Y = BaseSpace_->getDimension();
    if(N > 3 && Y != 2)
    {
      OMPL_ERROR("Assumed input is SE(2)xRN -> R2, but got %d -> %d dimensions.", N, Y);
      throw Exception("Invalid Dimensionality");
    }

    base::CompoundStateSpace *Bundle_compound = BundleSpace_->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    const std::vector<base::StateSpacePtr> Bundle_SE2_decomposed =
        Bundle_decomposed.at(0)->as<base::CompoundStateSpace>()->getSubspaces();

    const base::RealVectorStateSpace *Bundle_RN = 
      Bundle_decomposed.at(1)->as<base::RealVectorStateSpace>();
    unsigned int N_RN = Bundle_RN->getDimension();

    base::StateSpacePtr SO2(new base::SO2StateSpace());
    base::StateSpacePtr RN(new base::RealVectorStateSpace(N_RN));
    RN->as<base::RealVectorStateSpace>()->setBounds(Bundle_RN->getBounds());

    return SO2 + RN;
}
