#include <ompl/geometric/planners/multilevel/datastructures/components/SE3RN_R3.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

ompl::geometric::BundleSpaceComponent_SE3RN_R3::BundleSpaceComponent_SE3RN_R3(base::StateSpacePtr BundleSpace,
                                                                              base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
}

void ompl::geometric::BundleSpaceComponent_SE3RN_R3::projectFiber(const ompl::base::State *xBundle,
                                                                  ompl::base::State *xFiber) const
{
    const base::SE3StateSpace::StateType *xBundle_SE3 =
        xBundle->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
    const base::SO3StateSpace::StateType *xBundle_SO3 = &xBundle_SE3->rotation();
    const base::RealVectorStateSpace::StateType *xBundle_RN =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    base::SO3StateSpace::StateType *xFiber_SO3 =
        xFiber->as<base::CompoundState>()->as<base::SO3StateSpace::StateType>(0);
    base::RealVectorStateSpace::StateType *xFiber_RN =
        xFiber->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    xFiber_SO3->x = xBundle_SO3->x;
    xFiber_SO3->y = xBundle_SO3->y;
    xFiber_SO3->z = xBundle_SO3->z;
    xFiber_SO3->w = xBundle_SO3->w;

    for (unsigned int k = 0; k < getFiberDimension() - 3; k++)
    {
        xFiber_RN->values[k] = xBundle_RN->values[k];
    }
}

void ompl::geometric::BundleSpaceComponent_SE3RN_R3::projectBase(const ompl::base::State *xBundle,
                                                                 ompl::base::State *xBase) const
{
    const base::SE3StateSpace::StateType *xBundle_SE3 =
        xBundle->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
    base::RealVectorStateSpace::StateType *xBase_R3 = xBase->as<base::RealVectorStateSpace::StateType>();

    xBase_R3->values[0] = xBundle_SE3->getX();
    xBase_R3->values[1] = xBundle_SE3->getY();
    xBase_R3->values[2] = xBundle_SE3->getZ();
}

void ompl::geometric::BundleSpaceComponent_SE3RN_R3::liftState(const ompl::base::State *xBase,
                                                               const ompl::base::State *xFiber,
                                                               ompl::base::State *xBundle) const
{
    base::SE3StateSpace::StateType *xBundle_SE3 =
        xBundle->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
    base::SO3StateSpace::StateType *xBundle_SO3 = &xBundle_SE3->rotation();
    base::RealVectorStateSpace::StateType *xBundle_RN =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    const base::RealVectorStateSpace::StateType *xBase_R3 = xBase->as<base::RealVectorStateSpace::StateType>();
    const base::SO3StateSpace::StateType *xFiber_SO3 =
        xFiber->as<base::CompoundState>()->as<base::SO3StateSpace::StateType>(0);
    const base::RealVectorStateSpace::StateType *xFiber_RN =
        xFiber->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    xBundle_SE3->setXYZ(xBase_R3->values[0], xBase_R3->values[1], xBase_R3->values[2]);
    xBundle_SO3->x = xFiber_SO3->x;
    xBundle_SO3->y = xFiber_SO3->y;
    xBundle_SO3->z = xFiber_SO3->z;
    xBundle_SO3->w = xFiber_SO3->w;

    for (unsigned int k = 0; k < getFiberDimension() - 3; k++)
    {
        xBundle_RN->values[k] = xFiber_RN->values[k];
    }
}

ompl::base::StateSpacePtr ompl::geometric::BundleSpaceComponent_SE3RN_R3::computeFiberSpace()
{
    base::CompoundStateSpace *Bundle_compound = BundleSpace_->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    const std::vector<base::StateSpacePtr> Bundle_SE3_decomposed =
        Bundle_decomposed.at(0)->as<base::CompoundStateSpace>()->getSubspaces();

    const base::RealVectorStateSpace *Bundle_RN = Bundle_decomposed.at(1)->as<base::RealVectorStateSpace>();
    unsigned int N = Bundle_RN->getDimension();

    base::StateSpacePtr SO3(new base::SO3StateSpace());
    base::StateSpacePtr RN(new base::RealVectorStateSpace(N));
    RN->as<base::RealVectorStateSpace>()->setBounds(Bundle_RN->getBounds());

    return SO3 + RN;
}
