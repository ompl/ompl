#include <ompl/geometric/planners/quotientspace/datastructures/components/SE3RN_SE3RM.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

ompl::geometric::BundleSpaceComponent_SE3RN_SE3RM::BundleSpaceComponent_SE3RN_SE3RM(
    base::StateSpacePtr BundleSpace,
    base::StateSpacePtr BaseSpace):
  BaseT(BundleSpace, BaseSpace)
{
}

void ompl::geometric::BundleSpaceComponent_SE3RN_SE3RM::projectFiber(
    const ompl::base::State *xBundle,
    ompl::base::State *xFiber) const
{
    const base::RealVectorStateSpace::StateType *xBundle_RN =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    const base::RealVectorStateSpace::StateType *xFiber_RN = xFiber->as<base::RealVectorStateSpace::StateType>();

    unsigned int N = getDimension() - getFiberDimension() - 6;
    for (unsigned int k = N; k < getDimension() - 6; k++)
    {
        xFiber_RN->values[k - N] = xBundle_RN->values[k];
    }
}


void ompl::geometric::BundleSpaceComponent_SE3RN_SE3RM::projectBase(
    const ompl::base::State *xBundle,
    ompl::base::State *xBase) const
{
    const base::SE3StateSpace::StateType *xBundle_SE3 =
        xBundle->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
    const base::SO3StateSpace::StateType *xBundle_SO3 = &xBundle_SE3->rotation();
    const base::RealVectorStateSpace::StateType *xBundle_RN =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    base::SE3StateSpace::StateType *xBase_SE3 =
        xBase->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
    base::SO3StateSpace::StateType *xBase_SO3 = &xBase_SE3->rotation();
    base::RealVectorStateSpace::StateType *xBase_RM =
        xBase->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    xBase_SE3->setXYZ(xBundle_SE3->getX(), xBundle_SE3->getY(), xBundle_SE3->getZ());
    xBase_SO3->x = xBundle_SO3->x;
    xBase_SO3->y = xBundle_SO3->y;
    xBase_SO3->z = xBundle_SO3->z;
    xBase_SO3->w = xBundle_SO3->w;

    for (unsigned int k = 0; k < getBaseDimension() - 6; k++)
    {
        xBase_RM->values[k] = xBundle_RN->values[k];
    }
}


void ompl::geometric::BundleSpaceComponent_SE3RN_SE3RM::mergeStates(
    const ompl::base::State *xBase, 
    const ompl::base::State *xFiber, 
    ompl::base::State *xBundle) const
{
    base::SE3StateSpace::StateType *xBundle_SE3 =
        xBundle->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
    base::SO3StateSpace::StateType *xBundle_SO3 = &xBundle_SE3->rotation();
    base::RealVectorStateSpace::StateType *xBundle_RN =
        xBundle->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    const base::SE3StateSpace::StateType *xBase_SE3 =
        xBase->as<base::CompoundState>()->as<base::SE3StateSpace::StateType>(0);
    const base::SO3StateSpace::StateType *xBase_SO3 = &xBase_SE3->rotation();
    const base::RealVectorStateSpace::StateType *xBase_RM =
        xBase->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1);

    const base::RealVectorStateSpace::StateType *xFiber_RJ = xFiber->as<base::RealVectorStateSpace::StateType>();

    xBundle_SE3->setXYZ(xBase_SE3->getX(), xBase_SE3->getY(), xBase_SE3->getZ());
    xBundle_SO3->x = xBase_SO3->x;
    xBundle_SO3->y = xBase_SO3->y;
    xBundle_SO3->z = xBase_SO3->z;
    xBundle_SO3->w = xBase_SO3->w;

    //[X Y Z YAW PITCH ROLL] [1...M-1][M...N-1]
    // SE3                                        RN
    unsigned int M = getDimension() - getFiberDimension() - 6;
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

ompl::base::StateSpacePtr ompl::geometric::BundleSpaceComponent_SE3RN_SE3RM::computeFiberSpace()
{
    base::CompoundStateSpace *Bundle_compound = BundleSpace_->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();

    base::CompoundStateSpace *Base_compound = BaseSpace_->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();

    unsigned int N = Bundle_decomposed.at(1)->getDimension();
    unsigned int M = Base_decomposed.at(1)->getDimension();
    unsigned int NX = N - M;

    const base::RealVectorStateSpace *Bundle_RN = Bundle_decomposed.at(1)->as<base::RealVectorStateSpace>();
    
    // = std::make_shared<base::RealVectorStateSpace>(NX);
    base::StateSpacePtr RN(new base::RealVectorStateSpace(N));

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

