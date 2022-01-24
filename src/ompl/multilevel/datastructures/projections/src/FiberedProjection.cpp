/* Author: Andreas Orthey */

#include <ompl/multilevel/datastructures/projections/FiberedProjection.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>

using namespace ompl::multilevel;

FiberedProjection::FiberedProjection(ompl::base::StateSpacePtr bundleSpace, ompl::base::StateSpacePtr baseSpace)
  : Projection(bundleSpace, baseSpace)
{
}

void FiberedProjection::lift(const ompl::base::State *xBase, ompl::base::State *xBundle) const
{
    fiberSpaceSampler_->sampleUniform(xFiberTmp_);
    lift(xBase, xFiberTmp_, xBundle);
}

ompl::base::StateSpacePtr FiberedProjection::getFiberSpace() const
{
    return fiberSpace_;
}

bool FiberedProjection::isFibered() const
{
    return true;
}

unsigned int FiberedProjection::getFiberDimension() const
{
    if (fiberSpace_)
        return fiberSpace_->getDimension();
    else
        return 0;
}

std::string FiberedProjection::getFiberTypeAsString() const
{
    if (fiberSpace_)
        return stateTypeToString(fiberSpace_);
    else
        return "None";
}

void FiberedProjection::makeFiberSpace()
{
    fiberSpace_ = computeFiberSpace();

    if (fiberSpace_ != nullptr)
    {
        siFiberSpace_ = std::make_shared<ompl::base::SpaceInformation>(fiberSpace_);
        fiberSpaceSampler_ = siFiberSpace_->allocStateSampler();
        xFiberTmp_ = siFiberSpace_->allocState();
    }
}

ompl::base::StateSamplerPtr FiberedProjection::getFiberSamplerPtr() const
{
    return fiberSpaceSampler_;
}
