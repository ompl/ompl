/* Author: Andreas Orthey */

#include <ompl/multilevel/datastructures/projections/FiberedProjection.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>

using namespace ompl::multilevel;

FiberedProjection::FiberedProjection(ompl::base::StateSpacePtr bundleSpace, ompl::base::StateSpacePtr baseSpace)
  : Projection(bundleSpace, baseSpace)
{
}

void FiberedProjection::lift(
    const ompl::base::State *xBase, 
    ompl::base::State *xBundle) const
{
    fiberSpaceSampler_->sampleUniform(xFiberTmp_);
    liftState(xBase, xFiberTmp_, xBundle);
}

ompl::base::StateSpacePtr FiberedProjection::getFiberSpace() const
{
    return fiberSpace_;
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
    // fiberSpace_ = nullptr;
    // if (components_.size() > 1)
    // {
    //     fiberSpace_ = std::make_shared<CompoundStateSpace>();
    //     for (unsigned int m = 0; m < components_.size(); m++)
    //     {
    //         StateSpacePtr FiberM = components_.at(m)->getFiberSpace();
    //         double weight = (FiberM->getDimension() > 0 ? 1.0 : 0.0);
    //         std::static_pointer_cast<CompoundStateSpace>(fiberSpace_)->addSubspace(FiberM, weight);
    //     }
    // }
    // else
    // {
    //     fiberSpace_ = components_.front()->getFiberSpace();
    // }

    fiberSpace_ = computeFiberSpace();

    if (fiberSpace_ != nullptr)
    {
        siFiberSpace_ = std::make_shared<ompl::base::SpaceInformation>(fiberSpace_);
        fiberSpaceSampler_ = siFiberSpace_->allocStateSampler();
        xFiberTmp_ = siFiberSpace_->allocState();
    }
}

