/* Author: Andreas Orthey */

#include <ompl/multilevel/datastructures/ProjectionComponentWithFiber.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/util/Exception.h>

using namespace ompl::multilevel;

ProjectionComponentWithFiber::ProjectionComponentWithFiber(ompl::base::StateSpacePtr bundleSpace, ompl::base::StateSpacePtr baseSpace)
  : ProjectionComponent(bundleSpace, baseSpace)
{
    makeFiberSpace();


    if (fiberSpace_ != nullptr)
    {
        siFiberSpace_ = std::make_shared<SpaceInformation>(fiberSpace_);
        fiberSpaceSampler_ = siFiberSpace_->allocStateSampler();
        xFiberTmp_ = siFiberSpace_->allocState();
    }
}

void ProjectionComponentWithFiber::lift(
    const ompl::base::State *xBase, 
    ompl::base::State *xBundle) const
{
    fiberSpaceSampler_->sample(xFiberTmp_);
    liftState(xBase, xFiberTmp_, xBundle);
}

ompl::base::StateSpacePtr ProjectionComponentWithFiber::getFiberSpace() const
{
    return fiberSpace_;
}

unsigned int ProjectionComponentWithFiber::getFiberDimension() const
{
    if (fiberSpace_)
        return fiberSpace_->getDimension();
    else
        return 0;
}

std::string ProjectionComponentWithFiber::getFiberTypeAsString() const
{
    if (FiberSpace_)
        return stateTypeToString(FiberSpace_);
    else
        return "None";
}

void ProjectionComponentWithFiber::makeFiberSpace()
{
    fiberSpace_ = nullptr;
    if (components_.size() > 1)
    {
        fiberSpace_ = std::make_shared<CompoundStateSpace>();
        for (unsigned int m = 0; m < components_.size(); m++)
        {
            StateSpacePtr FiberM = components_.at(m)->getFiberSpace();
            double weight = (FiberM->getDimension() > 0 ? 1.0 : 0.0);
            std::static_pointer_cast<CompoundStateSpace>(fiberSpace_)->addSubspace(FiberM, weight);
        }
    }
    else
    {
        fiberSpace_ = components_.front()->getFiberSpace();
    }
}

