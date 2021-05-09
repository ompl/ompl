/* Author: Andreas Orthey */

#include <ompl/multilevel/datastructures/projections/FiberedProjection.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>

using namespace ompl::multilevel;
using namespace ompl::base;

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
        fiberSpace_->setup();
        siFiberSpace_ = std::make_shared<ompl::base::SpaceInformation>(fiberSpace_);
        fiberSpaceSampler_ = siFiberSpace_->allocStateSampler();
        xFiberTmp_ = siFiberSpace_->allocState();
    }
}

ompl::base::StateSamplerPtr FiberedProjection::getFiberSamplerPtr() const
{
    return fiberSpaceSampler_;
}

CompoundFiberedProjection::CompoundFiberedProjection(
    const base::StateSpacePtr &bundleSpace, const base::StateSpacePtr &baseSpace,
    const std::vector<ProjectionPtr> &components):
  FiberedProjection(bundleSpace, baseSpace), CompoundProjection(bundleSpace, baseSpace, components)
{
    //Verify that all components are fibered
    unsigned int M = components_.size();
    for (unsigned int m = 0; m < M; m++)
    {
        if(!components.at(m)->isFibered())
        {
            OMPL_ERROR("Init Compound Fiber Projection with Non-fibered projection component.");
            throw "NonFiberComponentError";
        }
    }
    FiberedProjection::setType(PROJECTION_COMPOUND);
    CompoundProjection::setType(PROJECTION_COMPOUND);
}

void CompoundFiberedProjection::lift(const ompl::base::State *xBase, ompl::base::State *xBundle) const
{
    return CompoundProjection::lift(xBase, xBundle);
}

void CompoundFiberedProjection::project(const ompl::base::State *xBundle, ompl::base::State *xBase) const
{
    return CompoundProjection::project(xBundle, xBase);
}

unsigned int CompoundFiberedProjection::getBaseDimension() const
{
    return CompoundProjection::getBaseDimension();
}

unsigned int CompoundFiberedProjection::getDimension() const
{
    return CompoundProjection::getDimension();
}

unsigned int CompoundFiberedProjection::getCoDimension() const
{
    return CompoundProjection::getCoDimension();
}

bool CompoundFiberedProjection::isFibered() const
{
    return true;
}

bool CompoundFiberedProjection::isCompound() const
{
    return true;
}

void CompoundFiberedProjection::print(std::ostream &out) const
{
    CompoundProjection::print(out);
}

void CompoundFiberedProjection::lift(
    const ompl::base::State *xBase, 
    const ompl::base::State *xFiber, 
    ompl::base::State *xBundle) const
{
    unsigned int M = components_.size();

    if (M > 1)
    {
        for (unsigned int m = 0; m < M; m++)
        {
            const State *xmBase = xBase->as<CompoundState>()->as<State>(m);
            const State *xmFiber = xFiber->as<CompoundState>()->as<State>(m);
            State *xmBundle = xBundle->as<CompoundState>()->as<State>(m);
            FiberedProjectionPtr projFibered = 
              std::static_pointer_cast<FiberedProjection>(components_.at(m));
            projFibered->lift(xmBase, xmFiber, xmBundle);
        }
    }
    else
    {
        FiberedProjectionPtr projFibered = 
          std::static_pointer_cast<FiberedProjection>(components_.front());
        projFibered->lift(xBase, xFiber, xBundle);
    }
}

void CompoundFiberedProjection::projectFiber(
    const ompl::base::State *xBundle, 
    ompl::base::State *xFiber) const
{
    unsigned int M = components_.size();

    if (M > 1)
    {
        for (unsigned int m = 0; m < M; m++)
        {
            if(components_.at(m)->getCoDimension() > 0)
            {
                FiberedProjectionPtr projFibered = 
                  std::static_pointer_cast<FiberedProjection>(components_.at(m));
                const State *xmBundle = xBundle->as<CompoundState>()->as<State>(m);
                State *xmFiber = xFiber->as<CompoundState>()->as<State>(m);
                projFibered->projectFiber(xmBundle, xmFiber);
            }
        }
    }
    else
    {
        FiberedProjectionPtr projFibered = 
          std::static_pointer_cast<FiberedProjection>(components_.front());
        projFibered->projectFiber(xBundle, xFiber);
    }
}

ompl::base::StateSpacePtr CompoundFiberedProjection::computeFiberSpace()
{
    unsigned int M = components_.size();

    StateSpacePtr compoundFiber = std::make_shared<ompl::base::CompoundStateSpace>();

    for (unsigned int m = 0; m < M; m++)
    {
        FiberedProjectionPtr projFibered = 
          std::static_pointer_cast<FiberedProjection>(components_.at(m));
        StateSpacePtr fiberM = projFibered->getFiberSpace();
        compoundFiber->as<CompoundStateSpace>()->addSubspace(fiberM, 1.0);
    }
    
    return compoundFiber;
}
