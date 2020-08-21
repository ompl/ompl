#include <ompl/multilevel/datastructures/pathrestriction/BasePathHead.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>

using namespace ompl::multilevel;

BasePathHead::BasePathHead(
        PathRestriction *restriction,
        Configuration* xCurrent,
        Configuration* xTarget)
{
    xCurrent_ = xCurrent;
    xTarget_ = xTarget;

    restriction_ = restriction;
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();

    if(graph->getBaseDimension() > 0)
    {
        base::SpaceInformationPtr base = graph->getBase();
        xBaseCurrent_ = base->allocState();
        graph->projectBase(xCurrent->state, xBaseCurrent_);
    }
    if(graph->getFiberDimension() > 0)
    {
        base::SpaceInformationPtr fiber = graph->getFiber();
        xFiberCurrent_ = fiber->allocState();
        xFiberTarget_ = fiber->allocState();
        graph->projectFiber(xCurrent->state, xFiberCurrent_);
        graph->projectFiber(xTarget->state, xFiberTarget_);
    }
}

BasePathHead::BasePathHead(const BasePathHead &rhs)
{
    xTarget_ = rhs.getTargetConfiguration();
    restriction_ = rhs.getRestriction();

    xCurrent_ = rhs.getConfiguration();
    locationOnBasePath_ = rhs.getLocationOnBasePath();

    lastValidIndexOnBasePath_ = rhs.getLastValidBasePathIndex();

    xFiberCurrent_ = rhs.getStateFiberNonConst();
    xBaseCurrent_ = rhs.getStateBaseNonConst();
    xFiberTarget_ = rhs.getStateTargetFiberNonConst();
    xTarget_ = rhs.getTargetConfiguration();
}

BasePathHead::~BasePathHead()
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    if(graph->getFiberDimension() > 0)
    {
        base::SpaceInformationPtr fiber = graph->getFiber();
        fiber->freeState(xFiberCurrent_);
        fiber->freeState(xFiberTarget_);
    }

    if(graph->getBaseDimension() > 0)
    {
        base::SpaceInformationPtr base = graph->getBase();
        base->freeState(xBaseCurrent_);
    }
}

PathRestriction* BasePathHead::getRestriction() const
{
  return restriction_;
}

void BasePathHead::print()
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();

    std::cout << "Head at:";
    bundle->printState(xCurrent_->state);
    std::cout << "Base location:" << getLocationOnBasePath() << std::endl;
}



Configuration* BasePathHead::getConfiguration() const
{
    return xCurrent_;
}
const ompl::base::State* BasePathHead::getState() const
{
  return xCurrent_->state;
}
const ompl::base::State* BasePathHead::getStateFiber() const
{
  return xFiberCurrent_;
}
const ompl::base::State* BasePathHead::getStateBase() const
{
  return xBaseCurrent_;
}
ompl::base::State* BasePathHead::getStateFiberNonConst() const
{
  return xFiberCurrent_;
}
ompl::base::State* BasePathHead::getStateBaseNonConst() const
{
  return xBaseCurrent_;
}
Configuration* BasePathHead::getTargetConfiguration() const
{
    return xTarget_;
}
const ompl::base::State* BasePathHead::getStateTargetFiber() const
{
  return xFiberTarget_;
}
ompl::base::State* BasePathHead::getStateTargetFiberNonConst() const
{
  return xFiberTarget_;
}

void BasePathHead::setCurrent(Configuration *newCurrent, double location)
{
    xCurrent_ = newCurrent;
    locationOnBasePath_ = location;

    lastValidIndexOnBasePath_ = 
      restriction_->getBasePathLastIndexFromLocation(location);

    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    if(graph->getBaseDimension() > 0)
    {
        base::SpaceInformationPtr base = graph->getBase();
        graph->projectBase(xCurrent_->state, xBaseCurrent_);
    }
    if(graph->getFiberDimension() > 0)
    {
        base::SpaceInformationPtr fiber = graph->getFiber();
        graph->projectFiber(xCurrent_->state, xFiberCurrent_);
    }
}

int BasePathHead::getLastValidBasePathIndex() const
{
    return lastValidIndexOnBasePath_;
}
double BasePathHead::getLocationOnBasePath() const
{
    return locationOnBasePath_;
}

int BasePathHead::getNumberOfRemainingStates()
{
    int Nstates = restriction_->getBasePath().size();
    return Nstates - (lastValidIndexOnBasePath_ + 1);
}

void BasePathHead::setLocationOnBasePath(double d)
{
    locationOnBasePath_ = d;
}

void BasePathHead::setLastValidBasePathIndex(int k)
{
    lastValidIndexOnBasePath_ = k;
}

const ompl::base::State* BasePathHead::getBaseStateAt(int k) const
{
    //----- | ---------------X-------|---------
    //    lastValid        xCurrent    basePath(lastValid + 1)
    if(k <= 0)
    {
        return xBaseCurrent_;
    }
    else{
        return restriction_->getBasePath().at(lastValidIndexOnBasePath_ + k);
    }
}
int BasePathHead::getBaseStateIndexAt(int k) const
{
    //----- | ---------------X-------|---------
    //    lastValid        xCurrent    basePath(lastValid + 1)
    return lastValidIndexOnBasePath_ + k;
}
