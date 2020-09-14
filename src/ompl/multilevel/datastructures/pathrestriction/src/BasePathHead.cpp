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

    locationOnBasePath_ = rhs.getLocationOnBasePath();
    lastValidIndexOnBasePath_ = rhs.getLastValidBasePathIndex();

    xCurrent_ = rhs.getConfiguration();
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
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();

    xCurrent_ = newCurrent;

    locationOnBasePath_ = location;

    lastValidIndexOnBasePath_ = 
      restriction_->getBasePathLastIndexFromLocation(location);

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

int BasePathHead::getNextValidBasePathIndex() const
{
    int Nlast = getRestriction()->size() - 1;
    if(lastValidIndexOnBasePath_ < Nlast)
    {
        return lastValidIndexOnBasePath_ + 1;
    }else
    {
        return Nlast;
    }
}

double BasePathHead::getLocationOnBasePath() const
{
    return locationOnBasePath_;
}

int BasePathHead::getNumberOfRemainingStates()
{
    //----- | ---------------X-------|---------|
    //    lastValid        xCurrent    
    //    would result in three (including current head)

    int Nstates = restriction_->getBasePath().size();
    return std::max(1, Nstates - (lastValidIndexOnBasePath_ + 1));
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
        int idx = std::min(restriction_->size() - 1, lastValidIndexOnBasePath_ + k);
        return restriction_->getBasePath().at(idx);
    }
}

int BasePathHead::getBaseStateIndexAt(int k) const
{
    //----- | ---------------X-------|---------
    //    lastValid        xCurrent    basePath(lastValid + 1)
  

    int idx = lastValidIndexOnBasePath_ + k;
    if(idx > restriction_->size()-1)
    {
      idx = restriction_->size() - 1;
      // std::cout << "idx " << idx << std::endl;
      // throw Exception("WrongIndex");
    }
    return idx;
}

void BasePathHead::print(std::ostream &out) const
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();
    base::SpaceInformationPtr base = graph->getBase();

    out << "[ Head at:";
    int idx = getLastValidBasePathIndex();
    bundle->printState(xCurrent_->state);
    out << "base location " << getLocationOnBasePath()
      << "/" << restriction_->getLengthBasePath()
      << " idx " << idx
      << "/" << restriction_->size()
      << std::endl;
    out << "last base state idx ";
    base->printState(restriction_->getBasePath().at(idx));
    out << "]" << std::endl;
}

namespace ompl
{
    namespace multilevel
    {
        std::ostream& operator<<(std::ostream &out, const BasePathHead& h)
        {
            h.print(out);
            return out;
        }
    }
}
