#include <ompl/multilevel/datastructures/pathrestriction/BasePathHead.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>

using namespace ompl::multilevel;

BasePathHead::BasePathHead(
        PathRestriction *restriction,
        Configuration* xStart,
        Configuration* xGoal)
{
    xStart_ = xStart;
    xGoal_ = xGoal;

    restriction_ = restriction;
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();

    if(graph->getBaseDimension() > 0)
    {
        base::SpaceInformationPtr base = graph->getBase();
        xBaseStart_ = base->allocState();
        graph->projectBase(xStart->state, xBaseStart_);
    }
    if(graph->getFiberDimension() > 0)
    {
        base::SpaceInformationPtr fiber = graph->getFiber();
        xFiberStart_ = fiber->allocState();
        xFiberGoal_ = fiber->allocState();
        graph->projectFiber(xStart->state, xFiberStart_);
        graph->projectFiber(xGoal->state, xFiberGoal_);
    }
}

BasePathHead::~BasePathHead()
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    if(graph->getFiberDimension() > 0)
    {
        base::SpaceInformationPtr fiber = graph->getFiber();
        fiber->freeState(xFiberStart_);
        fiber->freeState(xFiberGoal_);
    }

    if(graph->getBaseDimension() > 0)
    {
        base::SpaceInformationPtr base = graph->getBase();
        base->freeState(xBaseStart_);
    }
}

ompl::base::State* BasePathHead::getFiberElementStart()
{
    return xFiberStart_;
}

ompl::base::State* BasePathHead::getFiberElementGoal()
{
    return xFiberGoal_;
}

Configuration* BasePathHead::getStartConfiguration()
{
    return xStart_;
}

Configuration* BasePathHead::getGoalConfiguration()
{
    return xGoal_;
}

int BasePathHead::getLastValidBasePathIndex()
{
    return lastValidIndexOnBasePath_;
}
double BasePathHead::getLocationOnBasePath()
{
    return locationOnBasePath_;
}

int BasePathHead::getNumberOfRemainingStates()
{
    return restriction_->getBasePath().size() - lastValidIndexOnBasePath_;
}

void BasePathHead::setLocationOnBasePath(double d)
{
    locationOnBasePath_ = d;
}

void BasePathHead::setLastValidBasePathIndex(int k)
{
    lastValidIndexOnBasePath_ = k;
}

const ompl::base::State* BasePathHead::getBaseStateAt(int k)
{
    //----- | ---------------X-------|---------
    //    lastValid        xStart    basePath(lastValid + 1)
    if(k <= 0)
    {
        return xBaseStart_;
    }
    else{
        return restriction_->getBasePath().at(lastValidIndexOnBasePath_ + k);
    }
}
int BasePathHead::getBaseStateIndexAt(int k)
{
    //----- | ---------------X-------|---------
    //    lastValid        xStart    basePath(lastValid + 1)
    return lastValidIndexOnBasePath_ + k;
}
