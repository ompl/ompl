#include <ompl/multilevel/datastructures/pathrestriction/PathSection.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>

using namespace ompl::multilevel;

PathSection::PathSection(PathRestriction* restriction):
  restriction_(restriction)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    if (graph->getFiberDimension() > 0)
    {
        base::SpaceInformationPtr fiber = graph->getFiber();
        xFiberStart_ = fiber->allocState();
        xFiberGoal_ = fiber->allocState();
        xFiberTmp_ =  fiber->allocState();
    }
    if (graph->getBaseDimension() > 0)
    {
        base::SpaceInformationPtr base = graph->getBase();
        xBaseTmp_ = base->allocState();
    }
    base::SpaceInformationPtr bundle = graph->getBundle();
    xBundleTmp_ = bundle->allocState();
    lastValid_.first = bundle->allocState();
}

PathSection::~PathSection()
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();

    if (graph->getFiberDimension() > 0)
    {
        base::SpaceInformationPtr fiber = graph->getFiber();
        fiber->freeState(xFiberStart_);
        fiber->freeState(xFiberGoal_);
        fiber->freeState(xFiberTmp_);
    }
    if (graph->getBaseDimension() > 0)
    {
        base::SpaceInformationPtr base = graph->getBase();
        base->freeState(xBaseTmp_);
    }

    bundle->freeStates(section_);
    bundle->freeState(lastValid_.first);
    bundle->freeState(xBundleTmp_);
}

bool PathSection::checkMotion(
    Configuration* const xStart, 
    Configuration* const xGoal, 
    std::pair<base::State *, double>& lastValid_)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();

    base::SpaceInformationPtr bundle = graph->getBundle();

    Configuration *xLast = xStart;

    //assumption: xLast == section.at(k-1)
    for (unsigned int k = 1; k < section_.size(); k++)
    {
        lastValidIndexOnBasePath_ = sectionBaseStateIndices_.at(k-1);
        lastValidLocationOnBasePath_ = 
          restriction_->getLengthBasePathUntil(lastValidIndexOnBasePath_);

        if (bundle->checkMotion(xLast->state, section_.at(k), lastValid_))
        {
            if (k < section_.size() - 1)
            {
                xLast = addFeasibleSegment(xLast, section_.at(k));
            }
            else
            {
                addFeasibleGoalSegment(xLast, xGoal);
                return true;
            }
        }
        else
        {
            base::State* lastValidBaseState = restriction_->getBasePath().at(lastValidIndexOnBasePath_);

            graph->projectBase(lastValid_.first, xBaseTmp_);

            base::SpaceInformationPtr base = graph->getBase();

            double distBaseSegment = base->distance(lastValidBaseState, xBaseTmp_);

            lastValidLocationOnBasePath_ += distBaseSegment;

            // std::cout << "Stopped at last valid location: " << 
            //   lastValid_.second << "with k=" << k 
            //   << "/" << section_.size()<< std::endl;

            // std::cout << "Last valid base path index:" << 
            //   lastValidIndexOnBasePath_ << std::endl;

            // std::cout << "Last valid position on base path" << 
            //   lastValidLocationOnBasePath_ << std::endl;

            //############################################################################
            // Get Last valid
            //############################################################################
            if (lastValid_.second > 0)
            {
                // add last valid into the bundle graph
                xBundleLastValid_ = new Configuration(bundle, lastValid_.first);
                graph->addConfiguration(xBundleLastValid_);
                graph->addBundleEdge(xLast, xBundleLastValid_);
                xLast = xBundleLastValid_;
            }
            else
            {
                xBundleLastValid_ = xLast;
            }
            break;
        }
    }
    return false;
}

double PathSection::getLastValidBasePathLocation()
{
  return lastValidLocationOnBasePath_;
}
int PathSection::getLastValidBasePathIndex()
{
  return lastValidIndexOnBasePath_;
}

PathSection::Configuration* PathSection::getLastValidConfiguration()
{
    return xBundleLastValid_;
}

void PathSection::interpolateL1FiberFirst(
    const base::State *xFiberStart, 
    const base::State *xFiberGoal)
{
    section_.clear();
    sectionBaseStateIndices_.clear();

    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();
    const std::vector<base::State *> basePath = restriction_->getBasePath();

    if (graph->getFiberDimension() > 0)
    {
        section_.resize(basePath.size() + 1);
        bundle->allocStates(section_);

        graph->liftState(basePath.front(), xFiberStart, section_.front());
        sectionBaseStateIndices_.push_back(0);

        for (unsigned int k = 1; k < section_.size(); k++)
        {
            graph->liftState(basePath.at(k - 1), xFiberGoal, section_.at(k));
            sectionBaseStateIndices_.push_back(k - 1);
        }
    }
    else
    {
        section_.resize(basePath.size());
        bundle->allocStates(section_);
        for (unsigned int k = 0; k < basePath.size(); k++)
        {
            bundle->copyState(section_.at(k), basePath.at(k));
            sectionBaseStateIndices_.push_back(k);
        }
    }
}

void PathSection::interpolateL1FiberLast(
    const base::State *xFiberStart, 
    const base::State *xFiberGoal)
{
    section_.clear();
    sectionBaseStateIndices_.clear();

    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();
    const std::vector<base::State *> basePath = restriction_->getBasePath();

    if (graph->getFiberDimension() > 0)
    {
        section_.resize(basePath.size() + 1);
        bundle->allocStates(section_);
        for (unsigned int k = 0; k < basePath.size(); k++)
        {
            graph->liftState(basePath.at(k), xFiberStart, section_.at(k));
            sectionBaseStateIndices_.push_back(k);
        }
        sectionBaseStateIndices_.push_back(basePath.size());
        graph->liftState(basePath.back(), xFiberGoal, section_.back());
    }
    else
    {
        section_.resize(basePath.size());
        bundle->allocStates(section_);
        for (unsigned int k = 0; k < basePath.size(); k++)
        {
            bundle->copyState(section_.at(k), basePath.at(k));
            sectionBaseStateIndices_.push_back(k);
        }
    }
}

void PathSection::interpolateL2(
    const base::State *xFiberStart, 
    const base::State *xFiberGoal)
{
    section_.clear();
    sectionBaseStateIndices_.clear();

    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();
    const std::vector<base::State *> basePath = restriction_->getBasePath();

    section_.resize(basePath.size());

    bundle->allocStates(section_);

    double totalLengthBasePath = 0.0;
    for (unsigned int k = 1; k < basePath.size(); k++)
    {
        totalLengthBasePath += graph->getBase()->distance(basePath.at(k - 1), basePath.at(k));
    }

    if (graph->getFiberDimension() > 0)
    {
        double lengthCurrent = 0;

        for (unsigned int k = 0; k < basePath.size(); k++)
        {
            double step = lengthCurrent / totalLengthBasePath;

            graph->getFiber()->getStateSpace()->interpolate(xFiberStart, xFiberGoal, step, xFiberTmp_);

            graph->liftState(basePath.at(k), xFiberTmp_, section_.at(k));

            if (k < basePath.size() - 1)
            {
                lengthCurrent += graph->getBase()->distance(basePath.at(k), basePath.at(k + 1));
            }

            sectionBaseStateIndices_.push_back(k);
        }
    }
    else
    {
        for (unsigned int k = 0; k < basePath.size(); k++)
        {
            bundle->copyState(section_.at(k), basePath.at(k));
            sectionBaseStateIndices_.push_back(k);
        }
    }
}

BundleSpaceGraph::Configuration *
PathSection::addFeasibleSegment(
    Configuration *xLast, 
    base::State *sNext)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();

    base::SpaceInformationPtr bundle = graph->getBundle();

    Configuration *x = new Configuration(bundle, sNext);
    graph->addConfiguration(x);
    graph->addBundleEdge(xLast, x);

    x->parent = xLast;
    return x;
}

void PathSection::addFeasibleGoalSegment(
    Configuration *const xLast, 
    Configuration *const xGoal)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    if (xGoal->index <= 0)
    {
        graph->vGoal_ = graph->addConfiguration(xGoal);
    }
    graph->addBundleEdge(xLast, xGoal);

    xGoal->parent = xLast;
}

void PathSection::sanityCheck()
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();
    for (unsigned int k = 1; k < section_.size(); k++)
    {
        base::State *sk1 = section_.at(k - 1);
        base::State *sk2 = section_.at(k - 2);
        if (!bundle->checkMotion(sk1, sk2))
        {
            std::cout << "Error between states " << k - 1 << " and " << k << std::endl;
        }
    }
    throw Exception("Reported feasible path section, \
        but path section is infeasible.");
}
