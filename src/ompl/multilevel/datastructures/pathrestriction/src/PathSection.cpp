#include <ompl/multilevel/datastructures/pathrestriction/PathSection.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/multilevel/datastructures/pathrestriction/BasePathHead.h>

using namespace ompl::multilevel;

PathSection::PathSection(PathRestriction* restriction, BasePathHeadPtr head):
  restriction_(restriction), head_(head)
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

bool PathSection::checkMotion()
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();

    base::SpaceInformationPtr bundle = graph->getBundle();
    base::SpaceInformationPtr base = graph->getBase();

    Configuration *xStart = head_->getStartConfiguration();
    Configuration *xGoal = head_->getGoalConfiguration();

    Configuration *xLast = xStart;

    for (unsigned int k = 1; k < section_.size(); k++)
    {

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
            lastValidIndexOnSectionPath_ = k - 1;
            lastValidIndexOnBasePath_ = sectionBaseStateIndices_.at(k-1);

            base::State* lastValidBaseState = restriction_->getBasePath().at(lastValidIndexOnBasePath_);

            graph->projectBase(lastValid_.first, xBaseTmp_);

            double distBaseSegment = base->distance(lastValidBaseState, xBaseTmp_);

            lastValidLocationOnBasePath_ = 
              restriction_->getLengthBasePathUntil(lastValidIndexOnBasePath_)
              + distBaseSegment;

            // std::cout << "Stopped at last valid location: " << 
            //   lastValid_.second << "with k=" << k 
            //   << "/" << section_.size()<< std::endl;

            // std::cout << "Last valid base path index:" << 
            //   lastValidIndexOnBasePath_ << std::endl;

            // std::cout << "Last valid position on base path" << 
            //   lastValidLocationOnBasePath_ << " (" 
            //   << restriction_->getLengthBasePathUntil(lastValidIndexOnBasePath_) 
            //   << "+" << distBaseSegment << ")" << std::endl;

            //############################################################################
            // Get Last valid
            //############################################################################
            if (lastValid_.second > 0)
            {
                // add last valid into the bundle graph

                Configuration *xBundleLastValid = new Configuration(bundle, lastValid_.first);
                graph->addConfiguration(xBundleLastValid);
                graph->addBundleEdge(xLast, xBundleLastValid);
                xBundleLastValid_ = xBundleLastValid;
            }
            else
            {
                xBundleLastValid_ = xLast;
            }
            xLast = xBundleLastValid_;
            return false;
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
int PathSection::getLastValidSectionPathIndex()
{
  return lastValidIndexOnSectionPath_;
}

PathSection::Configuration* PathSection::getLastValidConfiguration()
{
    return xBundleLastValid_;
}

ompl::base::State* PathSection::at(int k) const
{
  return section_.at(k);
}

void PathSection::interpolateL1FiberFirst()
{
    section_.clear();
    sectionBaseStateIndices_.clear();

    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr base = graph->getBase();
    base::SpaceInformationPtr bundle = graph->getBundle();
    base::SpaceInformationPtr fiber = graph->getFiber();

    int size = head_->getNumberOfRemainingStates();

    if (graph->getFiberDimension() > 0)
    {
        const base::State* xFiberStart = head_->getFiberElementStart();
        const base::State* xFiberGoal = head_->getFiberElementGoal();

        section_.resize(size + 1);

        bundle->allocStates(section_);

        graph->liftState(head_->getBaseStateAt(0), xFiberStart, section_.front());

        sectionBaseStateIndices_.push_back(head_->getBaseStateIndexAt(0));

        for (unsigned int k = 1; k < section_.size(); k++)
        {
            graph->liftState(head_->getBaseStateAt(k - 1), xFiberGoal, section_.at(k));
            sectionBaseStateIndices_.push_back(head_->getBaseStateIndexAt(k - 1));
        }
    }
    else
    {
        section_.resize(size);

        bundle->allocStates(section_);

        for (int k = 0; k < size; k++)
        {
            bundle->copyState(section_.at(k), head_->getBaseStateAt(k));
            sectionBaseStateIndices_.push_back(head_->getBaseStateIndexAt(k));
        }
    }
}

void PathSection::interpolateL1FiberLast()
{
    section_.clear();
    sectionBaseStateIndices_.clear();

    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();

    int size = head_->getNumberOfRemainingStates();

    if (graph->getFiberDimension() > 0)
    {
        const base::State* xFiberStart = head_->getFiberElementStart();
        const base::State* xFiberGoal = head_->getFiberElementGoal();

        section_.resize(size + 1);

        bundle->allocStates(section_);

        for (int k = 0; k < size; k++)
        {
            graph->liftState(head_->getBaseStateAt(k), xFiberStart, section_.at(k));
            sectionBaseStateIndices_.push_back(head_->getBaseStateIndexAt(k));
        }
        graph->liftState(head_->getBaseStateAt(size-1), xFiberGoal, section_.back());
        sectionBaseStateIndices_.push_back(head_->getBaseStateIndexAt(size-1));
    }
    else
    {
        section_.resize(size);
        bundle->allocStates(section_);
        for (int k = 0; k < size; k++)
        {
            bundle->copyState(section_.at(k), head_->getBaseStateAt(k));
            sectionBaseStateIndices_.push_back(head_->getBaseStateIndexAt(k));
        }
    }
}

void PathSection::interpolateL2()
{
    throw ompl::Exception("NYI");
    // section_.clear();
    // sectionBaseStateIndices_.clear();

    // BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    // base::SpaceInformationPtr bundle = graph->getBundle();
    // const std::vector<base::State *> basePath = restriction_->getBasePath();

    // section_.resize(basePath.size());

    // bundle->allocStates(section_);

    // double totalLengthBasePath = 0.0;
    // for (unsigned int k = 1; k < basePath.size(); k++)
    // {
    //     totalLengthBasePath += graph->getBase()->distance(basePath.at(k - 1), basePath.at(k));
    // }

    // if (graph->getFiberDimension() > 0)
    // {
    //     double lengthCurrent = 0;

    //     for (unsigned int k = 0; k < basePath.size(); k++)
    //     {
    //         double step = lengthCurrent / totalLengthBasePath;

    //         graph->getFiber()->getStateSpace()->interpolate(xFiberStart, xFiberGoal, step, xFiberTmp_);

    //         graph->liftState(basePath.at(k), xFiberTmp_, section_.at(k));

    //         if (k < basePath.size() - 1)
    //         {
    //             lengthCurrent += graph->getBase()->distance(basePath.at(k), basePath.at(k + 1));
    //         }

    //         sectionBaseStateIndices_.push_back(k);
    //     }
    // }
    // else
    // {
    //     for (unsigned int k = 0; k < basePath.size(); k++)
    //     {
    //         bundle->copyState(section_.at(k), basePath.at(k));
    //         sectionBaseStateIndices_.push_back(k);
    //     }
    // }
}

BundleSpaceGraph::Configuration*
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
    Configuration *xLast, 
    Configuration *xGoal)
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
    bool feasible=true;
    for (unsigned int k = 1; k < section_.size(); k++)
    {
        base::State *sk1 = section_.at(k - 1);
        base::State *sk2 = section_.at(k);
        if (!bundle->checkMotion(sk1, sk2))
        {
            feasible = false;
            std::cout << "Error between states " << k - 1 << " and " << k << std::endl;
            bundle->printState(sk1);
            bundle->printState(sk2);
        }
    }

    if(section_.size() > 1)
    {
        base::State *sk1 = head_->getStartConfiguration()->state;
        base::State *sk2 = section_.at(1);
        if (!bundle->checkMotion(sk1, sk2))
        {
            feasible = false;
            std::cout << "Error between startState and second state" << std::endl;
            bundle->printState(sk1);
            bundle->printState(sk2);
        }
    }
    if(!feasible)
    {
      throw Exception("Reported feasible path section, \
        but path section is infeasible.");
    }
}

int PathSection::size() const
{
  return section_.size();
}

std::ostream &operator<<(std::ostream &out, const PathSection &s)
{
    out << "PathSection with " << s.size() << " states.";
    return out;
}
