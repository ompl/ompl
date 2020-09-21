#include <ompl/multilevel/datastructures/pathrestriction/PathSection.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/multilevel/datastructures/pathrestriction/BasePathHead.h>

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

bool PathSection::checkMotion(BasePathHeadPtr& head)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();

    base::SpaceInformationPtr bundle = graph->getBundle();
    base::SpaceInformationPtr base = graph->getBase();

    for (unsigned int k = 1; k < section_.size(); k++)
    {
        if (bundle->checkMotion(head->getState(), section_.at(k), lastValid_))
        {
            if (k < section_.size() - 1)
            {
                Configuration *xLast = 
                  addFeasibleSegment(head->getConfiguration(), section_.at(k));

                double locationOnBasePath = 
                  restriction_->getLengthBasePathUntil(sectionBaseStateIndices_.at(k));

                head->setCurrent(xLast, locationOnBasePath);
            }
            else
            {
                addFeasibleGoalSegment(head->getConfiguration(), head->getTargetConfiguration());
                //section_.at(k));
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


            double locationOnBasePath = 
                restriction_->getLengthBasePathUntil(lastValidIndexOnBasePath_) + distBaseSegment;

            //############################################################################
            // Get Last valid
            //############################################################################
            if (lastValid_.second > 0)
            {
                // add last valid into the bundle graph
                Configuration *xBundleLastValid = new Configuration(bundle, lastValid_.first);
                graph->addConfiguration(xBundleLastValid);
                graph->addBundleEdge(head->getConfiguration(), xBundleLastValid);

                head->setCurrent(xBundleLastValid, locationOnBasePath);
            }
            return false;
        }
    }
    return false;
}

// double PathSection::getLastValidBasePathLocation()
// {
//   return lastValidLocationOnBasePath_;
// }
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

const ompl::base::State* PathSection::back() const
{
  return section_.back();
}

const ompl::base::State* PathSection::front() const
{
  return section_.front();
}

void PathSection::interpolateL1FiberFirst(BasePathHeadPtr& head)
{
    section_.clear();
    sectionBaseStateIndices_.clear();

    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr base = graph->getBase();
    base::SpaceInformationPtr bundle = graph->getBundle();
    base::SpaceInformationPtr fiber = graph->getFiber();

    int size = head->getNumberOfRemainingStates() + 1;

    // std::cout << "Remaining states:" << size << std::endl;
    // std::cout << "Restriction size:" << restriction_->size() << std::endl;
    // std::cout << "Last valid idx:" << head->getLastValidBasePathIndex() << std::endl;
    // std::cout << "Next valid idx:" << head->getNextValidBasePathIndex() << std::endl;

    if (graph->getFiberDimension() > 0)
    {
        const base::State* xFiberStart = head->getStateFiber();
        const base::State* xFiberGoal = head->getStateTargetFiber();

        section_.resize(size + 1);

        bundle->allocStates(section_);

        graph->liftState(head->getBaseStateAt(0), xFiberStart, section_.front());

        sectionBaseStateIndices_.push_back(head->getBaseStateIndexAt(0));

        for (unsigned int k = 1; k < section_.size(); k++)
        {
            graph->liftState(head->getBaseStateAt(k - 1), xFiberGoal, section_.at(k));
            sectionBaseStateIndices_.push_back(head->getBaseStateIndexAt(k - 1));
        }
    }
    else
    {
        section_.resize(size);

        bundle->allocStates(section_);

        for (int k = 0; k < size; k++)
        {
            bundle->copyState(section_.at(k), head->getBaseStateAt(k));
            sectionBaseStateIndices_.push_back(head->getBaseStateIndexAt(k));
        }
    }
    // sanityCheck(head);
}

void PathSection::interpolateL1FiberLast(BasePathHeadPtr& head)
{
    section_.clear();
    sectionBaseStateIndices_.clear();

    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    const base::SpaceInformationPtr bundle = graph->getBundle();
    const base::SpaceInformationPtr base = graph->getBase();

    int size = head->getNumberOfRemainingStates() + 1;

    if (graph->getFiberDimension() > 0)
    {
        const base::State* xFiberStart = head->getStateFiber();
        const base::State* xFiberGoal = head->getStateTargetFiber();

        section_.resize(size + 1);

        bundle->allocStates(section_);

        for (int k = 0; k < size; k++)
        {
            graph->liftState(head->getBaseStateAt(k), xFiberStart, section_.at(k));
            sectionBaseStateIndices_.push_back(head->getBaseStateIndexAt(k));
        }
        graph->liftState(head->getBaseStateAt(size-1), xFiberGoal, section_.back());
        sectionBaseStateIndices_.push_back(head->getBaseStateIndexAt(size-1));
    }
    else
    {
        section_.resize(size);
        bundle->allocStates(section_);
        for (int k = 0; k < size; k++)
        {
            bundle->copyState(section_.at(k), head->getBaseStateAt(k));
            sectionBaseStateIndices_.push_back(head->getBaseStateIndexAt(k));
        }
    }
    sanityCheck(head);
}

void PathSection::interpolateL2(BasePathHeadPtr& head)
{
    section_.clear();
    sectionBaseStateIndices_.clear();

    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();
    const std::vector<base::State *> basePath = restriction_->getBasePath();

    int size = head->getNumberOfRemainingStates() + 1;

    section_.resize(size);
    bundle->allocStates(section_);

    if (graph->getFiberDimension() > 0)
    {
        const base::State* xFiberStart = head->getStateFiber();
        const base::State* xFiberGoal = head->getStateTargetFiber();

        double totalLengthBasePath = restriction_->getLengthBasePath();

        const base::SpaceInformationPtr fiber = graph->getFiber();
        for (unsigned int k = 0; k < restriction_->size(); k++)
        {
            double lengthCurrent = restriction_->getLengthBasePathUntil(k);
            double step = lengthCurrent / totalLengthBasePath;

            fiber->getStateSpace()->interpolate(xFiberStart, xFiberGoal, step, xFiberTmp_);

            graph->liftState(restriction_->getBaseStateAt(k), xFiberTmp_, section_.at(k));

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
    if (graph->vGoal_ <= 0)
    {
        graph->vGoal_ = graph->addConfiguration(xGoal);
    }
    graph->addBundleEdge(xLast, xGoal);

    xGoal->parent = xLast;
}

void PathSection::sanityCheck(BasePathHeadPtr& head)
{
    if(section_.size() > 0)
    {
        base::State *xi = head->getConfiguration()->state;
        base::State *xg = head->getTargetConfiguration()->state;

        BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
        base::SpaceInformationPtr bundle = graph->getBundle();
        base::SpaceInformationPtr base = graph->getBase();

        double d1 = bundle->distance(section_.front(), xi);
        double d2 = bundle->distance(section_.back(), xg);

        if( d1 > 1e-5 || d2 > 1e-5)
        {
            std::cout << "START STATE" << std::endl;
            bundle->printState(xi);
            std::cout << "START STATE (SECTION)" << std::endl;
            bundle->printState(section_.front());
            std::cout << "Dist:" << d1 << std::endl;
            std::cout << "GOAL STATE" << std::endl;
            bundle->printState(xg);
            std::cout << "GOAL STATE (SECTION)" << std::endl;
            bundle->printState(section_.back());
            std::cout << "Dist:" << d2 << std::endl;
            int size = head->getNumberOfRemainingStates();
            std::cout << "Section size: " << section_.size() << std::endl;
            std::cout << "Remaining states: " << size << std::endl;
            std::cout << "Restriction size:" << restriction_->size() << std::endl;
            std::cout << "Last valid idx:" << head->getLastValidBasePathIndex() << std::endl;
            std::cout << "Next valid idx:" << head->getNextValidBasePathIndex() << std::endl;
            std::cout << "Base states:" << std::endl;
            std::cout << *restriction_ << std::endl;
            throw Exception("Invalid Section");
        }
    }
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

void PathSection::print(std::ostream& out) const
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();
    base::SpaceInformationPtr base = graph->getBase();

    out << std::string(80, '-') << std::endl;
    out << "PATH SECTION" << std::endl;
    out << std::string(80, '-') << std::endl;

    out << section_.size() << " states over " 
      << restriction_->size() << " base states." << std::endl;

    int maxDisplay = 5;
    for (int k = 0; k < section_.size(); k++)
    {
        if(k > maxDisplay && (int)k < std::max(0, (int)section_.size() - maxDisplay)) continue;
        int idx = sectionBaseStateIndices_.at(k);
        out << "State " << k << ": ";
        bundle->printState(section_.at(k));
        out << "Over Base state (idx " << idx << ") ";
        base->printState(restriction_->getBasePath().at(idx));
        out << std::endl;
    }

    out << std::string(80, '-') << std::endl;
}

namespace ompl
{
    namespace multilevel
    {
        std::ostream& operator<<(std::ostream& out, const PathSection& s)
        {
            s.print(out);
            return out;
        }
    }
}
