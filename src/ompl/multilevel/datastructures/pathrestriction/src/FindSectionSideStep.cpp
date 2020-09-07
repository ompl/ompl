#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathSection.h>
#include <ompl/multilevel/datastructures/pathrestriction/BasePathHead.h>
#include <ompl/multilevel/datastructures/pathrestriction/FindSectionSideStep.h>
#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>

namespace ompl
{
    namespace magic
    {
        static const unsigned int PATH_SECTION_TREE_MAX_DEPTH = 3;
        static const unsigned int PATH_SECTION_TREE_MAX_BRANCHING = 10;
    }
}

using namespace ompl::multilevel;

FindSectionSideStep::FindSectionSideStep(PathRestriction* restriction):
  BaseT(restriction)
{
}

FindSectionSideStep::~FindSectionSideStep()
{
}

bool FindSectionSideStep::solve(BasePathHeadPtr& head)
{
    BasePathHeadPtr head2(head);

    bool foundFeasibleSection = recursiveSideStep(head);

    if(!foundFeasibleSection)
    {
        foundFeasibleSection = recursiveSideStep(head2, false);
    }

    return foundFeasibleSection;
}

bool FindSectionSideStep::recursiveSideStep(
    BasePathHeadPtr& head,
    bool interpolateFiberFirst, 
    unsigned int depth)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();
    base::SpaceInformationPtr base = graph->getBase();
    base::SpaceInformationPtr fiber = graph->getFiber();

    PathSectionPtr section = std::make_shared<PathSection>(restriction_);

    if (interpolateFiberFirst)
    {
        section->interpolateL1FiberFirst(head);
    }
    else
    {
        section->interpolateL1FiberLast(head);
    }

    if(section->checkMotion(head))
    {
        section->sanityCheck();
        return true;
    }

    static_cast<BundleSpaceGraph *>(graph->getBaseBundleSpace())
       ->getGraphSampler()
       ->setPathBiasStartSegment(head->getLocationOnBasePath());

    //############################################################################
    //Get last valid state information
    //############################################################################

    if (depth+1 >= magic::PATH_SECTION_TREE_MAX_DEPTH)
    {
        return false;
    }

    unsigned int infeasibleCtr = 0;

    double location = head->getLocationOnBasePath();

    for (unsigned int j = 0; j < magic::PATH_SECTION_TREE_MAX_BRANCHING; j++)
    {
        restriction_->interpolateBasePath(location, xBaseTmp_);

        if (!findFeasibleStateOnFiber(xBaseTmp_, xBundleTmp_))
        {
            infeasibleCtr++;
            continue;
        }

        if(bundle->checkMotion(head->getState(), xBundleTmp_))
        {
            Configuration *xSideStep = new Configuration(bundle, xBundleTmp_);
            graph->addConfiguration(xSideStep);
            graph->addBundleEdge(head->getConfiguration(), xSideStep);

            BasePathHeadPtr newHead(head);

            newHead->setCurrent(xSideStep, location);

            bool feasibleSection = recursiveSideStep(newHead, !interpolateFiberFirst, depth + 1);

            if(feasibleSection)
            {
                head = newHead;
                return true;
            }
        }

    }
    // std::cout << "Failed depth " << depth 
    //   << " after sampling " << infeasibleCtr 
    //   << " infeasible fiber elements on fiber"
    //   << std::endl;
    return false;
}


