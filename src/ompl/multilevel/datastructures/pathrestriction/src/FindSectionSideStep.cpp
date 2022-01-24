/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020,
 *  Max Planck Institute for Intelligent Systems (MPI-IS).
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the MPI-IS nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Andreas Orthey */

#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathSection.h>
#include <ompl/multilevel/datastructures/pathrestriction/Head.h>
#include <ompl/multilevel/datastructures/pathrestriction/FindSectionSideStep.h>
#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>
#include <ompl/multilevel/datastructures/projections/FiberedProjection.h>

namespace ompl
{
    namespace magic
    {
        static const unsigned int PATH_SECTION_TREE_MAX_DEPTH = 3;
        static const unsigned int PATH_SECTION_TREE_MAX_BRANCHING = 10;
    }
}

using namespace ompl::multilevel;

FindSectionSideStep::FindSectionSideStep(PathRestriction *restriction) : BaseT(restriction)
{
}

FindSectionSideStep::~FindSectionSideStep()
{
}

bool FindSectionSideStep::solve(HeadPtr &head)
{
    Configuration *q = head->getConfiguration();

    HeadPtr head2(head);

    bool foundFeasibleSection = recursiveSideStep(head);

    if (!foundFeasibleSection)
    {
        head->setCurrent(q, 0);
        foundFeasibleSection = recursiveSideStep(head, false);
    }

    std::stringstream buffer;
    buffer << *head;
    OMPL_DEVMSG1("Stopped section method at %s.", buffer.str().c_str());

    return foundFeasibleSection;
}

bool FindSectionSideStep::recursiveSideStep(HeadPtr &head, bool interpolateFiberFirst, unsigned int depth)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    FiberedProjectionPtr projection = std::static_pointer_cast<FiberedProjection>(graph->getProjection());
    base::SpaceInformationPtr bundle = graph->getBundle();
    base::SpaceInformationPtr base = graph->getBase();

    PathSectionPtr section = std::make_shared<PathSection>(restriction_);

    if (interpolateFiberFirst)
    {
        section->interpolateL1FiberFirst(head);
    }
    else
    {
        section->interpolateL1FiberLast(head);
    }

    if (section->checkMotion(head))
    {
        return true;
    }

    static_cast<BundleSpaceGraph *>(graph->getChild())
        ->getGraphSampler()
        ->setPathBiasStartSegment(head->getLocationOnBasePath());

    //############################################################################
    // Get last valid state information
    //############################################################################

    if (depth + 1 >= magic::PATH_SECTION_TREE_MAX_DEPTH)
    {
        return false;
    }

    double location = head->getLocationOnBasePath();

    base::State *xBase = base->allocState();

    restriction_->interpolateBasePath(location, xBase);

    bool found = false;

    for (unsigned int j = 0; j < magic::PATH_SECTION_TREE_MAX_BRANCHING; j++)
    {
        if (!findFeasibleStateOnFiber(xBase, xBundleTmp_))
        {
            continue;
        }

        if (bundle->checkMotion(head->getState(), xBundleTmp_))
        {
            Configuration *xSideStep = new Configuration(bundle, xBundleTmp_);
            graph->addConfiguration(xSideStep);
            graph->addBundleEdge(head->getConfiguration(), xSideStep);

            HeadPtr newHead(head);

            newHead->setCurrent(xSideStep, location);

            bool feasibleSection = recursiveSideStep(newHead, !interpolateFiberFirst, depth + 1);

            if (feasibleSection)
            {
                head = newHead;
                found = true;
                break;
            }
        }
    }
    base->freeState(xBase);
    return found;
}
