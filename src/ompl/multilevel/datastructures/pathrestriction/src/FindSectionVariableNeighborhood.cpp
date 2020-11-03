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
#include <ompl/multilevel/datastructures/pathrestriction/BasePathHead.h>
#include <ompl/multilevel/datastructures/pathrestriction/FindSectionVariableNeighborhood.h>
#include <ompl/multilevel/datastructures/pathrestriction/FindSectionAnalyzer.h>
#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>

namespace ompl
{
    namespace magic
    {
        static const unsigned int PATH_SECTION_MAX_DEPTH = 5;
        static const unsigned int PATH_SECTION_MAX_BRANCHING = 1000;
        static const unsigned int PATH_SECTION_MAX_SAMPLING = 10;
    }
}

using namespace ompl::multilevel;

FindSectionVariableNeighborhood::FindSectionVariableNeighborhood(PathRestriction *restriction) : BaseT(restriction)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();

    neighborhoodBaseSpace_.setValueInit(0);
    neighborhoodBaseSpace_.setValueTarget(2 * validBaseSpaceSegmentLength_);
    neighborhoodBaseSpace_.setCounterInit(0);
    neighborhoodBaseSpace_.setCounterTarget(magic::PATH_SECTION_MAX_SAMPLING);

    neighborhoodFiberSpace_.setValueInit(validFiberSpaceSegmentLength_);
    neighborhoodFiberSpace_.setValueTarget(4 * validFiberSpaceSegmentLength_);
    neighborhoodFiberSpace_.setCounterInit(0);
    neighborhoodFiberSpace_.setCounterTarget(magic::PATH_SECTION_MAX_BRANCHING);
}

FindSectionVariableNeighborhood::~FindSectionVariableNeighborhood()
{
    // BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    // base::SpaceInformationPtr bundle = graph->getBundle();
}

bool FindSectionVariableNeighborhood::solve(BasePathHeadPtr &head)
{
    BasePathHeadPtr head2(head);

    ompl::time::point tStart = ompl::time::now();
    bool foundFeasibleSection = variableNeighborhoodPatternSearch(head);
    ompl::time::point t1 = ompl::time::now();

    OMPL_WARN("FindSectionVariableNeighborhood required %.2fs.", ompl::time::seconds(t1 - tStart));

    if (!foundFeasibleSection)
    {
        foundFeasibleSection = variableNeighborhoodPatternSearch(head2, false);
        ompl::time::point t2 = ompl::time::now();
        OMPL_WARN("FindSectionVariableNeighborhood2 required %.2fs.", ompl::time::seconds(t2 - t1));
    }

    return foundFeasibleSection;
}

bool FindSectionVariableNeighborhood::sideStepAlongFiber(Configuration *&xOrigin, ompl::base::State *state)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();
    if (bundle->checkMotion(xOrigin->state, state))
    {
        //#########################################################
        // side step was successful.
        // Now interpolate from there to goal
        //#########################################################

        Configuration *xSideStep = new Configuration(bundle, state);
        graph->addConfiguration(xSideStep);
        graph->addBundleEdge(xOrigin, xSideStep);

        xOrigin = xSideStep;

        return true;
    }
    return false;
}

bool FindSectionVariableNeighborhood::variableNeighborhoodPatternSearch(BasePathHeadPtr &head,
                                                                        bool interpolateFiberFirst, int depth)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();
    base::SpaceInformationPtr base = graph->getBase();
    base::SpaceInformationPtr fiber = graph->getFiber();

    PathSectionPtr section = std::make_shared<PathSection>(restriction_);

    double prevLocation = head->getLocationOnBasePath();

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
        section->sanityCheck();
        return true;
    }

    static_cast<BundleSpaceGraph *>(graph->getBaseBundleSpace())
        ->getGraphSampler()
        ->setPathBiasStartSegment(head->getLocationOnBasePath());

    if (depth >= (int)magic::PATH_SECTION_MAX_DEPTH)
    {
        return false;
    }
    //############################################################################
    // Try different strategies to locally resolve constraint violation
    // Then call function recursively with clipped base path
    //############################################################################

    double curLocation = head->getLocationOnBasePath();

    if (curLocation <= prevLocation)
    {
        return false;
    }

    ////search for alternative openings
    unsigned int infeasibleCtr = 0;

    double headLocation = head->getLocationOnBasePath() + validBaseSpaceSegmentLength_;

    const ompl::base::StateSamplerPtr samplerBase = graph->getBaseSamplerPtr();
    const ompl::base::StateSamplerPtr samplerFiber = graph->getFiberSamplerPtr();

    neighborhoodBaseSpace_.reset();
    neighborhoodFiberSpace_.reset();

    int idxNext = head->getNextValidBasePathIndex();
    const base::State *xBaseTarget = restriction_->getBaseStateAt(idxNext);

    double bestDistance = base->distance(head->getStateBase(), xBaseTarget);

    FindSectionAnalyzer analyzer(head);

    const base::State *xBundleTarget = section->back();

    ompl::RNG rng;

    bool found = false;
    for (unsigned int j = 0; j < magic::PATH_SECTION_MAX_BRANCHING; j++)
    {
        double location = rng.uniformReal(headLocation, headLocation + validBaseSpaceSegmentLength_);

        // double location = headLocation;
        double eps = std::min(neighborhoodBaseSpace_(), location - head->getLocationOnBasePath());

        restriction_->interpolateBasePath(location, xBaseTmp_);

        samplerBase->sampleUniformNear(xBaseTmp_, xBaseTmp_, eps);

        // Select Fiber Space Element
        if (j % 2 == 0)
        {
            if (j % 10 == 0)
            {
                graph->projectFiber(xBundleTarget, xFiberTmp_);
            }
            else
            {
                samplerFiber->sampleUniformNear(xFiberTmp_, head->getStateFiber(), neighborhoodFiberSpace_());
            }
        }
        else
        {
            graph->sampleFiber(xFiberTmp_);
        }

        graph->liftState(xBaseTmp_, xFiberTmp_, xBundleTmp_);

        if (!bundle->isValid(xBundleTmp_))
        {
            analyzer("infeasible");
            infeasibleCtr++;
            continue;
        }
        double curDistance = base->distance(xBaseTmp_, xBaseTarget);

        if (curDistance < bestDistance)
        {
            bestDistance = curDistance;
        }
        else
        {
            analyzer("suboptimal");
            infeasibleCtr++;
            continue;
        }

        if (cornerStep(head, xBundleTmp_, location) || tripleStep(head, xBundleTmp_, location))
        {
            BasePathHeadPtr newHead(head);

            bool feasibleSection = variableNeighborhoodPatternSearch(newHead, !interpolateFiberFirst, depth + 1);
            if (feasibleSection)
            {
                found = true;
            }
            break;
        }
        else
        {
            analyzer("failed triple/corner step");
        }
    }
    analyzer.print();
    return found;
}
