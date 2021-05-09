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
#include <ompl/multilevel/datastructures/pathrestriction/FindSectionPatternDance.h>
#include <ompl/multilevel/datastructures/pathrestriction/HeadAnalyzer.h>
#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>
#include <ompl/multilevel/datastructures/projections/FiberedProjection.h>

namespace ompl
{
    namespace magic
    {
        // static const unsigned int PATH_SECTION_MAX_WRIGGLING = 1000;
        // static const unsigned int PATH_SECTION_MAX_DEPTH = 3; //3 seems max
        // static const unsigned int PATH_SECTION_MAX_BRANCHING = 1000;
        // static const unsigned int PATH_SECTION_MAX_TUNNELING = 1000;

        // Values for submission
        // static const unsigned int PATH_SECTION_MAX_WRIGGLING = 100;
        // static const unsigned int PATH_SECTION_MAX_DEPTH = 2;        // 3 seems max
        // static const unsigned int PATH_SECTION_MAX_BRANCHING = 500;  // 500
        // static const unsigned int PATH_SECTION_MAX_TUNNELING = 100;
        
        static const unsigned int PATH_SECTION_MAX_WRIGGLING = 100;
        static const unsigned int PATH_SECTION_MAX_DEPTH = 2;        // 3 seems max
        static const unsigned int PATH_SECTION_MAX_BRANCHING = 100;  // 500
        static const unsigned int PATH_SECTION_MAX_TUNNELING = 100;
    }
}

using namespace ompl::multilevel;

FindSectionPatternDance::FindSectionPatternDance(PathRestriction *restriction) : BaseT(restriction)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();

    if (graph->hasBaseSpace())
    {
        base::SpaceInformationPtr base = graph->getBase();
        xBaseFixed_ = base->allocState();
    }
}

FindSectionPatternDance::~FindSectionPatternDance()
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();
    if (graph->hasBaseSpace())
    {
        base::SpaceInformationPtr base = graph->getBase();
        base->freeState(xBaseFixed_);
    }
}

bool FindSectionPatternDance::solve(HeadPtr &head)
{
    HeadPtr head2(head);

    ompl::time::point tStart = ompl::time::now();
    bool foundFeasibleSection = recursivePatternSearch(head);
    ompl::time::point t1 = ompl::time::now();

    OMPL_DEBUG("FindSectionPatternDance required %.2fs.", ompl::time::seconds(t1 - tStart));

    if (!foundFeasibleSection)
    {
        foundFeasibleSection = recursivePatternSearch(head2, false);
        ompl::time::point t2 = ompl::time::now();
        OMPL_DEBUG("FindSectionPatternDance2 required %.2fs.", ompl::time::seconds(t2 - t1));
    }

    return foundFeasibleSection;
}

bool FindSectionPatternDance::sideStepAlongFiber(Configuration *&xOrigin, ompl::base::State *state)
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

bool FindSectionPatternDance::tunneling(HeadPtr &head)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    const ompl::base::StateSamplerPtr bundleSampler = graph->getBundleSamplerPtr();
    const ompl::base::StateSamplerPtr baseSampler = graph->getBaseSamplerPtr();
    ompl::base::SpaceInformationPtr bundle = graph->getBundle();
    ompl::base::SpaceInformationPtr base = graph->getBase();

    double curLocation = head->getLocationOnBasePath();

    bool found = false;

    bool hitTunnel = false;

    FiberedProjectionPtr projection = std::static_pointer_cast<FiberedProjection>(graph->getProjection());
    const ompl::base::StateSamplerPtr fiberSampler = projection->getFiberSamplerPtr();

    while (curLocation < restriction_->getLengthBasePath() && !found)
    {
        curLocation += validBaseSpaceSegmentLength_;

        restriction_->interpolateBasePath(curLocation, xBaseTmp_);

        for (unsigned int k = 0; k < 5; k++)
        {
            fiberSampler->sampleUniformNear(xFiberTmp_, head->getStateFiber(), validFiberSpaceSegmentLength_);

            projection->lift(xBaseTmp_, xFiberTmp_, xBundleTmp_);

            if (bundle->isValid(xBundleTmp_))
            {
                if (hitTunnel)
                {
                    found = true;
                    break;
                }
            }
            else
            {
                hitTunnel = true;
            }
        }
    }
    if (!found)
    {
    }
    else
    {
        found = false;

        const double locationEndTunnel = curLocation;
        // const double lengthTunnel = curLocation - head->getLocationOnBasePath();
        const base::State *xBundleTunnelEnd = bundle->cloneState(xBundleTmp_);

        Configuration *last = head->getConfiguration();

        curLocation = head->getLocationOnBasePath();

        double bestDistance = bundle->distance(last->state, xBundleTunnelEnd);

        bool makingProgress = true;

        base::State *xBase = base->allocState();

        while (curLocation < locationEndTunnel && makingProgress)
        {
            if (bundle->checkMotion(last->state, xBundleTunnelEnd))
            {
                Configuration *xTunnel = new Configuration(bundle, xBundleTunnelEnd);
                graph->addConfiguration(xTunnel);
                graph->addBundleEdge(last, xTunnel);

                head->setCurrent(xTunnel, locationEndTunnel);

                base->freeState(xBase);

                return true;
            }

            curLocation += validBaseSpaceSegmentLength_;

            restriction_->interpolateBasePath(curLocation, xBaseTmp_);

            makingProgress = false;

            unsigned int ctr = 0;

            neighborhoodRadiusBaseSpace_.reset();

            while (ctr++ < magic::PATH_SECTION_MAX_TUNNELING)
            {
                double d = neighborhoodRadiusBaseSpace_();

                baseSampler->sampleUniformNear(xBase, xBaseTmp_, d);

                fiberSampler->sampleUniformNear(xFiberTmp_, head->getStateFiber(), 4 * validFiberSpaceSegmentLength_);

                projection->lift(xBase, xFiberTmp_, xBundleTmp_);

                if (bundle->isValid(xBundleTmp_))
                {
                    double d = bundle->distance(xBundleTmp_, xBundleTunnelEnd);
                    if (d < bestDistance)
                    {
                        if (bundle->checkMotion(last->state, xBundleTmp_))
                        {
                            Configuration *xTunnelStep = new Configuration(bundle, xBundleTmp_);
                            graph->addConfiguration(xTunnelStep);
                            graph->addBundleEdge(last, xTunnelStep);

                            last = xTunnelStep;

                            makingProgress = true;
                            bestDistance = d;
                            // add new configurations, but do not change head
                            // yet
                        }
                    }
                }
            }
        }
        base->freeState(xBase);
    }

    return false;
}

bool FindSectionPatternDance::wriggleFree(HeadPtr &head)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    ompl::base::SpaceInformationPtr bundle = graph->getBundle();

    double curLocation = head->getLocationOnBasePath() + validBaseSpaceSegmentLength_;

    double epsilon = 4 * validFiberSpaceSegmentLength_;

    FiberedProjectionPtr projection = std::static_pointer_cast<FiberedProjection>(graph->getProjection());
    ompl::base::StateSpacePtr fiber = projection->getFiberSpace();
    const ompl::base::StateSamplerPtr fiberSampler = projection->getFiberSamplerPtr();
    const ompl::base::StateSamplerPtr baseSampler = graph->getBaseSamplerPtr();

    base::State *xBundleMidPoint = bundle->allocState();
    int steps = 0;

    while (curLocation < restriction_->getLengthBasePath())
    {
        unsigned int ctr = 0;
        bool madeProgress = false;

        restriction_->interpolateBasePath(curLocation, xBaseTmp_);

        neighborhoodRadiusBaseSpace_.reset();

        while (ctr++ < magic::PATH_SECTION_MAX_WRIGGLING)
        {
            double d = neighborhoodRadiusBaseSpace_();

            baseSampler->sampleUniformNear(xBaseTmp_, xBaseTmp_, d);

            fiberSampler->sampleUniformNear(xFiberTmp_, head->getStateFiber(), epsilon);

            projection->lift(xBaseTmp_, xFiberTmp_, xBundleTmp_);

            //#########################################################
            // L2 step
            //#########################################################
            //
            if (bundle->isValid(xBundleTmp_))
            {
                if (bundle->checkMotion(head->getState(), xBundleTmp_))
                {
                    Configuration *xWriggleStep = new Configuration(bundle, xBundleTmp_);
                    graph->addConfiguration(xWriggleStep);
                    graph->addBundleEdge(head->getConfiguration(), xWriggleStep);

                    head->setCurrent(xWriggleStep, curLocation);

                    madeProgress = true;
                    steps++;
                    break;
                }
                else
                {
                    //#########################################################
                    // try corner step
                    //#########################################################
                    const base::State *xBaseHead = head->getStateBase();
                    projection->lift(xBaseHead, xFiberTmp_, xBundleMidPoint);

                    if (bundle->checkMotion(head->getState(), xBundleMidPoint) &&
                        bundle->checkMotion(xBundleMidPoint, xBundleTmp_))
                    {
                        Configuration *xMidPointStep = new Configuration(bundle, xBundleMidPoint);
                        graph->addConfiguration(xMidPointStep);
                        graph->addBundleEdge(head->getConfiguration(), xMidPointStep);

                        Configuration *xWriggleStep = new Configuration(bundle, xBundleTmp_);
                        graph->addConfiguration(xWriggleStep);
                        graph->addBundleEdge(xMidPointStep, xWriggleStep);

                        head->setCurrent(xWriggleStep, curLocation);

                        madeProgress = true;
                        steps++;
                    }
                }
            }
        }
        if (!madeProgress)
        {
            break;
        }
        curLocation += validBaseSpaceSegmentLength_;
    }

    bundle->freeState(xBundleMidPoint);

    if (steps > 0)
    {
        // try moving back to restriction
        fiber->copyState(xFiberTmp_, head->getStateFiber());

        double location = head->getLocationOnBasePath() + validBaseSpaceSegmentLength_;
        restriction_->interpolateBasePath(location, xBaseTmp_);

        unsigned int ctr = 0;

        while (ctr++ < magic::PATH_SECTION_MAX_WRIGGLING)
        {
            projection->lift(xBaseTmp_, xFiberTmp_, xBundleTmp_);

            if (bundle->isValid(xBundleTmp_))
            {
                if (bundle->checkMotion(head->getState(), xBundleTmp_))
                {
                    Configuration *xHomingStep = new Configuration(bundle, xBundleTmp_);
                    graph->addConfiguration(xHomingStep);
                    graph->addBundleEdge(head->getConfiguration(), xHomingStep);

                    head->setCurrent(xHomingStep, location);
                    break;
                }
            }

            fiberSampler->sampleUniformNear(xFiberTmp_, head->getStateFiber(), epsilon);
        }
    }

    return (steps > 0);
}

bool FindSectionPatternDance::recursivePatternSearch(HeadPtr &head, bool interpolateFiberFirst,
                                                     unsigned int depth)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();
    base::SpaceInformationPtr base = graph->getBase();
    FiberedProjectionPtr projection = std::static_pointer_cast<FiberedProjection>(graph->getProjection());
    base::StateSpacePtr fiber = projection->getFiberSpace();

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
        // section->sanityCheck();
        return true;
    }

    // static_cast<BundleSpaceGraph *>(graph->getBaseBundleSpace())
    //    ->getGraphSampler()
    //    ->setPathBiasStartSegment(head->getLocationOnBasePath());

    //############################################################################
    // Get last valid state information
    //############################################################################

    if (depth >= magic::PATH_SECTION_MAX_DEPTH)
    {
        return false;
    }

    //############################################################################
    // Try different strategies to locally resolve constraint violation
    // Then call function recursively with clipped base path
    //############################################################################

    if (wriggleFree(head) || tunneling(head))
    {
        HeadPtr newHead(head);

        bool feasibleSection = recursivePatternSearch(newHead, false, depth + 1);
        if (feasibleSection)
        {
            return true;
        }
    }

    double curLocation = head->getLocationOnBasePath();

    if (curLocation <= prevLocation)
    {
        return false;
    }

    //############################################################################
    ////search for alternative openings
    //############################################################################

    double location = head->getLocationOnBasePath() + validBaseSpaceSegmentLength_;

    const ompl::base::StateSamplerPtr baseSampler = graph->getBaseSamplerPtr();
    const ompl::base::StateSamplerPtr fiberSampler = projection->getFiberSamplerPtr();

    // Use smoothly varying parameter to increase neighborhood on base space
    // while we did not find solution (we model here a change of belief of
    // staying tight to the path restriction and allowing deviations from it if
    // we cannot find feasible samples)

    ParameterSmoothStep neighborhoodBaseSpace;
    neighborhoodBaseSpace.setValueInit(0);
    neighborhoodBaseSpace.setValueTarget(validBaseSpaceSegmentLength_);
    neighborhoodBaseSpace.setCounterInit(0);
    neighborhoodBaseSpace.setCounterTarget(magic::PATH_SECTION_MAX_BRANCHING);
    neighborhoodBaseSpace.reset();

    HeadAnalyzer analyzer(head);
    // analyzer.disable();

    bool found = false;

    const base::State *xBundleTarget = section->back();
    const base::State *xBundleInit = section->front();

    //@TODO:
    //  Notes on possible improvements:
    //  -- Once we find valid sample, it usually represents a neighborhood of
    //  states. We should then start focusing on that neighborhood to
    //  exhaustively try to reach it.
    //  -- Try to save explored neighborhoods to reroute resources to other
    //  neighborhoods (i.e. like Tabu Search)
    //  -- Maybe even build neighborhood graph in non-reachable neighborhood
    //  (could be useful to explore later on once we go to normal graph/path restriction
    //  sampling mode)
    //  -- Might be a bad idea to just discard locally reachable states (i.e.
    //  sidesteps)
    //
    for (unsigned int j = 0; j < magic::PATH_SECTION_MAX_BRANCHING; j++)
    {
        //############################################################################
        // Move base state forward by validsegmentlength_ to penetrate slightly
        // the constraints. This will ensure that sampling along fiber space
        // later will not hit the current feasible region (or potential similar infeasible
        // regions which occur trough symmetries of the robots geometry---like
        // rotations of a cylinder in front of a hole)
        //############################################################################
        // interpolateBasePath(locationOnBasePath + validBaseSpaceSegmentLength_, xBaseTmp_);
        // double offset = std::max(validBaseSpaceSegmentLength_, epsNBH);

        location = head->getLocationOnBasePath() + validBaseSpaceSegmentLength_;
        // location = head->getLocationOnBasePath();

        restriction_->interpolateBasePath(location, xBaseTmp_);

        // double epsNBH = neighborhoodBaseSpace(); //every time we call it, the NBH size increases

        // baseSampler->sampleUniformNear(xBaseTmp_, xBaseTmp_, epsNBH);

        if (j == 0 || j % 10 == 0)
        {
            // Making a sidestep to the goal or start state is often
            // advantageous and similar to the rrt-style goal bias
            if (j % 20 == 0)
            {
                projection->projectFiber(xBundleTarget, xFiberTmp_);
            }
            else
            {
                projection->projectFiber(xBundleInit, xFiberTmp_);
            }
            // projection->projectFiber(xBundleTarget, xFiberTmp_);

            projection->lift(xBaseTmp_, xFiberTmp_, xBundleTmp_);

            if (!bundle->isValid(xBundleTmp_))
            {
                analyzer("infeasible");
                continue;
            }
        }
        else
        {
            if (!findFeasibleStateOnFiber(xBaseTmp_, xBundleTmp_))
            {
                analyzer("infeasible");
                continue;
            }

            // automatically accept states which are valid AND far away
            double dMax = graph->getRange();
            if (bundle->distance(head->getState(), xBundleTmp_) < dMax)
            {
                // accept nearby states only if motion is infeasible
                if (bundle->checkMotion(head->getState(), xBundleTmp_))
                {
                    analyzer("locally reachable (ignored)");
                    continue;
                }
            }
        }

        // if(cornerStep(head, xBundleTmp_, location) ||
        if (tripleStep(head, xBundleTmp_, location))
        {
            HeadPtr newHead(head);

            bool feasibleSection = recursivePatternSearch(newHead, false, depth + 1);
            if (feasibleSection)
            {
                found = true;
                break;
            }
            else
            {
                analyzer("no section");
                continue;
            }
        }
        else
        {
            analyzer("not reachable (triple step)");
            continue;
        }
    }
    analyzer.print();
    return found;
}
