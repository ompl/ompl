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
#include <ompl/multilevel/datastructures/pathrestriction/FindSection.h>
#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>
#include <ompl/multilevel/datastructures/Projection.h>
#include <ompl/multilevel/datastructures/projections/FiberedProjection.h>

namespace ompl
{
    namespace magic
    {
        static const unsigned int PATH_SECTION_MAX_FIBER_SAMPLING = 10;
    }
}

using namespace ompl::multilevel;

FindSection::FindSection(PathRestriction *restriction) : restriction_(restriction)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();

    if (!graph->getProjection()->isFibered())
    {
        OMPL_DEBUG("Finding section with non-fibered projection.");
        return;
    }

    FiberedProjectionPtr projection = std::static_pointer_cast<FiberedProjection>(graph->getProjection());
    if (graph->getCoDimension() > 0)
    {
        base::StateSpacePtr fiber = projection->getFiberSpace();
        xFiberStart_ = fiber->allocState();
        xFiberGoal_ = fiber->allocState();
        xFiberTmp_ = fiber->allocState();
        validFiberSpaceSegmentLength_ = fiber->getLongestValidSegmentLength();
    }
    if (graph->getBaseDimension() > 0)
    {
        base::SpaceInformationPtr base = graph->getBase();
        xBaseTmp_ = base->allocState();
        validBaseSpaceSegmentLength_ = base->getStateSpace()->getLongestValidSegmentLength();
    }
    base::SpaceInformationPtr bundle = graph->getBundle();
    xBundleTmp_ = bundle->allocState();

    validBundleSpaceSegmentLength_ = bundle->getStateSpace()->getLongestValidSegmentLength();

    neighborhoodRadiusBaseSpaceLambda_ = 1e-4;

    neighborhoodRadiusBaseSpace_.setLambda(neighborhoodRadiusBaseSpaceLambda_);
    neighborhoodRadiusBaseSpace_.setValueInit(0.0);
    neighborhoodRadiusBaseSpace_.setValueTarget(10 * validBaseSpaceSegmentLength_);
}

FindSection::~FindSection()
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    FiberedProjectionPtr projection = std::static_pointer_cast<FiberedProjection>(graph->getProjection());

    if (graph->getCoDimension() > 0)
    {
        base::StateSpacePtr fiber = projection->getFiberSpace();
        fiber->freeState(xFiberStart_);
        fiber->freeState(xFiberGoal_);
        fiber->freeState(xFiberTmp_);
    }
    if (graph->getBaseDimension() > 0)
    {
        base::SpaceInformationPtr base = graph->getBase();
        base->freeState(xBaseTmp_);
    }
    base::SpaceInformationPtr bundle = graph->getBundle();
    bundle->freeState(xBundleTmp_);
}

bool FindSection::findFeasibleStateOnFiber(const ompl::base::State *xBase, ompl::base::State *xBundle)
{
    unsigned int ctr = 0;
    bool found = false;

    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();

    FiberedProjectionPtr projection = std::static_pointer_cast<FiberedProjection>(graph->getProjection());
    base::SpaceInformationPtr bundle = graph->getBundle();
    base::SpaceInformationPtr base = graph->getBundle();

    const ompl::base::StateSamplerPtr samplerFiber = projection->getFiberSamplerPtr();

    if (graph->getCoDimension() > 0)
    {
        while (ctr++ < magic::PATH_SECTION_MAX_FIBER_SAMPLING && !found)
        {
            samplerFiber->sampleUniform(xFiberTmp_);

            projection->lift(xBase, xFiberTmp_, xBundle);

            // New sample must be valid AND not reachable from last valid
            if (bundle->isValid(xBundle))
            {
                found = true;
            }
        }
    }
    else
    {
        base->copyState(xBundle, xBase);
    }
    return found;
}

bool FindSection::tripleStep(HeadPtr &head, const ompl::base::State *sBundleGoal, double locationOnBasePathGoal)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();
    base::SpaceInformationPtr base = graph->getBase();

    base::State *xBundleStartTmp = bundle->allocState();
    base::State *xBundleGoalTmp = bundle->allocState();
    base::State *xBase = base->cloneState(head->getStateBase());
    const base::State *sBundleStart = head->getState();

    FiberedProjectionPtr projection = std::static_pointer_cast<FiberedProjection>(graph->getProjection());
    projection->projectFiber(sBundleStart, xFiberStart_);
    projection->projectFiber(sBundleGoal, xFiberGoal_);
    base::StateSpacePtr fiber = projection->getFiberSpace();

    double fiberDist = fiber->distance(xFiberStart_, xFiberGoal_);
    if (fiberDist < 1e-3)
        return false;

    bool found = false;

    // mid point heuristic
    fiber->interpolate(xFiberStart_, xFiberGoal_, 0.5, xFiberTmp_);

    double location = head->getLocationOnBasePath() - validBaseSpaceSegmentLength_;

    // Triple step connection attempt
    // xBundleStartTmp <------- xBundleStart
    //     |
    //     |
    //     |
    //     v
    // xBundleGoalTmp -------> xBundleGoal

    while (!found && location >= 0)
    {
        restriction_->interpolateBasePath(location, xBase);

        projection->lift(xBase, xFiberTmp_, xBundleStartTmp);

        if (bundle->isValid(xBundleStartTmp))
        {
            projection->lift(xBase, xFiberStart_, xBundleStartTmp);
            projection->lift(xBase, xFiberGoal_, xBundleGoalTmp);

            if (bundle->isValid(xBundleStartTmp) && bundle->isValid(xBundleGoalTmp))
            {
                if (bundle->checkMotion(xBundleStartTmp, xBundleGoalTmp))
                {
                    bool feasible = true;

                    double fiberStepSize = 2 * validFiberSpaceSegmentLength_;

                    if (!bundle->checkMotion(sBundleStart, xBundleStartTmp))
                    {
                        feasible = false;

                        double fiberLocation = 0.25 * fiberDist;
                        do
                        {
                            fiberLocation -= fiberStepSize;

                            fiber->interpolate(xFiberStart_, xFiberGoal_, fiberLocation / fiberDist, xFiberTmp_);

                            projection->lift(xBase, xFiberTmp_, xBundleStartTmp);

                            if (bundle->checkMotion(sBundleStart, xBundleStartTmp) &&
                                bundle->checkMotion(xBundleStartTmp, xBundleGoalTmp))
                            {
                                feasible = true;
                                break;
                            }
                        } while (fiberLocation > -0.25 * fiberDist);
                        // try to repair
                    }
                    if (feasible && !bundle->checkMotion(xBundleGoalTmp, sBundleGoal))
                    {
                        feasible = false;

                        double fiberLocation = 0.25 * fiberDist;
                        do
                        {
                            fiberLocation += fiberStepSize;

                            fiber->interpolate(xFiberStart_, xFiberGoal_, fiberLocation / fiberDist, xFiberTmp_);

                            projection->lift(xBase, xFiberTmp_, xBundleGoalTmp);

                            if (bundle->checkMotion(xBundleGoalTmp, sBundleGoal) &&
                                bundle->checkMotion(xBundleStartTmp, xBundleGoalTmp))
                            {
                                feasible = true;
                                break;
                            }

                        } while (fiberLocation < 1.25 * fiberDist);
                    }
                    if (feasible)
                    {
                        found = true;
                    }
                    break;
                }
            }
        }

        location -= validBaseSpaceSegmentLength_;
    }

    if (found)
    {
        Configuration *xBackStep = new Configuration(bundle, xBundleStartTmp);
        graph->addConfiguration(xBackStep);
        graph->addBundleEdge(head->getConfiguration(), xBackStep);

        Configuration *xSideStep = new Configuration(bundle, xBundleGoalTmp);
        graph->addConfiguration(xSideStep);
        graph->addBundleEdge(xBackStep, xSideStep);

        // xBaseTmp_ is on last valid fiber.
        Configuration *xGoal = new Configuration(bundle, sBundleGoal);
        graph->addConfiguration(xGoal);
        graph->addBundleEdge(xSideStep, xGoal);

        head->setCurrent(xGoal, locationOnBasePathGoal);
    }

    bundle->freeState(xBundleStartTmp);
    bundle->freeState(xBundleGoalTmp);
    base->freeState(xBase);
    return found;
}
