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
#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <ompl/base/Path.h>
#include <ompl/geometric/PathGeometric.h>
#include <numeric>

#define SANITY_CHECK_PATH_RESTRICTION
#undef SANITY_CHECK_PATH_RESTRICTION

using namespace ompl::multilevel;

PathRestriction::PathRestriction(BundleSpaceGraph *bundleSpaceGraph)
  : bundleSpaceGraph_(bundleSpaceGraph)
{
    if (bundleSpaceGraph_->getFiberDimension() > 0)
    {
        base::SpaceInformationPtr fiber = bundleSpaceGraph_->getFiber();
        xFiberStart_ = fiber->allocState();
        xFiberGoal_ = fiber->allocState();
        xFiberTmp_ =  fiber->allocState();
    }
    if (bundleSpaceGraph_->getBaseDimension() > 0)
    {
        base::SpaceInformationPtr base = bundleSpaceGraph_->getBase();
        xBaseTmp_ = base->allocState();
        validSegmentLength_ = base->getStateSpace()->getLongestValidSegmentLength();
    }
    base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();
    xBundleTmp_ = bundle->allocState();
    lastValid_.first = bundle->allocState();
}

PathRestriction::~PathRestriction()
{
    if (bundleSpaceGraph_->isDynamic())
        return;

    if (bundleSpaceGraph_->getFiberDimension() > 0)
    {
        base::SpaceInformationPtr fiber = bundleSpaceGraph_->getFiber();
        fiber->freeState(xFiberStart_);
        fiber->freeState(xFiberGoal_);
        fiber->freeState(xFiberTmp_);
    }
    if (bundleSpaceGraph_->getBaseDimension() > 0)
    {
        base::SpaceInformationPtr base = bundleSpaceGraph_->getBase();
        base->freeState(xBaseTmp_);
    }
    base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();
    bundle->freeState(lastValid_.first);
    bundle->freeState(xBundleTmp_);
}

void PathRestriction::clear()
{
    basePath_.clear();
}

void PathRestriction::setBasePath(ompl::base::PathPtr path)
{
    geometric::PathGeometricPtr geometricBasePath = std::static_pointer_cast<geometric::PathGeometric>(path);
    setBasePath(geometricBasePath->getStates());
}

void PathRestriction::setBasePath(std::vector<base::State *> basePath)
{
    basePath_ = basePath;

    lengthBasePath_ = 0.0;
    intermediateLengthsBasePath_.clear();
    for (unsigned int k = 1; k < basePath_.size(); k++)
    {
        double lk = bundleSpaceGraph_->getBase()->distance(basePath_.at(k - 1), basePath_.at(k));
        intermediateLengthsBasePath_.push_back(lk);
        lengthBasePath_ += lk;
    }
    OMPL_INFORM("Set new base path with %d states and length %f.", basePath_.size(), lengthBasePath_);
}

std::vector<ompl::base::State *> PathRestriction::interpolateSectionL1FL(
    const base::State *xFiberStart, const base::State *xFiberGoal, const std::vector<base::State *> basePath)
{
    std::vector<base::State *> bundlePath;

    base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();

    if (bundleSpaceGraph_->getFiberDimension() > 0)
    {
        bundlePath.resize(basePath.size() + 1);
        bundle->allocStates(bundlePath);
        for (unsigned int k = 0; k < bundlePath.size() - 1; k++)
        {
            bundleSpaceGraph_->liftState(basePath.at(k), xFiberStart, bundlePath.at(k));
        }
        bundleSpaceGraph_->liftState(basePath.back(), xFiberGoal, bundlePath.back());
    }
    else
    {
        bundlePath.resize(basePath.size());
        bundle->allocStates(bundlePath);
        for (unsigned int k = 0; k < basePath.size(); k++)
        {
            bundle->copyState(bundlePath.at(k), basePath.at(k));
        }
    }
    return bundlePath;
}

std::vector<ompl::base::State *> PathRestriction::interpolateSectionL1FF(
    const base::State *xFiberStart, const base::State *xFiberGoal, const std::vector<base::State *> basePath)
{
    std::vector<base::State *> bundlePath;

    base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();

    if (bundleSpaceGraph_->getFiberDimension() > 0)
    {
        bundlePath.resize(basePath.size() + 1);
        bundle->allocStates(bundlePath);

        bundleSpaceGraph_->liftState(basePath.front(), xFiberStart, bundlePath.front());
        for (unsigned int k = 1; k < bundlePath.size(); k++)
        {
            bundleSpaceGraph_->liftState(basePath.at(k - 1), xFiberGoal, bundlePath.at(k));
        }
    }
    else
    {
        bundlePath.resize(basePath.size());
        bundle->allocStates(bundlePath);
        for (unsigned int k = 0; k < basePath.size(); k++)
        {
            bundle->copyState(bundlePath.at(k), basePath.at(k));
        }
    }
    return bundlePath;
}

std::vector<ompl::base::State *> PathRestriction::interpolateSectionL2(
    const base::State *xFiberStart, const base::State *xFiberGoal, const std::vector<base::State *> basePath)
{
    std::vector<base::State *> bundlePath;
    bundlePath.resize(basePath.size());

    base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();

    bundle->allocStates(bundlePath);

    double totalLengthBasePath = 0.0;
    for (unsigned int k = 1; k < basePath.size(); k++)
    {
        totalLengthBasePath += bundleSpaceGraph_->getBase()->distance(basePath.at(k - 1), basePath.at(k));
    }

    if (bundleSpaceGraph_->getFiberDimension() > 0)
    {
        double lengthCurrent = 0;

        for (unsigned int k = 0; k < basePath.size(); k++)
        {
            double step = lengthCurrent / totalLengthBasePath;

            bundleSpaceGraph_->getFiber()->getStateSpace()->interpolate(xFiberStart, xFiberGoal, step, xFiberTmp_);

            bundleSpaceGraph_->liftState(basePath.at(k), xFiberTmp_, bundlePath.at(k));

            if (k < basePath.size() - 1)
            {
                lengthCurrent += bundleSpaceGraph_->getBase()->distance(basePath.at(k), basePath.at(k + 1));
            }
        }
    }
    else
    {
        for (unsigned int k = 0; k < basePath.size(); k++)
        {
            bundle->copyState(bundlePath.at(k), basePath.at(k));
        }
    }
    return bundlePath;
}

BundleSpaceGraph::Configuration *
PathRestriction::addFeasibleSegment(Configuration *xLast, base::State *sNext)
{
    base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();

    Configuration *x = new Configuration(bundle, sNext);
    bundleSpaceGraph_->addConfiguration(x);
    bundleSpaceGraph_->addBundleEdge(xLast, x);

    x->parent = xLast;

#ifdef SANITY_CHECK_PATH_RESTRICTION
    if (!bundle->checkMotion(xLast->state, x->state))
    {
        OMPL_ERROR("Not feasible from last");
        std::cout << std::string(80, '-') << std::endl;
        std::cout << "Last State" << std::endl;
        bundle->printState(xLast->state);
        std::cout << std::string(80, '-') << std::endl;
        std::cout << "Current State" << std::endl;
        bundle->printState(x->state);
        throw Exception("");
    }
#endif
    return x;
}

void PathRestriction::addFeasibleGoalSegment(Configuration *const xLast,
                                                                          Configuration *const xGoal)
{
    if (xGoal->index <= 0)
    {
        bundleSpaceGraph_->vGoal_ = bundleSpaceGraph_->addConfiguration(xGoal);
    }
    bundleSpaceGraph_->addBundleEdge(xLast, xGoal);

    xGoal->parent = xLast;

#ifdef SANITY_CHECK_PATH_RESTRICTION
    base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();
    if (!bundle->checkMotion(xLast->state, xGoal->state))
    {
        OMPL_DEBUG(std::string(80, '-').c_str());
        OMPL_DEBUG("Last State");
        bundle->printState(xLast->state);
        OMPL_DEBUG(std::string(80, '-').c_str());
        OMPL_DEBUG("Current State");
        bundle->printState(xGoal->state);
        throw Exception("Infeasible goal segment.");
    }
#endif
}

bool PathRestriction::sideStepAlongFiber(const base::State *xBase, base::State *xBundle)
{
    int ctr = 0;
    bool found = false;
    while (ctr++ < 10 && !found)
    {
        // sample model fiber
        bundleSpaceGraph_->sampleFiber(xFiberStart_);
        bundleSpaceGraph_->liftState(xBase, xFiberStart_, xBundle);

        if (bundleSpaceGraph_->getBundle()->isValid(xBundle))
        {
            found = true;
        }
    }
    return found;
}

bool PathRestriction::hasFeasibleSection(
    Configuration *const xStart, 
    Configuration *const xGoal)
{
    // check for quasisection computation module
    int type = bundleSpaceGraph_->getBundle()->getStateSpace()->getType();
    if (type == base::STATE_SPACE_DUBINS)  // || type == base::STATE_SPACE_DUBINS_AIRPLANE)
    {
        // Quasisections
        bundleSpaceGraph_->projectFiber(xStart->state, xFiberStart_);
        bundleSpaceGraph_->projectFiber(xGoal->state, xFiberGoal_);
        std::vector<base::State *> section = interpolateSectionL2(xFiberStart_, xFiberGoal_, basePath_);

        Configuration *xLast = xStart;

        for (unsigned int k = 1; k < section.size(); k++)
        {
            if (bundleSpaceGraph_->getBundle()->checkMotion(section.at(k - 1), section.at(k), lastValid_))
            {
                if (k < section.size() - 1)
                {
                    xLast = addFeasibleSegment(xLast, section.at(k));
                }
                else
                {
                    if (xGoal->index <= 0)
                    {
                        bundleSpaceGraph_->vGoal_ = bundleSpaceGraph_->addConfiguration(xGoal);
                    }
                    addFeasibleGoalSegment(xLast, xGoal);
                    OMPL_DEBUG("Found feasible path section (%d edges added)", k);
                    return true;
                }
            }
            else
            {
                addFeasibleSegment(xLast, lastValid_.first);
                return false;
            }
        }
        return true;
    }
    else
    {
        bool foundFeasibleSection = checkSectionL1BacktrackRecursive(xStart, xGoal, basePath_);
        if (!foundFeasibleSection)
        {
            // Try with inverse L1
            foundFeasibleSection = checkSectionL1BacktrackRecursive(xStart, xGoal, basePath_, false);
        }

#ifdef SANITY_CHECK_PATH_RESTRICTION
        if (foundFeasibleSection)
        {
            sanityCheckSection();
        }
#endif
        return foundFeasibleSection;
    }
}

bool PathRestriction::checkSectionL2(
    Configuration *const xStart, Configuration *const xGoal)
{
    bundleSpaceGraph_->projectFiber(xStart->state, xFiberStart_);
    bundleSpaceGraph_->projectFiber(xGoal->state, xFiberGoal_);

    std::vector<base::State *> section = interpolateSectionL2(xFiberStart_, xFiberGoal_, basePath_);

    Configuration *xLast = xStart;

    base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();

    bool found = false;
    for (unsigned int k = 1; k < section.size(); k++)
    {
        if (bundle->checkMotion(section.at(k - 1), section.at(k), lastValid_))
        {
            if (k < section.size() - 1)
            {
                xLast = addFeasibleSegment(xLast, section.at(k));
            }
            else
            {
                addFeasibleGoalSegment(xLast, xGoal);
                OMPL_DEBUG("Found feasible path section (%d edges added)", k);
                found = true;
                break;
            }
        }
        else
        {
            if (lastValid_.second > 0)
            {
                // add last valid into the bundle graph
                Configuration *xk = new Configuration(bundle, lastValid_.first);
                bundleSpaceGraph_->addConfiguration(xk);
                bundleSpaceGraph_->addBundleEdge(xLast, xk);
            }

            double length = std::accumulate(intermediateLengthsBasePath_.begin(),
                                            intermediateLengthsBasePath_.begin() + (k - 1), 0.0);

            length += lastValid_.second * bundleSpaceGraph_->getBase()->distance(basePath_.at(k - 1), basePath_.at(k));

            static_cast<BundleSpaceGraph *>(bundleSpaceGraph_->getParent())
                ->getGraphSampler()
                ->setPathBiasStartSegment(length);
            break;
        }
    }
    bundle->freeStates(section);
    return found;
}

const unsigned int PATH_SECTION_L1UTURN_MAX_DEPTH = 3;
const unsigned int PATH_SECTION_L1UTURN_MAX_BRANCHING = 10;
bool PathRestriction::checkSectionL1BacktrackRecursive(
    Configuration *const xStart, 
    Configuration *const xGoal, 
    const std::vector<base::State *> basePath,
    bool interpolateFiberFirst, 
    unsigned int depth, 
    double startLength)
{
    bundleSpaceGraph_->projectFiber(xStart->state, xFiberStart_);
    bundleSpaceGraph_->projectFiber(xGoal->state, xFiberGoal_);

    std::vector<base::State *> section;
    if (interpolateFiberFirst)
    {
        section = interpolateSectionL1FF(xFiberStart_, xFiberGoal_, basePath);
    }
    else
    {
        section = interpolateSectionL1FL(xFiberStart_, xFiberGoal_, basePath);
    }

    Configuration *xLast = xStart;

    base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();
    base::SpaceInformationPtr base = bundleSpaceGraph_->getBase();
    base::SpaceInformationPtr fiber = bundleSpaceGraph_->getFiber();

    // std::cout << std::string(80, '-') << std::endl;
    // std::cout << "SECTION:" << std::endl;
    // for (unsigned int k = 0; k < section.size(); k++)
    // {
    //     bundle->printState(section.at(k));
    // }
    // std::cout << std::string(80, '-') << std::endl;
    for (unsigned int k = 1; k < section.size(); k++)
    {
        if (bundle->checkMotion(section.at(k - 1), section.at(k), lastValid_))
        {
            if (k < section.size() - 1)
            {
                xLast = addFeasibleSegment(xLast, section.at(k));
            }
            else
            {
                addFeasibleGoalSegment(xLast, xGoal);
                OMPL_DEBUG("Found feasible path section");
                bundle->freeStates(section);

                return true;
            }
        }
        else
        {
            //############################################################################
            // Get Last valid
            //############################################################################
            Configuration *xLastValid{nullptr};
            // std::cout << "Stopped at last valid location: " << 
            //   lastValid_.second << "with k=" << k <<"/"<<section.size()<< std::endl;
            if (lastValid_.second > 0)
            {
                // add last valid into the bundle graph
                xLastValid = new Configuration(bundle, lastValid_.first);
                bundleSpaceGraph_->addConfiguration(xLastValid);
                bundleSpaceGraph_->addBundleEdge(xLast, xLastValid);
                xLast = xLastValid;
            }
            else
            {
                xLastValid = xLast;
            }

            //############################################################################
            // Get length until last Valid
            //############################################################################
            double locationOnBasePath = 0.0;
            unsigned int stopK = k;
            if(interpolateFiberFirst)
            {
              //NOTE: first segment of section does not exist on base path
                stopK -= 1;

            }
            if(stopK < 1)
            {
              //no movement on base path
              locationOnBasePath = 0;
            }else{
                for (unsigned int j = 1; j < stopK; j++)
                {
                    double dj = bundleSpaceGraph_->getBase()->distance(basePath.at(j - 1), basePath.at(j));
                    locationOnBasePath += dj;
                }

                if (stopK < basePath.size())
                {
                    double dLastSegment = 
                      bundleSpaceGraph_->getBase()->distance( 
                          basePath.at(stopK - 1), 
                          basePath.at(stopK));

                    double dLocationLastSegment = lastValid_.second * dLastSegment;

                    //make one more step into the constraint
                    if( dLocationLastSegment + validSegmentLength_ < dLastSegment)
                    {
                        dLocationLastSegment += validSegmentLength_;
                    }
                    locationOnBasePath += dLocationLastSegment;
                }
            }

            static_cast<BundleSpaceGraph *>(bundleSpaceGraph_->getParent())
                ->getGraphSampler()
                ->setPathBiasStartSegment(locationOnBasePath + startLength);

            if (depth + 1 >= PATH_SECTION_L1UTURN_MAX_DEPTH)
            {
                bundle->freeStates(section);
                return false;
            }

            //############################################################################
            // Side step randomly and interpolate from there towards goal
            //############################################################################
            unsigned int lastCtr = 
              bundleSpaceGraph_->interpolateAlongBasePath(basePath, locationOnBasePath, xBaseTmp_);

            std::vector<base::State *> basePathSegment = {basePath.begin() + lastCtr, basePath.end()};
            basePathSegment.insert(basePathSegment.begin(), xBaseTmp_);

            for (unsigned int j = 0; j < PATH_SECTION_L1UTURN_MAX_BRANCHING; j++)
            {
                //#############################################################
                // find feasible sample in current fiber
                //#############################################################
                if (!sideStepAlongFiber(xBaseTmp_, xBundleTmp_))
                {
                    continue;
                }

                //#############################################################
                // Make Uturn
                //#############################################################
                if (bundle->checkMotion(xLastValid->state, xBundleTmp_))
                {
                    Configuration *xSideStep = new Configuration(bundle, xBundleTmp_);
                    bundleSpaceGraph_->addConfiguration(xSideStep);
                    bundleSpaceGraph_->addBundleEdge(xLastValid, xSideStep);

                    //#########################################################
                    // side step was successful.
                    // Now interpolate from there to goal
                    //#########################################################

                    bool feasibleSection = checkSectionL1BacktrackRecursive(
                        xSideStep, xGoal, basePathSegment, 
                        !interpolateFiberFirst, depth + 1, locationOnBasePath);

                    if (feasibleSection)
                    {
                        bundle->freeStates(section);
                        return true;
                    }
                }else
                {
                    base::State* xBundleTest = bundle->allocState();
                    base::State* xBundleOpening = bundle->allocState();
                    base::State* xBase = base->cloneState(xBaseTmp_);

                    base::State* xFiberTest = xFiberStart_;
                    base::State* xFiberOpening = xFiberGoal_;

                    bundleSpaceGraph_->projectFiber(xLast->state, xFiberTest);
                    bundleSpaceGraph_->projectFiber(xBundleTmp_, xFiberOpening);

                    bool found = false;

                    double location = locationOnBasePath - validSegmentLength_;

                    //mid point heuristic 
                    fiber->getStateSpace()->interpolate(xFiberTest, xFiberOpening, 0.5, xFiberTest);

                    while(!found && location >= 0)
                    {
                        bundleSpaceGraph_->interpolateAlongBasePath(
                            basePath, location, xBase);

                        bundleSpaceGraph_->liftState(xBase, xFiberTest, xBundleTest);

                        if (bundle->isValid(xBundleTest))
                        {
                            //revert xBundleTest back to crevice fiber position
                            bundleSpaceGraph_->projectFiber(xLast->state, xFiberTest);
                            bundleSpaceGraph_->liftState(xBase, xFiberTest, xBundleTest);
                            bundleSpaceGraph_->liftState(xBase, xFiberOpening, xBundleOpening);
                            if(bundle->checkMotion(xBundleTest, xBundleOpening))
                            {
                                found = true;
                                break;
                            }
                        }

                        location -= validSegmentLength_;
                    }

                    if(found)
                    {

                        ///Create UTurn step sequence
                        // xBackStep <------- xBundleCrevice (xLastValid)
                        //     |
                        //     |
                        //     |
                        //     v
                        // xSideStep -------> xBundleOpening
                        Configuration *xBackStep = new Configuration(bundle, xBundleTest);
                        bundleSpaceGraph_->addConfiguration(xBackStep);
                        bundleSpaceGraph_->addBundleEdge(xLastValid, xBackStep);

                        Configuration *xSideStep = new Configuration(bundle, xBundleOpening);
                        bundleSpaceGraph_->addConfiguration(xSideStep);
                        bundleSpaceGraph_->addBundleEdge(xBackStep, xSideStep);

                        //xBaseTmp_ is on last valid fiber. 
                        bundleSpaceGraph_->liftState(xBaseTmp_, xFiberOpening, xBundleOpening);
                        Configuration *xOpening = new Configuration(bundle, xBundleOpening);
                        bundleSpaceGraph_->addConfiguration(xOpening);
                        bundleSpaceGraph_->addBundleEdge(xSideStep, xOpening);

                        std::cout << "INSERT UTURN" << std::endl;
                        bundle->printState(xLastValid->state);
                        bundle->printState(xBackStep->state);
                        bundle->printState(xSideStep->state);
                        bundle->printState(xOpening->state);

                        //Continue with fiber last interpolation. If there is an opening in
                        //geometry, it seems likely to be a passage -> so move
                        //straight (regardless of what happened before)
                        bool feasibleSection = checkSectionL1BacktrackRecursive(
                            xOpening, xGoal, basePathSegment, 
                            false, depth + 1, locationOnBasePath);

                        if (feasibleSection)
                        {
                            bundle->freeState(xBundleOpening);
                            bundle->freeState(xBundleTest);
                            base->freeState(xBase);
                            bundle->freeStates(section);
                            return true;
                        }
                    }
                    bundle->freeState(xBundleOpening);
                    bundle->freeState(xBundleTest);
                    base->freeState(xBase);
                }
            }

            break;
        }
    }
    bundle->freeStates(section);
    return false;
}

const unsigned int PATH_SECTION_TREE_MAX_DEPTH = 3;
const unsigned int PATH_SECTION_TREE_MAX_BRANCHING = 10;

bool PathRestriction::checkSectionL1Recursive(
    Configuration *const xStart, Configuration *const xGoal, const std::vector<base::State *> basePath,
    bool interpolateFiberFirst, unsigned int depth, double startLength)
{
    bundleSpaceGraph_->projectFiber(xStart->state, xFiberStart_);
    bundleSpaceGraph_->projectFiber(xGoal->state, xFiberGoal_);

    base::SpaceInformationPtr bundle = bundle;

    std::vector<base::State *> section;
    if (interpolateFiberFirst)
    {
        section = interpolateSectionL1FF(xFiberStart_, xFiberGoal_, basePath);
    }
    else
    {
        section = interpolateSectionL1FL(xFiberStart_, xFiberGoal_, basePath);
    }

    Configuration *xLast = xStart;

    for (unsigned int k = 1; k < section.size(); k++)
    {
        if (bundle->checkMotion(section.at(k - 1), section.at(k), lastValid_))
        {
            if (k < section.size() - 1)
            {
                xLast = addFeasibleSegment(xLast, section.at(k));
            }
            else
            {
                addFeasibleGoalSegment(xLast, xGoal);
                OMPL_DEBUG("Found feasible path section (%d edges added)", k);
                bundle->freeStates(section);

                return true;
            }
        }
        else
        {
            //############################################################################
            // Get Last valid
            //############################################################################
            Configuration *xLastValid{nullptr};
            if (lastValid_.second > 0)
            {
                // add last valid into the bundle graph
                xLastValid = new Configuration(bundle, lastValid_.first);
                bundleSpaceGraph_->addConfiguration(xLastValid);
                bundleSpaceGraph_->addBundleEdge(xLast, xLastValid);
                xLast = xLastValid;
            }
            else
            {
                xLastValid = xLast;
            }

            //############################################################################
            // Get length until last Valid
            //############################################################################
            //OLD VERSION (need testing)
            // double locationOnBasePath = 0.0;
            // for (unsigned int j = 1; j < k; j++)
            // {
            //     double dj = bundleSpaceGraph_->getBase()->distance(basePath.at(j - 1), basePath.at(j));
            //     locationOnBasePath += dj;
            // }

            // if (k < basePath.size())
            // {
            //     locationOnBasePath +=
            //         lastValid_.second * bundleSpaceGraph_->getBase()->distance(basePath.at(k - 1), basePath.at(k));
            // }

            double locationOnBasePath = 0.0;
            unsigned int stopK = k;
            if(interpolateFiberFirst)
            {
                stopK -= 1;
            }
            for (unsigned int j = 1; j < stopK; j++)
            {
                double dj = bundleSpaceGraph_->getBase()->distance(basePath.at(j - 1), basePath.at(j));
                locationOnBasePath += dj;
            }

            if (stopK < basePath.size())
            {
                locationOnBasePath +=
                    lastValid_.second * bundleSpaceGraph_->getBase()->distance(basePath.at(stopK - 1), basePath.at(stopK));
            }




            static_cast<BundleSpaceGraph *>(bundleSpaceGraph_->getParent())
                ->getGraphSampler()
                ->setPathBiasStartSegment(locationOnBasePath + startLength);

            if (depth + 1 >= PATH_SECTION_TREE_MAX_DEPTH)
            {
                bundle->freeStates(section);
                return false;
            }

            //############################################################################
            // Side step randomly and interpolate from there towards goal
            //############################################################################
            unsigned int lastCtr = bundleSpaceGraph_->interpolateAlongBasePath(basePath, locationOnBasePath, xBaseTmp_);

            std::vector<base::State *> basePathSegment = {basePath.begin() + lastCtr, basePath.end()};
            basePathSegment.insert(basePathSegment.begin(), xBaseTmp_);

            for (unsigned int j = 0; j < PATH_SECTION_TREE_MAX_BRANCHING; j++)
            {
                //#############################################################
                // find feasible sample in current fiber
                //#############################################################
                if (!sideStepAlongFiber(xBaseTmp_, xBundleTmp_))
                    continue;

                //#############################################################
                // check that we can connect new sample with last states
                //#############################################################
                if (bundle->checkMotion(xLastValid->state, xBundleTmp_))
                {
                    Configuration *xSideStep = new Configuration(bundle, xBundleTmp_);
                    bundleSpaceGraph_->addConfiguration(xSideStep);
                    bundleSpaceGraph_->addBundleEdge(xLastValid, xSideStep);

                    //#########################################################
                    // side step was successful.
                    // Now interpolate from there to goal
                    //#########################################################

                    bool feasibleSection = checkSectionL1Recursive(xSideStep, xGoal, basePathSegment,
                                                                       !interpolateFiberFirst, depth + 1, locationOnBasePath);

                    if (feasibleSection)
                    {
                        bundle->freeStates(section);
                        return true;
                    }
                }
            }

            break;
        }
    }
    bundle->freeStates(section);
    return false;
}

void PathRestriction::sanityCheckSection()
{
    if (!bundleSpaceGraph_->sameComponent(bundleSpaceGraph_->vStart_, bundleSpaceGraph_->vGoal_))
    {
        throw Exception("Reported feasible path section, \
            but start and goal are in different components.");
    }

    base::PathPtr solutionPath = bundleSpaceGraph_->getPath(bundleSpaceGraph_->vStart_, bundleSpaceGraph_->vGoal_);

    if (solutionPath == nullptr)
    {
        Configuration *q = bundleSpaceGraph_->qGoal_;
        bundleSpaceGraph_->printConfiguration(q);
        while (q->parent != nullptr)
        {
            bundleSpaceGraph_->printConfiguration(q);
            q = q->parent;
        }

        if (q != bundleSpaceGraph_->qStart_)
        {
            std::cout << bundleSpaceGraph_->getName() << " failed on level " << bundleSpaceGraph_->getLevel() << " dim "
                      << bundleSpaceGraph_->getBundleDimension() << "->" << bundleSpaceGraph_->getBaseDimension()
                      << std::endl;
            throw Exception("Reported feasible path section, \
                but path section is not existent.");
        }
    }

    geometric::PathGeometric &gpath = static_cast<geometric::PathGeometric &>(*solutionPath);

    bool valid = gpath.check();
    if (!valid)
    {
        OMPL_ERROR("Path section is invalid.");
        std::vector<base::State *> gStates = gpath.getStates();
        for (unsigned int k = 1; k < gStates.size(); k++)
        {
            base::State *sk1 = gStates.at(k - 1);
            base::State *sk2 = gStates.at(k - 2);
            if (!bundleSpaceGraph_->getBundle()->checkMotion(sk1, sk2))
            {
                std::cout << "Error between states " << k - 1 << " and " << k << std::endl;
            }
        }
        throw Exception("Reported feasible path section, \
            but path section is infeasible.");
    }
}
