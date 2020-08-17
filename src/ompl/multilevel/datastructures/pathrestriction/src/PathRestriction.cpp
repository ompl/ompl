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
#include <ompl/multilevel/datastructures/pathrestriction/PathSection.h>
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

BundleSpaceGraph* PathRestriction::getBundleSpaceGraph()
{
  return bundleSpaceGraph_;
}

void PathRestriction::setBasePath(ompl::base::PathPtr path)
{
    geometric::PathGeometricPtr geometricBasePath = 
      std::static_pointer_cast<geometric::PathGeometric>(path);
    setBasePath(geometricBasePath->getStates());
}

void PathRestriction::setBasePath(std::vector<base::State *> basePath)
{
    basePath_ = basePath;

    lengthBasePath_ = 0.0;

    lengthsIntermediateBasePath_.clear();
    lengthsCumulativeBasePath_.clear();
    for (unsigned int k = 1; k < basePath_.size(); k++)
    {
        double lk = bundleSpaceGraph_->getBase()->distance(basePath_.at(k - 1), basePath_.at(k));
        lengthsIntermediateBasePath_.push_back(lk);
        lengthBasePath_ += lk;
        lengthsCumulativeBasePath_.push_back(lengthBasePath_);
    }
    OMPL_DEBUG("Set new base path with %d states and length %f.", basePath_.size(), lengthBasePath_);
}

const std::vector<ompl::base::State*> PathRestriction::getBasePath()
{
  return basePath_;
}

double PathRestriction::getLengthBasePath()
{
  return lengthBasePath_;
}
//distance between base states k and k+1
double PathRestriction::getLengthIntermediateBasePath(int k)
{
  return lengthsIntermediateBasePath_.at(k);
}

double PathRestriction::getLengthBasePathUntil(int k)
{
  if(k <= 0) return 0;
  else return lengthsCumulativeBasePath_.at(k-1);
}

bool PathRestriction::findFeasibleStateOnFiber(const base::State *xBase, base::State *xBundle)
{
    int ctr = 0;
    bool found = false;
    while (ctr++ < 10 && !found)
    {
        // sample model fiber
        bundleSpaceGraph_->sampleFiber(xFiberTmp_);
        bundleSpaceGraph_->liftState(xBase, xFiberTmp_, xBundle);

        if (bundleSpaceGraph_->getBundle()->isValid(xBundle))
        {
            found = true;
        }
    }
    return found;
}

bool PathRestriction::tripleStep(
    Configuration* &xBundleLastValid, 
    const base::State *sBundleGoal, 
    base::State *sBase,
    double startLocation)
{
    base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();
    base::SpaceInformationPtr base = bundleSpaceGraph_->getBase();
    base::SpaceInformationPtr fiber = bundleSpaceGraph_->getFiber();

    //Triple step connection attempt
    // xBundleStartTmp <------- xBundleStart
    //     |
    //     |
    //     |
    //     v
    // xBundleGoalTmp -------> xBundleGoal

    base::State* xBundleStartTmp = bundle->allocState();
    base::State* xBundleGoalTmp = bundle->allocState();
    base::State* xBase = base->cloneState(sBase);

    bundleSpaceGraph_->projectFiber(xBundleLastValid->state, xFiberStart_);
    bundleSpaceGraph_->projectFiber(sBundleGoal, xFiberGoal_);

    bool found = false;

    //mid point heuristic 
    fiber->getStateSpace()->interpolate(xFiberStart_, xFiberGoal_, 0.5, xFiberTmp_);

    double location = startLocation - validSegmentLength_;

    while(!found && location >= 0)
    {
        bundleSpaceGraph_->interpolateAlongBasePath(
            basePath_, location, xBase);

        bundleSpaceGraph_->liftState(xBase, xFiberTmp_, xBundleStartTmp);

        if (bundle->isValid(xBundleStartTmp))
        {
            bundleSpaceGraph_->liftState(xBase, xFiberStart_, xBundleStartTmp);
            bundleSpaceGraph_->liftState(xBase, xFiberGoal_, xBundleGoalTmp);
            if(bundle->checkMotion(xBundleStartTmp, xBundleGoalTmp))
            {
                if(bundle->checkMotion(xBundleGoalTmp, sBundleGoal))
                {
                    found = true;
                    break;
                }
            }
        }

        location -= validSegmentLength_;
    }

    if(found)
    {
        Configuration *xBackStep = new Configuration(bundle, xBundleStartTmp);
        bundleSpaceGraph_->addConfiguration(xBackStep);
        bundleSpaceGraph_->addBundleEdge(xBundleLastValid, xBackStep);

        Configuration *xSideStep = new Configuration(bundle, xBundleGoalTmp);
        bundleSpaceGraph_->addConfiguration(xSideStep);
        bundleSpaceGraph_->addBundleEdge(xBackStep, xSideStep);

        //xBaseTmp_ is on last valid fiber. 
        Configuration *xGoal = new Configuration(bundle, sBundleGoal);
        bundleSpaceGraph_->addConfiguration(xGoal);
        bundleSpaceGraph_->addBundleEdge(xSideStep, xGoal);

        std::cout << "INSERT TRIPLE STEP (BWD-SIDE-FWD)" << std::endl;
        bundle->printState(xBundleLastValid->state);
        bundle->printState(xBackStep->state);
        bundle->printState(xSideStep->state);
        bundle->printState(xGoal->state);

        xBundleLastValid = xGoal;
    }

    bundle->freeState(xBundleStartTmp);
    bundle->freeState(xBundleGoalTmp);
    base->freeState(xBase);
    return found;
}

bool PathRestriction::sideStepAlongFiber(Configuration *xOrigin, base::State *state)
{
    base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();
    if (bundle->checkMotion(xOrigin->state, state))
    {
        //#########################################################
        // side step was successful.
        // Now interpolate from there to goal
        //#########################################################

        Configuration *xSideStep = new Configuration(bundle, state);
        bundleSpaceGraph_->addConfiguration(xSideStep);
        bundleSpaceGraph_->addBundleEdge(xOrigin, xSideStep);

        xOrigin = xSideStep;

        return true;

    }
    return false;
}

bool PathRestriction::wriggleFree(
    const base::State *xBundleOrigin, 
    const base::State *xBundleGoal, 
    base::State *xBundleBestState,
    double &distProgress)
{

    const ompl::base::StateSamplerPtr bundleSampler = bundleSpaceGraph_->getBundleSamplerPtr();
    ompl::base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();

    int ctr = 0;

    base::State* xBundleNext = bundle->allocState();
    double originDist = bundle->distance(xBundleOrigin, xBundleGoal);
    double bestDist = originDist;

    double epsilon = 0.1;
    std::cout << "Start wriggling" << std::endl;
    std::cout << "ORIGIN dist " << bestDist << std::endl;
    bundle->printState(xBundleOrigin);
    while (ctr++ < 10)
    {
        bundleSampler->sampleUniformNear(xBundleNext, xBundleOrigin, epsilon);
        double dist = bundle->distance(xBundleNext, xBundleGoal);
        if(dist < bestDist && 
            bundle->isValid(xBundleNext) &&
            bundle->checkMotion(xBundleOrigin, xBundleNext))
        {
            //set cur to next. Continue from there
            bundle->copyState(xBundleBestState, xBundleNext);
            bestDist = dist;
            std::cout << "Improved state to " << "bestDist:" << bestDist << std::endl;
            bundle->printState(xBundleBestState);
        }

    }
    bundle->freeState(xBundleNext);

    distProgress = originDist - bestDist;

    std::cout << "Progress: " << distProgress << std::endl;

    return (distProgress > 0);
}

bool PathRestriction::hasFeasibleSection(
    Configuration *const xStart, 
    Configuration *const xGoal)
{
    bool foundFeasibleSection = checkSectionL1BacktrackRecursive(xStart, xGoal, basePath_);
    if (!foundFeasibleSection)
    {
        // Try with inverse L1
        foundFeasibleSection = checkSectionL1BacktrackRecursive(xStart, xGoal, basePath_, false);
    }

    return foundFeasibleSection;
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
    base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();
    base::SpaceInformationPtr base = bundleSpaceGraph_->getBase();
    base::SpaceInformationPtr fiber = bundleSpaceGraph_->getFiber();

    bundleSpaceGraph_->projectFiber(xStart->state, xFiberStart_);
    bundleSpaceGraph_->projectFiber(xGoal->state, xFiberGoal_);

    PathSectionPtr section = std::make_shared<PathSection>(this);

    if (interpolateFiberFirst)
    {
        section->interpolateL1FiberFirst(xFiberStart_, xFiberGoal_);
    }
    else
    {
        section->interpolateL1FiberLast(xFiberStart_, xFiberGoal_);
    }

    if(section->checkMotion(xStart, xGoal, lastValid_))
    {
        return true;
    }

    if (depth >= PATH_SECTION_L1UTURN_MAX_DEPTH)
    {
        return false;
    }

    Configuration *xLastValid = section->getLastValidConfiguration();

    //############################################################################
    //Clip base path to start at last valid
    //############################################################################
    bundleSpaceGraph_->projectBase(xLastValid->state, xBaseTmp_);

    int lastCtr = section->getLastValidBasePathIndex();

    std::vector<base::State *> basePathSegment = {basePath.begin() + lastCtr + 1, basePath.end()};

    basePathSegment.insert(basePathSegment.begin(), xBaseTmp_);

    double locationOnBasePath = section->getLastValidBasePathLocation();

    //############################################################################
    //Try different strategies to locally resolve constraint violation
    //Then call function recursively with clipped base path
    //############################################################################

    static_cast<BundleSpaceGraph *>(bundleSpaceGraph_->getParent())
       ->getGraphSampler()
       ->setPathBiasStartSegment(locationOnBasePath + startLength);

    for (unsigned int j = 0; j < PATH_SECTION_L1UTURN_MAX_BRANCHING; j++)
    {
        //#####################################################################
        // (1) Try wriggling to escape crevice
        //#####################################################################
       
        //#####################################################################
        // (2) Find other opening to traverse path restriction.
        //
        // find feasible sample xBundleTmp_ in fiber space over the last valid
        // base path state
        //#############################################################
        if (!findFeasibleStateOnFiber(xBaseTmp_, xBundleTmp_))
        {
            continue;
        }

        //#############################################################
        // (2a) Use other opening to sidestep
        //#############################################################
        bool foundAlternative = false;

        if(sideStepAlongFiber(xLastValid, xBundleTmp_))
        {
          foundAlternative = true;
        }else{
            std::cout << "Triple stepping:" << std::endl;
            if(tripleStep(xLastValid, xBundleTmp_, xBaseTmp_, locationOnBasePath))
            {
              foundAlternative = true;
            }
        }

        if(foundAlternative)
        {
            std::cout << "Go recursive:" << std::endl;
            bool feasibleSection = checkSectionL1BacktrackRecursive(
                xLastValid, xGoal, basePathSegment, 
                false, depth + 1, locationOnBasePath);

            return feasibleSection;
        }
    }
    return false;
}


    //Configuration *xLast = xStart;

    //base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();
    //base::SpaceInformationPtr base = bundleSpaceGraph_->getBase();
    //base::SpaceInformationPtr fiber = bundleSpaceGraph_->getFiber();

    //for (unsigned int k = 1; k < section.size(); k++)
    //{
    //    if (bundle->checkMotion(section.at(k - 1), section.at(k), lastValid_))
    //    {
    //        if (k < section.size() - 1)
    //        {
    //            xLast = addFeasibleSegment(xLast, section.at(k));
    //        }
    //        else
    //        {
    //          //TODO: what if xLast does NOT equal section.at(k-1)?
    //            addFeasibleGoalSegment(xLast, xGoal);
    //            OMPL_DEBUG("Found feasible path section");
    //            bundle->freeStates(section);

    //            return true;
    //        }
    //    }
    //    else
    //    {
    //        //############################################################################
    //        // Get Last valid
    //        //############################################################################
    //        Configuration *xLastValid{nullptr};
    //        // std::cout << "Stopped at last valid location: " << 
    //        //   lastValid_.second << "with k=" << k <<"/"<<section.size()<< std::endl;
    //        if (lastValid_.second > 0)
    //        {
    //            // add last valid into the bundle graph
    //            xLastValid = new Configuration(bundle, lastValid_.first);
    //            bundleSpaceGraph_->addConfiguration(xLastValid);
    //            bundleSpaceGraph_->addBundleEdge(xLast, xLastValid);
    //            xLast = xLastValid;
    //        }
    //        else
    //        {
    //            xLastValid = xLast;
    //        }

    //        //############################################################################
    //        // Get length until last Valid
    //        //############################################################################
    //        double locationOnBasePath = 0.0;
    //        unsigned int stopK = k;
    //        if(interpolateFiberFirst)
    //        {
    //          //NOTE: first segment of section does not exist on base path
    //            stopK -= 1;

    //        }
    //        if(stopK < 1)
    //        {
    //          //no movement on base path
    //          locationOnBasePath = 0;
    //        }else{
    //            for (unsigned int j = 1; j < stopK; j++)
    //            {
    //                double dj = bundleSpaceGraph_->getBase()->distance(basePath.at(j - 1), basePath.at(j));
    //                locationOnBasePath += dj;
    //            }

    //            if (stopK < basePath.size())
    //            {
    //                double dLastSegment = 
    //                  bundleSpaceGraph_->getBase()->distance( 
    //                      basePath.at(stopK - 1), 
    //                      basePath.at(stopK));

    //                double dLocationLastSegment = lastValid_.second * dLastSegment;

    //                //make one more step into the constraint
    //                if( dLocationLastSegment + validSegmentLength_ < dLastSegment)
    //                {
    //                    dLocationLastSegment += validSegmentLength_;
    //                }
    //                locationOnBasePath += dLocationLastSegment;
    //            }
    //        }

    //        static_cast<BundleSpaceGraph *>(bundleSpaceGraph_->getParent())
    //            ->getGraphSampler()
    //            ->setPathBiasStartSegment(locationOnBasePath + startLength);

    //        if (depth + 1 >= PATH_SECTION_L1UTURN_MAX_DEPTH)
    //        {
    //            return false;
    //        }

    //        //############################################################################
    //        // Clip base path segment to last valid
    //        //############################################################################

    //        bundle->projectBase(xLastValid->state, xBaseTmp_)

    //        int lastCtr = section->getLastValidBasePathIndex();

    //        std::vector<base::State *> basePathSegment = 
    //        {basePath.begin() + lastCtr + 1, basePath.end()};

    //        basePathSegment.insert(basePathSegment.begin(), xBaseTmp_);

    //        //double dist = 0;
    //        //if(wriggleFree(lastValid_.first, section.at(stopK+1), xBundleTmp_, dist))
    //        //{
    //        //    std::cout << "Successful wriggle step" << std::endl;
    //        //    //If wriggle succeeds, we are likely not in a crevice-like
    //        //    //structure in cspace, but we can continue
    //        //    Configuration *xContinue = new Configuration(bundle, xBundleTmp_);
    //        //    bundleSpaceGraph_->addConfiguration(xContinue);
    //        //    bundleSpaceGraph_->addBundleEdge(xLastValid, xContinue);

    //        //    bool feasibleSection = checkSectionL1BacktrackRecursive(
    //        //        xContinue, xGoal, basePathSegment, 
    //        //        false, depth + 1, locationOnBasePath + dist);

    //        //    bundle->freeStates(section);

    //        //    if (feasibleSection)
    //        //    {
    //        //        return true;
    //        //    }
    //        //    return false;
    //        //}


    //        break;
    //    }
    //}
    //bundle->freeStates(section);
    //return false;

//const unsigned int PATH_SECTION_TREE_MAX_DEPTH = 3;
//const unsigned int PATH_SECTION_TREE_MAX_BRANCHING = 10;

//bool PathRestriction::checkSectionL1Recursive(
//    Configuration *const xStart, Configuration *const xGoal, const std::vector<base::State *> basePath,
//    bool interpolateFiberFirst, unsigned int depth, double startLength)
//{
//    bundleSpaceGraph_->projectFiber(xStart->state, xFiberStart_);
//    bundleSpaceGraph_->projectFiber(xGoal->state, xFiberGoal_);

//    base::SpaceInformationPtr bundle = bundle;

//    std::vector<base::State *> section;
//    if (interpolateFiberFirst)
//    {
//        section = interpolateSectionL1FF(xFiberStart_, xFiberGoal_, basePath);
//    }
//    else
//    {
//        section = interpolateSectionL1FL(xFiberStart_, xFiberGoal_, basePath);
//    }

//    Configuration *xLast = xStart;

//    for (unsigned int k = 1; k < section.size(); k++)
//    {
//        if (bundle->checkMotion(section.at(k - 1), section.at(k), lastValid_))
//        {
//            if (k < section.size() - 1)
//            {
//                xLast = addFeasibleSegment(xLast, section.at(k));
//            }
//            else
//            {
//                addFeasibleGoalSegment(xLast, xGoal);
//                OMPL_DEBUG("Found feasible path section (%d edges added)", k);
//                bundle->freeStates(section);

//                return true;
//            }
//        }
//        else
//        {
//            //############################################################################
//            // Get Last valid
//            //############################################################################
//            Configuration *xLastValid{nullptr};
//            if (lastValid_.second > 0)
//            {
//                // add last valid into the bundle graph
//                xLastValid = new Configuration(bundle, lastValid_.first);
//                bundleSpaceGraph_->addConfiguration(xLastValid);
//                bundleSpaceGraph_->addBundleEdge(xLast, xLastValid);
//                xLast = xLastValid;
//            }
//            else
//            {
//                xLastValid = xLast;
//            }

//            //############################################################################
//            // Get length until last Valid
//            //############################################################################

//            double locationOnBasePath = 0.0;
//            unsigned int stopK = k;
//            if(interpolateFiberFirst)
//            {
//                stopK -= 1;
//            }
//            for (unsigned int j = 1; j < stopK; j++)
//            {
//                double dj = bundleSpaceGraph_->getBase()->distance(basePath.at(j - 1), basePath.at(j));
//                locationOnBasePath += dj;
//            }

//            if (stopK < basePath.size())
//            {
//                locationOnBasePath +=
//                    lastValid_.second * bundleSpaceGraph_->getBase()->distance(basePath.at(stopK - 1), basePath.at(stopK));
//            }

//            static_cast<BundleSpaceGraph *>(bundleSpaceGraph_->getParent())
//                ->getGraphSampler()
//                ->setPathBiasStartSegment(locationOnBasePath + startLength);

//            if (depth + 1 >= PATH_SECTION_TREE_MAX_DEPTH)
//            {
//                bundle->freeStates(section);
//                return false;
//            }

//            //############################################################################
//            // Side step randomly and interpolate from there towards goal
//            //############################################################################
//            unsigned int lastCtr = bundleSpaceGraph_->interpolateAlongBasePath(basePath, locationOnBasePath, xBaseTmp_);

//            std::vector<base::State *> basePathSegment = {basePath.begin() + lastCtr, basePath.end()};
//            basePathSegment.insert(basePathSegment.begin(), xBaseTmp_);

//            for (unsigned int j = 0; j < PATH_SECTION_TREE_MAX_BRANCHING; j++)
//            {
//                //#############################################################
//                // find feasible sample in current fiber
//                //#############################################################
//                if (!sideStepAlongFiber(xBaseTmp_, xBundleTmp_))
//                    continue;

//                //#############################################################
//                // check that we can connect new sample with last states
//                //#############################################################
//                if (bundle->checkMotion(xLastValid->state, xBundleTmp_))
//                {
//                    Configuration *xSideStep = new Configuration(bundle, xBundleTmp_);
//                    bundleSpaceGraph_->addConfiguration(xSideStep);
//                    bundleSpaceGraph_->addBundleEdge(xLastValid, xSideStep);

//                    //#########################################################
//                    // side step was successful.
//                    // Now interpolate from there to goal
//                    //#########################################################

//                    bool feasibleSection = checkSectionL1Recursive(xSideStep, xGoal, basePathSegment,
//                                                                       !interpolateFiberFirst, depth + 1, locationOnBasePath);

//                    if (feasibleSection)
//                    {
//                        bundle->freeStates(section);
//                        return true;
//                    }
//                }
//            }

//            break;
//        }
//    }
//    bundle->freeStates(section);
//    return false;
//}

// bool PathRestriction::checkSectionL2(
//     Configuration *const xStart, Configuration *const xGoal)
// {
//     bundleSpaceGraph_->projectFiber(xStart->state, xFiberStart_);
//     bundleSpaceGraph_->projectFiber(xGoal->state, xFiberGoal_);

//     std::vector<base::State *> section = interpolateSectionL2(xFiberStart_, xFiberGoal_, basePath_);

//     Configuration *xLast = xStart;

//     base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();

//     bool found = false;
//     for (unsigned int k = 1; k < section.size(); k++)
//     {
//         if (bundle->checkMotion(section.at(k - 1), section.at(k), lastValid_))
//         {
//             if (k < section.size() - 1)
//             {
//                 xLast = addFeasibleSegment(xLast, section.at(k));
//             }
//             else
//             {
//                 addFeasibleGoalSegment(xLast, xGoal);
//                 OMPL_DEBUG("Found feasible path section (%d edges added)", k);
//                 found = true;
//                 break;
//             }
//         }
//         else
//         {
//             if (lastValid_.second > 0)
//             {
//                 // add last valid into the bundle graph
//                 Configuration *xk = new Configuration(bundle, lastValid_.first);
//                 bundleSpaceGraph_->addConfiguration(xk);
//                 bundleSpaceGraph_->addBundleEdge(xLast, xk);
//             }

//             double length = std::accumulate(intermediateLengthsBasePath_.begin(),
//                                             intermediateLengthsBasePath_.begin() + (k - 1), 0.0);

//             length += lastValid_.second * bundleSpaceGraph_->getBase()->distance(basePath_.at(k - 1), basePath_.at(k));

//             static_cast<BundleSpaceGraph *>(bundleSpaceGraph_->getParent())
//                 ->getGraphSampler()
//                 ->setPathBiasStartSegment(length);
//             break;
//         }
//     }
//     bundle->freeStates(section);
//     return found;
// }

    // check for quasisection computation module
    // int type = bundleSpaceGraph_->getBundle()->getStateSpace()->getType();
    // if (type == base::STATE_SPACE_DUBINS)  // || type == base::STATE_SPACE_DUBINS_AIRPLANE)
    // {
    //     // Quasisections
    //     bundleSpaceGraph_->projectFiber(xStart->state, xFiberStart_);
    //     bundleSpaceGraph_->projectFiber(xGoal->state, xFiberGoal_);
    //     std::vector<base::State *> section = interpolateSectionL2(xFiberStart_, xFiberGoal_, basePath_);

    //     Configuration *xLast = xStart;

    //     for (unsigned int k = 1; k < section.size(); k++)
    //     {
    //         if (bundleSpaceGraph_->getBundle()->checkMotion(section.at(k - 1), section.at(k), lastValid_))
    //         {
    //             if (k < section.size() - 1)
    //             {
    //                 xLast = addFeasibleSegment(xLast, section.at(k));
    //             }
    //             else
    //             {
    //                 if (xGoal->index <= 0)
    //                 {
    //                     bundleSpaceGraph_->vGoal_ = bundleSpaceGraph_->addConfiguration(xGoal);
    //                 }
    //                 addFeasibleGoalSegment(xLast, xGoal);
    //                 OMPL_DEBUG("Found feasible path section (%d edges added)", k);
    //                 return true;
    //             }
    //         }
    //         else
    //         {
    //             addFeasibleSegment(xLast, lastValid_.first);
    //             return false;
    //         }
    //     }
    //     return true;
    // }
    // else
    // {
        // bool foundFeasibleSection = checkSectionL1BacktrackRecursive(xStart, xGoal, basePath_);
        // if (!foundFeasibleSection)
        // {
        //     // Try with inverse L1
        //     foundFeasibleSection = checkSectionL1BacktrackRecursive(xStart, xGoal, basePath_, false);
        // }

        // return foundFeasibleSection;
    // }
