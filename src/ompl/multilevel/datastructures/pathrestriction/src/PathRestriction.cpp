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
#include <ompl/multilevel/datastructures/pathrestriction/BasePathHead.h>
#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathSection.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <ompl/base/Path.h>
#include <ompl/geometric/PathGeometric.h>
#include <numeric>

namespace ompl
{
    namespace magic
    {
        static const unsigned int PATH_SECTION_MAX_WRIGGLING = 20;
        static const unsigned int PATH_SECTION_MAX_DEPTH = 3;
        static const unsigned int PATH_SECTION_MAX_BRANCHING = 20;
    }
}


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

const std::vector<ompl::base::State*>& PathRestriction::getBasePath()
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

bool PathRestriction::findFeasibleStateOnFiber(
    const base::State *xBase, 
    base::State *xBundle)
{
    unsigned int ctr = 0;
    bool found = false;
    base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();
    while (ctr++ < 10 && !found)
    {
        // sample model fiber
        bundleSpaceGraph_->sampleFiber(xFiberTmp_);
        bundleSpaceGraph_->liftState(xBase, xFiberTmp_, xBundle);

        //New sample must be valid AND not reachable from last valid
        if (bundle->isValid(xBundle))
        {
            found = true;
        }
    }
    return found;
}

bool PathRestriction::tripleStep(
    Configuration* &xBundleLastValid, 
    const base::State *sBundleGoal, 
    const base::State *sBaseLastValid,
    const base::State *sBasePrevious)
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
    base::State* xBase = base->cloneState(sBaseLastValid);
    const base::State *sBundleStart = xBundleLastValid->state; 

    bundleSpaceGraph_->projectFiber(sBundleStart, xFiberStart_);
    bundleSpaceGraph_->projectFiber(sBundleGoal, xFiberGoal_);

    bool found = false;

    //mid point heuristic 
    fiber->getStateSpace()->interpolate(xFiberStart_, xFiberGoal_, 0.5, xFiberTmp_);

    double dist = base->distance(sBasePrevious, sBaseLastValid);

    double location = dist - validSegmentLength_;

    //try until last milestone
    while(!found && location >= 0)
    {
        base->getStateSpace()->interpolate(sBasePrevious, sBaseLastValid, location, xBase);

        bundleSpaceGraph_->liftState(xBase, xFiberTmp_, xBundleStartTmp);

        if (bundle->isValid(xBundleStartTmp))
        {
            bundleSpaceGraph_->liftState(xBase, xFiberStart_, xBundleStartTmp);
            bundleSpaceGraph_->liftState(xBase, xFiberGoal_, xBundleGoalTmp);
            if(!bundle->isValid(xBundleStartTmp)
                || !bundle->isValid(xBundleGoalTmp))
            {
                //if those elements are infeasible, then everything is
                //infeasible. Break.
                break;
            }
            if(bundle->checkMotion(xBundleStartTmp, xBundleGoalTmp))
            {
                //need to be connectable
                if(bundle->checkMotion(xBundleGoalTmp, sBundleGoal)
                    && bundle->checkMotion(xBundleStartTmp, sBundleStart))
                {
                    found = true;
                }
                break;
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

        // std::cout << "INSERT TRIPLE STEP (BWD-SIDE-FWD)" << std::endl;
        // bundle->printState(xBundleLastValid->state);
        // bundle->printState(xBackStep->state);
        // bundle->printState(xSideStep->state);
        // bundle->printState(xGoal->state);

        xBundleLastValid = xGoal;
    }

    bundle->freeState(xBundleStartTmp);
    bundle->freeState(xBundleGoalTmp);
    base->freeState(xBase);
    return found;
}

bool PathRestriction::sideStepAlongFiber(Configuration* &xOrigin, base::State *state)
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
bool PathRestriction::tunneling(
    Configuration* &xLastValid,
    const base::State *xBundleGoal)
{
    const ompl::base::StateSamplerPtr bundleSampler = bundleSpaceGraph_->getBundleSamplerPtr();
    ompl::base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();

    std::cout << "Tunneling along interpolated section" << std::endl;

    base::State* xBundleSample = bundle->allocState();
    double originDist = bundle->distance(xLastValid->state, xBundleGoal);

    double tunnelLength = validSegmentLength_;

    bool found = false;

    //assume we start at tunnel beginning --- trying to find tunnel end

    bool hitTunnel = false;
    while (tunnelLength < std::min(originDist, 10*validSegmentLength_) && !found)
    {
        bundle->getStateSpace()->interpolate(xLastValid->state, xBundleGoal, tunnelLength, xBundleSample);
        if(bundle->isValid(xBundleSample) && !hitTunnel)
        {
            found = true;
        }else{
            hitTunnel = true;
            tunnelLength += validSegmentLength_;
        }
    }
    if(!found)
    {
        std::cout << "Could not find tunnel" << std::endl;
        std::cout << "Dist:" << tunnelLength << "/" << originDist << std::endl;
    }else{
        double epsilon = tunnelLength;
        if(found)
        {
            std::cout << "Found valid state after " << tunnelLength << " distance." << std::endl;
            std::cout << "Trying to tunnel through using bisection" << std::endl;

            //try midway heuristic (basically bisect tunnel, try sidesteps around
            //it)
            found = false;
            const base::State* xBundleTunnelEnd = bundle->cloneState(xBundleSample);
            std::cout << "END OF TUNNEL:" << std::endl;
            bundle->printState(xBundleTunnelEnd);

            unsigned int ctr = 0;
            while (ctr++ < magic::PATH_SECTION_MAX_WRIGGLING && !found)
            {
                bundle->getStateSpace()->interpolate(xLastValid->state, 
                    xBundleTunnelEnd, 0.5, xBundleSample);
                bundleSampler->sampleUniformNear(xBundleSample, xBundleSample, epsilon);

                if(bundle->isValid(xBundleSample))
                {
                    if(bundle->checkMotion(xLastValid->state, xBundleSample))
                    {
                        Configuration *xTunnelStep = new Configuration(bundle, xBundleSample);
                        bundleSpaceGraph_->addConfiguration(xTunnelStep);
                        bundleSpaceGraph_->addBundleEdge(xLastValid, xTunnelStep);

                        xLastValid = xTunnelStep;

                        if(bundle->checkMotion(xTunnelStep->state, xBundleTunnelEnd))
                        {
                            Configuration *xTunnelEnd = new Configuration(bundle, xBundleTunnelEnd);
                            bundleSpaceGraph_->addConfiguration(xTunnelEnd);
                            bundleSpaceGraph_->addBundleEdge(xTunnelStep, xTunnelEnd);

                            std::cout << "Successfully tunneled at iter " << ctr << std::endl;
                            bundle->printState(xLastValid->state);
                            bundle->printState(xTunnelEnd->state);
                            found = true;

                            xLastValid = xTunnelEnd;
                        }
                    }
                }
            }

            // bundle->freeState(xBundleTunnelEnd);
        }
    }


    bundle->freeState(xBundleSample);
    return found;
}

bool PathRestriction::wriggleFree(
    Configuration* &xLastValid,
    const base::State *xBundleGoal)
{
    double d;
    return wriggleFree(xLastValid, xBundleGoal, d);
}

bool PathRestriction::wriggleFree(
    Configuration* &xLastValid,
    const base::State *xBundleGoal, 
    double &distProgress)
{

    const ompl::base::StateSamplerPtr bundleSampler = bundleSpaceGraph_->getBundleSamplerPtr();
    ompl::base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();

    // std::cout << "Wriggle from" << std::endl;
    // bundle->printState(xLastValid->state);
    // std::cout << "Wriggle towards" << std::endl;
    // bundle->printState(xBundleGoal);

    unsigned int ctr = 0;

    base::State* xBundleNext = bundle->allocState();
    double originDist = bundle->distance(xLastValid->state, xBundleGoal);
    double bestDist = originDist;

    double epsilon = 0.5;
    while (ctr++ < magic::PATH_SECTION_MAX_WRIGGLING)
    {
        bundleSampler->sampleUniformNear(xBundleNext, xLastValid->state, epsilon);
        double dist = bundle->distance(xBundleNext, xBundleGoal);
        if(dist < bestDist && 
            bundle->isValid(xBundleNext) &&
            bundle->checkMotion(xLastValid->state, xBundleNext))
        {
            //set cur to next. Continue from there
            // bundle->copyState(xBundleBestState, xBundleNext);
            bestDist = dist;
            // std::cout << "Add wriggle state with new dist " << bestDist << std::endl;
            // bundle->printState(xBundleNext);

            Configuration *xWriggleStep = new Configuration(bundle, xBundleNext);
            bundleSpaceGraph_->addConfiguration(xWriggleStep);
            bundleSpaceGraph_->addBundleEdge(xLastValid, xWriggleStep);

            xLastValid = xWriggleStep;
        }
    }

    bundle->freeState(xBundleNext);

    distProgress = originDist - bestDist;

    return (distProgress > 0);
}

bool PathRestriction::hasFeasibleSection(
    Configuration *const xStart, 
    Configuration *const xGoal)
{
    BasePathHeadPtr head = std::make_shared<BasePathHead>(this, xStart, xGoal);

    bool foundFeasibleSection = findSection(head);

    // if(!foundFeasibleSection)
    // {
    //     foundFeasibleSection = findSection(head, false);
    // }

    if(foundFeasibleSection)
    {
        OMPL_DEBUG("Found Feasible Section on level %d", bundleSpaceGraph_->getLevel());
    }else{
        OMPL_DEBUG("No feasible section found over base path");
    }

    return foundFeasibleSection;
}


#define COUT(depth) (std::cout << std::string(4*depth, '>'))

bool PathRestriction::findSection(
    const BasePathHeadPtr head,
    bool interpolateFiberFirst, 
    unsigned int depth)
{
    COUT(depth) << std::string(20, '-') << std::endl;
    COUT(depth) << "Calling findsection at depth " << depth << std::endl;

    base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();
    base::SpaceInformationPtr base = bundleSpaceGraph_->getBase();
    base::SpaceInformationPtr fiber = bundleSpaceGraph_->getFiber();

    COUT(depth) << "Depth " << depth 
      << " xLastValid:";
    bundle->printState(head->getStartConfiguration()->state);
    COUT(depth) << std::endl;

    PathSectionPtr section = std::make_shared<PathSection>(this, head);

    if (interpolateFiberFirst)
    {
        section->interpolateL1FiberFirst();
    }
    else
    {
        section->interpolateL1FiberLast();
    }

    if(section->checkMotion())
    {
        COUT(depth) << "Interpolation Section successful" << std::endl;
        section->sanityCheck();
        return true;
    }

    double locationOnBasePath = section->getLastValidBasePathLocation();

    static_cast<BundleSpaceGraph *>(bundleSpaceGraph_->getParent())
       ->getGraphSampler()
       ->setPathBiasStartSegment(locationOnBasePath);

    if (depth >= magic::PATH_SECTION_MAX_DEPTH)
    {
        COUT(depth) << "Reporting failure at depth " << depth << std::endl;
        return false;
    }

    //############################################################################
    //Get last valid state information
    //############################################################################

    Configuration *xLastValid = section->getLastValidConfiguration();

    int idxBase = section->getLastValidBasePathIndex();

    int idxSection = section->getLastValidSectionPathIndex();


    //############################################################################
    //Try different strategies to locally resolve constraint violation
    //Then call function recursively with clipped base path
    //############################################################################

    COUT(depth) << "Depth " << depth 
      << " xLastValid:";
    bundle->printState(xLastValid->state);
    COUT(depth) << std::endl;

    // if(wriggleFree(xLastValid, section->at(idxSection+1)))
    // {
    //     BasePathHeadPtr newHead = 
    //       std::make_shared<BasePathHead>(this, xLastValid, head->getGoalConfiguration());

    //     newHead->setLastValidBasePathIndex(idxBase);

    //     const base::State* baseStateLastIndex = head->getBaseStateAt(idxBase);

    //     bundleSpaceGraph_->projectBase(xLastValid->state, xBaseTmp_);

    //     double d = base->distance(baseStateLastIndex, xBaseTmp_);
    //     double dLast = section->getLastValidBasePathLocation();

    //     newHead->setLocationOnBasePath(dLast + d);

    //     COUT(depth) << "Depth " << depth 
    //       << " xLastValid (after wriggling):";
    //     bundle->printState(xLastValid->state);
    //     COUT(depth) << std::endl;

    //     bool feasibleSection = findSection(newHead, false, depth + 1);
    //     if(feasibleSection)
    //     {
    //         COUT(depth) << "Success (depth " << depth << ") after wriggle." << std::endl;
    //         return true;
    //     }
    // }

    if(tunneling(xLastValid, section->at(idxSection+1)))
    {
        BasePathHeadPtr newHead = 
          std::make_shared<BasePathHead>(this, xLastValid, head->getGoalConfiguration());
        newHead->setLastValidBasePathIndex(idxBase);

        const base::State* baseStateLastIndex = getBasePath().at(idxBase);

        bundleSpaceGraph_->projectBase(xLastValid->state, xBaseTmp_);

        double d = base->distance(baseStateLastIndex, xBaseTmp_);
        double dLast = section->getLastValidBasePathLocation();

        newHead->setLocationOnBasePath(dLast + d);

        COUT(depth) << "Depth " << depth 
          << " xLastValid (after tunneling):";
        bundle->printState(xLastValid->state);
        COUT(depth) << std::endl;

        bool feasibleSection = findSection(newHead, false, depth + 1);
        if(feasibleSection)
        {
            COUT(depth) << "Success (depth " << depth << ") after tunneling." << std::endl;
            return true;
        }

    }

    //############################################################################
    // Move base state forward by validsegmentlength_ to penetrate slightly
    // the constraints. This will ensure that sampling along fiber space
    // later will not hit the current feasible region (or potential similar infeasible
    // regions which occur trough symmetries of the robots geometry---like
    // rotations of a cylinder in front of a hole)
    //############################################################################
    // bundleSpaceGraph_->projectBase(xLastValid->state, xBaseTmp_);

    // const base::State* baseStateNextIndex = head->getBaseStateAt(idxBase+1);

    // double d = base->distance(xBaseTmp_, baseStateNextIndex);
    // std::cout << "distance " << d << " to " << idxBase+1 << std::endl;
    // double stepsize = validSegmentLength_;
    // if(d >= stepsize)
    // {
    //   base->getStateSpace()->interpolate(
    //       xBaseTmp_, baseStateNextIndex, stepsize/d, xBaseTmp_);
    // }
    // std::cout << "Advanced base state by " << stepsize << std::endl;
    // base->printState(xBaseTmp_);
    //############################################################################

    //search for alternative openings
    unsigned int infeasibleCtr = 0;

    for (unsigned int j = 0; j < magic::PATH_SECTION_MAX_BRANCHING; j++)
    {
        bundleSpaceGraph_->projectBase(xLastValid->state, xBaseTmp_);

        if (!findFeasibleStateOnFiber(xBaseTmp_, xBundleTmp_))
        {
            infeasibleCtr++;
            continue;
        }

        if(sideStepAlongFiber(xLastValid, xBundleTmp_))
        {
            COUT(depth) << "Depth " << depth 
              << " xLastValid (after sidestep):";
            bundle->printState(xLastValid->state);
            COUT(depth) << std::endl;

            BasePathHeadPtr newHead = 
              std::make_shared<BasePathHead>(this, xLastValid, head->getGoalConfiguration());
            newHead->setLastValidBasePathIndex(idxBase);

            const base::State* baseStateLastIndex = getBasePath().at(idxBase);

            bundleSpaceGraph_->projectBase(xLastValid->state, xBaseTmp_);

            double d = base->distance(baseStateLastIndex, xBaseTmp_);
            double dLast = section->getLastValidBasePathLocation();

            newHead->setLocationOnBasePath(dLast + d);

            bool feasibleSection = findSection(newHead, false, depth + 1);
            if(feasibleSection)
            {
                COUT(depth) << "Success (depth " << depth << ") after sidestep." << std::endl;
                return true;
            }

        }else{
            const base::State* baseStateLastIndex = getBasePath().at(idxBase);

            if(tripleStep(xLastValid, xBundleTmp_, xBaseTmp_, baseStateLastIndex))
            {
                COUT(depth) << "Depth " << depth 
                  << " xLastValid (after triplestep):";
                bundle->printState(xLastValid->state);
                COUT(depth) << std::endl;

                BasePathHeadPtr newHead = 
                  std::make_shared<BasePathHead>(this, xLastValid, head->getGoalConfiguration());
                newHead->setLastValidBasePathIndex(idxBase);

                const base::State* baseStateLastIndex = getBasePath().at(idxBase);

                bundleSpaceGraph_->projectBase(xLastValid->state, xBaseTmp_);

                double d = base->distance(baseStateLastIndex, xBaseTmp_);
                double dLast = section->getLastValidBasePathLocation();

                newHead->setLocationOnBasePath(dLast + d);

                bool feasibleSection = findSection(newHead, false, depth + 1);
                if(feasibleSection)
                {
                    COUT(depth) << "Success (depth " << depth << ") after 3step." << std::endl;
                    return true;
                }
            }
        }
    }
    COUT(depth) << "Failed depth " << depth 
      << " after sampling " << infeasibleCtr 
      << " infeasible fiber elements on fiber";
    COUT(depth) << std::endl;
    return false;
}

