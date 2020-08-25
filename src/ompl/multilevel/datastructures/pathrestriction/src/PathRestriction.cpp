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
        static const unsigned int PATH_SECTION_MAX_WRIGGLING = 100;
        static const unsigned int PATH_SECTION_MAX_DEPTH = 2;
        static const unsigned int PATH_SECTION_MAX_BRANCHING = 100;
        static const unsigned int PATH_SECTION_MAX_TUNNELING = 100;
    }
}

void waitForKeypressed()
{
    // do 
    // {
    //     std::cout << '\n' << "Press a key to continue...";
    // } while (std::cin.get() != '\n');
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
        validFiberSpaceSegmentLength_ = fiber->getStateSpace()->getLongestValidSegmentLength();
    }
    if (bundleSpaceGraph_->getBaseDimension() > 0)
    {
        base::SpaceInformationPtr base = bundleSpaceGraph_->getBase();
        xBaseTmp_ = base->allocState();
        validBaseSpaceSegmentLength_ = base->getStateSpace()->getLongestValidSegmentLength();
    }
    base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();
    xBundleTmp_ = bundle->allocState();
    xBundleTemporaries_.resize(magic::PATH_SECTION_MAX_DEPTH * magic::PATH_SECTION_MAX_BRANCHING);
    bundle->allocStates(xBundleTemporaries_);

    validBundleSpaceSegmentLength_ = bundle->getStateSpace()->getLongestValidSegmentLength();

    neighborhoodRadius_.setLambda(neighborhoodRadiusLambda_);
    neighborhoodRadius_.setInitValue(validBaseSpaceSegmentLength_);
    neighborhoodRadius_.setTargetValue(10*validBaseSpaceSegmentLength_);
}

PathRestriction::~PathRestriction()
{
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
    bundle->freeStates(xBundleTemporaries_);
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

void PathRestriction::interpolateBasePath(double t, base::State* &state) const
{
    assert(t>=0);
    assert(t<=lengthBasePath_);

    unsigned int ctr = 0;
    while(t > lengthsCumulativeBasePath_.at(ctr) && ctr < lengthsCumulativeBasePath_.size() - 1)
    {
        ctr++;
    }

    base::State *s1 = basePath_.at(ctr);
    base::State *s2 = basePath_.at(ctr+1);
    double d = lengthsIntermediateBasePath_.at(ctr);

    double dCum = (ctr > 0?lengthsCumulativeBasePath_.at(ctr-1):0.0);
    double step = (t - dCum)/d;
    
    base::SpaceInformationPtr base = bundleSpaceGraph_->getBase();
    base->getStateSpace()->interpolate(s1, s2, step, state);
}

const std::vector<ompl::base::State*>& PathRestriction::getBasePath() const
{
  return basePath_;
}

double PathRestriction::getLengthBasePath() const
{
  return lengthBasePath_;
}

int PathRestriction::size() const
{
  return basePath_.size();
}

//distance between base states k and k+1
double PathRestriction::getLengthIntermediateBasePath(int k)
{
  return lengthsIntermediateBasePath_.at(k);
}

double PathRestriction::getLengthBasePathUntil(int k)
{
  if(k > (int)size())
  {
    std::cout << "Wrong index k=" << k << "/" << size() << std::endl;
    throw Exception("WrongIndex");
  }
  if(k <= 0) return 0;
  else
  {
    return lengthsCumulativeBasePath_.at(k-1);
  }
}

int PathRestriction::getBasePathLastIndexFromLocation(double d)
{
    if(d > lengthBasePath_)
    {
        return size() - 1;

        // std::cout << "Location: " << d << std::endl;
        // std::cout << "Length  : " << lengthBasePath_ << std::endl;
        // OMPL_ERROR("location not on base path");
        // for(uint k = 0; k < lengthsCumulativeBasePath_.size(); k++)
        // {
        //     std::cout << k << ":" << lengthsCumulativeBasePath_.at(k) << " (" <<
        //       lengthsIntermediateBasePath_.at(k) << ")" << std::endl;
        // }
        // throw Exception("InvalidLocation");
    }
    unsigned int ctr = 0;
    while(d >= lengthsCumulativeBasePath_.at(ctr) && ctr < lengthsCumulativeBasePath_.size() - 1)
    {
        ctr++;
    }
    return ctr;
}

bool PathRestriction::findFeasibleStateOnFiber(
    const base::State *xBase, 
    base::State *xBundle)
{
    unsigned int ctr = 0;
    bool found = false;
    base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();
    base::SpaceInformationPtr base = bundleSpaceGraph_->getBundle();
    // const ompl::base::StateSamplerPtr samplerBase = bundleSpaceGraph_->getBaseSamplerPtr();

    while (ctr++ < 10 && !found)
    {
        // sample model fiber
        // samplerBase->sampleUniformNear(xBaseTmp_, xBase, validBaseSpaceSegmentLength_);

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

bool PathRestriction::tripleStep(
    BasePathHeadPtr& head,
    const base::State *sBundleGoal,
    double locationOnBasePathGoal)
{
    base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();
    base::SpaceInformationPtr base = bundleSpaceGraph_->getBase();
    base::SpaceInformationPtr fiber = bundleSpaceGraph_->getFiber();

    base::State* xBundleStartTmp = bundle->allocState();
    base::State* xBundleGoalTmp = bundle->allocState();
    base::State* xBase = base->cloneState(head->getStateBase());
    const base::State *sBundleStart = head->getState();

    // const ompl::base::StateSamplerPtr fiberSampler = bundleSpaceGraph_->getFiberSamplerPtr();
    // const ompl::base::StateSamplerPtr baseSampler = bundleSpaceGraph_->getBaseSamplerPtr();

    bundleSpaceGraph_->projectFiber(sBundleStart, xFiberStart_);
    bundleSpaceGraph_->projectFiber(sBundleGoal, xFiberGoal_);

    bool found = false;

    //mid point heuristic 
    fiber->getStateSpace()->interpolate(xFiberStart_, xFiberGoal_, 0.5, xFiberTmp_);

    double location = head->getLocationOnBasePath() - validBaseSpaceSegmentLength_;

    //Triple step connection attempt
    // xBundleStartTmp <------- xBundleStart
    //     |
    //     |
    //     |
    //     v
    // xBundleGoalTmp -------> xBundleGoal

    while(!found && location >= 0)
    {
        interpolateBasePath(location, xBase);

        bundleSpaceGraph_->liftState(xBase, xFiberTmp_, xBundleStartTmp);

        if (bundle->isValid(xBundleStartTmp))
        {
            bundleSpaceGraph_->liftState(xBase, xFiberStart_, xBundleStartTmp);
            bundleSpaceGraph_->liftState(xBase, xFiberGoal_, xBundleGoalTmp);

            if(bundle->isValid(xBundleStartTmp)
                && bundle->isValid(xBundleGoalTmp))
            {
                if(bundle->checkMotion(xBundleStartTmp, xBundleGoalTmp))
                {
                    
                    bool feasible = true;

                    double fiberDist = fiber->distance(xFiberStart_, xFiberGoal_);
                    double fiberStepSize = validFiberSpaceSegmentLength_;

                    if(!bundle->checkMotion(sBundleStart, xBundleStartTmp))
                    {
                      feasible = false;

                      double fiberLocation = 0.5*fiberDist;
                      do{
                        fiberLocation -= fiberStepSize;
                        
                        fiber->getStateSpace()->interpolate(
                            xFiberStart_, xFiberGoal_, fiberLocation/fiberDist, xFiberTmp_);


                        bundleSpaceGraph_->liftState(xBase, xFiberTmp_, xBundleStartTmp);

                        if(bundle->checkMotion(sBundleStart, xBundleStartTmp)
                            && bundle->checkMotion(xBundleStartTmp, xBundleGoalTmp))
                        {
                          feasible = true;
                          break;
                        }
                      }while(fiberLocation > -0.5*fiberDist);
                      //try to repair
                    }
                    if(feasible && !bundle->checkMotion(xBundleGoalTmp, sBundleGoal))
                    {
                      feasible = false;

                      double fiberLocation = 0.5*fiberDist;
                      do{
                        fiberLocation += fiberStepSize;

                        fiber->getStateSpace()->interpolate(
                            xFiberStart_, xFiberGoal_, fiberLocation/fiberDist, xFiberTmp_);

                        // bundleSpaceGraph_->liftState(xBaseTmp_, xFiberTmp_, xBundleGoalTmp);
                        bundleSpaceGraph_->liftState(xBase, xFiberTmp_, xBundleGoalTmp);

                        if(bundle->checkMotion(xBundleGoalTmp, sBundleGoal)
                            && bundle->checkMotion(xBundleStartTmp, xBundleGoalTmp))
                        {
                          feasible = true;
                          break;
                        }

                      }while(fiberLocation < 1.5*fiberDist);
                    }
                    if(feasible)
                    {
                        found = true;
                    }
                    break;
                }
            }
        }

        location -= validBaseSpaceSegmentLength_;
    }

    if(found)
    {
        Configuration *xBackStep = new Configuration(bundle, xBundleStartTmp);
        bundleSpaceGraph_->addConfiguration(xBackStep);
        bundleSpaceGraph_->addBundleEdge(head->getConfiguration(), xBackStep);


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

        head->setCurrent(xGoal, locationOnBasePathGoal);
    }

    bundle->freeState(xBundleStartTmp);
    bundle->freeState(xBundleGoalTmp);
    base->freeState(xBase);
    return found;
}

bool PathRestriction::tunneling(
    BasePathHeadPtr& head)
{
    const ompl::base::StateSamplerPtr bundleSampler = bundleSpaceGraph_->getBundleSamplerPtr();
    const ompl::base::StateSamplerPtr baseSampler = bundleSpaceGraph_->getBaseSamplerPtr();
    ompl::base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();
    ompl::base::SpaceInformationPtr base = bundleSpaceGraph_->getBase();

    double curLocation = head->getLocationOnBasePath();

    head->print();


    bool found = false;

    bool hitTunnel = false;

    const ompl::base::StateSamplerPtr fiberSampler = bundleSpaceGraph_->getFiberSamplerPtr();

    while (curLocation < lengthBasePath_ && !found)
    {
        curLocation += validBaseSpaceSegmentLength_;

        interpolateBasePath(curLocation, xBaseTmp_);

        for(uint k = 0; k < 5; k++)
        {
            fiberSampler->sampleUniformNear(xFiberTmp_, 
                head->getStateFiber(), validFiberSpaceSegmentLength_);

            bundleSpaceGraph_->liftState(xBaseTmp_, xFiberTmp_, xBundleTmp_);

            if(bundle->isValid(xBundleTmp_))
            {
                if(hitTunnel)
                {
                    found = true;
                    break;
                }
            }else{
                hitTunnel = true;
            }
        }
    }
    if(!found)
    {
    }else{

        //length of tunnel


        if(found)
        {
            found = false;


            const double locationEndTunnel = curLocation;
            const double lengthTunnel = curLocation - head->getLocationOnBasePath();
            const base::State* xBundleTunnelEnd = bundle->cloneState(xBundleTmp_);

            std::cout << "Found " << lengthTunnel << " dist tunnel" << std::endl;

            Configuration *last = head->getConfiguration();

            curLocation = head->getLocationOnBasePath();

            double bestDistance = bundle->distance(last->state, xBundleTunnelEnd);

            bool makingProgress = true;

            base::State* xBase = base->allocState();

            while (curLocation < locationEndTunnel && makingProgress)
            {
                if(bundle->checkMotion(last->state, xBundleTunnelEnd))
                {
                    Configuration *xTunnel = new Configuration(bundle, xBundleTunnelEnd);
                    bundleSpaceGraph_->addConfiguration(xTunnel);
                    bundleSpaceGraph_->addBundleEdge(last, xTunnel);

                    head->setCurrent(xTunnel, locationEndTunnel);

                    base->freeState(xBase);

                    std::cout << "Tunnel successful" << std::endl;
                    return true;
                }

                curLocation += validBaseSpaceSegmentLength_;

                interpolateBasePath(curLocation, xBaseTmp_);

                makingProgress = false;

                unsigned int ctr = 0;

                while(ctr++ < magic::PATH_SECTION_MAX_TUNNELING)
                {
                    double d = neighborhoodRadius_();
                    std::cout << "nbh radius: " << d << " (valid: "
                      << validBaseSpaceSegmentLength_ << ")" << std::endl;
                    std::cout << neighborhoodRadius_.getCounter() << std::endl;

                    baseSampler->sampleUniformNear(xBase, xBaseTmp_, d);

                    fiberSampler->sampleUniformNear(xFiberTmp_, head->getStateFiber(), 
                        4*validFiberSpaceSegmentLength_);

                    bundleSpaceGraph_->liftState(xBase, xFiberTmp_, xBundleTmp_);

                    if(bundle->isValid(xBundleTmp_))
                    {
                        double d = bundle->distance(xBundleTmp_, xBundleTunnelEnd);
                        if( d < bestDistance)
                        {
                            if(bundle->checkMotion(last->state, xBundleTmp_))
                            {
                                Configuration *xTunnelStep = new Configuration(bundle, xBundleTmp_);
                                bundleSpaceGraph_->addConfiguration(xTunnelStep);
                                bundleSpaceGraph_->addBundleEdge(last, xTunnelStep);

                                last = xTunnelStep;

                                makingProgress = true;
                                bestDistance = d;
                                //add new configurations, but do not change head
                                //yet
                            }
                        }
                    }
                }
            }
            if(!makingProgress)
            {
              std::cout << "Stopped tunnel at " << curLocation << " of tunnel distance "
               << head->getLocationOnBasePath() << " to " << locationEndTunnel << std::endl;
            }
            base->freeState(xBase);
        }
    }

    return false;
}

bool PathRestriction::wriggleFree(BasePathHeadPtr& head)
{
    ompl::base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();

    double curLocation = head->getLocationOnBasePath() + validBaseSpaceSegmentLength_;

    double epsilon = 4*validFiberSpaceSegmentLength_;

    const ompl::base::StateSamplerPtr fiberSampler = bundleSpaceGraph_->getFiberSamplerPtr();
    const ompl::base::StateSamplerPtr baseSampler = bundleSpaceGraph_->getBaseSamplerPtr();

    int steps = 0;
    while (curLocation < lengthBasePath_)
    {
        unsigned int ctr = 0;
        bool madeProgress = false;

        interpolateBasePath(curLocation, xBaseTmp_);

        while(ctr++ < magic::PATH_SECTION_MAX_WRIGGLING)
        {
            baseSampler->sampleUniformNear(xBaseTmp_, xBaseTmp_, neighborhoodRadius_());

            fiberSampler->sampleUniformNear(xFiberTmp_, head->getStateFiber(), epsilon);

            bundleSpaceGraph_->liftState(xBaseTmp_, xFiberTmp_, xBundleTmp_);

            if(bundle->isValid(xBundleTmp_) &&
                bundle->checkMotion(head->getState(), xBundleTmp_))
            {
                Configuration *xWriggleStep = new Configuration(bundle, xBundleTmp_);
                bundleSpaceGraph_->addConfiguration(xWriggleStep);
                bundleSpaceGraph_->addBundleEdge(head->getConfiguration(), xWriggleStep);

                head->setCurrent(xWriggleStep, curLocation);

                madeProgress = true;
                steps++;
                break;
            }
        }
        if(!madeProgress)
        {
            break;
        }
        curLocation += validBaseSpaceSegmentLength_;
    }
    std::cout << "Made " << steps << " wriggle steps." << std::endl;
    head->print();

    return (steps > 0);
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
        OMPL_ERROR("Found Feasible Section on level %d", bundleSpaceGraph_->getLevel());
    }else{
        OMPL_DEBUG("No feasible section found over base path");
    }

    return foundFeasibleSection;
}


#define COUT(depth) (std::cout << std::string(4*depth, '>'))

bool PathRestriction::findSection(
    BasePathHeadPtr& head,
    bool interpolateFiberFirst, 
    unsigned int depth)
{
    COUT(depth) << std::string(20, '-') << std::endl;
    COUT(depth) << "Calling findsection at depth " << depth << std::endl;

    base::SpaceInformationPtr bundle = bundleSpaceGraph_->getBundle();
    base::SpaceInformationPtr base = bundleSpaceGraph_->getBase();
    base::SpaceInformationPtr fiber = bundleSpaceGraph_->getFiber();

    PathSectionPtr section = std::make_shared<PathSection>(this);

    double prevLocation = head->getLocationOnBasePath();

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
        COUT(depth) << "Interpolation Section successful" << std::endl;
        section->sanityCheck();
        return true;
    }

    static_cast<BundleSpaceGraph *>(bundleSpaceGraph_->getParent())
       ->getGraphSampler()
       ->setPathBiasStartSegment(head->getLocationOnBasePath());

    //############################################################################
    //Get last valid state information
    //############################################################################

    COUT(depth) << "Depth " << depth 
      << " head (after checkMotion):";
    head->print();

    waitForKeypressed();

    if (depth >= magic::PATH_SECTION_MAX_DEPTH)
    {
        std::cout << std::string(80, '#') << std::endl;
        COUT(depth) << "Reporting failure at depth " << depth << std::endl;
        std::cout << std::string(80, '#') << std::endl;
        return false;
    }

    //############################################################################
    //Try different strategies to locally resolve constraint violation
    //Then call function recursively with clipped base path
    //############################################################################

    std::cout << "Trying wriggle..." << std::endl;

    if(wriggleFree(head))
    {
        COUT(depth) << "Depth " << depth 
          << " xLastValid (after wriggling):";
        head->print();
        COUT(depth) << std::endl;

        BasePathHeadPtr newHead(head);

        bool feasibleSection = findSection(newHead, false, depth + 1);
        if(feasibleSection)
        {
            COUT(depth) << "Success (depth " << depth << ") after wriggle." << std::endl;
            return true;
        }
    }

    std::cout << "Trying tunnel..." << std::endl;

    if(tunneling(head))
    {
        COUT(depth) << "Depth " << depth 
          << " xLastValid (after tunnel):";
        head->print();
        COUT(depth) << std::endl;

        BasePathHeadPtr newHead(head);
        bool feasibleSection = findSection(newHead, false, depth + 1);
        if(feasibleSection)
        {
            COUT(depth) << "Success (depth " << depth << ") after tunnel." << std::endl;
            return true;
        }
    }

    double curLocation = head->getLocationOnBasePath();

    if(curLocation <= prevLocation)
    {
        //stuck in crevice
        return false;
    }

    ////search for alternative openings
    unsigned int infeasibleCtr = 0;

    // unsigned int size = 0;

    // int offset = depth * magic::PATH_SECTION_MAX_DEPTH;

    double location = head->getLocationOnBasePath() + validBaseSpaceSegmentLength_;

    std::cout << "Location to do fiber test: " << location << std::endl;
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

        interpolateBasePath(location, xBaseTmp_);

        if (!findFeasibleStateOnFiber(xBaseTmp_, xBundleTmp_))
        {
            // std::cout << "Dismissed: infeasible fiber element" << std::endl;
            infeasibleCtr++;
            continue;
        }

        //check that we did not previously visited this area

        // bool found = true;
        // for(uint k = 0; k < size; k++)
        // {
        //     if(bundle->checkMotion(xBundleTemporaries_.at(offset + k), xBundleTmp_))
        //     {
        //         infeasibleCtr++;
        //         // std::cout << "Dismissed: already sampled area before" << std::endl;
        //         found = false;
        //         break;
        //     }
        // }
        // if(!found)
        // {
        //   continue;
        // }

        // bundle->copyState(xBundleTemporaries_.at(offset + size), xBundleTmp_);
        // size++;

        if(bundle->checkMotion(head->getState(), xBundleTmp_))
        {
            ////side step along fiber
            //Configuration *xSideStep = new Configuration(bundle, xBundleTmp_);
            //bundleSpaceGraph_->addConfiguration(xSideStep);
            //bundleSpaceGraph_->addBundleEdge(head->getConfiguration(), xSideStep);

            //head->setCurrent(xSideStep, curLocation);
        }else{
            if(tripleStep(head, xBundleTmp_, location))
            {
                BasePathHeadPtr newHead(head);

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

