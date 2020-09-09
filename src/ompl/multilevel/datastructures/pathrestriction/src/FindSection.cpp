#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathSection.h>
#include <ompl/multilevel/datastructures/pathrestriction/BasePathHead.h>
#include <ompl/multilevel/datastructures/pathrestriction/FindSection.h>
#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>

namespace ompl
{
    namespace magic
    {
        static const unsigned int PATH_SECTION_MAX_FIBER_SAMPLING = 10;
    }
}

using namespace ompl::multilevel;

FindSection::FindSection(PathRestriction* restriction):
  restriction_(restriction)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    if (graph->getFiberDimension() > 0)
    {
        base::SpaceInformationPtr fiber = graph->getFiber();
        xFiberStart_ = fiber->allocState();
        xFiberGoal_ = fiber->allocState();
        xFiberTmp_ =  fiber->allocState();
        validFiberSpaceSegmentLength_ = fiber->getStateSpace()->getLongestValidSegmentLength();
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
    neighborhoodRadiusBaseSpace_.setValueTarget(10*validBaseSpaceSegmentLength_);

    neighborhoodCornerStep_.setValueInit(0);
    neighborhoodCornerStep_.setValueTarget(validBaseSpaceSegmentLength_);
    neighborhoodCornerStep_.setCounterInit(0);
    neighborhoodCornerStep_.setCounterTarget(magic::PATH_SECTION_MAX_FIBER_SAMPLING);
}

FindSection::~FindSection()
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
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
    base::SpaceInformationPtr bundle = graph->getBundle();
    bundle->freeState(xBundleTmp_);
}

bool FindSection::findFeasibleStateOnFiber(
    const base::State *xBase, 
    base::State *xBundle)
{
    unsigned int ctr = 0;
    bool found = false;

    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();
    base::SpaceInformationPtr base = graph->getBundle();
    // const ompl::base::StateSamplerPtr samplerBase = graph->getBaseSamplerPtr();

    while (ctr++ < magic::PATH_SECTION_MAX_FIBER_SAMPLING && !found)
    {
        // sample model fiber
        // samplerBase->sampleUniformNear(xBaseTmp_, xBase, validBaseSpaceSegmentLength_);

        graph->sampleFiber(xFiberTmp_);

        graph->liftState(xBase, xFiberTmp_, xBundle);

        //New sample must be valid AND not reachable from last valid
        if (bundle->isValid(xBundle))
        {
            found = true;
        }
    }
    return found;
}

bool FindSection::cornerStep(
    BasePathHeadPtr& head,
    const base::State *xBundleTarget,
    double locationOnBasePathTarget)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();
    base::SpaceInformationPtr base = graph->getBase();
    base::SpaceInformationPtr fiber = graph->getFiber();
    const ompl::base::StateSamplerPtr samplerBase = graph->getBaseSamplerPtr();


    const base::State *xBundleHead = head->getState();

    base::State *xBaseHead = base->cloneState(head->getStateBase());

    base::State* xBundleMidPoint = bundle->allocState();

    graph->projectFiber(xBundleTarget, xFiberGoal_);
    graph->projectFiber(xBundleHead, xFiberStart_);

    //Corner step connection attempt
    // xBundleHead
    //     |
    //     |
    //     |
    //     v
    // xBundleMidPoint -------> xBundleTarget

    // xBundleHead -----------> xBundleMidpoint
    //                               |
    //                               |
    //                               |
    //                               v
    //                          xBundleTarget

    bool found = false;

    unsigned int ctr = 0;

    neighborhoodCornerStep_.reset();
    while(ctr++ < magic::PATH_SECTION_MAX_FIBER_SAMPLING)
    {
        samplerBase->sampleUniformNear(xBaseTmp_, xBaseHead, neighborhoodCornerStep_());

        //############################################################################
        //try fiber first
        graph->liftState(xBaseTmp_, xFiberGoal_, xBundleMidPoint);

        if (bundle->isValid(xBundleMidPoint))
        {
            if(bundle->checkMotion(xBundleHead, xBundleMidPoint)
                && bundle->checkMotion(xBundleMidPoint, xBundleTarget))
            {
                Configuration *xMidPointStep = new Configuration(bundle, xBundleMidPoint);
                graph->addConfiguration(xMidPointStep);
                graph->addBundleEdge(head->getConfiguration(), xMidPointStep);

                Configuration *xTarget = new Configuration(bundle, xBundleTarget);
                graph->addConfiguration(xTarget);
                graph->addBundleEdge(xMidPointStep, xTarget);

                head->setCurrent(xTarget, locationOnBasePathTarget);
                found = true;
                break;
            }
        }
        //############################################################################
        //try fiber last
        graph->liftState(xBaseTmp_, xFiberStart_, xBundleMidPoint);

        if (bundle->isValid(xBundleMidPoint))
        {
            if(bundle->checkMotion(xBundleHead, xBundleMidPoint)
                && bundle->checkMotion(xBundleMidPoint, xBundleTarget))
            {
                Configuration *xMidPointStep = new Configuration(bundle, xBundleMidPoint);
                graph->addConfiguration(xMidPointStep);
                graph->addBundleEdge(head->getConfiguration(), xMidPointStep);

                Configuration *xTarget = new Configuration(bundle, xBundleTarget);
                graph->addConfiguration(xTarget);
                graph->addBundleEdge(xMidPointStep, xTarget);

                head->setCurrent(xTarget, locationOnBasePathTarget);
                found = true;
                break;
            }
        }
    }

    bundle->freeState(xBundleMidPoint);
    base->freeState(xBaseHead);
    return found;
}

bool FindSection::tripleStep(
    BasePathHeadPtr& head,
    const base::State *sBundleGoal,
    double locationOnBasePathGoal)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();
    base::SpaceInformationPtr base = graph->getBase();
    base::SpaceInformationPtr fiber = graph->getFiber();

    base::State* xBundleStartTmp = bundle->allocState();
    base::State* xBundleGoalTmp = bundle->allocState();
    base::State* xBase = base->cloneState(head->getStateBase());
    const base::State *sBundleStart = head->getState();

    graph->projectFiber(sBundleStart, xFiberStart_);
    graph->projectFiber(sBundleGoal, xFiberGoal_);

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
        restriction_->interpolateBasePath(location, xBase);

        graph->liftState(xBase, xFiberTmp_, xBundleStartTmp);

        if (bundle->isValid(xBundleStartTmp))
        {
            graph->liftState(xBase, xFiberStart_, xBundleStartTmp);
            graph->liftState(xBase, xFiberGoal_, xBundleGoalTmp);

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

                        graph->liftState(xBase, xFiberTmp_, xBundleStartTmp);

                        if(bundle->checkMotion(sBundleStart, xBundleStartTmp)
                            && bundle->checkMotion(xBundleStartTmp, xBundleGoalTmp))
                        {
                          feasible = true;
                          break;
                        }
                      }while(fiberLocation > -0.2*fiberDist);
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

                        // graph->liftState(xBaseTmp_, xFiberTmp_, xBundleGoalTmp);
                        graph->liftState(xBase, xFiberTmp_, xBundleGoalTmp);

                        if(bundle->checkMotion(xBundleGoalTmp, sBundleGoal)
                            && bundle->checkMotion(xBundleStartTmp, xBundleGoalTmp))
                        {
                          feasible = true;
                          break;
                        }

                      }while(fiberLocation < 1.2*fiberDist);
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
        graph->addConfiguration(xBackStep);
        graph->addBundleEdge(head->getConfiguration(), xBackStep);


        Configuration *xSideStep = new Configuration(bundle, xBundleGoalTmp);
        graph->addConfiguration(xSideStep);
        graph->addBundleEdge(xBackStep, xSideStep);

        //xBaseTmp_ is on last valid fiber. 
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

