#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathSection.h>
#include <ompl/multilevel/datastructures/pathrestriction/BasePathHead.h>
#include <ompl/multilevel/datastructures/pathrestriction/FindSectionVariableNeighborhood.h>
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

FindSectionVariableNeighborhood::FindSectionVariableNeighborhood(PathRestriction* restriction):
  BaseT(restriction)
{
    std::cout << "FINDSECTION variableNeighborhoodPatternSearch" << std::endl;
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();

    neighborhoodBaseSpace_.setValueInit(0);
    neighborhoodBaseSpace_.setValueTarget(2*validBaseSpaceSegmentLength_);
    neighborhoodBaseSpace_.setCounterInit(0);
    neighborhoodBaseSpace_.setCounterTarget(magic::PATH_SECTION_MAX_SAMPLING);

    neighborhoodFiberSpace_.setValueInit(0);
    neighborhoodFiberSpace_.setValueTarget(4*validFiberSpaceSegmentLength_);
    neighborhoodFiberSpace_.setCounterInit(0);
    neighborhoodFiberSpace_.setCounterTarget(magic::PATH_SECTION_MAX_BRANCHING);
}

FindSectionVariableNeighborhood::~FindSectionVariableNeighborhood()
{
    // BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    // base::SpaceInformationPtr bundle = graph->getBundle();
}

bool FindSectionVariableNeighborhood::solve(BasePathHeadPtr& head)
{
    BasePathHeadPtr head2(head);

    ompl::time::point tStart = ompl::time::now();
    bool foundFeasibleSection = variableNeighborhoodPatternSearch(head);
    ompl::time::point t1 = ompl::time::now();

    OMPL_WARN("FindSectionVariableNeighborhood required %.2fs.", ompl::time::seconds(t1 - tStart));

    if(!foundFeasibleSection)
    {
        foundFeasibleSection = variableNeighborhoodPatternSearch(head2, false);
        ompl::time::point t2 = ompl::time::now();
        OMPL_WARN("FindSectionVariableNeighborhood2 required %.2fs.", ompl::time::seconds(t2 - t1));
    }

    return foundFeasibleSection;
}

bool FindSectionVariableNeighborhood::sideStepAlongFiber(Configuration* &xOrigin, base::State *state)
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

bool FindSectionVariableNeighborhood::cornerStep(
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

    //Corner step connection attempt
    // xBundleHead
    //     |
    //     |
    //     |
    //     v
    // xBundleMidPoint -------> xBundleTarget

    bool found = false;

    unsigned int ctr = 0;

    neighborhoodBaseSpace_.reset();
    while(ctr++ < magic::PATH_SECTION_MAX_SAMPLING)
    {
        samplerBase->sampleUniformNear(xBaseTmp_, xBaseHead, neighborhoodBaseSpace_());

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
    }

    bundle->freeState(xBundleMidPoint);
    base->freeState(xBaseHead);
    return found;
}

bool FindSectionVariableNeighborhood::tripleStep(
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

                        // graph->liftState(xBaseTmp_, xFiberTmp_, xBundleGoalTmp);
                        graph->liftState(xBase, xFiberTmp_, xBundleGoalTmp);

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

bool FindSectionVariableNeighborhood::variableNeighborhoodPatternSearch(
    BasePathHeadPtr& head,
    bool interpolateFiberFirst,
    int depth)
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


    if(section->checkMotion(head))
    {
        section->sanityCheck();
        return true;
    }

    static_cast<BundleSpaceGraph *>(graph->getBaseBundleSpace())
       ->getGraphSampler()
       ->setPathBiasStartSegment(head->getLocationOnBasePath());

    if(depth >= magic::PATH_SECTION_MAX_DEPTH)
    {
        return false;
    }
    //############################################################################
    //Try different strategies to locally resolve constraint violation
    //Then call function recursively with clipped base path
    //############################################################################

    double curLocation = head->getLocationOnBasePath();

    if(curLocation <= prevLocation)
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

    double bestDistance = bundle->distance(head->getState(), xBaseTarget);

    std::cout << "best dist:" << bestDistance << std::endl;

    ompl::RNG rng;

    for (unsigned int j = 0; j < magic::PATH_SECTION_MAX_BRANCHING; j++)
    {
        double location = rng.uniformReal(headLocation, 
            headLocation + validBaseSpaceSegmentLength_);

        restriction_->interpolateBasePath(location, xBaseTmp_);

        // samplerBase->sampleUniformNear(xBaseTmp_, xBaseTmp_, neighborhoodBaseSpace_());

        if(j < 0.5*magic::PATH_SECTION_MAX_BRANCHING)
        {
            samplerFiber->sampleUniformNear(xFiberTmp_, xFiberTmp_, neighborhoodFiberSpace_());
        }else{
            graph->sampleFiber(xFiberTmp_);
        }

        graph->liftState(xBaseTmp_, xFiberTmp_, xBundleTmp_);

        if (!bundle->isValid(xBundleTmp_))
        {
            infeasibleCtr++;
            continue;
        }
        double curDistance = bundle->distance(xBundleTmp_, xBaseTarget);

        if(curDistance < bestDistance)
        {
            bestDistance = curDistance;
        }else
        {
            infeasibleCtr++;
            continue;
        }

        if( cornerStep(head, xBundleTmp_, location)
         || tripleStep(head, xBundleTmp_, location))
        {
            BasePathHeadPtr newHead(head);

            bool feasibleSection = variableNeighborhoodPatternSearch(newHead, 
                !interpolateFiberFirst, depth + 1);
            if(feasibleSection)
            {
                return true;
            }
            break;
        }else{
          std::cout << "Failed sidestep" << std::endl;
        }

    }
    std::cout << "Failed depth " << depth 
      << " after sampling " << infeasibleCtr 
      << " infeasible fiber elements on fiber" << std::endl;
    return false;
}


