#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathSection.h>
#include <ompl/multilevel/datastructures/pathrestriction/BasePathHead.h>
#include <ompl/multilevel/datastructures/pathrestriction/FindSectionPatternDance.h>
#include <ompl/multilevel/datastructures/pathrestriction/FindSectionAnalyzer.h>
#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>

namespace ompl
{
    namespace magic
    {
        static const unsigned int PATH_SECTION_MAX_WRIGGLING = 100;
        static const unsigned int PATH_SECTION_MAX_DEPTH = 2; //3 seems max
        static const unsigned int PATH_SECTION_MAX_BRANCHING = 500;
        static const unsigned int PATH_SECTION_MAX_TUNNELING = 100;
    }
}

using namespace ompl::multilevel;

FindSectionPatternDance::FindSectionPatternDance(PathRestriction* restriction):
  BaseT(restriction)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();

    neighborhoodBaseSpace_.setValueInit(0);
    neighborhoodBaseSpace_.setValueTarget(validBaseSpaceSegmentLength_);
    neighborhoodBaseSpace_.setCounterInit(0);
    neighborhoodBaseSpace_.setCounterTarget(magic::PATH_SECTION_MAX_BRANCHING);
    for(uint k = 0; k < magic::PATH_SECTION_MAX_DEPTH; k++)
    {
        ParameterSmoothStep param;
        param.setValueInit(0.01*validBaseSpaceSegmentLength_);
        param.setValueTarget(2*validBaseSpaceSegmentLength_);
        param.setCounterInit(0);
        param.setCounterTarget(magic::PATH_SECTION_MAX_BRANCHING);
        neighborhoodBaseSpacePerDepth_.push_back(param);
    }

    if(graph->hasBaseSpace())
    {
        base::SpaceInformationPtr base = graph->getBase();
        xBaseFixed_ = base->allocState();
    }
}

FindSectionPatternDance::~FindSectionPatternDance()
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    base::SpaceInformationPtr bundle = graph->getBundle();
    if(graph->hasBaseSpace())
    {
        base::SpaceInformationPtr base = graph->getBase();
        base->freeState(xBaseFixed_);
    }
}

bool FindSectionPatternDance::solve(BasePathHeadPtr& head)
{
    BasePathHeadPtr head2(head);

    ompl::time::point tStart = ompl::time::now();
    bool foundFeasibleSection = recursivePatternSearch(head);
    ompl::time::point t1 = ompl::time::now();

    OMPL_DEBUG("FindSectionPatternDance required %.2fs.", ompl::time::seconds(t1 - tStart));

    if(!foundFeasibleSection)
    {
        foundFeasibleSection = recursivePatternSearch(head2, false);
        ompl::time::point t2 = ompl::time::now();
        OMPL_DEBUG("FindSectionPatternDance2 required %.2fs.", ompl::time::seconds(t2 - t1));
    }

    return foundFeasibleSection;
}

bool FindSectionPatternDance::sideStepAlongFiber(Configuration* &xOrigin, base::State *state)
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

bool FindSectionPatternDance::tunneling(
    BasePathHeadPtr& head)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    const ompl::base::StateSamplerPtr bundleSampler = graph->getBundleSamplerPtr();
    const ompl::base::StateSamplerPtr baseSampler = graph->getBaseSamplerPtr();
    ompl::base::SpaceInformationPtr bundle = graph->getBundle();
    ompl::base::SpaceInformationPtr base = graph->getBase();

    double curLocation = head->getLocationOnBasePath();

    bool found = false;

    bool hitTunnel = false;

    const ompl::base::StateSamplerPtr fiberSampler = graph->getFiberSamplerPtr();

    while (curLocation < restriction_->getLengthBasePath() && !found)
    {
        curLocation += validBaseSpaceSegmentLength_;

        restriction_->interpolateBasePath(curLocation, xBaseTmp_);

        for(uint k = 0; k < 5; k++)
        {
            fiberSampler->sampleUniformNear(xFiberTmp_, 
                head->getStateFiber(), validFiberSpaceSegmentLength_);

            graph->liftState(xBaseTmp_, xFiberTmp_, xBundleTmp_);

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
            // const double lengthTunnel = curLocation - head->getLocationOnBasePath();
            const base::State* xBundleTunnelEnd = bundle->cloneState(xBundleTmp_);

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

                while(ctr++ < magic::PATH_SECTION_MAX_TUNNELING)
                {
                    double d = neighborhoodRadiusBaseSpace_();

                    baseSampler->sampleUniformNear(xBase, xBaseTmp_, d);

                    fiberSampler->sampleUniformNear(xFiberTmp_, head->getStateFiber(), 
                        4*validFiberSpaceSegmentLength_);

                    graph->liftState(xBase, xFiberTmp_, xBundleTmp_);

                    if(bundle->isValid(xBundleTmp_))
                    {
                        double d = bundle->distance(xBundleTmp_, xBundleTunnelEnd);
                        if( d < bestDistance)
                        {
                            if(bundle->checkMotion(last->state, xBundleTmp_))
                            {
                                Configuration *xTunnelStep = new Configuration(bundle, xBundleTmp_);
                                graph->addConfiguration(xTunnelStep);
                                graph->addBundleEdge(last, xTunnelStep);

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
            // if(!makingProgress)
            // {
            //   std::cout << "Stopped tunnel at " << curLocation << " of tunnel distance "
            //    << head->getLocationOnBasePath() << " to " << locationEndTunnel << std::endl;
            // }
            base->freeState(xBase);
        }
    }

    return false;
}

bool FindSectionPatternDance::wriggleFree(BasePathHeadPtr& head)
{
    BundleSpaceGraph *graph = restriction_->getBundleSpaceGraph();
    ompl::base::SpaceInformationPtr bundle = graph->getBundle();
    ompl::base::SpaceInformationPtr fiber = graph->getFiber();

    double curLocation = head->getLocationOnBasePath() + validBaseSpaceSegmentLength_;

    double epsilon = 4*validFiberSpaceSegmentLength_;

    const ompl::base::StateSamplerPtr fiberSampler = graph->getFiberSamplerPtr();
    const ompl::base::StateSamplerPtr baseSampler = graph->getBaseSamplerPtr();


    base::State* xBundleMidPoint = bundle->allocState();
    int steps = 0;

    while (curLocation < restriction_->getLengthBasePath())
    {
        unsigned int ctr = 0;
        bool madeProgress = false;

        restriction_->interpolateBasePath(curLocation, xBaseTmp_);

        neighborhoodRadiusBaseSpace_.reset();

        while(ctr++ < magic::PATH_SECTION_MAX_WRIGGLING)
        {
            double d = neighborhoodRadiusBaseSpace_();

            baseSampler->sampleUniformNear(xBaseTmp_, xBaseTmp_, d);

            fiberSampler->sampleUniformNear(xFiberTmp_, head->getStateFiber(), epsilon);

            graph->liftState(xBaseTmp_, xFiberTmp_, xBundleTmp_);


            //#########################################################
            //L2 step
            //#########################################################
            //
            if(bundle->isValid(xBundleTmp_))
            {
                if(bundle->checkMotion(head->getState(), xBundleTmp_))
                {
                    Configuration *xWriggleStep = new Configuration(bundle, xBundleTmp_);
                    graph->addConfiguration(xWriggleStep);
                    graph->addBundleEdge(head->getConfiguration(), xWriggleStep);

                    head->setCurrent(xWriggleStep, curLocation);

                    madeProgress = true;
                    steps++;
                    break;
                }else{
                    //#########################################################
                    //try corner step
                    //#########################################################
                    const base::State* xBaseHead = head->getStateBase();
                    graph->liftState(xBaseHead, xFiberTmp_, xBundleMidPoint);
                    if(bundle->checkMotion(head->getState(), xBundleMidPoint)
                        && bundle->checkMotion(xBundleMidPoint, xBundleTmp_))
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
        if(!madeProgress)
        {
            break;
        }
        curLocation += validBaseSpaceSegmentLength_;
    }

    bundle->freeState(xBundleMidPoint);

    if(steps > 0)
    {
        //try moving back to restriction
        fiber->copyState(xFiberTmp_, head->getStateFiber());

        double location = head->getLocationOnBasePath() + validBaseSpaceSegmentLength_;
        restriction_->interpolateBasePath(location, xBaseTmp_);

        unsigned int ctr = 0;

        while(ctr++ < magic::PATH_SECTION_MAX_WRIGGLING)
        {
            graph->liftState(xBaseTmp_, xFiberTmp_, xBundleTmp_);

            if(bundle->isValid(xBundleTmp_))
            {
                if(bundle->checkMotion(head->getState(), xBundleTmp_))
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


#define COUT(depth) (std::cout << std::string(4*depth, '>'))

bool FindSectionPatternDance::recursivePatternSearch(
    BasePathHeadPtr& head,
    bool interpolateFiberFirst, 
    unsigned int depth)
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

    //############################################################################
    //Get last valid state information
    //############################################################################

    if (depth >= magic::PATH_SECTION_MAX_DEPTH)
    {
        return false;
    }

    //############################################################################
    //Try different strategies to locally resolve constraint violation
    //Then call function recursively with clipped base path
    //############################################################################

    if(wriggleFree(head))// || tunneling(head))
    {
        BasePathHeadPtr newHead(head);

        bool feasibleSection = recursivePatternSearch(newHead, false, depth + 1);
        if(feasibleSection)
        {
            return true;
        }
    }

    double curLocation = head->getLocationOnBasePath();

    if(curLocation <= prevLocation)
    {
        return false;
    }

    //############################################################################
    ////search for alternative openings
    //############################################################################

    double location = head->getLocationOnBasePath() + validBaseSpaceSegmentLength_;

    const ompl::base::StateSamplerPtr baseSampler = graph->getBaseSamplerPtr();

    neighborhoodBaseSpace_.reset();
    neighborhoodBaseSpacePerDepth_.at(depth).reset();

    FindSectionAnalyzer analyzer(head);

    analyzer.disable();

    bool found  = false;
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

        double epsNBH = neighborhoodBaseSpacePerDepth_.at(depth)();

        double offset = std::max(validBaseSpaceSegmentLength_, epsNBH);

        location = head->getLocationOnBasePath() + offset;

        restriction_->interpolateBasePath(location, xBaseTmp_);

        // double epsNBH = neighborhoodBaseSpace_();

        baseSampler->sampleUniformNear(xBaseTmp_, xBaseTmp_, epsNBH);

        if (!findFeasibleStateOnFiber(xBaseTmp_, xBundleTmp_))
        {
            analyzer("infeasible");
            continue;
        }

        if(bundle->checkMotion(head->getState(), xBundleTmp_))
        {
            analyzer("locally reachable (ignored)");
            continue;
        }

        if(cornerStep(head, xBundleTmp_, location) || 
            tripleStep(head, xBundleTmp_, location))
        {
            BasePathHeadPtr newHead(head);

            bool feasibleSection = recursivePatternSearch(newHead, false, depth + 1);
            if(feasibleSection)
            {
                found = true;
                break;
            }else{
                analyzer("no section");
                continue;
            }
        }else{
            analyzer("not reachable (triple step)");
            continue;
        }
    }
    analyzer.print();
    return found;
}


