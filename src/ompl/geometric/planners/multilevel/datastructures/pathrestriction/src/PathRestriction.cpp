#include <ompl/geometric/planners/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/geometric/planners/multilevel/datastructures/graphsampler/GraphSampler.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <ompl/base/Path.h>
#include <ompl/geometric/PathGeometric.h>

#define SANITY_CHECK_PATH_RESTRICTION
#undef SANITY_CHECK_PATH_RESTRICTION

ompl::geometric::BundleSpacePathRestriction::BundleSpacePathRestriction(BundleSpaceGraph *bundleSpaceGraph)
  : bundleSpaceGraph_(bundleSpaceGraph)
{
    if (bundleSpaceGraph_->getFiberDimension() > 0)
    {
        xFiberStart_ = bundleSpaceGraph_->getFiber()->allocState();
        xFiberGoal_ = bundleSpaceGraph_->getFiber()->allocState();
        xFiberTmp_ = bundleSpaceGraph_->getFiber()->allocState();
    }
    if (bundleSpaceGraph_->getBaseDimension() > 0)
    {
        xBaseTmp_ = bundleSpaceGraph_->getBase()->allocState();
    }
    xBundleTmp_ = bundleSpaceGraph_->getBundle()->allocState();

    lastValid_.first = bundleSpaceGraph_->getBundle()->allocState();
}

ompl::geometric::BundleSpacePathRestriction::~BundleSpacePathRestriction()
{
    if (bundleSpaceGraph_->isDynamic())
        return;

    if (bundleSpaceGraph_->getFiberDimension() > 0)
    {
        bundleSpaceGraph_->getFiber()->freeState(xFiberStart_);
        bundleSpaceGraph_->getFiber()->freeState(xFiberGoal_);
        bundleSpaceGraph_->getFiber()->freeState(xFiberTmp_);
    }
    if (bundleSpaceGraph_->getBaseDimension() > 0)
    {
        bundleSpaceGraph_->getBase()->freeState(xBaseTmp_);
    }
    bundleSpaceGraph_->getBundle()->freeState(lastValid_.first);
    bundleSpaceGraph_->getBundle()->freeState(xBundleTmp_);
}

void ompl::geometric::BundleSpacePathRestriction::reset()
{
    basePath_.clear();
}

void ompl::geometric::BundleSpacePathRestriction::setBasePath(ompl::base::PathPtr path)
{
    PathGeometricPtr geometricBasePath = std::static_pointer_cast<PathGeometric>(path);
    setBasePath(geometricBasePath->getStates());
}

void ompl::geometric::BundleSpacePathRestriction::setBasePath(std::vector<base::State *> basePath)
{
    basePath_ = basePath;

    lengthBasePath_ = 0.0;
    intermediateLengthsBasePath_.clear();
    for (uint k = 1; k < basePath_.size(); k++)
    {
        double lk = bundleSpaceGraph_->getBase()->distance(basePath_.at(k - 1), basePath_.at(k));
        intermediateLengthsBasePath_.push_back(lk);
        lengthBasePath_ += lk;
    }
    OMPL_INFORM("Set new base path with %d states and length %f.", basePath_.size(), lengthBasePath_);
}

std::vector<ompl::base::State *> ompl::geometric::BundleSpacePathRestriction::interpolateSectionL1FL(
    const base::State *xFiberStart, const base::State *xFiberGoal, const std::vector<base::State *> basePath)
{
    std::vector<base::State *> bundlePath;

    if (bundleSpaceGraph_->getFiberDimension() > 0)
    {
        bundlePath.resize(basePath.size() + 1);
        bundleSpaceGraph_->getBundle()->allocStates(bundlePath);
        for (uint k = 0; k < bundlePath.size() - 1; k++)
        {
            bundleSpaceGraph_->liftState(basePath.at(k), xFiberStart, bundlePath.at(k));
        }
        bundleSpaceGraph_->liftState(basePath.back(), xFiberGoal, bundlePath.back());
    }
    else
    {
        bundlePath.resize(basePath.size());
        bundleSpaceGraph_->getBundle()->allocStates(bundlePath);
        for (uint k = 0; k < basePath.size(); k++)
        {
            bundleSpaceGraph_->getBundle()->copyState(bundlePath.at(k), basePath.at(k));
        }
    }
    return bundlePath;
}

std::vector<ompl::base::State *> ompl::geometric::BundleSpacePathRestriction::interpolateSectionL1FF(
    const base::State *xFiberStart, const base::State *xFiberGoal, const std::vector<base::State *> basePath)
{
    std::vector<base::State *> bundlePath;

    if (bundleSpaceGraph_->getFiberDimension() > 0)
    {
        bundlePath.resize(basePath.size() + 1);
        bundleSpaceGraph_->getBundle()->allocStates(bundlePath);

        bundleSpaceGraph_->liftState(basePath.front(), xFiberStart, bundlePath.front());
        for (uint k = 1; k < bundlePath.size(); k++)
        {
            bundleSpaceGraph_->liftState(basePath.at(k - 1), xFiberGoal, bundlePath.at(k));
        }
    }
    else
    {
        bundlePath.resize(basePath.size());
        bundleSpaceGraph_->getBundle()->allocStates(bundlePath);
        for (uint k = 0; k < basePath.size(); k++)
        {
            bundleSpaceGraph_->getBundle()->copyState(bundlePath.at(k), basePath.at(k));
        }
    }
    return bundlePath;
}

std::vector<ompl::base::State *> ompl::geometric::BundleSpacePathRestriction::interpolateSectionL2(
    const base::State *xFiberStart, const base::State *xFiberGoal, const std::vector<base::State *> basePath)
{
    std::vector<base::State *> bundlePath;
    bundlePath.resize(basePath.size());
    bundleSpaceGraph_->getBundle()->allocStates(bundlePath);

    double totalLengthBasePath = 0.0;
    for (uint k = 1; k < basePath.size(); k++)
    {
        totalLengthBasePath += bundleSpaceGraph_->getBase()->distance(basePath.at(k - 1), basePath.at(k));
    }

    if (bundleSpaceGraph_->getFiberDimension() > 0)
    {
        double lengthCurrent = 0;

        for (uint k = 0; k < basePath.size(); k++)
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
        for (uint k = 0; k < basePath.size(); k++)
        {
            bundleSpaceGraph_->getBundle()->copyState(bundlePath.at(k), basePath.at(k));
        }
    }
    return bundlePath;
}

ompl::geometric::BundleSpaceGraph::Configuration *
ompl::geometric::BundleSpacePathRestriction::addFeasibleSegment(Configuration *xLast, base::State *sNext)
{
    Configuration *x = new Configuration(bundleSpaceGraph_->getBundle(), sNext);
    bundleSpaceGraph_->addConfiguration(x);
    bundleSpaceGraph_->addBundleEdge(xLast, x);

    x->parent = xLast;

#ifdef SANITY_CHECK_PATH_RESTRICTION
    if (!bundleSpaceGraph_->getBundle()->checkMotion(xLast->state, x->state))
    {
        OMPL_ERROR("Not feasible from last");
        std::cout << std::string(80, '-') << std::endl;
        std::cout << "Last State" << std::endl;
        bundleSpaceGraph_->getBundle()->printState(xLast->state);
        std::cout << std::string(80, '-') << std::endl;
        std::cout << "Current State" << std::endl;
        bundleSpaceGraph_->getBundle()->printState(x->state);
        throw Exception("");
    }
#endif
    return x;
}

void ompl::geometric::BundleSpacePathRestriction::addFeasibleGoalSegment(Configuration *const xLast,
                                                                         Configuration *const xGoal)
{
    if (xGoal->index <= 0)
    {
        bundleSpaceGraph_->vGoal_ = bundleSpaceGraph_->addConfiguration(xGoal);
    }
    bundleSpaceGraph_->addBundleEdge(xLast, xGoal);

    xGoal->parent = xLast;

#ifdef SANITY_CHECK_PATH_RESTRICTION
    if (!bundleSpaceGraph_->getBundle()->checkMotion(xLast->state, xGoal->state))
    {
        std::cout << std::string(80, '-') << std::endl;
        std::cout << "Last State" << std::endl;
        bundleSpaceGraph_->getBundle()->printState(xLast->state);
        std::cout << std::string(80, '-') << std::endl;
        std::cout << "Current State" << std::endl;
        bundleSpaceGraph_->getBundle()->printState(xGoal->state);
        throw Exception("Infeasible goal segment.");
    }
#endif
}

bool ompl::geometric::BundleSpacePathRestriction::sideStepAlongFiber(const base::State *xBase, base::State *xBundle)
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

bool ompl::geometric::BundleSpacePathRestriction::hasFeasibleSection(Configuration *const xStart,
                                                                     Configuration *const xGoal)
{
    // check for quasisection computation module
    int type = bundleSpaceGraph_->getBundle()->getStateSpace()->getType();
    if (type == base::STATE_SPACE_DUBINS || type == base::STATE_SPACE_DUBINS_AIRPLANE)
    {
        // Quasisections
        bundleSpaceGraph_->projectFiber(xStart->state, xFiberStart_);
        bundleSpaceGraph_->projectFiber(xGoal->state, xFiberGoal_);
        std::vector<base::State *> section = interpolateSectionL2(xFiberStart_, xFiberGoal_, basePath_);

        Configuration *xLast = xStart;

        for (uint k = 1; k < section.size(); k++)
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
        bool foundFeasibleSection = checkSectionRecursiveRepair(xStart, xGoal, basePath_);
        if (!foundFeasibleSection)
        {
            // Try with inverse L1
            foundFeasibleSection = checkSectionRecursiveRepair(xStart, xGoal, basePath_, false);
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

bool ompl::geometric::BundleSpacePathRestriction::checkSection(Configuration *const xStart, Configuration *const xGoal)
{
    bundleSpaceGraph_->projectFiber(xStart->state, xFiberStart_);
    bundleSpaceGraph_->projectFiber(xGoal->state, xFiberGoal_);

    std::vector<base::State *> section = interpolateSectionL2(xFiberStart_, xFiberGoal_, basePath_);

    Configuration *xLast = xStart;

    bool found = false;
    for (uint k = 1; k < section.size(); k++)
    {
        if (bundleSpaceGraph_->getBundle()->checkMotion(section.at(k - 1), section.at(k), lastValid_))
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
                Configuration *xk = new Configuration(bundleSpaceGraph_->getBundle(), lastValid_.first);
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
    bundleSpaceGraph_->getBundle()->freeStates(section);
    return found;
}

const unsigned int PATH_SECTION_TREE_MAX_DEPTH = 3;
const unsigned int PATH_SECTION_TREE_MAX_BRANCHING = 10;

bool ompl::geometric::BundleSpacePathRestriction::checkSectionRecursiveRepair(Configuration *const xStart,
                                                                              Configuration *const xGoal,
                                                                              const std::vector<base::State *> basePath,
                                                                              bool interpolateL1, unsigned int depth,
                                                                              double startLength)
{
    bundleSpaceGraph_->projectFiber(xStart->state, xFiberStart_);
    bundleSpaceGraph_->projectFiber(xGoal->state, xFiberGoal_);

    std::vector<base::State *> section;
    if (interpolateL1)
    {
        section = interpolateSectionL1FF(xFiberStart_, xFiberGoal_, basePath);
    }
    else
    {
        section = interpolateSectionL1FL(xFiberStart_, xFiberGoal_, basePath);
    }

    Configuration *xLast = xStart;

    for (uint k = 1; k < section.size(); k++)
    {
        if (bundleSpaceGraph_->getBundle()->checkMotion(section.at(k - 1), section.at(k), lastValid_))
        {
            if (k < section.size() - 1)
            {
                xLast = addFeasibleSegment(xLast, section.at(k));
            }
            else
            {
                addFeasibleGoalSegment(xLast, xGoal);
                OMPL_DEBUG("Found feasible path section (%d edges added)", k);
                bundleSpaceGraph_->getBundle()->freeStates(section);

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
                xLastValid = new Configuration(bundleSpaceGraph_->getBundle(), lastValid_.first);
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
            for (uint j = 1; j < k; j++)
            {
                double dj = bundleSpaceGraph_->getBase()->distance(basePath.at(j - 1), basePath.at(j));
                locationOnBasePath += dj;
            }

            if (k < basePath.size())
            {
                locationOnBasePath +=
                    lastValid_.second * bundleSpaceGraph_->getBase()->distance(basePath.at(k - 1), basePath.at(k));
            }

            static_cast<BundleSpaceGraph *>(bundleSpaceGraph_->getParent())
                ->getGraphSampler()
                ->setPathBiasStartSegment(locationOnBasePath + startLength);

            if (depth + 1 >= PATH_SECTION_TREE_MAX_DEPTH)
            {
                bundleSpaceGraph_->getBundle()->freeStates(section);
                return false;
            }

            //############################################################################
            // Side step randomly and interpolate from there towards goal
            //############################################################################
            unsigned int lastCtr = bundleSpaceGraph_->interpolateAlongBasePath(basePath, locationOnBasePath, xBaseTmp_);

            std::vector<base::State *> basePathSegment = {basePath.begin() + lastCtr, basePath.end()};
            basePathSegment.insert(basePathSegment.begin(), xBaseTmp_);

            for (uint j = 0; j < PATH_SECTION_TREE_MAX_BRANCHING; j++)
            {
                //#############################################################
                // find feasible sample in current fiber
                //#############################################################
                if (!sideStepAlongFiber(xBaseTmp_, xBundleTmp_))
                    continue;

                //#############################################################
                // check that we can connect new sample with last states
                //#############################################################
                if (bundleSpaceGraph_->getBundle()->checkMotion(xLastValid->state, xBundleTmp_))
                {
                    Configuration *xSideStep = new Configuration(bundleSpaceGraph_->getBundle(), xBundleTmp_);
                    bundleSpaceGraph_->addConfiguration(xSideStep);
                    bundleSpaceGraph_->addBundleEdge(xLastValid, xSideStep);

                    //#########################################################
                    // side step was successful.
                    // Now interpolate from there to goal
                    //#########################################################

                    bool feasibleSection = checkSectionRecursiveRepair(xSideStep, xGoal, basePathSegment,
                                                                       !interpolateL1, depth + 1, locationOnBasePath);

                    if (feasibleSection)
                    {
                        bundleSpaceGraph_->getBundle()->freeStates(section);
                        return true;
                    }
                }
            }

            break;
        }
    }
    bundleSpaceGraph_->getBundle()->freeStates(section);
    return false;
}

void ompl::geometric::BundleSpacePathRestriction::sanityCheckSection()
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
        for (uint k = 1; k < gStates.size(); k++)
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
